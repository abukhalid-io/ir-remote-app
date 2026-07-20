/*
 * ============================================================
 *  IR Remote BLE — Seeed XIAO ESP32-C6
 * ============================================================
 *  Wiring:
 *    D0 (GPIO0)  → IR LED Transmitter (+ resistor 100Ω ke GND)
 *    D2 (GPIO2)  → TSOP1838 OUT (Receiver)
 *    3V3         → TSOP1838 Vcc
 *    GND         → TSOP1838 GND & IR LED Kathode
 *    GPIO15      → USER LED onboard (LOW=ON)
 *
 *  Arsitektur:
 *    - ESP32-C6 expose BLE Nordic UART Service (NUS)
 *    - Web app di Vercel connect via Web Bluetooth API
 *    - Database IR dari Flipper Zero community repo (GitHub)
 *    - Command protocol via BLE UART (JSON lines)
 *
 *  Command dari web app → ESP32-C6 (via BLE RX):
 *    {"cmd":"capture","name":"TV_Power"}
 *    {"cmd":"replay","timings":[...]}      ← timings dari DB atau hasil capture
 *    {"cmd":"status"}
 *    {"cmd":"test_sensor"}               ← test TSOP1838: tunggu sinyal remote 5 detik
 *    {"cmd":"test_led"}                  ← test IR LED: nyalakan ~2 detik (cek kamera HP)
 *    {"cmd":"test_loop"}                 ← test loopback TX→RX
 *    {"cmd":"set_freq","hz":38000}       ← ganti frekuensi carrier (30000-60000 Hz)
 *
 *  Response ESP32-C6 → web app (via BLE TX):
 *    {"event":"captured","name":"TV_Power","timings":[...]}
 *    {"event":"done","msg":"replay selesai"}
 *    {"event":"status","capturing":false,"replaying":false}
 *    {"event":"freq_set","hz":38000,"msg":"Carrier 38 kHz OK"}
 *    {"event":"test_led","status":"start",...} / {"event":"test_led","ok":true,...}
 *    {"event":"test_sensor","ok":true,"count":42,"preview":[...]}
 *    {"event":"test_loop","ok":true,"symbols":5,"msg":"..."}
 *    {"event":"error","msg":"..."}
 *
 *  Library: built-in Arduino ESP32 3.x (BLE + RMT)
 *  Board  : Seeed XIAO ESP32C6 (esp32 board package >= 3.0.0)
 * ============================================================
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

// ── Pin Config ───────────────────────────────────────────────
// XIAO ESP32-C6: D0=GPIO0, D1=GPIO1, D2=GPIO2, D3=GPIO21
#define PIN_IR_TX          GPIO_NUM_0   // D0 → IR LED
#define PIN_IR_RX          GPIO_NUM_2   // D2 → TSOP1838
#define PIN_LED            15           // GPIO15 USER LED (LOW=ON)

// ── Timing ──────────────────────────────────────────────────
// FIX BUG E: dulu replay dikirim berulang selama 2 detik penuh (~15-20x per
// tap), bikin tombol toggle (power) & step (channel/volume) nyasar. Sekarang
// jumlah kirim per tap dibatasi tetap, meniru satu kali tekan remote asli.
#define REPLAY_REPEAT_COUNT   2          // jumlah kirim per 1 tap tombol
#define REPLAY_REPEAT_GAP_MS  40         // jeda antar pengiriman ulang
#define CAPTURE_TIMEOUT_MS    5000       // timeout capture 5 detik

// ── RMT ─────────────────────────────────────────────────────
#define RMT_RESOLUTION_HZ   1000000UL  // 1µs per tick
#define IR_CARRIER_HZ       38000
// FIX BUG F: 64 simbol (128 pulsa) kekecilan untuk sinyal AC panjang (Daikin/
// Mitsubishi/Fujitsu bisa 150-400+ pulsa) — dinaikkan supaya tidak terpotong.
#define MAX_RMT_SYMBOLS     200
#define MAX_TIMINGS         400         // max timing per sinyal (selaras dgn MAX_RMT_SYMBOLS)

// ── BLE Nordic UART Service (NUS) ───────────────────────────
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHAR_RX_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // write
#define NUS_CHAR_TX_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // notify

// ── BLE Globals ─────────────────────────────────────────────
BLEServer*          pServer     = nullptr;
BLECharacteristic*  pTxChar     = nullptr;
bool                bleConnected = false;
static String       rxBuffer    = "";   // FIX BUG 2: akumulasi chunks BLE

// FIX BUG H: notify() sebaiknya tidak dipanggil langsung dari dalam callback
// GATT write (onWrite) — bisa bentrok dgn task BLE host di beberapa versi stack
// Bluedroid/NimBLE-Arduino. Antrikan pesan di sini, kirim betulan dari loop().
static String        g_pending_notify      = "";
static volatile bool g_pending_notify_flag = false;

void queue_notify(const String& msg) {
  g_pending_notify      = msg;
  g_pending_notify_flag = true;
}

// ── RMT Handles ─────────────────────────────────────────────
static rmt_channel_handle_t  rx_chan  = NULL;
static rmt_channel_handle_t  tx_chan  = NULL;
static rmt_encoder_handle_t  copy_enc = NULL;
static QueueHandle_t          rx_queue = NULL;
static rmt_symbol_word_t      rx_buf[MAX_RMT_SYMBOLS];

// ── Capture / Replay State ───────────────────────────────────
volatile bool  g_capturing     = false;
volatile bool  g_replaying     = false;
static bool    rmt_rx_active   = false;  // FIX BUG 3: track state RMT receive
unsigned long  g_capture_start = 0;
String         g_pending_name  = "";

// ── LED State Machine ────────────────────────────────────────
unsigned long  g_led_timer     = 0;
int            g_led_phase     = 0;

// ── Timings buffer ───────────────────────────────────────────
uint16_t       g_cap_timings[MAX_TIMINGS];
int            g_cap_count     = 0;

// FIX BUG B: replay buffer terpisah, eksekusi di loop() bukan di callback
static uint16_t g_replay_buf[MAX_TIMINGS];
static int      g_replay_count  = 0;
static bool     g_replay_pending = false;

// ── Test / Diagnostic command state ──────────────────────────
volatile bool     g_test_led_pending    = false;
volatile bool     g_test_sensor_pending = false;
volatile bool     g_test_loop_pending   = false;
volatile uint32_t g_set_freq_hz         = 0;   // 0 = tidak ada perubahan
static   uint32_t g_carrier_hz          = IR_CARRIER_HZ;

// ─────────────────────────────────────────────────────────────
//  BLE Send helper
// ─────────────────────────────────────────────────────────────
void ble_send(const String& msg) {
  if (!bleConnected || pTxChar == nullptr) return;
  pTxChar->setValue((uint8_t*)msg.c_str(), msg.length());
  pTxChar->notify();
  delay(10);
}

// ─────────────────────────────────────────────────────────────
//  BLE Callbacks
// ─────────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    Serial.println("[BLE] Client connected");
    digitalWrite(PIN_LED, LOW);  // LED ON saat connected
    g_led_phase = 0;
    g_led_timer = millis();
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    rxBuffer              = "";     // FIX BUG 2: bersihkan buffer saat disconnect
    g_capturing           = false;  // reset state
    g_replaying           = false;  // reset jika disconnect saat replay berlangsung
    g_replay_pending      = false;
    rmt_rx_active         = false;
    g_test_led_pending    = false;
    g_test_sensor_pending = false;
    g_test_loop_pending   = false;
    g_set_freq_hz         = 0;
    Serial.println("[BLE] Client disconnected, restart advertising...");
    BLEDevice::startAdvertising();
  }
};

// ─────────────────────────────────────────────────────────────
//  Parse command dari BLE (JSON sederhana tanpa library)
//  Format: {"cmd":"capture","name":"TV_Power"}
//          {"cmd":"replay","timings":[500,600,...]}
//          {"cmd":"status"}
// ─────────────────────────────────────────────────────────────
String json_get(const String& json, const String& key) {
  String k = "\"" + key + "\":";
  int idx = json.indexOf(k);
  if (idx < 0) return "";
  idx += k.length();
  while (idx < (int)json.length() && json[idx] == ' ') idx++;
  if (json[idx] == '"') {
    // string value
    int end = json.indexOf('"', idx + 1);
    return json.substring(idx + 1, end);
  }
  if (json[idx] == '[') {
    // FIX BUG 1: array — scan sampai bracket penutup yang sesuai
    int depth = 0, end = idx;
    while (end < (int)json.length()) {
      if (json[end] == '[') depth++;
      else if (json[end] == ']') { depth--; if (depth == 0) { end++; break; } }
      end++;
    }
    return json.substring(idx, end);
  }
  // number / boolean — berhenti di koma atau kurung kurawal
  int end = idx;
  while (end < (int)json.length() && json[end] != ',' && json[end] != '}') end++;
  return json.substring(idx, end);
}

// Parse array "[500,600,...]" → timings buffer, returns count.
// FIX BUG G: dulu berhenti diam-diam kalau data > maxn (sinyal AC panjang dari
// DB Flipper terpotong tanpa pemberitahuan). Sekarang tetap scan sampai akhir
// array dan set *truncated=true kalau ada data yang tidak muat, biar caller
// bisa warn user alih-alih replay diam-diam salah/tidak lengkap.
int parse_timings(const String& arr, uint16_t* buf, int maxn, bool* truncated) {
  int n = 0;
  int i = arr.indexOf('[');
  if (i < 0) return 0;
  i++;
  if (truncated) *truncated = false;
  while (i < (int)arr.length()) {
    while (i < (int)arr.length() && (arr[i] == ' ' || arr[i] == ',')) i++;
    if (i >= (int)arr.length() || arr[i] == ']') break;
    int end = i;
    while (end < (int)arr.length() && arr[end] != ',' && arr[end] != ']') end++;
    if (n < maxn) {
      buf[n++] = (uint16_t)arr.substring(i, end).toInt();
    } else if (truncated) {
      *truncated = true;
    }
    i = end;
  }
  return n;
}

// ─────────────────────────────────────────────────────────────
//  Capture process
//  TSOP active-LOW: level=0 saat burst, level=1 saat idle
//  Simpan level di bit-15: 0=burst, 1=space; durasi di [14:0]
// ─────────────────────────────────────────────────────────────
int capture_process(const rmt_rx_done_event_data_t& ev, uint16_t* timings, int maxn, bool* truncated) {
  int count = 0;
  if (truncated) *truncated = false;
  for (size_t i = 0; i < ev.num_symbols; i++) {
    const rmt_symbol_word_t& s = ev.received_symbols[i];
    if (s.duration0 > 0) {
      if (count < maxn) {
        uint16_t t = (uint16_t)(s.duration0 & 0x7FFF);
        if (s.level0 == 1) t |= 0x8000;
        timings[count++] = t;
      } else if (truncated) *truncated = true;
    }
    if (s.duration1 > 0) {
      if (count < maxn) {
        uint16_t t = (uint16_t)(s.duration1 & 0x7FFF);
        if (s.level1 == 1) t |= 0x8000;
        timings[count++] = t;
      } else if (truncated) *truncated = true;
    }
  }
  // Strip leading spaces (TSOP idle sebelum burst pertama)
  int skip = 0;
  while (skip < count && (timings[skip] & 0x8000)) skip++;
  if (skip > 0 && skip < count) {
    count -= skip;
    memmove(timings, timings + skip, count * sizeof(uint16_t));
  }
  return count;
}

// ─────────────────────────────────────────────────────────────
//  Signal Replay — kirim REPLAY_REPEAT_COUNT kali (bukan time-based)
// ─────────────────────────────────────────────────────────────
void signal_replay(uint16_t* timings, int count) {
  if (count < 2) {
    ble_send("{\"event\":\"error\",\"msg\":\"timings terlalu pendek\"}");
    return;
  }

  static rmt_symbol_word_t syms[MAX_TIMINGS / 2 + 2];
  int n = 0;

  for (int i = 0; i < count - 1 && n < (int)(MAX_TIMINGS / 2); i += 2) {
    uint16_t t0 = timings[i];
    uint16_t t1 = timings[i + 1];
    uint16_t dur0 = t0 & 0x7FFF;
    uint16_t dur1 = t1 & 0x7FFF;
    if (dur0 == 0) continue;
    syms[n].level0    = (t0 & 0x8000) ? 0 : 1;  // burst→1, space→0
    syms[n].duration0 = dur0 > 32767 ? 32767 : dur0;
    syms[n].level1    = (t1 & 0x8000) ? 0 : 1;
    syms[n].duration1 = dur1 > 32767 ? 32767 : dur1;
    n++;
  }

  if (n == 0) {
    ble_send("{\"event\":\"error\",\"msg\":\"0 symbols\"}");
    return;
  }
  syms[n].val = 0;

  rmt_transmit_config_t tcfg = {};
  tcfg.loop_count = 0;

  g_replaying = true;
  digitalWrite(PIN_LED, LOW);  // LED solid ON selama replay

  // FIX BUG E: kirim jumlah tetap (bukan time-based) — 1 tap tombol web = 1
  // "tekan", bukan tombol ditahan. REPLAY_REPEAT_COUNT kecil tapi >1 untuk
  // keandalan penerimaan, tanpa membuat toggle/step code nyasar berkali-kali.
  int sent = 0;
  for (int i = 0; i < REPLAY_REPEAT_COUNT; i++) {
    esp_err_t err = rmt_transmit(tx_chan, copy_enc,
                                 syms, n * sizeof(rmt_symbol_word_t), &tcfg);
    if (err != ESP_OK) break;
    rmt_tx_wait_all_done(tx_chan, 500);
    sent++;
    if (i < REPLAY_REPEAT_COUNT - 1) delay(REPLAY_REPEAT_GAP_MS);
  }

  g_replaying = false;
  g_led_phase = 0;
  g_led_timer = millis();
  digitalWrite(PIN_LED, HIGH);

  Serial.printf("[TX] Done. sent x%d\n", sent);
  ble_send("{\"event\":\"done\",\"msg\":\"replay selesai\"}");
}

// ─────────────────────────────────────────────────────────────
//  BLE RX Callback — proses command dari web app
// ─────────────────────────────────────────────────────────────
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    // FIX BUG 2: akumulasi chunk sampai JSON lengkap
    // FIX BUG D: batas buffer 8KB, buang kalau overflow (data corrupt)
    if (rxBuffer.length() > 8192) {
      Serial.println("[BLE] rxBuffer overflow, reset");
      rxBuffer = "";
    }
    rxBuffer += pChar->getValue().c_str();

    // Cari JSON lengkap: cek keseimbangan kurung kurawal
    int depth = 0, jsonEnd = -1;
    for (int i = 0; i < (int)rxBuffer.length(); i++) {
      if (rxBuffer[i] == '{') depth++;
      else if (rxBuffer[i] == '}') {
        depth--;
        if (depth == 0) { jsonEnd = i; break; }
      }
    }
    if (jsonEnd < 0) {
      Serial.printf("[BLE RX] buffering... (%d bytes)\n", rxBuffer.length());
      return;  // JSON belum lengkap, tunggu chunk berikutnya
    }

    String val = rxBuffer.substring(0, jsonEnd + 1);
    rxBuffer = rxBuffer.substring(jsonEnd + 1);  // sisa untuk command berikutnya
    val.trim();
    Serial.printf("[BLE RX] %s\n", val.c_str());

    String cmd = json_get(val, "cmd");

    if (cmd == "capture") {
      if (g_capturing || g_replaying || g_replay_pending) {
        queue_notify("{\"event\":\"error\",\"msg\":\"busy\"}");
        return;
      }
      g_pending_name  = json_get(val, "name");
      if (g_pending_name.isEmpty()) g_pending_name = "untitled";
      g_cap_count     = 0;
      g_capture_start = millis();
      g_capturing     = true;
      g_led_phase     = 0;
      g_led_timer     = millis();
      Serial.printf("[CAP] Mulai capture: %s\n", g_pending_name.c_str());
      queue_notify("{\"event\":\"capturing\",\"name\":\"" + g_pending_name + "\"}");
    }

    else if (cmd == "replay") {
      if (g_capturing || g_replaying || g_replay_pending) {
        queue_notify("{\"event\":\"error\",\"msg\":\"busy\"}");
        return;
      }
      String arr = json_get(val, "timings");
      // FIX BUG B: simpan ke buffer global, eksekusi di loop()
      bool arr_truncated = false;
      int cnt = parse_timings(arr, g_replay_buf, MAX_TIMINGS, &arr_truncated);
      if (cnt < 4) {
        queue_notify("{\"event\":\"error\",\"msg\":\"timings invalid\"}");
        return;
      }
      if (arr_truncated) {
        // FIX BUG G: kasih tahu user kalau sinyal dipotong (device limit), bukan diam-diam
        queue_notify("{\"event\":\"warning\",\"msg\":\"sinyal terpotong ke " + String(MAX_TIMINGS) + " timing (device limit)\"}");
      }
      g_replay_count   = cnt;
      g_replay_pending = true;  // loop() akan eksekusi replay
    }

    else if (cmd == "status") {
      String s = "{\"event\":\"status\",\"capturing\":";
      s += g_capturing ? "true" : "false";
      s += ",\"replaying\":";
      s += g_replaying ? "true" : "false";
      s += ",\"ble\":true}";
      queue_notify(s);
    }

    else if (cmd == "test_led") {
      if (g_capturing || g_replaying || g_replay_pending ||
          g_test_led_pending || g_test_sensor_pending || g_test_loop_pending) {
        queue_notify("{\"event\":\"error\",\"msg\":\"busy\"}"); return;
      }
      g_test_led_pending = true;
    }

    else if (cmd == "test_sensor") {
      if (g_capturing || g_replaying || g_replay_pending ||
          g_test_led_pending || g_test_sensor_pending || g_test_loop_pending) {
        queue_notify("{\"event\":\"error\",\"msg\":\"busy\"}"); return;
      }
      g_test_sensor_pending = true;
    }

    else if (cmd == "test_loop") {
      if (g_capturing || g_replaying || g_replay_pending ||
          g_test_led_pending || g_test_sensor_pending || g_test_loop_pending) {
        queue_notify("{\"event\":\"error\",\"msg\":\"busy\"}"); return;
      }
      g_test_loop_pending = true;
    }

    else if (cmd == "set_freq") {
      String hz_str = json_get(val, "hz");
      if (hz_str.length() > 0) {
        uint32_t hz = (uint32_t)hz_str.toInt();
        if (hz >= 30000 && hz <= 60000) g_set_freq_hz = hz;
        else queue_notify("{\"event\":\"error\",\"msg\":\"frekuensi tidak valid (30-60 kHz)\"}");
      }
    }

    else {
      queue_notify("{\"event\":\"error\",\"msg\":\"unknown cmd\"}");
    }
  }
};

// ─────────────────────────────────────────────────────────────
//  cmd_set_freq — ganti frekuensi carrier IR TX
// ─────────────────────────────────────────────────────────────
void cmd_set_freq(uint32_t hz) {
  Serial.printf("[FREQ] Set carrier → %lu Hz\n", hz);
  rmt_disable(tx_chan);
  rmt_carrier_config_t carrier = {};
  carrier.frequency_hz = hz;
  carrier.duty_cycle   = 0.33f;
  esp_err_t err = rmt_apply_carrier(tx_chan, &carrier);
  rmt_enable(tx_chan);
  if (err == ESP_OK) {
    g_carrier_hz = hz;
    ble_send("{\"event\":\"freq_set\",\"hz\":" + String(hz) + ",\"msg\":\"Carrier " + String(hz / 1000) + " kHz OK\"}");
  } else {
    ble_send("{\"event\":\"error\",\"msg\":\"set_freq gagal\"}");
  }
}

// ─────────────────────────────────────────────────────────────
//  cmd_test_led — nyalakan IR LED ~2 detik untuk cek kamera HP
// ─────────────────────────────────────────────────────────────
void cmd_test_led() {
  ble_send("{\"event\":\"test_led\",\"status\":\"start\",\"msg\":\"LED menyala ~2 detik, arahkan kamera ke LED\"}");
  Serial.println("[TEST] LED: start");
  digitalWrite(PIN_LED, LOW);  // LED onboard ON selama test

  // 47 simbol × {level0=1,32767µs, level1=1,32767µs} ≈ 3 detik carrier terus-menerus
  static rmt_symbol_word_t syms[48];
  for (int i = 0; i < 47; i++) {
    syms[i].level0 = 1; syms[i].duration0 = 32767;
    syms[i].level1 = 1; syms[i].duration1 = 32767;
  }
  syms[47].val = 0;  // terminator

  rmt_transmit_config_t tcfg = {};
  esp_err_t err = rmt_transmit(tx_chan, copy_enc, syms, 47 * sizeof(rmt_symbol_word_t), &tcfg);
  if (err == ESP_OK) {
    rmt_tx_wait_all_done(tx_chan, 4000);
    ble_send("{\"event\":\"test_led\",\"ok\":true,\"msg\":\"Selesai. Kamera HP harus terlihat cahaya ungu/putih di ujung LED\"}");
  } else {
    ble_send("{\"event\":\"test_led\",\"ok\":false,\"msg\":\"RMT TX error\"}");
  }
  digitalWrite(PIN_LED, HIGH);
  g_led_timer = millis(); g_led_phase = 0;
  Serial.println("[TEST] LED: done");
}

// ─────────────────────────────────────────────────────────────
//  cmd_test_sensor — tunggu sinyal IR dari remote selama 5 detik
// ─────────────────────────────────────────────────────────────
void cmd_test_sensor() {
  ble_send("{\"event\":\"test_sensor\",\"status\":\"start\",\"msg\":\"Menunggu sinyal... tekan tombol remote ke sensor (5 detik)\"}");
  Serial.println("[TEST] Sensor: listening 5s...");

  rmt_rx_clean_start();  // FIX BUG K
  rmt_receive_config_t rcfg = {};
  rcfg.signal_range_min_ns = 1000;
  rcfg.signal_range_max_ns = 15000000;
  esp_err_t err = rmt_receive(rx_chan, rx_buf, sizeof(rx_buf), &rcfg);
  if (err != ESP_OK) {
    ble_send("{\"event\":\"test_sensor\",\"ok\":false,\"msg\":\"RMT RX error\"}");
    return;
  }
  rmt_rx_active = true;

  // FIX BUG I: dulu blocking penuh 5 detik bikin LED heartbeat "freeze" (device
  // kelihatan hang). Polling per 50ms + led_update() supaya tetap berkedip.
  rmt_rx_done_event_data_t ev;
  bool got = false;
  unsigned long waitStart = millis();
  while (millis() - waitStart < 5000) {
    if (xQueueReceive(rx_queue, &ev, pdMS_TO_TICKS(50)) == pdTRUE) { got = true; break; }
    led_update();
  }
  rmt_rx_active = false;

  if (!got) {
    Serial.println("[TEST] Sensor: timeout (no signal)");
    ble_send("{\"event\":\"test_sensor\",\"ok\":false,\"msg\":\"Tidak ada sinyal. Cek wiring: TSOP OUT\\u2192D2, Vcc\\u21923V3, GND\\u2192GND\"}");
    return;
  }

  static uint16_t tmp[MAX_TIMINGS];
  int count = capture_process(ev, tmp, MAX_TIMINGS, nullptr);
  Serial.printf("[TEST] Sensor: %d pulses (%d symbols)\n", count, (int)ev.num_symbols);

  if (count < 4) {
    ble_send("{\"event\":\"test_sensor\",\"ok\":false,\"msg\":\"Sinyal terlalu pendek (" + String(count) + " pulses)\"}");
    return;
  }

  // Kirim preview (12 pulse pertama) untuk ditampilkan sebagai waveform
  String msg = "{\"event\":\"test_sensor\",\"ok\":true,\"count\":" + String(count);
  msg += ",\"msg\":\"Sensor OK! " + String(count) + " pulses diterima\",\"preview\":[";
  int n = count < 12 ? count : 12;
  for (int i = 0; i < n; i++) {
    if (i) msg += ",";
    bool isSpace = (tmp[i] & 0x8000) != 0;
    uint16_t dur = tmp[i] & 0x7FFF;
    msg += "{\"d\":" + String(dur) + ",\"s\":" + (isSpace ? "1" : "0") + "}";
  }
  msg += "]}";
  ble_send(msg);
}

// ─────────────────────────────────────────────────────────────
//  cmd_test_loop — kirim burst IR lewat TX, cek apakah RX terima
// ─────────────────────────────────────────────────────────────
void cmd_test_loop() {
  ble_send("{\"event\":\"test_loop\",\"status\":\"start\",\"msg\":\"Mengirim burst IR...\"}");
  Serial.println("[TEST] Loopback: TX->RX");

  // Aktifkan RX terlebih dahulu
  rmt_rx_clean_start();  // FIX BUG K
  rmt_receive_config_t rcfg = {};
  rcfg.signal_range_min_ns = 1000;
  rcfg.signal_range_max_ns = 15000000;
  esp_err_t rx_err = rmt_receive(rx_chan, rx_buf, sizeof(rx_buf), &rcfg);
  if (rx_err != ESP_OK) {
    ble_send("{\"event\":\"test_loop\",\"ok\":false,\"msg\":\"RMT RX error\"}");
    return;
  }
  rmt_rx_active = true;

  // Kirim burst NEC-like: AGC 9ms + 4.5ms space + 5 × 560µs pair + trailing 560µs
  static rmt_symbol_word_t test_syms[8];
  test_syms[0].level0 = 1; test_syms[0].duration0 = 9000;  // AGC burst
  test_syms[0].level1 = 0; test_syms[0].duration1 = 4500;  // AGC space
  for (int i = 1; i <= 5; i++) {
    test_syms[i].level0 = 1; test_syms[i].duration0 = 560;
    test_syms[i].level1 = 0; test_syms[i].duration1 = 560;
  }
  test_syms[6].level0 = 1; test_syms[6].duration0 = 560;   // trailing mark
  test_syms[6].level1 = 0; test_syms[6].duration1 = 0;
  test_syms[7].val = 0;  // terminator

  rmt_transmit_config_t tcfg = {};
  esp_err_t tx_err = rmt_transmit(tx_chan, copy_enc, test_syms, 8 * sizeof(rmt_symbol_word_t), &tcfg);
  if (tx_err != ESP_OK) {
    rmt_rx_active = false;
    ble_send("{\"event\":\"test_loop\",\"ok\":false,\"msg\":\"RMT TX error\"}");
    return;
  }
  rmt_tx_wait_all_done(tx_chan, 300);

  // Tunggu hasil dari RX (300ms timeout, polling supaya LED tetap update)
  rmt_rx_done_event_data_t ev;
  bool got = false;
  unsigned long loopWaitStart = millis();
  while (millis() - loopWaitStart < 300) {
    if (xQueueReceive(rx_queue, &ev, pdMS_TO_TICKS(20)) == pdTRUE) { got = true; break; }
    led_update();
  }
  rmt_rx_active = false;

  if (!got || ev.num_symbols == 0) {
    Serial.println("[TEST] Loopback: FAIL (no RX)");
    ble_send("{\"event\":\"test_loop\",\"ok\":false,\"msg\":\"Loopback GAGAL. Cek: LED arah D0, sensor arah D2, LED fisik menghadap TSOP1838\"}");
    return;
  }

  uint32_t d0 = ev.received_symbols[0].duration0;
  Serial.printf("[TEST] Loopback: OK, %d symbols, first=%lu us\n", (int)ev.num_symbols, d0);
  ble_send("{\"event\":\"test_loop\",\"ok\":true,\"symbols\":" + String(ev.num_symbols) +
           ",\"msg\":\"Loopback OK! LED & sensor berfungsi (burst " + String(d0) + "us terdeteksi)\"}");
}

// ─────────────────────────────────────────────────────────────
//  FIX BUG K: reset bersih channel RX + antrian sebelum tiap sesi
//  receive baru (capture / test_sensor / test_loop).
//
//  Root cause: kalau sebuah rmt_receive() TIMEOUT tanpa pernah melihat
//  satu edge pun (TSOP diam total, gak ada sinyal), hardware RMT tetap
//  "armed" menunggu selamanya — gap-timeout internnya cuma jalan SETELAH
//  edge pertama muncul, jadi kalau nol edge, gak pernah dianggap selesai.
//  Kode lama cuma reset flag software rmt_rx_active=false lalu give up,
//  TANPA bilang ke hardware buat berhenti. Akibatnya:
//    1) rmt_receive() berikutnya gagal "channel not in enable state"
//       (ESP_ERR_INVALID_STATE) karena channel masih dianggap sibuk.
//    2) KALAU received event lama (basi, dari sesi sebelumnya atau dari
//       gangguan lain) masih nyangkut di rx_queue (gak pernah di-reset),
//       capture BARU langsung "berhasil" dalam hitungan milidetik dengan
//       data yang SALAH/BASI — bukan sinyal remote yang baru ditekan.
//       Ini yang bikin hasil capture "gak sinkron" pas direplay.
//  Fix: disable+enable ulang channel (paksa balik ke state bersih) dan
//  kosongkan antrian SEBELUM tiap rmt_receive() baru, bukan cuma setelah.
// ─────────────────────────────────────────────────────────────
void rmt_rx_clean_start() {
  rmt_disable(rx_chan);
  rmt_enable(rx_chan);
  xQueueReset(rx_queue);
  rmt_rx_active = false;
}

// ─────────────────────────────────────────────────────────────
//  LED State Machine
//  IDLE       : heartbeat — 50ms ON per 1 detik
//  CAPTURING  : blink cepat 150ms/150ms
//  REPLAYING  : solid ON (dikontrol signal_replay)
//  CONNECTED  : heartbeat lebih cepat (tidak idle)
// ─────────────────────────────────────────────────────────────
void led_update() {
  if (g_replaying) return;

  unsigned long now = millis();
  if (g_capturing) {
    if (now - g_led_timer >= 150) {
      g_led_timer = now;
      g_led_phase ^= 1;
      digitalWrite(PIN_LED, g_led_phase ? LOW : HIGH);
    }
  } else {
    unsigned long period = now - g_led_timer;
    if (g_led_phase == 0 && period >= 950) {
      g_led_phase = 1; g_led_timer = now;
      digitalWrite(PIN_LED, LOW);
    } else if (g_led_phase == 1 && period >= 50) {
      g_led_phase = 0; g_led_timer = now;
      digitalWrite(PIN_LED, HIGH);
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  RMT RX Callback (ISR context)
// ─────────────────────────────────────────────────────────────
static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t ch,
                                      const rmt_rx_done_event_data_t* ev,
                                      void* ctx) {
  BaseType_t woken = pdFALSE;
  rmt_rx_done_event_data_t copy = *ev;
  xQueueSendFromISR((QueueHandle_t)ctx, &copy, &woken);
  return woken == pdTRUE;
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=================================");
  Serial.println("  IR Remote BLE — XIAO ESP32-C6 ");
  Serial.println("=================================");

  // USER LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);  // ON saat boot
  g_led_timer = millis();

  // LittleFS (opsional — untuk simpan sinyal lokal)
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS gagal (diabaikan)");
  }

  // ── RMT TX ─────────────────────────────────────────────────
  rmt_tx_channel_config_t tx_cfg = {};
  tx_cfg.gpio_num          = PIN_IR_TX;
  tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  tx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
  tx_cfg.mem_block_symbols = 48;
  tx_cfg.trans_queue_depth = 4;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_chan));

  rmt_carrier_config_t carrier = {};
  carrier.frequency_hz   = IR_CARRIER_HZ;
  carrier.duty_cycle     = 0.33f;
  ESP_ERROR_CHECK(rmt_apply_carrier(tx_chan, &carrier));

  rmt_copy_encoder_config_t enc_cfg = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &copy_enc));
  ESP_ERROR_CHECK(rmt_enable(tx_chan));
  Serial.println("[RMT] TX ready on GPIO0");

  // ── RMT RX ─────────────────────────────────────────────────
  rx_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));

  rmt_rx_channel_config_t rx_cfg = {};
  rx_cfg.gpio_num          = PIN_IR_RX;
  rx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  rx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
  rx_cfg.mem_block_symbols = 48;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_chan));

  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = rmt_rx_done_cb;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, rx_queue));
  ESP_ERROR_CHECK(rmt_enable(rx_chan));
  Serial.println("[RMT] RX ready on GPIO2");

  // ── BLE ────────────────────────────────────────────────────
  BLEDevice::init("IR-Remote-C6");
  BLEDevice::setMTU(247);  // FIX BUG J: minta MTU lebih besar dari default 23 byte
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

  pTxChar = pService->createCharacteristic(
    NUS_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxChar->addDescriptor(new BLE2902());

  BLECharacteristic* pRxChar = pService->createCharacteristic(
    NUS_CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxChar->setCallbacks(new RxCallbacks());

  pService->start();

  // Set TX power maksimum supaya sinyal BLE lebih kuat
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);   // interval minimum advertising ~7.5ms
  pAdv->setMaxPreferred(0x12);   // interval maksimum ~22.5ms
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising: IR-Remote-C6");
  Serial.println("=================================\n");
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────
void loop() {
  led_update();

  // FIX BUG H: kirim notifikasi yang diantrikan dari onWrite() (GATT write context)
  if (g_pending_notify_flag) {
    g_pending_notify_flag = false;
    ble_send(g_pending_notify);
  }

  // FIX BUG B: eksekusi replay di sini, bukan di BLE callback
  if (g_replay_pending && !g_capturing) {
    g_replay_pending = false;
    signal_replay(g_replay_buf, g_replay_count);
    return;
  }

  // ── Eksekusi test / diagnostic commands ──────────────────────
  if (!g_capturing && !g_replaying && !g_replay_pending) {
    if (g_set_freq_hz != 0) {
      uint32_t hz = g_set_freq_hz; g_set_freq_hz = 0;
      cmd_set_freq(hz); return;
    }
    if (g_test_led_pending) {
      g_test_led_pending = false; cmd_test_led(); return;
    }
    if (g_test_sensor_pending) {
      g_test_sensor_pending = false; cmd_test_sensor(); return;
    }
    if (g_test_loop_pending) {
      g_test_loop_pending = false; cmd_test_loop(); return;
    }
  }

  if (!g_capturing) return;

  // Timeout capture — FIX BUG K: reset bersih channel+antrian saat timeout,
  // bukan cuma flag software (lihat penjelasan di rmt_rx_clean_start())
  if (millis() - g_capture_start > CAPTURE_TIMEOUT_MS) {
    g_capturing = false;
    rmt_rx_clean_start();
    Serial.println("[CAP] Timeout");
    ble_send("{\"event\":\"error\",\"msg\":\"timeout, tidak ada sinyal IR\"}");
    return;
  }

  // FIX BUG 3 + FIX BUG K: mulai receive baru dengan channel/antrian bersih
  if (!rmt_rx_active) {
    rmt_rx_clean_start();
    rmt_receive_config_t rcfg = {};
    rcfg.signal_range_min_ns = 1000;
    rcfg.signal_range_max_ns = 15000000;
    esp_err_t err = rmt_receive(rx_chan, rx_buf, sizeof(rx_buf), &rcfg);
    if (err != ESP_OK) {
      Serial.printf("[RMT] rmt_receive error: %d\n", err);
      return;
    }
    rmt_rx_active = true;
  }

  // Tunggu hasil (non-blocking, 50ms timeout)
  rmt_rx_done_event_data_t ev;
  if (xQueueReceive(rx_queue, &ev, pdMS_TO_TICKS(50)) != pdTRUE) return;

  rmt_rx_active = false;
  g_capturing = false;

  // Proses timings
  bool cap_truncated = false;
  g_cap_count = capture_process(ev, g_cap_timings, MAX_TIMINGS, &cap_truncated);
  if (g_cap_count < 6) {
    Serial.println("[CAP] Sinyal terlalu pendek");
    ble_send("{\"event\":\"error\",\"msg\":\"sinyal terlalu pendek\"}");
    return;
  }
  if (cap_truncated) {
    // FIX BUG G: kasih tahu user kalau sinyal capture dipotong (device limit)
    Serial.println("[CAP] Sinyal terpotong (melebihi MAX_TIMINGS)");
    ble_send("{\"event\":\"warning\",\"msg\":\"sinyal terpotong ke " + String(MAX_TIMINGS) + " timing, mungkin tidak lengkap\"}");
  }

  Serial.printf("[CAP] OK: %d timings\n", g_cap_count);

  // Kirim timings ke web app via BLE
  String resp = "{\"event\":\"captured\",\"name\":\"";
  resp += g_pending_name;
  resp += "\",\"timings\":[";
  for (int i = 0; i < g_cap_count; i++) {
    if (i) resp += ",";
    resp += String(g_cap_timings[i]);
  }
  resp += "]}";

  // FIX BUG J: CHUNK dulu 500 — asumsi MTU besar itu gak terjamin. ESP32
  // minta MTU 247 (lihat BLEDevice::setMTU di setup()), tapi kalau central
  // gak setuju/gagal negosiasi, bisa jatuh ke MTU default 23 byte. Notify()
  // di lib ini TIDAK auto-fragment — kalau msg lebih panjang dari (MTU-3),
  // paketnya silently terpotong dan JSON.parse() di browser gagal diam-diam,
  // client gak pernah dapat event apapun → bleBusy macet permanen ("selalu
  // busy"). Turunin ke nilai yang aman walau MTU jatuh ke nilai kecil.
  const int CHUNK = 150;
  if ((int)resp.length() <= CHUNK) {
    ble_send(resp);
  } else {
    // Kirim header dulu, lalu timings dalam beberapa chunk
    String header = "{\"event\":\"captured_start\",\"name\":\"" + g_pending_name + "\",\"count\":" + String(g_cap_count) + "}";
    ble_send(header);
    delay(20);
    String chunk = "{\"event\":\"timings\",\"data\":[";
    bool first = true;
    for (int i = 0; i < g_cap_count; i++) {
      String val = (first ? "" : ",") + String(g_cap_timings[i]);
      if ((int)(chunk.length() + val.length() + 2) > CHUNK) {
        chunk += "]}";
        ble_send(chunk);
        delay(20);
        chunk = "{\"event\":\"timings\",\"data\":[";
        first = true;
      }
      chunk += val;
      first = false;
    }
    chunk += "]}";
    ble_send(chunk);
    delay(20);
    ble_send("{\"event\":\"captured_end\"}");
  }
}
