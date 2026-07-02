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
 *    {"cmd":"replay","timings":[...]}   ← timings dari DB atau hasil capture
 *    {"cmd":"status"}
 *
 *  Response ESP32-C6 → web app (via BLE TX):
 *    {"event":"captured","name":"TV_Power","timings":[...]}
 *    {"event":"done","msg":"replay selesai"}
 *    {"event":"status","capturing":false,"replaying":false}
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
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
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
#define REPLAY_DURATION_MS  2000        // kirim berulang 2 detik
#define CAPTURE_TIMEOUT_MS  5000        // timeout capture 5 detik

// ── RMT ─────────────────────────────────────────────────────
#define RMT_RESOLUTION_HZ   1000000UL  // 1µs per tick
#define IR_CARRIER_HZ       38000
#define MAX_RMT_SYMBOLS     64
#define MAX_TIMINGS         300         // max timing per sinyal

// ── BLE Nordic UART Service (NUS) ───────────────────────────
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHAR_RX_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // write
#define NUS_CHAR_TX_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // notify

// ── BLE Globals ─────────────────────────────────────────────
BLEServer*          pServer     = nullptr;
BLECharacteristic*  pTxChar     = nullptr;
bool                bleConnected = false;
static String       rxBuffer    = "";   // FIX BUG 2: akumulasi chunks BLE

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
    rxBuffer        = "";     // FIX BUG 2: bersihkan buffer saat disconnect
    g_capturing     = false;  // reset state
    g_replaying     = false;  // reset jika disconnect saat replay berlangsung
    g_replay_pending= false;
    rmt_rx_active   = false;
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

// Parse array "[500,600,...]" → timings buffer, returns count
int parse_timings(const String& arr, uint16_t* buf, int maxn) {
  int n = 0;
  int i = arr.indexOf('[');
  if (i < 0) return 0;
  i++;
  while (i < (int)arr.length() && n < maxn) {
    while (i < (int)arr.length() && (arr[i] == ' ' || arr[i] == ',')) i++;
    if (arr[i] == ']') break;
    int end = i;
    while (end < (int)arr.length() && arr[end] != ',' && arr[end] != ']') end++;
    buf[n++] = (uint16_t)arr.substring(i, end).toInt();
    i = end;
  }
  return n;
}

// ─────────────────────────────────────────────────────────────
//  Capture process
//  TSOP active-LOW: level=0 saat burst, level=1 saat idle
//  Simpan level di bit-15: 0=burst, 1=space; durasi di [14:0]
// ─────────────────────────────────────────────────────────────
int capture_process(const rmt_rx_done_event_data_t& ev, uint16_t* timings, int maxn) {
  int count = 0;
  for (size_t i = 0; i < ev.num_symbols && count < maxn - 2; i++) {
    const rmt_symbol_word_t& s = ev.received_symbols[i];
    if (s.duration0 > 0) {
      uint16_t t = (uint16_t)(s.duration0 & 0x7FFF);
      if (s.level0 == 1) t |= 0x8000;
      timings[count++] = t;
    }
    if (s.duration1 > 0) {
      uint16_t t = (uint16_t)(s.duration1 & 0x7FFF);
      if (s.level1 == 1) t |= 0x8000;
      timings[count++] = t;
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
//  Signal Replay — kirim berulang selama REPLAY_DURATION_MS
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

  unsigned long replayEnd = millis() + REPLAY_DURATION_MS;
  int sent = 0;
  do {
    esp_err_t err = rmt_transmit(tx_chan, copy_enc,
                                 syms, n * sizeof(rmt_symbol_word_t), &tcfg);
    if (err != ESP_OK) break;
    rmt_tx_wait_all_done(tx_chan, 500);
    sent++;
    delay(40);
  } while (millis() < replayEnd);

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
        ble_send("{\"event\":\"error\",\"msg\":\"busy\"}");
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
      ble_send("{\"event\":\"capturing\",\"name\":\"" + g_pending_name + "\"}");
    }

    else if (cmd == "replay") {
      if (g_capturing || g_replaying || g_replay_pending) {
        ble_send("{\"event\":\"error\",\"msg\":\"busy\"}");
        return;
      }
      String arr = json_get(val, "timings");
      // FIX BUG B: simpan ke buffer global, eksekusi di loop()
      int cnt = parse_timings(arr, g_replay_buf, MAX_TIMINGS);
      if (cnt < 4) {
        ble_send("{\"event\":\"error\",\"msg\":\"timings invalid\"}");
        return;
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
      ble_send(s);
    }

    else {
      ble_send("{\"event\":\"error\",\"msg\":\"unknown cmd\"}");
    }
  }
};

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
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);

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

  // FIX BUG B: eksekusi replay di sini, bukan di BLE callback
  if (g_replay_pending && !g_capturing) {
    g_replay_pending = false;
    signal_replay(g_replay_buf, g_replay_count);
    return;
  }

  if (!g_capturing) return;

  // Timeout capture — FIX BUG C: reset rmt_rx_active saat timeout
  if (millis() - g_capture_start > CAPTURE_TIMEOUT_MS) {
    g_capturing    = false;
    rmt_rx_active  = false;  // FIX BUG C
    Serial.println("[CAP] Timeout");
    ble_send("{\"event\":\"error\",\"msg\":\"timeout, tidak ada sinyal IR\"}");
    return;
  }

  // FIX BUG 3: hanya mulai receive baru kalau channel tidak sedang aktif
  if (!rmt_rx_active) {
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
  g_cap_count = capture_process(ev, g_cap_timings, MAX_TIMINGS);
  if (g_cap_count < 6) {
    Serial.println("[CAP] Sinyal terlalu pendek");
    ble_send("{\"event\":\"error\",\"msg\":\"sinyal terlalu pendek\"}");
    return;
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

  // Kirim dalam chunks jika terlalu panjang (BLE MTU ~512 bytes)
  const int CHUNK = 500;
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
