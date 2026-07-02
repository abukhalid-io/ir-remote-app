# IR Remote — ESP32-C6 BLE

Universal IR remote control via Web Bluetooth + Seeed XIAO ESP32-C6.

## Features

- **Browse 1,174+ remotes** from the Flipper Zero IRDB (TVs, ACs, projectors, and 27 more categories)
- **Capture** any IR signal from a physical remote and replay it via BLE
- **Decode** captured signals (NEC, NECext, Samsung32, RC5, SIRC, Kaseikyo)
- **Export** signals as Flipper Zero `.ir` files
- **PWA** — installable on Android/desktop, works offline after first load

## Hardware

| Pin | Connect to |
|-----|-----------|
| D0 (GPIO0) | IR LED anode (+ 100Ω resistor to GND) |
| D2 (GPIO2) | TSOP1838 OUT |
| 3V3 | TSOP1838 Vcc |
| GND | TSOP1838 GND + IR LED cathode |

Board: **Seeed XIAO ESP32-C6**

## Setup

### Firmware
1. Open `ir_ble_xiao_c6/ir_ble_xiao_c6.ino` in Arduino IDE
2. Board: `Seeed XIAO ESP32C6` (requires esp32 package ≥ 3.0.0)
3. Upload — device advertises as **IR-Remote-C6**

### Web App
Open `public/index.html` in Chrome (Android or desktop).  
For local development: `npx serve public` then open `http://localhost:3000`.

For production deploy: `vercel --prod` (requires `vercel.json` in repo root).

## BLE Protocol

Commands (web → device):

```json
{"cmd":"capture","name":"TV_Power"}
{"cmd":"replay","timings":[9000,4500,560,...]}
{"cmd":"status"}
```

Events (device → web):

```json
{"event":"capturing"}
{"event":"captured","name":"TV_Power","timings":[...]}
{"event":"done","msg":"replay selesai"}
{"event":"error","msg":"..."}
```

## Project Structure

```
ir-remote-app/
├── ir_ble_xiao_c6/          # Arduino firmware
│   └── ir_ble_xiao_c6.ino
└── public/                  # Web app (static files)
    ├── index.html           # Single-file PWA
    ├── manifest.json
    ├── sw.js                # Service worker
    ├── icon-192.png
    ├── icon-512.png
    ├── ir_db/               # JSON database (30 categories, 717 brands)
    └── flipper_irdb/        # Raw .ir files (1,227 files)
```

## IR Database

Source: [Flipper Zero IRDB](https://github.com/Lucaslhm/Flipper-IRDB) — community-contributed IR signal database.
