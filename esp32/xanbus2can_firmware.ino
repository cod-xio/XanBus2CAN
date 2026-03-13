/**
 * XanBus to CAN Bridge - ESP32 Firmware
 * Schneider Electric / Xantrex XanBus Protocol Converter
 *
 * Supported Boards:
 *   - ESP32 DevKit v1      (classic, dual-core, UART2)
 *   - ESP32-C3 SuperMini   (RISC-V, single-core, UART1, RGB LED)
 *
 * Hardware:
 *   - RS485 Transceiver (MAX3485 / SN65HVD72)
 *   - MCP2515 CAN Controller (SPI)
 *   - WiFi for MQTT logging
 *
 * Pin Mapping:
 *  ┌──────────────┬────────────┬────────────┐
 *  │ Function     │ ESP32      │ ESP32-C3   │
 *  ├──────────────┼────────────┼────────────┤
 *  │ RS485 RX     │ GPIO 16    │ GPIO  5    │
 *  │ RS485 TX     │ GPIO 17    │ GPIO  4    │
 *  │ RS485 DE     │ GPIO  4    │ GPIO  3    │
 *  │ RS485 /RE    │ GPIO  5    │ GPIO  2    │
 *  │ CAN CS       │ GPIO  5    │ GPIO 10    │
 *  │ CAN INT      │ GPIO 15    │ GPIO  6    │
 *  │ SPI SCK      │ GPIO 18    │ GPIO  8    │
 *  │ SPI MOSI     │ GPIO 23    │ GPIO  9    │
 *  │ SPI MISO     │ GPIO 19    │ GPIO  7    │
 *  │ Status LED   │ GPIO  2    │ RGB WS2812 │
 *  └──────────────┴────────────┴────────────┘
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <mcp2515.h>
#include <ArduinoJson.h>

// ─── BOARD DETECTION ──────────────────────────────────────────────────────────
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV) || \
    defined(ARDUINO_LOLIN_C3_MINI)
  #ifndef BOARD_ESP32C3
    #define BOARD_ESP32C3
  #endif
#endif

// ─── PIN DEFINITIONS ──────────────────────────────────────────────────────────
#ifdef BOARD_ESP32C3
  // ESP32-C3: only UART0 (USB) + UART1 available
  // Use UART1 for RS485, reassign SPI pins to avoid flash conflict
  #define RS485_RX_PIN    5
  #define RS485_TX_PIN    4
  #define RS485_DE_PIN    3
  #define RS485_RE_PIN    2

  #define CAN_CS_PIN      10
  #define CAN_INT_PIN     6
  #define SPI_SCK_PIN     0   // Reassigned to avoid GPIO8 (flash/LED conflict)
  #define SPI_MOSI_PIN    1
  #define SPI_MISO_PIN    7

  // ESP32-C3 SuperMini: WS2812B RGB LED on GPIO8
  // Simple color via bit-banging (no Neopixel lib needed)
  #define HAS_RGB_LED  1
  #define RGB_PIN      8
  #define LED_STATUS_PIN (-1)

  // ESP32-C3 uses Serial (UART0=USB) + Serial1 (UART1=RS485)
  #define RS485_SERIAL Serial1

#else
  // ESP32 DevKit v1 classic
  #define RS485_RX_PIN    16
  #define RS485_TX_PIN    17
  #define RS485_DE_PIN    4
  #define RS485_RE_PIN    5

  #define CAN_CS_PIN      5
  #define CAN_INT_PIN     15
  #define SPI_SCK_PIN     18
  #define SPI_MOSI_PIN    23
  #define SPI_MISO_PIN    19

  #define LED_STATUS_PIN  2
  #define HAS_RGB_LED  0
  #define RS485_SERIAL Serial2
#endif

// ─── RGB LED HELPER (ESP32-C3 WS2812B) ───────────────────────────────────────
#if HAS_RGB_LED
// Minimal WS2812 single-pixel driver (no library needed)
static void rgb_send_bit(bool bit) {
  if (bit) {
    digitalWrite(RGB_PIN, HIGH); delayMicroseconds(1);
    digitalWrite(RGB_PIN, LOW);  delayMicroseconds(1);
  } else {
    digitalWrite(RGB_PIN, HIGH);
    digitalWrite(RGB_PIN, LOW);  delayMicroseconds(1);
  }
}
static void rgb_set(uint8_t r, uint8_t g, uint8_t b) {
  // WS2812 order: G R B
  uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
  noInterrupts();
  for (int i = 23; i >= 0; i--) rgb_send_bit((color >> i) & 1);
  interrupts();
  delayMicroseconds(50); // reset pulse
}
static void led_set_color(uint8_t r, uint8_t g, uint8_t b) { rgb_set(r, g, b); }
#else
static void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
  // Classic ESP32: only toggle blue LED for status
  (void)r; (void)g; (void)b;
  if (LED_STATUS_PIN >= 0)
    digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
}
#endif
// ═══════════════════════════════════════════════════════════════
//  ⚙️  KONFIGURATION – hier anpassen!
// ═══════════════════════════════════════════════════════════════

// ── WiFi ──────────────────────────────────────────────────────
#define CFG_WIFI_SSID       ""              // WLAN-Name (leer = direkt AP-Modus)
#define CFG_WIFI_PASSWORD   ""              // WLAN-Passwort

// ── MQTT (nur im Router-Modus aktiv) ──────────────────────────
#define CFG_MQTT_BROKER     "192.168.1.100" // IP des MQTT-Brokers
#define CFG_MQTT_PORT       1883
#define CFG_MQTT_USER       ""
#define CFG_MQTT_PASS       ""

// ── Gerät ─────────────────────────────────────────────────────
#define CFG_DEVICE_ID       "xanbus_bridge_01"
#define CFG_HOSTNAME        "xanbus"        // http://xanbus.local (mDNS)

// ── Statische IP (alle leer = DHCP) ───────────────────────────
#define CFG_STATIC_IP       ""              // z.B. "192.168.1.80"
#define CFG_STATIC_GW       ""              // z.B. "192.168.1.1"
#define CFG_STATIC_MASK     ""              // z.B. "255.255.255.0"
#define CFG_STATIC_DNS      ""              // z.B. "192.168.1.1"

// ── Access Point Fallback ─────────────────────────────────────
// Greift automatisch wenn:
//   • CFG_WIFI_SSID leer ist, ODER
//   • Router nicht erreichbar ist (nach 15s Timeout)
// Dann: WLAN "XanBus-XXXXXX" erscheint, Passwort: xanbus99
//       Dashboard: http://192.168.4.1
// ═══════════════════════════════════════════════════════════════

// ─── NVS CONFIG MANAGER ───────────────────────────────────────────────────────
#include <Preferences.h>
#include <ESPmDNS.h>
#include <WebServer.h>

Preferences prefs;
WebServer   cfgServer(80);

// Runtime config (loaded from NVS, falls back to CFG_* defaults)
struct NetConfig {
  char wifi_ssid[64];
  char wifi_pass[64];
  char mqtt_broker[64];
  int  mqtt_port;
  char mqtt_user[32];
  char mqtt_pass[32];
  char device_id[32];
  char hostname[32];
  char static_ip[16];
  char static_gw[16];
  char static_mask[16];
  char static_dns[16];
} cfg;

// Helper: read string from NVS silently (avoids NOT_FOUND log spam on first boot)
static String nvs_get(Preferences& p, const char* key, const char* def) {
  if (!p.isKey(key)) return String(def);
  return p.getString(key, def);
}
static int nvs_get_int(Preferences& p, const char* key, int def) {
  if (!p.isKey(key)) return def;
  return p.getInt(key, def);
}

void cfg_load() {
  prefs.begin("xanbus", true);  // true = read-only (faster, safer)
  strlcpy(cfg.wifi_ssid,   nvs_get(prefs,"wifi_ssid",   CFG_WIFI_SSID).c_str(),    sizeof(cfg.wifi_ssid));
  strlcpy(cfg.wifi_pass,   nvs_get(prefs,"wifi_pass",   CFG_WIFI_PASSWORD).c_str(),sizeof(cfg.wifi_pass));
  strlcpy(cfg.mqtt_broker, nvs_get(prefs,"mqtt_broker", CFG_MQTT_BROKER).c_str(),  sizeof(cfg.mqtt_broker));
  cfg.mqtt_port =      nvs_get_int(prefs,"mqtt_port",   CFG_MQTT_PORT);
  strlcpy(cfg.mqtt_user,   nvs_get(prefs,"mqtt_user",   CFG_MQTT_USER).c_str(),    sizeof(cfg.mqtt_user));
  strlcpy(cfg.mqtt_pass,   nvs_get(prefs,"mqtt_pass",   CFG_MQTT_PASS).c_str(),    sizeof(cfg.mqtt_pass));
  strlcpy(cfg.device_id,   nvs_get(prefs,"device_id",   CFG_DEVICE_ID).c_str(),    sizeof(cfg.device_id));
  strlcpy(cfg.hostname,    nvs_get(prefs,"hostname",    CFG_HOSTNAME).c_str(),      sizeof(cfg.hostname));
  strlcpy(cfg.static_ip,   nvs_get(prefs,"static_ip",   CFG_STATIC_IP).c_str(),    sizeof(cfg.static_ip));
  strlcpy(cfg.static_gw,   nvs_get(prefs,"static_gw",   CFG_STATIC_GW).c_str(),    sizeof(cfg.static_gw));
  strlcpy(cfg.static_mask, nvs_get(prefs,"static_mask", CFG_STATIC_MASK).c_str(),  sizeof(cfg.static_mask));
  strlcpy(cfg.static_dns,  nvs_get(prefs,"static_dns",  CFG_STATIC_DNS).c_str(),   sizeof(cfg.static_dns));
  prefs.end();

  bool first_boot = (strcmp(cfg.wifi_ssid, CFG_WIFI_SSID) == 0);
  if (first_boot) {
    Serial.println("Config: first boot – using firmware defaults.");
    Serial.println("  → WiFi-Daten in main.cpp eintragen oder nach dem Verbinden");
    Serial.println("    per Web-Interface setzen: http://<IP> oder http://xanbus.local");
  } else {
    Serial.printf("Config: ssid='%s'  host='%s'  ip=%s\n",
                  cfg.wifi_ssid, cfg.hostname,
                  strlen(cfg.static_ip) > 0 ? cfg.static_ip : "DHCP");
  }
}

void cfg_save() {
  prefs.begin("xanbus", false);
  prefs.putString("wifi_ssid",   cfg.wifi_ssid);
  prefs.putString("wifi_pass",   cfg.wifi_pass);
  prefs.putString("mqtt_broker", cfg.mqtt_broker);
  prefs.putInt   ("mqtt_port",   cfg.mqtt_port);
  prefs.putString("mqtt_user",   cfg.mqtt_user);
  prefs.putString("mqtt_pass",   cfg.mqtt_pass);
  prefs.putString("device_id",   cfg.device_id);
  prefs.putString("hostname",    cfg.hostname);
  prefs.putString("static_ip",   cfg.static_ip);
  prefs.putString("static_gw",   cfg.static_gw);
  prefs.putString("static_mask", cfg.static_mask);
  prefs.putString("static_dns",  cfg.static_dns);
  prefs.end();
  Serial.println("Config saved to NVS.");
}

void cfg_reset() {
  prefs.begin("xanbus", false);
  prefs.clear();
  prefs.end();
  Serial.println("Config reset to firmware defaults. Restarting...");
  delay(500);
  ESP.restart();
}

// ─── CAN ID MAPPING ───────────────────────────────────────────────────────────
// Maps XanBus PGNs to standard CAN IDs for downstream systems
#define CAN_ID_BATTERY_VOLTAGE      0x100
#define CAN_ID_BATTERY_CURRENT      0x101
#define CAN_ID_BATTERY_SOC          0x102
#define CAN_ID_BATTERY_TEMP         0x103
#define CAN_ID_SOLAR_POWER          0x110
#define CAN_ID_SOLAR_VOLTAGE        0x111
#define CAN_ID_SOLAR_CURRENT        0x112
#define CAN_ID_INVERTER_STATUS      0x120
#define CAN_ID_INVERTER_AC_V        0x121
#define CAN_ID_INVERTER_AC_HZ       0x122
#define CAN_ID_INVERTER_KW          0x123
#define CAN_ID_CHARGER_STATUS       0x130
#define CAN_ID_CHARGER_MODE         0x131
#define CAN_ID_SYSTEM_STATUS        0x140

// ─── DATA STRUCTURES ─────────────────────────────────────────────────────────
struct BatteryData {
  float voltage;        // Volts
  float current;        // Amps (positive = charge, negative = discharge)
  float soc;            // State of Charge %
  float temperature;    // Celsius
  float capacity_ah;    // Amp-hours
  uint8_t instance;
  uint32_t timestamp;
  bool valid;
};

struct SolarData {
  float pv_voltage;     // Volts
  float pv_current;     // Amps
  float pv_power;       // Watts
  float output_voltage; // Battery/output voltage
  float output_current; // Output current
  uint8_t controller_state; // 0=off, 1=MPPT, 2=bulk, 3=absorption, 4=float
  uint8_t instance;
  uint32_t timestamp;
  bool valid;
};

struct InverterData {
  float ac_voltage;     // Volts RMS
  float ac_current;     // Amps RMS
  float ac_frequency;   // Hz
  float ac_power;       // Watts
  float dc_input_v;     // DC input voltage
  uint8_t state;        // 0=off, 1=standby, 2=inverting, 3=fault
  uint8_t instance;
  uint32_t timestamp;
  bool valid;
};

struct ChargerData {
  float output_voltage; // Volts
  float output_current; // Amps
  float input_voltage;  // AC input Volts
  float input_current;  // AC input Amps
  uint8_t mode;         // 0=off, 1=bulk, 2=absorption, 3=float, 4=equalize
  uint8_t instance;
  uint32_t timestamp;
  bool valid;
};

// Global data store
BatteryData  g_battery  = {0};
SolarData    g_solar    = {0};
InverterData g_inverter = {0};
ChargerData  g_charger  = {0};

// ─── XANBUS FRAME STRUCTURE ───────────────────────────────────────────────────
// XanBus uses NMEA 2000 PGN framing over RS485
// Frame: [SOF][Priority+PGN 4B][Source][Dest][DLC][Data...][CRC16][EOF]
struct XanBusFrame {
  uint8_t  priority;      // 0-7
  uint32_t pgn;           // Parameter Group Number
  uint8_t  source;        // Source address (0-253)
  uint8_t  destination;   // Destination (255 = broadcast)
  uint8_t  dlc;           // Data length 0-8 (or fast-packet length)
  uint8_t  data[223];     // Up to 223 bytes (fast-packet)
  uint16_t crc;
  bool     fast_packet;
  uint8_t  seq_id;
  uint8_t  frame_count;
};

// ─── GLOBAL OBJECTS ───────────────────────────────────────────────────────────
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
MCP2515      mcp2515(CAN_CS_PIN);

// RS485 receive buffer
#define RS485_BUF_SIZE 512
uint8_t  rs485_buf[RS485_BUF_SIZE];
uint16_t rs485_buf_len = 0;

// Fast-packet reassembly buffers (per source address)
struct FastPacketBuffer {
  uint8_t  data[223];
  uint8_t  seq_id;
  uint8_t  frames_received;
  uint8_t  total_frames;
  uint16_t total_bytes;
  uint16_t bytes_received;
  bool     active;
  uint32_t start_time;
};
FastPacketBuffer fp_buffers[254]; // one per source address

// Statistics
struct Stats {
  uint32_t frames_rx;
  uint32_t frames_tx_can;
  uint32_t mqtt_published;
  uint32_t parse_errors;
  uint32_t crc_errors;
} stats = {0};

uint32_t last_mqtt_publish = 0;
uint32_t last_status_blink = 0;
bool     mqtt_connected    = false;

// ─── WEB INTERFACE (Dashboard + Settings, Passwortschutz) ────────────────────
// Neue Felder in NetConfig für MQTT-Interval, Topic-Prefix, Web-Passwort
// Diese werden ZUSÄTZLICH zu den bestehenden Feldern gespeichert.

// ── Neue Config-Defaults ──────────────────────────────────────────────────────
#define CFG_WEB_USER        "admin"
#define CFG_WEB_PASS        "xanbus"
#define CFG_MQTT_INTERVAL   1000    // ms
#define CFG_MQTT_TOPIC      "xanbus"

// ── Erweiterte Config-Felder (werden in cfg_load/cfg_save ergänzt) ────────────
// Hinweis: Diese werden nach dem bestehenden cfg struct separat geladen
static char   cfg_web_user[32]     = CFG_WEB_USER;
static char   cfg_web_pass[32]     = CFG_WEB_PASS;
static int    cfg_mqtt_interval    = CFG_MQTT_INTERVAL;
static char   cfg_mqtt_topic[64]   = CFG_MQTT_TOPIC;

void cfg_ext_load() {
  prefs.begin("xanbus2", true);
  strlcpy(cfg_web_user,    nvs_get(prefs,"web_user",  CFG_WEB_USER).c_str(),  sizeof(cfg_web_user));
  strlcpy(cfg_web_pass,    nvs_get(prefs,"web_pass",  CFG_WEB_PASS).c_str(),  sizeof(cfg_web_pass));
  cfg_mqtt_interval =      nvs_get_int(prefs,"mqtt_iv", CFG_MQTT_INTERVAL);
  strlcpy(cfg_mqtt_topic,  nvs_get(prefs,"mqtt_topic",CFG_MQTT_TOPIC).c_str(),sizeof(cfg_mqtt_topic));
  prefs.end();
}
void cfg_ext_save() {
  prefs.begin("xanbus2", false);
  prefs.putString("web_user",  cfg_web_user);
  prefs.putString("web_pass",  cfg_web_pass);
  prefs.putInt   ("mqtt_iv",   cfg_mqtt_interval);
  prefs.putString("mqtt_topic",cfg_mqtt_topic);
  prefs.end();
}

// ── HTTP Basic Auth helper ────────────────────────────────────────────────────
static bool web_auth() {
  if (!cfgServer.authenticate(cfg_web_user, cfg_web_pass)) {
    cfgServer.requestAuthentication(DIGEST_AUTH, "XanBus", "Login erforderlich");
    return false;
  }
  return true;
}

// ── Inverter state label ──────────────────────────────────────────────────────
static const char* inv_state_str(uint8_t s) {
  switch(s) { case 0: return "Aus"; case 1: return "Standby";
               case 2: return "Läuft"; case 3: return "Fehler"; }
  return "Unbekannt";
}
static const char* chg_mode_str(uint8_t m) {
  switch(m) { case 0: return "Aus"; case 1: return "Bulk";
               case 2: return "Absorption"; case 3: return "Float"; case 4: return "Equalize"; }
  return "Unbekannt";
}
static const char* sol_state_str(uint8_t s) {
  switch(s) { case 0: return "Aus"; case 1: return "MPPT";
               case 2: return "Bulk"; case 3: return "Absorption"; case 4: return "Float"; }
  return "Unbekannt";
}

// ─── HTML PAGE ────────────────────────────────────────────────────────────────
static const char WEB_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="de"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>XanBus Bridge</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Courier New',monospace;background:#070b0f;color:#b8ccd8;min-height:100vh}
header{background:linear-gradient(90deg,#003a5c,#070b0f);border-bottom:1px solid #0f2030;
  padding:14px 20px;display:flex;align-items:center;gap:12px}
.logo{font-size:20px}
h1{font-size:15px;letter-spacing:3px;color:#c8d8e8}
h1 span{color:#00c8ff}
.tabs{display:flex;background:#0a1018;border-bottom:1px solid #0f2030}
.tab{padding:11px 24px;font-size:11px;letter-spacing:2px;text-transform:uppercase;
  cursor:pointer;color:#3a5870;border-bottom:2px solid transparent;transition:all .2s}
.tab.active{color:#00c8ff;border-bottom-color:#00c8ff}
.tab:hover:not(.active){color:#7a9ab0}
.page{display:none;padding:20px;max-width:860px;margin:0 auto}
.page.active{display:block}

/* ── Status bar ── */
.statusbar{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:18px}
.sb-item{background:#0d1520;border:1px solid #0f2030;border-radius:6px;
  padding:6px 12px;font-size:11px;display:flex;align-items:center;gap:6px}
.sb-label{color:#2a4a60;text-transform:uppercase;letter-spacing:1px}
.sb-val{color:#00c8ff}
.dot{width:7px;height:7px;border-radius:50%;display:inline-block}
.dot.ok{background:#00e870;box-shadow:0 0 6px #00e870}
.dot.warn{background:#ffaa00;box-shadow:0 0 6px #ffaa00}
.dot.err{background:#ff3355;box-shadow:0 0 6px #ff3355}

/* ── Dashboard cards ── */
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;margin-bottom:16px}
.card{background:#0d1520;border:1px solid #0f2030;border-radius:10px;padding:16px;position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px}
.card.battery::before{background:linear-gradient(90deg,#00c8ff,#0060a0)}
.card.solar::before{background:linear-gradient(90deg,#ffd700,#ff8800)}
.card.inverter::before{background:linear-gradient(90deg,#00e870,#00a060)}
.card.charger::before{background:linear-gradient(90deg,#cc88ff,#6600cc)}
.card-icon{font-size:22px;margin-bottom:8px}
.card-title{font-size:10px;letter-spacing:2px;text-transform:uppercase;color:#2a4a60;margin-bottom:12px}
.metric{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:5px}
.metric-label{font-size:10px;color:#2a4a60;text-transform:uppercase;letter-spacing:1px}
.metric-value{font-size:16px;color:#c8d8e8;font-weight:700;letter-spacing:1px}
.metric-unit{font-size:10px;color:#3a5870;margin-left:3px}
.state-badge{display:inline-block;padding:2px 8px;border-radius:3px;font-size:10px;
  letter-spacing:1px;text-transform:uppercase;margin-top:6px}
.state-on{background:#00e87022;color:#00e870;border:1px solid #00e87044}
.state-off{background:#ff335522;color:#ff3355;border:1px solid #ff335544}
.state-warn{background:#ffaa0022;color:#ffaa00;border:1px solid #ffaa0044}

/* SOC bar */
.soc-bar-wrap{margin-top:8px}
.soc-bar-bg{background:#0a1220;border-radius:3px;height:6px;overflow:hidden}
.soc-bar-fill{height:100%;border-radius:3px;transition:width .5s ease}
.soc-label{font-size:10px;color:#3a5870;margin-bottom:3px}

/* Power flow */
.flow{background:#0d1520;border:1px solid #0f2030;border-radius:10px;padding:16px;margin-bottom:16px}
.flow-title{font-size:10px;letter-spacing:2px;text-transform:uppercase;color:#2a4a60;margin-bottom:14px}
.flow-row{display:flex;align-items:center;justify-content:space-between;gap:8px;flex-wrap:wrap}
.flow-node{text-align:center;min-width:80px}
.flow-node-icon{font-size:20px}
.flow-node-label{font-size:9px;color:#2a4a60;text-transform:uppercase;letter-spacing:1px;margin-top:2px}
.flow-node-val{font-size:13px;color:#c8d8e8;margin-top:2px}
.flow-arrow{flex:1;text-align:center;color:#0f2030;font-size:18px;min-width:30px}
.flow-arrow.active{color:#00c8ff}

/* ── Settings ── */
.section{background:#0d1520;border:1px solid #0f2030;border-radius:10px;padding:18px;margin-bottom:14px}
.section-title{font-size:10px;letter-spacing:2px;text-transform:uppercase;color:#2a4a60;
  margin-bottom:14px;padding-bottom:8px;border-bottom:1px solid #0f2030}
.field{margin-bottom:11px}
label{display:block;font-size:10px;color:#3a5870;text-transform:uppercase;letter-spacing:1px;margin-bottom:4px}
input,select{width:100%;background:#080c12;border:1px solid #0f2030;color:#b8ccd8;
  padding:7px 10px;border-radius:4px;font-family:monospace;font-size:13px;outline:none;
  transition:border-color .2s}
input:focus,select:focus{border-color:#00c8ff}
input[type=number]{-moz-appearance:textfield}
.hint{font-size:10px;color:#1a3040;margin-top:3px}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.row3{display:grid;grid-template-columns:2fr 1fr 1fr;gap:10px}
.row-ip{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.ip-mode{display:flex;gap:10px;margin-bottom:10px}
.radio-label{display:flex;align-items:center;gap:6px;cursor:pointer;
  padding:6px 12px;border:1px solid #0f2030;border-radius:4px;font-size:11px;
  color:#3a5870;transition:all .2s}
.radio-label:has(input:checked){border-color:#00c8ff;color:#00c8ff;background:#00c8ff11}
.radio-label input{width:auto;display:none}
.btn{padding:9px 20px;border:none;border-radius:4px;font-family:monospace;
  font-size:11px;font-weight:700;letter-spacing:1px;cursor:pointer;text-transform:uppercase;transition:all .2s}
.btn-primary{background:#00c8ff;color:#070b0f}
.btn-primary:hover{background:#00ddff;box-shadow:0 0 14px #00c8ff66}
.btn-danger{background:#1a2a3a;color:#ff3355;border:1px solid #ff335533}
.btn-danger:hover{background:#ff335511}
.btn-row{display:flex;gap:10px;margin-top:6px;flex-wrap:wrap}
.msg{padding:10px 14px;border-radius:6px;font-size:12px;margin-bottom:14px;display:none}
.msg.ok{background:#00e87011;border:1px solid #00e87033;color:#00e870}
.msg.err{background:#ff335511;border:1px solid #ff335533;color:#ff3355}
</style></head>
<body>
<header>
  <div class="logo">⚡</div>
  <h1>XanBus <span>Bridge</span></h1>
</header>

<div class="tabs">
  <div class="tab active" onclick="switchTab('dashboard',this)">📊 Dashboard</div>
  <div class="tab" onclick="switchTab('settings',this)">⚙️ Settings</div>
</div>

<!-- ═══════════════════════════════ DASHBOARD ════════════════════════════════ -->
<div id="dashboard" class="page active">
  <div class="statusbar">
    <div class="sb-item"><span class="sb-label">IP</span><span class="sb-val" id="s-ip">--</span></div>
    <div class="sb-item"><span class="sb-label">Host</span><span class="sb-val" id="s-host">--</span></div>
    <div class="sb-item"><span class="sb-label">WiFi</span><span id="s-rssi-dot" class="dot warn"></span><span class="sb-val" id="s-rssi">--</span></div>
    <div class="sb-item"><span class="sb-label">MQTT</span><span id="s-mqtt-dot" class="dot warn"></span><span class="sb-val" id="s-mqtt">--</span></div>
    <div class="sb-item"><span class="sb-label">Uptime</span><span class="sb-val" id="s-uptime">--</span></div>
    <div class="sb-item"><span class="sb-label">Frames</span><span class="sb-val" id="s-frames">0</span></div>
  </div>

  <!-- Power flow -->
  <div class="flow">
    <div class="flow-title">⚡ Energiefluss</div>
    <div class="flow-row">
      <div class="flow-node">
        <div class="flow-node-icon">☀️</div>
        <div class="flow-node-label">Solar</div>
        <div class="flow-node-val" id="f-solar">-- W</div>
      </div>
      <div class="flow-arrow" id="fa-solar">→</div>
      <div class="flow-node">
        <div class="flow-node-icon">🔋</div>
        <div class="flow-node-label">Batterie</div>
        <div class="flow-node-val" id="f-batt">-- V</div>
      </div>
      <div class="flow-arrow" id="fa-inv">→</div>
      <div class="flow-node">
        <div class="flow-node-icon">🔌</div>
        <div class="flow-node-label">Wechselrichter</div>
        <div class="flow-node-val" id="f-inv">-- W</div>
      </div>
    </div>
  </div>

  <!-- Metric cards -->
  <div class="grid">
    <!-- Battery -->
    <div class="card battery">
      <div class="card-icon">🔋</div>
      <div class="card-title">Batterie</div>
      <div class="metric">
        <span class="metric-label">Spannung</span>
        <span><span class="metric-value" id="b-v">--</span><span class="metric-unit">V</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">Strom</span>
        <span><span class="metric-value" id="b-i">--</span><span class="metric-unit">A</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">Temperatur</span>
        <span><span class="metric-value" id="b-t">--</span><span class="metric-unit">°C</span></span>
      </div>
      <div class="soc-bar-wrap">
        <div class="soc-label">SOC: <span id="b-soc-val">-- %</span></div>
        <div class="soc-bar-bg"><div class="soc-bar-fill" id="b-soc-bar" style="width:0%;background:#00c8ff"></div></div>
      </div>
    </div>

    <!-- Solar -->
    <div class="card solar">
      <div class="card-icon">☀️</div>
      <div class="card-title">Solar</div>
      <div class="metric">
        <span class="metric-label">PV Spannung</span>
        <span><span class="metric-value" id="sol-v">--</span><span class="metric-unit">V</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">PV Strom</span>
        <span><span class="metric-value" id="sol-i">--</span><span class="metric-unit">A</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">Leistung</span>
        <span><span class="metric-value" id="sol-w">--</span><span class="metric-unit">W</span></span>
      </div>
      <span class="state-badge state-off" id="sol-state">--</span>
    </div>

    <!-- Inverter -->
    <div class="card inverter">
      <div class="card-icon">🔌</div>
      <div class="card-title">Wechselrichter</div>
      <div class="metric">
        <span class="metric-label">AC Spannung</span>
        <span><span class="metric-value" id="inv-v">--</span><span class="metric-unit">V</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">AC Strom</span>
        <span><span class="metric-value" id="inv-i">--</span><span class="metric-unit">A</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">Frequenz</span>
        <span><span class="metric-value" id="inv-hz">--</span><span class="metric-unit">Hz</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">Leistung</span>
        <span><span class="metric-value" id="inv-w">--</span><span class="metric-unit">W</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">DC Eingang</span>
        <span><span class="metric-value" id="inv-dc">--</span><span class="metric-unit">V</span></span>
      </div>
      <span class="state-badge state-off" id="inv-state">--</span>
    </div>

    <!-- Charger -->
    <div class="card charger">
      <div class="card-icon">⚡</div>
      <div class="card-title">Ladegerät</div>
      <div class="metric">
        <span class="metric-label">Ausgang</span>
        <span><span class="metric-value" id="chg-ov">--</span><span class="metric-unit">V</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">Ladestrom</span>
        <span><span class="metric-value" id="chg-oi">--</span><span class="metric-unit">A</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">AC Eingang</span>
        <span><span class="metric-value" id="chg-iv">--</span><span class="metric-unit">V</span></span>
      </div>
      <div class="metric">
        <span class="metric-label">AC Strom</span>
        <span><span class="metric-value" id="chg-ii">--</span><span class="metric-unit">A</span></span>
      </div>
      <span class="state-badge state-off" id="chg-state">--</span>
    </div>
  </div>
</div>

<!-- ═══════════════════════════════ SETTINGS ════════════════════════════════ -->
<div id="settings" class="page">
  <div class="msg ok" id="msg-ok">✅ Gespeichert – Gerät startet neu…</div>
  <div class="msg err" id="msg-err">❌ Fehler beim Speichern.</div>

  <!-- WiFi -->
  <div class="section">
    <div class="section-title">📶 WiFi</div>
    <div class="field"><label>SSID</label>
      <input id="s-wifi-ssid" placeholder="Netzwerkname" autocomplete="off">
    </div>
    <div class="field"><label>Passwort</label>
      <input id="s-wifi-pass" type="password" placeholder="Leer lassen = unverändert">
    </div>
  </div>

  <!-- Netzwerk / IP -->
  <div class="section">
    <div class="section-title">🌐 Netzwerk</div>
    <div class="field"><label>Hostname (mDNS)</label>
      <input id="s-hostname" placeholder="xanbus">
      <div class="hint">Erreichbar als http://&lt;hostname&gt;.local</div>
    </div>
    <div class="field"><label>Geräte-ID (MQTT Client)</label>
      <input id="s-device-id" placeholder="xanbus_bridge_01">
    </div>
    <div class="field">
      <label>IP-Konfiguration</label>
      <div class="ip-mode">
        <label class="radio-label"><input type="radio" name="ip_mode" value="dhcp" id="r-dhcp"> DHCP</label>
        <label class="radio-label"><input type="radio" name="ip_mode" value="static" id="r-static"> Statisch</label>
      </div>
      <div id="static-fields" style="display:none">
        <div class="row-ip">
          <div class="field"><label>IP-Adresse</label><input id="s-ip-addr" placeholder="192.168.1.80"></div>
          <div class="field"><label>Gateway</label><input id="s-gw" placeholder="192.168.1.1"></div>
          <div class="field"><label>Subnetzmaske</label><input id="s-mask" placeholder="255.255.255.0"></div>
          <div class="field"><label>DNS</label><input id="s-dns" placeholder="192.168.1.1"></div>
        </div>
      </div>
    </div>
  </div>

  <!-- MQTT -->
  <div class="section">
    <div class="section-title">📨 MQTT</div>
    <div class="row3">
      <div class="field"><label>Broker-Adresse</label>
        <input id="s-mqtt-broker" placeholder="192.168.1.100">
      </div>
      <div class="field"><label>Port</label>
        <input id="s-mqtt-port" type="number" placeholder="1883" min="1" max="65535">
      </div>
      <div class="field"><label>Interval (ms)</label>
        <input id="s-mqtt-iv" type="number" placeholder="1000" min="100" max="60000">
        <div class="hint">100 – 60000 ms</div>
      </div>
    </div>
    <div class="field"><label>Topic-Prefix</label>
      <input id="s-mqtt-topic" placeholder="xanbus">
      <div class="hint">Veröffentlicht als &lt;prefix&gt;/battery, /solar, /inverter, /charger</div>
    </div>
    <div class="row2">
      <div class="field"><label>Benutzername</label><input id="s-mqtt-user" placeholder="optional"></div>
      <div class="field"><label>Passwort</label><input id="s-mqtt-mpass" type="password" placeholder="optional"></div>
    </div>
  </div>

  <!-- Web-Passwort -->
  <div class="section">
    <div class="section-title">🔐 Web-Interface Passwort</div>
    <div class="row2">
      <div class="field"><label>Benutzername</label>
        <input id="s-web-user" placeholder="admin">
      </div>
      <div class="field"><label>Passwort</label>
        <input id="s-web-pass" type="password" placeholder="Leer = unverändert">
        <div class="hint">Standard: admin / xanbus</div>
      </div>
    </div>
  </div>

  <div class="btn-row">
    <button class="btn btn-primary" onclick="saveSettings()">💾 Speichern &amp; Neustart</button>
    <button class="btn btn-danger" onclick="if(confirm('Alle Einstellungen zurücksetzen?'))doReset()">🗑 Werksreset</button>
  </div>
</div>

<script>
// ── Tab switch ────────────────────────────────────────────────────────────────
function switchTab(id, el) {
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  el.classList.add('active');
}

// ── Helpers ───────────────────────────────────────────────────────────────────
function val(id, data, key, dec) {
  const el = document.getElementById(id);
  if (!el) return;
  const v = data[key];
  el.textContent = (v === null || v === undefined || v === 0 && key!=='b_soc')
    ? '--' : (dec !== undefined ? v.toFixed(dec) : v);
}
function fmt(v, dec=1) { return (v===null||v===undefined) ? '--' : Number(v).toFixed(dec); }

function setState(id, state, labels) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = labels[state] || 'Unbekannt';
  el.className = 'state-badge ' + (state===2?'state-on':state===0?'state-off':'state-warn');
}

// ── SOC bar color ─────────────────────────────────────────────────────────────
function socColor(pct) {
  if (pct > 60) return '#00e870';
  if (pct > 30) return '#ffaa00';
  return '#ff3355';
}

// ── Uptime formatter ──────────────────────────────────────────────────────────
function fmtUptime(ms) {
  const s=Math.floor(ms/1000), m=Math.floor(s/60), h=Math.floor(m/60), d=Math.floor(h/24);
  if (d>0) return d+'d '+String(h%24).padStart(2,'0')+'h';
  return String(h%24).padStart(2,'0')+':'+String(m%60).padStart(2,'0')+':'+String(s%60).padStart(2,'0');
}

// ── Poll live data ────────────────────────────────────────────────────────────
function pollData() {
  fetch('/api/data').then(r=>r.json()).then(d=>{
    // Status bar
    document.getElementById('s-ip').textContent    = d.ip   || '--';
    document.getElementById('s-host').textContent  = (d.hostname||'--')+'.local';
    document.getElementById('s-rssi').textContent  = d.rssi ? d.rssi+'dBm' : '--';
    document.getElementById('s-rssi-dot').className = 'dot '+(d.rssi&&d.rssi>-75?'ok':'warn');
    document.getElementById('s-mqtt').textContent  = d.mqtt ? 'OK' : 'Getrennt';
    document.getElementById('s-mqtt-dot').className = 'dot '+(d.mqtt?'ok':'err');
    document.getElementById('s-uptime').textContent = fmtUptime(d.uptime_ms||0);
    document.getElementById('s-frames').textContent = d.frames||0;

    // Battery
    const b = d.battery || {};
    document.getElementById('b-v').textContent = fmt(b.voltage,2);
    document.getElementById('b-i').textContent = fmt(b.current,1);
    document.getElementById('b-t').textContent = fmt(b.temperature,1);
    const soc = b.soc ?? 0;
    document.getElementById('b-soc-val').textContent = fmt(soc,1)+' %';
    const bar = document.getElementById('b-soc-bar');
    bar.style.width = soc+'%'; bar.style.background = socColor(soc);

    // Solar
    const s = d.solar || {};
    document.getElementById('sol-v').textContent = fmt(s.pv_voltage,1);
    document.getElementById('sol-i').textContent = fmt(s.pv_current,1);
    document.getElementById('sol-w').textContent = fmt(s.pv_power,0);
    const ss = document.getElementById('sol-state');
    const slabels = ['Aus','MPPT','Bulk','Absorption','Float'];
    ss.textContent = slabels[s.state]||'--';
    ss.className = 'state-badge '+(s.state>0?'state-on':'state-off');

    // Inverter
    const iv = d.inverter || {};
    document.getElementById('inv-v').textContent  = fmt(iv.ac_voltage,1);
    document.getElementById('inv-i').textContent  = fmt(iv.ac_current,1);
    document.getElementById('inv-hz').textContent = fmt(iv.ac_frequency,2);
    document.getElementById('inv-w').textContent  = fmt(iv.ac_power,0);
    document.getElementById('inv-dc').textContent = fmt(iv.dc_input_v,2);
    const ist = document.getElementById('inv-state');
    const ilabels = ['Aus','Standby','Läuft','Fehler'];
    ist.textContent = ilabels[iv.state]||'--';
    ist.className = 'state-badge '+(iv.state===2?'state-on':iv.state===0?'state-off':'state-warn');

    // Charger
    const c = d.charger || {};
    document.getElementById('chg-ov').textContent = fmt(c.output_voltage,2);
    document.getElementById('chg-oi').textContent = fmt(c.output_current,1);
    document.getElementById('chg-iv').textContent = fmt(c.input_voltage,1);
    document.getElementById('chg-ii').textContent = fmt(c.input_current,1);
    const cst = document.getElementById('chg-state');
    const clabels = ['Aus','Bulk','Absorption','Float','Equalize'];
    cst.textContent = clabels[c.mode]||'--';
    cst.className = 'state-badge '+(c.mode>0?'state-on':'state-off');

    // Power flow arrows
    document.getElementById('f-solar').textContent = fmt(s.pv_power,0)+' W';
    document.getElementById('f-batt').textContent  = fmt(b.voltage,1)+' V / '+fmt(soc,0)+'%';
    document.getElementById('f-inv').textContent   = fmt(iv.ac_power,0)+' W';
    document.getElementById('fa-solar').className  = 'flow-arrow '+(s.pv_power>0?'active':'');
    document.getElementById('fa-inv').className    = 'flow-arrow '+(iv.ac_power>0?'active':'');

  }).catch(()=>{});
}

// ── Load settings ─────────────────────────────────────────────────────────────
function loadSettings() {
  fetch('/api/config').then(r=>r.json()).then(d=>{
    document.getElementById('s-wifi-ssid').value  = d.wifi_ssid||'';
    document.getElementById('s-hostname').value   = d.hostname||'';
    document.getElementById('s-device-id').value  = d.device_id||'';
    document.getElementById('s-mqtt-broker').value= d.mqtt_broker||'';
    document.getElementById('s-mqtt-port').value  = d.mqtt_port||1883;
    document.getElementById('s-mqtt-iv').value    = d.mqtt_interval||1000;
    document.getElementById('s-mqtt-topic').value = d.mqtt_topic||'xanbus';
    document.getElementById('s-mqtt-user').value  = d.mqtt_user||'';
    document.getElementById('s-web-user').value   = d.web_user||'admin';
    const hasStatic = d.static_ip && d.static_ip.length > 0;
    document.getElementById(hasStatic?'r-static':'r-dhcp').checked = true;
    if(hasStatic) {
      document.getElementById('static-fields').style.display='block';
      document.getElementById('s-ip-addr').value = d.static_ip||'';
      document.getElementById('s-gw').value      = d.static_gw||'';
      document.getElementById('s-mask').value    = d.static_mask||'';
      document.getElementById('s-dns').value     = d.static_dns||'';
    }
  });
}

// ── IP mode toggle ────────────────────────────────────────────────────────────
document.querySelectorAll('input[name=ip_mode]').forEach(r => {
  r.addEventListener('change', () => {
    document.getElementById('static-fields').style.display =
      document.getElementById('r-static').checked ? 'block' : 'none';
  });
});

// ── Save settings ─────────────────────────────────────────────────────────────
function saveSettings() {
  const isStatic = document.getElementById('r-static').checked;
  const body = {
    wifi_ssid:    document.getElementById('s-wifi-ssid').value,
    wifi_pass:    document.getElementById('s-wifi-pass').value,
    hostname:     document.getElementById('s-hostname').value,
    device_id:    document.getElementById('s-device-id').value,
    mqtt_broker:  document.getElementById('s-mqtt-broker').value,
    mqtt_port:    parseInt(document.getElementById('s-mqtt-port').value)||1883,
    mqtt_interval:parseInt(document.getElementById('s-mqtt-iv').value)||1000,
    mqtt_topic:   document.getElementById('s-mqtt-topic').value,
    mqtt_user:    document.getElementById('s-mqtt-user').value,
    mqtt_pass:    document.getElementById('s-mqtt-mpass').value,
    web_user:     document.getElementById('s-web-user').value,
    web_pass:     document.getElementById('s-web-pass').value,
    static_ip:    isStatic ? document.getElementById('s-ip-addr').value : '',
    static_gw:    isStatic ? document.getElementById('s-gw').value : '',
    static_mask:  isStatic ? document.getElementById('s-mask').value : '',
    static_dns:   isStatic ? document.getElementById('s-dns').value : '',
  };
  fetch('/save', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
    .then(r=>r.ok ? showMsg('ok') : showMsg('err'))
    .catch(()=>showMsg('err'));
}

function doReset() { fetch('/reset').finally(()=>{}); }

function showMsg(type) {
  document.getElementById('msg-ok').style.display  = type==='ok'  ? 'block' : 'none';
  document.getElementById('msg-err').style.display = type==='err' ? 'block' : 'none';
}

// ── Init ──────────────────────────────────────────────────────────────────────
pollData();
setInterval(pollData, 2000);
loadSettings();
</script>
</body></html>
)HTML";

// ─── API: Live Data ───────────────────────────────────────────────────────────
void web_handle_api_data() {
  if (!web_auth()) return;
  // Build JSON with all live values
  String j = "{";
  j += "\"ip\":\""     + WiFi.localIP().toString() + "\",";
  j += "\"hostname\":\"" + String(cfg.hostname) + "\",";
  j += "\"rssi\":"     + String(WiFi.RSSI()) + ",";
  j += "\"mqtt\":"     + String(mqtt_connected ? "true" : "false") + ",";
  j += "\"uptime_ms\":" + String(millis()) + ",";
  j += "\"frames\":"   + String(stats.frames_rx) + ",";
  // Battery
  j += "\"battery\":{";
  j += "\"voltage\":"     + (isnan(g_battery.voltage)     ? "null" : String(g_battery.voltage,     3)) + ",";
  j += "\"current\":"     + (isnan(g_battery.current)     ? "null" : String(g_battery.current,     2)) + ",";
  j += "\"soc\":"         + (isnan(g_battery.soc)         ? "null" : String(g_battery.soc,         2)) + ",";
  j += "\"temperature\":" + (isnan(g_battery.temperature) ? "null" : String(g_battery.temperature, 2));
  j += "},";
  // Solar
  j += "\"solar\":{";
  j += "\"pv_voltage\":"  + (isnan(g_solar.pv_voltage)  ? "null" : String(g_solar.pv_voltage,  2)) + ",";
  j += "\"pv_current\":"  + (isnan(g_solar.pv_current)  ? "null" : String(g_solar.pv_current,  2)) + ",";
  j += "\"pv_power\":"    + (isnan(g_solar.pv_power)    ? "null" : String(g_solar.pv_power,    1)) + ",";
  j += "\"state\":"       + String(g_solar.controller_state);
  j += "},";
  // Inverter
  j += "\"inverter\":{";
  j += "\"ac_voltage\":"   + (isnan(g_inverter.ac_voltage)   ? "null" : String(g_inverter.ac_voltage,   2)) + ",";
  j += "\"ac_current\":"   + (isnan(g_inverter.ac_current)   ? "null" : String(g_inverter.ac_current,   2)) + ",";
  j += "\"ac_frequency\":" + (isnan(g_inverter.ac_frequency) ? "null" : String(g_inverter.ac_frequency, 2)) + ",";
  j += "\"ac_power\":"     + (isnan(g_inverter.ac_power)     ? "null" : String(g_inverter.ac_power,     1)) + ",";
  j += "\"dc_input_v\":"   + (isnan(g_inverter.dc_input_v)   ? "null" : String(g_inverter.dc_input_v,   2)) + ",";
  j += "\"state\":"        + String(g_inverter.state);
  j += "},";
  // Charger
  j += "\"charger\":{";
  j += "\"output_voltage\":" + (isnan(g_charger.output_voltage) ? "null" : String(g_charger.output_voltage, 2)) + ",";
  j += "\"output_current\":" + (isnan(g_charger.output_current) ? "null" : String(g_charger.output_current, 2)) + ",";
  j += "\"input_voltage\":"  + (isnan(g_charger.input_voltage)  ? "null" : String(g_charger.input_voltage,  2)) + ",";
  j += "\"input_current\":"  + (isnan(g_charger.input_current)  ? "null" : String(g_charger.input_current,  2)) + ",";
  j += "\"mode\":"           + String(g_charger.mode);
  j += "}";
  j += "}";
  cfgServer.send(200, "application/json", j);
}

// ─── API: Config GET ──────────────────────────────────────────────────────────
void web_handle_api_config() {
  if (!web_auth()) return;
  String j = "{";
  j += "\"wifi_ssid\":\""   + String(cfg.wifi_ssid)     + "\",";
  j += "\"hostname\":\""    + String(cfg.hostname)       + "\",";
  j += "\"device_id\":\""   + String(cfg.device_id)     + "\",";
  j += "\"static_ip\":\""   + String(cfg.static_ip)     + "\",";
  j += "\"static_gw\":\""   + String(cfg.static_gw)     + "\",";
  j += "\"static_mask\":\"" + String(cfg.static_mask)   + "\",";
  j += "\"static_dns\":\""  + String(cfg.static_dns)    + "\",";
  j += "\"mqtt_broker\":\"" + String(cfg.mqtt_broker)   + "\",";
  j += "\"mqtt_port\":"     + String(cfg.mqtt_port)     + ",";
  j += "\"mqtt_user\":\""   + String(cfg.mqtt_user)     + "\",";
  j += "\"mqtt_interval\":" + String(cfg_mqtt_interval) + ",";
  j += "\"mqtt_topic\":\""  + String(cfg_mqtt_topic)    + "\",";
  j += "\"web_user\":\""    + String(cfg_web_user)      + "\"";
  j += "}";
  cfgServer.send(200, "application/json", j);
}

// ─── API: Save (JSON POST) ────────────────────────────────────────────────────
void web_handle_save() {
  if (!web_auth()) return;
  if (cfgServer.method() != HTTP_POST) { cfgServer.send(405,"text/plain","POST required"); return; }

  // Parse JSON body
  String body = cfgServer.arg("plain");
  JsonDocument doc;
  if (deserializeJson(doc, body)) {
    cfgServer.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  // WiFi
  if (doc["wifi_ssid"].is<const char*>()) strlcpy(cfg.wifi_ssid, doc["wifi_ssid"], sizeof(cfg.wifi_ssid));
  if (doc["wifi_pass"].is<const char*>() && strlen(doc["wifi_pass"]) > 0)
    strlcpy(cfg.wifi_pass, doc["wifi_pass"], sizeof(cfg.wifi_pass));
  // Network
  if (doc["hostname"].is<const char*>())   strlcpy(cfg.hostname,    doc["hostname"],   sizeof(cfg.hostname));
  if (doc["device_id"].is<const char*>())  strlcpy(cfg.device_id,   doc["device_id"],  sizeof(cfg.device_id));
  if (doc["static_ip"].is<const char*>())  strlcpy(cfg.static_ip,   doc["static_ip"],  sizeof(cfg.static_ip));
  if (doc["static_gw"].is<const char*>())  strlcpy(cfg.static_gw,   doc["static_gw"],  sizeof(cfg.static_gw));
  if (doc["static_mask"].is<const char*>())strlcpy(cfg.static_mask, doc["static_mask"],sizeof(cfg.static_mask));
  if (doc["static_dns"].is<const char*>()) strlcpy(cfg.static_dns,  doc["static_dns"], sizeof(cfg.static_dns));
  // MQTT
  if (doc["mqtt_broker"].is<const char*>())strlcpy(cfg.mqtt_broker, doc["mqtt_broker"],sizeof(cfg.mqtt_broker));
  if (doc["mqtt_port"].is<int>())    cfg.mqtt_port = doc["mqtt_port"].as<int>();
  if (doc["mqtt_user"].is<const char*>())  strlcpy(cfg.mqtt_user,   doc["mqtt_user"],  sizeof(cfg.mqtt_user));
  if (doc["mqtt_pass"].is<const char*>() && strlen(doc["mqtt_pass"]) > 0)
    strlcpy(cfg.mqtt_pass, doc["mqtt_pass"], sizeof(cfg.mqtt_pass));
  // Extended
  if (doc["mqtt_interval"].is<int>()) cfg_mqtt_interval = constrain(doc["mqtt_interval"].as<int>(), 100, 60000);
  if (doc["mqtt_topic"].is<const char*>()) strlcpy(cfg_mqtt_topic, doc["mqtt_topic"], sizeof(cfg_mqtt_topic));
  if (doc["web_user"].is<const char*>())   strlcpy(cfg_web_user,   doc["web_user"],   sizeof(cfg_web_user));
  if (doc["web_pass"].is<const char*>() && strlen(doc["web_pass"]) > 0)
    strlcpy(cfg_web_pass, doc["web_pass"], sizeof(cfg_web_pass));

  cfg_save();
  cfg_ext_save();

  cfgServer.send(200, "application/json", "{\"ok\":true}");
  delay(400);
  ESP.restart();
}

// ─── Reset ────────────────────────────────────────────────────────────────────
void web_handle_reset() {
  if (!web_auth()) return;
  cfgServer.send(200, "application/json", "{\"ok\":true}");
  delay(200);
  cfg_reset();
}

// ─── Root: serve page ─────────────────────────────────────────────────────────
void web_handle_root() {
  if (!web_auth()) return;
  cfgServer.send_P(200, "text/html", WEB_PAGE);
}

// ─── Register all routes ──────────────────────────────────────────────────────
void web_setup() {
  cfgServer.on("/",            HTTP_GET,  web_handle_root);
  cfgServer.on("/api/data",   HTTP_GET,  web_handle_api_data);
  cfgServer.on("/api/config", HTTP_GET,  web_handle_api_config);
  cfgServer.on("/save",       HTTP_POST, web_handle_save);
  cfgServer.on("/reset",      HTTP_GET,  web_handle_reset);
  cfgServer.begin();
  Serial.println("Web server started: http://" + WiFi.localIP().toString());
}

void web_loop() { cfgServer.handleClient(); }


// Runtime config accessors — always read from cfg struct (loaded by cfg_load())
#define WIFI_SSID     cfg.wifi_ssid
#define WIFI_PASSWORD cfg.wifi_pass
#define MQTT_BROKER   cfg.mqtt_broker
#define MQTT_PORT     cfg.mqtt_port
#define MQTT_USER     cfg.mqtt_user
#define MQTT_PASS     cfg.mqtt_pass
#define DEVICE_ID     cfg.device_id

// MQTT Topics
// MQTT topics: built dynamically from cfg_mqtt_topic prefix
static char _mqtt_topic_buf[80];
static const char* mqtt_topic(const char* suffix) {
  snprintf(_mqtt_topic_buf, sizeof(_mqtt_topic_buf), "%s/%s", cfg_mqtt_topic, suffix);
  return _mqtt_topic_buf;
}
#define MQTT_TOPIC_BATTERY   mqtt_topic("battery")
#define MQTT_TOPIC_SOLAR     mqtt_topic("solar")
#define MQTT_TOPIC_INVERTER  mqtt_topic("inverter")
#define MQTT_TOPIC_CHARGER   mqtt_topic("charger")
#define MQTT_TOPIC_STATUS    mqtt_topic("status")

// ─── XANBUS / NMEA2000 PGN DEFINITIONS ───────────────────────────────────────
// Schneider XanBus PGNs (NMEA 2000 compatible)
#define PGN_BATTERY_STATUS          0x1F214   // 127508 - Battery Status
#define PGN_DC_DETAILED_STATUS      0x1F213   // 127507 - DC Detailed Status
#define PGN_CHARGER_STATUS          0x1F21C   // 127516 - Charger Status
#define PGN_INVERTER_STATUS         0x1F21D   // 127517 - Inverter Status
#define PGN_SOLAR_CONTROLLER        0x1F218   // 127512 - Solar Controller
#define PGN_AC_INPUT                0x1F211   // 127505 - AC Input Status
#define PGN_AC_OUTPUT               0x1F212   // 127506 - AC Output Status
#define PGN_PRODUCT_INFO            0x1EF00   // 126996 - Product Information
#define PGN_ISO_ADDRESS_CLAIM       0x1EE00   // 126208 - ISO Address Claim
#define PGN_SYS_TIME                0x1F010   // 126992 - System Time

// Schneider-specific PGNs
#define PGN_XANBUS_BATTERY_EXT      0x1FF00   // Extended Battery Data
#define PGN_XANBUS_SYSTEM_MODE      0x1FF01   // System Mode
#define PGN_XANBUS_FAULT            0x1FF02   // Fault/Warning



// ─── UTILITY FUNCTIONS ────────────────────────────────────────────────────────
// Decode signed 16-bit value from 2 bytes (little-endian) with resolution
float decode_int16(uint8_t* data, int offset, float resolution) {
  int16_t raw = (int16_t)((data[offset+1] << 8) | data[offset]);
  if (raw == 0x7FFF || raw == (int16_t)0x8000) return NAN;
  return (float)raw * resolution;
}

// Decode unsigned 16-bit value
float decode_uint16(uint8_t* data, int offset, float resolution) {
  uint16_t raw = (uint16_t)((data[offset+1] << 8) | data[offset]);
  if (raw == 0xFFFF) return NAN;
  return (float)raw * resolution;
}

// Decode unsigned 32-bit value
float decode_uint32(uint8_t* data, int offset, float resolution) {
  uint32_t raw = ((uint32_t)data[offset+3] << 24) |
                 ((uint32_t)data[offset+2] << 16) |
                 ((uint32_t)data[offset+1] << 8)  |
                  (uint32_t)data[offset];
  if (raw == 0xFFFFFFFF) return NAN;
  return (float)raw * resolution;
}

uint8_t decode_uint8_bits(uint8_t byte, int start_bit, int num_bits) {
  uint8_t mask = (1 << num_bits) - 1;
  return (byte >> start_bit) & mask;
}

uint16_t crc16_xanbus(uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

// ─── PGN PARSERS ─────────────────────────────────────────────────────────────

// PGN 127508 - Battery Status
void parse_battery_status(uint8_t* data, uint8_t len, uint8_t source) {
  if (len < 8) return;
  g_battery.instance    = data[0];
  g_battery.voltage     = decode_uint16(data, 1, 0.01f);    // 0.01V resolution
  g_battery.current     = decode_int16(data,  3, 0.1f);     // 0.1A resolution
  g_battery.temperature = decode_uint16(data, 5, 0.01f) - 273.15f; // Kelvin -> Celsius
  // Byte 7: SID (sequence ID)
  g_battery.timestamp   = millis();
  g_battery.valid       = true;
  stats.frames_rx++;
}

// PGN 127507 - DC Detailed Status (includes SOC)
void parse_dc_detailed(uint8_t* data, uint8_t len, uint8_t source) {
  if (len < 7) return;
  // Byte 2-3: SOC (0.004% resolution, 0-25600 = 0-100%)
  float soc = decode_uint16(data, 2, 0.004f);
  if (!isnan(soc)) g_battery.soc = soc;
  g_battery.capacity_ah = decode_uint16(data, 4, 1.0f);
}

// PGN 127512 - Solar Controller
void parse_solar_controller(uint8_t* data, uint8_t len, uint8_t source) {
  if (len < 8) return;
  g_solar.instance          = data[0];
  g_solar.controller_state  = decode_uint8_bits(data[1], 0, 4);
  g_solar.output_voltage    = decode_uint16(data, 2, 0.01f);  // Battery V
  g_solar.output_current    = decode_int16(data,  4, 0.1f);   // Output A
  g_solar.pv_voltage        = decode_uint16(data, 6, 0.01f);  // PV voltage
  // PV current calculated from power or separate PGN
  if (!isnan(g_solar.pv_voltage) && g_solar.pv_voltage > 0 && !isnan(g_solar.output_current)) {
    // Estimate PV current from output power / PV voltage (simplified)
    g_solar.pv_power = g_solar.output_voltage * g_solar.output_current;
    g_solar.pv_current = (g_solar.pv_voltage > 5.0f) ? g_solar.pv_power / g_solar.pv_voltage : 0;
  }
  g_solar.timestamp = millis();
  g_solar.valid     = true;
}

// PGN 127517 - Inverter Status
void parse_inverter_status(uint8_t* data, uint8_t len, uint8_t source) {
  if (len < 6) return;
  g_inverter.instance    = data[0];
  g_inverter.state       = decode_uint8_bits(data[1], 0, 4);
  g_inverter.ac_voltage  = decode_uint16(data, 2, 0.01f);     // VAC RMS
  g_inverter.ac_current  = decode_int16(data,  4, 0.1f);      // A RMS
  if (!isnan(g_inverter.ac_voltage) && !isnan(g_inverter.ac_current))
    g_inverter.ac_power  = g_inverter.ac_voltage * g_inverter.ac_current;
  g_inverter.timestamp   = millis();
  g_inverter.valid       = true;
}

// PGN 127505 - AC Input
void parse_ac_input(uint8_t* data, uint8_t len, uint8_t source) {
  if (len < 8) return;
  g_inverter.dc_input_v  = decode_uint16(data, 2, 0.01f);
  g_inverter.ac_frequency= decode_uint16(data, 6, 0.01f);
}

// PGN 127516 - Charger Status
void parse_charger_status(uint8_t* data, uint8_t len, uint8_t source) {
  if (len < 8) return;
  g_charger.instance        = data[0];
  g_charger.mode            = decode_uint8_bits(data[1], 0, 4);
  g_charger.output_voltage  = decode_uint16(data, 2, 0.01f);
  g_charger.output_current  = decode_int16(data,  4, 0.1f);
  g_charger.input_voltage   = decode_uint16(data, 6, 0.01f);
  g_charger.timestamp       = millis();
  g_charger.valid           = true;
}

// ─── CAN FRAME BUILDER ───────────────────────────────────────────────────────
void send_can_battery() {
  struct can_frame frame;
  // Battery voltage + current in one frame
  frame.can_id  = CAN_ID_BATTERY_VOLTAGE;
  frame.can_dlc = 8;
  // Bytes 0-1: Voltage * 100 (e.g. 2540 = 25.40V)
  int16_t v_raw = (int16_t)(g_battery.voltage * 100.0f);
  int16_t i_raw = (int16_t)(g_battery.current * 10.0f);
  int16_t t_raw = (int16_t)(g_battery.temperature * 10.0f);
  uint16_t s_raw= (uint16_t)(g_battery.soc * 10.0f);
  frame.data[0] = v_raw & 0xFF;
  frame.data[1] = (v_raw >> 8) & 0xFF;
  frame.data[2] = i_raw & 0xFF;
  frame.data[3] = (i_raw >> 8) & 0xFF;
  frame.data[4] = t_raw & 0xFF;
  frame.data[5] = (t_raw >> 8) & 0xFF;
  frame.data[6] = s_raw & 0xFF;
  frame.data[7] = (s_raw >> 8) & 0xFF;
  mcp2515.sendMessage(&frame);
  stats.frames_tx_can++;
}

void send_can_solar() {
  struct can_frame frame;
  frame.can_id  = CAN_ID_SOLAR_POWER;
  frame.can_dlc = 8;
  int16_t pv_v = (int16_t)(g_solar.pv_voltage * 100.0f);
  int16_t pv_i = (int16_t)(g_solar.pv_current * 100.0f);
  int16_t pw   = (int16_t)(g_solar.pv_power);
  frame.data[0] = pv_v & 0xFF;
  frame.data[1] = (pv_v >> 8) & 0xFF;
  frame.data[2] = pv_i & 0xFF;
  frame.data[3] = (pv_i >> 8) & 0xFF;
  frame.data[4] = pw & 0xFF;
  frame.data[5] = (pw >> 8) & 0xFF;
  frame.data[6] = g_solar.controller_state;
  frame.data[7] = g_solar.instance;
  mcp2515.sendMessage(&frame);
  stats.frames_tx_can++;
}

void send_can_inverter() {
  struct can_frame frame;
  frame.can_id  = CAN_ID_INVERTER_STATUS;
  frame.can_dlc = 8;
  int16_t ac_v  = (int16_t)(g_inverter.ac_voltage * 10.0f);
  int16_t ac_hz = (int16_t)(g_inverter.ac_frequency * 100.0f);
  int16_t ac_kw = (int16_t)(g_inverter.ac_power);
  frame.data[0] = g_inverter.state;
  frame.data[1] = 0x00;
  frame.data[2] = ac_v & 0xFF;
  frame.data[3] = (ac_v >> 8) & 0xFF;
  frame.data[4] = ac_hz & 0xFF;
  frame.data[5] = (ac_hz >> 8) & 0xFF;
  frame.data[6] = ac_kw & 0xFF;
  frame.data[7] = (ac_kw >> 8) & 0xFF;
  mcp2515.sendMessage(&frame);
  stats.frames_tx_can++;
}

void send_can_charger() {
  struct can_frame frame;
  frame.can_id  = CAN_ID_CHARGER_STATUS;
  frame.can_dlc = 8;
  int16_t cv = (int16_t)(g_charger.output_voltage * 100.0f);
  int16_t ci = (int16_t)(g_charger.output_current * 10.0f);
  int16_t iv = (int16_t)(g_charger.input_voltage * 10.0f);
  frame.data[0] = g_charger.mode;
  frame.data[1] = g_charger.instance;
  frame.data[2] = cv & 0xFF;
  frame.data[3] = (cv >> 8) & 0xFF;
  frame.data[4] = ci & 0xFF;
  frame.data[5] = (ci >> 8) & 0xFF;
  frame.data[6] = iv & 0xFF;
  frame.data[7] = (iv >> 8) & 0xFF;
  mcp2515.sendMessage(&frame);
  stats.frames_tx_can++;
}

// ─── MQTT PUBLISHER ───────────────────────────────────────────────────────────
void mqtt_publish_battery() {
  JsonDocument doc;
  doc["ts"]          = millis();
  doc["voltage"]     = serialized(String(g_battery.voltage, 2));
  doc["current"]     = serialized(String(g_battery.current, 1));
  doc["soc"]         = serialized(String(g_battery.soc, 1));
  doc["temp"]        = serialized(String(g_battery.temperature, 1));
  doc["capacity_ah"] = serialized(String(g_battery.capacity_ah, 0));
  doc["instance"]    = g_battery.instance;
  char buf[256]; serializeJson(doc, buf);
  mqttClient.publish(MQTT_TOPIC_BATTERY, buf, true);
  stats.mqtt_published++;
}

void mqtt_publish_solar() {
  JsonDocument doc;
  doc["ts"]          = millis();
  doc["pv_voltage"]  = serialized(String(g_solar.pv_voltage, 2));
  doc["pv_current"]  = serialized(String(g_solar.pv_current, 2));
  doc["pv_power"]    = serialized(String(g_solar.pv_power, 0));
  doc["out_voltage"] = serialized(String(g_solar.output_voltage, 2));
  doc["out_current"] = serialized(String(g_solar.output_current, 1));
  const char* states[] = {"Off","MPPT","Bulk","Absorption","Float","Equalize"};
  doc["state"]       = (g_solar.controller_state < 6) ? states[g_solar.controller_state] : "Unknown";
  char buf[256]; serializeJson(doc, buf);
  mqttClient.publish(MQTT_TOPIC_SOLAR, buf, true);
  stats.mqtt_published++;
}

void mqtt_publish_inverter() {
  JsonDocument doc;
  doc["ts"]          = millis();
  doc["ac_voltage"]  = serialized(String(g_inverter.ac_voltage, 1));
  doc["ac_current"]  = serialized(String(g_inverter.ac_current, 1));
  doc["ac_frequency"]= serialized(String(g_inverter.ac_frequency, 2));
  doc["ac_power"]    = serialized(String(g_inverter.ac_power, 0));
  doc["dc_voltage"]  = serialized(String(g_inverter.dc_input_v, 2));
  const char* states[] = {"Off","Standby","Inverting","Fault"};
  doc["state"]       = (g_inverter.state < 4) ? states[g_inverter.state] : "Unknown";
  char buf[256]; serializeJson(doc, buf);
  mqttClient.publish(MQTT_TOPIC_INVERTER, buf, true);
  stats.mqtt_published++;
}

void mqtt_publish_charger() {
  JsonDocument doc;
  doc["ts"]          = millis();
  doc["out_voltage"] = serialized(String(g_charger.output_voltage, 2));
  doc["out_current"] = serialized(String(g_charger.output_current, 1));
  doc["in_voltage"]  = serialized(String(g_charger.input_voltage, 1));
  const char* modes[] = {"Off","Bulk","Absorption","Float","Equalize"};
  doc["mode"]        = (g_charger.mode < 5) ? modes[g_charger.mode] : "Unknown";
  char buf[256]; serializeJson(doc, buf);
  mqttClient.publish(MQTT_TOPIC_CHARGER, buf, true);
  stats.mqtt_published++;
}

void mqtt_publish_status() {
  JsonDocument doc;
  doc["uptime_s"]      = millis() / 1000;
  doc["frames_rx"]     = stats.frames_rx;
  doc["frames_can_tx"] = stats.frames_tx_can;
  doc["mqtt_pub"]      = stats.mqtt_published;
  doc["parse_err"]     = stats.parse_errors;
  doc["crc_err"]       = stats.crc_errors;
  doc["wifi_rssi"]     = WiFi.RSSI();
  char buf[256]; serializeJson(doc, buf);
  mqttClient.publish(MQTT_TOPIC_STATUS, buf, false);
}

// ─── XANBUS RS485 PARSER ──────────────────────────────────────────────────────
// XanBus uses ISO 11783 / NMEA 2000 framing
// Frame Header (4 bytes): Priority(3bit) | PGN(18bit) | Source(8bit)
// Followed by data bytes
void process_xanbus_frame(uint8_t* frame, uint16_t len) {
  if (len < 5) { stats.parse_errors++; return; }

  // Extract 29-bit CAN-style header from first 4 bytes (big-endian in XanBus RS485)
  uint32_t header = ((uint32_t)frame[0] << 24) | ((uint32_t)frame[1] << 16) |
                    ((uint32_t)frame[2] << 8)  |  (uint32_t)frame[3];

  uint8_t  priority    = (header >> 26) & 0x07;
  uint32_t pgn         = (header >> 8)  & 0x03FFFF;
  uint8_t  source      = header & 0xFF;
  uint8_t  destination = 0xFF; // broadcast by default
  uint8_t* data        = frame + 4;
  uint8_t  data_len    = len - 4;

  // PDU1 format: destination in byte 2 of PGN field
  if (((pgn >> 8) & 0xFF) < 240) {
    destination = (pgn >> 8) & 0xFF;
    pgn = pgn & 0x03FF00; // zero out destination field from PGN
  }

  // Dispatch based on PGN
  switch (pgn & 0x1FFFF) {
    case 0x1F214: parse_battery_status(data, data_len, source);    send_can_battery();  break;
    case 0x1F213: parse_dc_detailed(data, data_len, source);                            break;
    case 0x1F218: parse_solar_controller(data, data_len, source);  send_can_solar();    break;
    case 0x1F21D: parse_inverter_status(data, data_len, source);   send_can_inverter(); break;
    case 0x1F211: parse_ac_input(data, data_len, source);                               break;
    case 0x1F21C: parse_charger_status(data, data_len, source);    send_can_charger();  break;
    default:
      // Publish unknown PGN as raw hex for debugging
      if (mqtt_connected) {
        char topic[64]; snprintf(topic, sizeof(topic), "xanbus/pgn/0x%05X", pgn);
        char hex_buf[128];
        int pos = 0;
        for (int i = 0; i < min(data_len, (uint8_t)20); i++)
          pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", data[i]);
        mqttClient.publish(topic, hex_buf, false);
      }
      break;
  }
}

// Fast-packet reassembly
void process_fast_packet(uint8_t* frame, uint16_t len, uint32_t pgn, uint8_t source) {
  if (len < 2) return;
  uint8_t byte0    = frame[0];
  uint8_t seq_id   = (byte0 >> 5) & 0x07;
  uint8_t frame_no = byte0 & 0x1F;

  FastPacketBuffer& fp = fp_buffers[source];

  if (frame_no == 0) {
    // First frame: byte1 = total bytes
    fp.seq_id         = seq_id;
    fp.total_bytes    = frame[1];
    fp.bytes_received = len - 2;
    fp.frames_received= 1;
    fp.total_frames   = (fp.total_bytes + 5) / 6 + 1;
    fp.active         = true;
    fp.start_time     = millis();
    memcpy(fp.data, frame + 2, len - 2);
  } else if (fp.active && fp.seq_id == seq_id) {
    // Subsequent frames
    uint16_t offset = 6 + (frame_no - 1) * 7;
    if (offset + (len - 1) <= sizeof(fp.data)) {
      memcpy(fp.data + offset, frame + 1, len - 1);
      fp.bytes_received += len - 1;
      fp.frames_received++;
    }
    // Check if complete
    if (fp.bytes_received >= fp.total_bytes) {
      uint8_t complete_frame[227];
      // Re-add PGN header
      uint32_t hdr = ((uint32_t)3 << 26) | (pgn << 8) | source;
      complete_frame[0] = (hdr >> 24) & 0xFF;
      complete_frame[1] = (hdr >> 16) & 0xFF;
      complete_frame[2] = (hdr >> 8)  & 0xFF;
      complete_frame[3] =  hdr & 0xFF;
      memcpy(complete_frame + 4, fp.data, fp.total_bytes);
      process_xanbus_frame(complete_frame, fp.total_bytes + 4);
      fp.active = false;
    }
  } else if (millis() - fp.start_time > 500) {
    fp.active = false; // Timeout stale reassembly
  }
}

// ─── RS485 RECEIVE ───────────────────────────────────────────────────────────
#define XANBUS_SOF 0xAA
#define XANBUS_EOF 0x55

void read_rs485() {
  while (RS485_SERIAL.available()) {
    uint8_t b = RS485_SERIAL.read();
    if (rs485_buf_len == 0 && b != XANBUS_SOF) continue; // Wait for SOF
    rs485_buf[rs485_buf_len++] = b;
    if (rs485_buf_len >= RS485_BUF_SIZE) { rs485_buf_len = 0; continue; }

    // Check for EOF
    if (rs485_buf_len >= 6 && rs485_buf[rs485_buf_len - 1] == XANBUS_EOF) {
      // Validate CRC
      uint16_t calc_crc = crc16_xanbus(rs485_buf + 1, rs485_buf_len - 4);
      uint16_t recv_crc = ((uint16_t)rs485_buf[rs485_buf_len - 3] << 8) |
                                     rs485_buf[rs485_buf_len - 2];
      if (calc_crc != recv_crc) {
        stats.crc_errors++;
        rs485_buf_len = 0;
        continue;
      }
      // Process valid frame (strip SOF/CRC/EOF)
      process_xanbus_frame(rs485_buf + 1, rs485_buf_len - 4);
      rs485_buf_len = 0;
    }
  }
}

// ─── WIFI + MQTT ─────────────────────────────────────────────────────────────
// ─── WIFI: STA + AP-Fallback ──────────────────────────────────────────────────
// AP-Konfiguration (wenn kein Router erreichbar)
#define AP_SSID_PREFIX  "XanBus-"   // SSID = "XanBus-XXXXXX" (letzte 3 Byte MAC)
#define AP_PASSWORD     "xanbus99"  // Mindestens 8 Zeichen
#define AP_IP           192,168,4,1
#define AP_CHANNEL      6

// Modi
enum XbWifiMode { XB_WIFI_NONE, XB_WIFI_STA, XB_WIFI_AP };
XbWifiMode current_wifi_mode = XB_WIFI_NONE;

// AP starten
void start_ap() {
  WiFi.disconnect(true);
  delay(100);

  // SSID aus letzten 3 Byte der MAC generieren: "XanBus-A1B2C3"
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char ap_ssid[24];
  snprintf(ap_ssid, sizeof(ap_ssid), "%s%02X%02X%02X",
           AP_SSID_PREFIX, mac[3], mac[4], mac[5]);

  IPAddress ap_ip(AP_IP);
  IPAddress ap_gw(AP_IP);
  IPAddress ap_mask(255, 255, 255, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ap_ip, ap_gw, ap_mask);
  WiFi.softAP(ap_ssid, AP_PASSWORD, AP_CHANNEL);

  Serial.println("════════════════════════════════════════");
  Serial.println("  ⚠  Kein Router erreichbar!");
  Serial.println("  📶 Access Point gestartet:");
  Serial.printf ("     SSID:     %s\n", ap_ssid);
  Serial.printf ("     Passwort: %s\n", AP_PASSWORD);
  Serial.printf ("     IP:       %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("  🌐 Web-Interface: http://192.168.4.1");
  Serial.println("════════════════════════════════════════");

  // mDNS im AP-Modus (funktioniert nicht auf allen Clients, aber schadet nicht)
  const char* host = strlen(cfg.hostname) > 0 ? cfg.hostname : cfg.device_id;
  if (MDNS.begin(host)) {
    MDNS.addService("http", "tcp", 80);
  }

  // Captive-Portal-DNS: alle DNS-Anfragen auf 192.168.4.1 umleiten
  // (damit Browser automatisch das Config-Portal öffnen)
  // Einfache Implementierung: jeder UDP-DNS-Request → unsere IP
  web_setup();    // Config-Portal auf Port 80 starten
  current_wifi_mode = XB_WIFI_AP;

#if HAS_RGB_LED
  led_set_color(80, 40, 0); // Orange = AP-Modus
#endif
}

// Verbindung zum Router herstellen
void connect_wifi() {
  // Kein SSID konfiguriert → direkt AP starten
  if (strlen(cfg.wifi_ssid) == 0) {
    Serial.println("WiFi: Kein SSID konfiguriert → AP-Modus");
    start_ap();
    return;
  }

  const char* host = strlen(cfg.hostname) > 0 ? cfg.hostname : cfg.device_id;
  WiFi.setHostname(host);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  // Statische IP
  if (strlen(cfg.static_ip) > 0 && strlen(cfg.static_gw) > 0) {
    IPAddress ip, gw, mask, dns;
    ip.fromString(cfg.static_ip);
    gw.fromString(cfg.static_gw);
    mask.fromString(strlen(cfg.static_mask) > 0 ? cfg.static_mask : "255.255.255.0");
    dns.fromString(strlen(cfg.static_dns)   > 0 ? cfg.static_dns  : cfg.static_gw);
    WiFi.config(ip, gw, mask, dns);
    Serial.printf("WiFi: Statische IP=%s  GW=%s\n", cfg.static_ip, cfg.static_gw);
  }

  Serial.printf("WiFi: verbinde mit '%s' …\n", cfg.wifi_ssid);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);

  const uint32_t timeout_ms = 15000;
  const uint32_t t0         = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi OK  IP=%-16s  RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (MDNS.begin(host)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("mDNS: http://%s.local\n", host);
    }
    web_setup();
    current_wifi_mode = XB_WIFI_STA;
#if HAS_RGB_LED
    led_set_color(0, 50, 0); // Grün = verbunden
#endif
  } else {
    // Router nicht erreichbar → AP-Fallback
    Serial.println("WiFi: Verbindung fehlgeschlagen → AP-Fallback");
    start_ap();
  }
}

void connect_mqtt() {
  if (current_wifi_mode != XB_WIFI_STA) return;  // Kein MQTT im AP-Modus
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(512);
  String client_id = String(DEVICE_ID) + "_" + String(random(0xFFFF), HEX);
  if (mqttClient.connect(client_id.c_str(), MQTT_USER, MQTT_PASS)) {
    mqtt_connected = true;
    Serial.println("MQTT connected");
    mqttClient.publish(MQTT_TOPIC_STATUS, "{\"event\":\"online\"}", true);
  } else {
    Serial.printf("MQTT failed: %d\n", mqttClient.state());
    mqtt_connected = false;
  }
}

// Periodisch prüfen ob STA-Verbindung verloren → AP-Fallback oder Reconnect
void wifi_maintain() {
  static uint32_t last_check = 0;
  if (millis() - last_check < 10000) return;
  last_check = millis();

  if (current_wifi_mode == XB_WIFI_STA && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi: Verbindung verloren → versuche Reconnect …");
    WiFi.reconnect();
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000)
      delay(300);

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi: Reconnect OK  IP=%s\n",
                    WiFi.localIP().toString().c_str());
#if HAS_RGB_LED
      led_set_color(0, 50, 0);
#endif
    } else {
      Serial.println("WiFi: Reconnect fehlgeschlagen → AP-Fallback");
      start_ap();
    }
  }

  // Im AP-Modus: prüfen ob Router wieder da → zurück zu STA wechseln
  if (current_wifi_mode == XB_WIFI_AP &&
      strlen(cfg.wifi_ssid) > 0 &&
      strcmp(cfg.wifi_ssid, "DEIN_WLAN_NAME") != 0) {
    Serial.println("WiFi: AP-Modus – prüfe ob Router jetzt erreichbar …");
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000)
      delay(300);

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi: Router wieder da! IP=%s\n",
                    WiFi.localIP().toString().c_str());
      current_wifi_mode = XB_WIFI_STA;
      cfgServer.stop();   // AP-Config-Server stoppen
      const char* host = strlen(cfg.hostname) > 0 ? cfg.hostname : cfg.device_id;
      MDNS.begin(host);
      MDNS.addService("http", "tcp", 80);
      web_setup();        // Config-Server neu starten (STA-IP)
      connect_mqtt();
#if HAS_RGB_LED
      led_set_color(0, 50, 0);
#endif
    } else {
      // Immer noch kein Router → zurück zu AP
      start_ap();
    }
  }
}

// ─── SETUP & LOOP ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== XanBus to CAN Bridge v1.2 ===");
#ifdef BOARD_ESP32C3
  Serial.println("Board: ESP32-C3");
  pinMode(RGB_PIN, OUTPUT);
  led_set_color(0, 0, 80); // Blau = Boot
#else
  Serial.println("Board: ESP32");
  pinMode(LED_STATUS_PIN, OUTPUT);
#endif

  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_RE_PIN, LOW);

  RS485_SERIAL.begin(250000, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.println("RS485 ready @ 250kbps");

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CAN_CS_PIN);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("MCP2515 CAN ready @ 250kbps");

  memset(fp_buffers, 0, sizeof(fp_buffers));
  cfg_load();
  cfg_ext_load();

  // WiFi: versuche Router → Fallback auf eigenen AP
  connect_wifi();
  connect_mqtt();

  Serial.println("System ready. Listening on XanBus...");
}

void loop() {
  // MQTT keepalive (nur im STA-Modus)
  if (current_wifi_mode == XB_WIFI_STA) {
    if (!mqttClient.connected()) {
      static uint32_t last_mqtt_retry = 0;
      if (millis() - last_mqtt_retry > 5000) {
        last_mqtt_retry = millis();
        connect_mqtt();
      }
    } else {
      mqttClient.loop();
    }
  }

  read_rs485();
  web_loop();        // Config-Portal (STA oder AP)
  wifi_maintain();   // Reconnect / AP→STA-Wechsel alle 10s prüfen

  if (millis() - last_mqtt_publish > (uint32_t)cfg_mqtt_interval && mqtt_connected) {
    last_mqtt_publish = millis();
    if (g_battery.valid)  mqtt_publish_battery();
    if (g_solar.valid)    mqtt_publish_solar();
    if (g_inverter.valid) mqtt_publish_inverter();
    if (g_charger.valid)  mqtt_publish_charger();
    static uint32_t last_stat = 0;
    if (millis() - last_stat > 10000) { last_stat = millis(); mqtt_publish_status(); }
  }

  // Status-LED
  if (millis() - last_status_blink > (stats.frames_rx > 0 ? 200 : 1000)) {
    last_status_blink = millis();
    static bool blink_state = false;
    blink_state = !blink_state;
#if HAS_RGB_LED
    if (current_wifi_mode == XB_WIFI_AP) {
      // Orange blinken = AP-Modus, warte auf Config
      led_set_color(blink_state ? 80 : 0, blink_state ? 40 : 0, 0);
    } else if (stats.frames_rx > 0) {
      led_set_color(0, blink_state ? 80 : 0, 0); // Grün = Daten
    } else {
      led_set_color(0, 0, blink_state ? 20 : 0); // Blau langsam = idle
    }
#else
    if (LED_STATUS_PIN >= 0) digitalWrite(LED_STATUS_PIN, blink_state);
#endif
  }
}
