// AstraCYD v0.2.3 — read-only OBD-II dashboard for ESP32-2432S028R.
// Target: classic ESP32-WROOM-32, 2.8" ILI9341 + XPT2046 CYD.
// IMPORTANT: This is a first hardware prototype; confirm your CYD revision.
// All commands below are OBD-II READ commands. No ECU coding/flashing.

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <BluetoothSerial.h>
#include <esp_gap_bt_api.h>
#include <Preferences.h>
#include <math.h>
#include <atomic>

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP requires a classic ESP32 (not ESP32-S3)."
#endif

static constexpr int SCREEN_W = 320;
static constexpr int SCREEN_H = 240;
static constexpr int BACKLIGHT_PIN = 21;
// CYD touch uses a DIFFERENT SPI peripheral and pins than its TFT.
static constexpr int TP_IRQ = 36;
static constexpr int TP_CS = 33;
static constexpr int TP_SCK = 25;
static constexpr int TP_MISO = 39;
static constexpr int TP_MOSI = 32;


// XPT2046 raw calibration varies; these are INITIAL estimates only.
// For a mirrored touch, change the corresponding MIN/MAX order.
static constexpr int TOUCH_X_MIN = 240;
static constexpr int TOUCH_X_MAX = 3800;
static constexpr int TOUCH_Y_MIN = 240;
static constexpr int TOUCH_Y_MAX = 3800;

TFT_eSPI tft;
SPIClass touchSPI(VSPI);
XPT2046_Touchscreen touch(TP_CS, TP_IRQ);
BluetoothSerial elm;
Preferences prefs;

constexpr uint16_t BG    = 0x0841;    // near black
constexpr uint16_t PANEL = 0x18C3;
constexpr uint16_t LINE  = 0x39C7;
constexpr uint16_t AMBER = 0xFCA0;
constexpr uint16_t WHITE = 0xFFFF;
constexpr uint16_t GREY  = 0xA514;
constexpr uint16_t GREEN = 0x37E6;
constexpr uint16_t RED   = 0xF800;

enum Page : uint8_t { HOME, ENGINE, TRIP, SETTINGS, ADAPTERS };
Page page = HOME;

struct Telemetry {
  int rpm = -1;
  int speed = -1;
  int coolant = -999;
  int load = -1;
  int throttle = -1;
  float voltage = NAN;
  uint32_t rpmAt = 0, speedAt = 0, coolantAt = 0, loadAt = 0, throttleAt = 0, voltageAt = 0;
} data;

bool pin0000 = false;
bool wasConnected = false;
bool touchHeld = false;
bool reconnectRequested = false;
std::atomic<bool> connecting{false};
std::atomic<bool> scanDone{false};
QueueHandle_t scanQueue = nullptr;
bool scanRequested = false;
bool scanOk = false;
bool btReady = false;
bool hasAdapter = false;
uint8_t adapterAddress[6]{};
String adapterName;
constexpr int SCAN_LIMIT = 32;
constexpr int SCAN_ROWS = 4;
struct ScanDevice {
  uint8_t address[6];
  char name[33];
};
ScanDevice devices[SCAN_LIMIT];
int deviceCount = 0;
int deviceOffset = 0;
uint32_t ecuAt = 0;
uint32_t nextBtAttempt = 0;
uint32_t lastDraw = 0;
uint32_t lastTouch = 0;
uint32_t sessionStart = 0;
uint32_t successfulPackets = 0;
String btState = "OFFLINE";

const char* initCommands[] = {"ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATSP0", "ATST32"};
constexpr uint8_t INIT_COUNT = sizeof(initCommands) / sizeof(initCommands[0]);
const char* pollCommands[] = {"010C", "010D", "010C", "0104", "010D", "0105", "010C", "0111", "010D", "ATRV"};
constexpr uint8_t POLL_COUNT = sizeof(pollCommands) / sizeof(pollCommands[0]);
uint8_t initIndex = 0;
uint8_t pollIndex = 0;
bool awaiting = false;
String currentCommand;
String rxBuffer;
uint32_t sentAt = 0;

bool fresh(uint32_t timestamp, uint32_t ageMs = 7000) {
  return timestamp != 0 && (uint32_t)(millis() - timestamp) < ageMs;
}

void putText(int x, int y, const String &value, int font, uint16_t col, uint8_t datum = TL_DATUM) {
  tft.setTextDatum(datum);
  tft.setTextColor(col, BG);
  tft.drawString(value, x, y, font);
}

String fmtValue(bool valid, int value, const char* unit = "") {
  if (!valid) return "--";
  return String(value) + unit;
}

// F17 ratios depend on gearbox variant/final drive. These are CALIBRATION SEEDS,
// not verified values for this specific car. Estimate only with clutch engaged.
// Approximate engine RPM divided by road speed in km/h (gears 1..5).
float rpmPerKmh[5] = {123.0f, 70.5f, 47.0f, 36.7f, 29.2f};

int estimatedGear() {
  if (!fresh(data.rpmAt, 2000) || !fresh(data.speedAt, 2000) || data.speed < 12 || data.rpm < 900) return 0;
  const int32_t sampleSkew = static_cast<int32_t>(data.rpmAt - data.speedAt);
  if (sampleSkew < -1000 || sampleSkew > 1000) return 0;
  const float observed = float(data.rpm) / data.speed;
  int nearest = 0;
  float minError = 999;
  for (int i = 0; i < 5; ++i) {
    float err = fabsf(observed - rpmPerKmh[i]) / rpmPerKmh[i];
    if (err < minError) { minError = err; nearest = i + 1; }
  }
  return (minError <= 0.12f) ? nearest : 0;
}

enum ShiftAdvice { NO_ADVICE, SHIFT_UP, SHIFT_DOWN };
ShiftAdvice getAdvice(int gear) {
  if (!gear || !fresh(data.rpmAt) || !fresh(data.loadAt, 3500) || !fresh(data.throttleAt, 3500)) return NO_ADVICE;
  if (gear < 5 && data.rpm > 3100 && data.throttle < 40 && data.load < 65)
    return SHIFT_UP;
  if (gear > 1 && data.rpm < 1550 && data.load > 40 && data.throttle > 18 &&
      data.speed * rpmPerKmh[gear - 2] < 3800.0f)
    return SHIFT_DOWN;
  return NO_ADVICE;
}

enum Field { BT_STATUS, PRIMARY, SECONDARY, THIRD, FOURTH, TEMPERATURE, ENGINE_LOAD, VOLTAGE, SCAN_ADDR_1, SCAN_ADDR_2, SCAN_ADDR_3, SCAN_ADDR_4, FIELD_COUNT };

struct DisplayField {
  String value;
  uint16_t color = 0;
};
DisplayField fields[FIELD_COUNT];
int drawnPage = -1;
int drawnAdvice = -1;

void updateText(Field field, int x, int y, const String &value, int font,
                uint16_t color, uint8_t datum, uint16_t width, uint16_t background = BG) {
  DisplayField &previous = fields[field];
  if (previous.value == value && previous.color == color) return;
  previous.value = value;
  previous.color = color;
  tft.setTextDatum(datum);
  tft.setTextColor(color, background);
  tft.setTextPadding(width);
  tft.drawString(value, x, y, font);
  tft.setTextPadding(0);
}

void drawHeader() {
  String status = btState;
  if (wasConnected) {
    status = initIndex < INIT_COUNT ? "BT / INIT" :
        (fresh(ecuAt) ? "OBD OK" : "WAIT ECU");
  }
  updateText(BT_STATUS, 310, 6, status, 2, fresh(ecuAt) ? GREEN : GREY, TR_DATUM, 140, PANEL);
}

void drawTabs() {
  const char* tabs[] = {"DRIVE", "ENGINE", "TRIP", "SETUP"};
  tft.fillRect(0, 209, 320, 31, PANEL);
  for (int i = 0; i < 4; ++i) {
    int x = i * 80;
    const bool active = (page == ADAPTERS ? SETTINGS : page) == static_cast<Page>(i);
    if (active) tft.fillRoundRect(x + 2, 211, 76, 27, 4, LINE);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(active ? AMBER : WHITE, active ? LINE : PANEL);
    tft.drawString(tabs[i], x + 40, 216, 2);
  }
}

void labelValue(Field field, int y, const char *label, const String &value, bool newPage) {
  if (newPage) {
    tft.fillRoundRect(9, y, 302, 30, 5, PANEL);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(WHITE, PANEL);
    tft.drawString(label, 17, y + 6, 2);
  }
  updateText(field, 302, y + 6, value, 2, AMBER, TR_DATUM, 120, PANEL);
}

void drawHome(bool newPage) {
  if (newPage) {
    tft.drawRoundRect(7, 37, 88, 110, 8, LINE);
    tft.drawRoundRect(102, 37, 116, 110, 8, AMBER);
    tft.drawRoundRect(225, 37, 88, 110, 8, LINE);
    putText(51, 49, "RPM", 2, GREY, TC_DATUM);
    putText(160, 45, "GEAR ~", 2, GREY, TC_DATUM);
    putText(269, 49, "KM/H", 2, GREY, TC_DATUM);
  }

  const int gear = estimatedGear();
  const ShiftAdvice advice = getAdvice(gear);
  updateText(PRIMARY, 51, 88, fmtValue(fresh(data.rpmAt), data.rpm), 4, WHITE, MC_DATUM, 80);
  updateText(SECONDARY, 160, 91, gear ? String(gear) : "-", 7, AMBER, MC_DATUM, 90);
  updateText(THIRD, 269, 88, fmtValue(fresh(data.speedAt), data.speed), 4, WHITE, MC_DATUM, 80);

  if (drawnAdvice != advice) {
    drawnAdvice = advice;
    const uint16_t background = advice == NO_ADVICE ? PANEL : 0x4100;
    const char *message = "GEAR ESTIMATE / CHECK";
    if (advice == SHIFT_UP) message = "^   SHIFT UP";
    if (advice == SHIFT_DOWN) message = "v   SHIFT DOWN";
    tft.fillRoundRect(12, 152, 296, 26, 6, background);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(advice == NO_ADVICE ? GREY : AMBER, background);
    tft.drawString(message, 160, 157, 2);
  }

  const String temp = fmtValue(fresh(data.coolantAt), data.coolant, "C");
  const String volt = fresh(data.voltageAt) ? String(data.voltage, 1) + "V" : "--";
  updateText(TEMPERATURE, 12, 188, "ECT " + temp, 2, WHITE, TL_DATUM, 90);
  updateText(ENGINE_LOAD, 160, 188, "LOAD " + fmtValue(fresh(data.loadAt), data.load, "%"),
             2, WHITE, TC_DATUM, 116);
  updateText(VOLTAGE, 310, 188, volt, 2, WHITE, TR_DATUM, 65);
}

void drawEngine(bool newPage) {
  labelValue(PRIMARY, 38, "COOLANT", fmtValue(fresh(data.coolantAt), data.coolant, " C"), newPage);
  labelValue(SECONDARY, 73, "ENGINE LOAD", fmtValue(fresh(data.loadAt), data.load, " %"), newPage);
  labelValue(THIRD, 108, "THROTTLE", fmtValue(fresh(data.throttleAt), data.throttle, " %"), newPage);
  const String volt = fresh(data.voltageAt) ? String(data.voltage, 2) + " V" : "--";
  labelValue(FOURTH, 143, "OBD VOLTAGE", volt, newPage);
  if (newPage) putText(16, 185, "ECU DATA / READ ONLY", 2, GREY);
}

void drawTrip(bool newPage) {
  labelValue(PRIMARY, 38, "SPEED", fmtValue(fresh(data.speedAt), data.speed, " km/h"), newPage);
  labelValue(SECONDARY, 73, "ENGINE SPEED", fmtValue(fresh(data.rpmAt), data.rpm, " rpm"), newPage);
  labelValue(THIRD, 108, "RX RESPONSES", String(successfulPackets), newPage);
  labelValue(FOURTH, 143, "SESSION", String((millis() - sessionStart) / 60000UL) + " min", newPage);
  if (newPage) putText(16, 185, "NO GPS / FUEL ESTIMATE YET", 2, GREY);
}

void drawSettings(bool newPage) {
  labelValue(PRIMARY, 38, "SCAN BT (TAP)", "OPEN", newPage);
  labelValue(SECONDARY, 73, "BT PIN (TAP)", pin0000 ? "0000" : "1234", newPage);
  labelValue(THIRD, 108, "RECONNECT (TAP)", "NOW", newPage);
  labelValue(FOURTH, 143, "ADAPTER", hasAdapter ? adapterName : "NONE", newPage);
  if (newPage) putText(16, 182, "Touch tab bar to change page", 2, GREY);
}

String shortName(String name, unsigned limit) {
  for (unsigned i = 0; i < name.length(); ++i) {
    if (static_cast<uint8_t>(name[i]) < 32 || static_cast<uint8_t>(name[i]) > 126) name.setCharAt(i, '?');
  }
  return name.length() > limit ? name.substring(0, limit - 3) + "..." : name;
}

void drawAdapters(bool newPage) {
  const bool scanning = scanRequested || connecting.load();
  const char *caption = scanning ? "Scanning Bluetooth Classic..." :
      (!scanOk ? "Scan failed" : (deviceCount ? "Tap your adapter" : "No devices found"));
  updateText(TEMPERATURE, 12, 31, caption, 2, WHITE, TL_DATUM, 296);
  const Field rows[] = {PRIMARY, SECONDARY, THIRD, FOURTH};
  for (int row = 0; row < SCAN_ROWS; ++row) {
    const int y = 51 + row * 29;
    const int index = deviceOffset + row;
    if (newPage) tft.fillRect(9, y, 302, 27, PANEL);
    String label = " ";
    String address = " ";
    if (!scanning && index < deviceCount) {
      const ScanDevice &entry = devices[index];
      label = entry.name[0] ? shortName(entry.name, 20) : String("(unnamed)");
      char text[18];
      snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
               entry.address[0], entry.address[1], entry.address[2],
               entry.address[3], entry.address[4], entry.address[5]);
      address = text;
    }
    updateText(rows[row], 16, y + 5, label, 2, AMBER, TL_DATUM, 177, PANEL);
    updateText(static_cast<Field>(SCAN_ADDR_1 + row), 304, y + 5, address, 2, WHITE, TR_DATUM, 105, PANEL);
  }
  if (newPage) {
    putText(12, 180, "PREV", 2, WHITE);
    putText(105, 180, "RESCAN", 2, WHITE);
    putText(200, 180, "NEXT", 2, WHITE);
    putText(312, 180, "BACK", 2, WHITE, TR_DATUM);
  }
}
void redraw() {
  if (drawnPage < 0) {
    tft.fillRect(0, 0, SCREEN_W, 28, PANEL);
    tft.drawFastHLine(0, 27, SCREEN_W, AMBER);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(AMBER, PANEL);
    tft.drawString("ASTRA H", 8, 6, 2);
  }
  drawHeader();

  const bool newPage = drawnPage != page;
  if (newPage) {
    drawnPage = page;
    drawnAdvice = -1;
    for (int i = PRIMARY; i < FIELD_COUNT; ++i) fields[i] = DisplayField{};
    tft.fillRect(0, 28, SCREEN_W, 181, BG);
    drawTabs();
  }
  switch (page) {
    case HOME: drawHome(newPage); break;
    case ENGINE: drawEngine(newPage); break;
    case TRIP: drawTrip(newPage); break;
    case SETTINGS: drawSettings(newPage); break;
    case ADAPTERS: drawAdapters(newPage); break;
  }
}
void requestScan() {
  deviceOffset = 0;
  scanRequested = true;
  page = ADAPTERS;
}

void selectAdapter(const ScanDevice &device) {
  if (prefs.putBytes("elmAddr", device.address, sizeof(adapterAddress)) != sizeof(adapterAddress)) {
    Serial.println("Could not save adapter address");
    return;
  }
  adapterName = shortName(device.name[0] ? device.name : "Unnamed", 12);
  prefs.putString("elmName", adapterName);
  memcpy(adapterAddress, device.address, sizeof(adapterAddress));
  hasAdapter = true;
  reconnectRequested = true;
  page = SETTINGS;
}

void handleTouch() {
  if (!touch.tirqTouched() || !touch.touched()) {
    touchHeld = false;
    return;
  }
  if (touchHeld) return;
  if ((uint32_t)(millis() - lastTouch) < 300) return;
  TS_Point p = touch.getPoint();
  if (p.z < 250) return;
  touchHeld = true;
  int x = constrain((int)map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, SCREEN_W - 1), 0, SCREEN_W - 1);
  int y = constrain((int)map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, SCREEN_H - 1), 0, SCREEN_H - 1);
  Serial.printf("touch raw=(%d,%d,%d) mapped=(%d,%d)\n", p.x, p.y, p.z, x, y);
  lastTouch = millis();
  if (y >= 207) {
    page = (Page)constrain(x / 80, 0, 3);
  } else if (page == ADAPTERS) {
    if (y >= 175 && y < 207 && x >= 262) {
      page = SETTINGS;
    } else if (!scanRequested && !connecting.load()) {
      if (y >= 51 && y < 167) {
        const int index = deviceOffset + (y - 51) / 29;
        if (index < deviceCount) selectAdapter(devices[index]);
      } else if (y >= 175 && y < 207) {
        if (x < 83 && deviceOffset > 0) deviceOffset -= SCAN_ROWS;
        else if (x >= 83 && x < 172) requestScan();
        else if (x >= 172 && x < 262 && deviceOffset + SCAN_ROWS < deviceCount) deviceOffset += SCAN_ROWS;
      }
    }
  } else if (page == SETTINGS && x > 8 && x < 312) {
    if (y >= 38 && y < 68) {
      requestScan();
    }
    if (y >= 73 && y <= 104) {
      pin0000 = !pin0000;
      prefs.putBool("pin0000", pin0000);
      reconnectRequested = true;
    } else if (y >= 108 && y <= 140) {
      reconnectRequested = true;
    }
  }
  redraw();
}

void resetTelemetry() {
  data = Telemetry{};
  ecuAt = 0;
  initIndex = 0;
  pollIndex = 0;
  awaiting = false;
  rxBuffer = "";
}

int hexPair(const String &s, int offset) {
  if (offset < 0 || offset + 2 > (int)s.length() || !isxdigit(s[offset]) || !isxdigit(s[offset + 1])) return -1;
  char temp[3] = {s[offset], s[offset + 1], 0};
  char *end = nullptr;
  long n = strtol(temp, &end, 16);
  return (end == temp + 2) ? (int)n : -1;
}

void parseOBD(const String &command, String answer) {
  // With ATH0 ATS0, replies normally look like "410C0FA0".
  answer.toUpperCase();
  answer.replace(" ", "");
  answer.replace("\r", "\n");
  uint32_t previousPackets = successfulPackets;
  const String prefix = "\n41" + command.substring(2);
  int start = ("\n" + answer).indexOf(prefix);
  // Prefix index in the padded string equals the payload index in answer.
  int a = -1, b = -1;
  if (start >= 0) {
    a = hexPair(answer, start + 4);
    b = hexPair(answer, start + 6);
  }
  if (command == "010C" && a >= 0 && b >= 0) {
    data.rpm = ((a << 8) | b) / 4; data.rpmAt = millis(); ++successfulPackets;
  } else if (command == "010D" && a >= 0) {
    data.speed = a; data.speedAt = millis(); ++successfulPackets;
  } else if (command == "0105" && a >= 0) {
    data.coolant = a - 40; data.coolantAt = millis(); ++successfulPackets;
  } else if (command == "0104" && a >= 0) {
    data.load = (a * 100 + 127) / 255; data.loadAt = millis(); ++successfulPackets;
  } else if (command == "0111" && a >= 0) {
    data.throttle = (a * 100 + 127) / 255; data.throttleAt = millis(); ++successfulPackets;
  } else if (command == "ATRV") {
    answer.trim();
    int v = answer.indexOf('V');
    if (v > 0) {
      // After ATE0 the response starts with a voltage, e.g. "14.2V".
      String rawV = answer.substring(0, v);
      int dot = rawV.indexOf('.');
      if (dot > 0 && dot <= 3) {
        float n = rawV.toFloat();
        if (n >= 8.0f && n < 20.0f) { data.voltage = n; data.voltageAt = millis(); ++successfulPackets; }
      }
    }
  }
  if (command.startsWith("01") && successfulPackets != previousPackets) ecuAt = millis();
  Serial.printf("ELM [%s] => %s\n", command.c_str(), answer.c_str());
}

void sendELM(const char* cmd) {
  currentCommand = cmd;
  rxBuffer = "";
  awaiting = true;
  sentAt = millis();
  elm.print(cmd);
  elm.print('\r');
}

void connectAdapter(void*) {
  elm.connect(adapterAddress);
  nextBtAttempt = millis() + 8000;
  connecting.store(false);
  vTaskDelete(nullptr);
}

void restartAdapter() {
  if (elm.connected()) elm.disconnect();
  resetTelemetry();
  wasConnected = false;
  btState = "BT RETRY";
  nextBtAttempt = millis() + 1000;
}

void scanCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *params) {
  if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT &&
      params->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
    scanDone.store(true);
  }
  if (event != ESP_BT_GAP_DISC_RES_EVT) return;

  ScanDevice entry{};
  memcpy(entry.address, params->disc_res.bda, sizeof(entry.address));
  for (int i = 0; i < params->disc_res.num_prop; ++i) {
    const auto &property = params->disc_res.prop[i];
    const uint8_t *name = nullptr;
    int length = 0;
    if (property.type == ESP_BT_GAP_DEV_PROP_BDNAME) {
      name = static_cast<const uint8_t*>(property.val);
      length = property.len;
    } else if (property.type == ESP_BT_GAP_DEV_PROP_EIR && !entry.name[0]) {
      auto *eir = static_cast<uint8_t*>(property.val);
      uint8_t size = 0;
      name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &size);
      if (!name) name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &size);
      length = size;
    }
    if (name && length > 0) {
      const int copied = min(length, static_cast<int>(sizeof(entry.name)) - 1);
      memcpy(entry.name, name, copied);
      entry.name[copied] = '\0';
    }
  }
  xQueueSend(scanQueue, &entry, 0);
}

void scanAdapters(void*) {
  if (elm.connected()) elm.disconnect();
  xQueueReset(scanQueue);
  deviceCount = 0;
  scanDone.store(false);
  // BluetoothSerial discovery refuses scans after an address has been selected.
  // Use a temporary GAP callback and restore the SPP stack before reconnecting.
  const bool registered = esp_bt_gap_register_callback(scanCallback) == ESP_OK;
  scanOk = registered && esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0) == ESP_OK;
  if (scanOk) {
    const uint32_t start = millis();
    while (millis() - start < 14000) {
      ScanDevice entry;
      if (xQueueReceive(scanQueue, &entry, pdMS_TO_TICKS(100)) == pdTRUE) {
        int index = 0;
        while (index < deviceCount && memcmp(devices[index].address, entry.address, sizeof(entry.address))) ++index;
        if (index < deviceCount && entry.name[0]) devices[index] = entry;
        else if (index == deviceCount && deviceCount < SCAN_LIMIT) devices[deviceCount++] = entry;
      } else if (scanDone.load()) {
        break;
      }
    }
    if (!scanDone.load()) esp_bt_gap_cancel_discovery();
  }
  if (registered) {
    elm.end();
    btReady = elm.begin("AstraCYD", true);
  }
  connecting.store(false);
  vTaskDelete(nullptr);
}
void pollOBD() {
  if (connecting.load()) return;
  if (scanRequested) {
    scanRequested = false;
    resetTelemetry();
    wasConnected = false;
    btState = "BT SCANNING";
    scanOk = false;
    deviceCount = 0;
    if (btReady && scanQueue) {
      connecting.store(true);
      if (xTaskCreate(scanAdapters, "ELM scan", 4096, nullptr, 1, nullptr) != pdPASS) {
        connecting.store(false);
      }
    }
    return;
  }
  if (btState == "BT SCANNING") {
    btState = btReady ? "SELECT ADAPTER" : "BT ERROR";
    nextBtAttempt = millis();
  }
  if (!btReady || page == ADAPTERS) return;
  if (reconnectRequested) {
    reconnectRequested = false;
    restartAdapter();
    return;
  }
  if (!elm.connected()) {
    if (wasConnected) {
      restartAdapter();
      btState = "BT LOST";
    }
    if (!hasAdapter) {
      btState = "SELECT ADAPTER";
      return;
    }
    if (btState == "BT CONNECT...") btState = "BT RETRY";
    if (static_cast<int32_t>(millis() - nextBtAttempt) >= 0) {
      btState = "BT CONNECT...";
      elm.setPin(pin0000 ? "0000" : "1234");
      connecting.store(true);
      if (xTaskCreate(connectAdapter, "ELM connect", 4096, nullptr, 1, nullptr) != pdPASS) {
        connecting.store(false);
        btState = "BT RETRY";
        nextBtAttempt = millis() + 8000;
      }
    }
    return;
  }
  if (!wasConnected) resetTelemetry();
  wasConnected = true;
  while (elm.available()) {
    char c = (char)elm.read();
    if (c == '>') {
      if (awaiting) {
        if (initIndex < INIT_COUNT) {
          String response = rxBuffer;
          response.toUpperCase();
          response.trim();
          if (response.isEmpty() || response.indexOf('?') >= 0 ||
              (initIndex > 0 && response.indexOf("OK") < 0)) {
            restartAdapter();
            return;
          }
          ++initIndex;
        } else {
          parseOBD(currentCommand, rxBuffer);
        }
        awaiting = false;
      }
      rxBuffer = "";
    } else if (rxBuffer.length() < 300 && (c >= 32 || c == '\r' || c == '\n')) {
      rxBuffer += c;
    }
  }
  if (awaiting && (uint32_t)(millis() - sentAt) > (initIndex == 0 ? 3500UL : 8000UL)) {
    Serial.printf("ELM timeout: %s\n", currentCommand.c_str());
    restartAdapter();
    return;
  }
  if (!awaiting) {
    if (initIndex < INIT_COUNT) sendELM(initCommands[initIndex]);
    else { sendELM(pollCommands[pollIndex]); pollIndex = (pollIndex + 1) % POLL_COUNT; }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(BG);
  tft.setTextColor(WHITE, BG);
  touchSPI.begin(TP_SCK, TP_MISO, TP_MOSI, TP_CS);
  touch.begin(touchSPI);
  touch.setRotation(1);
  prefs.begin("astracyd", false);
  pin0000 = prefs.getBool("pin0000", false);
  hasAdapter = prefs.getBytesLength("elmAddr") == sizeof(adapterAddress) &&
      prefs.getBytes("elmAddr", adapterAddress, sizeof(adapterAddress)) == sizeof(adapterAddress);
  adapterName = prefs.getString("elmName", "Saved");
  scanQueue = xQueueCreate(20, sizeof(ScanDevice));
  btReady = elm.begin("AstraCYD", true);
  sessionStart = millis();
  btState = btReady ? "BT START" : "BT ERROR";
  nextBtAttempt = millis() + 1000;
  redraw();
  Serial.println("AstraCYD v0.2.3 ready; ELM queries are READ-ONLY");
}

void loop() {
  pollOBD();
  handleTouch();
  if ((uint32_t)(millis() - lastDraw) > 500) {
    redraw();
    lastDraw = millis();
  }
  delay(5);
}
