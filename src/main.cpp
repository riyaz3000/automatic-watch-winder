#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- Hardware configuration --------------------
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

// ESP32-DevKitC-32E header labels: VP=GPIO36, VN=GPIO39, IO34=GPIO34, IO35=GPIO35.
// These pins are input-only and have no internal pull-ups, so use 10k external
// pull-ups unless the OLED button board already provides stable active-LOW outputs.
constexpr uint8_t BTN_UP = 36;      // K1 -> VP / GPIO36
constexpr uint8_t BTN_DOWN = 39;    // K2 -> VN / GPIO39
constexpr uint8_t BTN_SELECT = 34;  // K3 -> IO34 / GPIO34, hold to confirm WiFi password
constexpr uint8_t BTN_BACK = 35;    // K4 -> IO35 / GPIO35, hold from status for WiFi setup
constexpr bool BUTTONS_USE_INTERNAL_PULLUPS = false;

// 28BYJ-48 + ULN2003 on ESP32-DevKitC-32E.
// AccelStepper HALF4WIRE constructor order is IN1, IN3, IN2, IN4.
constexpr uint8_t MOTOR_PINS[3][4] = {
  {13, 27, 14, 26},  // W1 logical IN1=IO13, IN2=IO14, IN3=IO27, IN4=IO26
  {25, 33, 32, 23},  // W2 logical IN1=IO25, IN2=IO32, IN3=IO33, IN4=IO23
  {19, 18, 17, 16},  // W3 logical IN1=IO19, IN2=IO17, IN3=IO18, IN4=IO16
};

// Optional active-LOW physical enable switches. Leave as -1 if not fitted.
constexpr int8_t PHYSICAL_ENABLE_SWITCH_PINS[3] = {-1, -1, -1};

constexpr long STEPS_PER_TURN = 4096;
constexpr uint32_t BURST_PERIOD_MS = 30000;
constexpr uint16_t MIN_TPD = 0;
constexpr uint16_t MAX_TPD = 2400;
constexpr uint16_t TPD_STEP = 25;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr char SETUP_AP_SSID[] = "WatchWinder-Setup";
constexpr uint8_t DNS_PORT = 53;

enum DirectionMode : uint8_t {
  DIR_CLOCKWISE = 0,
  DIR_ANTICLOCKWISE = 1,
  DIR_BIDIRECTIONAL = 2,
};

struct WinderSettings {
  bool enabled = true;
  uint16_t tpd = 650;
  DirectionMode direction = DIR_BIDIRECTIONAL;
};

struct AppSettings {
  WinderSettings winders[3];
  String ssid;
  String password;
};

AppSettings settings;
Preferences prefs;
WebServer server(80);
DNSServer dnsServer;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

const char *directionName(DirectionMode mode) {
  switch (mode) {
    case DIR_CLOCKWISE: return "CW";
    case DIR_ANTICLOCKWISE: return "CCW";
    case DIR_BIDIRECTIONAL: return "BI";
  }
  return "?";
}

DirectionMode nextDirection(DirectionMode mode) {
  return static_cast<DirectionMode>((static_cast<uint8_t>(mode) + 1) % 3);
}

class DebouncedButton {
 public:
  explicit DebouncedButton(uint8_t pin) : pin_(pin) {}

  void begin() {
    pinMode(pin_, BUTTONS_USE_INTERNAL_PULLUPS ? INPUT_PULLUP : INPUT);
    stableState_ = digitalRead(pin_);
    lastReading_ = stableState_;
  }

  void update(uint32_t now) {
    clicked_ = false;
    longPressed_ = false;
    bool reading = digitalRead(pin_);

    if (reading != lastReading_) {
      lastDebounceMs_ = now;
      lastReading_ = reading;
    }

    if ((now - lastDebounceMs_) >= debounceMs_ && reading != stableState_) {
      stableState_ = reading;
      if (isPressed()) {
        pressedAtMs_ = now;
        longSent_ = false;
      } else {
        if (!longSent_ && pressedAtMs_ != 0) clicked_ = true;
        pressedAtMs_ = 0;
      }
    }

    if (isPressed() && !longSent_ && pressedAtMs_ != 0 && (now - pressedAtMs_) >= longPressMs_) {
      longPressed_ = true;
      longSent_ = true;
    }
  }

  bool clicked() const { return clicked_; }
  bool longPressed() const { return longPressed_; }
  bool isPressed() const { return stableState_ == LOW; }

 private:
  uint8_t pin_;
  bool stableState_ = HIGH;
  bool lastReading_ = HIGH;
  bool clicked_ = false;
  bool longPressed_ = false;
  bool longSent_ = false;
  uint32_t lastDebounceMs_ = 0;
  uint32_t pressedAtMs_ = 0;
  static constexpr uint16_t debounceMs_ = 35;
  static constexpr uint16_t longPressMs_ = 900;
};

DebouncedButton btnUp(BTN_UP);
DebouncedButton btnDown(BTN_DOWN);
DebouncedButton btnSelect(BTN_SELECT);
DebouncedButton btnBack(BTN_BACK);

class WinderMotor {
 public:
  WinderMotor(uint8_t in1, uint8_t in3, uint8_t in2, uint8_t in4)
      : stepper_(AccelStepper::HALF4WIRE, in1, in3, in2, in4) {}

  void begin(WinderSettings *cfg, uint8_t index, int8_t enableSwitchPin) {
    cfg_ = cfg;
    index_ = index;
    enableSwitchPin_ = enableSwitchPin;
    if (enableSwitchPin_ >= 0) pinMode(enableSwitchPin_, INPUT_PULLUP);
    stepper_.setMaxSpeed(700.0);
    stepper_.setAcceleration(350.0);
    nextBurstMs_ = millis() + 1000UL + index_ * 500UL;
  }

  void update(uint32_t now) {
    stepper_.run();
    moving_ = stepper_.distanceToGo() != 0;

    if (!isActive() || cfg_->tpd == 0) {
      if (stepper_.distanceToGo() == 0) stepper_.disableOutputs();
      moving_ = false;
      return;
    }

    if (stepper_.distanceToGo() != 0 || static_cast<int32_t>(now - nextBurstMs_) < 0) return;

    double exactSteps = (static_cast<double>(cfg_->tpd) * STEPS_PER_TURN) /
                        (86400000.0 / BURST_PERIOD_MS);
    exactSteps += fractionalSteps_;
    long burstSteps = static_cast<long>(exactSteps);
    fractionalSteps_ = exactSteps - burstSteps;
    if (burstSteps <= 0) burstSteps = 1;

    int8_t sign = 1;
    if (cfg_->direction == DIR_ANTICLOCKWISE) sign = -1;
    if (cfg_->direction == DIR_BIDIRECTIONAL) {
      sign = bidirectionalFlip_ ? -1 : 1;
      bidirectionalFlip_ = !bidirectionalFlip_;
    }

    stepper_.enableOutputs();
    stepper_.move(sign * burstSteps);
    moving_ = true;
    nextBurstMs_ = now + BURST_PERIOD_MS;
  }

  bool moving() const { return moving_; }
  bool isActive() const {
    bool physicalEnabled = enableSwitchPin_ < 0 || digitalRead(enableSwitchPin_) == LOW;
    return cfg_->enabled && physicalEnabled;
  }

 private:
  mutable AccelStepper stepper_;
  WinderSettings *cfg_ = nullptr;
  uint8_t index_ = 0;
  int8_t enableSwitchPin_ = -1;
  uint32_t nextBurstMs_ = 0;
  double fractionalSteps_ = 0;
  bool bidirectionalFlip_ = false;
  bool moving_ = false;
};

WinderMotor motors[3] = {
  WinderMotor(MOTOR_PINS[0][0], MOTOR_PINS[0][1], MOTOR_PINS[0][2], MOTOR_PINS[0][3]),
  WinderMotor(MOTOR_PINS[1][0], MOTOR_PINS[1][1], MOTOR_PINS[1][2], MOTOR_PINS[1][3]),
  WinderMotor(MOTOR_PINS[2][0], MOTOR_PINS[2][1], MOTOR_PINS[2][2], MOTOR_PINS[2][3]),
};

void saveSettings() {
  prefs.begin("winder", false);
  for (uint8_t i = 0; i < 3; i++) {
    String prefix = "w" + String(i);
    prefs.putBool((prefix + "en").c_str(), settings.winders[i].enabled);
    prefs.putUShort((prefix + "tpd").c_str(), settings.winders[i].tpd);
    prefs.putUChar((prefix + "dir").c_str(), static_cast<uint8_t>(settings.winders[i].direction));
  }
  prefs.putString("ssid", settings.ssid);
  prefs.putString("pass", settings.password);
  prefs.end();
}

void loadSettings() {
  prefs.begin("winder", true);
  for (uint8_t i = 0; i < 3; i++) {
    String prefix = "w" + String(i);
    settings.winders[i].enabled = prefs.getBool((prefix + "en").c_str(), true);
    settings.winders[i].tpd = prefs.getUShort((prefix + "tpd").c_str(), 650);
    uint8_t dir = prefs.getUChar((prefix + "dir").c_str(), DIR_BIDIRECTIONAL);
    settings.winders[i].direction = dir <= DIR_BIDIRECTIONAL ? static_cast<DirectionMode>(dir) : DIR_BIDIRECTIONAL;
  }
  settings.ssid = prefs.getString("ssid", "");
  settings.password = prefs.getString("pass", "");
  prefs.end();
}

enum ScreenMode {
  SCREEN_STATUS,
  SCREEN_MENU,
  SCREEN_EDIT_TPD,
  SCREEN_WIFI_SCAN,
  SCREEN_WIFI_PICK,
  SCREEN_WIFI_PASSWORD,
  SCREEN_WIFI_CONNECTING,
};

ScreenMode screen = SCREEN_STATUS;
uint8_t menuIndex = 0;
uint8_t selectedWinder = 0;
uint8_t selectedNetwork = 0;
int16_t wifiNetworkCount = 0;
String wifiPasswordDraft;
uint8_t wifiCharIndex = 0;
bool displayDirty = true;
uint32_t lastDisplayMs = 0;
uint32_t wifiConnectStartedMs = 0;
bool apMode = false;
bool dnsPortalRunning = false;

const char WIFI_CHARS[] = " abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()-_=+[]{};:'\",.<>/?\\|`~";

String htmlEscape(const String &value);

void markDisplayDirty() {
  displayDirty = true;
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(SETUP_AP_SSID);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  dnsPortalRunning = true;
  apMode = true;
  markDisplayDirty();
}

void beginWifiConnect() {
  if (settings.ssid.length() == 0) {
    startAccessPoint();
    return;
  }

  if (apMode) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_STA);
    dnsServer.stop();
    dnsPortalRunning = false;
  }

  WiFi.begin(settings.ssid.c_str(), settings.password.c_str());
  wifiConnectStartedMs = millis();
  markDisplayDirty();
}

void serviceWifi(uint32_t now) {
  if (dnsPortalRunning) dnsServer.processNextRequest();

  if (WiFi.status() == WL_CONNECTED && apMode) {
    dnsServer.stop();
    dnsPortalRunning = false;
    WiFi.softAPdisconnect(true);
    apMode = false;
    wifiConnectStartedMs = 0;
    markDisplayDirty();
    return;
  }

  if (!apMode && WiFi.status() == WL_CONNECTED && wifiConnectStartedMs != 0) {
    wifiConnectStartedMs = 0;
    markDisplayDirty();
  }

  if (WiFi.status() != WL_CONNECTED && wifiConnectStartedMs != 0 &&
      (now - wifiConnectStartedMs) > WIFI_CONNECT_TIMEOUT_MS) {
    wifiConnectStartedMs = 0;
    if (apMode) {
      markDisplayDirty();
      return;
    }
    startAccessPoint();
  }
}

String activeIpAddress() {
  if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
  if (apMode) return WiFi.softAPIP().toString();
  return "Not connected";
}

String wifiStatusText() {
  if (WiFi.status() == WL_CONNECTED) return "Connected to " + htmlEscape(WiFi.SSID());
  if (apMode) return "Setup hotspot active";
  if (settings.ssid.length() > 0) return "Connecting to " + htmlEscape(settings.ssid);
  return "No WiFi configured";
}

String htmlEscape(const String &value) {
  String out;
  out.reserve(value.length());
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else out += c;
  }
  return out;
}

String dashboardHtml() {
  String html = F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>Watch Winder</title><style>"
                  "body{font-family:system-ui,Segoe UI,sans-serif;margin:24px;background:#f6f7f9;color:#16181d}"
                  "main{max-width:760px;margin:auto}section{background:white;border:1px solid #d9dde5;border-radius:8px;padding:16px;margin:14px 0}"
                  "label{display:block;margin:10px 0 4px;font-weight:600}input,select,button{font:inherit;padding:8px;border-radius:6px;border:1px solid #aeb6c4}"
                  "button{background:#1f6feb;color:white;border-color:#1f6feb;cursor:pointer}.row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px}"
                  ".status{background:#eef6ff;border-color:#b7d7ff}.muted{color:#596273}"
                  "@media(max-width:640px){.row{grid-template-columns:1fr}}</style></head><body><main>"
                  "<h1>3-Watch Winder</h1><section class='status'><h2>WiFi</h2><p>");
  html += wifiStatusText();
  html += F("</p><p><strong>Web address:</strong> http://");
  html += activeIpAddress();
  html += F("/</p>");
  if (apMode) {
    html += F("<p class='muted'>Connect your phone to ");
    html += SETUP_AP_SSID;
    html += F(", then open http://192.168.4.1/ if this page does not open automatically.</p>");
  }
  html += F("<form method='POST' action='/wifi'>"
            "<label>Home WiFi SSID</label><input name='ssid' value='");
  html += htmlEscape(settings.ssid);
  html += F("'><label>Home WiFi Password</label><input name='pass' type='password' value='");
  html += htmlEscape(settings.password);
  html += F("'><p><button type='submit'>Save WiFi and Connect</button></p></form></section>"
            "<form method='POST' action='/save'><div class='row'>");

  for (uint8_t i = 0; i < 3; i++) {
    html += F("<section><h2>Winder ");
    html += String(i + 1);
    html += F("</h2><label><input type='checkbox' name='en");
    html += String(i);
    html += F("' ");
    if (settings.winders[i].enabled) html += F("checked");
    html += F("> Enabled</label><label>TPD</label><input type='number' min='0' max='2400' step='25' name='tpd");
    html += String(i);
    html += F("' value='");
    html += String(settings.winders[i].tpd);
    html += F("'><label>Direction</label><select name='dir");
    html += String(i);
    html += F("'>");
    for (uint8_t d = 0; d < 3; d++) {
      html += F("<option value='");
      html += String(d);
      html += F("'");
      if (settings.winders[i].direction == d) html += F(" selected");
      html += F(">");
      html += directionName(static_cast<DirectionMode>(d));
      html += F("</option>");
    }
    html += F("</select></section>");
  }

  html += F("</div><button type='submit'>Save Winder Settings</button></form>"
            "</main></body></html>");
  return html;
}

void handleRoot() {
  server.send(200, "text/html", dashboardHtml());
}

void handleSave() {
  for (uint8_t i = 0; i < 3; i++) {
    settings.winders[i].enabled = server.hasArg("en" + String(i));
    int requestedTpd = server.arg("tpd" + String(i)).toInt();
    settings.winders[i].tpd = static_cast<uint16_t>(constrain(requestedTpd, static_cast<int>(MIN_TPD), static_cast<int>(MAX_TPD)));
    uint8_t dir = static_cast<uint8_t>(constrain(server.arg("dir" + String(i)).toInt(), 0, 2));
    settings.winders[i].direction = static_cast<DirectionMode>(dir);
  }
  saveSettings();
  markDisplayDirty();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleWifiSave() {
  settings.ssid = server.arg("ssid");
  settings.password = server.arg("pass");
  saveSettings();
  server.sendHeader("Location", "/");
  server.send(303);
  beginWifiConnect();
  markDisplayDirty();
}

void setupServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/gen_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.on("/fwlink", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/wifi", HTTP_POST, handleWifiSave);
  server.onNotFound(handleRoot);
  server.begin();
}

const char *menuLabel(uint8_t idx) {
  static char label[20];
  uint8_t w = idx / 3;
  uint8_t item = idx % 3;
  if (idx < 9) {
    const char *field = item == 0 ? "Enable" : item == 1 ? "TPD" : "Direction";
    snprintf(label, sizeof(label), "W%u %s", w + 1, field);
    return label;
  }
  return idx == 9 ? "WiFi Setup" : "Back";
}

void drawStatus() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED || apMode) {
    display.print(F("IP "));
    display.print(activeIpAddress());
  } else {
    display.print(F("Watch Winder"));
  }

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t y = 16 + i * 14;
    display.setCursor(0, y);
    display.printf("W%u %s", i + 1, motors[i].isActive() ? "ON " : "OFF");
    display.setCursor(44, y);
    display.printf("%4u", settings.winders[i].tpd);
    display.setCursor(76, y);
    display.print(directionName(settings.winders[i].direction));
    display.setCursor(108, y);
    display.print(motors[i].moving() ? "*" : "-");
  }
  display.setCursor(0, 56);
  if (WiFi.status() == WL_CONNECTED || apMode) {
    display.print(WiFi.status() == WL_CONNECTED ? F("WiFi dashboard ready") : F("Setup hotspot active"));
  } else {
    display.print(F("K3 menu  K4 hold WiFi"));
  }
  display.display();
}

void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Menu"));
  display.setCursor(44, 0);
  if (WiFi.status() == WL_CONNECTED || apMode) display.print(activeIpAddress());
  uint8_t first = menuIndex > 4 ? menuIndex - 4 : 0;
  for (uint8_t row = 0; row < 5 && first + row < 11; row++) {
    uint8_t idx = first + row;
    display.setCursor(0, 12 + row * 10);
    display.print(idx == menuIndex ? ">" : " ");
    display.print(menuLabel(idx));
    if (idx < 9) {
      uint8_t w = idx / 3;
      uint8_t item = idx % 3;
      display.setCursor(92, 12 + row * 10);
      if (item == 0) display.print(settings.winders[w].enabled ? "ON" : "OFF");
      if (item == 1) display.print(settings.winders[w].tpd);
      if (item == 2) display.print(directionName(settings.winders[w].direction));
    }
  }
  display.display();
}

void drawEditTpd() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("Winder %u TPD", selectedWinder + 1);
  display.setTextSize(2);
  display.setCursor(30, 24);
  display.print(settings.winders[selectedWinder].tpd);
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(F("K1/K2 change K3 save"));
  display.display();
}

void drawWifiScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (screen == SCREEN_WIFI_SCAN) {
    display.print(WiFi.scanComplete() == WIFI_SCAN_RUNNING ? F("Scanning WiFi...") : F("Starting scan..."));
  } else if (screen == SCREEN_WIFI_PICK) {
    display.print(F("Select network"));
    uint8_t first = selectedNetwork > 4 ? selectedNetwork - 4 : 0;
    for (uint8_t row = 0; row < 5 && first + row < wifiNetworkCount; row++) {
      int idx = first + row;
      display.setCursor(0, 12 + row * 10);
      display.print(idx == selectedNetwork ? ">" : " ");
      String ssid = WiFi.SSID(idx);
      if (ssid.length() > 18) ssid = ssid.substring(0, 18);
      display.print(ssid);
    }
  } else if (screen == SCREEN_WIFI_PASSWORD) {
    display.print(settings.ssid);
    display.setCursor(0, 16);
    display.print(F("Pass: "));
    String masked = "";
    for (uint8_t i = 0; i < wifiPasswordDraft.length(); i++) masked += "*";
    display.print(masked);
    display.setTextSize(2);
    display.setCursor(56, 34);
    display.print(WIFI_CHARS[wifiCharIndex] == ' ' ? '_' : WIFI_CHARS[wifiCharIndex]);
    display.setTextSize(1);
    display.setCursor(0, 56);
    display.print(F("K3 add/hold connect"));
  } else {
    display.print(F("Connecting..."));
    display.setCursor(0, 18);
    display.print(settings.ssid);
  }
  display.display();
}

void renderDisplay(uint32_t now) {
  if (!displayDirty && (now - lastDisplayMs) < 1000) return;
  lastDisplayMs = now;
  displayDirty = false;

  if (screen == SCREEN_STATUS) drawStatus();
  else if (screen == SCREEN_MENU) drawMenu();
  else if (screen == SCREEN_EDIT_TPD) drawEditTpd();
  else drawWifiScreen();
}

void startWifiScan() {
  WiFi.scanDelete();
  WiFi.scanNetworks(true, true);
  selectedNetwork = 0;
  wifiNetworkCount = 0;
  screen = SCREEN_WIFI_SCAN;
  markDisplayDirty();
}

void handleWifiScanState() {
  if (screen != SCREEN_WIFI_SCAN) return;
  int result = WiFi.scanComplete();
  if (result >= 0) {
    wifiNetworkCount = result;
    screen = result > 0 ? SCREEN_WIFI_PICK : SCREEN_MENU;
    markDisplayDirty();
  }
}

void handleButtons(uint32_t now) {
  btnUp.update(now);
  btnDown.update(now);
  btnSelect.update(now);
  btnBack.update(now);

  if (screen == SCREEN_STATUS) {
    if (btnSelect.clicked()) {
      screen = SCREEN_MENU;
      markDisplayDirty();
    }
    if (btnBack.longPressed()) startWifiScan();
    return;
  }

  if (screen == SCREEN_MENU) {
    if (btnUp.clicked() && menuIndex > 0) menuIndex--;
    if (btnDown.clicked() && menuIndex < 10) menuIndex++;
    if (btnBack.clicked()) screen = SCREEN_STATUS;
    if (btnSelect.clicked()) {
      if (menuIndex < 9) {
        selectedWinder = menuIndex / 3;
        uint8_t item = menuIndex % 3;
        if (item == 0) {
          settings.winders[selectedWinder].enabled = !settings.winders[selectedWinder].enabled;
          saveSettings();
        } else if (item == 1) {
          screen = SCREEN_EDIT_TPD;
        } else {
          settings.winders[selectedWinder].direction = nextDirection(settings.winders[selectedWinder].direction);
          saveSettings();
        }
      } else if (menuIndex == 9) {
        startWifiScan();
      } else {
        screen = SCREEN_STATUS;
      }
    }
    markDisplayDirty();
    return;
  }

  if (screen == SCREEN_EDIT_TPD) {
    if (btnUp.clicked() && settings.winders[selectedWinder].tpd < MAX_TPD) {
      uint16_t nextTpd = settings.winders[selectedWinder].tpd + TPD_STEP;
      settings.winders[selectedWinder].tpd = nextTpd > MAX_TPD ? MAX_TPD : nextTpd;
    }
    if (btnDown.clicked() && settings.winders[selectedWinder].tpd >= TPD_STEP) {
      settings.winders[selectedWinder].tpd -= TPD_STEP;
    }
    if (btnSelect.clicked()) {
      saveSettings();
      screen = SCREEN_MENU;
    }
    if (btnBack.clicked()) screen = SCREEN_MENU;
    markDisplayDirty();
    return;
  }

  if (screen == SCREEN_WIFI_PICK) {
    if (btnUp.clicked() && selectedNetwork > 0) selectedNetwork--;
    if (btnDown.clicked() && selectedNetwork + 1 < wifiNetworkCount) selectedNetwork++;
    if (btnSelect.clicked() && wifiNetworkCount > 0) {
      settings.ssid = WiFi.SSID(selectedNetwork);
      wifiPasswordDraft = "";
      wifiCharIndex = 0;
      screen = SCREEN_WIFI_PASSWORD;
    }
    if (btnBack.clicked()) screen = SCREEN_MENU;
    markDisplayDirty();
    return;
  }

  if (screen == SCREEN_WIFI_PASSWORD) {
    uint8_t maxChar = strlen(WIFI_CHARS) - 1;
    if (btnUp.clicked()) wifiCharIndex = wifiCharIndex == maxChar ? 0 : wifiCharIndex + 1;
    if (btnDown.clicked()) wifiCharIndex = wifiCharIndex == 0 ? maxChar : wifiCharIndex - 1;
    if (btnSelect.clicked() && wifiPasswordDraft.length() < 63) wifiPasswordDraft += WIFI_CHARS[wifiCharIndex];
    if (btnBack.clicked()) {
      if (wifiPasswordDraft.length() > 0) wifiPasswordDraft.remove(wifiPasswordDraft.length() - 1);
      else screen = SCREEN_WIFI_PICK;
    }
    if (btnSelect.longPressed()) {
      settings.password = wifiPasswordDraft;
      saveSettings();
      beginWifiConnect();
      screen = SCREEN_WIFI_CONNECTING;
    }
    markDisplayDirty();
    return;
  }

  if (screen == SCREEN_WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED || apMode || btnBack.clicked()) screen = SCREEN_STATUS;
    markDisplayDirty();
  }
}

void setup() {
  Serial.begin(115200);
  loadSettings();

  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  btnUp.begin();
  btnDown.begin();
  btnSelect.begin();
  btnBack.begin();

  for (uint8_t i = 0; i < 3; i++) {
    motors[i].begin(&settings.winders[i], i, PHYSICAL_ENABLE_SWITCH_PINS[i]);
  }

  beginWifiConnect();
  setupServer();
  markDisplayDirty();
}

void loop() {
  uint32_t now = millis();
  handleButtons(now);
  handleWifiScanState();
  serviceWifi(now);
  for (uint8_t i = 0; i < 3; i++) motors[i].update(now);
  server.handleClient();
  renderDisplay(now);
}
