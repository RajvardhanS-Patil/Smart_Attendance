// ============================================================
//  SmartAttend — ESP32 RFID Attendance System
//  FIXES:
//   [1] TM1637 shows live blinking clock correctly
//   [2] LCD backlight wakes INSTANTLY on ultrasonic detect
//   [3] Backlight logic decoupled from sonar poll interval
//   [4] All previous fixes retained
// ============================================================

#include <ESP32Servo.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// ── USER CONFIG ───────────────────────────────────────────
#define WIFI_SSID     "LIGHT"
#define WIFI_PASSWORD "aaaaAA1."
#define GAS_URL       "https://script.google.com/macros/s/AKfycbwpvGWDUt5ZVAlyr7reHMJKEgLA7gHYC3TFonNvr68sFUinuTrF_1L6GdgdCc8Xq1XS/exec"

// ── LCD CONFIG ────────────────────────────────────────────
#define LCD_ADDRESS   0x27
#define LCD_COLS      16
#define LCD_ROWS      2
#define LCD_TIMEOUT   5000UL

// ── PIN DEFINITIONS ───────────────────────────────────────
#define SS_PIN    5
#define RST_PIN   4
#define LED_GREEN 2
#define LED_RED   15
#define BUZZER    13
#define TRIG_PIN  25
#define ECHO_PIN  26
#define CLK_PIN   32
#define DIO_PIN   33
#define SERVO_PIN 27

// ── TIMING CONSTANTS ─────────────────────────────────────
#define DETECT_CM            40
#define LCD_SLEEP_MS         5000UL
#define SCAN_COOLDOWN        500UL
#define SONAR_INTERVAL       50UL       // <-- reduced to 50ms for fast detection
#define NTP_INTERVAL         60000UL
#define SEG_INTERVAL         500UL      // <-- 500ms so colon blinks at 1Hz
#define SERVO_OPEN_MS        700UL
#define BOOT_BACKLIGHT_GRACE 15000UL

// ── CLASS TIME CUTOFF ─────────────────────────────────────
#define LATE_HOUR   9
#define LATE_MINUTE 0

// ── STUDENT DATABASE ──────────────────────────────────────
struct Student {
  const char* uid;
  const char* name;
  const char* roll;
};

Student knownCards[] = {
  { "610B8117", "Rajvardhan Patil", "1070901" },
  { "39E447B7", "Sanika Patil",     "1070903" },
  { "493546B7", "Sharwari Patil",   "1070905" },
  { "4957B8B7", "Ganesh Pingale",   "1070919" },
  { "29B7D9B",  "Harsh",            "1070972" },
  { "43774C16", "Raj Patil",        "1070973" },
};
const int STUDENT_COUNT = sizeof(knownCards) / sizeof(knownCards[0]);

// ── GLOBALS ───────────────────────────────────────────────
MFRC522           rfid(SS_PIN, RST_PIN);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
WiFiUDP           ntpUDP;
NTPClient         timeClient(ntpUDP, "pool.ntp.org", 19800);
TM1637Display     segDisplay(CLK_PIN, DIO_PIN);
Servo             gateServo;

unsigned long lastScanMillis     = 0;
unsigned long lastResultMillis   = 0;
unsigned long lastSegUpdate      = 0;
unsigned long lastNtpUpdate      = 0;
unsigned long lastSonarCheck     = 0;
unsigned long lastDetectedMillis = 0;
unsigned long servoOpenedAt      = 0;
unsigned long bootTime           = 0;

bool showingResult = false;
bool backlightOn   = false;
bool servoIsOpen   = false;
bool colonState    = false;   // <-- track colon separately for TM1637
String lastUID     = "";

// ── FUNCTION PROTOTYPES ───────────────────────────────────
void showIdle();
void connectWiFi();
void logToSheets(String uid, String name, String roll,
                 String status, String timeStr, String dateStr);
String urlEncode(String str);
String lcdPad(String s, int len);
void greenFlash(int times);
void redFlash(int times);
void yellowSignal();
void blinkBoth(int times);
void errorLoop();
float getDistanceCM();
void updateSegDisplay();
void wakeBacklight();    // <-- new helper

// ── ULTRASONIC ────────────────────────────────────────────
float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  if (dur == 0) return 999.0;
  return dur * 0.034 / 2.0;
}

// ── WAKE BACKLIGHT INSTANTLY ──────────────────────────────
// Call this any time something is detected or a card is scanned.
// It turns the backlight on immediately without waiting for sonar poll.
void wakeBacklight() {
  lastDetectedMillis = millis();
  if (!backlightOn) {
    lcd.backlight();
    backlightOn = true;
    if (!showingResult) showIdle();
  }
}

// ── 4-DIGIT DISPLAY ───────────────────────────────────────
// Key fix: colonState toggles every SEG_INTERVAL (500ms) → colon blinks 1Hz.
// showNumberDecEx with leadingZero=true always shows HH:MM (e.g. 0930).
void updateSegDisplay() {
  int h = timeClient.getHours();
  int m = timeClient.getMinutes();

  // Guard: if NTP gave 0:00 at startup before sync, still show something
  int timeNum = h * 100 + m;

  colonState = !colonState;
  uint8_t dots = colonState ? 0b01000000 : 0b00000000;
  segDisplay.showNumberDecEx(timeNum, dots, true); // true = leading zero
}

// ── SETUP ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, LOW);
  pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   LOW);
  pinMode(BUZZER,    OUTPUT); digitalWrite(BUZZER,    LOW);
  pinMode(TRIG_PIN,  OUTPUT);
  pinMode(ECHO_PIN,  INPUT);

  // LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  backlightOn = true;
  bootTime    = millis();
  lcd.setCursor(0, 0); lcd.print(lcdPad("SmartAttend",  16));
  lcd.setCursor(0, 1); lcd.print(lcdPad("Booting...",   16));
  blinkBoth(2);

  // Segment display — brightness once, show dashes while waiting for NTP
  segDisplay.setBrightness(5);
  // Show "----" (all segments minus) while NTP not ready
  uint8_t dash[4] = { SEG_G, SEG_G, SEG_G, SEG_G };
  segDisplay.setSegments(dash);

  // RFID
  SPI.begin();
  rfid.PCD_Init();
  delay(50);
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("RFID version: 0x"); Serial.println(v, HEX);
  if (v == 0x00 || v == 0xFF) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(lcdPad("RFID ERROR!",      16));
    lcd.setCursor(0, 1); lcd.print(lcdPad("Check P4/P5 pins", 16));
    errorLoop();
  }

  // WiFi
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(lcdPad("Connecting WiFi", 16));
  lcd.setCursor(0, 1); lcd.print(lcdPad("Please wait...",  16));
  connectWiFi();

  // NTP — try a few times to make sure we get a real time
  timeClient.begin();
  for (int i = 0; i < 5; i++) {
    if (timeClient.update()) break;
    delay(500);
  }
  lastNtpUpdate = millis();

  String t = timeClient.getFormattedTime();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(lcdPad("Time synced IST", 16));
  lcd.setCursor(0, 1); lcd.print(lcdPad(t,                  16));
  delay(1500);

  // Segment display — NOW show real time
  colonState = true;
  updateSegDisplay();
  lastSegUpdate = millis();

  // Servo
  gateServo.attach(SERVO_PIN);
  gateServo.write(0);

  showIdle();
  greenFlash(2);
  Serial.println("=== SmartAttend Ready ===");
}

// ── MAIN LOOP ─────────────────────────────────────────────
void loop() {

  unsigned long now = millis();

  // ── Segment display: toggle colon every 500ms ──────────
  if (now - lastSegUpdate >= SEG_INTERVAL) {
    lastSegUpdate = now;
    updateSegDisplay();
  }

  // ── NTP sync: once per minute ──────────────────────────
  if (now - lastNtpUpdate >= NTP_INTERVAL) {
    lastNtpUpdate = now;
    timeClient.update();
  }

  // ── Ultrasonic: poll every 50ms for fast backlight wake ─
  if (now - lastSonarCheck >= SONAR_INTERVAL) {
    lastSonarCheck = now;
    float dist = getDistanceCM();

    if (dist < DETECT_CM) {
      // Object detected — wake backlight IMMEDIATELY
      wakeBacklight();
    } else {
      // No object — check if backlight should sleep
      bool inGrace = (now - bootTime) < BOOT_BACKLIGHT_GRACE;
      if (backlightOn && !inGrace &&
          (now - lastDetectedMillis) > LCD_SLEEP_MS) {
        lcd.noBacklight();
        backlightOn = false;
      }
    }
  }

  // ── Servo close: non-blocking ──────────────────────────
  if (servoIsOpen && (now - servoOpenedAt) >= SERVO_OPEN_MS) {
    gateServo.write(0);
    servoIsOpen = false;
  }

  // ── Auto-return to idle ────────────────────────────────
  if (showingResult && (now - lastResultMillis) > LCD_TIMEOUT) {
    showingResult = false;
    showIdle();
  }

  // ── WiFi reconnect ─────────────────────────────────────
  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  // ── RFID: no card → exit ───────────────────────────────
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  // ── Build UID ──────────────────────────────────────────
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // ── Cooldown ───────────────────────────────────────────
  now = millis();
  if (uid == lastUID && (now - lastScanMillis) < SCAN_COOLDOWN) return;
  lastUID        = uid;
  lastScanMillis = now;

  // ── Lookup student ─────────────────────────────────────
  Student* found = nullptr;
  for (int i = 0; i < STUDENT_COUNT; i++) {
    if (uid.equals(knownCards[i].uid)) { found = &knownCards[i]; break; }
  }

  // ── Time & date ────────────────────────────────────────
  int hour   = timeClient.getHours();
  int minute = timeClient.getMinutes();

  unsigned long epochIST = timeClient.getEpochTime();
  long days = epochIST / 86400L;
  long z    = days + 719468;
  long era  = (z >= 0 ? z : z - 146096) / 146097;
  long doe  = z - era * 146097;
  long yoe  = (doe - doe/1460 + doe/36524 - doe/146096) / 365;
  long y    = yoe + era * 400;
  long doy  = doe - (365*yoe + yoe/4 - yoe/100);
  long mp   = (5*doy + 2) / 153;
  int  day  = doy - (153*mp+2)/5 + 1;
  int  mon  = mp + (mp < 10 ? 3 : -9);
  if (mon <= 2) y++;

  String dateStr = (day < 10 ? "0" : "") + String((int)day)  + "/" +
                   (mon < 10 ? "0" : "") + String((int)mon)  + "/" +
                   String((int)y);

  String ampm    = (hour >= 12) ? "PM" : "AM";
  int    h12     = (hour % 12 == 0) ? 12 : (hour % 12);
  String timeDisp = (h12 < 10 ? "0" : "") + String(h12) + ":" +
                    (minute < 10 ? "0" : "") + String(minute) + ampm;
  String time24   = (hour < 10 ? "0" : "") + String(hour) + ":" +
                    (minute < 10 ? "0" : "") + String(minute);

  // ── Wake backlight on scan ─────────────────────────────
  wakeBacklight();

  // ── Handle scan result ─────────────────────────────────
  if (found != nullptr) {
    bool   isLate = (hour > LATE_HOUR) ||
                    (hour == LATE_HOUR && minute >= LATE_MINUTE);
    String status = isLate ? "LATE" : "PRESENT";
    String name   = String(found->name);

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(lcdPad(name, 16));
    if (isLate) {
      lcd.setCursor(0, 1); lcd.print(lcdPad("LATE  " + timeDisp, 16));
      yellowSignal();
    } else {
      lcd.setCursor(0, 1); lcd.print(lcdPad("IN    " + timeDisp, 16));
      greenFlash(2);
    }

    gateServo.write(90);
    servoOpenedAt = millis();
    servoIsOpen   = true;

    delay(100);
    lcd.setCursor(0, 1); lcd.print(lcdPad("Logging...", 16));
    logToSheets(uid, name, String(found->roll), status, time24, dateStr);

    lcd.setCursor(0, 1);
    lcd.print(isLate ? lcdPad("LATE  " + timeDisp, 16)
                     : lcdPad("IN    " + timeDisp, 16));

  } else {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(lcdPad("Access DENIED", 16));
    lcd.setCursor(0, 1); lcd.print(lcdPad("Gate  LOCKED",  16));
    redFlash(3);
    logToSheets(uid, "UNKNOWN", "N/A", "DENIED", time24, dateStr);
  }

  showingResult    = true;
  lastResultMillis = millis();
}

// ── LCD HELPERS ───────────────────────────────────────────
void showIdle() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(lcdPad("SmartAttend",     16));
  lcd.setCursor(0, 1); lcd.print(lcdPad("Scan your card..", 16));
}

String lcdPad(String s, int len) {
  while ((int)s.length() < len) s += " ";
  if ((int)s.length() > len) s = s.substring(0, len);
  return s;
}

// ── WIFI ──────────────────────────────────────────────────
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500); attempts++;
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK IP: "); Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(lcdPad("WiFi Connected!", 16));
    lcd.setCursor(0, 1); lcd.print(lcdPad(WiFi.localIP().toString(), 16));
    delay(1200);
    greenFlash(1);
  } else {
    Serial.println("WiFi FAILED — offline mode");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(lcdPad("WiFi FAILED!",    16));
    lcd.setCursor(0, 1); lcd.print(lcdPad("Offline mode...", 16));
    redFlash(2);
    delay(1500);
  }
}

// ── LOG TO GOOGLE SHEETS ──────────────────────────────────
void logToSheets(String uid, String name, String roll,
                 String status, String timeStr, String dateStr) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Sheets skip — no WiFi");
    return;
  }
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = String(GAS_URL) +
               "?uid="    + uid               +
               "&name="   + urlEncode(name)   +
               "&roll="   + roll              +
               "&status=" + status            +
               "&time="   + timeStr           +
               "&date="   + urlEncode(dateStr);
  http.begin(client, url);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  Serial.print("Sheets HTTP: "); Serial.println(code);
  http.end();
}

String urlEncode(String str) {
  String out = "";
  for (int i = 0; i < (int)str.length(); i++) {
    char c = str[i];
    if (c == ' ')      out += "+";
    else if (c == '/') out += "%2F";
    else               out += String(c);
  }
  return out;
}

// ── LED + BUZZER ──────────────────────────────────────────
void greenFlash(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(BUZZER, HIGH); delay(80); digitalWrite(BUZZER, LOW);
    delay(200); digitalWrite(LED_GREEN, LOW); delay(120);
  }
}

void redFlash(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER, HIGH); delay(40); digitalWrite(BUZZER, LOW);
    delay(60);
    digitalWrite(BUZZER, HIGH); delay(40); digitalWrite(BUZZER, LOW);
    delay(200); digitalWrite(LED_RED, LOW); delay(100);
  }
}

void yellowSignal() {
  digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, HIGH);
  digitalWrite(BUZZER, HIGH); delay(300); digitalWrite(BUZZER, LOW);
  delay(400);
  digitalWrite(LED_GREEN, LOW); digitalWrite(LED_RED, LOW);
}

void blinkBoth(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, HIGH);
    delay(150);
    digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, LOW);
    delay(150);
  }
}

void errorLoop() {
  while (true) {
    digitalWrite(LED_RED, HIGH); delay(100);
    digitalWrite(LED_RED, LOW);  delay(100);
  }
}