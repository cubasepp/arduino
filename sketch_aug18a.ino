#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <driver/usb_serial_jtag.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <sys/time.h>

TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// I2C Pins fuer GY-291 (ADXL345)
#define I2C_SDA 18
#define I2C_SCL 17

// NEU: Interrupt-Ausgang INT1 des GY-291 muss hierher verkabelt werden!
#define PIN_INT1 16

#define BTN_1 0
#define BTN_2 14
#define LONG_PRESS_MS 1500

#define PIN_POWER_ON 15
#define PIN_LCD_BL 38

// --- ADXL345 Register (wir schreiben direkt, die Library kann das nicht) ---
#define ADXL_ADDR 0x53
#define REG_THRESH_ACT 0x24
#define REG_ACT_INACT_CTL 0x27
#define REG_BW_RATE 0x2C
#define REG_POWER_CTL 0x2D
#define REG_INT_ENABLE 0x2E
#define REG_INT_MAP 0x2F
#define REG_INT_SOURCE 0x30
#define REG_DATA_FORMAT 0x31

// 62.5 mg pro Schritt. 1 waere der empfindlichste Wert, liegt aber im
// Eigenrauschen des Sensors - dann weckt er sich selbst.
#define THRESH_ACT_VALUE 2

#define NOISE_FACTOR 4.0
#define MIN_THRESHOLD 0.05

#define WINDOW_MS 100
// Die Pumpe vibriert dauerhaft, Klopfen/Stossen nur kurz. Je laenger die
// Bestaetigung, desto robuster gegen Klopfen. Die Laufzeit zaehlt trotzdem
// ab der ersten Vibration. Anpassen anhand der "Burst"-Zeilen im Serial Monitor.
#define ON_TIME_MS 1500
#define GAP_MS 200                  // so lange darf die Vibration aussetzen
#define OFF_TIME_MS 3000

#define CAL_MS 1000

#define SLEEP_AFTER_MS 4000         // so lange Ruhe, dann Tiefschlaf
                                    // (Pumpenbestaetigung braucht nur 500 ms,
                                    //  laenger warten kostet nur Strom)
#define NO_SLEEP_AFTER_BOOT_MS 30000 // Fenster zum Programmieren nach dem Einschalten
#define DISPLAY_BUTTON_MS 20000UL   // Tastendruck: so lange bleibt es an
#define DISPLAY_RESULT_MS 10000UL   // nach Pumpenende: Ergebnis so lange zeigen
#define DISPLAY_BOOT_MS 30000UL     // nach dem Einschalten

// --- WLAN / Shot-Meldung ---
#define WIFI_SSID "MEIN_SSID"
#define WIFI_PASS "MEIN_PASSWORT"
#define SHOT_URL "https://mv-web.net/webhooks/coffee_shots"   // Ziel des POST
#define SHOT_API_HEADER "X-API-Key"   // Name des Headers, falls der Server einen anderen erwartet
#define SHOT_API_KEY "DEIN_API_KEY"   // NICHT committen
#define WIFI_TIMEOUT_MS 10000

// --- Messlog im Flash (Ringpuffer, ueberlebt Tiefschlaf und Neustart) ---
#define LOG_N 100
#define LOG_MIN_BURST_MS 300        // kuerzere verworfene Ausschlaege nicht loggen
#define LOG_DUMP_AFTER_MS 8000UL    // nach Kaltstart einmal automatisch ausgeben

// --- Display-Layouts: 0 = nur grosse Zeit (Standard), 1 = alles ---
#define LAYOUT_COUNT 2
#define LAYOUT_TIMER 0
#define CLICK_GAP_MS 500            // laenger Pause = Klickfolge zu Ende
#define TIMER_RESULT_MS 1000UL      // Timer-Layout: so lange bleibt die Zeit nach Pumpenende

#define PIN_BAT_VOLT 4
#define BAT_INTERVAL_MS 5000
#define BAT_USB_VOLT 4.35
#define BAT_MIN_VOLT 3.00

#define BAR_X 15
#define BAR_Y 135
#define BAR_W 290
#define BAR_H 25
#define BAR_INNER_X (BAR_X + 2)
#define BAR_INNER_Y (BAR_Y + 2)
#define BAR_INNER_W (BAR_W - 4)
#define BAR_INNER_H (BAR_H - 4)

// Zaehler ueberleben den Tiefschlaf im RTC-Speicher
RTC_DATA_ATTR uint32_t rtcRunSeconds = 0;   // laufender Durchgang, wird zurueckgesetzt
RTC_DATA_ATTR uint32_t rtcTotalSec = 0;     // Summe ueber alle Durchgaenge
RTC_DATA_ATTR uint32_t rtcStarts = 0;
RTC_DATA_ATTR bool rtcWasSleeping = false;

float hfVibration = 0;
float maxVibration = 0;
float lastDrawnMax = -1;
int percent = 0;

float baselineMagnitude = 9.81;
float hfNoise = 0;
float pumpThreshold = 0.5;
float vibMax = 2.0;
bool calFromFlash = false;

bool pumpRunning = false;
int lastDrawnState = -1;
uint32_t lastDrawnRunSec = 0xFFFFFFFF;
uint32_t lastDrawnUptime = 0xFFFFFFFF;

unsigned long displayOnSinceMs = 0;   // fuer das Messlog
char displayReason = 'B';             // P=Pumpe, T=Taste, B=Boot, K=Kalibrierung
bool sessionPumpRan = false;
float runPeak = 0;
double runSumHf = 0;
long runN = 0;
unsigned long logDumpAt = 0;

unsigned long aboveSince = 0;
float burstPeak = 0;              // hoechster Wert des laufenden Ausschlags (nur Messung)
unsigned long lastAboveMs = 0;
unsigned long runStartMs = 0;

float savedMax = 0;
unsigned long lastSaveMs = 0;

unsigned long btnDownSince = 0;
bool longHandled = false;
bool wakePress = false;

uint8_t layout = 0;                 // wird im Flash gemerkt
uint32_t timerSec = 0;              // Zeit, die das Timer-Layout anzeigt (bleibt nach Pumpenende stehen)
uint32_t lastDrawnTimer = 0xFFFFFFFF;
int clickCount = 0;
unsigned long lastClickMs = 0;

bool backlightPending = false;     // Beleuchtung erst an, wenn das Bild fertig gezeichnet ist
bool displayReady = false;
bool displayOn = false;
unsigned long displayUntilMs = 0;   // bis wann das Display an bleiben soll

float batVoltage = 0;
int batPercent = -1;
int lastDrawnBat = -999;
unsigned long lastBatMs = 0;

bool wokeFromSleep = false;
bool wokeByButton = false;
unsigned long bootMs = 0;
bool logFirstWindow = true;   // erste Messung nach dem Weckruf immer melden


// =====================================================
// ADXL345 direkt ansprechen
// =====================================================

void adxlWrite(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


uint8_t adxlRead(uint8_t reg) {

  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL_ADDR, 1);

  return Wire.available() ? Wire.read() : 0;
}


// Betrieb: schnell messen, kein Interrupt
void adxlConfigureMeasure() {

  adxlWrite(REG_INT_ENABLE, 0x00);
  adxlWrite(REG_BW_RATE, 0x0B);       // 200 Hz, normaler Modus
  adxlWrite(REG_DATA_FORMAT, 0x29);   // INT aktiv LOW, volle Aufloesung, +/-4g
  adxlWrite(REG_POWER_CTL, 0x08);     // Messbetrieb

  adxlRead(REG_INT_SOURCE);           // altes Ereignis loeschen
}


// Schlafwache: Sensor beobachtet selbst und zieht INT1 auf LOW
void adxlConfigureWakeSource() {

  adxlWrite(REG_INT_ENABLE, 0x00);

  adxlWrite(REG_THRESH_ACT, THRESH_ACT_VALUE);

  // 0xF0 = AC-gekoppelt + X/Y/Z aktiv.
  // AC ist zwingend: sonst wuerde die Schwerkraft dauerhaft ausloesen.
  adxlWrite(REG_ACT_INACT_CTL, 0xF0);

  // Bewusst NICHT im Sparmodus: der rauscht so stark, dass die Weckschwelle
  // von selbst ausloest. Normalbetrieb kostet ~80 uA mehr - neben dem
  // Ruhestrom des Boards faellt das nicht ins Gewicht.
  adxlWrite(REG_BW_RATE, 0x0B);       // 200 Hz, normaler Modus
  adxlWrite(REG_DATA_FORMAT, 0x29);   // INT aktiv LOW
  adxlWrite(REG_INT_MAP, 0x00);       // Aktivitaet auf INT1
  adxlWrite(REG_POWER_CTL, 0x08);

  adxlRead(REG_INT_SOURCE);           // Zustand zuruecksetzen
  adxlWrite(REG_INT_ENABLE, 0x10);    // erst jetzt scharf schalten
}


// =====================================================
// MESSLOG (Flash-Ringpuffer, Ausgabe per Serial: 'l' = anzeigen, 'x' = loeschen)
// =====================================================

struct LogRec {
  uint32_t t;        // Sekunden seit dem Einschalten (laeuft im Tiefschlaf weiter)
  char type;         // W=Aufwachen  B=Ausschlag verworfen  P=Pumpe  D=Display
  char reason;
  uint16_t a, b, c, d;
};


uint16_t clamp16(long v) {
  return v < 0 ? 0 : (v > 65535 ? 65535 : (uint16_t)v);
}


uint16_t milli(float v) {
  return clamp16((long)(v * 1000.0f + 0.5f));
}


void logEvent(char type, char reason, long a, long b, long c, long d) {

  struct timeval tv;
  gettimeofday(&tv, NULL);

  LogRec r;
  r.t = (uint32_t)tv.tv_sec;
  r.type = type;
  r.reason = reason;
  r.a = clamp16(a);
  r.b = clamp16(b);
  r.c = clamp16(c);
  r.d = clamp16(d);

  uint32_t n = prefs.getUInt("lgn", 0);

  char key[8];
  snprintf(key, sizeof(key), "lg%u", (unsigned)(n % LOG_N));

  prefs.putBytes(key, &r, sizeof(r));
  prefs.putUInt("lgn", n + 1);
}


void dumpLog() {

  uint32_t n = prefs.getUInt("lgn", 0);
  uint32_t first = (n > LOG_N) ? n - LOG_N : 0;

  Serial.println("--- LOG START ---");
  Serial.println("nr,t_s,typ,grund,a,b,c,d");
  Serial.println("W Aufwachen (grund S=Sensor T=Taste C=Kaltstart): a=Ausloeser mg, d=Schwelle*1000");
  Serial.println("B Ausschlag verworfen: a=Dauer ms, b=Spitze*1000, d=Schwelle*1000");
  Serial.println("P Pumpe gelaufen: a=Sekunden, b=Spitze*1000, c=Mittel*1000, d=Schwelle*1000");
  Serial.println("D Display war an (grund P=Pumpe T=Taste B=Boot K=Kalibrierung): a=Dauer in 0.1 s, b=Pumpe lief 1/0");

  for (uint32_t i = first; i < n; i++) {

    char key[8];
    snprintf(key, sizeof(key), "lg%u", (unsigned)(i % LOG_N));

    LogRec r;
    if (prefs.getBytes(key, &r, sizeof(r)) != sizeof(r)) {
      continue;
    }

    Serial.printf("%u,%u,%c,%c,%u,%u,%u,%u\n", (unsigned)i, (unsigned)r.t,
                  r.type, r.reason, r.a, r.b, r.c, r.d);
  }

  Serial.println("--- LOG ENDE ---");
}


// =====================================================
// AKKU
// =====================================================

// 16 Messungen mit kleinem Abstand: direkt hintereinander gelesene Werte
// mitteln nur das schnelle Rauschen weg, nicht das langsame.
float readBatteryVolts() {

  uint32_t sum = 0;

  for (int i = 0; i < 16; i++) {
    sum += analogReadMilliVolts(PIN_BAT_VOLT);
    delay(2);
  }

  return (sum / 16.0) * 2.0 / 1000.0;
}


int batteryPercent(float v) {

  // Unterhalb von 3.98 V aus dem eigenen Entladelog (1000 mAh, 16.8 Tage
  // ab 3.98 V): Prozent = verbleibende Laufzeit bis 3.30 V. 3.30 V ist
  // 0 %, danach bleiben noch ein paar Stunden. Darueber Standardkurve.
  const float volts[]   = {3.30, 3.35, 3.40, 3.45, 3.50, 3.55, 3.60,
                           3.65, 3.70, 3.75, 3.80, 3.85, 3.90, 3.95,
                           3.98, 4.10, 4.20};
  const int   percent[] = {   0,    3,    4,    7,   11,   16,   20,
                             24,   27,   33,   44,   56,   63,   68,
                             80,   92,  100};
  const int n = sizeof(volts) / sizeof(volts[0]);

  if (v <= volts[0]) return 0;
  if (v >= volts[n - 1]) return 100;

  for (int i = 0; i < n - 1; i++) {
    if (v < volts[i + 1]) {
      float f = (v - volts[i]) / (volts[i + 1] - volts[i]);
      return percent[i] + f * (percent[i + 1] - percent[i]);
    }
  }

  return 100;
}


// =====================================================
// KALIBRIERUNG
// =====================================================

void applyCalibration() {

  pumpThreshold = hfNoise * NOISE_FACTOR;

  if (pumpThreshold < MIN_THRESHOLD) {
    pumpThreshold = MIN_THRESHOLD;
  }

  vibMax = pumpThreshold * 4.0;
}


bool loadCalibration() {

  if (!prefs.getBool("calok", false)) {
    return false;
  }

  baselineMagnitude = prefs.getFloat("base", 9.81);
  hfNoise = prefs.getFloat("hfn", 0.05);

  applyCalibration();

  return true;
}


float readMagnitude() {

  sensors_event_t e;
  accel.getEvent(&e);

  return sqrt(e.acceleration.x * e.acceleration.x
            + e.acceleration.y * e.acceleration.y
            + e.acceleration.z * e.acceleration.z);
}


void ensureDisplay();
void drawStaticScreen();


void doCalibration() {

  ensureDisplay();

  digitalWrite(PIN_LCD_BL, HIGH);
  displayOn = true;
  displayOnSinceMs = millis();
  displayReason = 'K';
  sessionPumpRan = false;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("Kalibriere...", 10, 5);
  tft.drawString("Maschine muss AUS sein", 10, 30);

  double sm = 0;
  long n = 0;

  unsigned long t0 = millis();

  while (millis() - t0 < CAL_MS) {
    sm += readMagnitude();
    n++;
  }

  baselineMagnitude = sm / n;

  double sumSq = 0;
  long n2 = 0;
  float prev = baselineMagnitude;

  t0 = millis();

  while (millis() - t0 < CAL_MS) {

    float m = readMagnitude();
    double d = m - prev;
    sumSq += d * d;
    prev = m;
    n2++;
  }

  hfNoise = sqrt(sumSq / n2);

  applyCalibration();

  prefs.putFloat("base", baselineMagnitude);
  prefs.putFloat("hfn", hfNoise);
  prefs.putBool("calok", true);

  calFromFlash = false;

  Serial.print("hfNoise=");
  Serial.print(hfNoise, 4);
  Serial.print("  threshold=");
  Serial.println(pumpThreshold, 3);
}


// =====================================================
// DISPLAY
// =====================================================

// Unter einer Stunde mm:ss, darueber h:mm - so tut sich sofort etwas
String formatRuntime(uint32_t sec) {

  char buf[12];

  if (sec < 60) {
    sprintf(buf, "%us", sec);            // unter einer Minute: reine Sekunden
  } else if (sec < 3600) {
    sprintf(buf, "%02u:%02u", sec / 60, sec % 60);
  } else {
    sprintf(buf, "%uh%02u", sec / 3600, (sec % 3600) / 60);
  }

  return String(buf);
}


void ensureDisplay() {

  if (displayReady) {
    return;
  }

  tft.init();
  digitalWrite(PIN_LCD_BL, LOW);   // init() kann die Beleuchtung selbst einschalten
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  displayReady = true;
}


void drawStaticScreen() {

  ensureDisplay();

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  lastDrawnTimer = 0xFFFFFFFF;

  if (layout == LAYOUT_TIMER) {
    lastDrawnState = -1;
    return;     // Timer-Layout zeichnet alles selbst
  }

  tft.setTextSize(2);
  tft.drawString("[B33]", 10, 5);

  tft.drawString("THR: " + String(pumpThreshold, 2), 170, 75);

  tft.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, TFT_WHITE);

  lastDrawnState = -1;
  lastDrawnMax = -1;
  lastDrawnBat = -999;
  lastDrawnRunSec = 0xFFFFFFFF;
  lastDrawnUptime = 0xFFFFFFFF;
}


void wakeDisplay(char reason) {

  ensureDisplay();

  displayOnSinceMs = millis();
  displayReason = reason;
  sessionPumpRan = false;

  tft.writecommand(0x11);   // Sleep Out
  delay(120);

  displayOn = true;

  drawStaticScreen();

  // Beleuchtung erst nach dem ersten kompletten Bild (siehe showBacklight),
  // sonst blitzt kurz der alte Inhalt bzw. ein halb gezeichnetes Bild auf
  backlightPending = true;
}


void showBacklight() {

  if (backlightPending) {
    digitalWrite(PIN_LCD_BL, HIGH);
    backlightPending = false;
  }
}


void blankDisplay() {

  if (!displayOn) {
    return;   // war schon aus - nicht nochmal melden
  }

  digitalWrite(PIN_LCD_BL, LOW);
  backlightPending = false;

  if (displayReady) {
    tft.writecommand(0x10);   // Sleep In - spart auch ohne Beleuchtung Strom
  }

  displayOn = false;

  logEvent('D', displayReason, (millis() - displayOnSinceMs) / 100,
           sessionPumpRan ? 1 : 0, 0, 0);

  Serial.println("Display aus.");
}


// =====================================================
// SHOT MELDEN (WLAN nur kurz an, sonst frisst es den Akku)
// =====================================================

bool sendShot(uint32_t sec, float peak, float mean) {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(100);
  }

  bool ok = false;

  if (WiFi.status() == WL_CONNECTED) {

    WiFiClientSecure client;
    client.setInsecure();   // kein Zertifikat-Check - fuer http:// egal

    HTTPClient http;
    http.setTimeout(5000);

    if (http.begin(client, SHOT_URL)) {

      http.addHeader("Content-Type", "application/json");
      http.addHeader(SHOT_API_HEADER, SHOT_API_KEY);

      String body = "{\"duration\":" + String(sec)
                  + ",\"starts\":" + String(rtcStarts)
                  + ",\"peak_vib\":" + String(peak, 3)
                  + ",\"mean_vib\":" + String(mean, 3)
                  + ",\"threshold\":" + String(pumpThreshold, 3)
                  + ",\"battery\":" + String(batVoltage, 2) + "}";

      int code = http.POST(body);

      Serial.print("POST -> ");
      Serial.println(code);

      ok = (code >= 200 && code < 300);
      http.end();
    }

  } else {

    Serial.println("WLAN nicht verbunden.");
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  return ok;
}


// =====================================================
// TIEFSCHLAF
// =====================================================

void saveCounters() {

  prefs.putUInt("totalsec", rtcTotalSec);
  prefs.putUInt("starts", rtcStarts);

  if (maxVibration != savedMax) {
    prefs.putFloat("maxvib", maxVibration);
    savedMax = maxVibration;
  }
}


void goToDeepSleep() {

  blankDisplay();

  Serial.println("-> Tiefschlaf.");
  Serial.flush();

  saveCounters();

  adxlConfigureWakeSource();

  // Peripherie-Spannung MUSS anbleiben, sonst verliert der Sensor Strom
  // und kann uns nicht mehr wecken.
  rtc_gpio_hold_en((gpio_num_t)PIN_POWER_ON);
  gpio_hold_en((gpio_num_t)PIN_LCD_BL);
  gpio_deep_sleep_hold_en();

  // Taste braucht im Schlaf einen Pullup, der Sensor-Ausgang nicht
  rtc_gpio_pullup_en((gpio_num_t)BTN_2);
  rtc_gpio_pulldown_dis((gpio_num_t)BTN_2);

  // Beide Quellen ziehen im Ereignisfall auf LOW
  esp_sleep_enable_ext1_wakeup(
    (1ULL << PIN_INT1) | (1ULL << BTN_2),
    ESP_EXT1_WAKEUP_ANY_LOW
  );

  rtcWasSleeping = true;

  esp_deep_sleep_start();
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  bootMs = millis();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  wokeFromSleep = (cause == ESP_SLEEP_WAKEUP_EXT1);

  if (wokeFromSleep) {

    uint64_t status = esp_sleep_get_ext1_wakeup_status();
    wokeByButton = (status & (1ULL << BTN_2)) != 0;

    Serial.print("Aufgewacht durch: ");
    if (wokeByButton) {
      Serial.println("TASTE");
    } else if (status & (1ULL << PIN_INT1)) {
      Serial.println("SENSOR (INT1)");
    } else {
      Serial.print("unklar, Maske=0x");
      Serial.println((uint32_t)status, HEX);
    }

  } else {

    Serial.println("Kaltstart (kein Aufwachen aus dem Tiefschlaf).");
  }

  // Halte-Zustaende der Pins wieder freigeben
  gpio_deep_sleep_hold_dis();
  rtc_gpio_hold_dis((gpio_num_t)PIN_POWER_ON);
  gpio_hold_dis((gpio_num_t)PIN_LCD_BL);

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(PIN_INT1, INPUT);

  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, LOW);

  prefs.begin("pumpe", false);

  maxVibration = prefs.getFloat("maxvib", 0);
  savedMax = maxVibration;

  layout = prefs.getUChar("layout", 0) % LAYOUT_COUNT;

  if (!wokeFromSleep) {
    // Nach echtem Neustart die Zaehler aus dem Flash holen
    rtcTotalSec = prefs.getUInt("totalsec", 0);
    rtcStarts = prefs.getUInt("starts", 0);
    rtcRunSeconds = 0;
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!accel.begin()) {

    ensureDisplay();
    digitalWrite(PIN_LCD_BL, HIGH);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("ADXL345 fehlt!", 10, 5);

    Serial.println("ADXL345 nicht gefunden!");

    while (1) delay(10);
  }

  adxlConfigureMeasure();

  bool forceCal = (digitalRead(BTN_2) == LOW) && !wokeFromSleep;

  if (!forceCal && loadCalibration()) {
    calFromFlash = true;
  } else {
    doCalibration();
  }

  batVoltage = readBatteryVolts();
  batPercent = batteryPercent(batVoltage);

  // Nur bei Kaltstart oder Taste. Ein Sensor-Weckruf ist noch keine
  // laufende Pumpe - sonst blitzt das Display bei jeder Erschuetterung auf.
  // Bestaetigt sich die Pumpe, schaltet die Pumpenlogik das Display selbst an.
  if (!wokeFromSleep || wokeByButton) {

    wakeDisplay(wokeByButton ? 'T' : 'B');
    displayUntilMs = millis() + (wokeByButton ? DISPLAY_BUTTON_MS : DISPLAY_BOOT_MS);
  }

  unsigned long now = millis();

  if (!wokeFromSleep) {
    logDumpAt = now + LOG_DUMP_AFTER_MS;
  }

  lastAboveMs = now;
  lastBatMs = now;
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // ---------- Messfenster ----------

  unsigned long windowStart = millis();

  double sumSqHf = 0;
  double peakDev = 0;
  long samples = 0;

  float prev = readMagnitude();

  while (millis() - windowStart < WINDOW_MS) {

    float m = readMagnitude();

    double d = m - prev;
    sumSqHf += d * d;
    prev = m;

    // Groesste Abweichung von der Ruhelage - in derselben Einheit,
    // in der auch die Weckschwelle des Sensors arbeitet
    double dev = fabs(m - baselineMagnitude);
    if (dev > peakDev) {
      peakDev = dev;
    }

    samples++;
  }

  hfVibration = sqrt(sumSqHf / samples);

  // In mg umrechnen: die Weckschwelle zaehlt in Schritten zu 62,5 mg
  int peakMg = (int)(peakDev / 9.80665 * 1000.0);

  unsigned long now = millis();


  // ---------- Serial-Befehle fuers Messlog ----------

  while (Serial.available()) {

    char c = Serial.read();

    if (c == 'l') {
      dumpLog();
    } else if (c == 'x') {
      prefs.putUInt("lgn", 0);
      Serial.println("Log geloescht.");
    }
  }

  if (logDumpAt != 0 && (long)(now - logDumpAt) >= 0) {
    dumpLog();
    logDumpAt = 0;
  }


  // ---------- Tasten ----------

  bool btnDown = (digitalRead(BTN_1) == LOW || digitalRead(BTN_2) == LOW);

  if (btnDown) {

    if (btnDownSince == 0) {

      btnDownSince = now;

      Serial.print("Taste gedrueckt. Display war ");
      Serial.println(displayOn ? "AN" : "AUS");

      if (!displayOn) {
        wakePress = true;
      }
    }

    if (!longHandled && (now - btnDownSince >= LONG_PRESS_MS)) {

      longHandled = true;

      doCalibration();
      drawStaticScreen();

      lastAboveMs = millis();
      displayUntilMs = millis() + DISPLAY_BUTTON_MS;
      aboveSince = 0;
      pumpRunning = false;

      return;
    }

  } else {

    if (btnDownSince != 0 && !longHandled && !wakePress) {

      clickCount++;
      lastClickMs = now;
    }

    btnDownSince = 0;
    longHandled = false;
    wakePress = false;
  }


  // ---------- Klicks auswerten ----------

  if (!btnDown && clickCount > 0 && (now - lastClickMs > CLICK_GAP_MS)) {

    if (clickCount >= 3) {

      // 3x kurz: Layout wechseln
      layout = (layout + 1) % LAYOUT_COUNT;
      prefs.putUChar("layout", layout);
      Serial.print("Layout ");
      Serial.println(layout);

      drawStaticScreen();

    } else {

      // 1-2x kurz: Maximum zuruecksetzen wie bisher
      maxVibration = 0;
      prefs.putFloat("maxvib", 0);
      savedMax = 0;
    }

    clickCount = 0;
  }


  // ---------- Pumpenerkennung ----------

  bool above = (hfVibration >= pumpThreshold);
  bool wasRunning = pumpRunning;

  if (above) {

    lastAboveMs = now;

    if (aboveSince == 0) {
      aboveSince = now;
      burstPeak = 0;
    }

    if (hfVibration > burstPeak) {
      burstPeak = hfVibration;
    }

    if (!pumpRunning && (now - aboveSince >= ON_TIME_MS)) {
      pumpRunning = true;
    }

  } else {

    if (now - lastAboveMs > GAP_MS) {

      // Messung: Ausschlag, der nicht lang genug war (z.B. Klopfen)
      if (aboveSince != 0 && !pumpRunning) {

        unsigned long burstMs = lastAboveMs - aboveSince + WINDOW_MS;

        if (burstMs >= LOG_MIN_BURST_MS) {
          logEvent('B', '-', burstMs, milli(burstPeak), 0, milli(pumpThreshold));
        }

        Serial.print("Burst verworfen: ");
        Serial.print(lastAboveMs - aboveSince + WINDOW_MS);
        Serial.print(" ms, Spitze ");
        Serial.print(burstPeak, 3);
        Serial.print("  (Schwelle ");
        Serial.print(pumpThreshold, 3);
        Serial.print(", noetig ");
        Serial.print(ON_TIME_MS);
        Serial.println(" ms)");
      }

      aboveSince = 0;
    }

    if (pumpRunning && (now - lastAboveMs >= OFF_TIME_MS)) {
      pumpRunning = false;
    }
  }

  if (!wasRunning && pumpRunning) {

    rtcStarts++;

    // aboveSince ist der echte Beginn - nicht erst der Moment,
    // in dem die 500 ms Bestaetigung voll waren
    runStartMs = aboveSince;
    rtcRunSeconds = 0;

    runPeak = burstPeak;
    runSumHf = 0;
    runN = 0;
  }

  // Nur bis zur letzten echten Vibration zaehlen - in den 3 s, bis die
  // Pumpe als aus gilt, bleibt die Anzeige stehen statt weiterzulaufen
  if (pumpRunning) {
    rtcRunSeconds = (lastAboveMs - runStartMs) / 1000;
    timerSec = rtcRunSeconds;
  }

  // Messwerte des Durchgangs (nur Fenster mit echter Vibration)
  if (pumpRunning && above) {
    if (hfVibration > runPeak) {
      runPeak = hfVibration;
    }
    runSumHf += hfVibration;
    runN++;
  }

  if (wasRunning && !pumpRunning) {

    // Die Nachlaufzeit gehoert nicht zur Laufzeit: gemessen wird bis zur
    // letzten echten Vibration, nicht bis zum Ablauf der Abschaltfrist.
    uint32_t finalSec = (lastAboveMs - runStartMs) / 1000;

    rtcRunSeconds = 0;          // Anzeige sofort zuruecksetzen
    rtcTotalSec += finalSec;

    logEvent('P', '-', finalSec, milli(runPeak),
             runN ? milli(runSumHf / runN) : 0, milli(pumpThreshold));

    Serial.print("Durchgang beendet nach ");
    Serial.print(finalSec);
    Serial.println("s");

    sendShot(finalSec, runPeak, runN ? runSumHf / runN : 0);

    // Timer-Layout: Endzeit stehen lassen, dann ausblenden
    timerSec = finalSec;
    if (layout == LAYOUT_TIMER) {
      displayUntilMs = millis() + TIMER_RESULT_MS;
    }
  }


  // ---------- MAX ----------

  if (hfVibration > maxVibration) {
    maxVibration = hfVibration;
  }


  percent = (int)(hfVibration / vibMax * 100.0);
  percent = constrain(percent, 0, 100);


  if (now - lastBatMs >= BAT_INTERVAL_MS) {

    float v = readBatteryVolts();

    // Traege nachfuehren statt jeden Messwert zu uebernehmen. Die Spannung
    // sackt unter Last real ab (Display an = ~40 mA mehr), und auf der
    // steilen LiPo-Kurve werden aus 0,03 V gleich mehrere Prozentpunkte.
    batVoltage = (batVoltage > 0.5) ? (batVoltage * 0.8 + v * 0.2) : v;

    batPercent = batteryPercent(batVoltage);
    lastBatMs = now;
  }


  // Was hat uns geweckt? Staerke in mg, direkt vergleichbar mit der
  // Weckschwelle (aktuell THRESH_ACT_VALUE * 62,5 mg)
  if (logFirstWindow) {

    Serial.print("Ausloeser: ");
    Serial.print(peakMg);
    Serial.print(" mg  (Weckschwelle ");
    Serial.print(THRESH_ACT_VALUE * 62.5, 0);
    Serial.println(" mg)");

    logEvent('W', wokeFromSleep ? (wokeByButton ? 'T' : 'S') : 'C',
             peakMg, 0, 0, milli(pumpThreshold));

    logFirstWindow = false;
  }

  // Nur melden wenn wirklich etwas los ist - sonst geht das Wichtige unter
  if (above || pumpRunning || (wasRunning != pumpRunning)) {

    Serial.print("vib=");
    Serial.print(hfVibration, 4);
    Serial.print("  thr=");
    Serial.print(pumpThreshold, 3);
    Serial.print("  lauf=");
    Serial.print(rtcRunSeconds);
    Serial.print("s  -> ");
    Serial.println(pumpRunning ? "PUMPE AN" : "AUS");
  }


  // ---------- Display an/aus ----------

  // Taste: Display manuell anschalten / Zeit verlaengern
  if (btnDown) {

    displayUntilMs = now + DISPLAY_BUTTON_MS;

    if (!displayOn) {
      wakeDisplay('T');
    }
  }

  // Solange die Pumpe laeuft, wird die Frist staendig nach vorne geschoben.
  // Beim Abschalten laeuft sie dann genau DISPLAY_RESULT_MS spaeter ab -
  // das ist die Zeit, in der du das Ergebnis siehst.
  if (pumpRunning) {

    displayUntilMs = now + DISPLAY_RESULT_MS;

    if (!displayOn) {
      wakeDisplay('P');
    }

    sessionPumpRan = true;
  }

  if (displayOn && (long)(now - displayUntilMs) >= 0) {
    blankDisplay();
  }


  // ---------- Tiefschlaf ----------

  bool bootWindowOver = !wokeFromSleep
                        ? (millis() - bootMs > NO_SLEEP_AFTER_BOOT_MS)
                        : true;

  if (!pumpRunning && !displayOn && bootWindowOver
      && (now - lastAboveMs >= SLEEP_AFTER_MS)) {

    goToDeepSleep();
  }


  if (!displayOn) {
    return;
  }


  // ---------- Zeichnen ----------

  if (layout == LAYOUT_TIMER) {

    int st = pumpRunning ? 1 : 0;

    if (timerSec != lastDrawnTimer || st != lastDrawnState) {

      char buf[8];
      sprintf(buf, "%02u:%02u", (timerSec / 60) % 100, timerSec % 60);

      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(pumpRunning ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
      tft.setTextSize(2);
      tft.setTextPadding(320);
      tft.drawString(buf, 160, 85, 7);     // Font 7 = grosse 7-Segment-Ziffern
      tft.setTextPadding(0);
      tft.setTextDatum(TL_DATUM);

      lastDrawnTimer = timerSec;
      lastDrawnState = st;
    }

    showBacklight();
    return;
  }

  int stateNow = pumpRunning ? 1 : 0;

  if (stateNow != lastDrawnState) {

    tft.setTextSize(3);
    tft.setTextPadding(9 * 18);

    if (pumpRunning) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("PUMPE AN", 10, 35);
    } else {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("PUMPE AUS", 10, 35);
    }

    lastDrawnState = stateNow;
  }

  // Laufzeit des Boards seit dem letzten Start / Aufwachen
  uint32_t uptimeSec = millis() / 1000;

  if (uptimeSec != lastDrawnUptime) {

    tft.setTextSize(2);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setTextPadding(9 * 12);
    tft.drawString("AN " + formatRuntime(uptimeSec), 85, 5);

    lastDrawnUptime = uptimeSec;
  }

  // Mit eingestecktem Akku steigt die Spannung beim Laden oft nicht
  // ueber BAT_USB_VOLT - deshalb zusaetzlich fragen, ob ein Rechner am
  // USB haengt. Ein reines Netzteil erkennt das nicht.
  bool onUsb = usb_serial_jtag_is_connected()
               || batVoltage >= BAT_USB_VOLT || batVoltage < BAT_MIN_VOLT;

  // USB als eigener Wert, sonst wird beim Wechsel 100 % -> USB nicht
  // neu gezeichnet (die Prozentzahl bleibt dabei ja gleich)
  int shownBat = onUsb ? 1000 : batPercent;

  if (shownBat != lastDrawnBat) {

    tft.setTextSize(2);
    tft.setTextPadding(4 * 12);

    if (onUsb) {
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.drawString("USB", 230, 5);
    } else {
      if (batPercent > 50) tft.setTextColor(TFT_GREEN, TFT_BLACK);
      else if (batPercent > 20) tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      else tft.setTextColor(TFT_RED, TFT_BLACK);

      tft.drawString(String(batPercent) + "%", 230, 5);
    }

    lastDrawnBat = shownBat;
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setTextPadding(11 * 12);
  tft.drawString("VIB: " + String(hfVibration, 2), 10, 75);

  if (rtcRunSeconds != lastDrawnRunSec) {

    tft.setTextPadding(11 * 12);
    tft.drawString("ZEIT: " + formatRuntime(rtcRunSeconds), 10, 100);

    lastDrawnRunSec = rtcRunSeconds;
  }

  if (maxVibration != lastDrawnMax) {

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextPadding(10 * 12);
    tft.drawString("MAX: " + String(maxVibration, 2), 170, 100);

    lastDrawnMax = maxVibration;
  }

  tft.setTextPadding(0);

  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextPadding(52 * 6);
  tft.drawString(
    "Starts: " + String(rtcStarts) + "   Gesamt: " + formatRuntime(rtcTotalSec)
      + "   Akku: " + String(batVoltage, 2) + "V",
    10,
    162
  );
  tft.setTextPadding(0);


  // ---------- Balken ----------

  int barWidth = map(percent, 0, 100, 0, BAR_INNER_W);
  barWidth = constrain(barWidth, 0, BAR_INNER_W);

  if (barWidth > 0) {
    tft.fillRect(BAR_INNER_X, BAR_INNER_Y, barWidth, BAR_INNER_H,
                 pumpRunning ? TFT_GREEN : TFT_YELLOW);
  }

  if (barWidth < BAR_INNER_W) {
    tft.fillRect(BAR_INNER_X + barWidth, BAR_INNER_Y,
                 BAR_INNER_W - barWidth, BAR_INNER_H, TFT_BLACK);
  }

  showBacklight();
}
