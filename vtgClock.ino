/* ============================================================
 *  ESP32-C3 Clock — 32x8 WS2812B + LDR + MQTT + HA + Weather
 *  + RTTTL мелодии (устройство + браузер, Web Audio API)
 *  
 *  Пины:
 *    MATRIX_PIN = GPIO3   (WS2812B DIN)
 *    LDR_PIN    = GPIO1   (ADC1_CH1)
 *    I2C_SDA    = GPIO5
 *    I2C_SCL    = GPIO6
 *    BUZZER     = GPI04
 *
 *  В Arduino IDE:
 *    Tools → USB CDC On Boot = "Enabled"
 *    Tools → Flash Size      = "4MB (32Mb)"
 *    Tools → Partition Scheme= "Minimal SPIFFS (1.9MB APP with OTA)"
 * ============================================================ */

#define FIRMWARE_VERSION "4.1.1"

// ---------------- Hardware ----------------
#define MATRIX_PIN      3
#define NUM_LEDS       256
#define MATRIX_WIDTH   32
#define MATRIX_HEIGHT  8
#define LDR_PIN         1
#define LDR_ADC_MIN     200
#define LDR_ADC_MAX     3800
#define LDR_SMOOTHING   0.2f
#define I2C_SDA_PIN     5
#define I2C_SCL_PIN     6
#define BUZZER_PIN      4
#define BUZZER_PWM_RES  10

// ---------------- Defaults ----------------
#define BRIGHTNESS_DEFAULT  15
#define TZ_DEFAULT_MINUTES  180
#define MIN_BRIGHTNESS_HARD 1
#define LDR_INFLUENCE_DEF   100
#define SENSOR_INTERVAL     30000UL
#define WIFI_CHECK_MS       10000UL
#define WIFI_RETRY_MS       30000UL
#define HOLIDAYS_MAX        24
#define TIME_MODE_DURATION    20000UL
#define WEATHER_MODE_DURATION 10000UL
#define BUZZER_VOL_DEFAULT    70

#define NIGHT_START_HOUR_DEF  22
#define NIGHT_END_HOUR_DEF    7
#define NIGHT_BRIGHTNESS_DEF  3
#define MANUAL_BRIGHTNESS_DEF 60

const char* AP_SSID = "vtgClock-AP";
const char* AP_PASS = "12345678";
#define OTA_PASSWORD    "admin"
#define STATE_PUBLISH_INTERVAL  30000UL
#define NTP_SYNC_INTERVAL       3600000UL
#define LOG_BUFFER_SIZE         30

const char* DEVICE_MODEL        = "vtgClockESP32C3";
const char* DEVICE_MANUFACTURER = "vtgHome";

const char* HOL_CAT_RUSSIA = "Праздники России";
const char* HOL_CAT_PROF   = "Профессиональные праздники";
const char* HOL_CAT_INTL   = "Международные праздники";
const char* HOL_CAT_CORP   = "Корпоративные праздники";
const char* HOL_CAT_ECO    = "Экологические праздники";
const char* HOL_CAT_MED    = "Медицинские праздники";
const char* HOL_CAT_INFO   = "Неофициальные праздники";

// ---------------- Buzzer PWM compat (core 2.x / 3.x) ----------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define BZ_WRITE(d) ledcWrite(BUZZER_PIN, (d))
  #define BZ_TONE(f)  ledcWriteTone(BUZZER_PIN, (f))
#else
  #define BZ_WRITE(d) ledcWrite(0, (d))
  #define BZ_TONE(f)  ledcWriteTone(0, (f))
#endif

// ---------------- Includes ----------------
#include <Arduino.h>
#include <WiFi.h>
#include <FastLED.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

extern "C" {
#include "mongoose.h"
}

// ---------------- Globals ----------------
CRGB leds[NUM_LEDS];
Preferences prefs;
struct mg_mgr mgr;
struct mg_connection *mqtt_conn = nullptr;

String deviceName = "vtgClock";

enum BrightMode : uint8_t { BRIGHT_SENSOR = 0, BRIGHT_SCHEDULE = 1, BRIGHT_MANUAL = 2 };

uint8_t brightnessMode = BRIGHT_SENSOR;
uint8_t maxBrightness = BRIGHTNESS_DEFAULT;
uint8_t minBrightness = MIN_BRIGHTNESS_HARD;
uint8_t ldrInfluence = LDR_INFLUENCE_DEF;
uint8_t currentBrightness = BRIGHTNESS_DEFAULT;
uint8_t manualBrightness = MANUAL_BRIGHTNESS_DEF;
uint8_t nightBrightness = NIGHT_BRIGHTNESS_DEF;
uint8_t nightStartHour = NIGHT_START_HOUR_DEF;
uint8_t nightEndHour = NIGHT_END_HOUR_DEF;
bool isNightMode = false;

String ntpServer = "pool.ntp.org";
int timezoneOffsetMinutes = TZ_DEFAULT_MINUTES;
bool wifiConnected = false;
uint8_t buzzerVolume = BUZZER_VOL_DEFAULT;
static uint32_t lastToneFreq = 0;

int ldrRaw = 0;
float ldrFiltered = 0;

bool mqttEnabled = false;
String mqttServer = "192.168.1.100";
int mqttPort = 1883;
String mqttUser, mqttPassword, mqttPrefix = "clock";
bool haDiscovery = true;
String mqttClientId;
String topicState, topicSet, topicAvail;
bool mqttConnected = false;
unsigned long lastMqttAttempt = 0;

bool needPublishState = false;
bool needPublishDiscovery = false;
unsigned long lastStatePublish = 0;

volatile bool otaInProgress = false;
bool rebootPending = false;
unsigned long rebootAt = 0;
unsigned long lastNtpSync = 0;

String logBuffer[LOG_BUFFER_SIZE];
int logIndex = 0, logCount = 0;
int apScrollOffset = MATRIX_WIDTH;
unsigned long apScrollLast = 0;

// ---------------- Melodies ----------------
struct Melody { String name; String rtttl; };
Melody melodies[3];

enum MelodyState { MEL_IDLE, MEL_PLAYING };
MelodyState melodyState = MEL_IDLE;
int melodyPlayingId = -1;
unsigned long noteEndMs = 0;
int melodyParsePos = 0;
int melodyDefaultDur = 4;
int melodyDefaultOctave = 5;
int melodyBPM = 100;

// ---------------- Weather ----------------
uint8_t weatherMode = 0;
String weatherZip = "";
float weatherLat = 0.0f, weatherLon = 0.0f;
uint32_t weatherInterval = 1800;
float wxTempC = NAN;
int wxHumidity = -1;
String wxDesc = "", wxDescRu = "", wxCity = "", wxError = "";
volatile unsigned long wxLastFetch = 0;
bool wxValid = false;
int weatherCode = 0;

uint16_t weatherAnimPhase = 0;
bool showWeatherMode = false;
unsigned long displayModeStartAt = 0;
enum WeatherIconType { WI_UNKNOWN, WI_SUN, WI_PARTLY, WI_CLOUD, WI_FOG, WI_RAIN, WI_SNOW, WI_THUNDER };

// ---------------- Holidays ----------------
bool holidayCatRussia = true, holidayCatProf = true, holidayCatIntl = true;
bool holidayCatCorp = true, holidayCatEco = true, holidayCatMed = true, holidayCatInfo = true;
String todayHolidays[HOLIDAYS_MAX];
String todayHolidaysCats[HOLIDAYS_MAX];
int todayHolidaysCount = 0;
String todayHolidaysDate = "";
String holidayError = "";
bool holidayValid = false;
bool needRecalcHolidays = false;

// ---------------- Sensor ----------------
enum SensorType : uint8_t { SENSOR_NONE = 0, SENSOR_BME280, SENSOR_BMP280, SENSOR_AHT20_BMP280 };
uint8_t sensorType = SENSOR_NONE;
bool sensorOk = false;
float sensorTemp = NAN, sensorHum = NAN, sensorPress = NAN;
unsigned long sensorLastRead = 0;
Adafruit_BME280* bme280Sensor = nullptr;
Adafruit_BMP280* bmp280Sensor = nullptr;
Adafruit_AHTX0* aht20Sensor = nullptr;
Adafruit_BMP280* bmp280Combo = nullptr;

// ---------------- Holiday databases ----------------
struct FixedHoliday { uint8_t month; uint8_t day; const char* name; const char* cat; };
struct EasterHoliday { int16_t offset; const char* name; const char* cat; uint8_t orthodox; };
struct NthWeekdayHoliday { uint8_t month; uint8_t weekday; int8_t nth; const char* name; const char* cat; };

static const FixedHoliday FIXED_HOLIDAYS[] = {
  {1,1,"Новый год","Праздники России"},{1,2,"Новогодние каникулы","Праздники России"},
  {1,3,"Новогодние каникулы","Праздники России"},{1,4,"Новогодние каникулы","Праздники России"},
  {1,5,"Новогодние каникулы","Праздники России"},{1,6,"Новогодние каникулы","Праздники России"},
  {1,7,"Рождество Христово","Праздники России"},{1,8,"Новогодние каникулы","Праздники России"},
  {2,23,"День защитника Отечества","Праздники России"},
  {3,8,"Международный женский день","Праздники России"},
  {5,1,"Праздник Весны и Труда","Праздники России"},
  {5,9,"День Победы","Праздники России"},
  {6,12,"День России","Праздники России"},
  {11,4,"День народного единства","Праздники России"},
  {1,1,"Новый год","Международные праздники"},
  {3,8,"Женский день","Международные праздники"},
  {5,1,"День труда","Международные праздники"},
  {6,1,"День защиты детей","Международные праздники"},
  {9,21,"День мира","Международные праздники"},
  {10,1,"День пожилых людей","Международные праздники"},
  {10,24,"День ООН","Международные праздники"},
  {11,16,"День толерантности","Международные праздники"},
  {12,10,"День прав человека","Международные праздники"},
  {12,25,"Рождество (католическое)","Международные праздники"},
  {4,12,"День космонавтики","Профессиональные праздники"},
  {5,7,"День радио","Профессиональные праздники"},
  {5,27,"День библиотек","Профессиональные праздники"},
  {6,5,"День эколога","Профессиональные праздники"},
  {6,8,"День социального работника","Профессиональные праздники"},
  {7,20,"День шахмат","Профессиональные праздники"},
  {10,5,"День учителя","Профессиональные праздники"},
  {11,20,"День педиатра","Профессиональные праздники"},
  {12,3,"День юриста","Профессиональные праздники"},
  {11,12,"День работников Сбербанка","Корпоративные праздники"},
  {1,11,"День заповедников","Экологические праздники"},
  {1,15,"День зимующих птиц","Экологические праздники"},
  {3,3,"День дикой природы","Экологические праздники"},
  {4,7,"День бобра","Экологические праздники"},
  {4,15,"День экологических знаний","Экологические праздники"},
  {5,12,"День экологического образования","Экологические праздники"},
  {6,5,"День окружающей среды","Экологические праздники"},
  {9,9,"День снегиря","Экологические праздники"},
  {11,12,"Синичкин день","Экологические праздники"},
  {1,13,"День борьбы с депрессией","Медицинские праздники"},
  {1,19,"День патологоанатома","Медицинские праздники"},
  {2,9,"День стоматолога","Медицинские праздники"},
  {2,21,"День фельдшера","Медицинские праздники"},
  {3,6,"День зубного врача","Медицинские праздники"},
  {3,24,"День борьбы с туберкулезом","Медицинские праздники"},
  {4,20,"День донора крови","Медицинские праздники"},
  {5,12,"День медицинской сестры","Медицинские праздники"},
  {6,21,"День медицинского работника","Медицинские праздники"},
  {11,12,"День борьбы с пневмонией","Медицинские праздники"},
  {1,11,"День спасибо","Неофициальные праздники"},
  {1,21,"День объятий","Неофициальные праздники"},
  {2,4,"Праздник хорошего настроения","Неофициальные праздники"},
  {3,1,"День комплиментов","Неофициальные праздники"},
  {4,1,"День смеха","Неофициальные праздники"},
  {5,3,"День солнца","Неофициальные праздники"},
  {6,2,"День лета","Неофициальные праздники"},
  {7,7,"День роз и шоколада","Неофициальные праздники"},
  {8,19,"День тельняшки","Неофициальные праздники"},
  {9,11,"День трезвости","Неофициальные праздники"},
  {9,25,"День дочери","Неофициальные праздники"},
  {11,12,"День специалиста по безопасности","Неофициальные праздники"},
};

static const EasterHoliday EASTER_HOLIDAYS[] = {
  {-55,"Масленица (начало)","Праздники России",1},
  {-49,"Прощёное воскресенье","Праздники России",1},
  {-7,"Вербное воскресенье","Праздники России",1},
  {0,"Пасха","Праздники России",1},
  {9,"Радоница","Праздники России",1},
  {39,"Вознесение Господне","Праздники России",1},
  {49,"Троица","Праздники России",1},
  {0,"Пасха (католическая)","Международные праздники",0},
};

static const NthWeekdayHoliday NTH_WEEKDAY_HOLIDAYS[] = {
  {5,0,2,"День матери","Международные праздники"},
  {6,0,3,"День отца","Международные праздники"},
  {7,5,-1,"День системного администратора","Профессиональные праздники"},
  {11,0,-1,"День матери в России","Профессиональные праздники"},
  {11,4,2,"Всемирный день качества","Неофициальные праздники"},
  {11,4,2,"Всемирный день юзабилити","Неофициальные праздники"},
};

// ---------------- Font ----------------
static const uint8_t FONT_3x5[][5] PROGMEM = {
  {0b111,0b101,0b101,0b101,0b111},{0b010,0b110,0b010,0b010,0b111},
  {0b111,0b001,0b111,0b100,0b111},{0b111,0b001,0b111,0b001,0b111},
  {0b101,0b101,0b111,0b001,0b001},{0b111,0b100,0b111,0b001,0b111},
  {0b111,0b100,0b111,0b101,0b111},{0b111,0b001,0b001,0b001,0b001},
  {0b111,0b101,0b111,0b101,0b111},{0b111,0b101,0b111,0b001,0b111},
  {0b111,0b101,0b111,0b101,0b101},{0b111,0b101,0b110,0b101,0b111},
  {0b111,0b100,0b100,0b100,0b111},{0b110,0b101,0b101,0b101,0b110},
  {0b111,0b100,0b111,0b100,0b111},{0b111,0b100,0b111,0b100,0b100},
  {0b111,0b100,0b101,0b101,0b111},{0b101,0b101,0b111,0b101,0b101},
  {0b111,0b010,0b010,0b010,0b111},{0b001,0b001,0b001,0b101,0b111},
  {0b101,0b101,0b110,0b101,0b101},{0b100,0b100,0b100,0b100,0b111},
  {0b101,0b111,0b111,0b101,0b101},{0b101,0b111,0b111,0b111,0b101},
  {0b111,0b101,0b101,0b101,0b111},{0b111,0b101,0b111,0b100,0b100},
  {0b111,0b101,0b101,0b111,0b001},{0b111,0b101,0b111,0b110,0b101},
  {0b111,0b100,0b111,0b001,0b111},{0b111,0b010,0b010,0b010,0b010},
  {0b101,0b101,0b101,0b101,0b111},{0b101,0b101,0b101,0b101,0b010},
  {0b101,0b101,0b111,0b111,0b101},{0b101,0b101,0b010,0b101,0b101},
  {0b101,0b101,0b010,0b010,0b010},{0b111,0b001,0b010,0b100,0b111},
  {0b000,0b000,0b000,0b000,0b010},{0b000,0b010,0b000,0b010,0b000},
  {0b000,0b000,0b111,0b000,0b000},{0b000,0b000,0b000,0b000,0b000},
};

static int charIndex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'Z') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'z') return 10 + (c - 'a');
  if (c == '.') return 36; if (c == ':') return 37; if (c == '-') return 38;
  return 39;
}

static void drawChar(char c, int x, int y, CRGB color) {
  int idx = charIndex(c);
  for (int row = 0; row < 5; row++) {
    uint8_t bits = pgm_read_byte(&FONT_3x5[idx][row]);
    for (int col = 0; col < 3; col++)
      if (bits & (0b100 >> col)) {
        int px = x + col, py = y + row;
        if (px >= 0 && px < MATRIX_WIDTH && py >= 0 && py < MATRIX_HEIGHT)
          leds[py * MATRIX_WIDTH + px] = color;
      }
  }
}

// ============================================================
// Prototypes
// ============================================================
static void sendJson(struct mg_connection *c, const String& json);
static String getQueryParam(const char *query, const char *key);
static String getPostParam(struct mg_http_message *hm, const char *key);
static String urlDecode(const String& s);
static String jsonEscape(const String& s);
static void handleApiStatus(struct mg_connection *c);
static void handleApiTime(struct mg_connection *c);
static void handleApiWeather(struct mg_connection *c);
static void handleApiLogs(struct mg_connection *c);
static void handleApiMelodies(struct mg_connection *c);
static WeatherIconType getWeatherIconType(int code);
static const char* getWeatherIconName(int code);
static const char* getWeatherColor(int code);
static const char* getWeatherDescRu(int code);
void connectMQTT();
void disconnectMQTT();
bool mqttPublishJson(const String& topic, const String& json);
void addLog(const String& msg);
String getLocalTimeString();
void processCommand(const String& cmd);
bool applyHaJsonCommand(const String& msg);
void publishState();
void publishDiscovery();
void calculateHolidays();
void fetchWeather();
void readSensor();
void readLDR();
void applyBrightness();
void updateClockDisplay();
void updateApModeDisplay();
void updateDisplay();
void drawWeatherAnimation();
void checkWiFiConnection();
void connectToWiFi();
void startAPMode();
void syncTime();
void setupOTA();
void initSensor();
void loadSettings();
void saveSettings();
void loadMelodies();
void saveMelodies();
void stopMelody();
void playNextNote();
void startMelody(int id);
void melodyTick();
bool checkNightTime();

// ============================================================
// Utilities
// ============================================================
static void sendJson(struct mg_connection *c, const String& json) {
  mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", json.c_str());
}
static String getQueryParam(const char *query, const char *key) {
  if (!query) return "";
  String q = String(query); String search = String(key) + "=";
  int start = q.indexOf(search); if (start < 0) return "";
  start += search.length(); int end = q.indexOf('&', start);
  if (end < 0) end = q.length();
  return q.substring(start, end);
}
static String urlDecode(const String& s) {
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '+') out += ' ';
    else if (c == '%' && i + 2 < s.length()) {
      char hex[3] = {s[i+1], s[i+2], 0}; out += (char)strtol(hex, NULL, 16); i += 2;
    } else out += c;
  }
  return out;
}
static String getPostParam(struct mg_http_message *hm, const char *key) {
  String body = String(hm->body.buf, hm->body.len);
  String search = String(key) + "=";
  int start = 0;
  while (start < (int)body.length()) {
    int next = body.indexOf('&', start);
    if (next < 0) next = body.length();
    String pair = body.substring(start, next);
    if (pair.startsWith(search)) return urlDecode(pair.substring(search.length()));
    start = next + 1;
  }
  return "";
}
static String jsonEscape(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"': o += "\\\""; break; case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break; case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
        else o += c;
    }
  }
  return o;
}
void addLog(const String& msg) {
  String line = "[" + getLocalTimeString() + "] " + msg;
  logBuffer[logIndex] = line;
  logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
  if (logCount < LOG_BUFFER_SIZE) logCount++;
  Serial.println(line);
}
String getLocalTimeString() {
  time_t now = time(nullptr);
  if (now < 100000) return "??:??:??";
  time_t local = now + timezoneOffsetMinutes * 60;
  struct tm tm; localtime_r(&local, &tm);
  char buf[32]; strftime(buf, sizeof(buf), "%d.%m.%Y %H:%M:%S", &tm);
  return String(buf);
}

bool checkNightTime() {
  struct tm ti;
  if (!getLocalTime(&ti, 10)) return false;
  int h = ti.tm_hour;
  if (nightStartHour == nightEndHour) return false;
  if (nightStartHour < nightEndHour) return (h >= nightStartHour && h < nightEndHour);
  return (h >= nightStartHour || h < nightEndHour);
}

// ============================================================
// Holiday helpers
// ============================================================
static bool isLeapYear(int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }
static void addDaysToDate(int& year, int& month, int& day, int offset) {
  static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  auto daysInMonth = [](int y, int m) { return dim[m - 1] + ((m == 2 && isLeapYear(y)) ? 1 : 0); };
  day += offset;
  while (day > daysInMonth(year, month)) { day -= daysInMonth(year, month); month++; if (month > 12) { month = 1; year++; } }
  while (day < 1) { month--; if (month < 1) { month = 12; year--; } day += daysInMonth(year, month); }
}
static void getEasterDate(int year, bool orthodox, int& month, int& day) {
  if (orthodox) {
    int a = year % 4, b = year % 7, c = year % 19;
    int d = (19 * c + 15) % 30, e = (2 * a + 4 * b - d + 34) % 7;
    int mJ = (d + e + 114) / 31, dJ = ((d + e + 114) % 31) + 1;
    int y2 = year, m2 = mJ, d2 = dJ;
    addDaysToDate(y2, m2, d2, 13); month = m2; day = d2;
  } else {
    int a = year % 19, b = year / 100, c = year % 100;
    int d = b / 4, e = b % 4, f = (b + 8) / 25, g = (b - f + 1) / 3;
    int h = (19 * a + b - d - g + 15) % 30;
    int i = c / 4, k = c % 4, l = (32 + 2 * e + 2 * i - h - k) % 7;
    int m = (a + 11 * h + 22 * l) / 451;
    month = (h + l - 7 * m + 114) / 31;
    day = ((h + l - 7 * m + 114) % 31) + 1;
  }
}
static int getNthWeekdayOfMonth(int year, int month, int weekday, int nth) {
  struct tm t = {0};
  t.tm_year = year - 1900; t.tm_mon = month - 1; t.tm_mday = 1; t.tm_hour = 12; t.tm_isdst = 0;
  mktime(&t);
  int firstWday = t.tm_wday;
  int offset = (weekday - firstWday + 7) % 7;
  if (nth > 0) return 1 + offset + (nth - 1) * 7;
  static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int dim2 = dim[month - 1] + ((month == 2 && isLeapYear(year)) ? 1 : 0);
  t.tm_mday = dim2; mktime(&t);
  int lastWday = t.tm_wday;
  return dim2 - (lastWday - weekday + 7) % 7;
}
static bool isCategoryEnabled(const char* cat) {
  if (strcmp(cat, HOL_CAT_RUSSIA) == 0) return holidayCatRussia;
  if (strcmp(cat, HOL_CAT_PROF) == 0) return holidayCatProf;
  if (strcmp(cat, HOL_CAT_INTL) == 0) return holidayCatIntl;
  if (strcmp(cat, HOL_CAT_CORP) == 0) return holidayCatCorp;
  if (strcmp(cat, HOL_CAT_ECO) == 0) return holidayCatEco;
  if (strcmp(cat, HOL_CAT_MED) == 0) return holidayCatMed;
  if (strcmp(cat, HOL_CAT_INFO) == 0) return holidayCatInfo;
  return false;
}
void calculateHolidays() {
  todayHolidaysCount = 0; holidayValid = false; holidayError = "";
  struct tm now_tm;
  if (!getLocalTime(&now_tm, 100)) { holidayError = "no time"; return; }
  int year = now_tm.tm_year + 1900, month = now_tm.tm_mon + 1, day = now_tm.tm_mday;
  char buf[11]; snprintf(buf, sizeof(buf), "%04d-%02d-%02d", year, month, day);
  todayHolidaysDate = String(buf);

  auto addHoliday = [](const char* name, const char* cat) {
    if (todayHolidaysCount >= HOLIDAYS_MAX) return;
    for (int i = 0; i < todayHolidaysCount; i++) if (todayHolidays[i] == name) return;
    todayHolidays[todayHolidaysCount] = name;
    todayHolidaysCats[todayHolidaysCount] = cat;
    todayHolidaysCount++;
  };

  for (size_t i = 0; i < sizeof(FIXED_HOLIDAYS) / sizeof(FIXED_HOLIDAYS[0]); i++) {
    const FixedHoliday& h = FIXED_HOLIDAYS[i];
    if (h.month != month || h.day != day) continue;
    if (!isCategoryEnabled(h.cat)) continue;
    addHoliday(h.name, h.cat);
  }
  int oM, oD, cM, cD;
  getEasterDate(year, true, oM, oD); getEasterDate(year, false, cM, cD);
  for (size_t i = 0; i < sizeof(EASTER_HOLIDAYS) / sizeof(EASTER_HOLIDAYS[0]); i++) {
    const EasterHoliday& h = EASTER_HOLIDAYS[i];
    if (!isCategoryEnabled(h.cat)) continue;
    int y2 = year, m2 = h.orthodox ? oM : cM, d2 = h.orthodox ? oD : cD;
    addDaysToDate(y2, m2, d2, h.offset);
    if (y2 == year && m2 == month && d2 == day) addHoliday(h.name, h.cat);
  }
  for (size_t i = 0; i < sizeof(NTH_WEEKDAY_HOLIDAYS) / sizeof(NTH_WEEKDAY_HOLIDAYS[0]); i++) {
    const NthWeekdayHoliday& h = NTH_WEEKDAY_HOLIDAYS[i];
    if (h.month != month) continue;
    if (!isCategoryEnabled(h.cat)) continue;
    if (getNthWeekdayOfMonth(year, month, h.weekday, h.nth) == day) addHoliday(h.name, h.cat);
  }
  if (isCategoryEnabled(HOL_CAT_PROF)) {
    int progDay = 256 - (243 + (isLeapYear(year) ? 1 : 0));
    if (month == 9 && day == progDay) addHoliday("День программиста", HOL_CAT_PROF);
  }
  holidayValid = (todayHolidaysCount > 0);
  holidayError = holidayValid ? "" : "no holidays today";
  needPublishState = true;
}

// ============================================================
// Melodies
// ============================================================
void loadMelodies() {
  prefs.begin("clock", true);
  for (int i = 0; i < 3; i++) {
    melodies[i].name = prefs.getString(("mel" + String(i) + "name").c_str(), "");
    melodies[i].rtttl = prefs.getString(("mel" + String(i) + "rtttl").c_str(), "");
  }
  prefs.end();
}
void saveMelodies() {
  prefs.begin("clock", false);
  for (int i = 0; i < 3; i++) {
    prefs.putString(("mel" + String(i) + "name").c_str(), melodies[i].name);
    prefs.putString(("mel" + String(i) + "rtttl").c_str(), melodies[i].rtttl);
  }
  prefs.end();
}
void stopMelody() {
  ledcWrite(BUZZER_PIN, 0);
  lastToneFreq = 0;
  melodyState = MEL_IDLE;
  melodyPlayingId = -1;
}

void playNextNote() {
  if (melodyPlayingId < 0 || melodyPlayingId >= 3) { stopMelody(); return; }
  String& rtttl = melodies[melodyPlayingId].rtttl;
  if (melodyParsePos >= (int)rtttl.length()) { stopMelody(); return; }

  int endPos = rtttl.indexOf(',', melodyParsePos);
  if (endPos < 0) endPos = rtttl.length();
  String note = rtttl.substring(melodyParsePos, endPos);
  melodyParsePos = endPos + 1;
  note.trim(); note.toLowerCase();

  if (note.length() == 0) {
    if (melodyParsePos < (int)rtttl.length()) playNextNote(); else stopMelody();
    return;
  }

  int idx = 0;
  int dur = 0;
  while (idx < (int)note.length() && isDigit(note[idx])) { dur = dur * 10 + (note[idx] - '0'); idx++; }
  bool durDotted = false;
  if (idx < (int)note.length() && note[idx] == '.') { durDotted = true; idx++; }
  if (dur == 0) dur = melodyDefaultDur;
  if (idx >= (int)note.length()) { if (melodyParsePos < (int)rtttl.length()) playNextNote(); else stopMelody(); return; }

  char c = note[idx]; int noteNum = -1; bool isPause = false;
  if (c == 'p') { isPause = true; idx++; }
  else if (c >= 'a' && c <= 'g') {
    switch (c) {
      case 'c': noteNum = 0; break;  case 'd': noteNum = 2; break;
      case 'e': noteNum = 4; break;  case 'f': noteNum = 5; break;
      case 'g': noteNum = 7; break;  case 'a': noteNum = 9; break;
      case 'b': noteNum = 11; break;
    }
    idx++;
  }
  if (!isPause && noteNum < 0) { stopMelody(); return; }

  bool sharp = false;
  if (idx < (int)note.length() && note[idx] == '#') { sharp = true; idx++; }
  if (sharp && noteNum >= 0) noteNum++;

  int octave = melodyDefaultOctave;
  if (idx < (int)note.length() && note[idx] >= '0' && note[idx] <= '9') { octave = note[idx] - '0'; idx++; }

  // Пунктир после октавы (стандарт RTTTL: "c5.")
  bool noteDotted = false;
  if (idx < (int)note.length() && note[idx] == '.') { noteDotted = true; idx++; }

  bool dotted = durDotted || noteDotted;
  float freq = 0;
  if (!isPause) {
    if (octave < 2) octave = 2;
    if (octave > 7) octave = 7;
    freq = 440.0f * powf(2.0f, (float)(octave - 4) + (float)(noteNum - 9) / 12.0f);
  }

  float ms = (4.0f / (float)dur) * (60000.0f / (float)melodyBPM);
  if (dotted) ms *= 1.5f;

  if (isPause || buzzerVolume == 0 || freq < 20.0f || freq > 20000.0f) {
    BZ_WRITE(0);
  } else {
    BZ_TONE((uint32_t)freq);
    uint32_t maxDuty = (1u << BUZZER_PWM_RES) - 1u;
    uint32_t duty = (maxDuty / 2u);
    BZ_WRITE(duty);
  }

  noteEndMs = millis() + (unsigned long)ms;
}

void startMelody(int id) {
  if (id < 0 || id >= 3) return;
  String& rtttl = melodies[id].rtttl;
  if (rtttl.length() == 0) { addLog("Melody empty"); return; }
  int c1 = rtttl.indexOf(':'); if (c1 < 0) { addLog("Melody: no ':'"); return; }
  int c2 = rtttl.indexOf(':', c1 + 1); if (c2 < 0) { addLog("Melody: no header end"); return; }
  String header = rtttl.substring(c1 + 1, c2);
  melodyParsePos = c2 + 1;
  melodyDefaultDur = 4; melodyDefaultOctave = 5; melodyBPM = 100;
  int pos = 0;
  while (pos < (int)header.length()) {
    int next = header.indexOf(',', pos); if (next < 0) next = header.length();
    String part = header.substring(pos, next); part.trim(); part.toLowerCase();
    if (part.startsWith("d=")) melodyDefaultDur = part.substring(2).toInt();
    else if (part.startsWith("o=")) melodyDefaultOctave = part.substring(2).toInt();
    else if (part.startsWith("b=")) melodyBPM = part.substring(2).toInt();
    pos = next + 1;
  }
  if (melodyDefaultDur < 1) melodyDefaultDur = 4;
  if (melodyBPM < 1) melodyBPM = 100;
  stopMelody();
  melodyState = MEL_PLAYING; melodyPlayingId = id;
  addLog("Melody: " + melodies[id].name);
  playNextNote();
}
void melodyTick() {
  if (melodyState == MEL_PLAYING && (long)(millis() - noteEndMs) >= 0) playNextNote();
}

// ============================================================
// NVS
// ============================================================
void loadSettings() {
  prefs.begin("clock", true);
  deviceName = prefs.getString("devName", "ESP32-Clock");
  brightnessMode = prefs.getUChar("brMode", BRIGHT_SENSOR);
  maxBrightness = prefs.getUChar("bright", BRIGHTNESS_DEFAULT);
  minBrightness = prefs.getUChar("minBright", MIN_BRIGHTNESS_HARD);
  ldrInfluence = prefs.getUChar("ldrInfl", LDR_INFLUENCE_DEF);
  manualBrightness = prefs.getUChar("manualB", MANUAL_BRIGHTNESS_DEF);
  nightBrightness = prefs.getUChar("nightB", NIGHT_BRIGHTNESS_DEF);
  nightStartHour = prefs.getUChar("nStart", NIGHT_START_HOUR_DEF);
  nightEndHour = prefs.getUChar("nEnd", NIGHT_END_HOUR_DEF);
  ntpServer = prefs.getString("ntp", "pool.ntp.org");
  timezoneOffsetMinutes = prefs.getInt("tz_min", TZ_DEFAULT_MINUTES);
  mqttEnabled = prefs.getBool("mqtt_en", false);
  mqttServer = prefs.getString("mqtt_srv", "192.168.1.100");
  mqttPort = prefs.getInt("mqtt_port", 1883);
  mqttUser = prefs.getString("mqtt_user", "");
  mqttPassword = prefs.getString("mqtt_pass", "");
  mqttPrefix = prefs.getString("mqtt_pfx", "clock");
  haDiscovery = prefs.getBool("ha_disc", true);
  weatherMode = prefs.getUChar("wx_mode", 0);
  weatherZip = prefs.getString("wx_zip", "");
  weatherLat = prefs.getFloat("wx_lat", 0.0f);
  weatherLon = prefs.getFloat("wx_lon", 0.0f);
  weatherInterval = prefs.getUInt("wx_int", 600);
  sensorType = prefs.getUChar("sensor_t", SENSOR_NONE);
  buzzerVolume = prefs.getUChar("buzzVol", BUZZER_VOL_DEFAULT);
  holidayCatRussia = prefs.getBool("hol_cat_ru", true);
  holidayCatProf = prefs.getBool("hol_cat_prof", true);
  holidayCatIntl = prefs.getBool("hol_cat_intl", true);
  holidayCatCorp = prefs.getBool("hol_cat_corp", true);
  holidayCatEco = prefs.getBool("hol_cat_eco", true);
  holidayCatMed = prefs.getBool("hol_cat_med", true);
  holidayCatInfo = prefs.getBool("hol_cat_info", true);
  prefs.end();
  if (brightnessMode > BRIGHT_MANUAL) brightnessMode = BRIGHT_SENSOR;
  if (maxBrightness > 255) maxBrightness = 255;
  if (maxBrightness < MIN_BRIGHTNESS_HARD) maxBrightness = MIN_BRIGHTNESS_HARD;
  if (minBrightness < MIN_BRIGHTNESS_HARD) minBrightness = MIN_BRIGHTNESS_HARD;
  if (minBrightness > maxBrightness) minBrightness = maxBrightness;
  if (manualBrightness < MIN_BRIGHTNESS_HARD) manualBrightness = MIN_BRIGHTNESS_HARD;
  if (manualBrightness > maxBrightness) manualBrightness = maxBrightness;
  if (nightBrightness < MIN_BRIGHTNESS_HARD) nightBrightness = MIN_BRIGHTNESS_HARD;
  if (nightBrightness > maxBrightness) nightBrightness = maxBrightness;
  if (nightStartHour > 23) nightStartHour = 23;
  if (nightEndHour > 23) nightEndHour = 23;
  if (ldrInfluence > 100) ldrInfluence = 100;
  if (sensorType > SENSOR_AHT20_BMP280) sensorType = SENSOR_NONE;
  if (buzzerVolume > 100) buzzerVolume = 100;
  FastLED.setBrightness(maxBrightness);
}
void saveSettings() {
  prefs.begin("clock", false);
  prefs.putString("devName", deviceName);
  prefs.putUChar("brMode", brightnessMode);
  prefs.putUChar("bright", maxBrightness);
  prefs.putUChar("minBright", minBrightness);
  prefs.putUChar("ldrInfl", ldrInfluence);
  prefs.putUChar("manualB", manualBrightness);
  prefs.putUChar("nightB", nightBrightness);
  prefs.putUChar("nStart", nightStartHour);
  prefs.putUChar("nEnd", nightEndHour);
  prefs.putString("ntp", ntpServer);
  prefs.putInt("tz_min", timezoneOffsetMinutes);
  prefs.putBool("mqtt_en", mqttEnabled);
  prefs.putString("mqtt_srv", mqttServer);
  prefs.putInt("mqtt_port", mqttPort);
  prefs.putString("mqtt_user", mqttUser);
  prefs.putString("mqtt_pass", mqttPassword);
  prefs.putString("mqtt_pfx", mqttPrefix);
  prefs.putBool("ha_disc", haDiscovery);
  prefs.putUChar("wx_mode", weatherMode);
  prefs.putString("wx_zip", weatherZip);
  prefs.putFloat("wx_lat", weatherLat);
  prefs.putFloat("wx_lon", weatherLon);
  prefs.putUInt("wx_int", weatherInterval);
  prefs.putUChar("sensor_t", sensorType);
  prefs.putUChar("buzzVol", buzzerVolume);
  prefs.putBool("hol_cat_ru", holidayCatRussia);
  prefs.putBool("hol_cat_prof", holidayCatProf);
  prefs.putBool("hol_cat_intl", holidayCatIntl);
  prefs.putBool("hol_cat_corp", holidayCatCorp);
  prefs.putBool("hol_cat_eco", holidayCatEco);
  prefs.putBool("hol_cat_med", holidayCatMed);
  prefs.putBool("hol_cat_info", holidayCatInfo);
  prefs.end();
}

// ============================================================
// WiFi / NTP
// ============================================================
void connectToWiFi() {
  WiFi.setHostname(deviceName.c_str());
  WiFi.mode(WIFI_STA); WiFi.begin();
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); Serial.print("."); attempts++; }
  Serial.println();
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected) addLog("WiFi OK: " + WiFi.localIP().toString());
  else addLog("WiFi FAILED");
}
void startAPMode() {
  WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASS);
  addLog(String("AP mode: SSID=") + AP_SSID);
  apScrollOffset = MATRIX_WIDTH;
}
void checkWiFiConnection() {
  static unsigned long lastCheck = 0, lastRetry = 0;
  unsigned long now = millis();
  if (now - lastCheck < WIFI_CHECK_MS) return;
  lastCheck = now;
  if (WiFi.getMode() == WIFI_MODE_AP) return;
  bool nowConnected = (WiFi.status() == WL_CONNECTED);
  if (nowConnected && !wifiConnected) {
    wifiConnected = true; needPublishState = true;
    lastNtpSync = 0; wxLastFetch = 0; needRecalcHolidays = true;
    addLog("WiFi reconnected");
  } else if (!nowConnected && wifiConnected) { wifiConnected = false; addLog("WiFi lost"); }
  if (!nowConnected && (now - lastRetry > WIFI_RETRY_MS)) {
    lastRetry = now; WiFi.disconnect(false, false); WiFi.begin();
  }
}
void syncTime() {
  configTime(0, 0, ntpServer.c_str(), "time.nist.gov");
  lastNtpSync = millis(); addLog("NTP sync: " + ntpServer);
}

// ============================================================
// Sensor
// ============================================================
static const char* sensorTypeName(uint8_t t) {
  switch (t) {
    case SENSOR_BME280: return "BME280";
    case SENSOR_BMP280: return "BMP280";
    case SENSOR_AHT20_BMP280: return "AHT20+BMP280";
    default: return "None";
  }
}
void initSensor() {
  if (sensorType == SENSOR_NONE) return;
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (sensorType == SENSOR_BME280) {
    bme280Sensor = new Adafruit_BME280();
    sensorOk = bme280Sensor->begin(0x76) || bme280Sensor->begin(0x77);
  } else if (sensorType == SENSOR_BMP280) {
    bmp280Sensor = new Adafruit_BMP280();
    sensorOk = bmp280Sensor->begin(0x76) || bmp280Sensor->begin(0x77);
  } else if (sensorType == SENSOR_AHT20_BMP280) {
    aht20Sensor = new Adafruit_AHTX0(); bmp280Combo = new Adafruit_BMP280();
    sensorOk = aht20Sensor->begin() && (bmp280Combo->begin(0x76) || bmp280Combo->begin(0x77));
  }
  addLog(String("Sensor: ") + sensorTypeName(sensorType) + (sensorOk ? " OK" : " FAIL"));
}
void readSensor() {
  if (!sensorOk || sensorType == SENSOR_NONE) return;
  unsigned long now = millis();
  if (sensorLastRead != 0 && (now - sensorLastRead) < SENSOR_INTERVAL) return;
  sensorLastRead = now;
  if (sensorType == SENSOR_BME280 && bme280Sensor) {
    sensorTemp = bme280Sensor->readTemperature();
    sensorHum = bme280Sensor->readHumidity();
    sensorPress = bme280Sensor->readPressure() / 100.0f;
  } else if (sensorType == SENSOR_BMP280 && bmp280Sensor) {
    sensorTemp = bmp280Sensor->readTemperature(); sensorHum = NAN;
    sensorPress = bmp280Sensor->readPressure() / 100.0f;
  } else if (sensorType == SENSOR_AHT20_BMP280 && aht20Sensor && bmp280Combo) {
    sensors_event_t h, t; aht20Sensor->getEvent(&h, &t);
    sensorTemp = t.temperature; sensorHum = h.relative_humidity;
    sensorPress = bmp280Combo->readPressure() / 100.0f;
  }
  if (!isnan(sensorTemp)) needPublishState = true;
}

// ============================================================
// Brightness
// ============================================================
void readLDR() {
  ldrRaw = analogRead(LDR_PIN);
  ldrFiltered = ldrFiltered * (1.0f - LDR_SMOOTHING) + ldrRaw * LDR_SMOOTHING;
}
void applyBrightness() {
  isNightMode = checkNightTime();
  switch (brightnessMode) {
    case BRIGHT_MANUAL:
      currentBrightness = (uint8_t)constrain((int)manualBrightness, (int)MIN_BRIGHTNESS_HARD, (int)maxBrightness);
      break;
    case BRIGHT_SCHEDULE:
      if (isNightMode) {
        currentBrightness = (uint8_t)constrain((int)nightBrightness, (int)MIN_BRIGHTNESS_HARD, (int)maxBrightness);
      } else {
        int span = (int)maxBrightness - (int)minBrightness;
        int v = constrain((int)ldrFiltered, LDR_ADC_MIN, LDR_ADC_MAX);
        int raw = map(v, LDR_ADC_MIN, LDR_ADC_MAX, 0, span);
        if (raw < 0) raw = 0; if (raw > span) raw = span;
        float ratio = powf((float)raw / (float)span, 1.4f);
        int bLDR = (int)minBrightness + (int)(ratio * span + 0.5f);
        int bFinal = (int)maxBrightness - ((int)maxBrightness - bLDR) * (int)ldrInfluence / 100;
        currentBrightness = (uint8_t)constrain(bFinal, (int)minBrightness, (int)maxBrightness);
      }
      break;
    case BRIGHT_SENSOR:
    default: {
      int span = (int)maxBrightness - (int)minBrightness;
      int v = constrain((int)ldrFiltered, LDR_ADC_MIN, LDR_ADC_MAX);
      int raw = map(v, LDR_ADC_MIN, LDR_ADC_MAX, 0, span);
      if (raw < 0) raw = 0; if (raw > span) raw = span;
      float ratio = powf((float)raw / (float)span, 1.4f);
      int bLDR = (int)minBrightness + (int)(ratio * span + 0.5f);
      int bFinal = (int)maxBrightness - ((int)maxBrightness - bLDR) * (int)ldrInfluence / 100;
      currentBrightness = (uint8_t)constrain(bFinal, (int)minBrightness, (int)maxBrightness);
      break;
    }
  }
  FastLED.setBrightness(currentBrightness);
}

// ============================================================
// Clock display
// ============================================================
void updateClockDisplay() {
  struct tm ti;
  if (!getLocalTime(&ti, 10)) {
    static unsigned long lastBlink = 0; static bool on = false;
    if (millis() - lastBlink > 400) {
      lastBlink = millis(); on = !on; FastLED.clear();
      if (on) { drawChar('S',4,1,CRGB::Orange); drawChar('Y',8,1,CRGB::Orange);
                drawChar('N',12,1,CRGB::Orange); drawChar('C',16,1,CRGB::Orange); }
      FastLED.show();
    }
    return;
  }
  FastLED.clear();
  int h1 = ti.tm_hour / 10, h2 = ti.tm_hour % 10;
  int m1 = ti.tm_min / 10, m2 = ti.tm_min % 10;
  drawChar('0' + h1, 7, 1, CRGB::Green); drawChar('0' + h2, 11, 1, CRGB::Green);
  leds[1 * MATRIX_WIDTH + 15] = CRGB::Green;
  leds[3 * MATRIX_WIDTH + 15] = CRGB::Green;
  drawChar('0' + m1, 17, 1, CRGB::Green); drawChar('0' + m2, 21, 1, CRGB::Green);
  int secPix = ti.tm_sec / 2;
  if (secPix >= 0 && secPix < MATRIX_WIDTH) leds[7 * MATRIX_WIDTH + secPix] = CRGB::Blue;
  FastLED.show();
}
void updateApModeDisplay() {
  if (millis() - apScrollLast < 60) return;
  apScrollLast = millis();
  String msg = "SSID:" + String(AP_SSID) + " IP:" + WiFi.softAPIP().toString() + " PW:" + String(AP_PASS) + "  ";
  int msgW = msg.length() * 4;
  FastLED.clear();
  for (int i = 0; i < (int)msg.length(); i++) {
    int x = apScrollOffset + i * 4;
    if (x >= -3 && x < MATRIX_WIDTH) drawChar(msg[i], x, 1, CRGB::Blue);
  }
  FastLED.show(); apScrollOffset--;
  if (apScrollOffset < -msgW) apScrollOffset = MATRIX_WIDTH;
}

// ============================================================
// Weather
// ============================================================
void fetchWeather() {
  if (!wifiConnected) return;
  wxLastFetch = millis();
  HTTPClient http; http.setTimeout(15000);
  String loc = "";
  if (weatherMode == 1 && weatherZip.length()) loc = weatherZip;
  else if (weatherMode == 2) { char b[32]; snprintf(b, sizeof(b), "%.4f,%.4f", weatherLat, weatherLon); loc = b; }
  String url = "https://wttr.in/" + loc + "?format=j1";
  if (!http.begin(url)) { wxError = "begin failed"; return; }
  int code = http.GET();
  if (code != HTTP_CODE_OK) { wxError = "HTTP " + String(code); http.end(); return; }
  String payload = http.getString(); http.end();
  if (payload.length() == 0 || payload[0] != '{') { wxError = "bad payload"; return; }
  StaticJsonDocument<256> filter;
  filter["current_condition"][0]["temp_C"] = true;
  filter["current_condition"][0]["humidity"] = true;
  filter["current_condition"][0]["weatherDesc"][0]["value"] = true;
  filter["current_condition"][0]["weatherCode"] = true;
  filter["nearest_area"][0]["areaName"][0]["value"] = true;
  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) { wxError = "json"; return; }
  JsonObject cc = doc["current_condition"][0];
  if (cc.isNull()) { wxError = "no cc"; return; }
  wxTempC = atof((const char*)(cc["temp_C"] | "0"));
  wxHumidity = atoi((const char*)(cc["humidity"] | "0"));
  wxDesc = (const char*)(cc["weatherDesc"][0]["value"] | "");
  weatherCode = atoi((const char*)(cc["weatherCode"] | "0"));
  wxDescRu = String(getWeatherDescRu(weatherCode));
  wxCity = (const char*)(doc["nearest_area"][0]["areaName"][0]["value"] | "");
  wxValid = true; wxError = ""; needPublishState = true;
  addLog("Weather: " + wxCity + " " + String(wxTempC, 1) + "C, " + wxDescRu);
}
static WeatherIconType getWeatherIconType(int code) {
  if (code == 0) return WI_UNKNOWN;
  if (code == 113) return WI_SUN;
  if (code == 116) return WI_PARTLY;
  if (code == 119 || code == 122) return WI_CLOUD;
  if (code == 143 || code == 248 || code == 260) return WI_FOG;
  if (code == 200 || code == 386 || code == 389 || code == 392 || code == 395) return WI_THUNDER;
  if (code == 179 || code == 182 || code == 185 || code == 227 || code == 230 ||
      code == 281 || code == 284 || code == 311 || code == 314 ||
      code == 317 || code == 320 || code == 323 || code == 326 ||
      code == 329 || code == 332 || code == 335 || code == 338 ||
      code == 350 || code == 362 || code == 365 || code == 368 ||
      code == 371 || code == 374 || code == 377) return WI_SNOW;
  if (code == 176 || code == 263 || code == 266 || code == 293 ||
      code == 296 || code == 299 || code == 302 || code == 305 ||
      code == 308 || code == 353 || code == 356 || code == 359) return WI_RAIN;
  return WI_RAIN;
}
static const char* getWeatherIconName(int code) {
  switch (getWeatherIconType(code)) {
    case WI_SUN: return "sun"; case WI_PARTLY: return "partly";
    case WI_CLOUD: return "cloud"; case WI_FOG: return "fog";
    case WI_RAIN: return "rain"; case WI_SNOW: return "snow";
    case WI_THUNDER: return "thunder"; default: return "unknown";
  }
}
static const char* getWeatherColor(int code) {
  switch (getWeatherIconType(code)) {
    case WI_SUN: return "#ffcc00"; case WI_PARTLY: return "#ffd966";
    case WI_CLOUD: return "#b0b0b0"; case WI_FOG: return "#a0a0a0";
    case WI_RAIN: return "#3399ff"; case WI_SNOW: return "#ffffff";
    case WI_THUNDER: return "#ffff33"; default: return "#888888";
  }
}
static const char* getWeatherDescRu(int code) {
  switch (code) {
    case 113: return "Ясно"; case 116: return "Переменная облачность";
    case 119: return "Облачно"; case 122: return "Пасмурно";
    case 143: return "Дымка"; case 176: return "Местами дождь";
    case 179: return "Местами снег"; case 182: return "Местами мокрый снег";
    case 185: return "Местами ледяная морось"; case 200: return "Грозовые очаги";
    case 227: return "Позёмок"; case 230: return "Метель";
    case 248: return "Туман"; case 260: return "Ледяной туман";
    case 263: return "Местами слабая морось"; case 266: return "Слабая морось";
    case 281: return "Ледяная морось"; case 284: return "Сильная ледяная морось";
    case 293: return "Местами слабый дождь"; case 296: return "Слабый дождь";
    case 299: return "Умеренный дождь временами"; case 302: return "Умеренный дождь";
    case 305: return "Сильный дождь временами"; case 308: return "Сильный дождь";
    case 311: return "Слабый ледяной дождь"; case 314: return "Умеренный ледяной дождь";
    case 317: return "Слабый мокрый снег"; case 320: return "Умеренный мокрый снег";
    case 323: return "Местами слабый снег"; case 326: return "Слабый снег";
    case 329: return "Местами умеренный снег"; case 332: return "Умеренный снег";
    case 335: return "Местами сильный снег"; case 338: return "Сильный снег";
    case 350: return "Ледяная крупа"; case 353: return "Слабый ливень";
    case 356: return "Умеренный ливень"; case 359: return "Проливной ливень";
    case 362: return "Слабый ливневый мокрый снег"; case 365: return "Сильный ливневый мокрый снег";
    case 368: return "Слабый ливневый снег"; case 371: return "Сильный ливневый снег";
    case 374: return "Слабый ливень ледяной крупы"; case 377: return "Сильный ливень ледяной крупы";
    case 386: return "Дождь с грозой"; case 389: return "Сильный дождь с грозой";
    case 392: return "Снег с грозой"; case 395: return "Сильный снег с грозой";
    default: return "—";
  }
}

// ---------------- 8x8 RGB Weather Icons (0 = transparent) ----------------
static const uint32_t RGB_ICON_SUN[8][8] = {
  {0x008000,0x008000,0x000000,0x000000,0x000000,0xFFFF00,0xFFFF00,0xFFFF00},
  {0x00B000,0x00B000,0x00B000,0x000000,0x000000,0x000000,0xFFFF00,0xFFFF00},
  {0x000000,0x8B5A2B,0x000000,0x00B000,0x000000,0x000000,0x000000,0xFFFF00},
  {0x000000,0x8B5A2B,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x8B5A2B,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0xFFCC00,0xFFCC00,0xFFCC00,0x000000,0x0088FF,0x0088FF,0x0088FF,0x0088FF},
  {0xFFCC00,0xFFCC00,0xFFCC00,0xFFCC00,0x0088FF,0x0088FF,0x0088FF,0x0088FF},
  {0xFFCC00,0xFFCC00,0xFFCC00,0xFFCC00,0xFFCC00,0x0088FF,0x0088FF,0x0088FF}
};
static const uint32_t RGB_ICON_PARTLY[8][8] = {
  {0x000000,0x000000,0xFFFF00,0xFFFF00,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0xFFFF00,0xFFFF00,0xFFFF00,0xFFFF00,0x444444,0x000000,0x000000},
  {0xFFFF00,0xFFFF00,0xFFFF00,0x666666,0xFFFFFF,0xFFFFFF,0x666666,0x000000},
  {0x666666,0xCCCCCC,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0x666666},
  {0x888888,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0x888888},
  {0x444444,0x888888,0x888888,0x888888,0x888888,0x888888,0x888888,0x444444},
  {0x000000,0x444444,0x444444,0xFFFF00,0x444444,0x444444,0x444444,0x000000},
  {0x000000,0x000000,0xFFFF00,0xFFFF00,0x000000,0x000000,0x000000,0x000000}
};
static const uint32_t RGB_ICON_CLOUD[8][8] = {
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x444444,0x444444,0x000000},
  {0x000000,0x444444,0x444444,0x000000,0x666666,0xFFFFFF,0xFFFFFF,0x666666},
  {0x444444,0xCCCCCC,0xFFFFFF,0x666666,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF},
  {0x888888,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC},
  {0x444444,0x888888,0x888888,0x888888,0x888888,0x888888,0x888888,0x444444},
  {0x000000,0x444444,0x444444,0x444444,0x444444,0x444444,0x444444,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000}
};
static const uint32_t RGB_ICON_FOG[8][8] = {
  {0x777777,0x888888,0x888888,0x888888,0x888888,0x888888,0x888888,0x777777},
  {0x000000,0x555555,0x666666,0x666666,0x666666,0x666666,0x555555,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000}
};
static const uint32_t RGB_ICON_RAIN[8][8] = {
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x444444,0x444444,0x000000},
  {0x000000,0x444444,0x444444,0x000000,0x666666,0xFFFFFF,0xFFFFFF,0x666666},
  {0x444444,0xCCCCCC,0xFFFFFF,0x666666,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF},
  {0x888888,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC},
  {0x444444,0x888888,0x888888,0x888888,0x888888,0x888888,0x888888,0x444444},
  {0x000000,0x0044FF,0x444444,0x444444,0x0044FF,0x444444,0x444444,0x000000},
  {0x000000,0x0044FF,0x000000,0x000000,0x0044FF,0x000000,0x000000,0x000000},
  {0x000000,0x0044FF,0x000000,0x000000,0x0044FF,0x000000,0x000000,0x000000}
};
static const uint32_t RGB_ICON_SNOW[8][8] = {
  {0x000000,0x0066FF,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x3399FF,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x3399FF,0x3399FF,0x3399FF,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x3399FF,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x0066FF,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x0066FF,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000}
};
static const uint32_t RGB_ICON_THUNDER[8][8] = {
  {0x000000,0x000000,0x000000,0x000000,0x000000,0x444444,0x444444,0x000000},
  {0x000000,0x444444,0x444444,0x000000,0x666666,0xFFFFFF,0xFFFFFF,0x666666},
  {0x444444,0xCCCCCC,0xFFFFFF,0x666666,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF},
  {0x888888,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC,0xCCCCCC},
  {0x444444,0x888888,0x888888,0x888888,0x888888,0x888888,0x888888,0x444444},
  {0x000000,0x444444,0x444444,0x444444,0x444444,0x444444,0x444444,0x000000},
  {0x000000,0x000000,0x000000,0xFFFF00,0x000000,0x000000,0x000000,0x000000},
  {0x000000,0x0033CC,0x000000,0x000000,0xFFFF00,0x000000,0x000000,0x000000}
};
static const uint8_t ICON_UNKNOWN[8] = {0x00,0x7C,0xC6,0xC6,0x0C,0x18,0x00,0x18};

static void drawBitmap8x8(int x, int y, const uint8_t* bmp, CRGB color) {
  for (int row = 0; row < 8; row++) {
    uint8_t bits = bmp[row];
    for (int col = 0; col < 8; col++)
      if (bits & (0x80 >> col)) {
        int px = x + col, py = y + row;
        if (px >= 0 && px < MATRIX_WIDTH && py >= 0 && py < MATRIX_HEIGHT)
          leds[py * MATRIX_WIDTH + px] = color;
      }
  }
}
static void drawRgbBitmap8x8(int x, int y, const uint32_t bmp[8][8]) {
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      uint32_t c = bmp[row][col];
      if (c != 0) {
        int px = x + col, py = y + row;
        if (px >= 0 && px < MATRIX_WIDTH && py >= 0 && py < MATRIX_HEIGHT) {
          leds[py * MATRIX_WIDTH + px] = CRGB((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF);
        }
      }
    }
  }
}

void drawWeatherAnimation() {
  FastLED.clear();
  weatherAnimPhase++;
  int xo = (MATRIX_WIDTH - 8) / 2;
  int yo = 0;
  int driftX = (int)(sin8(weatherAnimPhase * 2) - 128) / 42;
  switch (getWeatherIconType(weatherCode)) {
    case WI_SUN:     drawRgbBitmap8x8(xo, yo, RGB_ICON_SUN); break;
    case WI_PARTLY:  drawRgbBitmap8x8(xo + driftX, yo, RGB_ICON_PARTLY); break;
    case WI_CLOUD:   drawRgbBitmap8x8(xo + driftX, yo, RGB_ICON_CLOUD); break;
    case WI_FOG:     drawRgbBitmap8x8(xo + driftX, yo, RGB_ICON_FOG); break;
    case WI_RAIN:    drawRgbBitmap8x8(xo, yo, RGB_ICON_RAIN); break;
    case WI_SNOW:    drawRgbBitmap8x8(xo, yo, RGB_ICON_SNOW); break;
    case WI_THUNDER: drawRgbBitmap8x8(xo, yo, RGB_ICON_THUNDER); break;
    default:         drawBitmap8x8(xo, yo, ICON_UNKNOWN, CRGB(150, 150, 150)); break;
  }
  FastLED.show();
}
void updateDisplay() {
  unsigned long now = millis();
  unsigned long elapsed = now - displayModeStartAt;
  if (!showWeatherMode && elapsed >= TIME_MODE_DURATION && wxValid) {
    showWeatherMode = true; displayModeStartAt = now; weatherAnimPhase = 0;
  } else if (showWeatherMode && elapsed >= WEATHER_MODE_DURATION) {
    showWeatherMode = false; displayModeStartAt = now;
  }
  if (showWeatherMode) drawWeatherAnimation(); else updateClockDisplay();
}

// ============================================================
// MQTT
// ============================================================
static const char* brightModeName(uint8_t m) {
  switch (m) {
    case BRIGHT_SENSOR: return "sensor";
    case BRIGHT_SCHEDULE: return "schedule";
    case BRIGHT_MANUAL: return "manual";
    default: return "sensor";
  }
}
static void mqttEventHandler(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_MQTT_OPEN) {
    mqttConnected = true; addLog("MQTT connected");
    struct mg_mqtt_opts sub = {0};
    sub.topic = mg_str(topicSet.c_str()); sub.qos = 0;
    mg_mqtt_sub(c, &sub);
    struct mg_mqtt_opts pub = {0};
    pub.topic = mg_str(topicAvail.c_str()); pub.message = mg_str("online"); pub.retain = true;
    mg_mqtt_pub(c, &pub);
    needPublishDiscovery = haDiscovery; needPublishState = true;
  } else if (ev == MG_EV_MQTT_MSG) {
    struct mg_mqtt_message *mm = (struct mg_mqtt_message *) ev_data;
    String msg = String(mm->data.buf, mm->data.len);
    msg.trim();
    addLog("MQTT msg: " + msg);
    if (msg.startsWith("{")) applyHaJsonCommand(msg);
    else processCommand(msg);
  } else if (ev == MG_EV_CLOSE) { mqttConnected = false; addLog("MQTT disconnected"); }
}
void connectMQTT() {
  if (!mqttEnabled || !wifiConnected || mqtt_conn != nullptr) return;
  String url = "mqtt://" + mqttServer + ":" + String(mqttPort);
  struct mg_mqtt_opts opts = {0};
  opts.client_id = mg_str(mqttClientId.c_str());
  if (mqttUser.length()) opts.user = mg_str(mqttUser.c_str());
  if (mqttPassword.length()) opts.pass = mg_str(mqttPassword.c_str());
  opts.keepalive = 30; opts.clean = true;
  opts.topic = mg_str(topicAvail.c_str());
  opts.message = mg_str("offline"); opts.retain = true;
  mqtt_conn = mg_mqtt_connect(&mgr, url.c_str(), &opts, mqttEventHandler, nullptr);
  if (mqtt_conn == nullptr) addLog("MQTT connect failed");
}
void disconnectMQTT() {
  if (mqtt_conn) {
    struct mg_mqtt_opts opts = {0};
    opts.topic = mg_str(topicAvail.c_str()); opts.message = mg_str("offline"); opts.retain = true;
    mg_mqtt_pub(mqtt_conn, &opts);
    mqtt_conn->is_closing = 1; mqtt_conn = nullptr; mqttConnected = false;
  }
}
bool mqttPublishJson(const String& topic, const String& json) {
  if (!mqttConnected || !mqtt_conn) return false;
  struct mg_mqtt_opts opts = {0};
  opts.topic = mg_str(topic.c_str()); opts.message = mg_str(json.c_str()); opts.retain = true;
  return mg_mqtt_pub(mqtt_conn, &opts);
}
void publishState() {
  if (!mqttConnected) return;
  lastStatePublish = millis();
  DynamicJsonDocument doc(6144);
  doc["brightness"] = currentBrightness;
  doc["max_brightness"] = maxBrightness;
  doc["min_brightness"] = minBrightness;
  doc["ldr_influence"] = ldrInfluence;
  doc["brightness_mode"] = brightModeName(brightnessMode);
  doc["manual_brightness"] = manualBrightness;
  doc["night_brightness"] = nightBrightness;
  doc["night_start_hour"] = nightStartHour;
  doc["night_end_hour"] = nightEndHour;
  doc["is_night"] = isNightMode;
  doc["buzzer_volume"] = buzzerVolume;
  doc["hostname"] = deviceName;
  doc["ip"] = WiFi.localIP().toString();
  int rssi = WiFi.RSSI();
  doc["rssi"] = rssi;
  doc["wifi_percent"] = (rssi <= -100) ? 0 : (rssi >= -50 ? 100 : 2 * (rssi + 100));
  doc["weather_ok"] = wxValid;
  doc["weather_temp"] = isnan(wxTempC) ? 0 : wxTempC;
  doc["weather_desc"] = wxDesc; doc["weather_desc_ru"] = wxDescRu;
  doc["weather_city"] = wxCity; doc["weather_hum"] = wxHumidity;
  doc["weather_code"] = weatherCode; doc["weather_icon"] = getWeatherIconName(weatherCode);
  doc["melody_playing"] = (melodyState == MEL_PLAYING);
  doc["melody_id"] = melodyPlayingId;
  JsonArray harr = doc.createNestedArray("holidays");
  for (int i = 0; i < todayHolidaysCount; i++) harr.add(todayHolidays[i]);
  JsonArray carr = doc.createNestedArray("holiday_categories");
  for (int i = 0; i < todayHolidaysCount; i++) carr.add(todayHolidaysCats[i]);
  doc["holiday_count"] = todayHolidaysCount;
  doc["holiday_valid"] = holidayValid;
  doc["holiday_date"] = todayHolidaysDate;
  doc["holiday"] = todayHolidaysCount > 0 ? todayHolidays[0] : "";
  doc["holiday_cat_ru"] = holidayCatRussia;
  doc["holiday_cat_prof"] = holidayCatProf;
  doc["holiday_cat_intl"] = holidayCatIntl;
  doc["holiday_cat_corp"] = holidayCatCorp;
  doc["holiday_cat_eco"] = holidayCatEco;
  doc["holiday_cat_med"] = holidayCatMed;
  doc["holiday_cat_info"] = holidayCatInfo;
  doc["sensor_has_temp"] = !isnan(sensorTemp);
  doc["sensor_has_hum"] = !isnan(sensorHum);
  doc["sensor_has_press"] = !isnan(sensorPress);
  doc["sensor_temp"] = isnan(sensorTemp) ? 0 : sensorTemp;
  doc["sensor_hum"] = isnan(sensorHum) ? 0 : sensorHum;
  doc["sensor_press"] = isnan(sensorPress) ? 0 : sensorPress;
  String json; serializeJson(doc, json);
  mqttPublishJson(topicState, json);
}
void publishDiscovery() {
  if (!mqttConnected || !haDiscovery) return;
  auto pub = [](const String& topic, const String& json) {
    struct mg_mqtt_opts opts = {0};
    opts.topic = mg_str(topic.c_str()); opts.message = mg_str(json.c_str()); opts.retain = true;
    if (mqtt_conn) mg_mqtt_pub(mqtt_conn, &opts);
  };
  String base = "homeassistant/sensor/" + mqttClientId;
  const struct { const char* uid; const char* name; const char* tpl; const char* unit; } sensors[] = {
    {"wifi", "Wi-Fi (%)", "{{ value_json.wifi_percent }}", "%"},
    {"holiday_count", "Праздников сегодня", "{{ value_json.holiday_count }}", ""},
    {"wx_temp", "Температура (погода)", "{{ value_json.weather_temp }}", "°C"},
    {"wx_desc", "Описание погоды", "{{ value_json.weather_desc_ru }}", ""},
    {"weather_icon", "Иконка погоды", "{{ value_json.weather_icon }}", ""},
    {"brightness_now", "Текущая яркость", "{{ value_json.brightness }}", ""},
  };
  for (auto& s : sensors) {
    StaticJsonDocument<512> d;
    d["name"] = s.name;
    d["unique_id"] = mqttClientId + "_" + s.uid;
    d["state_topic"] = topicState; d["value_template"] = s.tpl;
    if (strlen(s.unit)) d["unit_of_measurement"] = s.unit;
    d["entity_category"] = "diagnostic";
    String json; serializeJson(d, json);
    pub(base + "_" + s.uid + "/config", json);
  }
  {
    StaticJsonDocument<1024> d;
    d["name"] = "Режим яркости";
    d["unique_id"] = mqttClientId + "_bright_mode";
    d["state_topic"] = topicState;
    d["value_template"] = "{{ value_json.brightness_mode }}";
    d["command_topic"] = topicSet;
    d["command_template"] = "BRIGHT_MODE:{{ value }}";
    JsonArray opts = d.createNestedArray("options");
    opts.add("sensor"); opts.add("schedule"); opts.add("manual");
    d["icon"] = "mdi:brightness-auto"; d["entity_category"] = "config";
    String json; serializeJson(d, json);
    pub("homeassistant/select/" + mqttClientId + "_bright_mode/config", json);
  }
  auto pubNumber = [&](const char* uid, const char* name, const char* tpl, const char* cmd,
                       int mn, int mx, int step, const char* unit, const char* icon) {
    StaticJsonDocument<512> d;
    d["name"] = name;
    d["unique_id"] = mqttClientId + "_" + uid;
    d["state_topic"] = topicState;
    d["value_template"] = tpl;
    d["command_topic"] = topicSet;
    d["command_template"] = cmd;
    d["min"] = mn; d["max"] = mx; d["step"] = step;
    d["mode"] = "slider";
    if (unit) d["unit_of_measurement"] = unit;
    d["icon"] = icon;
    d["entity_category"] = "config";
    String json; serializeJson(d, json);
    pub("homeassistant/number/" + mqttClientId + "_" + uid + "/config", json);
  };
  pubNumber("manual_br", "Ручная яркость", "{{ value_json.manual_brightness }}",
            "MANUAL_BRIGHTNESS:{{ value | int }}", 1, 255, 1, nullptr, "mdi:brightness-6");
  pubNumber("night_br", "Ночная яркость", "{{ value_json.night_brightness }}",
            "NIGHT_BRIGHTNESS:{{ value | int }}", 1, 255, 1, nullptr, "mdi:weather-night");
  pubNumber("nstart", "Начало ночи (час)", "{{ value_json.night_start_hour }}",
            "NIGHT_START:{{ value | int }}", 0, 23, 1, nullptr, "mdi:clock-start");
  pubNumber("nend", "Конец ночи (час)", "{{ value_json.night_end_hour }}",
            "NIGHT_END:{{ value | int }}", 0, 23, 1, nullptr, "mdi:clock-end");
  pubNumber("buzz_vol", "Громкость буззера", "{{ value_json.buzzer_volume }}",
            "BUZZER_VOLUME:{{ value | int }}", 0, 100, 1, "%", "mdi:volume-high");
  for (int i = 0; i < 3; i++) {
    StaticJsonDocument<512> d;
    String name = melodies[i].name.length() ? melodies[i].name : ("Мелодия " + String(i + 1));
    d["name"] = "Мелодия: " + name;
    d["unique_id"] = mqttClientId + "_mel" + String(i + 1);
    d["command_topic"] = topicSet;
    d["payload_press"] = "MELODY_PLAY:" + String(i + 1);
    d["icon"] = "mdi:music-note";
    String json; serializeJson(d, json);
    pub("homeassistant/button/" + mqttClientId + "_mel" + String(i + 1) + "/config", json);
  }
  {
    StaticJsonDocument<512> d;
    d["name"] = "Стоп мелодии"; d["unique_id"] = mqttClientId + "_mel_stop";
    d["command_topic"] = topicSet; d["payload_press"] = "MELODY_STOP";
    d["icon"] = "mdi:stop";
    String json; serializeJson(d, json);
    pub("homeassistant/button/" + mqttClientId + "_mel_stop/config", json);
  }
  addLog("HA discovery published");
}

// ============================================================
// Commands
// ============================================================
bool applyHaJsonCommand(const String& msg) {
  StaticJsonDocument<512> j;
  if (deserializeJson(j, msg)) return false;
  bool handled = false;
  if (j.containsKey("brightness"))      { processCommand("BRIGHTNESS:" + String((int)(j["brightness"] | 0))); handled = true; }
  if (j.containsKey("max_brightness"))  { processCommand("MAX_BRIGHTNESS:" + String((int)(j["max_brightness"] | 0))); handled = true; }
  if (j.containsKey("min_brightness"))  { processCommand("MIN_BRIGHTNESS:" + String((int)(j["min_brightness"] | 0))); handled = true; }
  if (j.containsKey("ldr_influence"))   { processCommand("LDR_INFLUENCE:" + String((int)(j["ldr_influence"] | 100))); handled = true; }
  if (j.containsKey("brightness_mode")) { processCommand("BRIGHT_MODE:" + String((const char*)(j["brightness_mode"] | "sensor"))); handled = true; }
  if (j.containsKey("manual_brightness")){ processCommand("MANUAL_BRIGHTNESS:" + String((int)(j["manual_brightness"] | 60))); handled = true; }
  if (j.containsKey("night_brightness")){ processCommand("NIGHT_BRIGHTNESS:" + String((int)(j["night_brightness"] | 3))); handled = true; }
  if (j.containsKey("night_start_hour")){ processCommand("NIGHT_START:" + String((int)(j["night_start_hour"] | 22))); handled = true; }
  if (j.containsKey("night_end_hour"))  { processCommand("NIGHT_END:" + String((int)(j["night_end_hour"] | 7))); handled = true; }
  if (j.containsKey("buzzer_volume"))   { processCommand("BUZZER_VOLUME:" + String((int)(j["buzzer_volume"] | 70))); handled = true; }
  if (j.containsKey("timezone_hours"))  { processCommand("TIMEZONE:" + String((float)(j["timezone_hours"] | 0.0))); handled = true; }
  if (j.containsKey("ntp_server"))      { processCommand("NTP:" + String((const char*)(j["ntp_server"] | ""))); handled = true; }
  if (j.containsKey("hostname"))        { processCommand("HOSTNAME:" + String((const char*)(j["hostname"] | ""))); handled = true; }
  if (j.containsKey("command"))         { processCommand(String((const char*)(j["command"] | ""))); handled = true; }
  return handled;
}
void processCommand(const String& raw) {
  String cmd = raw; cmd.trim();
  if (cmd.isEmpty()) return;
  if (cmd.equalsIgnoreCase("REBOOT")) { rebootPending = true; rebootAt = millis() + 1500; return; }
  if (cmd.equalsIgnoreCase("FACTORY_RESET")) {
    prefs.begin("clock", false); prefs.clear(); prefs.end();
    for (int i = 0; i < 3; i++) { melodies[i].name = ""; melodies[i].rtttl = ""; }
    buzzerVolume = BUZZER_VOL_DEFAULT;
    brightnessMode = BRIGHT_SENSOR;
    manualBrightness = MANUAL_BRIGHTNESS_DEF;
    nightBrightness = NIGHT_BRIGHTNESS_DEF;
    nightStartHour = NIGHT_START_HOUR_DEF;
    nightEndHour = NIGHT_END_HOUR_DEF;
    stopMelody();
    rebootPending = true; rebootAt = millis() + 1500; return;
  }
  if (cmd.equalsIgnoreCase("FACTORY_RESET_ALL")) {
    prefs.begin("clock", false); prefs.clear(); prefs.end();
    for (int i = 0; i < 3; i++) { melodies[i].name = ""; melodies[i].rtttl = ""; }
    buzzerVolume = BUZZER_VOL_DEFAULT;
    brightnessMode = BRIGHT_SENSOR;
    manualBrightness = MANUAL_BRIGHTNESS_DEF;
    nightBrightness = NIGHT_BRIGHTNESS_DEF;
    nightStartHour = NIGHT_START_HOUR_DEF;
    nightEndHour = NIGHT_END_HOUR_DEF;
    stopMelody();
    WiFi.disconnect(true, true);
    rebootPending = true; rebootAt = millis() + 1500; return;
  }
  if (cmd.equalsIgnoreCase("NTP_SYNC"))        { syncTime(); return; }
  if (cmd.equalsIgnoreCase("STATUS"))          { needPublishState = true; return; }
  if (cmd.equalsIgnoreCase("WEATHER_REFRESH")) { wxLastFetch = 0; return; }
  if (cmd.equalsIgnoreCase("HOLIDAY_REFRESH")) { needRecalcHolidays = true; return; }
  if (cmd.equalsIgnoreCase("MELODY_STOP"))     { stopMelody(); needPublishState = true; return; }
  if (cmd.startsWith("MELODY_PLAY:")) {
    int id = cmd.substring(12).toInt() - 1;
    if (id >= 0 && id < 3) startMelody(id);
    return;
  }
  auto starts = [&](const char* p) { return cmd.startsWith(p); };
  auto val = [&](const char* p) { return cmd.substring(strlen(p)); };

  if (starts("BRIGHTNESS:")) { currentBrightness = (uint8_t)constrain(val("BRIGHTNESS:").toInt(), (int)minBrightness, (int)maxBrightness); FastLED.setBrightness(currentBrightness); needPublishState = true; return; }
  if (starts("BRIGHT_MODE:")) {
    String m = val("BRIGHT_MODE:"); m.trim(); m.toLowerCase();
    if (m == "sensor") brightnessMode = BRIGHT_SENSOR;
    else if (m == "schedule") brightnessMode = BRIGHT_SCHEDULE;
    else if (m == "manual") brightnessMode = BRIGHT_MANUAL;
    else return;
    saveSettings(); applyBrightness(); needPublishState = true;
    addLog("Bright mode: " + m); return;
  }
  if (starts("MANUAL_BRIGHTNESS:")) { manualBrightness = (uint8_t)constrain(val("MANUAL_BRIGHTNESS:").toInt(), (int)MIN_BRIGHTNESS_HARD, (int)maxBrightness); saveSettings(); applyBrightness(); needPublishState = true; return; }
  if (starts("NIGHT_BRIGHTNESS:"))  { nightBrightness = (uint8_t)constrain(val("NIGHT_BRIGHTNESS:").toInt(), (int)MIN_BRIGHTNESS_HARD, (int)maxBrightness); saveSettings(); applyBrightness(); needPublishState = true; return; }
  if (starts("NIGHT_START:"))       { nightStartHour = (uint8_t)constrain(val("NIGHT_START:").toInt(), 0, 23); saveSettings(); applyBrightness(); needPublishState = true; return; }
  if (starts("NIGHT_END:"))         { nightEndHour = (uint8_t)constrain(val("NIGHT_END:").toInt(), 0, 23); saveSettings(); applyBrightness(); needPublishState = true; return; }
  if (starts("MAX_BRIGHTNESS:")) {
    maxBrightness = (uint8_t)constrain(val("MAX_BRIGHTNESS:").toInt(), (int)MIN_BRIGHTNESS_HARD, 255);
    if (maxBrightness < minBrightness) maxBrightness = minBrightness;
    if (manualBrightness > maxBrightness) manualBrightness = maxBrightness;
    if (nightBrightness > maxBrightness) nightBrightness = maxBrightness;
    saveSettings(); applyBrightness(); needPublishState = true; return;
  }
  if (starts("MIN_BRIGHTNESS:")) { minBrightness = (uint8_t)constrain(val("MIN_BRIGHTNESS:").toInt(), (int)MIN_BRIGHTNESS_HARD, (int)maxBrightness); saveSettings(); applyBrightness(); needPublishState = true; return; }
  if (starts("LDR_INFLUENCE:"))  { ldrInfluence = (uint8_t)constrain(val("LDR_INFLUENCE:").toInt(), 0, 100); saveSettings(); applyBrightness(); needPublishState = true; return; }
  if (starts("BUZZER_VOLUME:"))  { buzzerVolume = (uint8_t)constrain(val("BUZZER_VOLUME:").toInt(), 0, 100); saveSettings(); needPublishState = true; return; }
  if (starts("TIMEZONE:")) { timezoneOffsetMinutes = (int)round(val("TIMEZONE:").toFloat() * 60); saveSettings(); needRecalcHolidays = true; needPublishState = true; return; }
  if (starts("NTP:")) { ntpServer = val("NTP:"); ntpServer.trim(); saveSettings(); syncTime(); return; }
  if (starts("HOSTNAME:")) { deviceName = val("HOSTNAME:"); deviceName.trim(); saveSettings(); needPublishState = true; return; }
  if (starts("SENSOR_TYPE:")) { sensorType = (uint8_t)constrain(val("SENSOR_TYPE:").toInt(), 0, 3); saveSettings(); rebootPending = true; rebootAt = millis() + 5000; return; }
  if (starts("WEATHER_MODE:")) { weatherMode = (uint8_t)constrain(val("WEATHER_MODE:").toInt(), 0, 2); saveSettings(); wxLastFetch = 0; return; }
  if (starts("WEATHER_ZIP:"))  { weatherZip = val("WEATHER_ZIP:"); weatherZip.trim(); saveSettings(); wxLastFetch = 0; return; }
  if (starts("WEATHER_LAT:"))  { weatherLat = val("WEATHER_LAT:").toFloat(); saveSettings(); wxLastFetch = 0; return; }
  if (starts("WEATHER_LON:"))  { weatherLon = val("WEATHER_LON:").toFloat(); saveSettings(); wxLastFetch = 0; return; }
  if (starts("WEATHER_INTERVAL:")) { weatherInterval = (uint32_t)val("WEATHER_INTERVAL:").toInt(); saveSettings(); return; }
  if (starts("MQTT_ENABLE:"))  { mqttEnabled = val("MQTT_ENABLE:").toInt() == 1; saveSettings(); return; }
  if (starts("MQTT_SERVER:"))  { mqttServer = val("MQTT_SERVER:"); mqttServer.trim(); saveSettings(); return; }
  if (starts("MQTT_PORT:"))    { mqttPort = val("MQTT_PORT:").toInt(); saveSettings(); return; }
  if (starts("MQTT_USER:"))    { mqttUser = val("MQTT_USER:"); mqttUser.trim(); saveSettings(); return; }
  if (starts("MQTT_PASSWORD:")){ mqttPassword = val("MQTT_PASSWORD:"); mqttPassword.trim(); saveSettings(); return; }
  if (starts("MQTT_PREFIX:"))  { mqttPrefix = val("MQTT_PREFIX:"); mqttPrefix.trim(); saveSettings(); return; }
  if (starts("HA_DISCOVERY:")) { haDiscovery = val("HA_DISCOVERY:").toInt() == 1; saveSettings(); return; }
  if (starts("HOLIDAY_CAT_RU:"))   { holidayCatRussia = val("HOLIDAY_CAT_RU:").toInt() == 1; saveSettings(); needRecalcHolidays = true; return; }
  if (starts("HOLIDAY_CAT_PROF:")) { holidayCatProf = val("HOLIDAY_CAT_PROF:").toInt() == 1; saveSettings(); needRecalcHolidays = true; return; }
  if (starts("HOLIDAY_CAT_INTL:")) { holidayCatIntl = val("HOLIDAY_CAT_INTL:").toInt() == 1; saveSettings(); needRecalcHolidays = true; return; }
  if (starts("HOLIDAY_CAT_CORP:")) { holidayCatCorp = val("HOLIDAY_CAT_CORP:").toInt() == 1; saveSettings(); needRecalcHolidays = true; return; }
  if (starts("HOLIDAY_CAT_ECO:"))  { holidayCatEco = val("HOLIDAY_CAT_ECO:").toInt() == 1; saveSettings(); needRecalcHolidays = true; return; }
  if (starts("HOLIDAY_CAT_MED:"))  { holidayCatMed = val("HOLIDAY_CAT_MED:").toInt() == 1; saveSettings(); needRecalcHolidays = true; return; }
  if (starts("HOLIDAY_CAT_INFO:")) { holidayCatInfo = val("HOLIDAY_CAT_INFO:").toInt() == 1; saveSettings(); needRecalcHolidays = true; return; }
  addLog("Unknown command: " + cmd);
}

// ============================================================
// OTA
// ============================================================
void setupOTA() {
  ArduinoOTA.setHostname(deviceName.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){ otaInProgress = true; stopMelody(); if (mqttConnected) disconnectMQTT(); FastLED.clear(true); });
  ArduinoOTA.onEnd([](){ addLog("OTA end"); });
  ArduinoOTA.onError([](ota_error_t e){ addLog("OTA err " + String((int)e)); otaInProgress = false; });
  ArduinoOTA.begin();
}

// ============================================================
// HTTP API
// ============================================================
static void handleApiStatus(struct mg_connection *c) {
  DynamicJsonDocument doc(6144);
  doc["fw_version"] = FIRMWARE_VERSION;
  doc["brightness"] = currentBrightness;
  doc["max_brightness"] = maxBrightness;
  doc["min_brightness"] = minBrightness;
  doc["ldr_influence"] = ldrInfluence;
  doc["brightness_mode"] = brightModeName(brightnessMode);
  doc["manual_brightness"] = manualBrightness;
  doc["night_brightness"] = nightBrightness;
  doc["night_start_hour"] = nightStartHour;
  doc["night_end_hour"] = nightEndHour;
  doc["is_night"] = isNightMode;
  doc["buzzer_volume"] = buzzerVolume;
  doc["ntp_server"] = ntpServer;
  doc["timezone_minutes"] = timezoneOffsetMinutes;
  doc["timezone_hours"] = timezoneOffsetMinutes / 60.0;
  doc["hostname"] = deviceName;
  doc["ip"] = wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_mode"] = wifiConnected ? "STA" : "AP";
  int rssi = wifiConnected ? WiFi.RSSI() : -100;
  doc["rssi"] = rssi;
  doc["wifi_percent"] = (rssi <= -100) ? 0 : (rssi >= -50 ? 100 : 2 * (rssi + 100));
  doc["sensor_type_index"] = sensorType;
  doc["sensor_type"] = sensorTypeName(sensorType);
  doc["sensor_ok"] = sensorOk;
  doc["sensor_has_temp"] = !isnan(sensorTemp);
  doc["sensor_has_hum"] = !isnan(sensorHum);
  doc["sensor_has_press"] = !isnan(sensorPress);
  doc["sensor_temp"] = isnan(sensorTemp) ? 0 : sensorTemp;
  doc["sensor_hum"] = isnan(sensorHum) ? 0 : sensorHum;
  doc["sensor_press"] = isnan(sensorPress) ? 0 : sensorPress;
  doc["weather_mode"] = weatherMode;
  doc["weather_zip"] = weatherZip;
  doc["weather_lat"] = weatherLat;
  doc["weather_lon"] = weatherLon;
  doc["weather_interval"] = weatherInterval;
  doc["weather_code"] = weatherCode;
  doc["weather_icon"] = getWeatherIconName(weatherCode);
  doc["holiday_cat_ru"] = holidayCatRussia;
  doc["holiday_cat_prof"] = holidayCatProf;
  doc["holiday_cat_intl"] = holidayCatIntl;
  doc["holiday_cat_corp"] = holidayCatCorp;
  doc["holiday_cat_eco"] = holidayCatEco;
  doc["holiday_cat_med"] = holidayCatMed;
  doc["holiday_cat_info"] = holidayCatInfo;
  doc["holiday_valid"] = holidayValid;
  doc["holiday_count"] = todayHolidaysCount;
  doc["holiday_date"] = todayHolidaysDate;
  doc["holiday"] = todayHolidaysCount > 0 ? todayHolidays[0] : "";
  JsonArray harr = doc.createNestedArray("holidays");
  for (int i = 0; i < todayHolidaysCount; i++) harr.add(todayHolidays[i]);
  JsonArray carr = doc.createNestedArray("holiday_categories");
  for (int i = 0; i < todayHolidaysCount; i++) carr.add(todayHolidaysCats[i]);
  doc["mqtt_enabled"] = mqttEnabled;
  doc["mqtt_server"] = mqttServer;
  doc["mqtt_port"] = mqttPort;
  doc["mqtt_user"] = mqttUser;
  doc["mqtt_prefix"] = mqttPrefix;
  doc["ha_discovery"] = haDiscovery;
  doc["melody_playing"] = (melodyState == MEL_PLAYING);
  doc["melody_id"] = melodyPlayingId;
  String json; serializeJson(doc, json);
  sendJson(c, json);
}
static void handleApiTime(struct mg_connection *c) {
  time_t now = time(nullptr);
  StaticJsonDocument<128> doc;
  doc["ts"] = (now >= 100000) ? (uint32_t)now : 0;
  doc["tz_min"] = timezoneOffsetMinutes;
  doc["valid"] = (now >= 100000);
  String json; serializeJson(doc, json); sendJson(c, json);
}
static void handleApiWeather(struct mg_connection *c) {
  StaticJsonDocument<512> doc;
  doc["valid"] = wxValid; doc["error"] = wxError; doc["city"] = wxCity;
  doc["temp_c"] = isnan(wxTempC) ? 0 : wxTempC;
  doc["humidity"] = wxHumidity; doc["desc"] = wxDesc; doc["desc_ru"] = wxDescRu;
  doc["code"] = weatherCode; doc["icon"] = getWeatherIconName(weatherCode);
  doc["color"] = getWeatherColor(weatherCode);
  doc["age_s"] = wxValid ? (millis() - wxLastFetch) / 1000UL : 0;
  String json; serializeJson(doc, json); sendJson(c, json);
}
static void handleApiLogs(struct mg_connection *c) {
  String json = "[";
  int start = (logCount < LOG_BUFFER_SIZE) ? 0 : logIndex;
  for (int i = 0; i < logCount; i++) {
    if (i) json += ",";
    json += "\"" + jsonEscape(logBuffer[(start + i) % LOG_BUFFER_SIZE]) + "\"";
  }
  json += "]";
  sendJson(c, json);
}
static void handleApiMelodies(struct mg_connection *c) {
  // Ручная сборка JSON для предотвращения переполнения буфера при длинных RTTTL
  String json = "{\"melodies\":[";
  for (int i = 0; i < 3; i++) {
    if (i > 0) json += ",";
    json += "{\"id\":" + String(i + 1) + ",";
    json += "\"name\":\"" + jsonEscape(melodies[i].name) + "\",";
    json += "\"rtttl\":\"" + jsonEscape(melodies[i].rtttl) + "\"}";
  }
  json += "],";
  json += "\"volume\":" + String(buzzerVolume) + ",";
  json += "\"playing\":" + String(melodyState == MEL_PLAYING ? "true" : "false") + ",";
  json += "\"playing_id\":" + String(melodyPlayingId) + "}";
  sendJson(c, json);
}

// ============================================================
// HTML helpers
// ============================================================
static String navHeader(const String& active) {
  String s = "<p class='nav'>";
  s += "<a href='/'"; if (active == "home") s += " class='active'"; s += ">Главная</a>";
  s += "<a href='/melodies'"; if (active == "melodies") s += " class='active'"; s += ">Мелодии</a>";
  s += "<a href='/settings'"; if (active == "settings") s += " class='active'"; s += ">Настройки</a>";
  s += "</p>";
  return s;
}
static const char* COMMON_CSS =
  "body{font-family:Arial;margin:20px;max-width:760px;color:#222}"
  "h3{margin-top:20px;border-bottom:1px solid #ddd;padding-bottom:4px}"
  ".nav{margin:0 0 18px 0;padding-bottom:10px;border-bottom:1px solid #ddd}"
  ".nav a{margin-right:8px;padding:6px 12px;background:#e8e8e8;color:#333;text-decoration:none;border-radius:4px;display:inline-block}"
  ".nav a.active{background:#4a90d9;color:#fff}"
  ".section{padding:12px;background:#f9f9f9;border:1px solid #ddd;border-radius:5px;margin-bottom:10px}"
  ".info{background:#e8f4fd;padding:10px;border-radius:5px;margin-bottom:14px;font-family:monospace}"
  "label{display:inline-block;width:240px;vertical-align:top}"
  "input[type=text],input[type=number],input[type=password],select{width:200px;padding:4px;margin:2px 0}"
  "input[type=range]{width:220px;vertical-align:middle}"
  "textarea{width:100%;font-family:monospace;font-size:12px;padding:6px;box-sizing:border-box}"
  "button{padding:8px 16px;margin:4px;cursor:pointer;border:1px solid #bbb;border-radius:4px;background:#f6f6f6}"
  "button.danger{background:#c0392b;color:#fff;border-color:#a93226}"
  "button.play{background:#27ae60;color:#fff;border-color:#229954}"
  "button.web{background:#9b59b6;color:#fff;border-color:#7d3c98}"
  "button.primary{background:#4a90d9;color:#fff;border-color:#3a7abf}"
  ".cat-row label{display:block;width:auto}"
  ".wx-card{display:flex;align-items:center;gap:16px}"
  "#wx-icon{width:128px;height:128px;image-rendering:pixelated;border:1px solid #333;border-radius:8px;background:#111;padding:8px;box-sizing:border-box}"
  "#log{background:#f8f8f8;padding:10px;max-height:200px;overflow:auto;font-size:12px;font-family:monospace;border:1px solid #ddd}"
  "#holiday-box ul{margin:0;padding-left:20px}"
  ".mode-hint{font-size:12px;color:#666;margin-top:6px;line-height:1.4}"
  ".badge{display:inline-block;padding:2px 6px;border-radius:3px;font-size:11px;background:#eaeaea;color:#555;margin-left:6px}"
  ".badge.night{background:#3a4a6a;color:#e0e8ff}"
  // --- ИСПРАВЛЕННЫЙ БЛОК CSS ---
  ".mel-name-row{display:flex;align-items:center;gap:8px;flex-wrap:nowrap;margin-bottom:6px}"
  ".mel-name-row label{margin:0;flex:0 0 auto;width:auto;white-space:nowrap}"
  ".mel-name-row input[type=text]{flex:1 1 auto;min-width:120px;width:auto;margin:0;box-sizing:border-box}"
  ".mel-name-row button{margin:0;white-space:nowrap;flex:0 0 auto;padding:6px 12px}"
  "@media(max-width:600px){.mel-name-row{flex-wrap:wrap}}";
static const char* COMMON_JS =
  "function cmd(c){return fetch('/api?command='+encodeURIComponent(c))}"
  "function esc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/\"/g,'&quot;').replace(/'/g,'&#39;')}"
  "var _t={baseUtc:0,tzMin:0,fetchedAt:0,valid:false};"
  "function refreshTime(){return fetch('/api/time').then(r=>r.json()).then(d=>{_t.baseUtc=d.ts;_t.tzMin=d.tz_min;_t.fetchedAt=Date.now();_t.valid=!!d.valid})}"
  "function pad2(n){return String(n).padStart(2,'0')}"
  "function renderTime(){var e=document.getElementById('time');if(!e)return;if(!_t.valid){e.textContent='—';return;}"
  "var ms=(_t.baseUtc*1000)+(_t.tzMin*60*1000)+(Date.now()-_t.fetchedAt);var d=new Date(ms);"
  "e.textContent=pad2(d.getUTCDate())+'.'+pad2(d.getUTCMonth()+1)+'.'+d.getUTCFullYear()+' '+pad2(d.getUTCHours())+':'+pad2(d.getUTCMinutes())+':'+pad2(d.getUTCSeconds());}"
  "setInterval(renderTime,500);"
  "function loadLogs(){fetch('/api/logs').then(r=>r.json()).then(d=>{var e=document.getElementById('log');if(!e)return;e.innerHTML=d.map(esc).join('<br>');e.scrollTop=e.scrollHeight}).catch(()=>{})}";

// ============================================================
// HTTP handler
// ============================================================
static void httpEventHandler(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_HDRS) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    if (mg_match(hm->uri, mg_str("/api/ota/update"), NULL)) {
      mg_http_stream_body(c, hm, mg_http_ota_cb, NULL);
      addLog("OTA update started");
      return;
    }
  }
  if (ev != MG_EV_HTTP_MSG) return;
  struct mg_http_message *hm = (struct mg_http_message *) ev_data;
  String uri = String(hm->uri.buf, hm->uri.len);

  // ---- /api?command=... ----
  if (uri == "/api") {
    String q = String(hm->query.buf, hm->query.len);
    String command = urlDecode(getQueryParam(q.c_str(), "command"));
    if (command.length()) processCommand(command);
    mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "ok");
    return;
  }
  if (uri == "/api/status")  { handleApiStatus(c);  return; }
  if (uri == "/api/time")    { handleApiTime(c);    return; }
  if (uri == "/api/weather") { handleApiWeather(c); return; }
  if (uri == "/api/logs")    { handleApiLogs(c);    return; }
  if (uri == "/api/melodies"){ handleApiMelodies(c);return; }

  // ---- /api/melodies/save (POST) ----
  if (uri == "/api/melodies/save") {
    int id = getPostParam(hm, "id").toInt() - 1;
    if (id >= 0 && id < 3) {
      melodies[id].name = getPostParam(hm, "name");
      melodies[id].rtttl = getPostParam(hm, "rtttl");
      saveMelodies();
      needPublishDiscovery = haDiscovery;
      addLog("Melody saved: " + String(id + 1));
    }
    mg_http_reply(c, 200, "Content-Type: application/json\r\n", "{\"ok\":true}");
    return;
  }

  // ===== Главная =====
  if (uri == "/") {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Clock</title>";
    html += "<style>"; html += COMMON_CSS; html += "</style>";
    html += "<script>"; html += COMMON_JS;
    html += "var WX_RGB={"
    "sun:[['#008000','#008000',0,0,0,'#ff0','#ff0','#ff0'],['#0b0','#0b0','#0b0',0,0,0,'#ff0','#ff0'],[0,'#8b5a2b',0,'#0b0',0,0,0,'#ff0'],[0,'#8b5a2b',0,0,0,0,0,0],[0,'#8b5a2b',0,0,0,0,0,0],['#fc0','#fc0','#fc0',0,'#08f','#08f','#08f','#08f'],['#fc0','#fc0','#fc0','#fc0','#08f','#08f','#08f','#08f'],['#fc0','#fc0','#fc0','#fc0','#fc0','#08f','#08f','#08f']],"
    "partly:[[0,0,'#ff0','#ff0',0,0,0,0],[0,'#ff0','#ff0','#ff0','#ff0','#444',0,0],['#ff0','#ff0','#ff0','#666','#fff','#fff','#666',0],['#666','#ccc','#fff','#fff','#fff','#fff','#fff','#666'],['#888','#ccc','#ccc','#ccc','#ccc','#ccc','#ccc','#888'],['#444','#888','#888','#888','#888','#888','#888','#444'],[0,'#444','#444','#ff0','#444','#444','#444',0],[0,0,'#ff0','#ff0',0,0,0,0]],"
    "cloud:[[0,0,0,0,0,'#444','#444',0],[0,'#444','#444',0,'#666','#fff','#fff','#666'],['#444','#ccc','#fff','#666','#fff','#fff','#fff','#fff'],['#888','#ccc','#ccc','#ccc','#ccc','#ccc','#ccc','#ccc'],['#444','#888','#888','#888','#888','#888','#888','#444'],[0,'#444','#444','#444','#444','#444','#444',0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]],"
    "fog:[['#777','#888','#888','#888','#888','#888','#888','#777'],[0,'#555','#666','#666','#666','#666','#555',0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]],"
    "rain:[[0,0,0,0,0,'#444','#444',0],[0,'#444','#444',0,'#666','#fff','#fff','#666'],['#444','#ccc','#fff','#666','#fff','#fff','#fff','#fff'],['#888','#ccc','#ccc','#ccc','#ccc','#ccc','#ccc','#ccc'],['#444','#888','#888','#888','#888','#888','#888','#444'],[0,'#04f','#444','#444','#04f','#444','#444',0],[0,'#04f',0,0,'#04f',0,0,0],[0,'#04f',0,0,'#04f',0,0,0]],"
    "snow:[[0,'#06f',0,0,0,0,0,0],[0,0,0,0,0,'#39f',0,0],[0,0,0,0,'#39f','#39f','#39f',0],[0,0,0,0,0,'#39f',0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,'#06f',0,0,0,0,0,0],[0,'#06f',0,0,0,0,0,0]],"
    "thunder:[[0,0,0,0,0,'#444','#444',0],[0,'#444','#444',0,'#666','#fff','#fff','#666'],['#444','#ccc','#fff','#666','#fff','#fff','#fff','#fff'],['#888','#ccc','#ccc','#ccc','#ccc','#ccc','#ccc','#ccc'],['#444','#888','#888','#888','#888','#888','#888','#444'],[0,'#444','#444','#444','#444','#444','#444',0],[0,0,0,'#ff0',0,0,0,0],[0,'#03c',0,0,'#ff0',0,0,0]],"
    "unknown:[[0,0,'#888','#888','#888',0,0,0],[0,'#888',0,0,'#888',0,0,0],[0,0,0,0,'#888',0,0,0],[0,0,0,'#888',0,0,0,0],[0,0,0,'#888',0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,'#888',0,0,0,0],[0,0,0,'#888',0,0,0,0]]"
    "};"
    "function wxIconKey(code){"
    "if(code===113)return 'sun';if(code===116)return 'partly';"
    "if(code===119||code===122)return 'cloud';"
    "if(code===143||code===248||code===260)return 'fog';"
    "if(code===200||code===386||code===389||code===392||code===395)return 'thunder';"
    "if(code===179||code===182||code===185||code===227||code===230||"
    "code===281||code===284||code===311||code===314||"
    "code===317||code===320||code===323||code===326||"
    "code===329||code===332||code===335||code===338||"
    "code===350||code===362||code===365||code===368||"
    "code===371||code===374||code===377)return 'snow';"
    "if(code===176||code===263||code===266||code===293||"
    "code===296||code===299||code===302||code===305||"
    "code===308||code===353||code===356||code===359)return 'rain';"
    "return 'unknown';}"
    "var _wxAnim={code:0,color:'#888',running:false};"
    "function drawWxIcon(code,color){_wxAnim.code=code||0;_wxAnim.color=color||'#888';if(!_wxAnim.running){_wxAnim.running=true;requestAnimationFrame(wxAnimTick);}}"
    "function wxAnimTick(t){"
    "var cnv=document.getElementById('wx-icon');if(!cnv){_wxAnim.running=false;return;}"
    "var ctx=cnv.getContext('2d');ctx.clearRect(0,0,8,8);"
    "var icon=WX_RGB[wxIconKey(_wxAnim.code)]||WX_RGB.unknown;"
    "var phase=t/1000;var dx=0,dy=0;"
    "if(_wxAnim.code===119||_wxAnim.code===122||_wxAnim.code===116){"
    "dx=Math.round(Math.sin(phase*1.2));"
    "dy=Math.round(Math.sin(phase*0.9)*0.6);"
    "}"
    "for(var r=0;r<8;r++){for(var c=0;c<8;c++){var col=icon[r][c];if(!col)continue;"
    "ctx.fillStyle=col;ctx.fillRect(c+dx,r+dy,1,1);}}"
    "requestAnimationFrame(wxAnimTick);}"
    "function loadStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{"
    "var h=document.getElementById('holidays');if(!d.holiday_valid||!d.holidays||!d.holidays.length){h.innerHTML='<span style=\"color:#888\">Нет праздников</span>';}else{var x='<div style=\"color:#666;font-size:12px\">'+esc(d.holiday_date)+' · '+d.holiday_count+'</div><ul>';for(var i=0;i<d.holidays.length;i++){x+='<li><b>🎉 '+esc(d.holidays[i])+'</b> <span style=\"color:#888;font-size:11px\">('+esc(d.holiday_categories[i])+')</span></li>'}x+='</ul>';h.innerHTML=x;}"
    "var s=document.getElementById('sensor');if(!d.sensor_type||d.sensor_type==='None'){s.innerHTML='<span style=\"color:#888\">Датчик не подключён</span>';}else if(!d.sensor_ok){s.innerHTML='<span style=\"color:#c0392b\">'+esc(d.sensor_type)+': ошибка</span>';}else{var p=[];if(d.sensor_has_temp)p.push('<b>'+Number(d.sensor_temp).toFixed(1)+' °C</b>');if(d.sensor_has_hum)p.push('влажность <b>'+Number(d.sensor_hum).toFixed(0)+'%</b>');if(d.sensor_has_press)p.push('давление <b>'+Number(d.sensor_press).toFixed(0)+' hPa</b>');s.innerHTML=esc(d.sensor_type)+': '+p.join(', ');}"
    "var bEl=document.getElementById('bright-info');if(bEl){var bm={sensor:'по датчику',schedule:'расписание',manual:'вручную'}[d.brightness_mode]||d.brightness_mode;bEl.innerHTML='Режим: <b>'+bm+'</b> · Текущая яркость: <b>'+d.brightness+'</b> из '+d.max_brightness;if(d.is_night){bEl.innerHTML+=' <span class=\"badge night\">🌙 ночь</span>';}}"
    "}).catch(()=>{})}"
    "function loadWeather(){fetch('/api/weather').then(r=>r.json()).then(d=>{var e=document.getElementById('weather-info');if(!d.valid){e.innerHTML=d.error?('⚠ '+esc(d.error)):'Нет данных';drawWxIcon(0,'#666');return;}drawWxIcon(d.code,d.color);e.innerHTML='<b>'+esc(d.city)+'</b> — '+Number(d.temp_c).toFixed(1)+' °C<br><small style=\"color:#666\">'+esc(d.desc_ru)+' · влажность '+d.humidity+'%</small>';}).catch(()=>{})}"
    "window.onload=function(){loadStatus();refreshTime();loadLogs();loadWeather();setInterval(refreshTime,60000);setInterval(loadStatus,5000);setInterval(loadLogs,3000);setInterval(loadWeather,60000);};";
    html += "</script></head><body>";
    html += navHeader("home");
    html += "<h2>Локальное время: <span id='time'>—</span></h2>";
    html += "<div class='section' id='bright-info' style='font-family:monospace'></div>";
    html += "<h3>Погода</h3><div class='section wx-card'><canvas id='wx-icon' width='8' height='8'></canvas><div id='weather-info'>Загрузка…</div></div>";
    html += "<h3>Праздники сегодня</h3><div class='section'><div id='holidays'>Загрузка…</div></div>";
    html += "<h3>Датчик</h3><div class='section'><div id='sensor'>Загрузка…</div></div>";
    html += "<h3>Журнал</h3><div id='log'></div>";
    html += "</body></html>";
    mg_http_reply(c, 200, "Content-Type: text/html; charset=utf-8\r\n", "%s", html.c_str());
    return;
  }

  // ===== Мелодии =====
  if (uri == "/melodies") {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Мелодии</title>";
    html += "<style>"; html += COMMON_CSS; html += "</style>";
    html += "<script>"; html += COMMON_JS;
    html += "var actx=null;"
    "function getCtx(){if(!actx)actx=new (window.AudioContext||window.webkitAudioContext)();return actx}"
    "function splitRtttl(full){var s=String(full||'').trim();var c1=s.indexOf(':');if(c1<0)return{name:'',body:s};"
    "var head=s.substring(0,c1).trim();var body=s.substring(c1+1).trim();"
    "if(/^[dob]=/i.test(head)||head.indexOf(',')>=0||/^[dob]=/i.test(s))return{name:'',body:s};"
    "return{name:head,body:body};}"
    "function buildFull(id){var n=document.getElementById('mname'+id).value.trim();"
    "var b=document.getElementById('mrtttl'+id).value.trim();if(!n)n='Melody '+id;"
    "return n+':'+b;}"
    "function extractRtttlName(id){var ta=document.getElementById('mrtttl'+id);var nEl=document.getElementById('mname'+id);"
    "if(!ta||!nEl)return;var v=ta.value;var m=v.match(/^\\s*([^:]+):/);if(!m)return;"
    "var cand=m[1].trim();if(/^[dob]=/i.test(cand)||cand.indexOf(',')>=0)return;"
    "if(!nEl.value.trim()){nEl.value=cand;ta.value=v.substring(m[0].length);}}"
    "function playWeb(id){var r=buildFull(id);var m=r.split(':');if(m.length<3){alert('Неверный формат RTTTL (нужно header:notes)');return}"
    "var hdr=m[1],notes=m.slice(2).join(':').split(',');var defDur=4,defOct=5,bpm=100;"
    "hdr.split(',').forEach(function(p){p=p.trim().toLowerCase();if(p.startsWith('d='))defDur=+p.slice(2);else if(p.startsWith('o='))defOct=+p.slice(2);else if(p.startsWith('b='))bpm=+p.slice(2)});"
    "var ctx=getCtx();var t=ctx.currentTime+0.1;var names={c:0,d:2,e:4,f:5,g:7,a:9,b:11};"
    "notes.forEach(function(n){n=n.trim().toLowerCase();if(!n)return;"
    "var mi=n.match(/^(\\d*)([a-gp])(#?)(\\.?)(\\d*)$/);if(!mi)return;"
    "var dur=mi[1]?+mi[1]:defDur;var isP=mi[2]==='p';var sharp=!!mi[3];var dot=!!mi[4];var oct=mi[5]?+mi[5]:defOct;"
    "var semis=names[mi[2]]+(sharp?1:0);var freq=440*Math.pow(2,(oct-4)+(semis-9)/12);"
    "var ms=(4/dur)*(60000/bpm);if(dot)ms*=1.5;"
    "if(!isP&&freq>20&&freq<20000){var o=ctx.createOscillator();var g=ctx.createGain();o.frequency.value=freq;o.type='square';"
    "g.gain.setValueAtTime(0.0001,t);g.gain.exponentialRampToValueAtTime(0.2,t+0.01);g.gain.exponentialRampToValueAtTime(0.0001,t+ms/1000-0.02);"
    "o.connect(g).connect(ctx.destination);o.start(t);o.stop(t+ms/1000);}t+=ms/1000;});}"
    "function loadMelodies(){fetch('/api/melodies').then(r=>r.json()).then(d=>{"
    "var el=document.getElementById('melCards');var h='';"
    "for(var i=0;i<d.melodies.length;i++){var m=d.melodies[i];"
    "var sp=splitRtttl(m.rtttl||'');var name=m.name||sp.name||'';var body=sp.body||'';"
    "h+=\"<div class='section'>\";"
    "h+=\"<div class='mel-name-row'><label>Мелодия \"+m.id+\":</label><input type='text' id='mname\"+m.id+\"' value=\\\"\"+esc(name)+\"\\\">\";"
    "h+=\"<button class='play' onclick='cmd(\\\"MELODY_PLAY:\"+m.id+\"\\\")'>▶ На устройстве</button>\";"
    "h+=\"<button class='web' onclick='playWeb(\"+m.id+\")'>♫ Играть здесь</button></div>\";"
    "h+=\"<textarea id='mrtttl\"+m.id+\"' rows='5' placeholder='d=4,o=5,b=120:c,e,g' oninput='extractRtttlName(\"+m.id+\")'>\"+esc(body)+\"</textarea>\";"
    "h+=\"</div>\";}"
    "el.innerHTML=h;"
    "var v=document.getElementById('vol');if(v){v.value=d.volume;document.getElementById('volV').textContent=d.volume;}"
    "}).catch(e=>{document.getElementById('melCards').innerText='Ошибка загрузки: '+e;})}"
    "function saveAllMelodies(){var tasks=[];"
    "for(var i=1;i<=3;i++){"
    "var n=document.getElementById('mname'+i).value.trim();"
    "var b=document.getElementById('mrtttl'+i).value.trim();"
    "if(!n)n='Melody '+i;"
    "var full=n+':'+b;"
    "tasks.push(fetch('/api/melodies/save',{method:'POST',"
    "headers:{'Content-Type':'application/x-www-form-urlencoded'},"
    "body:'id='+i+'&name='+encodeURIComponent(n)+'&rtttl='+encodeURIComponent(full)}));}"
    "Promise.all(tasks).then(function(){alert('Сохранено');loadMelodies();})"
    ".catch(function(){alert('Ошибка сохранения');});}"
    "window.onload=function(){loadMelodies();refreshTime();};";
    html += "</script></head><body>";
    html += navHeader("melodies");
    html += "<h2>Мелодии</h2>";
    html += "<div class='section'><label>Громкость: <span id='volV'>—</span>%</label><input type='range' id='vol' min='0' max='100' oninput=\"document.getElementById('volV').textContent=this.value\" onchange=\"cmd('BUZZER_VOLUME:'+this.value)\"></div>";
    html += "<div id='melCards'>Загрузка…</div>";
    html += "<div class='section'>";
    html += "<button class='primary' onclick='saveAllMelodies()'>💾 Сохранить все мелодии</button>";
    html += "<button onclick=\"cmd('MELODY_STOP')\">■ Стоп на устройстве</button>";
    html += "</div>";
    html += "</body></html>";
    mg_http_reply(c, 200, "Content-Type: text/html; charset=utf-8\r\n", "%s", html.c_str());
    return;
  }

  // ===== Настройки =====
  if (uri == "/settings") {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Настройки</title>";
    html += "<style>"; html += COMMON_CSS; html += "</style>";
    html += "<script>"; html += COMMON_JS;
    html += "function ct(c){return fetch('/api?command='+encodeURIComponent(c)).then(r=>r.text())}"
    "function updateVis(){var m=document.getElementById('brMode').value;"
    "document.getElementById('manualRow').style.display=(m==='manual')?'block':'none';"
    "document.getElementById('nightRow').style.display=(m==='schedule')?'block':'none';}"
    "function loadSettings(){fetch('/api/status').then(r=>r.json()).then(d=>{"
    "document.getElementById('ver').textContent=d.fw_version;document.getElementById('host').textContent=d.hostname;document.getElementById('ip').textContent=d.ip;document.getElementById('ram').textContent=d.free_heap;"
    "document.getElementById('devName').value=d.hostname||'';document.getElementById('sType').value=String(d.sensor_type_index||0);"
    "document.getElementById('brMode').value=d.brightness_mode||'sensor';"
    "document.getElementById('minB').value=d.min_brightness;document.getElementById('maxB').value=d.max_brightness;"
    "document.getElementById('ldrI').value=d.ldr_influence;document.getElementById('ldrIv').textContent=d.ldr_influence+'%';"
    "document.getElementById('manualB').value=d.manual_brightness;document.getElementById('manualBv').textContent=d.manual_brightness;"
    "document.getElementById('nightB').value=d.night_brightness;document.getElementById('nightBv').textContent=d.night_brightness;"
    "document.getElementById('nStart').value=d.night_start_hour;document.getElementById('nEnd').value=d.night_end_hour;"
    "document.getElementById('tz').value=d.timezone_hours;document.getElementById('ntp').value=d.ntp_server||'';"
    "document.getElementById('catRU').checked=!!d.holiday_cat_ru;document.getElementById('catProf').checked=!!d.holiday_cat_prof;document.getElementById('catIntl').checked=!!d.holiday_cat_intl;"
    "document.getElementById('catCorp').checked=!!d.holiday_cat_corp;document.getElementById('catEco').checked=!!d.holiday_cat_eco;document.getElementById('catMed').checked=!!d.holiday_cat_med;document.getElementById('catInfo').checked=!!d.holiday_cat_info;"
    "document.getElementById('wxM').value=String(d.weather_mode||0);document.getElementById('wxZ').value=d.weather_zip||'';"
    "document.getElementById('wxLat').value=d.weather_lat||0;document.getElementById('wxLon').value=d.weather_lon||0;document.getElementById('wxInt').value=d.weather_interval||600;"
    "document.getElementById('mqttE').checked=!!d.mqtt_enabled;document.getElementById('mqttS').value=d.mqtt_server||'';document.getElementById('mqttP').value=d.mqtt_port||1883;"
    "document.getElementById('mqttU').value=d.mqtt_user||'';document.getElementById('mqttX').value=d.mqtt_prefix||'clock';document.getElementById('mqttPw').value='';"
    "document.getElementById('haD').checked=!!d.ha_discovery;updateVis();"
    "}).catch(()=>{})}"
    "function save(){"
    "ct('HOSTNAME:'+document.getElementById('devName').value);"
    "ct('BRIGHT_MODE:'+document.getElementById('brMode').value);"
    "ct('MIN_BRIGHTNESS:'+document.getElementById('minB').value);"
    "ct('MAX_BRIGHTNESS:'+document.getElementById('maxB').value);"
    "ct('LDR_INFLUENCE:'+document.getElementById('ldrI').value);"
    "ct('MANUAL_BRIGHTNESS:'+document.getElementById('manualB').value);"
    "ct('NIGHT_BRIGHTNESS:'+document.getElementById('nightB').value);"
    "ct('NIGHT_START:'+document.getElementById('nStart').value);"
    "ct('NIGHT_END:'+document.getElementById('nEnd').value);"
    "ct('TIMEZONE:'+document.getElementById('tz').value);"
    "ct('NTP:'+document.getElementById('ntp').value);"
    "ct('HOLIDAY_CAT_RU:'+(document.getElementById('catRU').checked?1:0));"
    "ct('HOLIDAY_CAT_PROF:'+(document.getElementById('catProf').checked?1:0));"
    "ct('HOLIDAY_CAT_INTL:'+(document.getElementById('catIntl').checked?1:0));"
    "ct('HOLIDAY_CAT_CORP:'+(document.getElementById('catCorp').checked?1:0));"
    "ct('HOLIDAY_CAT_ECO:'+(document.getElementById('catEco').checked?1:0));"
    "ct('HOLIDAY_CAT_MED:'+(document.getElementById('catMed').checked?1:0));"
    "ct('HOLIDAY_CAT_INFO:'+(document.getElementById('catInfo').checked?1:0));"
    "ct('WEATHER_MODE:'+document.getElementById('wxM').value);"
    "ct('WEATHER_ZIP:'+document.getElementById('wxZ').value);"
    "ct('WEATHER_LAT:'+document.getElementById('wxLat').value);"
    "ct('WEATHER_LON:'+document.getElementById('wxLon').value);"
    "ct('WEATHER_INTERVAL:'+document.getElementById('wxInt').value);"
    "ct('MQTT_ENABLE:'+(document.getElementById('mqttE').checked?1:0));"
    "ct('MQTT_SERVER:'+document.getElementById('mqttS').value);"
    "ct('MQTT_PORT:'+document.getElementById('mqttP').value);"
    "ct('MQTT_USER:'+document.getElementById('mqttU').value);"
    "var pw=document.getElementById('mqttPw').value;if(pw)ct('MQTT_PASSWORD:'+pw);"
    "ct('MQTT_PREFIX:'+document.getElementById('mqttX').value);"
    "ct('HA_DISCOVERY:'+(document.getElementById('haD').checked?1:0));"
    "ct('SENSOR_TYPE:'+document.getElementById('sType').value);"
    "alert('Сохранено');setTimeout(function(){location.reload()},800);}"
    "window.onload=function(){loadSettings();refreshTime();setInterval(refreshTime,1000);};";
    html += "</script></head><body>";
    html += navHeader("settings");
    html += "<h2>Настройки</h2>";

    html += "<h3>Устройство</h3><div class='section'>";
    html += "<div class='info'>FW: <span id='ver'>—</span> · Host: <span id='host'>—</span> · IP: <span id='ip'>—</span> · RAM: <span id='ram'>—</span></div>";
    html += "<label>Имя устройства</label><input type='text' id='devName'><br>";
    html += "<label>Тип датчика</label><select id='sType'><option value='0'>Нет</option><option value='1'>BME280</option><option value='2'>BMP280</option><option value='3'>AHT20+BMP280</option></select><br>";
    html += "</div>";

    html += "<h3>Яркость</h3><div class='section'>";
    html += "<label>Режим</label><select id='brMode' onchange='updateVis()'><option value='sensor'>По датчику (LDR)</option><option value='schedule'>Расписание (ночь)</option><option value='manual'>Вручную</option></select><br>";
    html += "<div class='mode-hint'>«По датчику» — яркость меняется от освещённости. «Расписание» — как по датчику, но ночью фиксированная ночная яркость. «Вручную» — всегда фиксированная ручная яркость.</div>";
    html += "<div id='manualRow' style='display:none'><label>Ручная яркость</label><input type='range' id='manualB' min='1' max='255' oninput=\"document.getElementById('manualBv').textContent=this.value\"> <span id='manualBv'>—</span><br></div>";
    html += "<div id='nightRow' style='display:none'><label>Ночная яркость</label><input type='range' id='nightB' min='1' max='255' oninput=\"document.getElementById('nightBv').textContent=this.value\"> <span id='nightBv'>—</span><br>";
    html += "<label>Ночь с (час)</label><input type='number' id='nStart' min='0' max='23'><br>";
    html += "<label>Ночь до (час)</label><input type='number' id='nEnd' min='0' max='23'><br></div>";
    html += "<label>Мин. яркость</label><input type='number' id='minB' min='1' max='255'><br>";
    html += "<label>Макс. яркость</label><input type='number' id='maxB' min='1' max='255'><br>";
    html += "<label>Влияние LDR <span id='ldrIv'>—</span></label><input type='range' id='ldrI' min='0' max='100' oninput=\"document.getElementById('ldrIv').textContent=this.value+'%'\"><br>";
    html += "</div>";

    html += "<h3>Время</h3><div class='section'>";
    html += "<label>NTP сервер</label><input type='text' id='ntp'><br>";
    html += "<label>Часовой пояс (часы)</label><input type='number' id='tz' step='0.5' min='-12' max='14'><br>";
    html += "</div>";

    html += "<h3>Праздники</h3><div class='section cat-row'>";
    html += "<label><input type='checkbox' id='catRU'> Праздники России</label>";
    html += "<label><input type='checkbox' id='catProf'> Профессиональные</label>";
    html += "<label><input type='checkbox' id='catIntl'> Международные</label>";
    html += "<label><input type='checkbox' id='catCorp'> Корпоративные</label>";
    html += "<label><input type='checkbox' id='catEco'> Экологические</label>";
    html += "<label><input type='checkbox' id='catMed'> Медицинские</label>";
    html += "<label><input type='checkbox' id='catInfo'> Неофициальные</label>";
    html += "</div>";

    html += "<h3>Погода</h3><div class='section'>";
    html += "<label>Режим</label><select id='wxM'><option value='0'>Авто (по IP)</option><option value='1'>Индекс (zip)</option><option value='2'>Координаты</option></select><br>";
    html += "<label>Индекс/zip</label><input type='text' id='wxZ'><br>";
    html += "<label>Широта</label><input type='number' id='wxLat' step='0.0001'><br>";
    html += "<label>Долгота</label><input type='number' id='wxLon' step='0.0001'><br>";
    html += "<label>Интервал (сек)</label><input type='number' id='wxInt' min='60'><br>";
    html += "</div>";

    html += "<h3>MQTT</h3><div class='section'>";
    html += "<label><input type='checkbox' id='mqttE'> Включить MQTT</label><br>";
    html += "<label>Сервер</label><input type='text' id='mqttS'><br>";
    html += "<label>Порт</label><input type='number' id='mqttP' min='1' max='65535'><br>";
    html += "<label>Пользователь</label><input type='text' id='mqttU'><br>";
    html += "<label>Пароль (пусто — не менять)</label><input type='password' id='mqttPw'><br>";
    html += "<label>Префикс топиков</label><input type='text' id='mqttX'><br>";
    html += "<label><input type='checkbox' id='haD'> HA Discovery</label><br>";
    html += "</div>";

    html += "<div class='section'>";
    html += "<button class='primary' onclick='save()'>Сохранить</button>";
    html += "<button onclick=\"cmd('NTP_SYNC')\">NTP синхронизация</button>";
    html += "<button onclick=\"cmd('WEATHER_REFRESH')\">Обновить погоду</button>";
    html += "<button onclick=\"cmd('HOLIDAY_REFRESH')\">Обновить праздники</button>";
    html += "<button onclick=\"if(confirm('Перезагрузить?'))cmd('REBOOT')\">Перезагрузить</button>";
    html += "<button class='danger' onclick=\"if(confirm('Сбросить настройки?'))cmd('FACTORY_RESET')\">Сброс</button>";
    html += "</div>";

    html += "</body></html>";
    mg_http_reply(c, 200, "Content-Type: text/html; charset=utf-8\r\n", "%s", html.c_str());
    return;
  }

  mg_http_reply(c, 404, "Content-Type: text/plain\r\n", "Not found");
}


// ============================================================
// ТЕСТОВАЯ МЕЛОДИЯ (временная, удалить после проверки)
// Играет "Frère Jacques" напрямую через ledcWriteTone + delay.
// Не использует RTTTL, startMelody() или melodyTick().
// ============================================================
void playTestMelody() {
  struct Note { uint16_t freq; uint16_t dur; };
  static const Note notes[] = {
    // Frère Jacques (C D E C | C D E C | E F G | E F G | ...)
    {262,300},{294,300},{330,300},{262,300},
    {262,300},{294,300},{330,300},{262,300},
    {330,300},{349,300},{392,600},
    {330,300},{349,300},{392,600},
    {392,150},{440,150},{392,150},{349,150},{330,300},{262,300},
    {392,150},{440,150},{392,150},{349,150},{330,300},{262,300},
    {262,300},{196,300},{262,600},
    {262,300},{196,300},{262,600},
  };
  const size_t n = sizeof(notes) / sizeof(notes[0]);

  Serial.printf("MELODY TEST: %u notes, GPIO%d\n", (unsigned)n, BUZZER_PIN);

  const uint32_t maxDuty = (1u << BUZZER_PWM_RES) - 1u;
  const uint32_t duty50 = maxDuty / 2u;  // 50% — как в ваших рабочих тестах

  for (size_t i = 0; i < n; i++) {
    ledcWriteTone(BUZZER_PIN, notes[i].freq);
    ledcWrite(BUZZER_PIN, duty50);
    delay(notes[i].dur);
    ledcWrite(BUZZER_PIN, 0);
    delay(30);
  }
  ledcWrite(BUZZER_PIN, 0);
  Serial.println(F("MELODY TEST: done"));
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("ESP32-C3 Clock v" FIRMWARE_VERSION);  

  loadSettings();
  loadMelodies();

  FastLED.addLeds<WS2812B, MATRIX_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(maxBrightness);
  FastLED.clear(true);

Serial.println(F("=== LEDC TEST ==="));

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // Пробуем разные каналы, пока не заработает
  ledcDetach(BUZZER_PIN);
  bool ok = ledcAttachChannel(BUZZER_PIN, 2000, BUZZER_PWM_RES, 0);  // канал 0
  Serial.printf("ledcAttachChannel(0): %s\n", ok ? "OK" : "FAIL");
#endif

/*ledcWriteTone(BUZZER_PIN, 440);
delay(3000);
ledcWrite(BUZZER_PIN, 0);*/

/*delay(1000);
//ledcAttach(BUZZER_PIN, 1000, 10);   // 1 кГц
ledcWrite(BUZZER_PIN, 512);          // 50% duty → ~1.6 В среднего
delay(3000);
ledcWrite(BUZZER_PIN, 0);*/

playTestMelody();


  pinMode(LDR_PIN, INPUT);

  connectToWiFi();
  if (!wifiConnected) startAPMode();

  syncTime();
  initSensor();

  mqttClientId = "clock-" + WiFi.macAddress();
  mqttClientId.replace(":", "");
  topicState = mqttPrefix + "/state";
  topicSet   = mqttPrefix + "/set";
  topicAvail = mqttPrefix + "/avail";

  mg_mgr_init(&mgr);
  mg_log_set(MG_LL_ERROR);
  mg_http_listen(&mgr, "http://0.0.0.0:80", httpEventHandler, nullptr);
  addLog("HTTP listen :80");

  if (wifiConnected) {
    if (MDNS.begin(deviceName.c_str())) MDNS.addService("http", "tcp", 80);
    setupOTA();
  }

  displayModeStartAt = millis();
  needRecalcHolidays = true;

 /* Serial.printf("BUZZER test on GPIO%d\n", BUZZER_PIN);
  BZ_TONE(1000); BZ_WRITE(512); delay(500); BZ_WRITE(0);*/
}

void loop() {
  if (rebootPending && (long)(millis() - rebootAt) >= 0) {
    addLog("Rebooting…");
    delay(50);
    ESP.restart();
  }
  if (otaInProgress) {
    mg_mgr_poll(&mgr, 10);
    return;
  }

  mg_mgr_poll(&mgr, 5);
  ArduinoOTA.handle();
  melodyTick(); 
  checkWiFiConnection();

  if (WiFi.getMode() == WIFI_MODE_AP) {
    updateApModeDisplay();
    return;
  }

  unsigned long now = millis();

  static unsigned long lastBright = 0, lastMel = 0;
  if (now - lastBright >= 1000) {
    lastBright = now;
    readLDR();
    applyBrightness();
    struct tm ti;
    if (getLocalTime(&ti, 10)) {
      static int lastDay = -1;
      if (ti.tm_mday != lastDay) {
        lastDay = ti.tm_mday;
        needRecalcHolidays = true;
      }
    }
  }
  /*if (now - lastMel >= 10) {
    lastMel = now;
    melodyTick();
  }*/

  readSensor();
  updateDisplay();

  if (!wifiConnected) return;

  if (needRecalcHolidays && lastNtpSync != 0) {
    calculateHolidays();
    needRecalcHolidays = false;
  }
  if (lastNtpSync == 0 || now - lastNtpSync >= NTP_SYNC_INTERVAL) {
    syncTime();
  }
  if (wxLastFetch == 0 ||
      now - wxLastFetch >= (unsigned long)weatherInterval * 1000UL) {
    fetchWeather();
  }

  connectMQTT();
  if (mqttConnected) {
    if (needPublishDiscovery) {
      publishDiscovery();
      needPublishDiscovery = false;
    }
    if (needPublishState || now - lastStatePublish >= STATE_PUBLISH_INTERVAL) {
      publishState();
      needPublishState = false;
    }
  }
}
