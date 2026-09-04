/*
 * ============================================================================
 *  Environmental Stationary Logger
 * ============================================================================
 *
 *  SYNOPSIS
 *  --------
 *  Firmware for an ESP32 Wrover-E based stationary environmental monitoring
 *  station.  The device continuously samples a range of physical parameters,
 *  exposes them over a local HTTP server, and uploads a summary to two public
 *  radiation-monitoring platforms every 61 seconds.
 *
 *  HARDWARE
 *  --------
 *  MCU       : ESP32 Wrover-E (dual-core, 240 MHz)
 *  Sensors   :
 *    - BME680 / BSEC    — temperature, pressure, humidity, IAQ, CO2eq, VOC (I2C 0x76)
 *    - HM3301           — PM1.0 / PM2.5 / PM10 particulate matter        (I2C)
 *    - TSL2561          — visible-light luminosity                        (I2C)
 *    - Grove HCHO       — formaldehyde (HCHO) analogue sensor             (GPIO34 / ADC)
 *    - 2x SBM-19 GM tubes — ionising radiation via ESP32 PCNT hardware   (GPIO13 / GPIO14)
 *    - ADC voltage divider — GM tube high-voltage monitor                 (GPIO33 / ADC)
 *
 *  ARCHITECTURE
 *  ------------
 *  Core 0 (Arduino loop):
 *    - Every  1 s : read both PCNT counters, correct for dead-time, feed
 *                   three movingAvg windows (combined, tube1, tube2), read
 *                   BSEC / TSL2561 / HCHO.
 *    - Every 31 s : read HM3301 PM sensor (BSEC needs ~30 s warm-up).
 *    - Every 61 s : snapshot all globals into an UploadSnapshot struct,
 *                   spawn the upload FreeRTOS task on Core 1, reset the
 *                   upload cycle timing, and keep the moving averages rolling.
 *    - On demand  : serve HTTP responses for "/" (HTML dashboard) and
 *                   "/json" (JSON payload).
 *
 *  Core 1 (FreeRTOS tasks):
 *    - uploadTask      : POST CPM + all environmental values to radmon.org
 *                        and data.uradmonitor.com using the UploadSnapshot
 *                        copy; frees the snapshot on completion.
 *    - updateState     : persist BSEC calibration state to EEPROM once IAQ
 *                        accuracy reaches 3, then at each STATE_SAVE_PERIOD.
 *
 *  RADIATION MEASUREMENT
 *  ---------------------
 *  Both tubes are counted via ESP32 PCNT hardware counters (Units 0 and 1).
 *  Each second:
 *    1. Raw counts are read and the hardware counters are immediately
 *       paused, cleared, and resumed (per-second window).
 *    2. Dead-time correction: actual_cps = raw_cps / (1 - raw_cps * deadTime)
 *    3. Corrected CPS is fed into three 120-/60-/60-point movingAvg windows.
 *    4. CPM = average_two_tube_CPS x 60
 *    5. Dose rate (µSv/h) = CPM x tubeConversionFactor (0.0015 for SBM-19)
 *
 *  UPLOAD PLATFORMS
 *  ----------------
 *  radmon.org         — CPM value via HTTP POST with query-string URL parameters (plain HTTP)
 *  data.uradmonitor.com — full environmental payload via HTTP POST to
 *                         /api/v1/upload/exp with X-User-id / X-User-hash /
 *                         X-Device-id headers (plain HTTP)
 *  NOTE: Both endpoints currently use plain HTTP (no TLS).  Credentials are
 *        stored in arduino_secrets.h and never hard-coded in this file.
 *
 *  FILES
 *  -----
 *  Environmental_Stationary_Logger_V1.2.ino  — this file
 *  arduino_secrets.h                         — WiFi / API credentials (gitignored)
 *  bsec_iaq.h                                — BSEC binary config (3.3 V, 3s LP, 4d)
 *  src/Digital_Light_TSL2561.h/.cpp          — local TSL2561 driver
 *
 *  VERSION HISTORY
 *  ---------------
 *  See FIRMWARE_CHANGELOG constant for the full version history.
 *  The constant is the single source of truth — it is used verbatim
 *  in the web changelog dialog, so editing it once updates both the
 *  source code and the runtime UI simultaneously.
 *
 *  AUTHOR
 *  ------
 *  Don Zalmrol
 *
 * ============================================================================
 */

// Include libraries
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <ArduinoHttpClient.h>
#include <esp_heap_caps.h>
#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <NTP.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <movingAvg.h>
#include <bsec.h>
#include <driver/pulse_cnt.h>
#include <Tomoto_HM330X.h>
#include <ArduinoJson.h>
//#include <esp32/clk.h>
#include <vector>

//#include <Digital_Light_TSL2561.h>
#include "src/Digital_Light_TSL2561.h"

// Include optional files
#include "arduino_secrets.h"
#include "logger_user_config.h"

// Configure the BSEC library with information about the sensor : 3.3V, 3S OPmode, 4D age of the sensor
const uint8_t bsec_config_iaq[] =
{ 
  #include "bsec_iaq.h"
};

// Defines
// Set 75 seconds watchdog timer
#define WDT_TIMEOUT 75

// Give Radmon a little more time than ArduinoHttpClient's 30 s default.
#define RADMON_HTTP_TIMEOUT_MS 45000UL

// Let the scheduler and idle task run briefly each loop iteration.
// PCNT keeps counting pulses in hardware, so a 1 ms cooperative pause is safe here.
#define LOOP_IDLE_DELAY_MS 1UL

#define FIRMWARE_VERSION "v1.4d"

// Single source of truth for the version history — used verbatim in the
// web changelog dialog so source and UI are always in sync.
static const char* const FIRMWARE_CHANGELOG =
  "v1.4d  (2026-04-19)  Dashboard & telemetry expansion:\n"
  "  - Per-tube CPM cards (Tube 1 CPM, Tube 2 CPM) on dashboard.\n"
  "  - Live CPM chart: combined + per-tube CPM.\n"
  "  - New live CPS chart: Tube 1, Tube 2, moving average.\n"
  "  - Live TH chart: pressure added on right Y-axis.\n"
  "  - CSV history: 16 columns (cps_tube1, cps_tube2,\n"
  "    pressure_hpa_x10, voc_ppm_x100). Pressure bug fixed\n"
  "    (was Pa x10, now correctly hPa x10).\n"
  "  - /graphs: Tube CPS on Radiation chart; VOC on Env chart;\n"
  "    pressure on TH chart (dual Y-axis).\n"
  "  - parseCsv backward-compatible: 12 / 14 / 16 columns.\n"
  "  - JSON: cpm1, cpm2 fields added.\n"
  "  - FIRMWARE_VERSION constant; footer version badge opens\n"
  "    a native changelog dialog on click.\n"
  "  - Radiation favicon (SVG data URI) on all pages.\n"
  "  - Page title: hostname : Environmental Logger.\n"
  "  - OTA/JSON nav links shown only on /config page.\n"
  "  - Theme picker moved into topbar nav (all pages).\n"
  "  - Footer icon: inline SVG DZ badge (offline-capable).\n\n"
  "v1.4c  (2026-04-18)  Local history logging + end-user config:\n"
  "  - Rolling CSV history on SPIFFS (61-second cadence).\n"
  "  - /graphs page renders retained CSV via Chart.js.\n"
  "  - Automatic CSV pruning keeps storage bounded.\n"
  "  - pruneHistoryLogIfNeeded(): O(n) scan replaced with\n"
  "    O(1) historyRowCount counter (init from SPIFFS at boot).\n"
  "  - All settings on /config with NVS persistence; no\n"
  "    firmware rebuild needed to reconfigure the device.\n"
  "  - Tube preset library: 9 built-in presets + custom.\n"
  "  - DST profiles: EU, US, AU, None (runtime selectable).\n"
  "  - Configurable: NTP server, station name, CPM gauge\n"
  "    full-scale, history retention, upload enable/disable.\n"
  "  - Footer on all pages with author, links, dynamic year.\n\n"
  "v1.4b  (2026-04-18)  Reliability + diagnostics pass:\n"
  "  - Graceful HM330X/PCNT failure handling.\n"
  "  - Upload status from locked snapshot (no torn reads).\n"
  "  - Dead-time saturation: warning + raw CPS fallback.\n"
  "  - Per-core CPU load added to dashboard and /json.\n"
  "  - /json unixTime now reports current NTP epoch.\n\n"
  "v1.4a  (2026-03-20)  Cleanup / correctness pass:\n"
  "  - BSEC save cadence uses elapsed time (not counter).\n"
  "  - 1 ms cooperative loop delay added.\n"
  "  - Tube dead time + conversion factor in JSON payload.\n"
  "  - HV upload helper renamed for clarity.\n\n"
  "v1.4   (2026-03-20)  PCNT driver migration:\n"
  "  - Migrated from deprecated driver/pcnt.h to\n"
  "    modern driver/pulse_cnt.h (ESP-IDF API).\n\n"
  "v1.3   (2026-03-19)  Dashboard overhaul:\n"
  "  - Dark-themed card-grid UI with gauge bars.\n"
  "  - Six live Chart.js graphs via /json polling.\n"
  "  - Streaming HTTP response (webPageChunks).\n"
  "  - Upload status badges (green/red).\n\n"
  "v1.2   (2026-03-18)  20+ bug-fix / hardening items:\n"
  "  - BSEC, PCNT, FreeRTOS task safety, JSON, dead-time,\n"
  "    ADC 12-bit correction, upload race condition fixes.\n\n"
  "v1.1   (2024-xx-xx)  HM3301 PM, TSL2561, Grove HCHO added.\n"
  "  - Hardware PCNT counters, ArduinoJson v7.\n"
  "  - uradmonitor.com upload added.\n\n"
  "v1.0   (2022-09-20)  Initial release: Geiger + BME680 + radmon upload.";

// Compact rolling CSV history stored on SPIFFS for the separate Graphs page.
// Keep a configurable recent history window: one sample is written every 61 seconds.
static const char *const HISTORY_LOG_PATH = "/history_recent.csv";
static const char *const HISTORY_LOG_TMP_PATH = "/history_recent.tmp";
static const char *const HISTORY_LOG_HEADER = "epoch,cpm,cps_tube1,cps_tube2,temp_c_x10,humidity_pct_x10,pressure_hpa_x10,iaq_x10,co2_ppm,voc_ppm_x100,pm01_ugm3,pm25_ugm3,pm10_ugm3,hv_v_x10,luminosity_lux,hcho_ppb";
static const size_t HISTORY_RETENTION_SAMPLES_PER_HOUR = 60U;
static const size_t DEFAULT_HISTORY_RETENTION_HOURS = 1U;
static const size_t MAX_HISTORY_RETENTION_HOURS = 24U;
static const size_t DEFAULT_HISTORY_RETENTION_SAMPLES = 60U;

static const char *const CUSTOM_TUBE_PRESET_ID = "custom";
static const char *const CUSTOM_TUBE_PRESET_LABEL = "Custom";
static const char *const DST_PROFILE_NONE_ID = "none";
static const char *const DST_PROFILE_EU_ID = "eu";
static const char *const DST_PROFILE_US_ID = "us";
static const char *const DST_PROFILE_AU_ID = "au";
static const int DEFAULT_TIMEZONE_OFFSET_MINUTES = 60;
static const int DEFAULT_DST_OFFSET_MINUTES = 60;
static const char *const DEFAULT_DST_PROFILE_ID = DST_PROFILE_EU_ID;
static const float DEFAULT_HV_CALIBRATION_FACTOR = 184.097f;
static const float DEFAULT_HCHO_R0 = 10.37f;
static const char *const DEFAULT_NTP_SERVER = "pool.ntp.org";
static const long DEFAULT_CPM_GAUGE_FULL_SCALE = 600L;
static const char *const DEFAULT_STATION_NAME = "Station Dashboard";
static const int UPLOAD_STATUS_DISABLED = -10;
static const int UPLOAD_STATUS_MISSING_CONFIG = -11;

// 360 minutes (6 hours) = saving 4 times a day
//#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000) 
const unsigned long STATE_SAVE_PERIOD = (1UL * 60UL * 1000UL);

// Snapshot struct passed to the updateState FreeRTOS task to avoid Core 0/1 race on BSEC globals
struct StateSnapshot {
  int   iaqAccuracy;
  float iaq;
};

// Snapshot struct passed to the upload FreeRTOS task to avoid race conditions on globals
struct UploadSnapshot {
  unsigned long cpm;
  unsigned long actual_cps_1;
  unsigned long actual_cps_2;
  float tubeVoltage;
  time_t epoch;
  float var_temperature;
  float var_humidity;
  float var_pressure;
  int luminosity;
  float var_voc;
  float var_co2;
  float var_hcho;
  int var_pm01;
  int var_pm25;
  int var_pm10;
};

struct UploadRecord {
  bool valid;
  unsigned long cpm;
  unsigned long actual_cps_1;
  unsigned long actual_cps_2;
  float tubeVoltage;
  time_t epoch;
  float var_temperature;
  float var_humidity;
  float var_pressure;
  int luminosity;
  float var_voc;
  float var_co2;
  float var_hcho;
  int var_pm01;
  int var_pm25;
  int var_pm10;
};

struct UploadStatusSnapshot {
  int statusCodeRadmon;
  int statusCodeURadmon;
  UploadRecord radmon;
  UploadRecord uradmon;
};

// Helper functions declarations
static void WiFiSetup(void);
static uint64_t getUptimeSeconds(void);
static String formatUptime(uint64_t uptimeSeconds);
float outputSieverts(float cpmValue);
float displayTubeVoltage(void);
float calculateHCHO(void);
static void uploadTaskFunction(void *parameter);
static void connectToRadMonLogger(UploadSnapshot *snap);
static void connectToURadMonLogger(UploadSnapshot *snap);
static void scanI2C(void);
static void loadState(void);
static void updateStateFunction(void *parameter);
static bool Init_PulseCounter_01(void);
static bool Init_PulseCounter_02(void);
static void Clean_Counters(void);
static String iaqColour(float iaq);
static String doseColour(float usv);
static String hvColour(float voltage);
static String uploadStatusText(int code);
static float estimateHvDrivePct(float voltage);
static float estimateHvDriveUploadPct(float voltage);
static float movingAvgToCpm(movingAvg &avg);
static unsigned long applyDeadTimeCorrection(int rawCps, const char *tubeLabel);
static void refreshCpuLoadMetrics(void);
static bool checkEspOk(const char *operation, esp_err_t errorCode);
static void copyUploadRecord(UploadRecord &dst, const UploadSnapshot *src);
static String radmonUploadDetailsHtml(const UploadRecord &record);
static String uradmonUploadDetailsHtml(const UploadRecord &record);
static void fillUploadStatusSnapshot(UploadStatusSnapshot &dst);
static size_t getHeapTotalBytes(void);
static size_t getAppPartitionTotalBytes(void);
static size_t getAppPartitionUsedBytes(void);
static size_t getAppPartitionFreeBytes(void);
static size_t getFsTotalBytes(void);
static size_t getFsFreeBytes(void);
static size_t getHistoryRetentionSamples(void);
static size_t getHistoryRetentionHours(void);
static String describePartitionSlot(const esp_partition_t *partition);
static int getWifiSignalPct(long rssiDbm);
static String wifiSecurityLabel(wifi_auth_mode_t authMode);
static String urlEncode(const String &value);
static String buildWifiScanResultsHtml(int networkCount, bool includeConfigLinks);
static String buildSpiffsDirectoryHtml(void);
static bool appendHistorySample(time_t sampleEpoch);
static void pruneHistoryLogIfNeeded(void);
static size_t countHistoryRows(void);
static void loadRuntimeSettings(void);
static void configureTimeRules(void);
static bool saveRuntimeSettingsFromRequest(String &notice, String &noticeClass);
static void saveRuntimeSetting(Preferences &prefs, const char *key, const String &value);
static const TubePresetDefinition *findTubePresetById(const String &presetId);
static String normalizeTubePresetId(const String &presetId);
static String normalizeDstProfileId(const String &profileId);
static String buildDstProfileOptionsHtml(const String &selectedProfileId);
static String dstProfileLabel(const String &profileId);
static String formatUtcOffsetMinutes(int offsetMinutes);
static String formatOperatingVoltageRange(float minimumVoltage, float maximumVoltage);
static String buildTubePresetOptionsHtml(const String &selectedPresetId);
static bool parsePositiveFloatField(const String &value, float &parsedValue);
static bool parseFloatFieldInRange(const String &value, float &parsedValue, float minimumValue, float maximumValue);
static bool parseLongFieldInRange(const String &value, long &parsedValue, long minimumValue, long maximumValue);
static String htmlEscape(const String &value);
static String changelogDialogHtml(void);
static String adminPageShell(const String &title, const String &subtitle, const String &body);
void handleOtaCheckPath();
void handleConfigPath();
void handleRebootPath();
void handleSimpleRebootPath();
void handleWifiScanPath();
void handleWifiScanPagePath();
void handleGraphsPath();
void handleHistoryCsvPath();
void handleHistoryDeletePath();
std::vector<String> webPageChunks(void);
String JsonPage(void);

#define VERBOSE_SERIAL_PRINT(...) do { if (serialVerboseEnabled) { Serial.print(__VA_ARGS__); } } while (0)
#define VERBOSE_SERIAL_PRINTLN(...) do { if (serialVerboseEnabled) { Serial.println(__VA_ARGS__); } } while (0)

// Variables
int increaseSecCount = 1, var_iaqAccuracy = 0, var_pm01 = 0, var_pm25 = 0, var_pm10 = 0, luminosity = 0;
float var_iaq = 0.0;
float var_temperature = 0.0, var_humidity = 0.0, var_pressure = 0.0, var_voc = 0.0, var_co2 = 0.0, var_hcho = 0.0, tubeVoltage = 0.0;
float loopActivePct = 0.0f;
float cpuLoadCore0Pct = 0.0f, cpuLoadCore1Pct = 0.0f;

//unsigned long cps_1 = 0, cps_2 = 0;
int cps_1 = 0, cps_2 = 0;
unsigned long actual_cps_1 = 0, actual_cps_2 = 0;
unsigned long cpm = 0, cpm1 = 0, cpm2 = 0;
unsigned long sensorMovingAvg = 0;
unsigned long previousMillis_1 = 0, previousMillis_2 = 0, previousMillis_3 = 0;
uint32_t loopWindowStartUs = 0, loopBusyUs = 0;
int i2cDevicesFound = 0;
bool spiffsMounted = false;
bool pmSensorAvailable = false;
bool pulseCountersReady = false;
bool serialVerboseEnabled = true;
size_t historyRowCount = 0;
unsigned long lastDeadTimeWarningMillis = 0;
String activeTubePresetId = String(LOGGER_DEFAULT_TUBE_PRESET_ID);
String activeTubePresetLabel = String(CUSTOM_TUBE_PRESET_LABEL);
float activeTubeDeadTimeSeconds = LOGGER_DEFAULT_TUBE_DEAD_TIME_SECONDS;
float activeTubeConversionFactor = LOGGER_DEFAULT_TUBE_CONVERSION_FACTOR_USV_PER_CPM;
float activeTubeOperatingVoltageMin = 0.0f;
float activeTubeOperatingVoltageMax = 0.0f;
String activeTubePresetNote = String("Custom values are saved in NVS; verify the operating voltage range against your tube datasheet.");
int activeTimezoneOffsetMinutes = DEFAULT_TIMEZONE_OFFSET_MINUTES;
int activeDstOffsetMinutes = DEFAULT_DST_OFFSET_MINUTES;
String activeDstProfileId = String(DEFAULT_DST_PROFILE_ID);
float activeHvCalibrationFactor = DEFAULT_HV_CALIBRATION_FACTOR;
float activeHchoR0 = DEFAULT_HCHO_R0;
size_t activeHistoryRetentionSamples = DEFAULT_HISTORY_RETENTION_SAMPLES;
bool activeRadmonUploadEnabled = true;
bool activeURadmonUploadEnabled = true;
String activeNtpServer = String(DEFAULT_NTP_SERVER);
long activeCpmGaugeFullScale = DEFAULT_CPM_GAUGE_FULL_SCALE;
String activeStationName = String(DEFAULT_STATION_NAME);

// Create rolling windows: combined radiation over 120 s, per-tube over 60 s
movingAvg cps_sensor(120);
movingAvg cps_sensor_01(60);
movingAvg cps_sensor_02(60);

// Create time_t object
time_t epoch = 0;

// Time NTP client
WiFiUDP wifiUdp;
NTP ntp(wifiUdp);

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
unsigned long lastStateSaveMillis = 0;

// Grove Laser PM2.5 Sensor (HM3301)
Tomoto_HM330X pm_sensor;

// ESP32 Pulse Counter (new driver/pulse_cnt.h API)
pcnt_unit_handle_t pcnt_unit_01 = NULL;
pcnt_unit_handle_t pcnt_unit_02 = NULL;
pcnt_channel_handle_t pcnt_chan_01 = NULL;
pcnt_channel_handle_t pcnt_chan_02 = NULL;

#define PCNT_H_LIM_VAL 1000          // Set the high limit count to trigger the interrupt
//#define PCNT_L_LIM_VAL 0          // Set the low limit count to trigger the interrupt (unused)
#define PCNT_FILTER_VAL 100         // Set the filter value 0-1023

#define PCNT_INPUT_SIG_IO_01 13     // Pulse Input GPIO13
#define PCNT_INPUT_SIG_IO_02 14     // Pulse Input GPIO14

//pcnt_isr_handle_t user_isr_handle = NULL; // User ISR handler for Interrupt (unused)
volatile bool eventTriggered = false;

// WiFi credentials
String my_ssid = String(SECRET_SSID);
String my_password = String(SECRET_PASS);
String my_hostname = String(SECRET_HOST);

// RADMON credentials
String UserName = String(SECRET_USER_NAME);
String DataSendingPassWord = String(SECRET_USER_PASS_01);
int statusCodeRadmon = 0;
UploadRecord lastRadmonUpload = {};

// URadMonitoring credentials
String USER_ID = String(SECRET_USER_ID);
String USER_KEY = String(SECRET_USER_KEY);
String DEVICE_ID = String(SECRET_DEVICE_ID);
int statusCodeURadmon = 0;
UploadRecord lastURadmonUpload = {};
portMUX_TYPE uploadStatusMux = portMUX_INITIALIZER_UNLOCKED;

// Set NTP server for Europe (currently unused; ntp.begin() uses pool default)
//const char *ntpServer = "europe.pool.ntp.org";

// Create a task handle
TaskHandle_t uploadTask = NULL;
TaskHandle_t updateState = NULL;

WebServer server(80);  // Set web server port
Preferences settingsStore;

// ISR callback for PCNT watch points
static bool Counter_ISR(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
  (void)unit;
  (void)edata;
  (void)user_ctx;
  eventTriggered = true;
  return false;  // no need to wake up from ISR
}

// Allocate the JSON document
JsonDocument doc;

//--------------------//
//--- START SETUP ---//
//------------------//
void setup()
{
  esp_err_t wdt_result = ESP_OK;

  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
  
  // Initialize UART on 115200 baud rate for ethernet shield
  Serial.begin(115200);

  Serial.println(F("Initializing Serial."));
  while (!Serial)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println(F("Serial initialized."));
  
  // Initialize watchdog
  Serial.println(F("Initializing WatchDogTimer (WDT)"));
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  wdt_result = esp_task_wdt_init(&wdt_config);

  if (wdt_result == ESP_ERR_INVALID_STATE)
  {
    wdt_result = esp_task_wdt_reconfigure(&wdt_config);
  }

  if (wdt_result != ESP_OK)
  {
    Serial.println("WDT configuration failed, error = " + String((int)wdt_result));
  }

  // Check if I2C bus is enabled
  if(!Wire.begin())
  {
    Serial.println(F("I2C bus not initialized!"));
  }
  
  else
  {
    Serial.println(F("I2C bus is initialized."));

    loadRuntimeSettings();
  
    // Connect to the WiFi
    WiFiSetup();

    spiffsMounted = SPIFFS.begin(false);
    if (!spiffsMounted)
    {
      Serial.println(F("WARNING: SPIFFS mount failed, attempting one-time format"));
      spiffsMounted = SPIFFS.begin(true);
    }

    if (spiffsMounted)
    {
      historyRowCount = countHistoryRows();
      Serial.println("SPIFFS mounted: free " + String(getFsFreeBytes() / 1024.0f, 1) + " kB / " + String(getFsTotalBytes() / 1024.0f, 1) + " kB, history rows=" + String(historyRowCount));
    }
    else
    {
      Serial.println(F("WARNING: SPIFFS unavailable after format attempt; disk space will show unavailable"));
    }

    // Test scan I2C address bus for devices
    scanI2C();
  
    // Initialize variables
    cps_1 = 0, cps_2 = 0;
    cpm = 0, cpm1 = 0, cpm2 = 0;
    actual_cps_1 = 0, actual_cps_2 = 0;
    sensorMovingAvg = 0;
    increaseSecCount = 1;
  
    // Initialize sensor
    cps_sensor.begin();
    cps_sensor_01.begin();
    cps_sensor_02.begin();
  
    // Initialize Arduino pins
    pinMode(33, INPUT_PULLUP);  // Set pin33 (GPIO33) input for ADC1_CHANNEL_5
    pinMode(34, INPUT_PULLUP);  // Set pin34 (GPIO34) input for HCHO sensor A
    analogReadResolution(12);
    analogSetPinAttenuation(33, ADC_11db);
    analogSetPinAttenuation(34, ADC_11db);

    pinMode(13, INPUT_PULLUP);  // Set pin13 (GPIO13) input for capturing GM Tube 01 events (pulses)
    pinMode(14, INPUT_PULLUP);  // Set pin14 (GPIO14) input for capturing GM Tube 02 events (pulses)

    //pinMode(26, OUTPUT);      // Set pin04 (GPIO26) as output for LED
    //pinMode(25, OUTPUT);      // Set pin25 (GPIO25) as output for LED upload = OK

    // Set the iaqSensor(BME680) to initialize on the I2C bus
    iaqSensor.begin(0x76, Wire);
    
    if (iaqSensor.bme68xStatus != BME68X_OK)
    {
      Serial.println(F("No valid BME680 found!"));
      Serial.println("BME680 error code : " + String(iaqSensor.bme68xStatus));
    }
    else
    {
      Serial.println(F("Valid BME680 found."));
      Serial.println("BSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));

      iaqSensor.setConfig(bsec_config_iaq);
      loadState();

      bsec_virtual_sensor_t sensorList[10] = 
      {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
      };

      iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    }

    if(!pm_sensor.begin())
    {
      Serial.println(F("WARNING: No valid HM330X found; PM readings disabled"));
      pmSensorAvailable = false;
    }

    else
    {
      Serial.println(F("Valid HM330X found."));
      pmSensorAvailable = true;
    }

    TSL2561.init();
    Serial.println(F("Valid TSL2561 found."));

    // Pulse Counter (PCNT)
    bool pcnt01Ready = Init_PulseCounter_01();
    bool pcnt02Ready = Init_PulseCounter_02();
    pulseCountersReady = pcnt01Ready && pcnt02Ready;
    if (!pulseCountersReady)
    {
      Serial.println(F("WARNING: Pulse counters unavailable; radiation readings disabled"));
    }

    configureTimeRules();
    ntp.begin(activeNtpServer.c_str());

    if (esp_task_wdt_status(NULL) != ESP_OK)
    {
      if (esp_task_wdt_add(NULL) != ESP_OK)
      {
        Serial.println(F("Failed to subscribe loopTask to WDT"));
      }
    }
  
    Serial.println(F("--- Start program ---"));
    Serial.println(F("\n"));
  }
}
//------------------//
//--- END SETUP ---//
//----------------//

//-------------------//
//--- START LOOP ---//
//-----------------//
void loop()
{
  uint32_t loopStartUs = micros();
  if (loopWindowStartUs == 0)
  {
    loopWindowStartUs = loopStartUs;
  }

  esp_task_wdt_reset();

  // Start Web diagnostics & data server on  http://localip:80
  server.handleClient();
  ElegantOTA.loop();
  
  ntp.update();

  if(eventTriggered)
  {
    VERBOSE_SERIAL_PRINTLN(F("--- ISR ---"));
    VERBOSE_SERIAL_PRINTLN("| Counter of tube exceeded the value of " + String(PCNT_H_LIM_VAL));
    VERBOSE_SERIAL_PRINTLN(F("--- ISR ---"));

    eventTriggered = false;

    //actual_cps_1 += actual_cps_1;
    //actual_cps_2 += actual_cps_2;
  }

  // If reached 01 seconds with a total of 60 seconds
  if (((millis() - previousMillis_1) >= 1000UL) && (increaseSecCount <= 60))
  {  
    if (pulseCountersReady)
    {
      // Get the pulse counter of unit_x and store it in var_y
      if (!checkEspOk("pcnt_unit_get_count tube 1", pcnt_unit_get_count(pcnt_unit_01, &cps_1)))
      {
        cps_1 = 0;
      }
      if (!checkEspOk("pcnt_unit_get_count tube 2", pcnt_unit_get_count(pcnt_unit_02, &cps_2)))
      {
        cps_2 = 0;
      }

      // Clear counters immediately so the next 1-second read reflects only that second's pulses.
      checkEspOk("pcnt_unit_stop tube 1", pcnt_unit_stop(pcnt_unit_01));
      checkEspOk("pcnt_unit_stop tube 2", pcnt_unit_stop(pcnt_unit_02));
      checkEspOk("pcnt_unit_clear_count tube 1", pcnt_unit_clear_count(pcnt_unit_01));
      checkEspOk("pcnt_unit_clear_count tube 2", pcnt_unit_clear_count(pcnt_unit_02));
      checkEspOk("pcnt_unit_start tube 1", pcnt_unit_start(pcnt_unit_01));
      checkEspOk("pcnt_unit_start tube 2", pcnt_unit_start(pcnt_unit_02));

      actual_cps_1 = applyDeadTimeCorrection(cps_1, "Tube 1");
      actual_cps_2 = applyDeadTimeCorrection(cps_2, "Tube 2");
    }
    else
    {
      cps_1 = 0;
      cps_2 = 0;
      actual_cps_1 = 0;
      actual_cps_2 = 0;
    }

    // Add combined CPS of both tubes as one datapoint per second (240-point = 240s window)
    cps_sensor.reading(actual_cps_1 + actual_cps_2);

    // Add a new datapoint entry for each tube to their individual moving average arrays
    cps_sensor_01.reading(actual_cps_1);
    cps_sensor_02.reading(actual_cps_2);

    float combinedCpm = movingAvgToCpm(cps_sensor);
    int _avg1 = cps_sensor_01.getAvg();
    int _avg2 = cps_sensor_02.getAvg();
    // Convert combined two-tube CPM back to average CPS per tube for logging.
    sensorMovingAvg = (unsigned long)roundf(combinedCpm / 120.0f);
    cpm1            = (_avg1 > 0) ? (unsigned long)roundf(movingAvgToCpm(cps_sensor_01)) : 0UL;
    cpm2            = (_avg2 > 0) ? (unsigned long)roundf(movingAvgToCpm(cps_sensor_02)) : 0UL;
    cpm             = (unsigned long)roundf(combinedCpm / 2.0f);
    
    VERBOSE_SERIAL_PRINTLN(F("--- 01 sec move ---"));
    VERBOSE_SERIAL_PRINTLN("| Tube 1 " + String(increaseSecCount) + " sec current count (cps_1) = " + String(actual_cps_1));
    VERBOSE_SERIAL_PRINTLN("| Tube 2 " + String(increaseSecCount) + " sec current count (cps_2) = " + String(actual_cps_2));
    //Serial.println("| Tubes current averaged mavg " + String(increaseSecCount) + " sec current count = " + String(sensorMovingAvg));
    VERBOSE_SERIAL_PRINTLN("| Tubes voltage = " + String(tubeVoltage, 3) + "V, note value is cached for 61 seconds.");
    VERBOSE_SERIAL_PRINTLN(F("--- 01 sec move ---"));

    // Display 1 second tick for a total of 60
    VERBOSE_SERIAL_PRINTLN(String(increaseSecCount) + " of 60");

    luminosity = TSL2561.readVisibleLux();
    var_hcho = calculateHCHO();

    if (iaqSensor.run())
    {
      var_temperature = iaqSensor.temperature;
      var_pressure = iaqSensor.pressure;
      var_humidity = iaqSensor.humidity;
      var_voc = iaqSensor.gasResistance;
      var_co2 = iaqSensor.co2Equivalent;
      
      var_iaq = iaqSensor.iaq;
      var_iaqAccuracy = iaqSensor.iaqAccuracy;

      // Uncomment below for IAQ testing
      //Serial.println("IAQ level = " + String(var_iaq));
      // 0 = Stabilizing, 1 = Uncertain, 2 = Calibrating, 3 = Calibrated
      //Serial.println("IAQ Accuracy = " + String(var_iaqAccuracy));

      // Initialize Task — snapshot BSEC accuracy/iaq now to avoid Core 0/1 race
      if (updateState == NULL)
      {
        StateSnapshot *ss = new StateSnapshot();
        if (ss == nullptr)
        {
          Serial.println(F("ERROR: Failed to allocate StateSnapshot"));
        }
        else
        {
          ss->iaqAccuracy = iaqSensor.iaqAccuracy;
          ss->iaq         = iaqSensor.iaq;

          if (xTaskCreatePinnedToCore
              (
                updateStateFunction, /* Function to implement the task */
                "updateState", /* Name of the task */
                10000,  /* Stack size in words */
                ss,  /* Task input parameter */
                0,  /* Priority of the task */
                &updateState,  /* Task handle. */
                1 /* Core where the task should run */
              ) != pdPASS)
          {
            Serial.println(F("ERROR: Failed to create updateState task"));
            delete ss;
          }
        }
      }
    }

    previousMillis_1 = millis();
    increaseSecCount++;
  }

  // If 31 seconds have been reached calculate HM330X values
  // BSEC needs 30 seconds to warm up
	if (((millis() - previousMillis_2) >= 31000UL) && (increaseSecCount == 31))
	{
    if (!pmSensorAvailable)
    {
      var_pm01 = 0;
      var_pm25 = 0;
      var_pm10 = 0;
    }

    else if (!pm_sensor.readSensor())
		{
			Serial.println(F("Failed to read values from HM330X"));
		}

    else
		{
      var_pm01 = pm_sensor.std.getPM1();
      var_pm25 = pm_sensor.std.getPM2_5();
      var_pm10 = pm_sensor.std.getPM10();

      // // Uncomment for testing
      // Serial.println("Sensor number = " + String(pm_sensor.getSensorNumber()));
      // Serial.println(F("Concentration based on CF=1 standard particlate matter (ug/m^3) --"));
      // Serial.println("PM1.0 = " + String(var_pm01));
      // Serial.println("PM2.5 = " + String(var_pm25));
      // Serial.println("PM10 = " + String(var_pm10));
    }

    previousMillis_2 = millis();
  }
  
  // If 61 seconds have been reached do uploads, local history logging, and reset vars
  if (((millis() - previousMillis_3) >= 61000UL) && (increaseSecCount >= 61))
  {
    tubeVoltage = displayTubeVoltage();
    epoch = ntp.epoch();
    appendHistorySample(epoch);

    // Extra WIFI connection check
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(F("Reconnecting to WiFi..."));
        WiFi.disconnect();
        WiFi.reconnect();
    }

    else
    {
      // Print new line
      VERBOSE_SERIAL_PRINTLN(F("\n"));

      VERBOSE_SERIAL_PRINTLN(F("--- 61 sec ---"));
      VERBOSE_SERIAL_PRINTLN("| Tube 1 CPS = " + String(actual_cps_1));
      VERBOSE_SERIAL_PRINTLN("| Tube 2 CPS = " + String(actual_cps_2));
      VERBOSE_SERIAL_PRINTLN("| Tubes avg. CPS = " + String(sensorMovingAvg));
      VERBOSE_SERIAL_PRINTLN("| Tubes voltage = " + String(tubeVoltage)  + "V");
      VERBOSE_SERIAL_PRINTLN(F("--- ... ---"));
      VERBOSE_SERIAL_PRINTLN("| var_temperature = " + String(var_temperature) + " C");
      VERBOSE_SERIAL_PRINTLN("| var_pressure = " + String(var_pressure) + " Pa");
      VERBOSE_SERIAL_PRINTLN("| var_humidity = " + String(var_humidity) + " %");
      VERBOSE_SERIAL_PRINTLN("| var_voc = " + String(var_voc) + " Ohm");
      VERBOSE_SERIAL_PRINTLN("| var_co2 = " + String(var_co2) + " ppm");
      VERBOSE_SERIAL_PRINTLN("| IAQ level = " + String(var_iaq));
      VERBOSE_SERIAL_PRINTLN("| IAQ Accuracy = " + String(var_iaqAccuracy));
      VERBOSE_SERIAL_PRINTLN(F("--- ... ---"));
      VERBOSE_SERIAL_PRINTLN("| luminosity = " + String(luminosity));
      VERBOSE_SERIAL_PRINTLN(F("--- ... ---"));
      VERBOSE_SERIAL_PRINTLN("| var_hcho = " + String(var_hcho));
      VERBOSE_SERIAL_PRINTLN(F("--- ... ---"));
      VERBOSE_SERIAL_PRINTLN("| var_pm01 = " + String(var_pm01));
      VERBOSE_SERIAL_PRINTLN("| var_pm25 = " + String(var_pm25));
      VERBOSE_SERIAL_PRINTLN("| var_pm10 = " + String(var_pm10));
      VERBOSE_SERIAL_PRINTLN(F("--- ... ---"));
      VERBOSE_SERIAL_PRINTLN(F("| Commence uploading"));
      VERBOSE_SERIAL_PRINTLN(F("| Resetting vars"));
      VERBOSE_SERIAL_PRINTLN(F("--- 61 sec ---"));

      VERBOSE_SERIAL_PRINTLN(F("--- 61 sec TESTING ---"));
      VERBOSE_SERIAL_PRINTLN("cps_sensor counts = " + String(cps_sensor.getCount()));
      VERBOSE_SERIAL_PRINTLN("Tube 1 CPM = " + String(cpm1));
      VERBOSE_SERIAL_PRINTLN("Tube 2 CPM = " + String(cpm2));
      VERBOSE_SERIAL_PRINTLN("Tubes averaged CPM = " + String(cpm));
      VERBOSE_SERIAL_PRINTLN(F("--- 61 sec TESTING ---"));

      VERBOSE_SERIAL_PRINTLN(F("\n"));

      // Initialize Task — snapshot globals now to avoid race with the 1-second loop on Core 0
      if (uploadTask == NULL)
      {
        UploadSnapshot *snap = new UploadSnapshot();

        // O2: guard against heap allocation failure
        if (snap == nullptr)
        {
          Serial.println(F("ERROR: Failed to allocate UploadSnapshot"));
        }
        else
        {
          snap->cpm             = cpm;
          snap->actual_cps_1    = actual_cps_1;
          snap->actual_cps_2    = actual_cps_2;
          snap->tubeVoltage     = tubeVoltage;
          snap->epoch           = epoch;
          snap->var_temperature = var_temperature;
          snap->var_humidity    = var_humidity;
          snap->var_pressure    = var_pressure;
          snap->luminosity      = luminosity;
          snap->var_voc         = var_voc;
          snap->var_co2         = var_co2;
          snap->var_hcho        = var_hcho;
          snap->var_pm01        = var_pm01;
          snap->var_pm25        = var_pm25;
          snap->var_pm10        = var_pm10;

          // O1: guard against task creation failure — free snap if it fails
          if (xTaskCreatePinnedToCore
              (
                uploadTaskFunction, /* Function to implement the task */
                "uploadTask", /* Name of the task */
                10000,  /* Stack size in words */
                snap,  /* Task input parameter */
                0,  /* Priority of the task */
                &uploadTask,  /* Task handle. */
                1 /* Core where the task should run */
              ) != pdPASS)
          {
            Serial.println(F("ERROR: Failed to create uploadTask"));
            delete snap;
          }
        }
      }

      // Reset upload cycle timing
      increaseSecCount = 1;
      
      Clean_Counters();

      // Reset WatchDogTimer
      esp_task_wdt_reset();

      previousMillis_1 = millis();
      previousMillis_2 = millis();
      previousMillis_3 = millis();
    }
  }

  uint32_t loopEndUs = micros();
  loopBusyUs += (loopEndUs - loopStartUs);

  uint32_t loopWindowElapsedUs = loopEndUs - loopWindowStartUs;
  if (loopWindowElapsedUs >= 1000000UL)
  {
    loopActivePct = constrain((loopBusyUs * 100.0f) / loopWindowElapsedUs, 0.0f, 100.0f);
    refreshCpuLoadMetrics();
    loopBusyUs = 0;
    loopWindowStartUs = loopEndUs;
  }

  // Avoid a busy-spin loop so the scheduler, WiFi stack, and idle task get regular time.
  delay(LOOP_IDLE_DELAY_MS);
}
//-----------------//
//--- END LOOP ---//
//---------------//

//------------------------//
//--- START functions ---//
//----------------------//

// Setup WiFi connection
static void WiFiSetup(void)
{
  const IPAddress noIp(0U, 0U, 0U, 0U);

  Serial.println(F("\n"));
  Serial.println("Connecting to = " + String(my_ssid));

  WiFi.mode(WIFI_STA);
  if (!WiFi.config(noIp, noIp, noIp, noIp, noIp))
  {
    Serial.println(F("WARNING: Failed to reset WiFi to DHCP mode"));
  }
  
  WiFi.begin(my_ssid.c_str(), my_password.c_str());
  WiFi.setHostname(my_hostname.c_str());
  
  // Set up Wifi connection
  while (WiFi.status() != WL_CONNECTED)
  {
    esp_task_wdt_reset();
    delay(500);
    Serial.print(".");
  }

  // Wifi is connected — guaranteed by the while loop above
  Serial.println(F("\n"));
  Serial.println(F("WiFi connected"));
  Serial.println("IP address = " +  String(WiFi.localIP().toString()));
  Serial.println("Gateway = " + String(WiFi.gatewayIP().toString()));
  Serial.println("SubnetMask = " + String(WiFi.subnetMask().toString()));
  Serial.println("DNS 1 = " + String(WiFi.dnsIP(0).toString()));
  Serial.println("DNS 2 = " + String(WiFi.dnsIP(1).toString()));
  Serial.println("MAC = " + String(WiFi.macAddress()));
  Serial.println("RSSI = " + String(WiFi.RSSI()) + " dB");
  Serial.println(F("\n"));

  server.on("/", handleRootPath);
  server.on("/json", handleJsonPath);
  server.on("/config", HTTP_GET, handleConfigPath);
  server.on("/config", HTTP_POST, handleConfigPath);
  server.on("/reboot", HTTP_POST, handleRebootPath);
  server.on("/restart", HTTP_POST, handleSimpleRebootPath);
  server.on("/wifi-scan", HTTP_GET, handleWifiScanPath);
  server.on("/wifi-scan-page", HTTP_GET, handleWifiScanPagePath);
  server.on("/graphs", HTTP_GET, handleGraphsPath);
  server.on("/history.csv", HTTP_GET, handleHistoryCsvPath);
  server.on("/history-delete", HTTP_POST, handleHistoryDeletePath);
  server.on("/ota-check", handleOtaCheckPath);
  ElegantOTA.begin(&server);
  server.begin();
  Serial.println(F("Server listening"));
  Serial.println("Config = http://" + String(WiFi.localIP().toString()) + "/config");
  Serial.println("ElegantOTA = http://" + String(WiFi.localIP().toString()) + "/update");
  Serial.println("OTA debug = http://" + String(WiFi.localIP().toString()) + "/ota-check");
}

static uint64_t getUptimeSeconds(void)
{
  return static_cast<uint64_t>(esp_timer_get_time() / 1000000ULL);
}

static String formatUptime(uint64_t uptimeSeconds)
{
  const uint64_t days = uptimeSeconds / 86400ULL;
  const uint64_t hours = (uptimeSeconds % 86400ULL) / 3600ULL;
  const uint64_t minutes = (uptimeSeconds % 3600ULL) / 60ULL;

  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%llu days, %02llu:%02llu",
           static_cast<unsigned long long>(days),
           static_cast<unsigned long long>(hours),
           static_cast<unsigned long long>(minutes));

  return String(buffer);
}

static void loadRuntimeSettings(void)
{
  if (!settingsStore.begin("envlogger", true))
  {
    Serial.println(F("WARNING: Failed to open settings namespace; using compiled defaults"));
    activeTimezoneOffsetMinutes = DEFAULT_TIMEZONE_OFFSET_MINUTES;
    activeDstOffsetMinutes = DEFAULT_DST_OFFSET_MINUTES;
    activeDstProfileId = String(DEFAULT_DST_PROFILE_ID);
    activeHvCalibrationFactor = DEFAULT_HV_CALIBRATION_FACTOR;
    activeHchoR0 = DEFAULT_HCHO_R0;
    activeHistoryRetentionSamples = DEFAULT_HISTORY_RETENTION_SAMPLES;
    activeRadmonUploadEnabled = true;
    activeURadmonUploadEnabled = true;
    activeNtpServer = String(DEFAULT_NTP_SERVER);
    activeCpmGaugeFullScale = DEFAULT_CPM_GAUGE_FULL_SCALE;
    activeStationName = String(DEFAULT_STATION_NAME);
    activeTubePresetId = String(LOGGER_DEFAULT_TUBE_PRESET_ID);
    const TubePresetDefinition *defaultPreset = findTubePresetById(activeTubePresetId);
    if (defaultPreset != NULL)
    {
      activeTubePresetLabel = String(defaultPreset->label);
      activeTubeDeadTimeSeconds = defaultPreset->deadTimeSeconds;
      activeTubeConversionFactor = defaultPreset->conversionFactorUsvPerCpm;
      activeTubeOperatingVoltageMin = defaultPreset->operatingVoltageMin;
      activeTubeOperatingVoltageMax = defaultPreset->operatingVoltageMax;
      activeTubePresetNote = String(defaultPreset->note);
    }
    return;
  }

  my_ssid = settingsStore.getString("wifi_ssid", String(SECRET_SSID));
  my_password = settingsStore.getString("wifi_pass", String(SECRET_PASS));
  my_hostname = settingsStore.getString("wifi_host", String(SECRET_HOST));
  UserName = settingsStore.getString("rad_user", String(SECRET_USER_NAME));
  DataSendingPassWord = settingsStore.getString("rad_pass", String(SECRET_USER_PASS_01));
  USER_ID = settingsStore.getString("ur_user", String(SECRET_USER_ID));
  USER_KEY = settingsStore.getString("ur_key", String(SECRET_USER_KEY));
  DEVICE_ID = settingsStore.getString("ur_dev", String(SECRET_DEVICE_ID));
  serialVerboseEnabled = settingsStore.getBool("ser_verbose", true);
  activeTimezoneOffsetMinutes = settingsStore.getInt("tz_offset", DEFAULT_TIMEZONE_OFFSET_MINUTES);
  activeDstOffsetMinutes = settingsStore.getInt("dst_offset", DEFAULT_DST_OFFSET_MINUTES);
  activeDstProfileId = normalizeDstProfileId(settingsStore.getString("dst_prof", String(DEFAULT_DST_PROFILE_ID)));
  activeHvCalibrationFactor = settingsStore.getFloat("hv_cal", DEFAULT_HV_CALIBRATION_FACTOR);
  activeHchoR0 = settingsStore.getFloat("hcho_r0", DEFAULT_HCHO_R0);
  activeHistoryRetentionSamples = (size_t)settingsStore.getInt("hist_keep", (int)DEFAULT_HISTORY_RETENTION_SAMPLES);
  activeRadmonUploadEnabled = settingsStore.getBool("rad_en", true);
  activeURadmonUploadEnabled = settingsStore.getBool("urad_en", true);
  activeNtpServer = settingsStore.getString("ntp_srv", String(DEFAULT_NTP_SERVER));
  activeCpmGaugeFullScale = (long)settingsStore.getInt("cpm_gauge", (int)DEFAULT_CPM_GAUGE_FULL_SCALE);
  activeStationName = settingsStore.getString("station_name", String(DEFAULT_STATION_NAME));

  if ((activeTimezoneOffsetMinutes < -720) || (activeTimezoneOffsetMinutes > 840))
  {
    activeTimezoneOffsetMinutes = DEFAULT_TIMEZONE_OFFSET_MINUTES;
  }
  if ((activeDstOffsetMinutes < 0) || (activeDstOffsetMinutes > 180))
  {
    activeDstOffsetMinutes = DEFAULT_DST_OFFSET_MINUTES;
  }
  if ((activeHvCalibrationFactor < 50.0f) || (activeHvCalibrationFactor > 400.0f))
  {
    activeHvCalibrationFactor = DEFAULT_HV_CALIBRATION_FACTOR;
  }
  if ((activeHchoR0 <= 0.0f) || (activeHchoR0 > 1000.0f))
  {
    activeHchoR0 = DEFAULT_HCHO_R0;
  }
  if ((activeHistoryRetentionSamples < DEFAULT_HISTORY_RETENTION_SAMPLES) || (activeHistoryRetentionSamples > (MAX_HISTORY_RETENTION_HOURS * HISTORY_RETENTION_SAMPLES_PER_HOUR)))
  {
    activeHistoryRetentionSamples = DEFAULT_HISTORY_RETENTION_SAMPLES;
  }
  if (activeNtpServer.length() == 0)
  {
    activeNtpServer = String(DEFAULT_NTP_SERVER);
  }
  if ((activeCpmGaugeFullScale < 50L) || (activeCpmGaugeFullScale > 10000L))
  {
    activeCpmGaugeFullScale = DEFAULT_CPM_GAUGE_FULL_SCALE;
  }
  if (activeStationName.length() == 0)
  {
    activeStationName = String(DEFAULT_STATION_NAME);
  }

  String storedTubePresetId = normalizeTubePresetId(settingsStore.getString("tube_preset", String(LOGGER_DEFAULT_TUBE_PRESET_ID)));
  const TubePresetDefinition *selectedTubePreset = findTubePresetById(storedTubePresetId);
  if (selectedTubePreset != NULL)
  {
    activeTubePresetId = String(selectedTubePreset->id);
    activeTubePresetLabel = String(selectedTubePreset->label);
    activeTubeDeadTimeSeconds = selectedTubePreset->deadTimeSeconds;
    activeTubeConversionFactor = selectedTubePreset->conversionFactorUsvPerCpm;
    activeTubeOperatingVoltageMin = selectedTubePreset->operatingVoltageMin;
    activeTubeOperatingVoltageMax = selectedTubePreset->operatingVoltageMax;
    activeTubePresetNote = String(selectedTubePreset->note);
  }
  else
  {
    activeTubePresetId = String(CUSTOM_TUBE_PRESET_ID);
    activeTubePresetLabel = String(CUSTOM_TUBE_PRESET_LABEL);
    activeTubeDeadTimeSeconds = settingsStore.getFloat("tube_dead", LOGGER_DEFAULT_TUBE_DEAD_TIME_SECONDS);
    activeTubeConversionFactor = settingsStore.getFloat("tube_conv", LOGGER_DEFAULT_TUBE_CONVERSION_FACTOR_USV_PER_CPM);
    activeTubeOperatingVoltageMin = 0.0f;
    activeTubeOperatingVoltageMax = 0.0f;
    activeTubePresetNote = String("Custom values are saved in NVS; verify the operating voltage range against your tube datasheet.");

    if (activeTubeDeadTimeSeconds <= 0.0f)
    {
      activeTubeDeadTimeSeconds = LOGGER_DEFAULT_TUBE_DEAD_TIME_SECONDS;
    }
    if (activeTubeConversionFactor <= 0.0f)
    {
      activeTubeConversionFactor = LOGGER_DEFAULT_TUBE_CONVERSION_FACTOR_USV_PER_CPM;
    }
  }
  settingsStore.end();

  Serial.println("Runtime config loaded. Hostname=" + my_hostname + ", WiFi SSID=" + my_ssid + ", tube=" + activeTubePresetLabel + ", timezone=" + formatUtcOffsetMinutes(activeTimezoneOffsetMinutes));
}

static void configureTimeRules(void)
{
  String dstProfileId = normalizeDstProfileId(activeDstProfileId);
  int dstMinutes = (dstProfileId == DST_PROFILE_NONE_ID) ? 0 : activeDstOffsetMinutes;

  if (dstProfileId == DST_PROFILE_US_ID)
  {
    ntp.ruleDST("DST", Second, Sun, Mar, 2, activeTimezoneOffsetMinutes + dstMinutes);
    ntp.ruleSTD("STD", First, Sun, Nov, 2, activeTimezoneOffsetMinutes);
    return;
  }

  if (dstProfileId == DST_PROFILE_AU_ID)
  {
    ntp.ruleDST("DST", First, Sun, Oct, 2, activeTimezoneOffsetMinutes + dstMinutes);
    ntp.ruleSTD("STD", First, Sun, Apr, 3, activeTimezoneOffsetMinutes);
    return;
  }

  ntp.ruleDST("DST", Last, Sun, Mar, 2, activeTimezoneOffsetMinutes + dstMinutes);
  ntp.ruleSTD("STD", Last, Sun, Oct, 3, activeTimezoneOffsetMinutes);
}

static bool saveRuntimeSettingsFromRequest(String &notice, String &noticeClass)
{
  String wifiSsid = server.arg("wifi_ssid");
  String wifiPassword = server.arg("wifi_password");
  String wifiHostname = server.arg("wifi_hostname");
  String radmonUser = server.arg("radmon_user");
  String radmonPassword = server.arg("radmon_password");
  String uradUserId = server.arg("urad_user_id");
  String uradUserKey = server.arg("urad_user_key");
  String uradDeviceId = server.arg("urad_device_id");
  String tubePresetId = normalizeTubePresetId(server.arg("tube_preset"));
  String tubeDeadTimeUs = server.arg("tube_dead_time_us");
  String tubeConversionFactorValue = server.arg("tube_conversion_factor");
  String timezoneOffsetValue = server.arg("timezone_offset_minutes");
  String dstProfileValue = normalizeDstProfileId(server.arg("dst_profile"));
  String dstOffsetValue = server.arg("dst_offset_minutes");
  String hvCalibrationValue = server.arg("hv_calibration_factor");
  String hchoR0Value = server.arg("hcho_r0");
  String historyRetentionValue = server.arg("history_retention_hours");
  String ntpServerValue = server.arg("ntp_server");
  String cpmGaugeFullScaleValue = server.arg("cpm_gauge_full_scale");
  String stationNameValue = server.arg("station_name");
  bool resetWifiPassword = server.hasArg("wifi_password_reset");
  bool resetRadmonPassword = server.hasArg("radmon_password_reset");
  bool resetURadUserKey = server.hasArg("urad_user_key_reset");
  bool serialVerbose = server.hasArg("serial_verbose");
  bool radmonUploadEnabled = server.hasArg("radmon_upload_enabled");
  bool uradmonUploadEnabled = server.hasArg("urad_upload_enabled");

  wifiSsid.trim();
  wifiPassword.trim();
  wifiHostname.trim();
  radmonUser.trim();
  radmonPassword.trim();
  uradUserId.trim();
  uradUserKey.trim();
  uradDeviceId.trim();
  tubeDeadTimeUs.trim();
  tubeConversionFactorValue.trim();
  timezoneOffsetValue.trim();
  dstOffsetValue.trim();
  hvCalibrationValue.trim();
  hchoR0Value.trim();
  historyRetentionValue.trim();
  ntpServerValue.trim();
  cpmGaugeFullScaleValue.trim();
  stationNameValue.trim();

  float customDeadTimeUs = 0.0f;
  float customConversionFactor = 0.0f;
  long parsedTimezoneOffset = DEFAULT_TIMEZONE_OFFSET_MINUTES;
  long parsedDstOffset = DEFAULT_DST_OFFSET_MINUTES;
  long parsedHistoryRetentionHours = (long)DEFAULT_HISTORY_RETENTION_HOURS;
  long parsedCpmGaugeFullScale = DEFAULT_CPM_GAUGE_FULL_SCALE;
  float parsedHvCalibration = DEFAULT_HV_CALIBRATION_FACTOR;
  float parsedHchoR0 = DEFAULT_HCHO_R0;

  if ((timezoneOffsetValue.length() > 0) && !parseLongFieldInRange(timezoneOffsetValue, parsedTimezoneOffset, -720L, 840L))
  {
    notice = "Timezone offset must be between -720 and 840 minutes.";
    noticeClass = "danger";
    return false;
  }
  if ((dstOffsetValue.length() > 0) && !parseLongFieldInRange(dstOffsetValue, parsedDstOffset, 0L, 180L))
  {
    notice = "DST offset must be between 0 and 180 minutes.";
    noticeClass = "danger";
    return false;
  }
  if ((hvCalibrationValue.length() > 0) && !parseFloatFieldInRange(hvCalibrationValue, parsedHvCalibration, 50.0f, 400.0f))
  {
    notice = "Tube HV calibration must be between 50 and 400.";
    noticeClass = "danger";
    return false;
  }
  if ((hchoR0Value.length() > 0) && !parseFloatFieldInRange(hchoR0Value, parsedHchoR0, 0.1f, 1000.0f))
  {
    notice = "HCHO R0 must be between 0.1 and 1000.";
    noticeClass = "danger";
    return false;
  }
  if ((historyRetentionValue.length() > 0) && !parseLongFieldInRange(historyRetentionValue, parsedHistoryRetentionHours, 1L, (long)MAX_HISTORY_RETENTION_HOURS))
  {
    notice = "Historic storage must be between 1 and 24 hours.";
    noticeClass = "danger";
    return false;
  }
  if ((cpmGaugeFullScaleValue.length() > 0) && !parseLongFieldInRange(cpmGaugeFullScaleValue, parsedCpmGaugeFullScale, 50L, 10000L))
  {
    notice = "CPM gauge full-scale must be between 50 and 10000.";
    noticeClass = "danger";
    return false;
  }

  if (tubePresetId == CUSTOM_TUBE_PRESET_ID)
  {
    if (!parsePositiveFloatField(tubeDeadTimeUs, customDeadTimeUs))
    {
      notice = "Custom tube dead time must be a positive number in microseconds.";
      noticeClass = "danger";
      return false;
    }
    if (!parsePositiveFloatField(tubeConversionFactorValue, customConversionFactor))
    {
      notice = "Custom tube conversion factor must be a positive number in uSv/h per CPM.";
      noticeClass = "danger";
      return false;
    }
  }

  if (!settingsStore.begin("envlogger", false))
  {
    notice = "Failed to open NVS for writing. Existing runtime settings were left unchanged.";
    noticeClass = "danger";
    return false;
  }

  saveRuntimeSetting(settingsStore, "wifi_ssid", wifiSsid);
  if (resetWifiPassword)
  {
    settingsStore.remove("wifi_pass");
  }
  else if (wifiPassword.length() > 0)
  {
    settingsStore.putString("wifi_pass", wifiPassword);
  }
  saveRuntimeSetting(settingsStore, "wifi_host", wifiHostname);
  saveRuntimeSetting(settingsStore, "rad_user", radmonUser);
  if (resetRadmonPassword)
  {
    settingsStore.remove("rad_pass");
  }
  else if (radmonPassword.length() > 0)
  {
    settingsStore.putString("rad_pass", radmonPassword);
  }
  saveRuntimeSetting(settingsStore, "ur_user", uradUserId);
  if (resetURadUserKey)
  {
    settingsStore.remove("ur_key");
  }
  else if (uradUserKey.length() > 0)
  {
    settingsStore.putString("ur_key", uradUserKey);
  }
  saveRuntimeSetting(settingsStore, "ur_dev", uradDeviceId);
  settingsStore.putBool("ser_verbose", serialVerbose);
  settingsStore.putBool("rad_en", radmonUploadEnabled);
  settingsStore.putBool("urad_en", uradmonUploadEnabled);
  if (timezoneOffsetValue.length() > 0)
  {
    settingsStore.putInt("tz_offset", (int)parsedTimezoneOffset);
  }
  else
  {
    settingsStore.remove("tz_offset");
  }
  settingsStore.putString("dst_prof", dstProfileValue);
  if (dstOffsetValue.length() > 0)
  {
    settingsStore.putInt("dst_offset", (int)parsedDstOffset);
  }
  else
  {
    settingsStore.remove("dst_offset");
  }
  if (hvCalibrationValue.length() > 0)
  {
    settingsStore.putFloat("hv_cal", parsedHvCalibration);
  }
  else
  {
    settingsStore.remove("hv_cal");
  }
  if (hchoR0Value.length() > 0)
  {
    settingsStore.putFloat("hcho_r0", parsedHchoR0);
  }
  else
  {
    settingsStore.remove("hcho_r0");
  }
  if (historyRetentionValue.length() > 0)
  {
    settingsStore.putInt("hist_keep", (int)(parsedHistoryRetentionHours * (long)HISTORY_RETENTION_SAMPLES_PER_HOUR));
  }
  else
  {
    settingsStore.remove("hist_keep");
  }
  saveRuntimeSetting(settingsStore, "ntp_srv", ntpServerValue);
  if (cpmGaugeFullScaleValue.length() > 0)
  {
    settingsStore.putInt("cpm_gauge", (int)parsedCpmGaugeFullScale);
  }
  else
  {
    settingsStore.remove("cpm_gauge");
  }
  saveRuntimeSetting(settingsStore, "station_name", stationNameValue);
  settingsStore.putString("tube_preset", tubePresetId);
  if (tubePresetId == CUSTOM_TUBE_PRESET_ID)
  {
    settingsStore.putFloat("tube_dead", customDeadTimeUs / 1000000.0f);
    settingsStore.putFloat("tube_conv", customConversionFactor);
  }
  else
  {
    settingsStore.remove("tube_dead");
    settingsStore.remove("tube_conv");
  }
  settingsStore.end();

  loadRuntimeSettings();
  configureTimeRules();
  ntp.begin(activeNtpServer.c_str());
  notice = "Settings saved to NVS. Calibration, history, and timezone settings apply immediately. Secret fields stay unchanged when left blank, or revert to defaults when their reset box is checked. Tube presets come from logger_user_config.h; choose Custom if you want to override them from the web UI. Reboot the device if you changed WiFi or hostname values.";
  noticeClass = "success";
  return true;
}

static void saveRuntimeSetting(Preferences &prefs, const char *key, const String &value)
{
  if (value.length() > 0)
  {
    prefs.putString(key, value);
  }
  else
  {
    prefs.remove(key);
  }
}

static const TubePresetDefinition *findTubePresetById(const String &presetId)
{
  for (size_t index = 0; index < TUBE_PRESET_DEFINITIONS_COUNT; ++index)
  {
    if (presetId.equalsIgnoreCase(TUBE_PRESET_DEFINITIONS[index].id))
    {
      return &TUBE_PRESET_DEFINITIONS[index];
    }
  }

  return NULL;
}

static String normalizeTubePresetId(const String &presetId)
{
  String normalizedPresetId = presetId;
  normalizedPresetId.trim();

  if (normalizedPresetId.equalsIgnoreCase(CUSTOM_TUBE_PRESET_ID))
  {
    return String(CUSTOM_TUBE_PRESET_ID);
  }

  const TubePresetDefinition *preset = findTubePresetById(normalizedPresetId);
  if (preset != NULL)
  {
    return String(preset->id);
  }

  return String(LOGGER_DEFAULT_TUBE_PRESET_ID);
}

static String normalizeDstProfileId(const String &profileId)
{
  String normalizedProfileId = profileId;
  normalizedProfileId.trim();

  if (normalizedProfileId.equalsIgnoreCase(DST_PROFILE_NONE_ID)) return String(DST_PROFILE_NONE_ID);
  if (normalizedProfileId.equalsIgnoreCase(DST_PROFILE_US_ID)) return String(DST_PROFILE_US_ID);
  if (normalizedProfileId.equalsIgnoreCase(DST_PROFILE_AU_ID)) return String(DST_PROFILE_AU_ID);
  return String(DST_PROFILE_EU_ID);
}

static String buildDstProfileOptionsHtml(const String &selectedProfileId)
{
  String normalizedProfileId = normalizeDstProfileId(selectedProfileId);
  String html;
  html.reserve(320);
  html += "<option value='none'" + String(normalizedProfileId == DST_PROFILE_NONE_ID ? " selected" : "") + ">No automatic DST</option>";
  html += "<option value='eu'" + String(normalizedProfileId == DST_PROFILE_EU_ID ? " selected" : "") + ">Europe / UK style</option>";
  html += "<option value='us'" + String(normalizedProfileId == DST_PROFILE_US_ID ? " selected" : "") + ">US / Canada style</option>";
  html += "<option value='au'" + String(normalizedProfileId == DST_PROFILE_AU_ID ? " selected" : "") + ">Australia style</option>";
  return html;
}

static String dstProfileLabel(const String &profileId)
{
  String normalizedProfileId = normalizeDstProfileId(profileId);
  if (normalizedProfileId == DST_PROFILE_NONE_ID) return String("No automatic DST");
  if (normalizedProfileId == DST_PROFILE_US_ID) return String("US / Canada style");
  if (normalizedProfileId == DST_PROFILE_AU_ID) return String("Australia style");
  return String("Europe / UK style");
}

static String formatUtcOffsetMinutes(int offsetMinutes)
{
  char sign = (offsetMinutes < 0) ? '-' : '+';
  int absoluteMinutes = abs(offsetMinutes);
  int offsetHours = absoluteMinutes / 60;
  int offsetRemainderMinutes = absoluteMinutes % 60;
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "UTC%c%02d:%02d", sign, offsetHours, offsetRemainderMinutes);
  return String(buffer);
}

static String formatOperatingVoltageRange(float minimumVoltage, float maximumVoltage)
{
  if ((minimumVoltage <= 0.0f) || (maximumVoltage <= 0.0f) || (maximumVoltage < minimumVoltage))
  {
    return String("Verify against datasheet");
  }

  return String(minimumVoltage, 0) + "-" + String(maximumVoltage, 0) + " V";
}

static String buildTubePresetOptionsHtml(const String &selectedPresetId)
{
  String html;
  html.reserve(1200);

  for (size_t index = 0; index < TUBE_PRESET_DEFINITIONS_COUNT; ++index)
  {
    const TubePresetDefinition &preset = TUBE_PRESET_DEFINITIONS[index];
    html += "<option value='" + htmlEscape(String(preset.id)) + "' data-dead-us='" + String(preset.deadTimeSeconds * 1000000.0f, 3) + "' data-conv='" + String(preset.conversionFactorUsvPerCpm, 6) + "' data-vmin='" + String(preset.operatingVoltageMin, 0) + "' data-vmax='" + String(preset.operatingVoltageMax, 0) + "' data-note='" + htmlEscape(String(preset.note)) + "'";
    if (selectedPresetId.equalsIgnoreCase(preset.id))
    {
      html += " selected";
    }
    html += ">" + htmlEscape(String(preset.label)) + " (" + htmlEscape(formatOperatingVoltageRange(preset.operatingVoltageMin, preset.operatingVoltageMax)) + ")</option>";
  }

  html += "<option value='" + String(CUSTOM_TUBE_PRESET_ID) + "' data-dead-us='' data-conv='' data-vmin='' data-vmax='' data-note='Use your tube datasheet and HV board documentation for the custom operating range.'";
  if (selectedPresetId.equalsIgnoreCase(CUSTOM_TUBE_PRESET_ID))
  {
    html += " selected";
  }
  html += ">" + String(CUSTOM_TUBE_PRESET_LABEL) + "</option>";
  return html;
}

static bool parsePositiveFloatField(const String &value, float &parsedValue)
{
  return parseFloatFieldInRange(value, parsedValue, 0.000001f, 1000000.0f);
}

static bool parseFloatFieldInRange(const String &value, float &parsedValue, float minimumValue, float maximumValue)
{
  String trimmedValue = value;
  trimmedValue.trim();
  if (trimmedValue.length() == 0)
  {
    return false;
  }

  char *endPtr = NULL;
  parsedValue = strtof(trimmedValue.c_str(), &endPtr);
  if ((endPtr == trimmedValue.c_str()) || (*endPtr != '\0'))
  {
    return false;
  }

  return (parsedValue >= minimumValue) && (parsedValue <= maximumValue);
}

static bool parseLongFieldInRange(const String &value, long &parsedValue, long minimumValue, long maximumValue)
{
  String trimmedValue = value;
  trimmedValue.trim();
  if (trimmedValue.length() == 0)
  {
    return false;
  }

  char *endPtr = NULL;
  parsedValue = strtol(trimmedValue.c_str(), &endPtr, 10);
  if ((endPtr == trimmedValue.c_str()) || (*endPtr != '\0'))
  {
    return false;
  }

  return (parsedValue >= minimumValue) && (parsedValue <= maximumValue);
}

static String htmlEscape(const String &value)
{
  String escaped = value;
  escaped.replace("&", "&amp;");
  escaped.replace("<", "&lt;");
  escaped.replace(">", "&gt;");
  escaped.replace("\"", "&quot;");
  escaped.replace("'", "&#39;");
  return escaped;
}

static String changelogDialogHtml()
{
  String html;
  html += "<style>#changelogDlg::backdrop{background:rgba(0,0,0,.65);backdrop-filter:blur(4px);}</style>";
  html += "<dialog id='changelogDlg' style='background:var(--card);border:1px solid var(--border);border-radius:18px;padding:24px;max-width:700px;width:90vw;max-height:80vh;overflow-y:auto;color:var(--text);'>";
  html += "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:16px;'>";
  html += "<strong style='font-size:1rem;'>Firmware Changelog</strong>";
  html += "<button onclick='document.getElementById(\"changelogDlg\").close()' style='background:none;border:none;color:var(--muted);cursor:pointer;font-size:1.4rem;line-height:1;'>&#10005;</button>";
  html += "</div>";
  html += "<pre style='white-space:pre-wrap;font-size:.79rem;color:var(--muted);line-height:1.7;margin:0;'>";
  html += FIRMWARE_CHANGELOG;
  html += "</pre>";
  html += "</dialog>";
  return html;
}

static String adminPageShell(const String &title, const String &subtitle, const String &body)
{
  String ipText = WiFi.localIP().toString();
  String uptimeText = formatUptime(getUptimeSeconds());
  uint64_t uptimeSeconds = getUptimeSeconds();
  String shell;

  shell.reserve(body.length() + 5900);
  shell += "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  shell += "<link rel='icon' type='image/svg+xml' href=\"data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>&#9762;</text></svg>\">";
  shell += "<title>" + htmlEscape(my_hostname) + " : Environmental Logger</title>";
  shell += "<link href='https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css' rel='stylesheet'>";
  shell += "<style>"
           ":root{--bg:#10161d;--bg-soft:#17212b;--card:#1c2733;--border:#334456;--text:#f3f7fb;--muted:#c5d1dd;--accent:#58a6ff;--accent-strong:#3d8bfd;--chip:#1a2834;--shadow:rgba(0,0,0,.28);--heroA:#17324a;--heroB:#245e86;--surface:rgba(255,255,255,.08);color-scheme:dark;}"
           "body[data-theme='light']{--bg:#eff4f8;--bg-soft:#f7fafc;--card:#ffffff;--border:#d4dee8;--text:#17212b;--muted:#5d6d7d;--accent:#1e6bd6;--accent-strong:#1451a8;--chip:#eef3f7;--shadow:rgba(22,41,66,.12);--heroA:#ddebf7;--heroB:#f7fbff;--surface:rgba(255,255,255,.72);color-scheme:light;}"
           "*{box-sizing:border-box;}body{margin:0;font-family:Segoe UI,Arial,sans-serif;background:radial-gradient(circle at top,var(--heroA),var(--bg) 38%);color:var(--text);transition:background-color .25s,color .25s;}"
           ".page-shell{max-width:1240px;margin:0 auto;padding:20px 16px 28px;}"
           ".topbar{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:12px;margin-bottom:14px;padding:12px 14px;border:1px solid var(--border);border-radius:18px;background:var(--surface);backdrop-filter:blur(12px);box-shadow:0 12px 28px var(--shadow);}"
           ".brand{display:flex;align-items:center;gap:10px;font-weight:700;letter-spacing:.02em;color:var(--text);}"
           ".brand-mark{width:34px;height:34px;border-radius:12px;display:grid;place-items:center;background:linear-gradient(135deg,var(--accent),var(--accent-strong));color:#fff;font-size:1rem;box-shadow:0 8px 18px rgba(33,109,214,.28);}"
           ".menu{display:flex;flex-wrap:wrap;gap:8px;align-items:center;}"
           ".menu a,.menu button,.theme-picker{border-radius:999px;border:1px solid var(--border);background:var(--card);color:var(--text);text-decoration:none;padding:8px 12px;font-size:.84rem;font-weight:600;box-shadow:0 8px 20px var(--shadow);}"
           ".menu a.active{background:linear-gradient(135deg,var(--accent),var(--accent-strong));color:#fff;border-color:transparent;}"
           ".theme-picker{display:flex;align-items:center;gap:8px;padding:6px 10px;}"
           ".theme-picker label{font-size:.75rem;text-transform:uppercase;letter-spacing:.06em;color:var(--muted);margin:0;}"
           ".theme-picker select{border:1px solid var(--border);background:var(--bg-soft);color:var(--text);border-radius:999px;padding:6px 10px;font:inherit;}"
           ".hero{padding:24px;border:1px solid var(--border);border-radius:24px;background:linear-gradient(135deg,var(--surface),rgba(255,255,255,.02)),linear-gradient(135deg,var(--heroA),var(--heroB));box-shadow:0 20px 40px var(--shadow);margin-bottom:22px;}"
           ".hero-grid{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:14px;align-items:end;}"
           ".hero h1{margin:0 0 6px;font-size:clamp(1.6rem,3vw,2.4rem);line-height:1.05;color:var(--text);}"
           ".hero p{margin:0;color:var(--muted);max-width:780px;}"
           ".hero-meta{display:flex;flex-wrap:wrap;gap:8px;margin-top:14px;}"
           ".hero-chip{padding:8px 12px;border:1px solid var(--border);border-radius:999px;background:var(--chip);font-size:.82rem;color:var(--text);}"
           ".card{background:var(--card)!important;border:1px solid var(--border)!important;border-radius:20px!important;box-shadow:0 18px 36px var(--shadow)!important;color:var(--text);}"
           ".card h2,.card h3,.card h4,.card h5,.card h6,.text-body-secondary,.form-label,.form-check-label,.fw-semibold,.metric{color:var(--text)!important;}"
           ".hint,.form-text,.small{color:var(--muted)!important;}"
           ".card p,.card li,.card td,.card div:not(.metric):not(.brand):not(.brand-mark):not(.menu):not(.theme-picker){color:inherit;}"
           ".metric{font-size:1.9rem;font-weight:800;letter-spacing:-.03em;}"
           ".mono{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;}"
           ".form-control,.form-select{background:var(--bg-soft)!important;border:1px solid var(--border)!important;color:var(--text)!important;border-radius:14px!important;}"
           ".form-control::placeholder{color:var(--muted)!important;}"
           ".btn-primary{background:linear-gradient(135deg,var(--accent),var(--accent-strong))!important;border:none!important;}"
           ".btn-warning{color:#17212b!important;}"
           ".btn-outline-primary{border-color:var(--accent)!important;color:var(--text)!important;background:transparent!important;}"
           ".btn-outline-primary:hover,.btn-outline-primary:focus{background:rgba(88,166,255,.16)!important;color:var(--text)!important;border-color:var(--accent)!important;}"
           ".btn-outline-secondary{border-color:var(--border)!important;color:var(--text)!important;background:transparent!important;}"
           ".btn-outline-secondary:hover{background:var(--chip)!important;color:var(--text)!important;}"
           ".alert{border-radius:16px!important;border:1px solid var(--border)!important;color:var(--text)!important;background:var(--bg-soft)!important;}"
           ".alert-secondary{background:var(--bg-soft)!important;color:var(--text)!important;border-color:var(--border)!important;}"
           ".alert-warning{background:rgba(255,193,7,.14)!important;color:var(--text)!important;border-color:rgba(255,193,7,.38)!important;}"
           ".alert-danger{background:rgba(220,53,69,.14)!important;color:var(--text)!important;border-color:rgba(220,53,69,.4)!important;}"
           ".alert-success{background:rgba(25,135,84,.14)!important;color:var(--text)!important;border-color:rgba(25,135,84,.36)!important;}"
           ".border,.border.rounded-3,.bg-body-tertiary{border-color:var(--border)!important;background:var(--bg-soft)!important;border-radius:18px!important;color:var(--text)!important;}"
           ".list-group{--bs-list-group-bg:var(--card);--bs-list-group-border-color:var(--border);--bs-list-group-color:var(--text);}"
           ".list-group-item{background:var(--card)!important;color:var(--text)!important;border-color:var(--border)!important;}"
           ".badge.text-bg-primary{background:linear-gradient(135deg,var(--accent),var(--accent-strong))!important;color:#fff!important;}"
           "a{color:var(--accent);}"
           "@media (max-width:820px){.hero-grid{grid-template-columns:1fr;}.topbar{padding:10px 12px;}.menu{width:100%;}}"
           "</style></head><body>";
  shell += "<main class='page-shell'>";
  shell += "<header class='topbar'><div class='brand'><span class='brand-mark'>&#9762;</span><span>Environmental Logger</span></div><nav class='menu'>";
  shell += "<a href='/'" + String(title == "Dashboard" ? " class='active'" : "") + ">Dashboard</a>";
  shell += "<a href='/graphs'" + String(title == "Graphs" ? " class='active'" : "") + ">Graphs</a>";
  shell += "<a href='/config'" + String(title == "Device Configuration" ? " class='active'" : "") + ">Config</a>";
  if (title == "Device Configuration") {
    shell += "<a href='/ota-check'" + String(title == "OTA Status" ? " class='active'" : "") + ">OTA Check</a>";
    shell += "<a href='/update'>OTA Update</a>";
    shell += "<a href='/json'>JSON</a>";
  }
  shell += "<div class='theme-picker'><label for='themeSelect'>Theme</label><select id='themeSelect'><option value='dark'>Dark</option><option value='light'>Light</option></select></div></nav></header>";
  shell += "<section class='hero'><div class='hero-grid'><div><h1>" + htmlEscape(title) + "</h1><p>" + htmlEscape(subtitle) + "</p><div class='hero-meta'><span class='hero-chip'>Device IP <span class='mono'>" + htmlEscape(ipText) + "</span></span><span class='hero-chip'>Hostname <span class='mono'>" + htmlEscape(my_hostname) + "</span></span><span class='hero-chip'><span id='adminUptimeValue' class='mono'>uptime " + htmlEscape(uptimeText) + "</span></span></div></div></div></section>";
  shell += body;
  shell += "<footer style='text-align:center;padding:22px 0 10px;font-size:.77rem;color:var(--muted);border-top:1px solid var(--border);margin-top:28px;'>&copy; 2022&ndash;" + String(ntp.formattedTime("%Y")) + " &middot; By <a href='https://www.don-zalmrol.be/' target='_blank' rel='noopener'>Don Zalmrol</a> &middot; <a href='https://github.com/DonZalmrol' target='_blank' rel='noopener'>GitHub</a> &middot; <a href='#' onclick='document.getElementById(\"changelogDlg\").showModal();return false;' style='color:var(--muted);text-decoration:none;font-family:monospace;'>" + String(FIRMWARE_VERSION) + "</a></footer>";
  shell += changelogDialogHtml();
  shell += "<script>(function(){var key='envLoggerTheme';var select=document.getElementById('themeSelect');var uptimeNode=document.getElementById('adminUptimeValue');var uptimeSeconds=" + String((unsigned long)uptimeSeconds) + ";function formatUptimeText(totalSeconds){var total=Math.max(0,Number(totalSeconds)||0);var days=Math.floor(total/86400);var hours=Math.floor((total%86400)/3600);var minutes=Math.floor((total%3600)/60);return 'uptime '+String(days)+' days, '+String(hours).padStart(2,'0')+':'+String(minutes).padStart(2,'0');}function tickUptime(){if(uptimeNode){uptimeNode.textContent=formatUptimeText(uptimeSeconds);}uptimeSeconds+=60;}function applyTheme(theme){document.body.setAttribute('data-theme',theme);try{localStorage.setItem(key,theme);}catch(error){}if(select&&select.value!==theme){select.value=theme;}try{document.dispatchEvent(new CustomEvent('envLoggerThemeChanged',{detail:{theme:theme}}));}catch(error){}}var saved='dark';try{saved=localStorage.getItem(key)||'dark';}catch(error){}applyTheme(saved);tickUptime();setInterval(tickUptime,60000);if(select){select.addEventListener('change',function(event){applyTheme(event.target.value);});}})();</script>";
  shell += "</main></body></html>";
  return shell;
}

// Convert CPM to uSV
float outputSieverts(float cpmValue)
{
  float uSV = cpmValue * activeTubeConversionFactor;

  return uSV;
}

// Measure Tube voltage through A0
float displayTubeVoltage(void)
{
  float adcInput = 0.0, lowVoltage = 0.0, voltage_offset = 0.0, highVoltage = 0.0;
  adcInput = (float)analogRead(33);

  // ESP32 pin 33 measures from 0-3.3V and has a width of 0-4095
  // Use 3.4 and 4096 as correcting values
  lowVoltage = ((adcInput * 3.4 ) / 4096.0);
  
  /*
    If you want to monitor high voltage on tubes with microcontroller ADC you can use A0 module output.
    ~190-195 conversion factor according to Alex - RH Electronics 2021-03-15
    ~184.097 calcultated on 2022-09-20 for ESP32 Wrover-E after research, see below

    Calculate your value with the "ADC Linearity calculator", yellow is your measured ADC value.
    Measure the voltage between your tubes and multiple x2, measure voltage of pin A0
    
    Measured:
     - ADC = 2863
     - Voltage of tube = 2x 218.8V = 437.6V
     - Voltage of A0 = 2.37V

    Calculated:
     - Voltage should be 2377mV
     - Conversion factor calculation = 437.6V / 2.377 = 184.097
     - Confirm with calculated high voltage table     
  */
  voltage_offset = activeHvCalibrationFactor;
  highVoltage = (lowVoltage * voltage_offset);

  // Uncomment for testing
  //Serial.println("| adcInput = " + String(adcInput, 3));f
  //Serial.println("| A0 LV = " + String(lowVoltage, 3));
  //Serial.println("| Tubes HV = " + String(highVoltage, 3));

  return highVoltage;
}

float calculateHCHO(void)
{
  // Source https://wiki.seeedstudio.com/Grove-HCHO_Sensor/
  float Rs = 0.0, ppm = 0.0;

  // Read the input on analog pin34 (GPIO34)
  int sensorValue = analogRead(34);
  if (sensorValue == 0) return 0.0;
  Rs = (4095.0 / sensorValue) - 1;

  // Calculated R0 = 10.37R on 2022-09-20
  float R0 = activeHchoR0;

  ppm=pow(10.0,((log10(Rs/R0)-0.0827)/(-0.4807)));

  // Uncomment for testing
  //Serial.println("sensorValue = " + String(sensorValue));
  //Serial.println("| R0 = " + String(R0));
  //Serial.println("| RS = " + String(Rs));
  //Serial.println("| ppm = " + String(ppm));

  return ppm;
}

// Contains the logic to start, manage and stop the uploads
static void uploadTaskFunction(void * parameter)
{
  esp_task_wdt_add(NULL);

  UploadSnapshot *snap = (UploadSnapshot *)parameter;

  // Connect to RadMon.org
  connectToRadMonLogger(snap);
  
  // Connect to Uradmonitor
  connectToURadMonLogger(snap);

  delete snap;

  // Display globals are intentionally not reset here — they are written exclusively
  // by Core 0's sensor loop and read by the web server on Core 0. Touching them from
  // Core 1 at this point would introduce a race condition with no functional benefit,
  // since the next sensor cycle will repopulate them.

  esp_task_wdt_delete(NULL);

  //vTaskDelete(uploadTask);
  uploadTask = NULL;
  vTaskDelete(NULL);
}

// Upload data to the RadMon.org server
static void connectToRadMonLogger(UploadSnapshot *snap)
{
  UploadRecord currentUpload = {};
  copyUploadRecord(currentUpload, snap);

  if (!activeRadmonUploadEnabled)
  {
    portENTER_CRITICAL(&uploadStatusMux);
    statusCodeRadmon = UPLOAD_STATUS_DISABLED;
    lastRadmonUpload = currentUpload;
    portEXIT_CRITICAL(&uploadStatusMux);
    VERBOSE_SERIAL_PRINTLN(F("Radmon upload disabled in runtime config; skipping upload."));
    return;
  }

  if ((UserName.length() == 0) || (DataSendingPassWord.length() == 0))
  {
    portENTER_CRITICAL(&uploadStatusMux);
    statusCodeRadmon = UPLOAD_STATUS_MISSING_CONFIG;
    lastRadmonUpload = currentUpload;
    portEXIT_CRITICAL(&uploadStatusMux);
    VERBOSE_SERIAL_PRINTLN(F("Radmon credentials missing; skipping upload."));
    return;
  }

  WiFiClient wifi;
  HttpClient client = HttpClient(wifi, "radmon.org", 80);
  client.setHttpResponseTimeout(RADMON_HTTP_TIMEOUT_MS);

  VERBOSE_SERIAL_PRINTLN(F("Connection to radmon monitoring platform succeeded!"));

  // Concat data for POST
  // API URL
  String ptr = "/radmon.php?function=submit";
  ptr += "&user=";
  ptr += UserName;
  ptr += "&password=";
  ptr += DataSendingPassWord;
  ptr += "&value=";
  ptr += String(snap->cpm);
  ptr += "&unit=CPM";
  //ptr += "&datetime=";
  //ptr += String(epoch);

  // Test output
  VERBOSE_SERIAL_PRINTLN("created PTR = " + ptr);

  client.beginRequest();
  client.post(ptr);
  client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client.sendHeader("Content-Length", 0);  // body is empty; ptr is the URL path, not the body
  client.beginBody();
  client.print("");
  client.endRequest();

  // read the status code and body of the response
  int responseCode = client.responseStatusCode();

  portENTER_CRITICAL(&uploadStatusMux);
  statusCodeRadmon = responseCode;
  lastRadmonUpload = currentUpload;
  portEXIT_CRITICAL(&uploadStatusMux);

  // Added on 2024-05-11 as a test to clean up the connection
  client.flush();

  // Give the client some time to stop
  yield();

  VERBOSE_SERIAL_PRINTLN(F("Connection to radmon monitoring platform Disconnected."));
  VERBOSE_SERIAL_PRINTLN("Status Code Radmon = " + String(responseCode));
  VERBOSE_SERIAL_PRINTLN(F("\n"));
}

// Upload data to the uradmonitor.com server
static void connectToURadMonLogger(UploadSnapshot *snap)
{
  UploadRecord currentUpload = {};
  copyUploadRecord(currentUpload, snap);
  float estimatedDrivePct = estimateHvDriveUploadPct(snap->tubeVoltage);

  if (!activeURadmonUploadEnabled)
  {
    portENTER_CRITICAL(&uploadStatusMux);
    statusCodeURadmon = UPLOAD_STATUS_DISABLED;
    lastURadmonUpload = currentUpload;
    portEXIT_CRITICAL(&uploadStatusMux);
    VERBOSE_SERIAL_PRINTLN(F("uRADMonitor upload disabled in runtime config; skipping upload."));
    return;
  }

  if ((USER_ID.length() == 0) || (USER_KEY.length() == 0) || (DEVICE_ID.length() == 0))
  {
    portENTER_CRITICAL(&uploadStatusMux);
    statusCodeURadmon = UPLOAD_STATUS_MISSING_CONFIG;
    lastURadmonUpload = currentUpload;
    portEXIT_CRITICAL(&uploadStatusMux);
    VERBOSE_SERIAL_PRINTLN(F("uRADMonitor credentials missing; skipping upload."));
    return;
  }

  WiFiClient wifi;
  HttpClient client = HttpClient(wifi, "data.uradmonitor.com", 80);

  VERBOSE_SERIAL_PRINTLN(F("Connection to uradmonitoring platform succeeded!"));
  
  /** Concat data for POST
  * API URL based on the expProtocol.h
  * https://github.com/radhoo/uradmonitor_kit1/blob/master/code/misc/expProtocol.h
  */
  String ptr = "/api/v1/upload/exp";
    ptr += "/01/";                // 01 = mandatory: local time in seconds
    ptr += snap->epoch;           // time epoch (unix) value

    ptr += "/02/";                // 02 = optional: temperature in degrees celsius
    ptr += snap->var_temperature; // temperature value

    ptr += "/03/";                // 03 = optional: barometric pressure in pascals
    ptr += snap->var_pressure;    // pressure value

    ptr += "/04/";                // 04 = optional: humidity as relative humidity in percentage %
    ptr += snap->var_humidity;    // humidity value

    ptr += "/05/";                // 05 = optional: luminosity as relative luminosity in percentage ‰
    ptr += snap->luminosity;      // luminosity value

    ptr += "/06/";                // 06 = optional: VOC (volatile organic compounds) in ohms
    ptr += snap->var_voc;         // VOC value

    ptr += "/07/";                // 07 = optional: CO2 (carbon dioxide) in ppm
    ptr += snap->var_co2;         // CO2 value

    ptr += "/08/";                // 08 = optional: formaldehyde in ppm
    ptr += snap->var_hcho;        // HCHO value

    ptr += "/09/";                // 09 = optional: particulate matter in micro grams per cubic meter
    ptr += snap->var_pm25;        // PM2.5 value

    //ptr += "/0A/";              // 0A = optional: device battery voltage in volts
    //ptr += batteryVoltage;      // battery voltage value

    ptr += "/0B/";                // 0B = optional: radiation measured on geiger tube in cpm
    ptr += snap->cpm;             // cpm value

    ptr += "/0C/";                // 0C = optional: high voltage geiger tube inverter voltage in volts
    ptr += snap->tubeVoltage;     // tube voltage value
    
    ptr += "/0D/";                // 0D = estimated HV drive index in %, derived from measured tube voltage
    ptr += estimatedDrivePct;

    ptr += "/0E/";                // 0E = optional: hardware version
    ptr += "107";                 // hardware value = original 106 | new 107 in 2024

    ptr += "/0F/";                // 0F = optional: software firmware version
    ptr += "124";                 // software value = original 124 | new 124 in 2024

    ptr += "/10/";                // 10 = optional: Tube ID
    ptr += "0x3";                 // 0x3 = GEIGER_TUBE_SBM19  | 0x6 = GEIGER_TUBE_SI22G

    //ptr += "/11/";                // 11 = optional: noise in dB
    //ptr += noiseDb;               // noiseDb = sound value

    ptr += "/12/";                // 12 = optional: particulate matter in micro grams per cubic meter
    ptr += snap->var_pm01;         // var_pm01 value

    ptr += "/13/";                // 13 = optional: particulate matter in micro grams per cubic meter
    ptr += snap->var_pm10;         // var_pm10 value

    //ptr += "/14/";                // 14 = optional: ozone in ppb
    //ptr += ozoneValue;            // ozoneValue = ozone value
  
  // Test output
  VERBOSE_SERIAL_PRINTLN("created EXP code = " + ptr);

  client.beginRequest();
  client.post(ptr);
  client.sendHeader("X-User-id", USER_ID.c_str());
  client.sendHeader("X-User-hash", USER_KEY.c_str());
  client.sendHeader("X-Device-id", DEVICE_ID.c_str());
  client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client.sendHeader("Content-Length", 0);  // body is empty; ptr is the URL path, not the body
  client.beginBody();
  client.print("");
  client.endRequest();

  // read the status code and body of the response
  int responseCode = client.responseStatusCode();

  portENTER_CRITICAL(&uploadStatusMux);
  statusCodeURadmon = responseCode;
  lastURadmonUpload = currentUpload;
  portEXIT_CRITICAL(&uploadStatusMux);

  // Added on 2024-05-11 as a test to clean up the connection
  client.flush();

  // Give the client some time to stop
  yield();

  VERBOSE_SERIAL_PRINTLN(F("Connection to uradmonitoring platform Disconnected."));
  VERBOSE_SERIAL_PRINTLN("Status Code URadmon = " + String(responseCode));
  VERBOSE_SERIAL_PRINTLN(F("\n"));
}

static void scanI2C(void)
{
  byte error, address;
  int nDevices;
  
  Serial.println("Scanning for I2C devices...");
  
  nDevices = 0;
  
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      
      if (address<16)
      {
        Serial.print("0");
      }
      
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      
      if (address<16)
      {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("Finished scanning for I2C devices\n");
  }

  i2cDevicesFound = nDevices;

  esp_task_wdt_reset();
  delay(5000);          
  esp_task_wdt_reset();
}

// Load latest data from EEPROM
static void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
  {
    // Existing state in EEPROM
    Serial.println(F("Reading state from EEPROM"));

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      bsecState[i] = EEPROM.read(i + 1);
      // Uncomment below to see to-be EEPROM output
      //Serial.println(bsecState[i], HEX);
    }
    iaqSensor.setState(bsecState);
    iaqSensor.getState(bsecState);
  }
  
  else
  {
    // Erase the EEPROM with zeroes
    Serial.println(F("Erasing EEPROM"));

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
    {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
  }
}

// Save data to EEPROM
static void updateStateFunction(void * parameter)
{
  esp_task_wdt_add(NULL);

  StateSnapshot *ss = (StateSnapshot *)parameter;

  bool updateEEPROM = false;
  unsigned long currentMillis = millis();
  
  /* 
    Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 
    0 = Stabilizing
    1 = Uncertain
    2 = Calibrating
    3 = Calibrated
  */
  if (lastStateSaveMillis == 0)
  {
    if (ss->iaqAccuracy >= 3)
    {
      updateEEPROM = true;
    }
  }
  
  else
  {
    if (((currentMillis - lastStateSaveMillis) >= STATE_SAVE_PERIOD) && (ss->iaq >= 25.0f))
    {
      updateEEPROM = true;
    }

    else
    {
      updateEEPROM = false;
    }
  }

  if (updateEEPROM)
  {
    iaqSensor.getState(bsecState);
    Serial.println(F("Saving state to EEPROM"));
    lastStateSaveMillis = currentMillis;

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++)
    {
      EEPROM.write(i + 1, bsecState[i]);
      // Uncomment below to see to-be EEPROM input
      //Serial.println(bsecState[i], HEX);
    }
    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }

  //else
  //{
    //Serial.println(F("Not saved state to EEPROM"));
  //}

  delete ss;
  
  esp_task_wdt_delete(NULL);

  //vTaskDelete(updateState);
  updateState = NULL;
  vTaskDelete(NULL); 
}

static bool Init_PulseCounter_01(void)
{
  // Configure pulse counter unit 0
  pcnt_unit_config_t unit_config = {
    .low_limit = -PCNT_H_LIM_VAL,
    .high_limit = PCNT_H_LIM_VAL,
  };
  if (!checkEspOk("pcnt_new_unit tube 1", pcnt_new_unit(&unit_config, &pcnt_unit_01)))
  {
    return false;
  }

  // Configure the IO pin: GPIO13 as edge signal input
  pcnt_chan_config_t chan_config = {
    .edge_gpio_num = PCNT_INPUT_SIG_IO_01,
    .level_gpio_num = -1,
  };
  if (!checkEspOk("pcnt_new_channel tube 1", pcnt_new_channel(pcnt_unit_01, &chan_config, &pcnt_chan_01)))
  {
    return false;
  }

  // Configure the GPIO input that counts positive edges only
  if (!checkEspOk(
    "pcnt_channel_set_edge_action tube 1",
    pcnt_channel_set_edge_action(
      pcnt_chan_01,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // count on rising edge
      PCNT_CHANNEL_EDGE_ACTION_HOLD        // do nothing on falling edge
    )))
  {
    return false;
  }

  // Set glitch filter (debounce): ignore pulses narrower than 1000 ns
  pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
  };
  if (!checkEspOk("pcnt_unit_set_glitch_filter tube 1", pcnt_unit_set_glitch_filter(pcnt_unit_01, &filter_config)))
  {
    return false;
  }

  // Register ISR callback for high limit event
  pcnt_event_callbacks_t cbs = {
    .on_reach = Counter_ISR,
  };
  if (!checkEspOk("pcnt_unit_register_event_callbacks tube 1", pcnt_unit_register_event_callbacks(pcnt_unit_01, &cbs, NULL)))
  {
    return false;
  }

  // Trigger callback when counter reaches the high watch point
  if (!checkEspOk("pcnt_unit_add_watch_point tube 1", pcnt_unit_add_watch_point(pcnt_unit_01, PCNT_H_LIM_VAL)))
  {
    return false;
  }

  // Enable and start the counter
  if (!checkEspOk("pcnt_unit_enable tube 1", pcnt_unit_enable(pcnt_unit_01))
      || !checkEspOk("pcnt_unit_clear_count tube 1", pcnt_unit_clear_count(pcnt_unit_01))
      || !checkEspOk("pcnt_unit_start tube 1", pcnt_unit_start(pcnt_unit_01)))
  {
    return false;
  }
  
  Serial.println(F("PCNT_01 Init Completed"));
  return true;
}

static bool Init_PulseCounter_02(void)
{
  // Configure pulse counter unit 1
  pcnt_unit_config_t unit_config = {
    .low_limit = -PCNT_H_LIM_VAL,
    .high_limit = PCNT_H_LIM_VAL,
  };
  if (!checkEspOk("pcnt_new_unit tube 2", pcnt_new_unit(&unit_config, &pcnt_unit_02)))
  {
    return false;
  }

  // Configure the IO pin: GPIO14 as edge signal input
  pcnt_chan_config_t chan_config = {
    .edge_gpio_num = PCNT_INPUT_SIG_IO_02,
    .level_gpio_num = -1,
  };
  if (!checkEspOk("pcnt_new_channel tube 2", pcnt_new_channel(pcnt_unit_02, &chan_config, &pcnt_chan_02)))
  {
    return false;
  }

  // Configure the GPIO input that counts positive edges only
  if (!checkEspOk(
    "pcnt_channel_set_edge_action tube 2",
    pcnt_channel_set_edge_action(
      pcnt_chan_02,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // count on rising edge
      PCNT_CHANNEL_EDGE_ACTION_HOLD        // do nothing on falling edge
    )))
  {
    return false;
  }

  // Set glitch filter (debounce): ignore pulses narrower than 1000 ns
  pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
  };
  if (!checkEspOk("pcnt_unit_set_glitch_filter tube 2", pcnt_unit_set_glitch_filter(pcnt_unit_02, &filter_config)))
  {
    return false;
  }

  // Register ISR callback for high limit event
  pcnt_event_callbacks_t cbs = {
    .on_reach = Counter_ISR,
  };
  if (!checkEspOk("pcnt_unit_register_event_callbacks tube 2", pcnt_unit_register_event_callbacks(pcnt_unit_02, &cbs, NULL)))
  {
    return false;
  }

  // Trigger callback when counter reaches the high watch point
  if (!checkEspOk("pcnt_unit_add_watch_point tube 2", pcnt_unit_add_watch_point(pcnt_unit_02, PCNT_H_LIM_VAL)))
  {
    return false;
  }

  // Enable and start the counter
  if (!checkEspOk("pcnt_unit_enable tube 2", pcnt_unit_enable(pcnt_unit_02))
      || !checkEspOk("pcnt_unit_clear_count tube 2", pcnt_unit_clear_count(pcnt_unit_02))
      || !checkEspOk("pcnt_unit_start tube 2", pcnt_unit_start(pcnt_unit_02)))
  {
    return false;
  }
  
  Serial.println(F("PCNT_02 Init Completed"));
  return true;
}

// Function to clean the Counter and its variables
static void Clean_Counters()                                       
{
  if (!pulseCountersReady)
  {
    return;
  }

  // Stop pulse counters to clear them
  checkEspOk("pcnt_unit_stop tube 1", pcnt_unit_stop(pcnt_unit_01));
  checkEspOk("pcnt_unit_stop tube 2", pcnt_unit_stop(pcnt_unit_02));

  // Clear the count
  checkEspOk("pcnt_unit_clear_count tube 1", pcnt_unit_clear_count(pcnt_unit_01));
  checkEspOk("pcnt_unit_clear_count tube 2", pcnt_unit_clear_count(pcnt_unit_02));

  // Restart pulse counters
  checkEspOk("pcnt_unit_start tube 1", pcnt_unit_start(pcnt_unit_01));
  checkEspOk("pcnt_unit_start tube 2", pcnt_unit_start(pcnt_unit_02));
}

void handleRootPath()
{
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  // Stream the page in chunks to avoid allocating one giant String on the heap
  for (const String &chunk : webPageChunks())
  {
    server.sendContent(chunk);
  }
  server.sendContent(""); // signal end of chunked response
}

void handleJsonPath()
{
  server.send(200, "application/json", JsonPage());
}

void handleConfigPath()
{
  String notice;
  String noticeClass = "success";
  bool wifiPasswordStored = (my_password.length() > 0);
  bool radmonPasswordStored = (DataSendingPassWord.length() > 0);
  bool uradKeyStored = (USER_KEY.length() > 0);
  bool serialVerboseChecked = serialVerboseEnabled;
  bool radmonUploadEnabledChecked = activeRadmonUploadEnabled;
  bool uradmonUploadEnabledChecked = activeURadmonUploadEnabled;
  String displayedWifiSsid = my_ssid;
  String displayedTubePresetId = activeTubePresetId;
  String displayedTubeDeadTimeUs = String(activeTubeDeadTimeSeconds * 1000000.0f, 3);
  String displayedTubeConversionFactor = String(activeTubeConversionFactor, 6);
  String displayedTubeVoltageRange = formatOperatingVoltageRange(activeTubeOperatingVoltageMin, activeTubeOperatingVoltageMax);
  String displayedTubeNote = activeTubePresetNote;
  String displayedTimezoneOffset = String(activeTimezoneOffsetMinutes);
  String displayedDstProfile = activeDstProfileId;
  String displayedDstOffset = String(activeDstOffsetMinutes);
  String displayedHvCalibrationFactor = String(activeHvCalibrationFactor, 3);
  String displayedHchoR0 = String(activeHchoR0, 3);
  String displayedHistoryRetentionHours = String((unsigned long)getHistoryRetentionHours());
  String displayedNtpServer = activeNtpServer;
  String displayedCpmGaugeFullScale = String(activeCpmGaugeFullScale);
  String displayedStationName = activeStationName;

  if ((server.method() == HTTP_GET) && server.hasArg("wifi_ssid_pick"))
  {
    displayedWifiSsid = server.arg("wifi_ssid_pick");
    displayedWifiSsid.trim();
    if (displayedWifiSsid.length() > 0)
    {
      notice = "Selected WiFi network copied into the SSID field below. Save Settings or Save and Reboot to apply it.";
    }
  }

  if (server.method() == HTTP_POST)
  {
    displayedTubePresetId = normalizeTubePresetId(server.arg("tube_preset"));
    displayedTubeDeadTimeUs = server.arg("tube_dead_time_us");
    displayedTubeConversionFactor = server.arg("tube_conversion_factor");
    displayedTimezoneOffset = server.arg("timezone_offset_minutes");
    displayedDstProfile = normalizeDstProfileId(server.arg("dst_profile"));
    displayedDstOffset = server.arg("dst_offset_minutes");
    displayedHvCalibrationFactor = server.arg("hv_calibration_factor");
    displayedHchoR0 = server.arg("hcho_r0");
    displayedHistoryRetentionHours = server.arg("history_retention_hours");
    displayedNtpServer = server.arg("ntp_server");
    displayedCpmGaugeFullScale = server.arg("cpm_gauge_full_scale");
    displayedStationName = server.arg("station_name");
    displayedTubeDeadTimeUs.trim();
    displayedTubeConversionFactor.trim();
    displayedTimezoneOffset.trim();
    displayedDstOffset.trim();
    displayedHvCalibrationFactor.trim();
    displayedHchoR0.trim();
    displayedHistoryRetentionHours.trim();
    displayedNtpServer.trim();
    displayedCpmGaugeFullScale.trim();
    displayedStationName.trim();

    if (displayedTubeDeadTimeUs.length() == 0)
    {
      displayedTubeDeadTimeUs = String(activeTubeDeadTimeSeconds * 1000000.0f, 3);
    }
    if (displayedTubeConversionFactor.length() == 0)
    {
      displayedTubeConversionFactor = String(activeTubeConversionFactor, 6);
    }

    if (saveRuntimeSettingsFromRequest(notice, noticeClass))
    {
      wifiPasswordStored = (my_password.length() > 0);
      radmonPasswordStored = (DataSendingPassWord.length() > 0);
      uradKeyStored = (USER_KEY.length() > 0);
      serialVerboseChecked = serialVerboseEnabled;
      radmonUploadEnabledChecked = activeRadmonUploadEnabled;
      uradmonUploadEnabledChecked = activeURadmonUploadEnabled;
      displayedTubePresetId = activeTubePresetId;
      displayedTubeDeadTimeUs = String(activeTubeDeadTimeSeconds * 1000000.0f, 3);
      displayedTubeConversionFactor = String(activeTubeConversionFactor, 6);
      displayedTubeVoltageRange = formatOperatingVoltageRange(activeTubeOperatingVoltageMin, activeTubeOperatingVoltageMax);
      displayedTubeNote = activeTubePresetNote;
      displayedTimezoneOffset = String(activeTimezoneOffsetMinutes);
      displayedDstProfile = activeDstProfileId;
      displayedDstOffset = String(activeDstOffsetMinutes);
      displayedHvCalibrationFactor = String(activeHvCalibrationFactor, 3);
      displayedHchoR0 = String(activeHchoR0, 3);
      displayedHistoryRetentionHours = String((unsigned long)getHistoryRetentionHours());
      displayedNtpServer = activeNtpServer;
      displayedCpmGaugeFullScale = String(activeCpmGaugeFullScale);
      displayedStationName = activeStationName;
    }
  }

  String body;
  body.reserve(8600);
  if (notice.length() > 0)
  {
    body += "<div class='alert alert-" + noticeClass + "' role='alert'>" + htmlEscape(notice) + "</div>";
  }

  body += "<div class='row g-4'>";
  body += "<div class='col-12 col-xl-8'><div class='card'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>Runtime Configuration</h2>";
  body += "<p class='hint mb-4'>Values saved here override the compiled defaults. Text fields fall back to <span class='mono'>arduino_secrets.h</span> when left blank, numeric fields fall back to their compiled firmware defaults when cleared, and secret fields stay unchanged unless you enter a replacement or tick their reset box.</p>";
  body += "<form method='post' action='/config' class='row g-3'>";

  body += "<div class='col-12'><h3 class='h6 text-uppercase text-body-secondary mb-1'>General</h3></div>";
  body += "<div class='col-md-6'><label class='form-label' for='station_name'>Station Name</label><input class='form-control' id='station_name' name='station_name' value='" + htmlEscape(displayedStationName) + "'><div class='form-text'>Label shown in the dashboard title. Leave blank to restore the default.</div></div>";
  body += "<div class='col-12'><h3 class='h6 text-uppercase text-body-secondary mb-1'>WiFi</h3></div>";
  body += "<div class='col-md-6'><label class='form-label' for='wifi_ssid'>SSID</label><input class='form-control' id='wifi_ssid' name='wifi_ssid' value='" + htmlEscape(displayedWifiSsid) + "'></div>";
  body += "<div class='col-md-6'><label class='form-label' for='wifi_password'>Password</label><input class='form-control' type='password' id='wifi_password' name='wifi_password' value='' placeholder='" + String(wifiPasswordStored ? "Stored value masked" : "Using default or empty") + "'><div class='form-text'>Leave blank to keep the current password.</div><div class='form-check mt-2'><input class='form-check-input' type='checkbox' id='wifi_password_reset' name='wifi_password_reset'><label class='form-check-label' for='wifi_password_reset'>Reset to compiled default</label></div></div>";
  body += "<div class='col-md-6'><label class='form-label' for='wifi_hostname'>Hostname</label><input class='form-control' id='wifi_hostname' name='wifi_hostname' value='" + htmlEscape(my_hostname) + "'></div>";
  body += "<div class='col-12'><div class='border rounded-3 p-3 bg-body-tertiary'><div class='d-flex flex-column flex-lg-row justify-content-between gap-3 align-items-lg-center'><div><div class='fw-semibold'>Nearby WiFi Networks</div><div class='hint'>Open the scanner page, pick an SSID there, and it will return here with the WiFi field prefilled.</div></div><a class='btn btn-outline-primary' href='/wifi-scan-page'>Open WiFi Scanner</a></div><div class='hint mt-3'>This flow uses a dedicated page instead of a large embedded script so scanning remains reliable on lightweight browsers and ESP32-hosted pages.</div></div></div>";

  body += "<div class='col-12 pt-2'><h3 class='h6 text-uppercase text-body-secondary mb-1'>radmon.org</h3></div>";
  body += "<div class='col-12'><div class='form-check form-switch'><input class='form-check-input' type='checkbox' id='radmon_upload_enabled' name='radmon_upload_enabled'" + String(radmonUploadEnabledChecked ? " checked" : "") + "><label class='form-check-label' for='radmon_upload_enabled'>Enable Radmon uploads</label><div class='form-text'>When disabled, the device keeps sampling normally but skips Radmon submissions completely.</div></div></div>";
  body += "<div class='col-md-6'><label class='form-label' for='radmon_user'>User Name</label><input class='form-control' id='radmon_user' name='radmon_user' value='" + htmlEscape(UserName) + "'></div>";
  body += "<div class='col-md-6'><label class='form-label' for='radmon_password'>Password</label><input class='form-control' type='password' id='radmon_password' name='radmon_password' value='' placeholder='" + String(radmonPasswordStored ? "Stored value masked" : "Using default or empty") + "'><div class='form-text'>Leave blank to keep the current password.</div><div class='form-check mt-2'><input class='form-check-input' type='checkbox' id='radmon_password_reset' name='radmon_password_reset'><label class='form-check-label' for='radmon_password_reset'>Reset to compiled default</label></div></div>";

  body += "<div class='col-12 pt-2'><h3 class='h6 text-uppercase text-body-secondary mb-1'>uRADMonitor</h3></div>";
  body += "<div class='col-12'><div class='form-check form-switch'><input class='form-check-input' type='checkbox' id='urad_upload_enabled' name='urad_upload_enabled'" + String(uradmonUploadEnabledChecked ? " checked" : "") + "><label class='form-check-label' for='urad_upload_enabled'>Enable uRADMonitor uploads</label><div class='form-text'>When disabled, the device keeps local logging and web features active but skips uRADMonitor submissions.</div></div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='urad_user_id'>User ID</label><input class='form-control' id='urad_user_id' name='urad_user_id' value='" + htmlEscape(USER_ID) + "'></div>";
  body += "<div class='col-md-4'><label class='form-label' for='urad_user_key'>User Key</label><input class='form-control' type='password' id='urad_user_key' name='urad_user_key' value='' placeholder='" + String(uradKeyStored ? "Stored value masked" : "Using default or empty") + "'><div class='form-text'>Leave blank to keep the current key.</div><div class='form-check mt-2'><input class='form-check-input' type='checkbox' id='urad_user_key_reset' name='urad_user_key_reset'><label class='form-check-label' for='urad_user_key_reset'>Reset to compiled default</label></div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='urad_device_id'>Device ID</label><input class='form-control' id='urad_device_id' name='urad_device_id' value='" + htmlEscape(DEVICE_ID) + "'></div>";

  body += "<div class='col-12 pt-2'><h3 class='h6 text-uppercase text-body-secondary mb-1'>Time and Region</h3></div>";
  body += "<div class='col-md-4'><label class='form-label' for='timezone_offset_minutes'>UTC Offset (minutes)</label><input class='form-control mono' id='timezone_offset_minutes' name='timezone_offset_minutes' value='" + htmlEscape(displayedTimezoneOffset) + "'><div class='form-text'>Examples: 60 for CET, 0 for UTC, -300 for EST.</div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='dst_profile'>DST Profile</label><select class='form-select' id='dst_profile' name='dst_profile'>" + buildDstProfileOptionsHtml(displayedDstProfile) + "</select><div class='form-text'>Choose the daylight-saving rule set that matches your region.</div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='dst_offset_minutes'>DST Shift (minutes)</label><input class='form-control mono' id='dst_offset_minutes' name='dst_offset_minutes' value='" + htmlEscape(displayedDstOffset) + "'><div class='form-text'>Usually 60 minutes. Set 0 or choose no automatic DST to disable shifting.</div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='ntp_server'>NTP Server</label><input class='form-control mono' id='ntp_server' name='ntp_server' value='" + htmlEscape(displayedNtpServer) + "'><div class='form-text'>Hostname of the time server. Default: pool.ntp.org. Leave blank to restore default.</div></div>";
  body += "<div class='col-12'><div class='border rounded-3 p-3 bg-body-tertiary'><div class='fw-semibold mb-1'>Current time policy</div><div class='hint'>Local clock uses <span class='mono'>" + htmlEscape(formatUtcOffsetMinutes(activeTimezoneOffsetMinutes)) + "</span> with <span class='mono'>" + htmlEscape(dstProfileLabel(activeDstProfileId)) + "</span> and a <span class='mono'>" + htmlEscape(String(activeDstProfileId == DST_PROFILE_NONE_ID ? 0 : activeDstOffsetMinutes)) + " min</span> DST shift.</div></div></div>";

  body += "<div class='col-12 pt-2'><h3 class='h6 text-uppercase text-body-secondary mb-1'>Diagnostics</h3></div>";
  body += "<div class='col-12'><div class='form-check form-switch'><input class='form-check-input' type='checkbox' id='serial_verbose' name='serial_verbose'" + String(serialVerboseChecked ? " checked" : "") + "><label class='form-check-label' for='serial_verbose'>Verbose serial logging</label><div class='form-text'>When ON: per-second CPS readings, 61-second sensor dumps, and upload traces are printed to the serial port. When OFF: only warnings and errors are printed.</div></div></div>";

  body += "<div class='col-12 pt-2'><h3 class='h6 text-uppercase text-body-secondary mb-1'>Calibration and Logging</h3></div>";
  body += "<div class='col-md-4'><label class='form-label' for='hv_calibration_factor'>Tube HV Calibration</label><input class='form-control mono' id='hv_calibration_factor' name='hv_calibration_factor' value='" + htmlEscape(displayedHvCalibrationFactor) + "'><div class='form-text'>ADC-to-HV multiplier used by the tube voltage display.</div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='hcho_r0'>HCHO R0</label><input class='form-control mono' id='hcho_r0' name='hcho_r0' value='" + htmlEscape(displayedHchoR0) + "'><div class='form-text'>Sensor calibration value used by the Grove HCHO conversion formula.</div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='history_retention_hours'>Historic Storage (hours)</label><input class='form-control mono' id='history_retention_hours' name='history_retention_hours' value='" + htmlEscape(displayedHistoryRetentionHours) + "'><div class='form-text'>Approximate Graphs history retained on SPIFFS. Default is 1 hour.</div></div>";
  body += "<div class='col-md-4'><label class='form-label' for='cpm_gauge_full_scale'>CPM Gauge Full-Scale</label><input class='form-control mono' id='cpm_gauge_full_scale' name='cpm_gauge_full_scale' value='" + htmlEscape(displayedCpmGaugeFullScale) + "'><div class='form-text'>CPM at which the radiation gauge reads 100%. Match to your tube type. Default: 600 CPM.</div></div>";
  body += "<div class='col-12'><div class='alert alert-warning mb-0'>Warning: increasing historic storage beyond 24 hours can fill the entire SPIFFS partition on this build.</div></div>";
  body += "<div class='col-12'><div class='hint'>Clear a numeric field if you want the firmware to return to its compiled default for that item.</div></div>";

  body += "<div class='col-12 pt-2'><h3 class='h6 text-uppercase text-body-secondary mb-1'>Radiation Tube Setup</h3></div>";
  body += "<div class='col-md-6'><label class='form-label' for='tube_preset'>Tube Type</label><select class='form-select' id='tube_preset' name='tube_preset'>" + buildTubePresetOptionsHtml(displayedTubePresetId) + "</select><div id='tubePresetHint' class='form-text'>Preset values are loaded from logger_user_config.h. Choose Custom to enter your own values below.</div></div>";
  body += "<div class='col-md-3'><label class='form-label' for='tube_dead_time_us'>Dead Time (us)</label><input class='form-control mono' id='tube_dead_time_us' name='tube_dead_time_us' value='" + htmlEscape(displayedTubeDeadTimeUs) + "'><div class='form-text'>Used for pulse dead-time correction.</div></div>";
  body += "<div class='col-md-3'><label class='form-label' for='tube_conversion_factor'>Conversion Factor</label><input class='form-control mono' id='tube_conversion_factor' name='tube_conversion_factor' value='" + htmlEscape(displayedTubeConversionFactor) + "'><div class='form-text'>Dose conversion in uSv/h per CPM.</div></div>";
  body += "<div class='col-12'><div class='border rounded-3 p-3 bg-body-tertiary'><div class='fw-semibold mb-1'>Active tube profile</div><div class='hint'>Current runtime selection: <span class='mono' id='tubeProfileName'>" + htmlEscape(activeTubePresetLabel) + "</span><br>Operating range: <span class='mono' id='tubeVoltageRange'>" + htmlEscape(displayedTubeVoltageRange) + "</span><br><span id='tubeProfileNote'>" + htmlEscape(displayedTubeNote) + "</span><br>Preset values stay in the dedicated header, while Custom values are saved in NVS so end users can adjust them without rebuilding firmware.</div></div></div>";

  body += "<div class='col-12 d-flex flex-wrap gap-2 pt-2'><button type='submit' class='btn btn-primary'>Save Settings</button><button type='submit' formaction='/reboot' formmethod='post' class='btn btn-warning'>Save and Reboot</button><button type='submit' formaction='/restart' formmethod='post' class='btn btn-outline-secondary'>Reboot Only</button><a class='btn btn-outline-secondary' href='/ota-check'>Review OTA Status</a><a class='btn btn-outline-secondary' href='/update'>Open OTA Update</a></div>";
  body += "</form></div></div></div>";

  body += "<div class='col-12 col-xl-4'><div class='card'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>Notes</h2>";
  body += "<ul class='mb-0 ps-3 hint'><li>WiFi and hostname changes need a reboot before they take effect.</li><li>Timezone, DST, calibration, and history settings are saved in NVS and can be adjusted without rebuilding the firmware.</li><li>Upload platforms can now be disabled independently from this page if you only want local monitoring.</li><li>Credentials are stored in NVS on the device and are no longer limited to build-time secrets.</li><li>Tube presets live in <span class='mono'>logger_user_config.h</span> so builders can ship cleaner defaults.</li><li>The compiled <span class='mono'>arduino_secrets.h</span> values remain as fallback defaults.</li><li>The OTA update page is still served by the ElegantOTA library.</li></ul>";
  body += "</div></div></div></div>";

  body += "<script>(function(){var preset=document.getElementById('tube_preset');var dead=document.getElementById('tube_dead_time_us');var conv=document.getElementById('tube_conversion_factor');var hint=document.getElementById('tubePresetHint');var voltage=document.getElementById('tubeVoltageRange');var note=document.getElementById('tubeProfileNote');var profileName=document.getElementById('tubeProfileName');var dstProfile=document.getElementById('dst_profile');var dstOffset=document.getElementById('dst_offset_minutes');if(!preset||!dead||!conv){return;}function readVoltageRange(option){var vmin=option?option.getAttribute('data-vmin'):'';var vmax=option?option.getAttribute('data-vmax'):'';if(!vmin||!vmax){return 'Verify against datasheet';}return vmin+'-'+vmax+' V';}function syncTubeFields(){var option=preset.options[preset.selectedIndex];var isCustom=preset.value==='custom';dead.readOnly=!isCustom;conv.readOnly=!isCustom;dead.setAttribute('aria-readonly',isCustom?'false':'true');conv.setAttribute('aria-readonly',isCustom?'false':'true');if(!isCustom&&option){dead.value=option.getAttribute('data-dead-us')||dead.value;conv.value=option.getAttribute('data-conv')||conv.value;}if(hint){hint.textContent=isCustom?'Custom keeps the values below editable and saves them in NVS.':'Preset values are loaded from logger_user_config.h and copied into the fields automatically.';}if(voltage){voltage.textContent=readVoltageRange(option);}if(note&&option){note.textContent=option.getAttribute('data-note')||'';}if(profileName&&option){profileName.textContent=option.text.split(' (')[0]||option.text;}}function syncDstFields(){if(!dstProfile||!dstOffset){return;}var disabled=dstProfile.value==='none';dstOffset.readOnly=disabled;dstOffset.setAttribute('aria-readonly',disabled?'true':'false');if(disabled&&(!dstOffset.value||dstOffset.value==='')){dstOffset.value='0';}}preset.addEventListener('change',syncTubeFields);if(dstProfile){dstProfile.addEventListener('change',syncDstFields);}syncTubeFields();syncDstFields();})();</script>";

  server.send(200, "text/html; charset=utf-8", adminPageShell("Device Configuration", "Edit runtime WiFi and upload credentials stored on the ESP32.", body));
}

void handleWifiScanPagePath()
{
  bool forceRescan = server.hasArg("rescan");
  int networkCount = WiFi.scanComplete();

  if (forceRescan && (networkCount >= 0))
  {
    WiFi.scanDelete();
    networkCount = -1;
  }

  if (networkCount == WIFI_SCAN_RUNNING)
  {
    String body;
    body.reserve(1200);
    body += "<div class='card'><div class='card-body'>";
    body += "<h2 class='h5 mb-3'>Scanning nearby networks</h2>";
    body += "<div class='alert alert-secondary'>The ESP32 radio is scanning for nearby SSIDs right now. This page will refresh automatically.</div>";
    body += "<div class='d-flex flex-wrap gap-2'><a class='btn btn-primary' href='/wifi-scan-page'>Refresh Results</a><a class='btn btn-outline-secondary' href='/config'>Back to Config</a></div>";
    body += "</div></div>";
    body += "<script>setTimeout(function(){window.location.replace('/wifi-scan-page');},1500);</script>";
    server.send(200, "text/html; charset=utf-8", adminPageShell("WiFi Scanner", "Scanning for nearby WiFi networks using a dedicated page flow.", body));
    return;
  }

  if (networkCount < 0)
  {
    WiFi.scanDelete();
    WiFi.scanNetworks(true, true);

    String body;
    body.reserve(1200);
    body += "<div class='card'><div class='card-body'>";
    body += "<h2 class='h5 mb-3'>Starting WiFi scan</h2>";
    body += "<div class='alert alert-secondary'>The scan has started. This page will refresh automatically when the ESP32 has results ready.</div>";
    body += "<div class='d-flex flex-wrap gap-2'><a class='btn btn-primary' href='/wifi-scan-page'>Refresh Results</a><a class='btn btn-outline-secondary' href='/config'>Back to Config</a></div>";
    body += "</div></div>";
    body += "<script>setTimeout(function(){window.location.replace('/wifi-scan-page');},1500);</script>";
    server.send(200, "text/html; charset=utf-8", adminPageShell("WiFi Scanner", "Scanning for nearby WiFi networks using a dedicated page flow.", body));
    return;
  }

  String body;
  body.reserve(4200);
  body += "<div class='row g-4'>";
  body += "<div class='col-12 col-xl-4'><div class='card h-100'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>Nearby WiFi Networks</h2>";
  body += "<div class='metric'>" + String(networkCount) + "</div>";
  body += "<div class='hint mt-2'>Pick a visible SSID below. The button returns to the config page with that SSID already copied into the WiFi field.</div>";
  body += "<div class='d-flex flex-wrap gap-2 mt-3'><a class='btn btn-primary' href='/wifi-scan-page?rescan=1'>Scan Again</a><a class='btn btn-outline-secondary' href='/config'>Back to Config</a></div>";
  body += "</div></div></div>";
  body += "<div class='col-12 col-xl-8'><div class='card h-100'><div class='card-body'>";
  body += buildWifiScanResultsHtml(networkCount, true);
  body += "</div></div></div></div>";
  server.send(200, "text/html; charset=utf-8", adminPageShell("WiFi Scanner", "Select a nearby SSID and send it back into the configuration form.", body));
}

void handleWifiScanPath()
{
  const String action = server.arg("action");

  if (action == "start")
  {
    int scanState = WiFi.scanComplete();
    if (scanState >= 0)
    {
      WiFi.scanDelete();
    }

    if (scanState == WIFI_SCAN_RUNNING)
    {
      server.send(202, "text/html; charset=utf-8", "<div class='alert alert-secondary mb-0'>A WiFi scan is already running. Waiting for results...</div>");
      return;
    }

    WiFi.scanNetworks(true, true);
    server.send(202, "text/html; charset=utf-8", "<div class='alert alert-secondary mb-0'>WiFi scan started. Waiting for nearby network results...</div>");
    return;
  }

  int networkCount = WiFi.scanComplete();
  if (networkCount == WIFI_SCAN_RUNNING)
  {
    server.send(202, "text/html; charset=utf-8", "<div class='alert alert-secondary mb-0'>Scanning nearby WiFi networks...</div>");
    return;
  }

  if (networkCount < 0)
  {
    server.send(503, "text/html; charset=utf-8", "<div class='alert alert-danger mb-0'>WiFi scan failed. Try again in a moment or enter the SSID manually.</div>");
    return;
  }

  server.send(200, "text/html; charset=utf-8", buildWifiScanResultsHtml(networkCount, false));
}

void handleHistoryCsvPath()
{
  if (!spiffsMounted)
  {
    server.send(503, "text/plain; charset=utf-8", "SPIFFS unavailable");
    return;
  }

  if (!SPIFFS.exists(HISTORY_LOG_PATH))
  {
    server.send(200, "text/csv; charset=utf-8", String(HISTORY_LOG_HEADER) + "\n");
    return;
  }

  File historyFile = SPIFFS.open(HISTORY_LOG_PATH, FILE_READ);
  if (!historyFile)
  {
    server.send(500, "text/plain; charset=utf-8", "Failed to open history CSV");
    return;
  }

  server.streamFile(historyFile, "text/csv; charset=utf-8");
  historyFile.close();
}

void handleHistoryDeletePath()
{
  String body;
  body.reserve(700);

  if (!spiffsMounted)
  {
    body += "<div class='alert alert-danger mb-0'>SPIFFS is unavailable, so the history CSV could not be deleted.</div>";
    server.send(200, "text/html; charset=utf-8", adminPageShell("History Delete", "Deleting the retained local history CSV.", body));
    return;
  }

  bool removedHistory = !SPIFFS.exists(HISTORY_LOG_PATH) || SPIFFS.remove(HISTORY_LOG_PATH);
  bool removedTemp = !SPIFFS.exists(HISTORY_LOG_TMP_PATH) || SPIFFS.remove(HISTORY_LOG_TMP_PATH);
  if (removedHistory)
  {
    historyRowCount = 0;
  }

  body += "<div class='card'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>History CSV Cleanup</h2>";
  if (removedHistory && removedTemp)
  {
    body += "<div class='alert alert-success mb-3'>The retained history CSV was deleted successfully. New samples will be written again on the next 61-second cycle.</div>";
  }
  else
  {
    body += "<div class='alert alert-danger mb-3'>The retained history CSV could not be deleted completely. Check SPIFFS state and try again.</div>";
  }
  body += "<div class='d-flex flex-wrap gap-2'><a class='btn btn-primary' href='/graphs'>Back to Graphs</a><a class='btn btn-outline-secondary' href='/history.csv'>Open CSV</a></div>";
  body += "</div></div>";

  server.send(200, "text/html; charset=utf-8", adminPageShell("History Delete", "Deleting the retained local history CSV.", body));
}

void handleGraphsPath()
{
  size_t retainedSamples = getHistoryRetentionSamples();
  size_t retainedHours = getHistoryRetentionHours();
  String body;
  body.reserve(12000);

  if (!spiffsMounted)
  {
    body += "<div class='alert alert-danger mb-0'>SPIFFS is unavailable, so historical CSV logging and the Graphs page cannot be used until the filesystem mounts correctly.</div>";
    server.send(200, "text/html; charset=utf-8", adminPageShell("Graphs", "Historical CSV graphs stored locally on SPIFFS.", body));
    return;
  }

  body += "<div class='row g-4'>";
  body += "<div class='col-12 col-xl-4'><div class='card h-100'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>History Storage</h2>";
  body += "<div class='metric'>" + String(retainedHours) + " hour" + String(retainedHours == 1U ? "" : "s") + "</div>";
  body += "<div class='hint mt-2'>This logger keeps about " + String(retainedHours) + " hour" + String(retainedHours == 1U ? "" : "s") + " of compact CSV history on SPIFFS, stored as roughly " + String(retainedSamples) + " rolling samples at the 61-second logging cadence.</div>";
  body += "<div id='historyMeta' class='alert alert-secondary mt-3 mb-0'>Loading retained samples from <span class='mono'>/history.csv</span>...</div>";
  body += "</div></div></div>";

  body += "<div class='col-12 col-xl-8'><div class='card h-100'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>History Actions</h2>";
  body += "<p class='hint mb-3'>Use this page for longer local trends than the live dashboard keeps in browser memory. Reload fetches the retained CSV directly from SPIFFS.</p>";
  body += "<div class='d-flex flex-wrap gap-2'><button type='button' class='btn btn-primary' id='historyReload'>Reload History</button><a class='btn btn-outline-secondary' href='/history.csv'>Download CSV</a><form method='post' action='/history-delete' onsubmit=\"return confirm('Delete the retained history CSV from SPIFFS?');\" class='d-inline'><button type='submit' class='btn btn-outline-secondary'>Delete CSV</button></form><a class='btn btn-outline-secondary' href='/'>Back to Dashboard</a></div>";
  body += "<div class='mt-4'><h3 class='h6 mb-3'>SPIFFS File Listing</h3>" + buildSpiffsDirectoryHtml() + "</div>";
  body += "</div></div></div>";
  body += "</div>";

  body += "<div class='card mt-4'><div class='card-body'><h2 class='h5 mb-3'>Radiation</h2><canvas id='graphsDose' height='90'></canvas></div></div>";
  body += "<div class='card mt-4'><div class='card-body'><h2 class='h5 mb-3'>Temperature, Humidity &amp; Pressure</h2><canvas id='graphsTempHumidity' height='90'></canvas></div></div>";
  body += "<div class='card mt-4'><div class='card-body'><h2 class='h5 mb-3'>IAQ and CO2</h2><canvas id='graphsAir' height='90'></canvas></div></div>";
  body += "<div class='card mt-4'><div class='card-body'><h2 class='h5 mb-3'>Particulate Matter</h2><canvas id='graphsPm' height='90'></canvas></div></div>";
  body += "<div class='card mt-4'><div class='card-body'><h2 class='h5 mb-3'>HV, Luminosity, HCHO &amp; VOC</h2><canvas id='graphsEnv' height='90'></canvas></div></div>";
    body += "<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.2/dist/chart.umd.min.js'></script>";
    body += "<script>(function(){"
      "const charts={};let lastRows=[];let palette={};"
      "function cssVar(name){return getComputedStyle(document.body).getPropertyValue(name).trim();}"
      "function syncThemePalette(){palette={text:cssVar('--text')||'#e0e0e0',grid:cssVar('--border')||'#334456',dose:'#f44336',cpm:'#ff9800',tube1:'#ce93d8',tube2:'#80cbc4',temp:'#ef5350',humidity:'#42a5f5',pressure:'#26c6da',iaq:'#ab47bc',co2:'#26a69a',pm1:'#aed581',pm25:'#ffca28',pm10:'#ef5350',hv:'#5b8def',light:'#ffd54f',hcho:'#7e57c2',voc:'#ff7043'};}"
      "function destroyCharts(){Object.values(charts).forEach(chart=>chart.destroy());Object.keys(charts).forEach(key=>delete charts[key]);}"
      "function makeChart(canvasId,datasets){return new Chart(document.getElementById(canvasId),{type:'line',data:{labels:[],datasets:datasets},options:{responsive:true,animation:false,interaction:{mode:'index',intersect:false},scales:{x:{ticks:{color:palette.text,maxTicksLimit:10},grid:{color:palette.grid}},y:{ticks:{color:palette.text},grid:{color:palette.grid}}},plugins:{legend:{labels:{color:palette.text}}}}});}"
      "function dataset(label,color,yAxisId){return{label:label,data:[],borderColor:color,backgroundColor:color+'22',fill:false,tension:.22,pointRadius:0,borderWidth:2,yAxisID:yAxisId||'y'};}"
      "function formatStamp(epoch){const date=new Date(epoch*1000);const pad=value=>String(value).padStart(2,'0');return date.getFullYear()+'-'+pad(date.getMonth()+1)+'-'+pad(date.getDate())+' '+pad(date.getHours())+':'+pad(date.getMinutes());}"
      "function parseCsv(text){const lines=text.trim().split(/\\r?\\n/);if(lines.length<=1){return [];}const conv=" + String(activeTubeConversionFactor, 6) + ";return lines.slice(1).map(line=>line.trim()).filter(Boolean).map(line=>{const cols=line.split(',');const n=cols.length;if(n<12){return null;}const epoch=Number(cols[0]);if(!Number.isFinite(epoch)||epoch<=0){return null;}if(n>=16){return{epoch:epoch,cpm:Number(cols[1])||0,tube1:Number(cols[2])||0,tube2:Number(cols[3])||0,temp:(Number(cols[4])||0)/10,humidity:(Number(cols[5])||0)/10,pressure:(Number(cols[6])||0)/10,iaq:(Number(cols[7])||0)/10,co2:Number(cols[8])||0,voc:(Number(cols[9])||0)/100,pm01:Number(cols[10])||0,pm25:Number(cols[11])||0,pm10:Number(cols[12])||0,hv:(Number(cols[13])||0)/10,luminosity:Number(cols[14])||0,hcho:(Number(cols[15])||0)/1000,dose:(Number(cols[1])||0)*conv};}if(n>=14){return{epoch:epoch,cpm:Number(cols[1])||0,tube1:0,tube2:0,temp:(Number(cols[2])||0)/10,humidity:(Number(cols[3])||0)/10,pressure:(Number(cols[4])||0)/10,iaq:(Number(cols[5])||0)/10,co2:Number(cols[6])||0,voc:(Number(cols[7])||0)/100,pm01:Number(cols[8])||0,pm25:Number(cols[9])||0,pm10:Number(cols[10])||0,hv:(Number(cols[11])||0)/10,luminosity:Number(cols[12])||0,hcho:(Number(cols[13])||0)/1000,dose:(Number(cols[1])||0)*conv};}return{epoch:epoch,cpm:Number(cols[1])||0,tube1:0,tube2:0,temp:(Number(cols[2])||0)/10,humidity:(Number(cols[3])||0)/10,pressure:0,iaq:(Number(cols[4])||0)/10,co2:Number(cols[5])||0,voc:0,pm01:Number(cols[6])||0,pm25:Number(cols[7])||0,pm10:Number(cols[8])||0,hv:(Number(cols[9])||0)/10,luminosity:Number(cols[10])||0,hcho:(Number(cols[11])||0)/1000,dose:(Number(cols[1])||0)*conv};}).filter(Boolean);}"
      "function updateMeta(rows){const meta=document.getElementById('historyMeta');if(!meta){return;}if(!rows.length){meta.className='alert alert-warning mt-3 mb-0';meta.textContent='No retained history samples yet. Wait for a 61-second cycle to complete.';return;}const first=rows[0].epoch;const last=rows[rows.length-1].epoch;const hours=((last-first)/3600).toFixed(1);meta.className='alert alert-success mt-3 mb-0';meta.innerHTML='Loaded <strong>'+String(rows.length)+'</strong> retained samples from <span class=\\'mono\\'>/history.csv</span>. Range: <strong>'+hours+' hours</strong> from '+formatStamp(first)+' to '+formatStamp(last)+'.';}"
      "function renderCharts(rows){lastRows=rows.slice();syncThemePalette();destroyCharts();charts.dose=makeChart('graphsDose',[dataset('Dose \u00B5Sv/h',palette.dose),dataset('CPM',palette.cpm),dataset('Tube 1 CPS',palette.tube1),dataset('Tube 2 CPS',palette.tube2)]);charts.th=new Chart(document.getElementById('graphsTempHumidity'),{type:'line',data:{labels:[],datasets:[{label:'Temp \u00B0C',data:[],borderColor:palette.temp,backgroundColor:palette.temp+'22',fill:false,tension:.22,pointRadius:0,borderWidth:2,yAxisID:'y'},{label:'Humidity %',data:[],borderColor:palette.humidity,backgroundColor:palette.humidity+'22',fill:false,tension:.22,pointRadius:0,borderWidth:2,yAxisID:'y'},{label:'Pressure hPa',data:[],borderColor:palette.pressure,backgroundColor:palette.pressure+'22',fill:false,tension:.22,pointRadius:0,borderWidth:2,yAxisID:'y2'}]},options:{responsive:true,animation:false,interaction:{mode:'index',intersect:false},scales:{x:{ticks:{color:palette.text,maxTicksLimit:10},grid:{color:palette.grid}},y:{ticks:{color:palette.text},grid:{color:palette.grid}},y2:{position:'right',ticks:{color:palette.text},grid:{drawOnChartArea:false}}},plugins:{legend:{labels:{color:palette.text}}}}});charts.air=makeChart('graphsAir',[dataset('IAQ',palette.iaq),dataset('CO2 ppm',palette.co2)]);charts.pm=makeChart('graphsPm',[dataset('PM1.0',palette.pm1),dataset('PM2.5',palette.pm25),dataset('PM10',palette.pm10)]);charts.env=makeChart('graphsEnv',[dataset('Tube HV V',palette.hv),dataset('Luminosity lux',palette.light),dataset('HCHO ppm',palette.hcho),dataset('VOC ppm',palette.voc)]);const labels=rows.map(row=>formatStamp(row.epoch));charts.dose.data.labels=labels;charts.th.data.labels=labels;charts.air.data.labels=labels;charts.pm.data.labels=labels;charts.env.data.labels=labels;charts.dose.data.datasets[0].data=rows.map(row=>row.dose);charts.dose.data.datasets[1].data=rows.map(row=>row.cpm);charts.dose.data.datasets[2].data=rows.map(row=>row.tube1);charts.dose.data.datasets[3].data=rows.map(row=>row.tube2);charts.th.data.datasets[0].data=rows.map(row=>row.temp);charts.th.data.datasets[1].data=rows.map(row=>row.humidity);charts.th.data.datasets[2].data=rows.map(row=>row.pressure);charts.air.data.datasets[0].data=rows.map(row=>row.iaq);charts.air.data.datasets[1].data=rows.map(row=>row.co2);charts.pm.data.datasets[0].data=rows.map(row=>row.pm01);charts.pm.data.datasets[1].data=rows.map(row=>row.pm25);charts.pm.data.datasets[2].data=rows.map(row=>row.pm10);charts.env.data.datasets[0].data=rows.map(row=>row.hv);charts.env.data.datasets[1].data=rows.map(row=>row.luminosity);charts.env.data.datasets[2].data=rows.map(row=>row.hcho);charts.env.data.datasets[3].data=rows.map(row=>row.voc);Object.values(charts).forEach(chart=>chart.update());}"
      "function refreshChartTheme(){if(lastRows.length){renderCharts(lastRows);}}"
      "async function loadHistory(){const meta=document.getElementById('historyMeta');if(meta){meta.className='alert alert-secondary mt-3 mb-0';meta.textContent='Loading retained samples from /history.csv...';}try{const response=await fetch('/history.csv?ts='+Date.now(),{cache:'no-store'});if(!response.ok){throw new Error('history fetch failed');}const csv=await response.text();const rows=parseCsv(csv);updateMeta(rows);renderCharts(rows);}catch(error){if(meta){meta.className='alert alert-danger mt-3 mb-0';meta.textContent='Failed to load /history.csv. Check SPIFFS status and try again.';}destroyCharts();lastRows=[];}}"
      "const reloadButton=document.getElementById('historyReload');if(reloadButton){reloadButton.addEventListener('click',loadHistory);}document.addEventListener('envLoggerThemeChanged',refreshChartTheme);syncThemePalette();loadHistory();"
      "})();</script>";

  server.send(200, "text/html; charset=utf-8", adminPageShell("Graphs", "Historical CSV graphs stored locally on SPIFFS.", body));
}

void handleRebootPath()
{
  String notice;
  String noticeClass = "success";
  if (!saveRuntimeSettingsFromRequest(notice, noticeClass))
  {
    String body;
    body.reserve(512);
    body += "<div class='card'><div class='card-body'>";
    body += "<h2 class='h5 mb-3'>Could Not Save Settings</h2>";
    body += "<div class='alert alert-" + noticeClass + " mb-3' role='alert'>" + htmlEscape(notice) + "</div>";
    body += "<div class='d-flex flex-wrap gap-2'><a class='btn btn-primary' href='/config'>Back to Config</a><button class='btn btn-outline-secondary' onclick='history.back()'>Back to Previous Form</button></div>";
    body += "</div></div>";
    server.send(400, "text/html; charset=utf-8", adminPageShell("Device Configuration", "Fix the invalid values and try saving again.", body));
    return;
  }

  String body;
  body.reserve(640);
  body += "<div class='card'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>Rebooting Device</h2>";
  body += "<p class='mb-3'>The ESP32 is applying the requested restart. Give it a few seconds to reconnect to WiFi, then reload the dashboard or config page.</p>";
  body += "<div class='alert alert-warning mb-0'>If you changed WiFi credentials or hostname, the device may come back on a new network name or IP address.</div>";
  body += "</div></div>";

  server.send(200, "text/html; charset=utf-8", adminPageShell("Rebooting", "Applying the saved configuration by restarting the device.", body));
  delay(250);
  ESP.restart();
}

void handleSimpleRebootPath()
{
  String body;
  body.reserve(480);
  body += "<div class='card'><div class='card-body'>";
  body += "<h2 class='h5 mb-3'>Rebooting Device</h2>";
  body += "<p class='mb-3'>The ESP32 is restarting without changing any stored settings. Reload the dashboard or config page after a few seconds.</p>";
  body += "<div class='alert alert-warning mb-0'>This action keeps the current runtime configuration exactly as stored in NVS.</div>";
  body += "</div></div>";

  server.send(200, "text/html; charset=utf-8", adminPageShell("Rebooting", "Restarting the device without modifying its saved configuration.", body));
  delay(250);
  ESP.restart();
}

void handleOtaCheckPath()
{
  const esp_partition_t *runningPartition = esp_ota_get_running_partition();
  const esp_partition_t *nextPartition = esp_ota_get_next_update_partition(NULL);
  size_t totalAppPartitionBytes = getAppPartitionTotalBytes();
  size_t usedAppPartitionBytes = getAppPartitionUsedBytes();
  size_t freeAppPartitionBytes = getAppPartitionFreeBytes();
  size_t totalFsBytes = getFsTotalBytes();
  size_t freeFsBytes = getFsFreeBytes();

  String body;
  body.reserve(2600);
  body += "<div class='row g-4'>";
  body += "<div class='col-md-6 col-xl-3'><div class='card h-100'><div class='card-body'><div class='text-body-secondary small text-uppercase mb-2'>WiFi</div><div class='metric'>" + String((WiFi.status() == WL_CONNECTED) ? "Online" : "Offline") + "</div><div class='hint mt-2'>RSSI: " + String(WiFi.RSSI()) + " dBm<br>IP: <span class='mono'>" + WiFi.localIP().toString() + "</span></div></div></div></div>";
  body += "<div class='col-md-6 col-xl-3'><div class='card h-100'><div class='card-body'><div class='text-body-secondary small text-uppercase mb-2'>Running Slot</div><div class='metric mono'>" + describePartitionSlot(runningPartition) + "</div><div class='hint mt-2'>Next slot: <span class='mono'>" + describePartitionSlot(nextPartition) + "</span></div></div></div></div>";
  body += "<div class='col-md-6 col-xl-3'><div class='card h-100'><div class='card-body'><div class='text-body-secondary small text-uppercase mb-2'>App Partition</div><div class='metric'>" + String(totalAppPartitionBytes / 1024.0f, 1) + " kB</div><div class='hint mt-2'>Used: " + String(usedAppPartitionBytes / 1024.0f, 1) + " kB<br>Headroom: " + String(freeAppPartitionBytes / 1024.0f, 1) + " kB</div></div></div></div>";
  body += "<div class='col-md-6 col-xl-3'><div class='card h-100'><div class='card-body'><div class='text-body-secondary small text-uppercase mb-2'>SPIFFS</div><div class='metric'>" + String(totalFsBytes / 1024.0f, 1) + " kB</div><div class='hint mt-2'>Mounted: " + String(spiffsMounted ? "yes" : "no") + "<br>Free: " + String(freeFsBytes / 1024.0f, 1) + " kB</div></div></div></div>";
  body += "</div>";
  body += "<div class='card mt-4'><div class='card-body'><h2 class='h5 mb-3'>Quick Checks</h2><div class='row g-3'>";
  body += "<div class='col-md-6'><div class='border rounded-3 p-3 h-100'><div class='fw-semibold mb-2'>Firmware OTA</div><div class='hint'>Current running OTA slot: <span class='mono'>" + describePartitionSlot(runningPartition) + "</span><br>Next update slot: <span class='mono'>" + describePartitionSlot(nextPartition) + "</span></div></div></div>";
  body += "<div class='col-md-6'><div class='border rounded-3 p-3 h-100'><div class='fw-semibold mb-2'>Filesystem</div><div class='hint'>SPIFFS total: " + String(totalFsBytes) + " bytes<br>SPIFFS free: " + String(freeFsBytes) + " bytes<br>SPIFFS used: " + String((freeFsBytes <= totalFsBytes) ? (totalFsBytes - freeFsBytes) : 0U) + " bytes</div></div></div>";
  body += "</div><div class='alert alert-secondary mt-3 mb-0'>If <span class='mono'>/update</span> returns 404 while this page loads, verify the installed ElegantOTA library version and reflash this exact sketch.</div></div></div>";

  server.send(200, "text/html; charset=utf-8", adminPageShell("OTA Status", "Validate firmware slots, app headroom, and SPIFFS state from the running device.", body));
}

// Helper: colour class based on IAQ value
static String iaqColour(float iaq)
{
  if (iaq < 51)  return "#4caf50"; // good
  if (iaq < 101) return "#8bc34a"; // average
  if (iaq < 151) return "#ffc107"; // little bad
  if (iaq < 201) return "#ff9800"; // bad
  if (iaq < 301) return "#f44336"; // worse
  return "#9c27b0";                 // very bad
}

// Helper: colour class based on dose rate (µSv/h)
static String doseColour(float usv)
{
  if (usv < 0.1)  return "#4caf50";
  if (usv < 0.5)  return "#ffc107";
  if (usv < 1.0)  return "#ff9800";
  return "#f44336";
}

static String uploadStatusText(int code)
{
  if (code == 200) return "OK";
  if (code == UPLOAD_STATUS_DISABLED) return "Disabled";
  if (code == UPLOAD_STATUS_MISSING_CONFIG) return "Missing Config";
  if (code == -1) return "Connect (-1)";
  if (code == -2) return "API (-2)";
  if (code == -3) return "Timeout (-3)";
  if (code == -4) return "Bad Reply (-4)";
  if (code > 0) return "HTTP " + String(code);
  return String(code);
}

static bool checkEspOk(const char *operation, esp_err_t errorCode)
{
  if (errorCode == ESP_OK)
  {
    return true;
  }

  Serial.println("ERROR: " + String(operation) + " failed, code = " + String((int)errorCode));
  return false;
}

static String hvColour(float voltage)
{
  // Healthy band chosen around the 400 V recommendation.
  if ((voltage < 350.0f) || (voltage > 475.0f)) return "#f44336";
  if ((voltage >= 380.0f) && (voltage <= 440.0f)) return "#4caf50";
  return "#ff9800";
}

static float estimateHvDrivePct(float voltage)
{
  return constrain(((voltage - 350.0f) * 100.0f) / 125.0f, 0.0f, 100.0f);
}

static float estimateHvDriveUploadPct(float voltage)
{
  // Upload a percentage-style value so 65% is sent as 65.0, not 650.
  return estimateHvDrivePct(voltage);
}

static unsigned long applyDeadTimeCorrection(int rawCps, const char *tubeLabel)
{
  if (rawCps <= 0)
  {
    return 0UL;
  }

  float denominator = 1.0f - (rawCps * activeTubeDeadTimeSeconds);
  if (denominator <= 0.0f)
  {
    unsigned long now = millis();
    if ((lastDeadTimeWarningMillis == 0UL) || ((now - lastDeadTimeWarningMillis) >= 5000UL))
    {
      Serial.println("WARNING: Dead-time correction saturated for " + String(tubeLabel) + "; using raw CPS fallback");
      lastDeadTimeWarningMillis = now;
    }
    return (unsigned long)rawCps;
  }

  return (unsigned long)(rawCps / denominator);
}

static void refreshCpuLoadMetrics(void)
{
  cpuLoadCore0Pct = constrain(100.0f - (float)ulTaskGetIdleRunTimePercentForCore(0), 0.0f, 100.0f);
  cpuLoadCore1Pct = constrain(100.0f - (float)ulTaskGetIdleRunTimePercentForCore(1), 0.0f, 100.0f);
}

static float movingAvgToCpm(movingAvg &avg)
{
  int sampleCount = avg.getCount();
  int *readings = avg.getReadings();
  long readingSum = 0;

  if ((sampleCount <= 0) || (readings == nullptr))
  {
    return 0.0f;
  }

  for (int index = 0; index < sampleCount; index++)
  {
    readingSum += readings[index];
  }

  return (readingSum * 60.0f) / sampleCount;
}

static size_t getHeapTotalBytes(void)
{
  return heap_caps_get_total_size(MALLOC_CAP_8BIT);
}

static size_t getAppPartitionTotalBytes(void)
{
  const esp_partition_t *runningPartition = esp_ota_get_running_partition();
  return (runningPartition != nullptr) ? runningPartition->size : 0U;
}

static size_t getAppPartitionUsedBytes(void)
{
  return ESP.getSketchSize();
}

static size_t getAppPartitionFreeBytes(void)
{
  size_t totalBytes = getAppPartitionTotalBytes();
  size_t usedBytes = getAppPartitionUsedBytes();
  return (usedBytes <= totalBytes) ? (totalBytes - usedBytes) : 0U;
}

static size_t getFsTotalBytes(void)
{
  return spiffsMounted ? SPIFFS.totalBytes() : 0U;
}

static size_t getFsFreeBytes(void)
{
  if (!spiffsMounted)
  {
    return 0U;
  }

  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  return (usedBytes <= totalBytes) ? (totalBytes - usedBytes) : 0U;
}

static int getWifiSignalPct(long rssiDbm)
{
  if (rssiDbm <= -100L)
  {
    return 0;
  }

  if (rssiDbm >= -50L)
  {
    return 100;
  }

  return (int)(2L * (rssiDbm + 100L));
}

static String wifiSecurityLabel(wifi_auth_mode_t authMode)
{
  switch (authMode)
  {
    case WIFI_AUTH_OPEN: return String("Open");
    case WIFI_AUTH_WEP: return String("WEP");
    case WIFI_AUTH_WPA_PSK: return String("WPA-PSK");
    case WIFI_AUTH_WPA2_PSK: return String("WPA2-PSK");
    case WIFI_AUTH_WPA_WPA2_PSK: return String("WPA/WPA2-PSK");
#ifdef WIFI_AUTH_WPA2_ENTERPRISE
    case WIFI_AUTH_WPA2_ENTERPRISE: return String("WPA2-Enterprise");
#endif
#ifdef WIFI_AUTH_WPA3_PSK
    case WIFI_AUTH_WPA3_PSK: return String("WPA3-PSK");
#endif
#ifdef WIFI_AUTH_WPA2_WPA3_PSK
    case WIFI_AUTH_WPA2_WPA3_PSK: return String("WPA2/WPA3-PSK");
#endif
#ifdef WIFI_AUTH_WAPI_PSK
    case WIFI_AUTH_WAPI_PSK: return String("WAPI-PSK");
#endif
    default: return String("Unknown");
  }
}

static String urlEncode(const String &value)
{
  static const char hexDigits[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() * 3);

  for (size_t index = 0; index < value.length(); ++index)
  {
    uint8_t ch = static_cast<uint8_t>(value[index]);
    bool isAlphaNum = ((ch >= 'a') && (ch <= 'z')) || ((ch >= 'A') && (ch <= 'Z')) || ((ch >= '0') && (ch <= '9'));
    bool isSafeSymbol = (ch == '-') || (ch == '_') || (ch == '.') || (ch == '~');

    if (isAlphaNum || isSafeSymbol)
    {
      encoded += static_cast<char>(ch);
    }
    else
    {
      encoded += '%';
      encoded += hexDigits[(ch >> 4) & 0x0F];
      encoded += hexDigits[ch & 0x0F];
    }
  }

  return encoded;
}

static String buildWifiScanResultsHtml(int networkCount, bool includeConfigLinks)
{
  String html;
  html.reserve(3200);
  html += "<div class='hint mb-3'>Results are returned after the async scan completes so the config page request stays stable while the radio scans.</div>";

  if (networkCount == 0)
  {
    html += "<div class='alert alert-secondary mb-0'>No nearby networks were detected. You can still enter the SSID manually.</div>";
    WiFi.scanDelete();
    return html;
  }

  html += "<div class='list-group list-group-flush border rounded-3 overflow-hidden'>";

  for (int index = 0; index < networkCount; ++index)
  {
    String ssid = WiFi.SSID(index);
    bool isHidden = (ssid.length() == 0);
    int32_t rssiDbm = WiFi.RSSI(index);
    int signalPct = getWifiSignalPct(rssiDbm);
    String channelText = String(WiFi.channel(index));
    String authText = wifiSecurityLabel(WiFi.encryptionType(index));
    String displaySsid = isHidden ? String("Hidden network") : ssid;
    bool isCurrent = (!isHidden && ssid == my_ssid);

    html += "<div class='list-group-item py-3'>";
    html += "<div class='d-flex flex-column flex-lg-row justify-content-between align-items-lg-center gap-3'>";
    html += "<div><div class='fw-semibold'>" + htmlEscape(displaySsid);
    if (isCurrent)
    {
      html += " <span class='badge text-bg-primary align-middle'>Current</span>";
    }
    html += "</div><div class='hint mt-1'>Signal: " + String(signalPct) + "% (" + String(rssiDbm) + " dBm) | Security: " + htmlEscape(authText) + " | Channel: " + htmlEscape(channelText) + "</div></div>";

    if (isHidden)
    {
      html += "<button type='button' class='btn btn-sm btn-outline-secondary' disabled>Manual entry required</button>";
    }
    else
    {
      if (includeConfigLinks)
      {
        html += "<a class='btn btn-sm btn-outline-primary' href='/config?wifi_ssid_pick=" + urlEncode(ssid) + "#wifi_ssid'>Use this network</a>";
      }
      else
      {
        html += "<button type='button' class='btn btn-sm btn-outline-primary wifi-select-btn' data-ssid='" + htmlEscape(ssid) + "'>Use this network</button>";
      }
    }

    html += "</div></div>";
  }

  html += "</div>";
  WiFi.scanDelete();
  return html;
}

static String buildSpiffsDirectoryHtml(void)
{
  String html;
  html.reserve(2200);

  if (!spiffsMounted)
  {
    html += "<div class='alert alert-danger mb-0'>SPIFFS is not mounted, so the storage directory cannot be shown.</div>";
    return html;
  }

  size_t usedBytes = SPIFFS.usedBytes();
  size_t totalBytes = SPIFFS.totalBytes();
  float usedPct = (totalBytes > 0U) ? (usedBytes * 100.0f / totalBytes) : 0.0f;

  html += "<div class='hint mb-3'>SPIFFS is flat on ESP32, so this shows the files stored in the filesystem root.</div>";
  html += "<div class='alert alert-secondary mb-3'>Used <span class='mono'>" + String(usedBytes) + "</span> of <span class='mono'>" + String(totalBytes) + "</span> bytes (<span class='mono'>" + String(usedPct, 1) + "%</span>).</div>";

  File root = SPIFFS.open("/");
  if (!root)
  {
    html += "<div class='alert alert-danger mb-0'>Failed to open the SPIFFS root directory.</div>";
    return html;
  }

  File entry = root.openNextFile();
  if (!entry)
  {
    html += "<div class='alert alert-warning mb-0'>No files are currently stored on SPIFFS.</div>";
    root.close();
    return html;
  }

  html += "<div class='list-group list-group-flush border rounded-3 overflow-hidden'>";
  while (entry)
  {
    String entryName = String(entry.name());
    size_t entrySize = entry.size();
    html += "<div class='list-group-item py-3'><div class='d-flex flex-column flex-lg-row justify-content-between align-items-lg-center gap-2'><div><div class='fw-semibold mono'>" + htmlEscape(entryName) + "</div><div class='hint mt-1'>Stored file on SPIFFS</div></div><div class='mono'>" + String(entrySize) + " bytes</div></div></div>";
    entry = root.openNextFile();
  }
  html += "</div>";

  root.close();
  return html;
}

static String describePartitionSlot(const esp_partition_t *partition)
{
  if (partition == nullptr)
  {
    return String("unavailable");
  }

  if (partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
  {
    return String("ota_0");
  }

  if (partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)
  {
    return String("ota_1");
  }

  if (partition->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY)
  {
    return String("factory");
  }

  if ((partition->label != nullptr) && (partition->label[0] != '\0'))
  {
    return String(partition->label);
  }

  return String("subtype 0x") + String((unsigned int)partition->subtype, HEX);
}

static void copyUploadRecord(UploadRecord &dst, const UploadSnapshot *src)
{
  dst.valid = true;
  dst.cpm = src->cpm;
  dst.actual_cps_1 = src->actual_cps_1;
  dst.actual_cps_2 = src->actual_cps_2;
  dst.tubeVoltage = src->tubeVoltage;
  dst.epoch = src->epoch;
  dst.var_temperature = src->var_temperature;
  dst.var_humidity = src->var_humidity;
  dst.var_pressure = src->var_pressure;
  dst.luminosity = src->luminosity;
  dst.var_voc = src->var_voc;
  dst.var_co2 = src->var_co2;
  dst.var_hcho = src->var_hcho;
  dst.var_pm01 = src->var_pm01;
  dst.var_pm25 = src->var_pm25;
  dst.var_pm10 = src->var_pm10;
}

static String radmonUploadDetailsHtml(const UploadRecord &record)
{
  if (!record.valid)
  {
    return "<div class='upload-values'>No upload yet.</div>";
  }

  return String("<div class='upload-values'>")
    + "<span>CPM: " + String(record.cpm) + "</span>"
    + "<span>Dose: " + String(outputSieverts(record.cpm), 4) + " &micro;Sv/h</span>"
    + "<span>Tube 1: " + String(record.actual_cps_1) + " cps</span>"
    + "<span>Tube 2: " + String(record.actual_cps_2) + " cps</span>"
    + "<span>Epoch: " + String((unsigned long)record.epoch) + "</span>"
    + "</div>";
}

static String uradmonUploadDetailsHtml(const UploadRecord &record)
{
  if (!record.valid)
  {
    return "<div class='upload-values'>No upload yet.</div>";
  }

  return String("<div class='upload-values'>")
    + "<span>CPM: " + String(record.cpm) + "</span>"
    + "<span>HV: " + String(record.tubeVoltage, 1) + " V</span>"
    + "<span>HV Drive Index: " + String(estimateHvDrivePct(record.tubeVoltage), 1) + " % est.</span>"
    + "<span>Temp: " + String(record.var_temperature, 1) + " C</span>"
    + "<span>Humidity: " + String(record.var_humidity, 1) + " %</span>"
    + "<span>Pressure: " + String(record.var_pressure / 100.0, 1) + " hPa</span>"
    + "<span>Light: " + String(record.luminosity) + " lux</span>"
    + "<span>VOC: " + String(record.var_voc / 1000.0, 1) + " kOhm</span>"
    + "<span>CO2: " + String(record.var_co2, 0) + " ppm</span>"
    + "<span>HCHO: " + String(record.var_hcho, 3) + " ppm</span>"
    + "<span>PM1: " + String(record.var_pm01) + "</span>"
    + "<span>PM2.5: " + String(record.var_pm25) + "</span>"
    + "<span>PM10: " + String(record.var_pm10) + "</span>"
    + "<span>Epoch: " + String((unsigned long)record.epoch) + "</span>"
    + "</div>";
}

static void fillUploadStatusSnapshot(UploadStatusSnapshot &dst)
{
  portENTER_CRITICAL(&uploadStatusMux);
  dst.statusCodeRadmon = statusCodeRadmon;
  dst.statusCodeURadmon = statusCodeURadmon;
  dst.radmon = lastRadmonUpload;
  dst.uradmon = lastURadmonUpload;
  portEXIT_CRITICAL(&uploadStatusMux);
}

static size_t getHistoryRetentionSamples(void)
{
  return activeHistoryRetentionSamples;
}

static size_t getHistoryRetentionHours(void)
{
  size_t samples = getHistoryRetentionSamples();
  size_t roundedHours = (samples + (HISTORY_RETENTION_SAMPLES_PER_HOUR / 2U)) / HISTORY_RETENTION_SAMPLES_PER_HOUR;
  if (roundedHours < DEFAULT_HISTORY_RETENTION_HOURS)
  {
    return DEFAULT_HISTORY_RETENTION_HOURS;
  }
  return roundedHours;
}

static bool appendHistorySample(time_t sampleEpoch)
{
  if (!spiffsMounted)
  {
    return false;
  }

  File historyFile = SPIFFS.open(HISTORY_LOG_PATH, FILE_APPEND);
  if (!historyFile)
  {
    Serial.println(F("WARNING: Failed to open history CSV for append"));
    return false;
  }

  if (historyFile.size() == 0U)
  {
    historyFile.println(HISTORY_LOG_HEADER);
  }

  long tempCx10 = lroundf(var_temperature * 10.0f);
  long humidityPctX10 = lroundf(var_humidity * 10.0f);
  long pressureHpaX10 = lroundf(var_pressure / 10.0f);
  long iaqX10 = lroundf(var_iaq * 10.0f);
  long co2Ppm = lroundf(var_co2);
  long vocPpmX100 = lroundf(var_voc * 100.0f);
  long tubeVoltageX10 = lroundf(tubeVoltage * 10.0f);
  long hchoPpb = lroundf(var_hcho * 1000.0f);

  char lineBuffer[200];
  snprintf(
    lineBuffer,
    sizeof(lineBuffer),
    "%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%ld,%d,%ld\n",
    (unsigned long)sampleEpoch,
    cpm,
    actual_cps_1,
    actual_cps_2,
    tempCx10,
    humidityPctX10,
    pressureHpaX10,
    iaqX10,
    co2Ppm,
    vocPpmX100,
    var_pm01,
    var_pm25,
    var_pm10,
    tubeVoltageX10,
    luminosity,
    hchoPpb);

  historyFile.print(lineBuffer);
  historyFile.close();

  historyRowCount++;
  pruneHistoryLogIfNeeded();
  return true;
}

// Count the number of data rows currently in the history CSV (excludes header).
// Called once at boot to initialise historyRowCount; not called again after that.
static size_t countHistoryRows(void)
{
  if (!spiffsMounted || !SPIFFS.exists(HISTORY_LOG_PATH))
  {
    return 0U;
  }
  File f = SPIFFS.open(HISTORY_LOG_PATH, FILE_READ);
  if (!f)
  {
    return 0U;
  }
  // Skip the header line
  f.readStringUntil('\n');
  size_t count = 0U;
  while (f.available())
  {
    String row = f.readStringUntil('\n');
    row.trim();
    if (row.length() > 0)
    {
      count++;
    }
  }
  f.close();
  return count;
}

static void pruneHistoryLogIfNeeded(void)
{
  if (!spiffsMounted || !SPIFFS.exists(HISTORY_LOG_PATH))
  {
    return;
  }

  size_t keepRowCount = getHistoryRetentionSamples();
  if (keepRowCount >= historyRowCount)
  {
    return;
  }

  // Open only to read the header line — we already know the row count
  File headerReadFile = SPIFFS.open(HISTORY_LOG_PATH, FILE_READ);
  if (!headerReadFile)
  {
    return;
  }
  String headerLine = headerReadFile.readStringUntil('\n');
  headerLine.trim();
  headerReadFile.close();

  size_t skipRowCount = historyRowCount - keepRowCount;
  File sourceFile = SPIFFS.open(HISTORY_LOG_PATH, FILE_READ);
  File tempFile = SPIFFS.open(HISTORY_LOG_TMP_PATH, FILE_WRITE);
  if (!sourceFile || !tempFile)
  {
    if (sourceFile) sourceFile.close();
    if (tempFile) tempFile.close();
    Serial.println(F("WARNING: Failed to prune history CSV"));
    return;
  }

  tempFile.println((headerLine.length() > 0) ? headerLine : String(HISTORY_LOG_HEADER));

  bool skippedHeader = false;
  size_t currentRow = 0U;
  size_t writtenRows = 0U;
  while (sourceFile.available())
  {
    String row = sourceFile.readStringUntil('\n');
    row.trim();
    if (!skippedHeader)
    {
      skippedHeader = true;
      continue;
    }
    if (row.length() == 0)
    {
      continue;
    }
    if (currentRow < skipRowCount)
    {
      currentRow++;
      continue;
    }
    tempFile.println(row);
    writtenRows++;
  }

  sourceFile.close();
  tempFile.close();

  SPIFFS.remove(HISTORY_LOG_PATH);
  if (!SPIFFS.rename(HISTORY_LOG_TMP_PATH, HISTORY_LOG_PATH))
  {
    Serial.println(F("WARNING: Failed to replace history CSV after pruning"));
  }
  else
  {
    historyRowCount = writtenRows;
  }
}

// Return the dashboard HTML as an array of chunks to stream with sendContent()
// Splitting avoids allocating one huge String on the heap at once.
std::vector<String> webPageChunks()
{
  std::vector<String> chunks;
  const String uptimeText = formatUptime(getUptimeSeconds());
  UploadStatusSnapshot uploadStatus = {};

  fillUploadStatusSnapshot(uploadStatus);

  // ── HEAD ─────────────────────────────────────────────────────────────────
  chunks.push_back(
    String("<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>")
    + "<link rel='icon' type='image/svg+xml' href=\"data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>&#9762;</text></svg>\">"
    + "<title>" + htmlEscape(my_hostname) + " : Environmental Logger</title>"
  );
  chunks.push_back(
    "<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.2/dist/chart.umd.min.js'></script>"
    "<style>"
    ":root{--bg:#10161d;--bg-soft:#17212b;--card:#1c2733;--border:#334456;--text:#f3f7fb;--muted:#c5d1dd;--accent:#58a6ff;--accent-strong:#3d8bfd;--tableHead:#1b2833;--tableRow:#16202a;--chip:#1a2834;--gaugeBg:#314151;--shadow:rgba(0,0,0,.28);--heroA:#17324a;--heroB:#245e86;--surface:rgba(255,255,255,.08);color-scheme:dark;}"
    "body[data-theme='light']{--bg:#eff4f8;--bg-soft:#f7fafc;--card:#ffffff;--border:#d4dee8;--text:#17212b;--muted:#5d6d7d;--accent:#1e6bd6;--accent-strong:#1451a8;--tableHead:#e5eef6;--tableRow:#f8fbfd;--chip:#eef3f7;--gaugeBg:#d8e1ea;--shadow:rgba(22,41,66,.12);--heroA:#ddebf7;--heroB:#f7fbff;--surface:rgba(255,255,255,.72);color-scheme:light;}"
    "*{box-sizing:border-box;margin:0;padding:0;}"
    "body{background:radial-gradient(circle at top,var(--heroA),var(--bg) 38%);color:var(--text);font-family:Segoe UI,Arial,sans-serif;transition:background-color .25s,color .25s;}"
    ".page-shell{max-width:1240px;margin:0 auto;padding:20px 16px 28px;}"
    ".topbar{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:12px;margin-bottom:14px;padding:12px 14px;border:1px solid var(--border);border-radius:18px;background:var(--surface);backdrop-filter:blur(12px);box-shadow:0 12px 28px var(--shadow);}"
    ".brand{display:flex;align-items:center;gap:10px;font-weight:700;letter-spacing:.02em;color:var(--text);}"
    ".brand-mark{width:34px;height:34px;border-radius:12px;display:grid;place-items:center;background:linear-gradient(135deg,var(--accent),var(--accent-strong));color:#fff;font-size:1rem;box-shadow:0 8px 18px rgba(33,109,214,.28);}"
    ".menu{display:flex;flex-wrap:wrap;gap:8px;align-items:center;}"
    ".menu-link{border-radius:999px;border:1px solid var(--border);background:var(--card);color:var(--text);text-decoration:none;padding:8px 12px;font-size:.84rem;font-weight:600;box-shadow:0 8px 20px var(--shadow);}"
    ".menu-link.active{background:linear-gradient(135deg,var(--accent),var(--accent-strong));color:#fff;border-color:transparent;}"
    ".hero{padding:24px;border:1px solid var(--border);border-radius:24px;background:linear-gradient(135deg,var(--surface),rgba(255,255,255,.02)),linear-gradient(135deg,var(--heroA),var(--heroB));box-shadow:0 20px 40px var(--shadow);margin-bottom:18px;}"
    ".hero h1{text-align:left;color:var(--text);margin-bottom:6px;font-size:clamp(1.7rem,3vw,2.5rem);}"
    ".hero-lead{color:var(--muted);font-size:.92rem;margin-bottom:10px;}"
    ".hero-meta{display:flex;flex-wrap:wrap;gap:8px;}"
    ".hero-chip{padding:8px 12px;border:1px solid var(--border);border-radius:999px;background:var(--chip);font-size:.82rem;color:var(--text);}"
    ".subtitle{text-align:left;color:var(--muted);font-size:.85rem;margin-bottom:0;}"
    ".toolbar{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:0;}"
    ".theme-picker{display:flex;align-items:center;gap:8px;background:var(--card);border:1px solid var(--border);border-radius:999px;padding:8px 12px;box-shadow:0 6px 18px var(--shadow);}"
    ".theme-picker label{font-size:.8rem;color:var(--muted);font-weight:700;letter-spacing:.04em;text-transform:uppercase;}"
    ".theme-picker select{background:var(--bg-soft);color:var(--text);border:1px solid var(--border);border-radius:999px;padding:6px 10px;font:inherit;outline:none;}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:14px;margin-bottom:20px;}"
    ".card{background:var(--card);border:1px solid var(--border);border-radius:18px;padding:14px;box-shadow:0 16px 34px var(--shadow);}"
    ".card-title{font-size:.75rem;color:var(--muted);text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px;}"
    ".card-value{font-size:2rem;font-weight:700;line-height:1;}"
    ".card-unit{font-size:.8rem;color:var(--muted);margin-top:2px;}"
    ".gauge-bg{background:var(--gaugeBg);border-radius:4px;height:8px;margin-top:10px;overflow:hidden;}"
    ".gauge-fill{height:100%;border-radius:4px;transition:width .4s;}"
    ".chart-card{background:var(--card);border:1px solid var(--border);border-radius:18px;padding:14px;margin-bottom:14px;box-shadow:0 16px 34px var(--shadow);}"
    ".chart-card h3{font-size:.85rem;color:var(--muted);margin-bottom:10px;}"
    "table{width:100%;border-collapse:collapse;font-size:.85rem;}"
    "th,td{border:1px solid var(--border);padding:8px 12px;text-align:left;}"
    "th{background:var(--tableHead);color:var(--accent);}"
    "tr:nth-child(even){background:var(--tableRow);}"
    ".badge{display:inline-block;align-self:flex-start;padding:1px 7px;border-radius:10px;font-size:.72rem;font-weight:700;min-width:0;width:auto;}"
    ".badge-ok{background:#1b5e20;color:#a5d6a7;}"
    ".badge-err{background:#b71c1c;color:#ef9a9a;}"
    ".badge-neutral{background:#37474f;color:#cfd8dc;}"
    ".status-cell{display:flex;flex-direction:column;align-items:flex-start;gap:4px;}"
    ".status-meta{font-size:.72rem;color:var(--muted);}"
    ".wifi-card details{margin-top:4px;}"
    ".wifi-card summary{list-style:none;cursor:pointer;}"
    ".wifi-card summary::-webkit-details-marker{display:none;}"
    ".wifi-summary{display:flex;align-items:center;gap:12px;}"
    ".wifi-icon{display:flex;align-items:flex-end;gap:3px;height:24px;}"
    ".wifi-bar{width:5px;border-radius:999px;background:var(--gaugeBg);transition:background-color .25s,opacity .25s;}"
    ".wifi-bar:nth-child(1){height:7px;}"
    ".wifi-bar:nth-child(2){height:12px;}"
    ".wifi-bar:nth-child(3){height:17px;}"
    ".wifi-bar:nth-child(4){height:22px;}"
    ".wifi-bar.active{background:var(--accent);}"
    ".wifi-bar.off{opacity:.35;}"
    ".wifi-state{display:block;font-size:.8rem;color:var(--muted);margin-top:2px;}"
    ".wifi-details{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:6px;font-size:.72rem;color:var(--muted);margin-top:10px;}"
    ".wifi-details span{background:var(--chip);border:1px solid var(--border);border-radius:6px;padding:6px 8px;}"
    ".upload-toggle summary{cursor:pointer;color:var(--accent);font-weight:700;list-style:none;}"
    ".upload-toggle summary::-webkit-details-marker{display:none;}"
    ".upload-toggle[open] summary{margin-bottom:8px;}"
    ".upload-values{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:6px;font-size:.72rem;color:var(--muted);margin-top:6px;}"
    ".upload-values span{background:var(--chip);border:1px solid var(--border);border-radius:6px;padding:6px 8px;}"
    "a{color:var(--accent);}"
    "section{margin-bottom:20px;}"
    "h2{color:var(--accent);font-size:1rem;margin-bottom:10px;border-bottom:1px solid var(--border);padding-bottom:4px;}"
    "@media (max-width:820px){.topbar{padding:10px 12px;}.menu{width:100%;}.toolbar{flex-direction:column;align-items:stretch;}.theme-picker{width:100%;justify-content:center;}.hero h1{text-align:center;}.subtitle{text-align:center;}.hero-meta{justify-content:center;}}"
    "</style>"
    "</head>"
    "<body><main class='page-shell'>"
  );

  // ── TITLE ────────────────────────────────────────────────────────────────
  chunks.push_back(
    String("<header class='topbar'><div class='brand'><span class='brand-mark'>&#9762;</span><span>Environmental Logger</span></div><nav class='menu'><a class='menu-link active' href='/'>Dashboard</a><a class='menu-link' href='/graphs'>Graphs</a><a class='menu-link' href='/config'>Config</a><div class='theme-picker'><label for='themeSelect'>Theme</label><select id='themeSelect'><option value='dark'>Dark</option><option value='light'>Light</option></select></div></nav></header>")
    + "<section class='hero'><h1>" + htmlEscape(activeStationName) + "</h1><p class='hero-lead'>Live environmental, radiation, storage, and upload telemetry from the ESP32 station.</p><div class='hero-meta'><span class='hero-chip'><span id='clockValue'>" + String(ntp.formattedTime("%T  |  %F"))
    + "</span></span><span class='hero-chip'><span id='cycleValue'>cycle " + String(increaseSecCount) + " / 60</span></span><span class='hero-chip'><span id='uptimeValue'>uptime " + uptimeText + "</span></span></div></section>"
  );

  // ── RADIATION CARDS ───────────────────────────────────────────────────────
  float usv   = outputSieverts(cpm);
  String dCol = doseColour(usv);
  String hvCol = hvColour(tubeVoltage);
  float hvDrivePct = estimateHvDrivePct(tubeVoltage);
  // CPM gauge: full-scale from runtime config (activeCpmGaugeFullScale)
  int cpmPct  = constrain((int)(cpm * 100UL / (unsigned long)activeCpmGaugeFullScale), 0, 100);
  // Tube voltage gauge: nominal 360–430 V, display against 500 V max
  int hvPct   = constrain((int)(tubeVoltage * 100.0 / 500.0), 0, 100);

  chunks.push_back(
    String("<section><h2>&#9762; Radiation</h2><div class='grid'>")
    // Dose rate
    + "<div class='card'>"
      "<div class='card-title'>Dose Rate</div>"
      "<div id='doseValue' class='card-value' style='color:" + dCol + "'>" + String(usv, 4) + "</div>"
      "<div class='card-unit'>&#xb5;Sv/h</div>"
      "<div class='gauge-bg'><div id='doseGauge' class='gauge-fill' style='width:" + String(cpmPct) + "%;background:" + dCol + ";'></div></div>"
    "</div>"
    // CPM
    + "<div class='card'>"
      "<div class='card-title'>CPM (2-tube avg, 120 s)</div>"
      "<div id='cpmValue' class='card-value'>" + String(cpm) + "</div>"
      "<div class='card-unit'>counts per minute</div>"
    "</div>"
    // Tube 1
    + "<div class='card'>"
      "<div class='card-title'>Tube 1 CPS</div>"
      "<div id='tube1Value' class='card-value'>" + String(actual_cps_1) + "</div>"
      "<div class='card-unit'>counts / s</div>"
    "</div>"
    // Tube 2
    + "<div class='card'>"
      "<div class='card-title'>Tube 2 CPS</div>"
      "<div id='tube2Value' class='card-value'>" + String(actual_cps_2) + "</div>"
      "<div class='card-unit'>counts / s</div>"
    "</div>"
    // Tube 1 CPM
    + "<div class='card'>"
      "<div class='card-title'>Tube 1 CPM</div>"
      "<div id='cpm1Value' class='card-value'>" + String(cpm1) + "</div>"
      "<div class='card-unit'>counts per minute</div>"
    "</div>"
    // Tube 2 CPM
    + "<div class='card'>"
      "<div class='card-title'>Tube 2 CPM</div>"
      "<div id='cpm2Value' class='card-value'>" + String(cpm2) + "</div>"
      "<div class='card-unit'>counts per minute</div>"
    "</div>"
    // HV
    + "<div class='card'>"
      "<div class='card-title'>Tube High Voltage</div>"
      "<div id='hvValue' class='card-value'>" + String(tubeVoltage, 1) + "</div>"
      "<div class='card-unit'>V</div>"
      "<div class='gauge-bg'><div id='hvGauge' class='gauge-fill' style='width:" + String(hvPct) + "%;background:" + hvCol + ";'></div></div>"
    "</div>"
    // Estimated HV drive index
    + "<div class='card'>"
      "<div class='card-title'>HV Drive Index</div>"
      "<div id='hvDriveValue' class='card-value'>" + String(hvDrivePct, 1) + "</div>"
      "<div class='card-unit'>% est.</div>"
      "<div class='gauge-bg'><div id='hvDriveGauge' class='gauge-fill' style='width:" + String((int)roundf(hvDrivePct)) + "%;background:" + hvCol + ";'></div></div>"
    "</div>"
    "</div></section>"
  );

  // ── AIR QUALITY CARDS ─────────────────────────────────────────────────────
  String iaqCol = iaqColour(var_iaq);
  int iaqPct    = constrain((int)(var_iaq * 100.0 / 500.0), 0, 100);
  // CO2: 400 ppm ambient .. 5000 ppm = very bad
  int co2Pct    = constrain((int)((var_co2 - 400.0) * 100.0 / 4600.0), 0, 100);
  // Humidity gauge 0-100 %
  int humPct    = constrain((int)var_humidity, 0, 100);

  static const char* iaqLabel[] = {"Stabilizing","Uncertain","Calibrating","Calibrated"};
  const char* accStr = (var_iaqAccuracy >= 0 && var_iaqAccuracy <= 3)
                       ? iaqLabel[var_iaqAccuracy] : "?";

  chunks.push_back(
    String("<section><h2>&#127807; Air Quality</h2><div class='grid'>")
    // IAQ
    + "<div class='card'>"
      "<div id='iaqTitle' class='card-title'>IAQ &mdash; " + String(accStr) + "</div>"
      "<div id='iaqValue' class='card-value' style='color:" + iaqCol + "'>" + String(var_iaq, 1) + "</div>"
      "<div class='card-unit'>/ 500</div>"
      "<div class='gauge-bg'><div id='iaqGauge' class='gauge-fill' style='width:" + String(iaqPct) + "%;background:" + iaqCol + ";'></div></div>"
    "</div>"
    // CO2
    + "<div class='card'>"
      "<div class='card-title'>CO&#178;</div>"
      "<div id='co2Value' class='card-value'>" + String(var_co2, 0) + "</div>"
      "<div class='card-unit'>ppm</div>"
      "<div class='gauge-bg'><div id='co2Gauge' class='gauge-fill' style='width:" + String(co2Pct) + "%;background:#ff9800;'></div></div>"
    "</div>"
    // VOC
    + "<div class='card'>"
      "<div class='card-title'>VOC (gas resistance)</div>"
      "<div id='vocValue' class='card-value'>" + String(var_voc / 1000.0, 1) + "</div>"
      "<div class='card-unit'>k&#937;</div>"
    "</div>"
    // HCHO
    + "<div class='card'>"
      "<div class='card-title'>Formaldehyde (CH&#8322;O)</div>"
      "<div id='hchoValue' class='card-value'>" + String(var_hcho, 3) + "</div>"
      "<div class='card-unit'>ppm</div>"
    "</div>"
    // PM1
    + "<div class='card'>"
      "<div class='card-title'>PM 1.0</div>"
      "<div id='pm01Value' class='card-value'>" + String(var_pm01) + "</div>"
      "<div class='card-unit'>&#xb5;g/m&#179;</div>"
    "</div>"
    // PM2.5
    + "<div class='card'>"
      "<div class='card-title'>PM 2.5</div>"
      "<div id='pm25Value' class='card-value'>" + String(var_pm25) + "</div>"
      "<div class='card-unit'>&#xb5;g/m&#179;</div>"
    "</div>"
    // PM10
    + "<div class='card'>"
      "<div class='card-title'>PM 10</div>"
      "<div id='pm10Value' class='card-value'>" + String(var_pm10) + "</div>"
      "<div class='card-unit'>&#xb5;g/m&#179;</div>"
    "</div>"
    "</div></section>"
  );

  // ── ENVIRONMENT CARDS ─────────────────────────────────────────────────────
  chunks.push_back(
    String("<section><h2>&#127777; Environment</h2><div class='grid'>")
    // Temperature
    + "<div class='card'>"
      "<div class='card-title'>Temperature</div>"
      "<div id='tempValue' class='card-value'>" + String(var_temperature, 1) + "</div>"
      "<div class='card-unit'>&#8451;</div>"
    "</div>"
    // Pressure
    + "<div class='card'>"
      "<div class='card-title'>Pressure</div>"
      "<div id='pressureValue' class='card-value'>" + String(var_pressure / 100.0, 1) + "</div>"
      "<div class='card-unit'>hPa</div>"
    "</div>"
    // Humidity
    + "<div class='card'>"
      "<div class='card-title'>Humidity</div>"
      "<div id='humidityValue' class='card-value'>" + String(var_humidity, 1) + "</div>"
      "<div class='card-unit'>% RH</div>"
      "<div class='gauge-bg'><div id='humidityGauge' class='gauge-fill' style='width:" + String(humPct) + "%;background:#64b5f6;'></div></div>"
    "</div>"
    // Luminosity
    + "<div class='card'>"
      "<div class='card-title'>Luminosity</div>"
      "<div id='luminosityValue' class='card-value'>" + String(luminosity) + "</div>"
      "<div class='card-unit'>lux</div>"
    "</div>"
    "</div></section>"
  );

  // ── RESOURCE CARDS ───────────────────────────────────────────────────────
  uint32_t freeHeapBytes = esp_get_free_heap_size();
  size_t totalHeapBytes = getHeapTotalBytes();
  size_t freeAppPartitionBytes = getAppPartitionFreeBytes();
  size_t totalAppPartitionBytes = getAppPartitionTotalBytes();
  size_t freeFsBytes = getFsFreeBytes();
  size_t totalFsBytes = getFsTotalBytes();
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  long wifiRssiDbm = wifiConnected ? WiFi.RSSI() : -100L;
  int wifiSignalPct = getWifiSignalPct(wifiRssiDbm);
  int wifiBarsActive = constrain((wifiSignalPct + 24) / 25, 0, 4);
  float heapFreePct = (totalHeapBytes > 0U) ? (freeHeapBytes * 100.0f / totalHeapBytes) : 0.0f;
  float appPartitionFreePct = (totalAppPartitionBytes > 0U) ? (freeAppPartitionBytes * 100.0f / totalAppPartitionBytes) : 0.0f;
  float fsFreePct = (totalFsBytes > 0U) ? (freeFsBytes * 100.0f / totalFsBytes) : 0.0f;
  String heapUnit = String(freeHeapBytes / 1024.0f, 1)
                    + " / " + String(totalHeapBytes / 1024.0f, 1)
                    + " kB (" + String(heapFreePct, 1) + "% free)";
  String appPartitionValue = (totalAppPartitionBytes > 0U) ? String(freeAppPartitionBytes / 1024.0f, 1) : String("n/a");
  String appPartitionUnit = (totalAppPartitionBytes > 0U)
                     ? (String(freeAppPartitionBytes / 1024.0f, 1)
                       + " / " + String(totalAppPartitionBytes / 1024.0f, 1)
                       + " kB (" + String(appPartitionFreePct, 1) + "% free)")
                     : String("App partition unavailable");
  String fsValue = spiffsMounted ? String(freeFsBytes / 1024.0f, 1) : String("n/a");
  String fsUnit = spiffsMounted
                  ? (String(freeFsBytes / 1024.0f, 1)
                     + " / " + String(totalFsBytes / 1024.0f, 1)
                     + " kB (" + String(fsFreePct, 1) + "% free)")
                  : String("SPIFFS unavailable");
  String wifiValue = wifiConnected ? (String(wifiRssiDbm) + " dBm") : String("offline");
  String wifiStatus = wifiConnected ? "connected" : "disconnected";
  String wifiDetails = String("<span>SSID: ") + String(my_ssid) + "</span>"
                     + "<span>RSSI: " + (wifiConnected ? String(wifiRssiDbm) + " dBm" : String("n/a")) + "</span>"
                     + "<span>Signal: " + String(wifiSignalPct) + "%</span>"
                     + "<span>IP: " + WiFi.localIP().toString() + "</span>"
                     + "<span>Gateway: " + WiFi.gatewayIP().toString() + "</span>"
                     + "<span>MAC: " + WiFi.macAddress() + "</span>";

  chunks.push_back(
    String("<section><h2>&#9881; Resources</h2><div class='grid'>")
    + "<div class='card'>"
      "<div class='card-title'>CPU Core 0</div>"
      "<div id='cpuLoadCore0Value' class='card-value'>" + String(cpuLoadCore0Pct, 1) + "</div>"
      "<div class='card-unit'>% busy</div>"
    "</div>"
    + "<div class='card'>"
      "<div class='card-title'>CPU Core 1</div>"
      "<div id='cpuLoadCore1Value' class='card-value'>" + String(cpuLoadCore1Pct, 1) + "</div>"
      "<div class='card-unit'>% busy</div>"
    "</div>"
    + "<div class='card'>"
      "<div class='card-title'>Loop Active</div>"
      "<div id='loopActiveValue' class='card-value'>" + String(loopActivePct, 1) + "</div>"
      "<div class='card-unit'>% of Core 0 loop task time</div>"
    "</div>"
    + "<div class='card'>"
      "<div class='card-title'>Free Heap</div>"
      "<div id='heapValue' class='card-value'>" + String(freeHeapBytes / 1024.0f, 1) + "</div>"
      "<div id='heapUnit' class='card-unit'>" + heapUnit + "</div>"
    "</div>"
    + "<div class='card'>"
      "<div class='card-title'>App Partition</div>"
      "<div id='appPartitionValue' class='card-value'>" + appPartitionValue + "</div>"
      "<div id='appPartitionUnit' class='card-unit'>" + appPartitionUnit + "</div>"
    "</div>"
    + "<div class='card'>"
      "<div class='card-title'>Disk Space</div>"
      "<div id='diskValue' class='card-value'>" + fsValue + "</div>"
      "<div id='diskUnit' class='card-unit'>" + fsUnit + "</div>"
    "</div>"
    + "<div class='card wifi-card'>"
      "<div class='card-title'>WiFi</div>"
      "<details>"
        "<summary><div class='wifi-summary'>"
          "<div id='wifiIcon' class='wifi-icon'>"
            "<span class='wifi-bar " + String((wifiBarsActive >= 1) ? "active" : "off") + "'></span>"
            "<span class='wifi-bar " + String((wifiBarsActive >= 2) ? "active" : "off") + "'></span>"
            "<span class='wifi-bar " + String((wifiBarsActive >= 3) ? "active" : "off") + "'></span>"
            "<span class='wifi-bar " + String((wifiBarsActive >= 4) ? "active" : "off") + "'></span>"
          "</div>"
          "<div><div id='wifiValue' class='card-value'>" + wifiValue + "</div><span id='wifiState' class='wifi-state'>" + wifiStatus + "</span></div>"
        "</div></summary>"
        "<div id='wifiDetails' class='wifi-details'>" + wifiDetails + "</div>"
      "</details>"
    "</div>"
    + "<div class='card'>"
      "<div class='card-title'>I2C Devices</div>"
      "<div id='i2cValue' class='card-value'>" + String(i2cDevicesFound) + "</div>"
      "<div class='card-unit'>found on bus</div>"
    "</div>"
    "</div></section>"
  );

  // ── UPLOAD STATUS ─────────────────────────────────────────────────────────
  String radBadge  = ((uploadStatus.statusCodeRadmon == UPLOAD_STATUS_DISABLED) || (uploadStatus.statusCodeRadmon == UPLOAD_STATUS_MISSING_CONFIG)) ? "badge-neutral" : ((uploadStatus.statusCodeRadmon == 200) ? "badge-ok" : "badge-err");
  String uradBadge = ((uploadStatus.statusCodeURadmon == UPLOAD_STATUS_DISABLED) || (uploadStatus.statusCodeURadmon == UPLOAD_STATUS_MISSING_CONFIG)) ? "badge-neutral" : ((uploadStatus.statusCodeURadmon == 200) ? "badge-ok" : "badge-err");
  String radStatusText = uploadStatusText(uploadStatus.statusCodeRadmon);
  String uradStatusText = uploadStatusText(uploadStatus.statusCodeURadmon);

  chunks.push_back(
    String("<section><h2>&#9652; Uploads</h2>")
    + "<table><tr><th>Platform</th><th>Status</th><th>Dashboard</th></tr>"
    + "<tr><td><details class='upload-toggle'><summary>Radmon</summary><div id='radmonDetails'>" + radmonUploadDetailsHtml(uploadStatus.radmon) + "</div></details></td>"
      "<td><div class='status-cell'><span id='radmonStatus' class='badge " + radBadge + "'>" + radStatusText + "</span><span id='radmonUpdated' class='status-meta'>Last upload: waiting for live data...</span></div></td>"
      "<td><a href='https://radmon.org/radmon.php?function=showuserpage&user=DonZalmrol' target='_blank' rel='noopener noreferrer'>radmon.org</a></td></tr>"
    + "<tr><td><details class='upload-toggle'><summary>uRADMonitor</summary><div id='uradmonDetails'>" + uradmonUploadDetailsHtml(uploadStatus.uradmon) + "</div></details></td>"
      "<td><div class='status-cell'><span id='uradmonStatus' class='badge " + uradBadge + "'>" + uradStatusText + "</span><span id='uradmonUpdated' class='status-meta'>Last upload: waiting for live data...</span></div></td>"
      "<td><a href='https://www.uradmonitor.com/tools/dashboard-04/?open=13000212' target='_blank' rel='noopener noreferrer'>uradmonitor.com</a></td></tr>"
    + "</table></section>"
  );

  // ── LIVE CHARTS (browser-side, poll /json every 5 s) ─────────────────────
  chunks.push_back(
    String("<section><h2>&#128200; Live History (auto-updates every 1 s)</h2>")
    + "<div class='chart-card'><h3>Dose rate &#xb5;Sv/h</h3><canvas id='cDose' height='80'></canvas></div>"
    + "<div class='chart-card'><h3>CPM (combined + per tube)</h3><canvas id='cCpm' height='80'></canvas></div>"
    + "<div class='chart-card'><h3>CPS per tube &amp; moving average</h3><canvas id='cCps' height='80'></canvas></div>"
    + "<div class='chart-card'><h3>Temperature &#8451;, Humidity %RH &amp; Pressure hPa</h3><canvas id='cTH' height='80'></canvas></div>"
    + "<div class='chart-card'><h3>IAQ</h3><canvas id='cIaq' height='80'></canvas></div>"
    + "<div class='chart-card'><h3>CO&#178; ppm</h3><canvas id='cCo2' height='80'></canvas></div>"
    + "<div class='chart-card'><h3>PM 1.0 / 2.5 / 10 &#xb5;g/m&#179;</h3><canvas id='cPm' height='80'></canvas></div>"
    + "</section>"
  );

  // ── CHART.JS SCRIPT ───────────────────────────────────────────────────────
  chunks.push_back(
    String("<script>")
    + "const MAX_PTS=60;"
    + "const FAST_POLL_MS=1000;"
    + "const THEME_KEY='envLoggerTheme';"
    + "let C={text:'#e0e0e0',grid:'#333',accent:'#64b5f6'};"
    + "const IAQ_LABELS=['Stabilizing','Uncertain','Calibrating','Calibrated'];"
    + "function byId(id){return document.getElementById(id);}"
    + "function clamp(v,min,max){return Math.min(max,Math.max(min,v));}"
    + "function setText(id,value){const node=byId(id);if(node)node.textContent=value;}"
    + "function setHtml(id,value){const node=byId(id);if(node)node.innerHTML=value;}"
    + "function pad2(value){return String(value).padStart(2,'0');}"
    + "function cssVar(name){return getComputedStyle(document.body).getPropertyValue(name).trim();}"
    + "function syncThemePalette(){C={text:cssVar('--text')||'#e0e0e0',grid:cssVar('--border')||'#333',accent:cssVar('--accent')||'#64b5f6'};}"
    + "function applyTheme(theme){document.body.setAttribute('data-theme',theme);localStorage.setItem(THEME_KEY,theme);const sel=byId('themeSelect');if(sel&&sel.value!==theme)sel.value=theme;syncThemePalette();updateChartTheme();}"
    + "function initTheme(){const saved=localStorage.getItem(THEME_KEY)||'dark';applyTheme(saved);const sel=byId('themeSelect');if(sel)sel.addEventListener('change',e=>applyTheme(e.target.value));}"
    + "function formatUptime(seconds){"
    + "  const total=Math.max(0,Number(seconds)||0);"
    + "  const days=Math.floor(total/86400);"
    + "  const hours=Math.floor((total%86400)/3600);"
    + "  const minutes=Math.floor((total%3600)/60);"
    + "  return 'uptime '+String(days)+' days, '+pad2(hours)+':'+pad2(minutes);"
    + "}"
    + "function uploadStatusText(code){"
    + "  const n=Number(code)||0;"
    + "  if(n===200)return 'OK';"
    + "  if(n===-1)return 'Connect (-1)';"
    + "  if(n===-2)return 'API (-2)';"
    + "  if(n===-3)return 'Timeout (-3)';"
    + "  if(n===-4)return 'Bad Reply (-4)';"
    + "  if(n>0)return 'HTTP '+String(n);"
    + "  return String(n);"
    + "}"
    + "function formatEpoch(epoch){"
    + "  const ts=Number(epoch)||0;"
    + "  if(ts<=0)return 'Last upload: none yet';"
    + "  const d=new Date(ts*1000);"
    + "  const stamp=d.getFullYear()+'-'+pad2(d.getMonth()+1)+'-'+pad2(d.getDate())+' '+pad2(d.getHours())+':'+pad2(d.getMinutes());"
    + "  return 'Last upload: '+stamp;"
    + "}"
    + "function doseColor(usv){if(usv<0.1)return '#4caf50';if(usv<0.5)return '#ffc107';if(usv<1.0)return '#ff9800';return '#f44336';}"
    + "function hvColor(voltage){if(voltage<350||voltage>475)return '#f44336';if(voltage>=380&&voltage<=440)return '#4caf50';return '#ff9800';}"
    + "function estimateHvDrivePct(voltage){return clamp(((Number(voltage)||0)-350)*100/125,0,100);}"
    + "function iaqColor(iaq){if(iaq<51)return '#4caf50';if(iaq<101)return '#8bc34a';if(iaq<151)return '#ffc107';if(iaq<201)return '#ff9800';if(iaq<301)return '#f44336';return '#9c27b0';}"
    + "function wifiSignalPct(rssi){const v=Number(rssi);if(!Number.isFinite(v)||v<=-100)return 0;if(v>=-50)return 100;return Math.round(2*(v+100));}"
    + "function wifiBars(pct){return Math.min(4,Math.max(0,Math.floor((pct+24)/25)));}"
    + "function defOpts(label,color){"
    + "  return{responsive:true,animation:false,"
    + "    scales:{"
    + "      x:{ticks:{color:C.text,maxTicksLimit:8},grid:{color:C.grid}},"
    + "      y:{ticks:{color:C.text},grid:{color:C.grid}}"
    + "    },"
    + "    plugins:{legend:{labels:{color:C.text}}}"
    + "  };"
    + "}"
    + "function mkChart(id,datasets,opts){"
    + "  return new Chart(document.getElementById(id),{"
    + "    type:'line',"
    + "    data:{labels:[],datasets:datasets},"
    + "    options:opts||defOpts()"
    + "  });"
    + "}"
    + "function ds(label,color,data){"
    + "  return{label:label,data:data||[],borderColor:color,backgroundColor:color+'22',"
    + "    fill:true,tension:0.3,pointRadius:2,borderWidth:2};"
    + "}"
    + "const dDose = ds('\xb5Sv/h','#f44336');"
    + "const dCpm  = ds('CPM','#ff9800');"
    + "const dTube1= ds('Tube 1 CPS','#ce93d8');"
    + "const dTube2= ds('Tube 2 CPS','#80cbc4');"
    + "const dCpsAvg=ds('CPS Avg','#ffca28');"
    + "const dCpm1 = ds('Tube 1 CPM','#ce93d8');"
    + "const dCpm2 = ds('Tube 2 CPM','#80cbc4');"
    + "const dTemp = ds('Temp \\u00B0C','#ef5350');"
    + "const dHum  = ds('Humidity %','#42a5f5');"
    + "const dPressure={label:'Pressure hPa',data:[],borderColor:'#26c6da',backgroundColor:'#26c6da22',fill:false,tension:0.3,pointRadius:2,borderWidth:2,yAxisID:'y2'};"
    + "const dIaq  = ds('IAQ','#ab47bc');"
    + "const dCo2  = ds('CO2 ppm','#26a69a');"
    + "const dPm1  = ds('PM1.0','#aed581');"
    + "const dPm25 = ds('PM2.5','#ffca28');"
    + "const dPm10 = ds('PM10','#ef5350');"
    + "const charts={"
    + "  dose: mkChart('cDose',[dDose]),"
    + "  cpm:  mkChart('cCpm', [dCpm,dCpm1,dCpm2]),"
    + "  cps:  mkChart('cCps', [dTube1,dTube2,dCpsAvg]),"
    + "  th:   new Chart(document.getElementById('cTH'),{type:'line',data:{labels:[],datasets:[dTemp,dHum,dPressure]},options:{responsive:true,animation:false,interaction:{mode:'index',intersect:false},scales:{x:{ticks:{color:C.text,maxTicksLimit:8},grid:{color:C.grid}},y:{ticks:{color:C.text},grid:{color:C.grid}},y2:{position:'right',ticks:{color:C.text},grid:{drawOnChartArea:false}}},plugins:{legend:{labels:{color:C.text}}}}}),"
    + "  iaq:  mkChart('cIaq', [dIaq]),"
    + "  co2:  mkChart('cCo2', [dCo2]),"
    + "  pm:   mkChart('cPm',  [dPm1,dPm25,dPm10])"
    + "};"
    + "function updateChartTheme(){Object.values(charts).forEach(chart=>{chart.options.scales.x.ticks.color=C.text;chart.options.scales.x.grid.color=C.grid;chart.options.scales.y.ticks.color=C.text;chart.options.scales.y.grid.color=C.grid;if(chart.options.scales.y2){chart.options.scales.y2.ticks.color=C.text;}chart.options.plugins.legend.labels.color=C.text;chart.update();});}"
    + "function updateHeader(d){"
    + "  if(d.clock)setText('clockValue',String(d.clock));"
    + "  setText('cycleValue','cycle '+String(Number(d.cycle)||0)+' / 60');"
    + "  setText('uptimeValue',formatUptime(d.uptimeSeconds));"
    + "}"
    + "function uploadDetailsHtml(u,includeDose){"
    + "  if(!u||!u.valid)return \"<div class='upload-values'>No upload yet.</div>\";"
    + "  let html=\"<div class='upload-values'>\";"
    + "  html+=\"<span>CPM: \"+String(Math.round(Number(u.cpm)||0))+\"</span>\";"
    + "  if(includeDose)html+=\"<span>Dose: \"+(Number(u.outputSieverts)||0).toFixed(4)+\" &micro;Sv/h</span>\";"
    + "  html+=\"<span>Tube 1: \"+String(Math.round(Number(u.actual_cps_1)||0))+\" cps</span>\";"
    + "  html+=\"<span>Tube 2: \"+String(Math.round(Number(u.actual_cps_2)||0))+\" cps</span>\";"
    + "  if(!includeDose){"
    + "    html+=\"<span>HV: \"+(Number(u.tubeVoltage)||0).toFixed(1)+\" V</span>\";"
    + "    html+=\"<span>HV Drive Index: \"+estimateHvDrivePct(u.tubeVoltage).toFixed(1)+\" % est.</span>\";"
    + "    html+=\"<span>Temp: \"+(Number(u.var_temperature)||0).toFixed(1)+\" C</span>\";"
    + "    html+=\"<span>Humidity: \"+(Number(u.var_humidity)||0).toFixed(1)+\" %</span>\";"
    + "    html+=\"<span>Pressure: \"+(Number(u.var_pressure_hpa)||0).toFixed(1)+\" hPa</span>\";"
    + "    html+=\"<span>Light: \"+String(Math.round(Number(u.luminosity)||0))+\" lux</span>\";"
    + "    html+=\"<span>VOC: \"+(Number(u.var_voc_kohm)||0).toFixed(1)+\" kOhm</span>\";"
    + "    html+=\"<span>CO2: \"+(Number(u.var_co2)||0).toFixed(0)+\" ppm</span>\";"
    + "    html+=\"<span>HCHO: \"+(Number(u.var_hcho)||0).toFixed(3)+\" ppm</span>\";"
    + "    html+=\"<span>PM1: \"+String(Math.round(Number(u.var_pm01)||0))+\"</span>\";"
    + "    html+=\"<span>PM2.5: \"+String(Math.round(Number(u.var_pm25)||0))+\"</span>\";"
    + "    html+=\"<span>PM10: \"+String(Math.round(Number(u.var_pm10)||0))+\"</span>\";"
    + "  }"
    + "  html+=\"<span>Epoch: \"+String(Math.round(Number(u.epoch)||0))+\"</span>\";"
    + "  html+=\"</div>\";"
    + "  return html;"
    + "}"
    + "function updateUploads(d){"
    + "  const radStatus=byId('radmonStatus');"
    + "  const uradStatus=byId('uradmonStatus');"
    + "  const radCode=Number(d.statusCodeRadmon)||0;"
    + "  const uradCode=Number(d.statusCodeURadmon)||0;"
    + "  if(radStatus){radStatus.textContent=uploadStatusText(radCode);radStatus.className='badge '+((radCode===-10||radCode===-11)?'badge-neutral':(radCode===200?'badge-ok':'badge-err'));}"
    + "  if(uradStatus){uradStatus.textContent=uploadStatusText(uradCode);uradStatus.className='badge '+((uradCode===-10||uradCode===-11)?'badge-neutral':(uradCode===200?'badge-ok':'badge-err'));}"
    + "  setText('radmonUpdated',formatEpoch(d.radmonUpload&&d.radmonUpload.epoch));"
    + "  setText('uradmonUpdated',formatEpoch(d.uradmonUpload&&d.uradmonUpload.epoch));"
    + "  setHtml('radmonDetails',uploadDetailsHtml(d.radmonUpload,true));"
    + "  setHtml('uradmonDetails',uploadDetailsHtml(d.uradmonUpload,false));"
    + "}"
    + "function updateRadiationCards(d){"
    + "  const dose=Number(d.outputSieverts)||0;"
    + "  const cpm=Number(d.totalcpm)||0;"
    + "  const tube1=Number(d.tube1)||0;"
    + "  const tube2=Number(d.tube2)||0;"
    + "  const hv=Number(d.tubeVoltage)||0;"
    + "  const hvDrive=Number(d.estimatedHvDrivePct);"
    + "  const color=doseColor(dose);"
    + "  setText('doseValue',dose.toFixed(4));"
    + "  byId('doseValue').style.color=color;"
    + "  byId('doseGauge').style.width=clamp(Math.round(cpm*100/(Number(d.cpmGaugeFullScale)||600)),0,100)+'%';"
    + "  byId('doseGauge').style.background=color;"
    + "  setText('cpmValue',String(Math.round(cpm)));"
    + "  setText('tube1Value',String(Math.round(tube1)));"
    + "  setText('tube2Value',String(Math.round(tube2)));"
    + "  setText('cpm1Value',String(Math.round(Number(d.cpm1)||0)));"
    + "  setText('cpm2Value',String(Math.round(Number(d.cpm2)||0)));"
    + "  setText('hvValue',hv.toFixed(1));"
    + "  byId('hvGauge').style.width=clamp(Math.round(hv*100/500),0,100)+'%';"
    + "  byId('hvGauge').style.background=hvColor(hv);"
    + "  setText('hvDriveValue',(Number.isFinite(hvDrive)?hvDrive:estimateHvDrivePct(hv)).toFixed(1));"
    + "  byId('hvDriveGauge').style.width=clamp(Math.round(Number.isFinite(hvDrive)?hvDrive:estimateHvDrivePct(hv)),0,100)+'%';"
    + "  byId('hvDriveGauge').style.background=hvColor(hv);"
    + "}"
    + "function updateResources(d){"
    + "  const freeHeapBytes=Number(d.freeHeapBytes)||0;"
    + "  const totalHeapBytes=Number(d.totalHeapBytes)||0;"
    + "  const heapFreePct=totalHeapBytes>0?(freeHeapBytes*100/totalHeapBytes):0;"
    + "  setText('cpuLoadCore0Value',(Number(d.cpuLoadCore0Pct)||0).toFixed(1));"
    + "  setText('cpuLoadCore1Value',(Number(d.cpuLoadCore1Pct)||0).toFixed(1));"
    + "  setText('loopActiveValue',(Number(d.loopActivePct)||0).toFixed(1));"
    + "  setText('heapValue',(freeHeapBytes/1024).toFixed(1));"
    + "  setText('heapUnit',(freeHeapBytes/1024).toFixed(1)+' / '+(totalHeapBytes/1024).toFixed(1)+' kB ('+heapFreePct.toFixed(1)+'% free)');"
    + "  const freeAppPartitionBytes=Number(d.freeAppPartitionBytes)||0;"
    + "  const totalAppPartitionBytes=Number(d.totalAppPartitionBytes)||0;"
    + "  const appPartitionFreePct=totalAppPartitionBytes>0?(freeAppPartitionBytes*100/totalAppPartitionBytes):0;"
    + "  setText('appPartitionValue',totalAppPartitionBytes>0?(freeAppPartitionBytes/1024).toFixed(1):'n/a');"
    + "  setText('appPartitionUnit',totalAppPartitionBytes>0?((freeAppPartitionBytes/1024).toFixed(1)+' / '+(totalAppPartitionBytes/1024).toFixed(1)+' kB ('+appPartitionFreePct.toFixed(1)+'% free)'):'App partition unavailable');"
    + "  const fsMounted=Boolean(d.fsMounted);"
    + "  const freeFsBytes=Number(d.freeFsBytes)||0;"
    + "  const totalFsBytes=Number(d.totalFsBytes)||0;"
    + "  const fsFreePct=totalFsBytes>0?(freeFsBytes*100/totalFsBytes):0;"
    + "  setText('diskValue',fsMounted?(freeFsBytes/1024).toFixed(1):'n/a');"
    + "  setText('diskUnit',fsMounted?((freeFsBytes/1024).toFixed(1)+' / '+(totalFsBytes/1024).toFixed(1)+' kB ('+fsFreePct.toFixed(1)+'% free)'):'SPIFFS unavailable');"
    + "  const wifiConnected=Boolean(d.wifiConnected);"
    + "  const wifiRssi=Number(d.wifiRssiDbm);"
    + "  const wifiPct=wifiSignalPct(wifiRssi);"
    + "  setText('wifiValue',wifiConnected&&Number.isFinite(wifiRssi)?wifiRssi.toFixed(0)+' dBm':'offline');"
    + "  setText('wifiState',wifiConnected?'connected':'disconnected');"
    + "  const wifiIcon=byId('wifiIcon');"
    + "  if(wifiIcon){const active=wifiBars(wifiPct);Array.from(wifiIcon.children).forEach((bar,index)=>{bar.className='wifi-bar '+(index<active?'active':'off');});}"
    + "  setHtml('wifiDetails','<span>SSID: '+String(d.wifiSsid||'')+'</span>'+'<span>RSSI: '+(wifiConnected&&Number.isFinite(wifiRssi)?wifiRssi.toFixed(0)+' dBm':'n/a')+'</span>'+'<span>Signal: '+String(wifiPct)+'%</span>'+'<span>IP: '+String(d.wifiIp||'0.0.0.0')+'</span>'+'<span>Gateway: '+String(d.wifiGateway||'0.0.0.0')+'</span>'+'<span>MAC: '+String(d.wifiMac||'')+'</span>');"
    + "  setText('i2cValue',String(Math.round(Number(d.i2cDevicesFound)||0)));"
    + "}"
    + "function updateSlowCards(d){"
    + "  const iaq=Number(d.iaq)||0;"
    + "  const iaqAcc=Number(d.iaqAccuracy);"
    + "  const co2=Number(d.co2)||0;"
    + "  const voc=Number(d.voc)||0;"
    + "  const hcho=Number(d.hcho)||0;"
    + "  const humidity=Number(d.humidity)||0;"
    + "  const iaqCol=iaqColor(iaq);"
    + "  setText('iaqTitle','IAQ - '+(IAQ_LABELS[iaqAcc]||'?'));"
    + "  setText('iaqValue',iaq.toFixed(1));"
    + "  byId('iaqValue').style.color=iaqCol;"
    + "  byId('iaqGauge').style.width=clamp(Math.round(iaq*100/500),0,100)+'%';"
    + "  byId('iaqGauge').style.background=iaqCol;"
    + "  setText('co2Value',co2.toFixed(0));"
    + "  byId('co2Gauge').style.width=clamp(Math.round((co2-400)*100/4600),0,100)+'%';"
    + "  setText('vocValue',(voc/1000).toFixed(1));"
    + "  setText('hchoValue',hcho.toFixed(3));"
    + "  setText('pm01Value',String(Math.round(Number(d.pm01)||0)));"
    + "  setText('pm25Value',String(Math.round(Number(d.pm25)||0)));"
    + "  setText('pm10Value',String(Math.round(Number(d.pm10)||0)));"
    + "  setText('tempValue',(Number(d.temperature)||0).toFixed(1));"
    + "  setText('pressureValue',(Number(d.pressure)||0).toFixed(1));"
    + "  setText('humidityValue',humidity.toFixed(1));"
    + "  byId('humidityGauge').style.width=clamp(Math.round(humidity),0,100)+'%';"
    + "  setText('luminosityValue',String(Math.round(Number(d.luminosity)||0)));"
    + "}"
    + "function addPt(chart,label,values){"
    + "  if(chart.data.labels.length>=MAX_PTS){"
    + "    chart.data.labels.shift();"
    + "    chart.data.datasets.forEach(d=>d.data.shift());"
    + "  }"
    + "  chart.data.labels.push(label);"
    + "  chart.data.datasets.forEach((d,i)=>d.data.push(values[i]));"
    + "  chart.update();"
    + "}"
    + "function updateCharts(d){"
    + "  const t=new Date().toLocaleTimeString();"
    + "  addPt(charts.dose,t,[d.outputSieverts]);"
    + "  addPt(charts.cpm,t,[d.totalcpm,d.cpm1,d.cpm2]);"
    + "  addPt(charts.cps,t,[d.tube1,d.tube2,d.sensorMovingAvg]);"
    + "  addPt(charts.th,t,[d.temperature,d.humidity,d.pressure]);"
    + "  addPt(charts.iaq,t,[d.iaq]);"
    + "  addPt(charts.co2,t,[d.co2]);"
    + "  addPt(charts.pm,t,[d.pm01,d.pm25,d.pm10]);"
    + "}"
    + "async function poll(){"
    + "  try{"
    + "    const r=await fetch('/json',{cache:'no-store'});"
    + "    if(!r.ok)return;"
    + "    const d=await r.json();"
    + "    updateHeader(d);"
    + "    updateRadiationCards(d);"
    + "    updateResources(d);"
    + "    updateSlowCards(d);"
    + "    updateUploads(d);"
    + "    updateCharts(d);"
    + "  }catch(e){console.warn('poll error',e);}"
    + "}"
    + "initTheme();"
    + "poll();"
    + "setInterval(poll,FAST_POLL_MS);"
    + "</script>"
    + "<footer style='text-align:center;padding:22px 0 10px;font-size:.77rem;color:var(--muted);border-top:1px solid var(--border);margin-top:28px;'>&copy; 2022&ndash;"
    + String(ntp.formattedTime("%Y"))
    + " &middot; By <a href='https://www.don-zalmrol.be/' target='_blank' rel='noopener'>Don Zalmrol</a> &middot; <a href='https://github.com/DonZalmrol' target='_blank' rel='noopener'>GitHub</a> &middot; <a href='#' onclick='document.getElementById(\"changelogDlg\").showModal();return false;' style='color:var(--muted);text-decoration:none;font-family:monospace;'>" + String(FIRMWARE_VERSION) + "</a></footer>"
    + changelogDialogHtml()
    + "</main></body></html>"
  );

  return chunks;
}

String JsonPage()
{
  String json = "";
  UploadStatusSnapshot uploadStatus = {};
  time_t currentEpoch = ntp.epoch();

  fillUploadStatusSnapshot(uploadStatus);

  doc.clear();
  doc["unixTime"] = currentEpoch;
  doc["uptimeSeconds"] = getUptimeSeconds();
  doc["clock"] = ntp.formattedTime("%T  |  %F");
  doc["cycle"] = increaseSecCount;
  doc["temperature"] = var_temperature;
  doc["pressure"] = var_pressure/100.0;
  doc["humidity"] = var_humidity;
  doc["luminosity"] = luminosity;
  doc["voc"] = var_voc;
  doc["co2"] = var_co2;
  doc["iaq"] = var_iaq;
  doc["iaqAccuracy"] = var_iaqAccuracy;
  doc["hcho"] = var_hcho;
  doc["pm01"] = var_pm01;
  doc["pm25"] = var_pm25;
  doc["pm10"] = var_pm10;
  doc["tubeVoltage"] = tubeVoltage;
  doc["estimatedHvDrivePct"] = estimateHvDrivePct(tubeVoltage);
  doc["tubePresetId"] = activeTubePresetId;
  doc["tubePresetLabel"] = activeTubePresetLabel;
  doc["tubeDeadTimeUs"] = activeTubeDeadTimeSeconds * 1000000.0f;
  doc["tubeConversionFactor"] = activeTubeConversionFactor;
  doc["tubeOperatingVoltageMin"] = activeTubeOperatingVoltageMin;
  doc["tubeOperatingVoltageMax"] = activeTubeOperatingVoltageMax;
  doc["tubeOperatingVoltageRange"] = formatOperatingVoltageRange(activeTubeOperatingVoltageMin, activeTubeOperatingVoltageMax);
  doc["tubePresetNote"] = activeTubePresetNote;
  doc["timezoneOffsetMinutes"] = activeTimezoneOffsetMinutes;
  doc["timezoneOffsetLabel"] = formatUtcOffsetMinutes(activeTimezoneOffsetMinutes);
  doc["dstProfile"] = activeDstProfileId;
  doc["dstProfileLabel"] = dstProfileLabel(activeDstProfileId);
  doc["dstOffsetMinutes"] = activeDstOffsetMinutes;
  doc["hvCalibrationFactor"] = activeHvCalibrationFactor;
  doc["hchoR0"] = activeHchoR0;
  doc["historyRetentionSamples"] = (unsigned long)activeHistoryRetentionSamples;
  doc["historyRetentionHours"] = (unsigned long)getHistoryRetentionHours();
  doc["ntpServer"] = activeNtpServer;
  doc["cpmGaugeFullScale"] = (long)activeCpmGaugeFullScale;
  doc["stationName"] = activeStationName;
  doc["loopActivePct"] = loopActivePct;
  doc["cpuLoadCore0Pct"] = cpuLoadCore0Pct;
  doc["cpuLoadCore1Pct"] = cpuLoadCore1Pct;
  doc["freeHeapBytes"] = esp_get_free_heap_size();
  doc["totalHeapBytes"] = getHeapTotalBytes();
  doc["usedAppPartitionBytes"] = getAppPartitionUsedBytes();
  doc["freeAppPartitionBytes"] = getAppPartitionFreeBytes();
  doc["totalAppPartitionBytes"] = getAppPartitionTotalBytes();
  doc["fsMounted"] = spiffsMounted;
  doc["freeFsBytes"] = getFsFreeBytes();
  doc["totalFsBytes"] = getFsTotalBytes();
  doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
  doc["wifiSsid"] = String(my_ssid);
  doc["wifiRssiDbm"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100;
  doc["wifiSignalPct"] = getWifiSignalPct((WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100L);
  doc["wifiIp"] = WiFi.localIP().toString();
  doc["wifiGateway"] = WiFi.gatewayIP().toString();
  doc["wifiMac"] = WiFi.macAddress();
  doc["i2cDevicesFound"] = i2cDevicesFound;
  doc["totalcpm"] = cpm;
  doc["tube1"] = actual_cps_1;
  doc["tube2"] = actual_cps_2;
  doc["cpm1"] = cpm1;
  doc["cpm2"] = cpm2;
  doc["sensorMovingAvg"] = sensorMovingAvg;
  doc["outputSieverts"] = outputSieverts(cpm);
  doc["statusCodeRadmon"] = uploadStatus.statusCodeRadmon;
  doc["statusCodeURadmon"] = uploadStatus.statusCodeURadmon;
  doc["radmonUploadEnabled"] = activeRadmonUploadEnabled;
  doc["uradmonUploadEnabled"] = activeURadmonUploadEnabled;

  JsonObject radmonJson = doc["radmonUpload"].to<JsonObject>();
  radmonJson["valid"] = uploadStatus.radmon.valid;
  radmonJson["cpm"] = uploadStatus.radmon.cpm;
  radmonJson["actual_cps_1"] = uploadStatus.radmon.actual_cps_1;
  radmonJson["actual_cps_2"] = uploadStatus.radmon.actual_cps_2;
  radmonJson["epoch"] = (unsigned long)uploadStatus.radmon.epoch;
  radmonJson["outputSieverts"] = outputSieverts(uploadStatus.radmon.cpm);

  JsonObject uradmonJson = doc["uradmonUpload"].to<JsonObject>();
  uradmonJson["valid"] = uploadStatus.uradmon.valid;
  uradmonJson["cpm"] = uploadStatus.uradmon.cpm;
  uradmonJson["actual_cps_1"] = uploadStatus.uradmon.actual_cps_1;
  uradmonJson["actual_cps_2"] = uploadStatus.uradmon.actual_cps_2;
  uradmonJson["tubeVoltage"] = uploadStatus.uradmon.tubeVoltage;
  uradmonJson["tubePresetId"] = activeTubePresetId;
  uradmonJson["tubePresetLabel"] = activeTubePresetLabel;
  uradmonJson["tubeDeadTimeUs"] = activeTubeDeadTimeSeconds * 1000000.0f;
  uradmonJson["tubeConversionFactor"] = activeTubeConversionFactor;
  uradmonJson["tubeOperatingVoltageMin"] = activeTubeOperatingVoltageMin;
  uradmonJson["tubeOperatingVoltageMax"] = activeTubeOperatingVoltageMax;
  uradmonJson["tubeOperatingVoltageRange"] = formatOperatingVoltageRange(activeTubeOperatingVoltageMin, activeTubeOperatingVoltageMax);
  uradmonJson["tubePresetNote"] = activeTubePresetNote;
  uradmonJson["timezoneOffsetMinutes"] = activeTimezoneOffsetMinutes;
  uradmonJson["timezoneOffsetLabel"] = formatUtcOffsetMinutes(activeTimezoneOffsetMinutes);
  uradmonJson["dstProfile"] = activeDstProfileId;
  uradmonJson["dstOffsetMinutes"] = activeDstOffsetMinutes;
  uradmonJson["hvCalibrationFactor"] = activeHvCalibrationFactor;
  uradmonJson["hchoR0"] = activeHchoR0;
  uradmonJson["historyRetentionSamples"] = (unsigned long)activeHistoryRetentionSamples;
  uradmonJson["historyRetentionHours"] = (unsigned long)getHistoryRetentionHours();
  uradmonJson["estimatedHvDrivePct"] = estimateHvDrivePct(uploadStatus.uradmon.tubeVoltage);
  uradmonJson["estimatedHvDrivePermille"] = estimateHvDriveUploadPct(uploadStatus.uradmon.tubeVoltage);
  uradmonJson["epoch"] = (unsigned long)uploadStatus.uradmon.epoch;
  uradmonJson["var_temperature"] = uploadStatus.uradmon.var_temperature;
  uradmonJson["var_humidity"] = uploadStatus.uradmon.var_humidity;
  uradmonJson["var_pressure_hpa"] = uploadStatus.uradmon.var_pressure / 100.0;
  uradmonJson["luminosity"] = uploadStatus.uradmon.luminosity;
  uradmonJson["var_voc_kohm"] = uploadStatus.uradmon.var_voc / 1000.0;
  uradmonJson["var_co2"] = uploadStatus.uradmon.var_co2;
  uradmonJson["var_hcho"] = uploadStatus.uradmon.var_hcho;
  uradmonJson["var_pm01"] = uploadStatus.uradmon.var_pm01;
  uradmonJson["var_pm25"] = uploadStatus.uradmon.var_pm25;
  uradmonJson["var_pm10"] = uploadStatus.uradmon.var_pm10;

  serializeJson(doc, json);

  return json;
}