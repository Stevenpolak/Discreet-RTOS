#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include <dimmable_light.h>
#include <Wire.h>
#include "max6675.h"
#include <PID_v1.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

// Forward declarations (Arduino's ctags prototype generation misses these).
int basePumpPowerForSetpoint(double Pumpsetpoint);
String getContentType(String filename);
void controlStep(unsigned long now);

// Wi-Fi credentials, loaded from config.json
String ssid;
String password;

WebServer server(80);

// SD card pins
#define SD_CS     32
#define SD_MOSI  25
#define SD_MISO  26
#define SD_SCK   33

//SSR Pins
#define SSR_PIN 13

// Dimmer Pins
#define syncPin        19   // Zero-cross sync input
#define thyristorPin   18   // Thyristor (dimmer PWM output)

// Thermocouple SPI (HSPI)
#define thermoCS  22
#define thermoCLK 23
#define thermoDO  21

//Pressure sensor pin
#define pressurepin 34

// Buzzer pin
#define BUZZER_PIN 4

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// PID state
double setpoint, input, output;
double Kp = 48.0, Ki = 8, Kd = 50.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//Pressure Variables
int PrePressureSetpoint = 3; // pre-infusion target, fixed
int PressureTarget = 9;      // overwritten once a shot starts
int pumppower = 0;
int maxPressure = 12;
double currentPressure = 0;

// What the PID/pump-ramp want; controlStep() resolves these against fault/mode before writing actuators.
double heaterDemand = 0;
int pumpDemand = 0;

// What was actually written to the actuators last cycle, after fault/mode resolution.
int resolvedPumpPower = 0;

// SSR deadman authorization deadline; declared early so buildTelemetrySnapshot() can see it. See its full use below.
volatile unsigned long ssrAuthorizedUntilMs = 0;

// Timing intervals
const unsigned long PRESS_INTERVAL = 50;  // ms, pressure sampling gate
const unsigned long PID_INTERVAL = 250;  // ms, PID sampling gate
const unsigned long PUMP_ADJUST_INTERVAL = 200;  // ms, pump ramp-rate gate

// Shot-end debounce: syncPin must read HIGH continuously across this many samples and this long. TODO(bench).
const unsigned long AC_OFF_DEBOUNCE_MS = 300;
const int AC_OFF_MIN_SAMPLES = 20;

// Timer Variables
unsigned long lastPIDTime = 0;
unsigned long DimlastUpdate = 0;
unsigned long lastPumpAdjust = 0;
unsigned long LastPressCall = 0;
unsigned long acDetectedTime = 0;
unsigned long elapsedTime = 0; // Shot time in milliseconds
int actime = 0;  // Shot time in seconds

//Steam Veriables
bool steaming = false;       // hysteresis flag for the steam-ready beep
bool steamRequested = false; // true while steam mode is active
double steamSetpoint;

//Other Variables
int offset = 9; // probe offset - ask for 100, get 91; tune this

bool acDetected = false;
bool shotStarted = false;
bool pumpPowerSetPreinf = false;
bool pumpPowerSetExtraction = false;

// AC-off debounce state
bool acOffPending = false;
unsigned long acOffSince = 0;
int acOffSamples = 0;

enum class OperatingMode : uint8_t { NORMAL, TEMP_ONLY };
enum class ShotState : uint8_t { IDLE, PREINFUSION, BLOOM, EXTRACTION, COMPLETE, FAULT };
enum class FaultCode : uint8_t { NONE, INVALID_TEMPERATURE };

const char* toString(OperatingMode mode) {
  switch (mode) {
    case OperatingMode::TEMP_ONLY: return "TEMP_ONLY";
    default: return "NORMAL";
  }
}

const char* toString(ShotState state) {
  switch (state) {
    case ShotState::PREINFUSION: return "PREINFUSION";
    case ShotState::BLOOM: return "BLOOM";
    case ShotState::EXTRACTION: return "EXTRACTION";
    case ShotState::COMPLETE: return "COMPLETE";
    case ShotState::FAULT: return "FAULT";
    default: return "IDLE";
  }
}

const char* toString(FaultCode code) {
  switch (code) {
    case FaultCode::INVALID_TEMPERATURE: return "INVALID_TEMPERATURE";
    default: return "NONE";
  }
}

OperatingMode currentMode = OperatingMode::NORMAL;
ShotState currentShotState = ShotState::IDLE;
FaultCode currentFault = FaultCode::NONE;

// Latched per-shot settings - brew temp, pressure target and phase durations can't change mid-shot.
struct ControlSettings {
  double setpoint = 0;         // brew target temperature, offset-adjusted
  int preinftime = 8;          // seconds, 0..20
  int bloomtime = 0;           // seconds, 0..20
  int pressuresetpoint = 9;    // bar, 3..13
  unsigned long revision = 0;  // bumped on every accepted edit
};

ControlSettings activeSettings;   // in effect while idle; source for the next shot's latch
ControlSettings shotSettings;     // latched at shot start; read-only for the shot's duration
ControlSettings pendingSettings;  // latest edit; promoted to active on return to IDLE

// A validated /adjust or /saveConfig request, sent to the control task via commandQueue.
struct ControlCommand {
  enum class Type : uint8_t {
    NONE,
    SET_BREW_SETPOINT,           // relative delta
    SET_BREW_SETPOINT_ABSOLUTE,  // absolute offset-adjusted target
    SET_PREINFTIME,
    SET_BLOOMTIME,
    SET_PRESSURESETPOINT,
    START_STEAM,
    STOP_STEAM,
    SET_MODE,                    // absolute OperatingMode
    SET_TUNINGS,                 // absolute Kp/Ki/Kd
    SET_STEAM_SETPOINT,          // absolute offset-adjusted value
    SET_OFFSET,                  // absolute value
  } type = Type::NONE;
  int delta = 0;                 // relative adjustment, or absolute value for SET_OFFSET
  double value = 0;              // absolute value for SET_BREW_SETPOINT_ABSOLUTE / SET_STEAM_SETPOINT
  OperatingMode mode = OperatingMode::NORMAL;  // used by SET_MODE
  double kp = 0, ki = 0, kd = 0;  // used by SET_TUNINGS
};

// Bounded queue: the only channel by which an HTTP handler may influence control state.
QueueHandle_t commandQueue = nullptr;
const int COMMAND_QUEUE_DEPTH = 8;

// Snapshot of control state published every cycle for the web side to read.
struct TelemetrySnapshot {
  OperatingMode mode;
  ShotState shotState;
  FaultCode fault;
  double temperature;  // offset-corrected, deg C
  double pressure;      // bar
  int pumpPower;         // requested/ramped level, before fault/mode resolution
  int resolvedPumpPower;  // actually written to the dimmer this cycle
  unsigned long elapsedShotTimeMs;
  unsigned long activeSettingsRevision;
  unsigned long shotSettingsRevision;
  unsigned long pendingSettingsRevision;
  unsigned long timestampMs;

  double displaySetpoint;  // setpoint, resolved for steam mode
  int pendingPreinftime;
  int pendingBloomtime;
  int pendingPressuresetpoint;
};

// Length-1: overwrite semantics, so publishing never blocks and reads never wait.
QueueHandle_t telemetryQueue = nullptr;

// Dedicated control task: runs controlStep() every 10ms, independent of loop().
const TickType_t CONTROL_TASK_PERIOD_TICKS = pdMS_TO_TICKS(10);

// Core 1 (with loopTask), priority 5 (above loopTask's 1, below WiFi/event tasks ~18-23) - preempts loop() without starving system tasks. Stack 4096B is a starting estimate, not measured. TODO(bench).
const uint32_t CONTROL_TASK_STACK_SIZE = 4096;
const UBaseType_t CONTROL_TASK_PRIORITY = 5;
const BaseType_t CONTROL_TASK_CORE = 1;

TaskHandle_t controlTaskHandle = nullptr;

// Diagnostics, updated every cycle here, reported from loop() to keep Serial off the critical task.
unsigned long controlTaskWorstCycleUs = 0;
unsigned long controlTaskDeadlineMisses = 0;
bool controlTaskWdtSubscribed = false;

// Subscribes to the Task WDT explicitly; runs controlStep() on a fixed period via vTaskDelayUntil().
void controlTaskEntry(void* pvParameters) {
  esp_err_t wdtErr = esp_task_wdt_add(NULL);
  controlTaskWdtSubscribed = (wdtErr == ESP_OK);
  if (!controlTaskWdtSubscribed) {
    Serial.printf("esp_task_wdt_add(controlTask) failed: %d\n", wdtErr);
  }

  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    unsigned long nowMs = millis();
    unsigned long cycleStartUs = micros();

    controlStep(nowMs);

    unsigned long cycleUs = micros() - cycleStartUs;
    if (cycleUs > controlTaskWorstCycleUs) controlTaskWorstCycleUs = cycleUs;

    // Feed the watchdog only after a full cycle completes.
    if (controlTaskWdtSubscribed) {
      esp_err_t resetErr = esp_task_wdt_reset();
      if (resetErr != ESP_OK) {
        Serial.printf("esp_task_wdt_reset(controlTask) failed: %d\n", resetErr);
        controlTaskWdtSubscribed = false;
      }
    }

    // pdFALSE means this cycle missed its deadline.
    if (xTaskDelayUntil(&lastWakeTime, CONTROL_TASK_PERIOD_TICKS) == pdFALSE) {
      controlTaskDeadlineMisses++;
    }
  }
}

TelemetrySnapshot buildTelemetrySnapshot(unsigned long now) {
  TelemetrySnapshot snap;
  snap.mode = currentMode;
  snap.shotState = currentShotState;
  snap.fault = currentFault;
  snap.temperature = input - offset;
  snap.pressure = currentPressure;
  snap.pumpPower = pumppower;
  snap.resolvedPumpPower = resolvedPumpPower;
  snap.elapsedShotTimeMs = acDetected ? elapsedTime : 0;
  snap.activeSettingsRevision = activeSettings.revision;
  snap.shotSettingsRevision = shotSettings.revision;
  snap.pendingSettingsRevision = pendingSettings.revision;
  snap.timestampMs = now;
  // While steaming, the live target is steamSetpoint, not the brew target.
  snap.displaySetpoint = steamRequested ? setpoint : pendingSettings.setpoint;
  snap.pendingPreinftime = pendingSettings.preinftime;
  snap.pendingBloomtime = pendingSettings.bloomtime;
  snap.pendingPressuresetpoint = pendingSettings.pressuresetpoint;
  return snap;
}

// Idle here means !acDetected, not currentShotState == IDLE (a telemetry label, not a control input).
void acceptPreinftimeEdit(int newValue) {
  pendingSettings.preinftime = newValue;
  pendingSettings.revision++;
  if (!acDetected) {
    activeSettings.preinftime = newValue;
    activeSettings.revision = pendingSettings.revision;
  }
}

void acceptBloomtimeEdit(int newValue) {
  pendingSettings.bloomtime = newValue;
  pendingSettings.revision++;
  if (!acDetected) {
    activeSettings.bloomtime = newValue;
    activeSettings.revision = pendingSettings.revision;
  }
}

void acceptPressuresetpointEdit(int newValue) {
  pendingSettings.pressuresetpoint = newValue;
  pendingSettings.revision++;
  if (!acDetected) {
    activeSettings.pressuresetpoint = newValue;
    activeSettings.revision = pendingSettings.revision;
  }
}

// Guarded by steamRequested, not steaming (a temperature-hysteresis flag, not a "steam wanted" flag).
void acceptBrewSetpointEdit(double newValue) {
  pendingSettings.setpoint = newValue;
  pendingSettings.revision++;
  if (!acDetected) {
    activeSettings.setpoint = newValue;
    activeSettings.revision = pendingSettings.revision;
    if (!steamRequested) setpoint = newValue;
  }
}

// Validates and applies one ControlCommand; the sole writer of control state.
bool applyControlCommand(const ControlCommand& cmd) {
  bool idle = !acDetected;
  switch (cmd.type) {
    case ControlCommand::Type::SET_PREINFTIME: {
      int base = idle ? activeSettings.preinftime : pendingSettings.preinftime;
      acceptPreinftimeEdit(constrain(base + cmd.delta, 0, 20));
      if (idle) pumppower = basePumpPowerForSetpoint(PrePressureSetpoint);
      return true;
    }
    case ControlCommand::Type::SET_BLOOMTIME: {
      int base = idle ? activeSettings.bloomtime : pendingSettings.bloomtime;
      acceptBloomtimeEdit(constrain(base + cmd.delta, 0, 20));
      return true;
    }
    case ControlCommand::Type::SET_BREW_SETPOINT: {
      double base = idle ? activeSettings.setpoint : pendingSettings.setpoint;
      acceptBrewSetpointEdit(constrain(base + cmd.delta, 10 + offset, 96 + offset));
      return true;
    }
    case ControlCommand::Type::SET_PRESSURESETPOINT: {
      int base = idle ? activeSettings.pressuresetpoint : pendingSettings.pressuresetpoint;
      int newValue = constrain(base + cmd.delta, 3, 13);
      acceptPressuresetpointEdit(newValue);
      if (idle) pumppower = basePumpPowerForSetpoint(newValue);
      return true;
    }
    case ControlCommand::Type::START_STEAM:
      steamRequested = true;
      setpoint = steamSetpoint;
      return true;
    case ControlCommand::Type::STOP_STEAM:
      steamRequested = false;
      setpoint = activeSettings.setpoint;
      return true;
    case ControlCommand::Type::SET_BREW_SETPOINT_ABSOLUTE:
      acceptBrewSetpointEdit(constrain(cmd.value, 10 + offset, 96 + offset));
      return true;
    case ControlCommand::Type::SET_MODE:
      currentMode = cmd.mode;
      return true;
    case ControlCommand::Type::SET_TUNINGS:
      Kp = cmd.kp;
      Ki = cmd.ki;
      Kd = cmd.kd;
      myPID.SetTunings(Kp, Ki, Kd);
      return true;
    case ControlCommand::Type::SET_STEAM_SETPOINT:
      steamSetpoint = cmd.value;
      return true;
    case ControlCommand::Type::SET_OFFSET:
      offset = cmd.delta;
      return true;
    default:
      return false;
  }
}

// Sends a command to commandQueue; returns false on a full/uncreated queue.
bool sendControlCommand(const ControlCommand& cmd) {
  return commandQueue != nullptr && xQueueSend(commandQueue, &cmd, 0) == pdTRUE;
}

DimmableLight light(thyristorPin);

// Buzzer state, driven by an ESP one-shot timer independent of loop().
volatile bool buzzerActive = false;
bool buzzerPinOn = false;
int buzzerBeepsRemaining = 0;
int buzzerOnMs = 0;
int buzzerOffMs = 0;
esp_timer_handle_t buzzerTimer = nullptr;
SemaphoreHandle_t buzzerMutex = nullptr;
const TickType_t BUZZER_MUTEX_WAIT_TICKS = pdMS_TO_TICKS(20); // bounds this callback's wait on the shared esp_timer task

// Bounded wait, not portMAX_DELAY: shares its dispatch task with the SSR deadman, which must never be blocked.
void buzzerTimerCallback(void *arg) {
  uint64_t nextDelayUs = 0;

  if (xSemaphoreTake(buzzerMutex, BUZZER_MUTEX_WAIT_TICKS) != pdTRUE) return;
  if (buzzerActive && buzzerPinOn) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerPinOn = false;
    nextDelayUs = (uint64_t)buzzerOffMs * 1000;
  } else if (buzzerActive) {
    buzzerBeepsRemaining--;
    if (buzzerBeepsRemaining <= 0) {
      buzzerActive = false;
    } else {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerPinOn = true;
      nextDelayUs = (uint64_t)buzzerOnMs * 1000;
    }
  }

  if (nextDelayUs > 0 && esp_timer_start_once(buzzerTimer, nextDelayUs) != ESP_OK) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerPinOn = false;
    buzzerActive = false;
  }
  xSemaphoreGive(buzzerMutex);
}

bool initBuzzer() {
  buzzerMutex = xSemaphoreCreateMutex();
  if (buzzerMutex == nullptr) return false;

  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = &buzzerTimerCallback;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.name = "buzzer";
  return esp_timer_create(&timerArgs, &buzzerTimer) == ESP_OK;
}

void queueBuzzer(int times, int beepDurationMs, int pauseDurationMs) {
  if (times <= 0 || beepDurationMs <= 0 || pauseDurationMs <= 0 ||
      buzzerTimer == nullptr || buzzerMutex == nullptr) return;

  xSemaphoreTake(buzzerMutex, portMAX_DELAY);
  if (buzzerActive) {
    xSemaphoreGive(buzzerMutex);
    return;
  }
  buzzerOnMs = beepDurationMs;
  buzzerOffMs = pauseDurationMs;
  buzzerBeepsRemaining = times;
  buzzerPinOn = true;
  buzzerActive = true;
  digitalWrite(BUZZER_PIN, HIGH);
  if (esp_timer_start_once(buzzerTimer, (uint64_t)buzzerOnMs * 1000) != ESP_OK) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerPinOn = false;
    buzzerActive = false;
  }
  xSemaphoreGive(buzzerMutex);
}

void waitForBuzzer() {
  while (buzzerActive) {
    delay(1);
  }
}

// Time-proportional SSR output with an independent deadman. controlStep() only ever refreshes an
// authorization deadline; ssrDeadmanCallback() is the only thing that writes SSR_PIN, and forces
// it off the instant the authorization goes stale - a wedged control task can't leave it energized.

const unsigned long SSR_WINDOW_MS = 1000;         // ms, time-proportional window. TODO(bench).
const unsigned long SSR_AUTH_TIMEOUT_MS = 200;    // ms, authorization validity without refresh. TODO(bench).
const unsigned long SSR_ENFORCE_INTERVAL_MS = 10; // ms, deadman poll cadence; also the real duty-resolution ceiling

static_assert(SSR_AUTH_TIMEOUT_MS >= 4 * SSR_ENFORCE_INTERVAL_MS,
  "SSR_AUTH_TIMEOUT_MS must stay well above SSR_ENFORCE_INTERVAL_MS.");

esp_timer_handle_t ssrDeadmanTimer = nullptr;

// Heartbeat only; the deadline check in ssrDeadmanCallback() is what actually protects the pin.
volatile unsigned long ssrDeadmanLastRunMs = 0;

// Set by ArduinoOTA callbacks; forces heaterOut off for the whole OTA session.
volatile bool otaInProgress = false;

// Runs in the esp_timer service task (shared with the buzzer), at ESP_TASK_TIMER_PRIO (22), above
// controlTask's priority - a wedged/crashed control task can't stop this from running.
void ssrDeadmanCallback(void* arg) {
  unsigned long now = millis();
  ssrDeadmanLastRunMs = now;
  bool authorized = (long)(ssrAuthorizedUntilMs - now) > 0;
  digitalWrite(SSR_PIN, authorized ? HIGH : LOW);
}

bool initSsrDeadman() {
  digitalWrite(SSR_PIN, LOW);  // safe default before the deadman is running
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = &ssrDeadmanCallback;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.name = "ssr_deadman";
  if (esp_timer_create(&timerArgs, &ssrDeadmanTimer) != ESP_OK) return false;
  return esp_timer_start_periodic(ssrDeadmanTimer, (uint64_t)SSR_ENFORCE_INTERVAL_MS * 1000) == ESP_OK;
}

//Functions
void handleApplyTheme() {

    if (!server.hasArg("name")) {
        server.send(400, "text/plain", "No theme specified");
        return;
    }

    String themeName = server.arg("name");

    if (!themeName.endsWith(".css")) {
        server.send(400, "text/plain", "Invalid theme file");
        return;
    }

    String sourcePath = "/" + themeName;
    String targetPath = "/global.css";

    if (!SD.exists(sourcePath)) {
        server.send(404, "text/plain", "Theme not found");
        return;
    }

    File sourceFile = SD.open(sourcePath, FILE_READ);

    if (!sourceFile) {
        server.send(500, "text/plain", "Failed to open theme file");
        return;
    }

    if (SD.exists(targetPath)) {
        SD.remove(targetPath);
    }

    File targetFile = SD.open(targetPath, FILE_WRITE);

    if (!targetFile) {
        sourceFile.close();
        server.send(500, "text/plain", "Failed to create global.css");
        return;
    }

    while (sourceFile.available()) {
        targetFile.write(sourceFile.read());
    }

    sourceFile.close();
    targetFile.close();

    server.send(200, "text/plain", "OK");

    File themeFile = SD.open("/currentTheme.txt", FILE_WRITE);

    if (themeFile) {
      themeFile.print(themeName);
      themeFile.close();
    }

}

void handleFileRequest() {
  String path = server.uri();
  if (path.endsWith("/")) path += "index.html";

  String contentType = getContentType(path);

  File file = SD.open(path);
  if (!file || file.isDirectory()) {
    server.send(404, "text/plain", "File Not Found");
    return;
  }

  server.streamFile(file, contentType);
  file.close();
}

void handleListFiles() {
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    server.send(500, "application/json", "[]");
    return;
  }

  String json = "[";
  File file = root.openNextFile();
  bool first = true;
  while (file) {
    if (!first) json += ",";
    json += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size()) + "}";
    first = false;
    file = root.openNextFile();
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing 'name' argument");
    return;
  }

  String filename = server.arg("name");
  if (!filename.startsWith("/")) {
    filename = "/" + filename;
  }

  if (SD.exists(filename)) {
    if (SD.remove(filename)) {
      server.send(200, "text/plain", "File deleted successfully");
    } else {
      server.send(500, "text/plain", "Failed to delete file");
    }
  } else {
    server.send(404, "text/plain", "File Not Found");
  }
}

void handleUpload() {
  static File uploadFile;
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    String filename = "/" + upload.filename;
    uploadFile = SD.open(filename, FILE_WRITE);
    if (!uploadFile) {
      Serial.println("Failed to open file for writing");
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      Serial.println("Upload finished");
    }
  }
}

void handleAdjust() {
  String var = server.arg("var");
  int val = server.arg("val").toInt();

  ControlCommand cmd;
  if (var == "preinftime") cmd.type = ControlCommand::Type::SET_PREINFTIME;
  else if (var == "bloomtime") cmd.type = ControlCommand::Type::SET_BLOOMTIME;
  else if (var == "setpoint") cmd.type = ControlCommand::Type::SET_BREW_SETPOINT;
  else if (var == "steam") cmd.type = ControlCommand::Type::START_STEAM;
  else if (var == "stopsteam") cmd.type = ControlCommand::Type::STOP_STEAM;
  else if (var == "pressuresetpoint") cmd.type = ControlCommand::Type::SET_PRESSURESETPOINT;
  cmd.delta = val;

  // Unrecognized var falls through to NONE and is a no-op, never queued.
  if (cmd.type == ControlCommand::Type::NONE) {
    server.send(200, "text/plain", "OK");
    return;
  }
  if (sendControlCommand(cmd)) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(503, "text/plain", "Busy, try again");
  }
}

void handleTemp() {
  StaticJsonDocument<128> doc;
  doc["temp"] = input - offset;
  doc["setpoint"] = setpoint - offset;
  doc["pressure"] = currentPressure;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetValues() {
  // Read via xQueuePeek() - never blocks; falls back to a direct build if the queue isn't ready yet.
  TelemetrySnapshot snap;
  if (telemetryQueue == nullptr || xQueuePeek(telemetryQueue, &snap, 0) != pdTRUE) {
    snap = buildTelemetrySnapshot(millis());
  }

  StaticJsonDocument<384> doc;
  doc["setpoint"] = int(snap.displaySetpoint - offset);
  doc["preinftime"] = snap.pendingPreinftime;
  doc["bloomtime"] = snap.pendingBloomtime;
  doc["pressure"] = snap.pressure;
  doc["pumppower"] = snap.pumpPower;
  doc["pressuresetpoint"] = snap.pendingPressuresetpoint;
  doc["actime"] = snap.elapsedShotTimeMs / 1000;
  doc["temp"] = snap.temperature;
  doc["Kp"] = Kp;
  doc["Ki"] = Ki;
  doc["Kd"] = Kd;
  doc["steamSetpoint"] = steamSetpoint;

  doc["mode"] = toString(snap.mode);
  doc["shotState"] = toString(snap.shotState);
  doc["fault"] = toString(snap.fault);
  doc["activeSettingsRevision"] = snap.activeSettingsRevision;
  doc["shotSettingsRevision"] = snap.shotSettingsRevision;
  doc["pendingSettingsRevision"] = snap.pendingSettingsRevision;
  doc["snapshotTimestampMs"] = snap.timestampMs;

  // heaterOn computed fresh from ssrAuthorizedUntilMs, not from a stale queued snapshot.
  doc["heaterOn"] = (long)(ssrAuthorizedUntilMs - millis()) > 0;
  doc["resolvedPumpPower"] = snap.resolvedPumpPower;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handlePressure() {
  server.send(200, "text/plain", String(currentPressure, 2));
}

void getCurrentTheme() {

    if (!SD.exists("/currentTheme.txt")) {
        server.send(200, "text/plain", "None");
        return;
    }

    File file = SD.open("/currentTheme.txt");

    if (!file) {
        server.send(500, "text/plain", "Error");
        return;
    }

    String theme = file.readString();

    file.close();

    server.send(200, "text/plain", theme);
}

// Computes heaterDemand only; controlStep() resolves it and hands off to the SSR deadman.
// Fault clears only after 3 consecutive good readings, to debounce an intermittent sensor.
const int FAULT_CLEAR_STABLE_READINGS = 3;
int faultClearStreak = 0;

const double MIN_PLAUSIBLE_TEMP_C = 5; // lower sanity bound on input; TODO(bench)

void runPID() {

  unsigned long PIDnow = millis();

  if ((PIDnow - lastPIDTime >= PID_INTERVAL)) {
    lastPIDTime = PIDnow;

    input = thermocouple.readCelsius();

    if (isnan(input) || input < MIN_PLAUSIBLE_TEMP_C || input > 160) {
      currentFault = FaultCode::INVALID_TEMPERATURE;
      faultClearStreak = 0;
      heaterDemand = 0;
      return;
    }
    faultClearStreak++;
    if (faultClearStreak >= FAULT_CLEAR_STABLE_READINGS) {
      currentFault = FaultCode::NONE;
    }

    myPID.Compute();
    heaterDemand = output; // full 0-255 range; controlStep() converts to a time-proportional duty

  }
}

void GetPressure(){

  if (!acDetected) { // no shot in progress - sensor reads pre-solenoid boiler pressure otherwise
    currentPressure = 0;
    return;
  }

  if (elapsedTime < 500) { // let pressure-build artifacts settle first
    currentPressure = 0;
    return;
  }

  unsigned long Pnow = millis();

  if (Pnow - LastPressCall >= PRESS_INTERVAL) {
    LastPressCall = Pnow;
    int raw = analogRead(pressurepin);
    currentPressure = (raw * (maxPressure / 4095.0));
    currentPressure = round(currentPressure * 100) / 100.0;  // 2 decimals
  }

}

String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  if (filename.endsWith(".css"))  return "text/css";
  if (filename.endsWith(".js"))   return "application/javascript";
  if (filename.endsWith(".png"))  return "image/png";
  if (filename.endsWith(".jpg"))  return "image/jpeg";
  if (filename.endsWith(".ico"))  return "image/x-icon";
  if (filename.endsWith(".json")) return "application/json";
  return "text/plain";
}

int basePumpPowerForSetpoint(double Pumpsetpoint) {
  if (Pumpsetpoint <= 3) return 140;
  if (Pumpsetpoint <= 4) return 143;
  if (Pumpsetpoint <= 5) return 147;
  if (Pumpsetpoint <= 6) return 149;
  if (Pumpsetpoint <= 7) return 153;
  if (Pumpsetpoint <= 8) return 155;
  if (Pumpsetpoint <= 9) return 160;
  if (Pumpsetpoint <= 10) return 170;
  return 190;
}

// Computes pumpDemand only; controlStep() is the sole writer of the dimmer.
void updatePumpRamp() {

  unsigned long now = millis();

  if (now - DimlastUpdate >= PRESS_INTERVAL) {
    DimlastUpdate = now;

    // Slow: adjust pump power once per PUMP_ADJUST_INTERVAL.
    if (now - lastPumpAdjust >= PUMP_ADJUST_INTERVAL) {
      lastPumpAdjust = now;
      if (currentPressure < PressureTarget - 0.2) { pumppower = constrain(pumppower + 1, 120, 255); }
      else if (currentPressure > PressureTarget + 0.2) { pumppower = constrain(pumppower - 2, 120, 255); }
    }

    // Fast: cut pump on overpressure every cycle.
    if (currentPressure > PressureTarget + 0.3) { pumpDemand = 0; }
    else { pumpDemand = pumppower; }

  }

}

void setupServerRoutes() {

  server.on("/upload", HTTP_POST, []() {
    server.sendHeader("Location", "/upload.html?success=1");
    server.send(303);
  }, handleUpload);

  server.on("/", HTTP_GET, []() {
    File file = SD.open("/index.html");
    if (!file) {
      server.send(500, "text/plain", "File not found");
      return;
    }
    server.streamFile(file, "text/html");
    file.close();
  });

  server.on("/getConfig", HTTP_GET, []() {

    File file = SD.open("/config.json", FILE_READ);

    if (!file) {
      Serial.println("Failed to open config.json");
      server.send(500, "text/plain", "Config not found");
      return;
    }

    server.streamFile(file, "application/json");

    file.close();

  });

  server.on("/saveConfig", HTTP_POST, []() {

    StaticJsonDocument<256> doc;

    if (deserializeJson(doc, server.arg("plain"))) {
      server.send(400, "text/plain", "Invalid JSON");
      return;
    }

    SD.remove("/config.json");

    File file = SD.open("/config.json", FILE_WRITE);

    if (!file) {
      server.send(500, "text/plain", "Write failed");
      return;
    }

    serializeJson(doc, file);

    file.close();

    // Only fields present in the client's JSON are sent, so an omitted field is never re-applied.
    bool tuningsOk = true;
    if (!doc["Kp"].isNull() && !doc["Ki"].isNull() && !doc["Kd"].isNull()) {
      ControlCommand tuningsCmd;
      tuningsCmd.type = ControlCommand::Type::SET_TUNINGS;
      tuningsCmd.kp = doc["Kp"];
      tuningsCmd.ki = doc["Ki"];
      tuningsCmd.kd = doc["Kd"];
      tuningsOk = sendControlCommand(tuningsCmd);
    }

    bool offsetOk = true;
    if (!doc["offset"].isNull()) {
      ControlCommand offsetCmd;
      offsetCmd.type = ControlCommand::Type::SET_OFFSET;
      offsetCmd.delta = doc["offset"];
      offsetOk = sendControlCommand(offsetCmd);
    }

    bool setpointOk = true;
    if (!doc["setpoint"].isNull()) {
      ControlCommand setpointCmd;
      setpointCmd.type = ControlCommand::Type::SET_BREW_SETPOINT_ABSOLUTE;
      setpointCmd.value = (double)doc["setpoint"] + offset;
      setpointOk = sendControlCommand(setpointCmd);
    }

    bool steamOk = true;
    if (!doc["steamSetpoint"].isNull()) {
      ControlCommand steamCmd;
      steamCmd.type = ControlCommand::Type::SET_STEAM_SETPOINT;
      steamCmd.value = (double)doc["steamSetpoint"] + offset;
      steamOk = sendControlCommand(steamCmd);
    }

    bool modeOk = true;
    if (!doc["PIDonly"].isNull()) {
      ControlCommand modeCmd;
      modeCmd.type = ControlCommand::Type::SET_MODE;
      modeCmd.mode = doc["PIDonly"] ? OperatingMode::TEMP_ONLY : OperatingMode::NORMAL;
      modeOk = sendControlCommand(modeCmd);
    }

    Serial.println("Config saved");

    if (tuningsOk && offsetOk && setpointOk && steamOk && modeOk) {
      server.send(200, "text/plain", "Saved");
    } else {
      server.send(503, "text/plain", "Saved to file, but busy - some settings not applied yet, try again");
    }

  });

  server.on("/applyTheme", HTTP_GET, handleApplyTheme);
  server.on("/getCurrentTheme", HTTP_GET, getCurrentTheme);
  server.on("/adjust", handleAdjust);
  server.on("/temp", handleTemp);
  server.on("/getValues", handleGetValues);
  server.on("/pressure", handlePressure);
  server.on("/listFiles", HTTP_GET, handleListFiles);
  server.on("/delete", HTTP_GET, handleDelete);
  server.onNotFound(handleFileRequest);

}

void startSD(){
  // setup()-only, so blocking here is safe.
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    queueBuzzer(3,1000,200);
  } else {
    Serial.println("SD card initialized.");
    queueBuzzer(1,100,100);
  }
  waitForBuzzer();
}

void loadSDConfig() {
  startSD();

  bool PIDonly = false; // config.json field; only meaningful here

  File configFile = SD.open("/config.json");
  if (configFile) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, configFile);
    configFile.close();

    ssid = doc["ssid"] | "";
    password = doc["password"] | "";
    Kp = doc["Kp"] | Kp;
    Ki = doc["Ki"] | Ki;
    Kd = doc["Kd"] | Kd;
    setpoint = doc["setpoint"] | setpoint;
    offset = doc["offset"] | offset;
    steamSetpoint = doc["steamSetpoint"] | steamSetpoint;
    PIDonly = doc["PIDonly"] | PIDonly;

  } else {
    Kp = 80;
    Ki = 6;
    Kd = 55;
    setpoint = 93;
    offset = 9;
    steamSetpoint = 140;
  }

  setpoint = setpoint + offset;
  steamSetpoint = steamSetpoint + offset;
  setpoint = constrain(setpoint, 10 + offset, 96 + offset); // same bound the live-session clamp uses
  myPID.SetTunings(Kp, Ki, Kd); // must be applied explicitly - the PID doesn't re-read the globals

  currentMode = PIDonly ? OperatingMode::TEMP_ONLY : OperatingMode::NORMAL;

  activeSettings.setpoint = setpoint;
  activeSettings.revision = 1;
  pendingSettings = activeSettings;
  shotSettings = activeSettings;

  SD.end();
  SPI.end();
  delay(200);
}

void startWiFi(){

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long t0 = millis();
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
   delay(500);
   Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    queueBuzzer(5,1000,200);
    Serial.println("FAILED connecting to WiFi");
  } else {
    queueBuzzer(1,500,100);
    Serial.println("WiFi Connected!");
    WiFi.config(WiFi.localIP(), IPAddress(0,0,0,0), WiFi.subnetMask(), IPAddress(0,0,0,0));
    Serial.println("IP:  " + WiFi.localIP().toString());
    Serial.println("GW:  " + WiFi.gatewayIP().toString());
    Serial.println("DNS: " + WiFi.dnsIP().toString());
  }
  waitForBuzzer();

}

void steam(){

  if (input > steamSetpoint - 5 && !steaming){
    queueBuzzer(3,100,100);
    steaming = true;
  }
  if (input < steamSetpoint - 20 && steaming){steaming = false;}

}

void setup() {

  Serial.begin(115200);

  // SSR pin/deadman first, before anything else - GPIO13 floats on reset otherwise.
  pinMode(SSR_PIN, OUTPUT);
  if (!initSsrDeadman()) {
    Serial.println("Failed to create SSR deadman timer - restarting");
    delay(100);
    ESP.restart();
  }

  delay(2000);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  if (!initBuzzer()) {
    Serial.println("Failed to initialize buzzer timer");
  }

  // Created before any HTTP route exists, so no handler can run against a null queue.
  commandQueue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(ControlCommand));
  telemetryQueue = xQueueCreate(1, sizeof(TelemetrySnapshot));
  if (commandQueue == nullptr || telemetryQueue == nullptr) {
    Serial.println("Failed to create control queues");
  }

  loadSDConfig();
  startWiFi();
  startSD(); // reopened after WiFi - starting WiFi while SD is open breaks the SD card

  ArduinoOTA.setHostname("Discreet");
  ArduinoOTA.setPassword("Discreet");
  // otaInProgress is checked every cycle by controlStep(), forcing the heater off for the whole OTA session.
  ArduinoOTA.onStart([]() {
    otaInProgress = true;
  });
  ArduinoOTA.onEnd([]() {
    otaInProgress = false;
  });
  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
  });
  ArduinoOTA.begin();

  setupServerRoutes();

  server.begin();
  Serial.println("HTTP server started.");

  pinMode(syncPin, INPUT);
  DimmableLight::setSyncPin(syncPin);
  DimmableLight::begin();

  myPID.SetSampleTime(250);
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);

  if (MDNS.begin("discreet")) {
    Serial.println("mDNS started - access at http://discreet.local");
  }

  delay(2000);

  // Started last, after everything controlStep() touches is initialized.
  BaseType_t controlTaskCreated = xTaskCreatePinnedToCore(
    controlTaskEntry, "ControlTask", CONTROL_TASK_STACK_SIZE, nullptr,
    CONTROL_TASK_PRIORITY, &controlTaskHandle, CONTROL_TASK_CORE);
  if (controlTaskCreated != pdPASS) {
    Serial.println("Failed to create control task - restarting");
    delay(100);
    ESP.restart();
  }
}

// Ends the current shot: resets tracking flags, zeroes pump demand, promotes pending settings.
void endShot() {
  acDetected = false;
  shotStarted = false;
  pumpPowerSetPreinf = false;
  pumpPowerSetExtraction = false;
  pumpDemand = 0;
  acOffPending = false;
  acOffSamples = 0;
  currentShotState = ShotState::COMPLETE;
  activeSettings = pendingSettings;
  if (!steamRequested) setpoint = activeSettings.setpoint;
}

// The single owner of all control decisions and both actuator outputs.
void controlStep(unsigned long now) {

  ControlCommand cmd;
  while (commandQueue != nullptr && xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
    applyControlCommand(cmd);
  }

  GetPressure();
  runPID();

  bool faulted = (currentFault != FaultCode::NONE);

  // AC detection, gated on no active fault to avoid re-detect churn while a fault is latched.
  if (digitalRead(syncPin) == LOW && !acDetected &&
      currentMode != OperatingMode::TEMP_ONLY && !faulted) {
    acDetectedTime = now;
    acDetected = true;
  }

  if (acDetected) {

    // Shot start: latch this shot's settings so mid-shot edits can't alter it.
    if (!shotStarted) {
      shotStarted = true;
      shotSettings = activeSettings;
      shotSettings.preinftime = (shotSettings.bloomtime > 0 && shotSettings.preinftime < 5) ? 8 : shotSettings.preinftime;
      if (!steamRequested) setpoint = shotSettings.setpoint;
    }

    elapsedTime = now - acDetectedTime;
    actime = elapsedTime / 1000;

    // --- PRE-INFUSION ---
    if (shotSettings.preinftime > 0 && actime < shotSettings.preinftime) {
      currentShotState = ShotState::PREINFUSION;
      if (currentPressure <= PrePressureSetpoint - 1) {
        pumppower = 255;
        pumpDemand = pumppower;
        pumpPowerSetPreinf = false;
      }
      else if (!pumpPowerSetPreinf) {
         PressureTarget = PrePressureSetpoint;
         pumppower = basePumpPowerForSetpoint(PressureTarget);
         pumpDemand = pumppower;
         pumpPowerSetPreinf = true;
         updatePumpRamp();
      }
      else updatePumpRamp();
    }

    // --- BLOOM ---
    else if (shotSettings.bloomtime > 0 && actime < shotSettings.preinftime + shotSettings.bloomtime) {
      currentShotState = ShotState::BLOOM;
      pumppower = 0;
      pumpDemand = pumppower;
    }

    // --- EXTRACTION ---
    else {
      currentShotState = ShotState::EXTRACTION;
      if (currentPressure < shotSettings.pressuresetpoint - 2) {
        pumppower = 255;
        pumpDemand = pumppower;
        pumpPowerSetExtraction = false;
      }
      else if (!pumpPowerSetExtraction) {
         PressureTarget = shotSettings.pressuresetpoint;
         pumppower = basePumpPowerForSetpoint(PressureTarget);
         pumpDemand = pumppower;
         pumpPowerSetExtraction = true;
         updatePumpRamp();
      }
      else updatePumpRamp();
    }

    // Shot-end debounce: require both elapsed time and a minimum number of observed HIGH samples.
    if (digitalRead(syncPin) == HIGH) {
      if (!acOffPending) {
        acOffPending = true;
        acOffSince = now;
        acOffSamples = 0;
      }
      acOffSamples++;
    } else {
      acOffPending = false;
      acOffSamples = 0;
    }

    if (acOffPending && acOffSamples >= AC_OFF_MIN_SAMPLES &&
        now - acOffSince >= AC_OFF_DEBOUNCE_MS) {
      endShot();
    }
  } else {
    currentShotState = ShotState::IDLE;
  }

  // FAULT overrides the phase label from any state, and aborts an in-progress shot.
  if (faulted) {
    if (acDetected) endShot();
    currentShotState = ShotState::FAULT;
  }

  // --- Output priority resolution: fault/safety off, mode restrictions, state-machine demand ---
  double heaterOut = heaterDemand;
  int pumpOut = pumpDemand;

  // 1. Fault - highest priority, overrides everything above.
  if (faulted) {
    heaterOut = 0;
    pumpOut = 0;
  }

  // otaInProgress forces the heater off too, but deliberately leaves the pump alone.
  if (otaInProgress) {
    heaterOut = 0;
  }

  // 2. Mode restriction - temp-only must never enable the pump, including mid-shot.
  if (currentMode == OperatingMode::TEMP_ONLY) {
    pumpOut = 0;
    if (acDetected) endShot();
  }

  // 3. Whatever remains is the resolved demand. Clamped defensively before the actuator write.
  pumpOut = constrain(pumpOut, 0, 255);
  heaterOut = constrain(heaterOut, 0, 255);

  // Time-proportional SSR authorization. Window state is local - nothing outside this function
  // should touch it. The window rolls on its own schedule regardless of demand; only the
  // authorization gate reacts to heaterOut, so a routine zero-output cycle can't starve the heater.
  static unsigned long ssrWindowStartMs = 0;
  static unsigned long ssrWindowOnTimeMs = 0;
  if (now - ssrWindowStartMs >= SSR_WINDOW_MS) {
    ssrWindowStartMs = now;
    ssrWindowOnTimeMs = ((unsigned long)heaterOut * SSR_WINDOW_MS) / 255;
  }
  // heaterOut > 0, not <= 0 inverted - NaN must fail safe into "no heat."
  bool desiredHeaterOn = (heaterOut > 0) && (now - ssrWindowStartMs) < ssrWindowOnTimeMs;

  // Refreshed every cycle, after all safety checks above. "Off" is `now`, never a fixed sentinel
  // (wraparound-unsafe once millis() passes 2^31).
  resolvedPumpPower = pumpOut;
  ssrAuthorizedUntilMs = desiredHeaterOn ? (now + SSR_AUTH_TIMEOUT_MS) : now;
  light.setBrightness(pumpOut);

  if (telemetryQueue != nullptr) {
    TelemetrySnapshot snap = buildTelemetrySnapshot(now);
    xQueueOverwrite(telemetryQueue, &snap);
  }

  // Called last, after the actuator writes - it can block on the buzzer mutex, which must never
  // delay a fault-driven actuator write.
  steam();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  // controlTask diagnostics, reported here (not inside the task) to keep Serial off the critical path.
  static unsigned long lastReportMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastReportMs >= 5000) {
    lastReportMs = nowMs;
    Serial.printf(
      "controlTask: worst cycle %lu us, deadline misses %lu, stack headroom %u bytes\n",
      controlTaskWorstCycleUs, controlTaskDeadlineMisses,
      (unsigned)uxTaskGetStackHighWaterMark(controlTaskHandle));
    controlTaskWorstCycleUs = 0;

    // Deadman heartbeat: flags if the esp_timer service task itself stops running.
    if (nowMs - ssrDeadmanLastRunMs > 10 * SSR_ENFORCE_INTERVAL_MS) {
      Serial.printf(
        "WARNING: SSR deadman has not run in %lu ms (expected every %lu ms)\n",
        nowMs - ssrDeadmanLastRunMs, SSR_ENFORCE_INTERVAL_MS);
    }
  }
}
