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

// Forward declarations for functions called before their point of
// definition later in this file. Arduino's automatic prototype generation
// (ctags-based) does not reliably handle every C++ construct used below
// (scoped enums, reference parameters, default member initializers), so
// these are declared explicitly rather than relied on implicitly.
int basePumpPowerForSetpoint(double Pumpsetpoint);
String getContentType(String filename);

// Wi-Fi Variables
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
#define syncPin        19   //19  // Zero-cross sync input       new 19
#define thyristorPin   18   //18  // Thyristor (dimmer PWM output)

// Thermocouple SPI (HSPI)
#define thermoCS  22 //22 // old 18
#define thermoCLK 23 //23  // old 19
#define thermoDO  21 // 21 // old 21

//Pressure sensor pin
#define pressurepin 34

// Buzzer pin definition
#define BUZZER_PIN 4  // Change this to the actual GPIO you're using

// Initialize Thermo COMMENTED OUT UNTIL DEFINED
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// PID Setup
double setpoint, input, output;
double Kp = 48.0, Ki = 8, Kd = 50.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//Pressure Variables
int PrePressureSetpoint = 3; // Fixed pre-infusion pressure target; not currently user-editable.
int PressureTarget = 9;      // Overwritten before use once a shot is in progress; matches the old default pressuresetpoint value.
int pumppower = 0;
int maxPressure = 12;
double currentPressure = 0;

// Actuator demand (Phase 3): what the temperature controller and the shot
// state machine each *want*, computed by runPID()/updatePumpRamp() and the
// shot-phase logic in controlStep(). Neither is an actuator write by
// itself - controlStep() is the only place that reads these, resolves them
// against fault/mode priority, and performs the actual digitalWrite(SSR_PIN)
// / light.setBrightness() calls, once per cycle.
bool heaterDemand = false;
int pumpDemand = 0;

// The actually-resolved output of the priority chain (Phase 3 review):
// what got written to the actuators this cycle, after fault/mode
// overrides, as opposed to heaterDemand/pumpDemand above which are what
// was *asked for* before those overrides. Telemetry publishes these, not
// the raw demand, so a bench operator (or the UI) can see a fault or a
// mode restriction actually took effect rather than just trusting it did.
bool resolvedHeaterOn = false;
int resolvedPumpPower = 0;

// Timeing Intervals
const unsigned long PRESS_INTERVAL = 50;  // X ms
const unsigned long PID_INTERVAL = 250;  // X ms

// Pump-power ramp cadence used by updatePumpRamp(): replaces a "every 4th
// qualifying call" counter with an explicit elapsed-time interval (4 x
// PRESS_INTERVAL, matching the original cadence) so the ramp rate no
// longer depends on how often updatePumpRamp() happens to be entered.
const unsigned long PUMP_ADJUST_INTERVAL = 200;  // X ms

// AC-off debounce: how long syncPin must read continuously HIGH (off) before a
// shot is considered ended. Replaces a loop-iteration counter (previously 100
// iterations) so shot-end detection no longer depends on loop() speed.
//
// Elapsed wall-clock time alone is not sufficient here: millis() keeps
// advancing even while loop() is stalled (e.g. inside the SD/OTA paths
// documented as blocking in docs/PHASE1_NOTES.md), so a stall could let a
// single post-stall sample satisfy a pure time check. AC_OFF_MIN_SAMPLES
// additionally requires that many real digitalRead() samples were taken
// during the debounce window, so a stall bookended by only one or two
// samples cannot end a shot on its own.
// TODO(bench): both constants are judgement calls, not measurements against
// real hardware; confirm on the bench before relying on them (see
// docs/PHASE1_NOTES.md).
const unsigned long AC_OFF_DEBOUNCE_MS = 300;
const int AC_OFF_MIN_SAMPLES = 20;

// Timer Variables
unsigned long lastPIDTime = 0;
unsigned long DimlastUpdate = 0;
unsigned long lastPumpAdjust = 0;
unsigned long LastPressCall = 0;
unsigned long lastPrintTime = 0;
unsigned long acDetectedTime = 0;
unsigned long elapsedTime = 0; // Shot time in milliseconds
int actime = 0;  // Shot time in seconds

//Steam Veriables
String brewTemp;
bool steaming = false;       // temperature-threshold hysteresis flag (steam() sets it once input reaches steamSetpoint-5); drives the steam-ready beep, not the setpoint mirror.
bool steamRequested = false; // set by the "steam"/"stopsteam" /adjust commands (Phase 2); the setpoint-mirror guard - see acceptBrewSetpointEdit().
double steamSetpoint;

//Other Variables
int offset = 9; // Due to probe location. If you ask for 100 you will get 91, tune this variable.

bool acDetected = false;
bool PIDonly = false;
bool shotStarted = false;
bool pumpPowerSetPreinf = false;
bool pumpPowerSetExtraction = false;

// AC-off debounce state. acOffPending is true once syncPin has been observed
// HIGH (off) at least once since it was last LOW (on); acOffSince is the
// millis() timestamp of that first HIGH sample; acOffSamples counts how many
// HIGH samples have actually been taken since then (see AC_OFF_MIN_SAMPLES).
// Replaces the old loop-iteration offcount.
bool acOffPending = false;
unsigned long acOffSince = 0;
int acOffSamples = 0;

// --- Explicit state and data models (Phase 2) -------------------------------
// See docs/PHASE2_NOTES.md for design rationale and scope decisions. These
// are value-only types; no FreeRTOS task, queue, or actuator-ownership
// change happens in this phase - everything below still runs synchronously
// inside the single Arduino loop().

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

// Latched, per-shot control settings. REWRITE_PLAN.md's "Settings lifecycle"
// specifically calls out brew temperature, pressure [target] and phase
// durations as unable to alter a shot already in progress - those four
// fields live here. Kp/Ki/Kd, offset, steamSetpoint and PIDonly are not
// called out there and stay as plain, immediately-applied globals,
// unchanged from Phase 1.
struct ControlSettings {
  double setpoint = 0;         // brew target temperature; same offset-adjusted internal representation the old `setpoint` global used
  int preinftime = 8;          // seconds, 0..20
  int bloomtime = 0;           // seconds, 0..20
  int pressuresetpoint = 9;    // bar, 3..13
  unsigned long revision = 0;  // bumped on every accepted edit
};

ControlSettings activeSettings;   // in effect whenever no shot is in progress; source for the next shot's latch
ControlSettings shotSettings;     // latched from activeSettings at shot start; read-only for the duration of the shot
ControlSettings pendingSettings;  // latest edit; promoted to activeSettings when the shot returns to IDLE

// Value-only description of a validated /adjust or /saveConfig request.
// Routed through commandQueue (Phase 4, see setup()/handleAdjust()/
// "/saveConfig" below): the HTTP handlers only ever construct one of these
// and xQueueSend() it; applyControlCommand() - the "control owner" - is the
// only thing that reads, validates/constrains and applies one, drained from
// the queue once per controlStep() cycle. Still the same single task today
// (no second task exists until Phase 5), but the queue is a real FreeRTOS
// queue so the mechanism is already exactly what Phase 5 needs.
struct ControlCommand {
  enum class Type : uint8_t {
    NONE,
    SET_BREW_SETPOINT,           // relative delta (from /adjust), matches its existing "val" semantics
    SET_BREW_SETPOINT_ABSOLUTE,  // absolute offset-adjusted target (from /saveConfig, which posts a full value, not a delta)
    SET_PREINFTIME,
    SET_BLOOMTIME,
    SET_PRESSURESETPOINT,
    START_STEAM,
    STOP_STEAM,
    SET_MODE,                    // absolute OperatingMode (from /saveConfig)
  } type = Type::NONE;
  int delta = 0;              // relative adjustment, used by the non-ABSOLUTE SET_* types
  double value = 0;            // absolute value, used by SET_BREW_SETPOINT_ABSOLUTE
  OperatingMode mode = OperatingMode::NORMAL;  // used by SET_MODE
};

// Bounded command queue (Phase 4): the only channel by which an HTTP
// handler may influence control state. A depth of 8 is generous headroom
// over realistic usage (one /adjust click at a time, drained every
// controlStep() cycle, which runs far faster than a human can click) -
// see docs/PHASE4_NOTES.md for what happens if it ever actually fills.
// Created in setup(); nullptr until then, so every send/receive site
// checks for that rather than assuming creation always succeeds.
QueueHandle_t commandQueue = nullptr;
const int COMMAND_QUEUE_DEPTH = 8;

// Complete, copyable snapshot of everything the service side might want to
// show or log. Published through telemetryQueue (Phase 4) with
// xQueueOverwrite() at the end of every controlStep() cycle; read with
// xQueuePeek() so a reader never waits and never consumes the only copy.
struct TelemetrySnapshot {
  OperatingMode mode;
  ShotState shotState;
  FaultCode fault;
  double temperature;  // offset-corrected, deg C
  double pressure;      // bar
  int pumpPower;         // requested/ramped level, before fault/mode resolution (Phase 0-2 meaning, unchanged)
  bool heaterOn;          // actually written to SSR_PIN this cycle, after resolution (Phase 3)
  int resolvedPumpPower;  // actually written to the dimmer this cycle, after resolution (Phase 3)
  unsigned long elapsedShotTimeMs;
  unsigned long activeSettingsRevision;
  unsigned long shotSettingsRevision;
  unsigned long pendingSettingsRevision;
  unsigned long timestampMs;

  // The pendingSettings display values (Phase 4): added so handleGetValues()
  // can read everything it needs from this snapshot instead of reaching
  // into pendingSettings/setpoint/steamRequested directly - those are the
  // exact "shared mutable control globals" this phase's checklist item
  // asks to confirm are gone from the web-handler side. displaySetpoint is
  // already resolved for steamRequested (see buildTelemetrySnapshot()),
  // matching what "setpoint" in /getValues has always shown.
  double displaySetpoint;
  int pendingPreinftime;
  int pendingBloomtime;
  int pendingPressuresetpoint;
};

// Length-1: xQueueOverwrite() always succeeds and always replaces whatever
// snapshot (if any) was sitting unread, so the publisher never blocks and a
// reader always gets the latest complete snapshot, never a partial one.
QueueHandle_t telemetryQueue = nullptr;

TelemetrySnapshot buildTelemetrySnapshot() {
  TelemetrySnapshot snap;
  snap.mode = currentMode;
  snap.shotState = currentShotState;
  snap.fault = currentFault;
  snap.temperature = input - offset;
  snap.pressure = currentPressure;
  snap.pumpPower = pumppower;
  snap.heaterOn = resolvedHeaterOn;
  snap.resolvedPumpPower = resolvedPumpPower;
  snap.elapsedShotTimeMs = acDetected ? elapsedTime : 0;
  snap.activeSettingsRevision = activeSettings.revision;
  snap.shotSettingsRevision = shotSettings.revision;
  snap.pendingSettingsRevision = pendingSettings.revision;
  snap.timestampMs = millis();
  // displaySetpoint: while steamRequested, the live PID target is
  // steamSetpoint, not the (unrelated) brew target in pendingSettings -
  // matches /temp and the chart during steam, same resolution
  // handleGetValues() has always applied (moved here in Phase 4).
  snap.displaySetpoint = steamRequested ? setpoint : pendingSettings.setpoint;
  snap.pendingPreinftime = pendingSettings.preinftime;
  snap.pendingBloomtime = pendingSettings.bloomtime;
  snap.pendingPressuresetpoint = pendingSettings.pressuresetpoint;
  return snap;
}

// Accepts an edit to one of the three latched int fields. While idle it
// takes effect immediately (there is no in-progress shot to protect);
// mid-shot it only updates pendingSettings and is promoted to
// activeSettings when the shot returns to IDLE (see loop()'s shot-end
// handling below).
//
// "Idle" here is `!acDetected`, not `currentShotState == ShotState::IDLE`.
// currentShotState is a telemetry label, not a control input: the FAULT
// override at the end of loop() sets it to FAULT even while genuinely idle
// (freezing every edit for as long as a fault is latched, with no shot
// required to clear it), and it holds COMPLETE for one iteration after
// shot-end into the next iteration's server.handleClient() (line ~1006),
// before the `else` branch reassigns IDLE - a request serviced in that
// window would otherwise be treated as mid-shot even though the shot has
// already ended and its settings already promoted. acDetected does not
// have either problem: it is cleared synchronously at shot-end (before
// promotion runs) and is never touched by fault handling.
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

// setpoint feeds myPID via a bound pointer (PID_v1 takes one at
// construction), so it can't be replaced by a struct field directly; it
// stays live-updated at the points where the effective value can change.
// Guarded by steamRequested, not steaming: steaming is a temperature-
// threshold hysteresis flag (steam() sets it once input actually reaches
// steamSetpoint-5, for the beep) and was never a "steam is wanted" flag.
// Guarding on it here let a steam request made during the boiler's warm-up
// (steaming still false) get silently overwritten back to the brew target
// by the very next idle edit, shot start or shot end.
void acceptBrewSetpointEdit(double newValue) {
  pendingSettings.setpoint = newValue;
  pendingSettings.revision++;
  if (!acDetected) {
    activeSettings.setpoint = newValue;
    activeSettings.revision = pendingSettings.revision;
    if (!steamRequested) setpoint = newValue;
  }
}

// Validates and applies one ControlCommand. Mirrors the constrain() ranges
// the original handleAdjust() enforced. Returns false for an unrecognized
// command, matching the old handler's silent-no-op-then-200 behaviour for
// unknown "var" values.
//
// `idle` is `!acDetected` - see the comment on acceptPreinftimeEdit() for
// why currentShotState is not used here.
//
// The old handler's pumppower side-effect on preinftime/pressuresetpoint
// edits (a pre-existing quirk, not something this PR set out to change) is
// now gated on `idle` too: it used to write the live pumppower global
// unconditionally, including mid-shot, which bypassed the settings latch
// this PR exists to add - a mid-shot pressuresetpoint edit would leave
// PressureTarget frozen in shotSettings (correct) while still slamming
// pumppower to a new value immediately (not correct), producing a real,
// unrequested pressure transient during the "protected" shot. Preserved
// only for the idle case, where nothing is being protected.
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
      // From /saveConfig, which posts a complete target rather than a
      // delta. No constrain() here, matching /saveConfig's pre-existing
      // behaviour (unlike /adjust's SET_BREW_SETPOINT, this was never
      // range-checked - a pre-existing gap, not something this phase set
      // out to fix).
      acceptBrewSetpointEdit(cmd.value);
      return true;
    case ControlCommand::Type::SET_MODE:
      // Keeps the legacy PIDonly mirror in sync, same as loadSDConfig()/
      // the rest of "/saveConfig" already do - PIDonly is still what gets
      // persisted to config.json.
      PIDonly = (cmd.mode == OperatingMode::TEMP_ONLY);
      currentMode = cmd.mode;
      return true;
    default:
      return false;
  }
}

DimmableLight light(thyristorPin);

// --- Timer-driven buzzer sequencer -----------------------------------------
// An ESP one-shot timer advances the pattern independently of loop(), so a
// slow web, SD or OTA call cannot stretch an ON pulse. This uses the ESP timer
// service, not another application-owned FreeRTOS task.
volatile bool buzzerActive = false;
bool buzzerPinOn = false;
int buzzerBeepsRemaining = 0;
int buzzerOnMs = 0;
int buzzerOffMs = 0;
esp_timer_handle_t buzzerTimer = nullptr;
SemaphoreHandle_t buzzerMutex = nullptr;

void buzzerTimerCallback(void *arg) {
  uint64_t nextDelayUs = 0;

  xSemaphoreTake(buzzerMutex, portMAX_DELAY);
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
  // The original blocking function could not overlap itself. Keep that
  // single-pattern contract and avoid replacing a timer while its callback
  // may be in flight.
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

//Functions
void handleApplyTheme() {

    if (!server.hasArg("name")) {
        server.send(400, "text/plain", "No theme specified");
        return;
    }

    String themeName = server.arg("name");

    // Safety: block invalid names
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

    // Open source file
    File sourceFile = SD.open(sourcePath, FILE_READ);

    if (!sourceFile) {
        server.send(500, "text/plain", "Failed to open theme file");
        return;
    }

    // Remove old global.css
    if (SD.exists(targetPath)) {
        SD.remove(targetPath);
    }

    // Create new global.css
    File targetFile = SD.open(targetPath, FILE_WRITE);

    if (!targetFile) {
        sourceFile.close();
        server.send(500, "text/plain", "Failed to create global.css");
        return;
    }

    // Copy file
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

  // "Pause" and "Resume" intentionally removed here (Phase 2, see
  // docs/PHASE2_NOTES.md and docs/REWRITE_PLAN.md "Operating model" -
  // there is no PAUSED shot state and no ambiguous question about whether
  // shot time should freeze). An unrecognized var, including a leftover
  // "Pause"/"Resume" request from an old client, falls through to
  // ControlCommand::Type::NONE and is a no-op, matching the old handler's
  // behaviour for any other unknown var.
  ControlCommand cmd;
  if (var == "preinftime") cmd.type = ControlCommand::Type::SET_PREINFTIME;
  else if (var == "bloomtime") cmd.type = ControlCommand::Type::SET_BLOOMTIME;
  else if (var == "setpoint") cmd.type = ControlCommand::Type::SET_BREW_SETPOINT;
  else if (var == "steam") cmd.type = ControlCommand::Type::START_STEAM;
  else if (var == "stopsteam") cmd.type = ControlCommand::Type::STOP_STEAM;
  else if (var == "pressuresetpoint") cmd.type = ControlCommand::Type::SET_PRESSURESETPOINT;
  cmd.delta = val;

  // Phase 4: this handler no longer calls applyControlCommand() itself -
  // it only constructs a command and hands it to the control owner via the
  // queue. A NONE command (unrecognized var) is never enqueued at all,
  // matching the old handler's no-op-then-200 behaviour without wasting a
  // queue slot on it. xQueueSend() with 0 ticks-to-wait never blocks the
  // web server; if the queue is ever actually full (see
  // docs/PHASE4_NOTES.md for how unlikely that is given the drain rate),
  // the client gets an explicit 503 rather than a silently dropped edit.
  if (cmd.type == ControlCommand::Type::NONE) {
    server.send(200, "text/plain", "OK");
    return;
  }
  if (commandQueue != nullptr && xQueueSend(commandQueue, &cmd, 0) == pdTRUE) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(503, "text/plain", "Busy, try again");
  }
}

void handleTemp() {
  // Create JSON document
  StaticJsonDocument<128> doc;
  doc["temp"] = input - offset;
  doc["setpoint"] = setpoint - offset;
  doc["pressure"] = currentPressure;

  // Use a buffer to generate the JSON, then send it
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetValues() {

  // preinftime/bloomtime/pressuresetpoint/setpoint are read from
  // snap.pending*/displaySetpoint (pendingSettings' values, as of the
  // snapshot), not activeSettings or shotSettings: the front end's
  // updateLabels() calls this right after posting an /adjust edit and
  // expects to see that edit reflected immediately, whether idle or
  // mid-shot. While idle, pendingSettings == activeSettings, so this is
  // unchanged from Phase 1 in the common case. displaySetpoint is already
  // resolved for steamRequested (see buildTelemetrySnapshot()).
  //
  // Read via xQueuePeek() (Phase 4), not by calling buildTelemetrySnapshot()
  // directly: this handler runs in server.handleClient(), which is called
  // before controlStep() in loop() every iteration, so it was already
  // reading state "as of the end of the previous cycle" even before this
  // phase - the queue just makes that explicit and, from Phase 5 on, safe
  // across an actual task boundary. Falls back to building one directly
  // only if the queue isn't ready yet (e.g. a request arriving before
  // controlStep() has run even once, or queue creation failed in setup()).
  TelemetrySnapshot snap;
  if (telemetryQueue == nullptr || xQueuePeek(telemetryQueue, &snap, 0) != pdTRUE) {
    snap = buildTelemetrySnapshot();
  }

  StaticJsonDocument<384> doc;
  doc["setpoint"] = int(snap.displaySetpoint - offset);
  doc["preinftime"] = snap.pendingPreinftime;
  doc["bloomtime"] = snap.pendingBloomtime;
  doc["pressure"] = snap.pressure;
  doc["pumppower"] = snap.pumpPower;
  doc["pressuresetpoint"] = snap.pendingPressuresetpoint;
  doc["actime"] = actime;
  doc["temp"] = snap.temperature;
  doc["Kp"] = Kp;
  doc["Ki"] = Ki;
  doc["Kd"] = Kd;
  doc["brewTemp"] = brewTemp;
  doc["steamSetpoint"] = steamSetpoint;

  // New in Phase 2: explicit mode/state/fault and settings-revision fields.
  // Additive - all fields above are unchanged from Phase 1.
  doc["mode"] = toString(snap.mode);
  doc["shotState"] = toString(snap.shotState);
  doc["fault"] = toString(snap.fault);
  doc["activeSettingsRevision"] = snap.activeSettingsRevision;
  doc["shotSettingsRevision"] = snap.shotSettingsRevision;
  doc["pendingSettingsRevision"] = snap.pendingSettingsRevision;
  doc["snapshotTimestampMs"] = snap.timestampMs;

  // New in Phase 3: what was actually written to each actuator last cycle,
  // after fault/mode resolution - distinct from "pumppower" above, which
  // is the requested/ramped level before that resolution. Lets a bench
  // operator confirm a fault or a mode restriction actually cut an output,
  // rather than only seeing the (possibly overridden) demand.
  doc["heaterOn"] = snap.heaterOn;
  doc["resolvedPumpPower"] = snap.resolvedPumpPower;

  // Use a buffer to generate the JSON, then send it
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

// Computes heaterDemand - it does not write SSR_PIN itself. controlStep()
// is the sole writer of actuator outputs (Phase 3); this only decides what
// the temperature controller *wants*, which controlStep() then resolves
// against fault/mode priority before writing the pin once per cycle.
//
// Clearing a fault requires FAULT_CLEAR_STABLE_READINGS consecutive valid
// samples, not just one - found necessary in Phase 3 review: a temperature
// fault now also forces the pump off (see controlStep()), so an
// intermittent thermocouple (a real, common failure mode - a loose crimp
// or EMI from the pump's own triac) that flaps between valid and NaN reads
// would otherwise couple straight into the pump chattering on and off
// every PID_INTERVAL. Setting the fault is NOT debounced - any single bad
// reading forces safe outputs immediately, which is the conservative
// direction to be fast on.
const int FAULT_CLEAR_STABLE_READINGS = 3;
int faultClearStreak = 0;

void runPID() {

  unsigned long PIDnow = millis();

  if ((PIDnow - lastPIDTime >= PID_INTERVAL)) {
    lastPIDTime = PIDnow;

    input = thermocouple.readCelsius();

    if (isnan(input) || input < 0 || input > 160) {
      currentFault = FaultCode::INVALID_TEMPERATURE;
      faultClearStreak = 0;
      heaterDemand = false;
      return;
    }
    faultClearStreak++;
    if (faultClearStreak >= FAULT_CLEAR_STABLE_READINGS) {
      currentFault = FaultCode::NONE;
    }

    myPID.Compute();

    heaterDemand = (output >= 127);

  }
}

void GetPressure(){

  if (!acDetected) { //Dont read pressure when not pulling a shot. Pressure sensor is before the solenoid so will show pressure build in boiler.
    currentPressure = 0;
    return;
  }

  if (elapsedTime < 500) { //Dont read pressure for the first .5 seconds to let pressure build up escape
    currentPressure = 0;
    return;
  }

  unsigned long Pnow = millis();

  if (Pnow - LastPressCall >= PRESS_INTERVAL) {
    LastPressCall = Pnow; 
    int raw = analogRead(pressurepin);
    currentPressure = (raw * (maxPressure / 4095.0));
    currentPressure = round(currentPressure * 100) / 100.0;  // Limit to 2 decimals
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

// Computes pumpDemand - it does not write the dimmer itself. controlStep()
// is the sole writer of actuator outputs (Phase 3); this only decides what
// the pressure-regulation ramp *wants*, which controlStep() then resolves
// against fault/mode priority before writing the dimmer once per cycle.
// Renamed from SetPump() (Phase 0-2) to reflect that it no longer sets
// anything actuator-facing.
void updatePumpRamp() {

  unsigned long now = millis();

  // Only update after boost every xxx ms
  if (now - DimlastUpdate > PRESS_INTERVAL) {
    DimlastUpdate = now;

    // Slow - adjust pump power every PUMP_ADJUST_INTERVAL of elapsed time,
    // not every Nth call: the previous "every 4th qualifying call" counter
    // only meant ~200ms when this function was entered at least every
    // PRESS_INTERVAL, which isn't guaranteed under loop() load.
    if (now - lastPumpAdjust >= PUMP_ADJUST_INTERVAL) {
      lastPumpAdjust = now;
      if (currentPressure < PressureTarget - 0.2) { pumppower = constrain(pumppower + 1, 120, 255); }
      else if (currentPressure > PressureTarget + 0.2) { pumppower = constrain(pumppower - 2, 120, 255); }
    }

    // Fast - cut pump if overpressure every 50ms
    if (currentPressure > PressureTarget + 0.3) { pumpDemand = 0; }
    else { pumpDemand = pumppower; }

  }

}

void setupServerRoutes() {

  server.on("/upload", HTTP_POST, []() {
    // This runs *after* handleUpload finishes
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
  
    // Remove old config
    SD.remove("/config.json");
  
    File file = SD.open("/config.json", FILE_WRITE);
  
    if (!file) {
      server.send(500, "text/plain", "Write failed");
      return;
    }
  
    serializeJson(doc, file);
  
    file.close();

    Kp = doc["Kp"];
    Ki = doc["Ki"];
    Kd = doc["Kd"];
    myPID.SetTunings(Kp, Ki, Kd); 

    offset = doc["offset"] | offset;
    // Routed through the command queue (Phase 4; the settings-lifecycle
    // accept path itself is unchanged from Phase 2): while idle this takes
    // effect immediately, matching the old behaviour; if a shot happens to
    // be in progress it becomes pending instead of altering that shot,
    // same as an /adjust "setpoint" edit. The fallback when the client
    // omits "setpoint" reads pendingSettings (the canonical brew target),
    // not the live `setpoint` global - the live value is whatever is
    // currently driving the PID, which is steamSetpoint while
    // steamRequested, and using it here would persist the steam target as
    // the new brew setpoint.
    ControlCommand setpointCmd;
    setpointCmd.type = ControlCommand::Type::SET_BREW_SETPOINT_ABSOLUTE;
    setpointCmd.value = (doc["setpoint"] | (pendingSettings.setpoint - offset)) + offset;
    if (commandQueue != nullptr) xQueueSend(commandQueue, &setpointCmd, 0);

    steamSetpoint = (doc["steamSetpoint"] | steamSetpoint) + offset;

    // Also routed through the queue (Phase 4) rather than writing PIDonly/
    // currentMode directly: this is safety-critical state (the actuator
    // output-priority resolution in controlStep() reads currentMode - see
    // docs/PHASE3_NOTES.md "Review round 1"), so it goes through the same
    // control-owner-validated path as everything else instead of being a
    // direct cross-context write.
    ControlCommand modeCmd;
    modeCmd.type = ControlCommand::Type::SET_MODE;
    modeCmd.mode = (doc["PIDonly"] | PIDonly) ? OperatingMode::TEMP_ONLY : OperatingMode::NORMAL;
    if (commandQueue != nullptr) xQueueSend(commandQueue, &modeCmd, 0);

    Serial.println("Config saved");
  
    server.send(200, "text/plain", "Saved");
  
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
  // Called only from setup(), before loop() begins, so blocking here to keep
  // the original boot-time beep timing is safe (nothing else needs to run
  // concurrently yet).
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

  // Read config
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

  // Both branches above leave setpoint/steamSetpoint as the raw
  // (un-offset) value; add offset once, uniformly. The old code added
  // offset to setpoint only in the if-branch, leaving setpoint and the
  // since-removed setpointBoot 'offset' apart in the else-branch (a
  // pre-existing bug, only reachable when config.json is missing/unreadable
  // and steam mode is used) - fixed as a direct consequence of retiring
  // setpointBoot in favour of activeSettings.setpoint below, since the two
  // could no longer be allowed to disagree.
  setpoint = setpoint + offset;
  steamSetpoint = steamSetpoint + offset;

  currentMode = PIDonly ? OperatingMode::TEMP_ONLY : OperatingMode::NORMAL;

  activeSettings.setpoint = setpoint;
  activeSettings.revision = 1;
  pendingSettings = activeSettings;
  shotSettings = activeSettings;

  //end SD and SPI
  SD.end();
  SPI.end();  // fully kill SPI
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
  // Called only from setup(), before loop() begins; see startSD() note above.
  waitForBuzzer();

}

void steam(){

  if (input > steamSetpoint - 5 && !steaming){
    // Called every loop() iteration; must not block control timing, so this
    // only queues the pattern. The ESP timer service advances it.
    queueBuzzer(3,100,100);
    steaming = true;
  }
  if (input < steamSetpoint - 20 && steaming){steaming = false;}
  
}

void setup() {

  Serial.begin(115200);
  delay(2000);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  if (!initBuzzer()) {
    Serial.println("Failed to initialize buzzer timer");
  }

  // Created before setupServerRoutes()/server.begin() below, so no HTTP
  // handler can ever run against a null queue (Phase 4). If creation
  // fails, commandQueue/telemetryQueue stay nullptr and every send/
  // receive/peek site already checks for that - see their own comments.
  commandQueue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(ControlCommand));
  telemetryQueue = xQueueCreate(1, sizeof(TelemetrySnapshot));
  if (commandQueue == nullptr || telemetryQueue == nullptr) {
    Serial.println("Failed to create control queues");
  }

  //load config.json
  loadSDConfig();
  //start WiFi
  startWiFi();

  // Start SD card again after Wifi (having SD running and starting wifi breaks the SD card)
  startSD();
 
  // OTA Setup
  ArduinoOTA.setHostname("Discreet"); // Set a unique hostname
  ArduinoOTA.setPassword("Discreet"); // Optional: Set a password for security
  ArduinoOTA.begin(); // Start OTA service

  // === Web Routes ===
  setupServerRoutes();

  // === Start Server ===
  server.begin();
  Serial.println("HTTP server started.");

    //Dimmer Setup
  pinMode(SSR_PIN, OUTPUT);
  pinMode(syncPin, INPUT);
  DimmableLight::setSyncPin(syncPin);
  DimmableLight::begin();

  // PID setup
  myPID.SetSampleTime(250); 
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
  
  if (MDNS.begin("discreet")) {
    Serial.println("mDNS started - access at http://discreet.local");
  }
  
  delay(2000); //delay before anything starts happening.
  //setpoint = 10; // OVERIDE FOR DEVELOPMENT
}

// Ends whatever shot is in progress: resets the shot-tracking flags, zeroes
// pump demand, promotes the latest pending settings, and reports COMPLETE
// for this iteration (the following iteration's `else` branch in
// controlStep() reports IDLE). Called both by the normal AC-off debounce
// firing and by a fault aborting a shot in progress - centralizing this
// avoids the two paths silently drifting apart, and was added specifically
// because leaving a fault to only mask actuator outputs (without ending
// the shot) caused two problems found in review: the low-pressure boost
// branch would see pressure collapse once the pump was actually off and
// re-latch pumpDemand to full power within a cycle or two, so the pump
// slammed to 255 with no ramp the moment the fault cleared; and acDetected
// staying true for the whole fault meant the settings-lifecycle idle gate
// (!acDetected - see docs/PHASE2_NOTES.md "Review round 1") never saw
// "idle", silently deferring every settings edit for as long as the fault
// persisted. If the paddle/switch is still engaged once a fault clears,
// AC detection picks it back up on the very next cycle and a fresh shot
// begins from pre-infusion - there is no attempt to resume a partial shot,
// matching this project's existing "no PAUSED state" position (see
// docs/REWRITE_PLAN.md "Operating model").
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

// The single boundary that owns all control decisions and both actuator
// outputs (Phase 3, docs/REWRITE_PLAN.md "Centralize control ownership").
// Everything above this point in the call chain (sensor sampling, the shot
// state machine, the PID and pump-ramp computations) only ever sets
// heaterDemand/pumpDemand or other control state; digitalWrite(SSR_PIN,...)
// and light.setBrightness(...) are called exactly once each, at the very
// end of this function, nowhere else. Still runs synchronously inside
// loop() - cross-boundary data now flows through commandQueue/
// telemetryQueue (Phase 4), but there is still only one task; Phase 5 is
// what actually moves this function into a dedicated FreeRTOS task.
//
// Takes `now` rather than calling millis() itself, matching the
// controlStep(now) signature docs/REWRITE_PLAN.md names, and the periodic-
// cycle pattern Phase 5's dedicated task will use (one now per cycle,
// sampled by the caller, rather than several slightly-drifting millis()
// reads taken at different points in the same logical cycle). GetPressure(),
// runPID() and updatePumpRamp() still sample millis() themselves for their
// own internal interval gates - only controlStep()'s own direct time
// arithmetic (the AC-detect timestamp and the AC-off debounce) uses `now`.
void controlStep(unsigned long now) {

  // Drain the command queue first, before anything else this cycle reads
  // control state: applies every currently-queued command, in order,
  // through applyControlCommand() - the sole "control owner" (Phase 4;
  // docs/REWRITE_PLAN.md "reject unsafe or invalid settings inside the
  // control owner"). No separate coalescing logic is needed to satisfy
  // "the latest complete pending revision wins": applyControlCommand()
  // already reads the current pendingSettings/activeSettings value at the
  // moment each command is applied (not at the moment it was submitted),
  // so a burst of relative edits to the same field simply accumulates
  // exactly as if applied one at a time by hand - see docs/PHASE4_NOTES.md.
  ControlCommand cmd;
  while (commandQueue != nullptr && xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
    applyControlCommand(cmd);
  }

  GetPressure();
  runPID();
  // steam() is called at the very end of this function, after the actuator
  // writes - see the comment down there for why.

  // Read once, right after runPID() (the only place currentFault is
  // written) - reused below for the AC-detect gate, the telemetry label
  // and the actuator override, so all three can never drift apart.
  bool faulted = (currentFault != FaultCode::NONE);

  // AC detection. Also gated on no active fault: without this, a fault
  // occurring while the paddle/switch stays engaged would re-detect AC and
  // start a fresh shot on literally every cycle, only to have the FAULT
  // handling below immediately call endShot() again that same cycle -
  // harmless (outputs stay correctly off throughout) but pointless churn.
  // Waiting for the fault to clear before allowing a new shot to start
  // makes the intent explicit instead of relying on immediate self-abort.
  if (digitalRead(syncPin) == LOW && !acDetected &&
      currentMode != OperatingMode::TEMP_ONLY && !faulted) {
    acDetectedTime = now;
    acDetected = true;
  }

  // pumpDemand is intentionally NOT reset to 0 here every cycle.
  // updatePumpRamp() (called below) only actually updates pumpDemand on
  // the iterations where its own PRESS_INTERVAL gate fires (~every 50ms);
  // on every iteration in between it must be left untouched so it keeps
  // commanding the pump's last ramped power, exactly like the pre-Phase-3
  // SetPump() did by simply not calling light.setBrightness() on those
  // iterations. Resetting it to 0 unconditionally here would command the
  // pump off on every one of those in-between iterations instead. Safety
  // still holds without this: pumpDemand starts at 0 at declaration, and
  // is explicitly zeroed in the shot-end block below - those are the only
  // two places it needs to reach 0 from.
  if (acDetected) {

    // Run once at shot start: entry actions for the IDLE -> PREINFUSION
    // transition. Latches this shot's settings so mid-shot edits (which
    // now land in pendingSettings - see acceptPreinftimeEdit() etc. above)
    // cannot alter the shot already in progress
    // (docs/REWRITE_PLAN.md "Settings lifecycle").
    if (!shotStarted) {
      shotStarted = true;
      shotSettings = activeSettings;
      shotSettings.preinftime = (shotSettings.bloomtime > 0 && shotSettings.preinftime < 5) ? 8 : shotSettings.preinftime;
      if (!steamRequested) setpoint = shotSettings.setpoint;
    }

    // Calculate shot time
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
         pumpPowerSetPreinf = true;  // prevents running again
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
         pumppower = basePumpPowerForSetpoint(PressureTarget); // runs once
         pumpDemand = pumppower;
         pumpPowerSetExtraction = true;  // prevents it running again
         updatePumpRamp();
      }
      else updatePumpRamp();
    }

    // AC turned off: require both elapsed wall-clock time AND a minimum
    // number of actually-observed HIGH samples before ending the shot.
    // Time alone is not safe here: millis() keeps advancing even while
    // loop() is stalled (SD/OTA paths are still blocking - see
    // docs/PHASE1_NOTES.md), so a stall could let a single post-stall
    // sample satisfy a pure time check and end a live shot. Requiring
    // AC_OFF_MIN_SAMPLES real samples means a stall bookended by only one
    // or two samples cannot trigger this on its own - loop() actually has
    // to keep running and keep reading syncPin as HIGH throughout.
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

  // FAULT has priority over the phase label above, in any state
  // (docs/REWRITE_PLAN.md "FAULT may be entered from any state"), and also
  // aborts a shot in progress rather than leaving it running underneath
  // the masked outputs - see the comment on endShot() for why.
  if (faulted) {
    if (acDetected) endShot();
    currentShotState = ShotState::FAULT;
  }

  // --- Output priority resolution and the only actuator write of the
  // cycle (docs/REWRITE_PLAN.md "Apply output priority: fault/safety off,
  // mode restrictions, state-machine demand, controller output") ---
  bool heaterOut = heaterDemand;
  int pumpOut = pumpDemand;

  // 1. Fault/safety off - highest priority, overrides everything above.
  // Previously (Phase 0-2) a temperature fault only forced the SSR off,
  // synchronously inside runPID(); the pump was never coupled to it, so a
  // fault occurring mid-shot left the pump running exactly as if nothing
  // were wrong. This is the fix: any fault now forces both actuators off,
  // every cycle for as long as it persists, regardless of what the shot
  // state machine or PID computed.
  if (faulted) {
    heaterOut = false;
    pumpOut = 0;
  }

  // 2. Mode restrictions - temp-only must never enable the pump. Already
  // true today as a side effect of AC detection being gated on mode (a
  // shot can never start in temp-only mode, so pumpDemand can never become
  // nonzero that way) - but that guarantee breaks if the mode is flipped
  // to temp-only while a shot is already in progress, since nothing
  // currently interrupts a running shot on a mode change. This is an
  // explicit, independent second guarantee - not just restating the first
  // one - and closes that gap. Reads currentMode (the Phase 2 enum), not
  // the raw PIDonly bool it mirrors: this is now safety-critical code, and
  // currentMode is the value Phase 2 designated as the source of truth -
  // found in review that the first version of this check read PIDonly
  // directly, which would have silently stopped enforcing this restriction
  // the moment anything set currentMode without also setting PIDonly.
  if (currentMode == OperatingMode::TEMP_ONLY) {
    pumpOut = 0;
  }

  // 3. Whatever remains is the state-machine demand, already resolved
  // through the pressure-regulation controller output above. Clamped
  // defensively - light.setBrightness() takes a uint8_t, so an
  // out-of-range int would silently wrap (e.g. -1 becomes 255, full
  // power) rather than fail loudly; nothing produces an out-of-range
  // value today, but this is the one choke point all pump output passes
  // through, which is exactly where that guarantee belongs. Write both
  // actuators exactly once, and record what was actually written for
  // telemetry.
  pumpOut = constrain(pumpOut, 0, 255);
  resolvedHeaterOn = heaterOut;
  resolvedPumpPower = pumpOut;
  digitalWrite(SSR_PIN, heaterOut ? HIGH : LOW);
  light.setBrightness(pumpOut);

  // Publish telemetry for this cycle (Phase 4). xQueueOverwrite() never
  // blocks and never fails - it always succeeds by definition on a
  // length-1 queue, replacing whatever was there. Placed after the
  // actuator writes so resolvedHeaterOn/resolvedPumpPower (read inside
  // buildTelemetrySnapshot()) reflect what was actually just written, not
  // the previous cycle's values.
  if (telemetryQueue != nullptr) {
    TelemetrySnapshot snap = buildTelemetrySnapshot();
    xQueueOverwrite(telemetryQueue, &snap);
  }

  // steam() is called here, after the actuator writes, not near the top of
  // this function with GetPressure()/runPID() as it was originally -
  // found in review that queueBuzzer() (which steam() can call) blocks on
  // a mutex shared with the buzzer's timer task (xSemaphoreTake(...,
  // portMAX_DELAY), see queueBuzzer() in the buzzer section), and having
  // any blocking call between a fault being detected (in runPID(), near
  // the top) and the safety-critical actuator write above was judged an
  // unacceptable risk, however small in practice - a stalled or starved
  // buzzer task must never be able to delay turning the heater off.
  // steam() doesn't feed anything the shot-phase logic above needs (it
  // only reads input/steamSetpoint and sets steaming, which nothing above
  // this point reads), so moving it here changes nothing else.
  steam();
}

void loop() {
  ArduinoOTA.handle(); // Handle OTA updates
  server.handleClient();
  controlStep(millis());
}
