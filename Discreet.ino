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

// Forward declarations for functions called before their point of
// definition later in this file. Arduino's automatic prototype generation
// (ctags-based) does not reliably handle every C++ construct used below
// (scoped enums, reference parameters, default member initializers), so
// these are declared explicitly rather than relied on implicitly.
int basePumpPowerForSetpoint(double Pumpsetpoint);
String getContentType(String filename);
// controlStep() has none of the constructs named above, but is declared
// hundreds of lines away from controlTaskEntry() (Phase 5), the first thing
// that calls it - ctags-based prototype generation didn't reach across that
// gap either (confirmed by a failed build without this line), so it's
// listed here too rather than left to chance.
void controlStep(unsigned long now);

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
// itself - controlStep() is the only place that reads these and resolves
// them against fault/mode priority. heaterDemand is the raw PID output
// (0-255, was a `bool` before Phase 6 - see runPID()); light.setBrightness()
// is still called directly from controlStep(), but SSR_PIN is not - see
// "Phase 6: time-proportional SSR output" below for why the heater path is
// different now.
double heaterDemand = 0;
int pumpDemand = 0;

// The actually-resolved output of the priority chain (Phase 3 review):
// what got written to the actuators this cycle, after fault/mode
// overrides, as opposed to heaterDemand/pumpDemand above which are what
// was *asked for* before those overrides. Telemetry publishes these, not
// the raw demand, so a bench operator (or the UI) can see a fault or a
// mode restriction actually took effect rather than just trusting it did.
// resolvedHeaterOn's meaning changed in Phase 6: with time-proportional SSR
// output, controlStep() no longer writes SSR_PIN itself (see below), so
// this is "authorized on for the current instant of the on/off window,
// after fault/mode resolution" - what controlStep() handed to the
// independent deadman this cycle, not necessarily what's physically on
// SSR_PIN at the moment a client reads this (a stale/expired authorization
// always reads back as off from the deadman's own enforcement, regardless
// of this value - see ssrDeadmanCallback()).
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

// SSR time-proportional window state (Phase 6) - controlTask-exclusive,
// unlike ssrAuthorizedOn/ssrAuthorizedUntilMs above, which cross into the
// deadman task. Only controlStep() ever reads or writes these.
unsigned long ssrWindowStartMs = 0;
unsigned long ssrWindowOnTimeMs = 0;

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
    // Added in Phase 5 review: Kp/Ki/Kd (+ myPID.SetTunings()), steamSetpoint
    // and offset were being written directly from /saveConfig (loopTask)
    // while read by the now-genuinely-concurrent controlTask - a real
    // torn-double/torn-PID-internal-state race that didn't exist before a
    // second task did. Routing them through the same control-owner queue as
    // everything else closes it the same way Phase 4 already closed it for
    // brew setpoint/mode. See docs/PHASE5_NOTES.md.
    SET_TUNINGS,                 // absolute Kp/Ki/Kd (from /saveConfig)
    SET_STEAM_SETPOINT,          // absolute offset-adjusted value, reuses `value` (from /saveConfig)
    SET_OFFSET,                  // absolute value, reuses `delta` (from /saveConfig)
  } type = Type::NONE;
  int delta = 0;              // relative adjustment (non-ABSOLUTE SET_* types) or absolute value (SET_OFFSET)
  double value = 0;            // absolute value, used by SET_BREW_SETPOINT_ABSOLUTE / SET_STEAM_SETPOINT
  OperatingMode mode = OperatingMode::NORMAL;  // used by SET_MODE
  double kp = 0, ki = 0, kd = 0;  // used by SET_TUNINGS
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
  bool heaterOn;          // authorized on for this instant, after resolution (Phase 3); the independent SSR deadman is what actually writes the pin from Phase 6 on - see resolvedHeaterOn's comment
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

// --- Phase 5: dedicated control task (docs/REWRITE_PLAN.md "Create the
// FreeRTOS control task") ---
//
// controlStep() itself is unchanged by this phase - it was already written
// to take `now` as a parameter and to read commandQueue/write telemetryQueue
// as its only cross-boundary contact points (Phase 4). This block only adds
// the task that calls it on its own schedule, independent of loop().
//
// Period: 10 ms, per REWRITE_PLAN.md. GetPressure()/runPID()/
// updatePumpRamp() already self-gate on PRESS_INTERVAL (50 ms) and
// PID_INTERVAL (250 ms) via their own millis() checks - calling
// controlStep() every 10 ms just makes those internal gates fire on their
// intended cadence instead of on whatever cadence loop() happened to reach
// controlStep() at; no restructuring of that gating was needed for this
// phase, matching PHASE4_NOTES.md's framing of this as "a relocation, not a
// redesign."
const TickType_t CONTROL_TASK_PERIOD_TICKS = pdMS_TO_TICKS(10);

// Priority/core, chosen deliberately rather than left at defaults:
//
// - Core 1 (APP_CPU): the same core Arduino's own loopTask runs on
//   (CONFIG_ARDUINO_RUNNING_CORE=1 for this target). Verified against this
//   project's actual esp32:esp32@2.0.17 sdkconfig (Phase 5 review), not
//   assumed: lwIP's tcpip_thread - which does the actual per-packet work
//   behind server.handleClient() - is pinned to core 0
//   (CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y), so core 1 is genuinely free of
//   the one task that would matter most. The Arduino event task
//   (arduino_events) does run on core 1 at a high priority
//   (ESP_TASKD_EVENT_PRIO, ~19), but it is idle except for infrequent
//   Wi-Fi/system events (connect/disconnect, got-IP), not a per-request cost.
// - Priority 5: strictly above loopTask's priority (hardcoded to 1 by the
//   Arduino core), so the scheduler preempts loop() the instant this task's
//   10 ms deadline arrives, regardless of what loop() is in the middle of.
//   Kept well below arduino_events/lwIP/Wi-Fi driver priorities (~18-23) so
//   this task cannot starve them.
// - Known, deliberately-not-worked-around gap: an OTA write's flash
//   erase/program calls disable interrupts and caches on both cores for its
//   duration (an ESP32 SPI-flash/XIP hardware constraint, not a scheduling
//   choice) - no core or priority choice prevents this task from being
//   stalled during an OTA upload. See docs/PHASE5_NOTES.md; this is a bench
//   item, not something core placement can fix.
// - Stack size 4096 bytes: controlStep() and everything it calls
//   (GetPressure(), runPID() - PID_v1's double-precision math -,
//   updatePumpRamp(), endShot(), buildTelemetrySnapshot(), a MAX6675 read)
//   do no dynamic allocation and no String work; 4096 is a conservative
//   starting budget, not a measurement - see uxTaskGetStackHighWaterMark()
//   below and the bench checklist in PHASE5_NOTES.md.
const uint32_t CONTROL_TASK_STACK_SIZE = 4096;
const UBaseType_t CONTROL_TASK_PRIORITY = 5;
const BaseType_t CONTROL_TASK_CORE = 1;

TaskHandle_t controlTaskHandle = nullptr;

// Diagnostics (REWRITE_PLAN.md "measure worst observed cycle time and
// deadline misses" / "measure stack high-water mark"): updated every cycle
// inside controlTaskEntry(), but *reported* from loop() (see the periodic
// block near the bottom of this file), not from inside the control task
// itself - found in review: Print::printf() falls back to malloc()+a second
// vsnprintf() for any formatted string 64 bytes or longer (this report line
// is longer than that), and Serial.write() can block once the UART's TX
// FIFO fills - neither belongs inside the highest-priority, safety-critical
// task, and both would have been silently excluded from the very cycle-time
// measurement they were reporting. controlTaskWorstCycleMaxUs is reset after
// each report so the number printed is a per-window worst case, not a
// once-ever spike that then reads as "still there" forever.
unsigned long controlTaskWorstCycleUs = 0;
unsigned long controlTaskDeadlineMisses = 0;
bool controlTaskWdtSubscribed = false;

// The task function itself: subscribes to the Task WDT explicitly (per
// REWRITE_PLAN.md, "do not rely on the watched idle tasks" - the default
// arduino-esp32 TWDT setup watches the idle tasks as a proxy for a runaway
// task starving them, which is a weaker guarantee than watching this task
// directly), then runs controlStep() on a fixed period using
// vTaskDelayUntil() so drift doesn't accumulate from this task's own
// execution time.
void controlTaskEntry(void* pvParameters) {
  esp_err_t wdtErr = esp_task_wdt_add(NULL);
  controlTaskWdtSubscribed = (wdtErr == ESP_OK);
  if (!controlTaskWdtSubscribed) {
    // Not recoverable in any useful way from inside this task - if the
    // explicit subscription itself failed (TWDT not initialized, or the
    // watched-task table is full), the whole point of this phase's
    // watchdog item is unmet. Logged once, loudly; controlStep() still runs
    // regardless, since not running it is strictly less safe than running
    // it unwatched. Logged once, not per-cycle: esp_task_wdt_reset() below
    // is skipped for the rest of this task's life once subscription has
    // failed (there being nothing to feed), so a per-cycle log here would
    // otherwise flood Serial at 100 Hz forever - found in review.
    Serial.printf("esp_task_wdt_add(controlTask) failed: %d\n", wdtErr);
  }

  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    unsigned long nowMs = millis();
    unsigned long cycleStartUs = micros();

    controlStep(nowMs);

    unsigned long cycleUs = micros() - cycleStartUs;
    if (cycleUs > controlTaskWorstCycleUs) controlTaskWorstCycleUs = cycleUs;

    // Feed the watchdog only after controlStep() has fully returned - i.e.
    // only after a complete cycle including the safety evaluation and the
    // actuator writes at the end of controlStep(), per REWRITE_PLAN.md. If
    // controlStep() ever hangs, this line is simply never reached and the
    // TWDT's own timeout - not this code - is what recovers the system.
    if (controlTaskWdtSubscribed) {
      esp_err_t resetErr = esp_task_wdt_reset();
      if (resetErr != ESP_OK) {
        // Only reachable if the TWDT unsubscribed this task out from under
        // it after a successful esp_task_wdt_add() (not expected - nothing
        // in this file calls esp_task_wdt_delete()); still checked and
        // logged per REWRITE_PLAN.md's "check and handle the return
        // values," but this task never resubscribes, so this can print at
        // most once per such an (unexpected) event, not every cycle.
        Serial.printf("esp_task_wdt_reset(controlTask) failed: %d\n", resetErr);
        controlTaskWdtSubscribed = false;
      }
    }

    // vTaskDelayUntil() itself is the deadline-miss signal: it returns
    // pdFALSE exactly when lastWakeTime was already due or past, i.e. this
    // cycle's work alone consumed a full period or more - no need to
    // separately re-derive that from the tick count (found in review: the
    // original code did, redundantly and only to tick resolution instead of
    // this call's own microsecond-driven timing).
    if (xTaskDelayUntil(&lastWakeTime, CONTROL_TASK_PERIOD_TICKS) == pdFALSE) {
      controlTaskDeadlineMisses++;
    }
  }
}

// Takes `now` rather than calling millis() itself - found in review (Phase
// 4 efficiency angle): this is called once per controlStep() cycle, which
// already samples `now` at the top of that function for exactly this kind
// of use; millis() itself is a ~250-400 cycle 64-bit-divide-backed call on
// this target; TelemetrySnapshot's only other caller (handleGetValues()'s
// direct-build fallback) doesn't have a `now` to reuse and passes millis()
// explicitly instead.
TelemetrySnapshot buildTelemetrySnapshot(unsigned long now) {
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
  snap.timestampMs = now;
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
      // delta. /saveConfig's pre-existing behaviour never range-checked
      // this value; now that it's routed through the control owner (the
      // one place REWRITE_PLAN.md's "reject unsafe or invalid settings"
      // item designates for that), clamp it the same as SET_BREW_SETPOINT
      // so an absolute edit can't push the heater target arbitrarily high.
      acceptBrewSetpointEdit(constrain(cmd.value, 10 + offset, 96 + offset));
      return true;
    case ControlCommand::Type::SET_MODE:
      // Keeps the legacy PIDonly mirror in sync, same as loadSDConfig()/
      // the rest of "/saveConfig" already do - PIDonly is still what gets
      // persisted to config.json.
      PIDonly = (cmd.mode == OperatingMode::TEMP_ONLY);
      currentMode = cmd.mode;
      return true;
    // The three cases below (Phase 5 review) move Kp/Ki/Kd/SetTunings(),
    // steamSetpoint and offset writes onto this exclusive-owner path -
    // matching every other control-relevant write in this switch - so
    // /saveConfig (loopTask) never mutates any of them directly. No new
    // validation is added beyond what each already had (none): the fix
    // here is the write's location, not new range-checking, which would be
    // a separate, undiscussed behaviour change.
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

// Single entry point for handing a command to commandQueue: null-checks the
// queue and reports whether the send actually succeeded, so every call site
// - handleAdjust() and /saveConfig alike - can react to a full/uncreated
// queue instead of silently discarding it. Found in review (Phase 4):
// handleAdjust() checked this itself and returned 503 on failure, but
// /saveConfig's two xQueueSend() calls did not, so a full or uncreated
// queue there let /saveConfig write the new setpoint/mode to config.json,
// drop both commands, and still answer "Saved" - the persisted config and
// the running machine would then disagree until the next reboot.
bool sendControlCommand(const ControlCommand& cmd) {
  return commandQueue != nullptr && xQueueSend(commandQueue, &cmd, 0) == pdTRUE;
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

// --- Phase 6: time-proportional SSR output + independent deadman
// (docs/REWRITE_PLAN.md "Consider time-proportional SSR output separately")
// ---
//
// Before this phase, controlStep() wrote SSR_PIN directly, synchronously,
// as part of its own cycle - a control-task wedge (see Phase 5's Task WDT
// item) left the pin at whatever it last was, for up to the WDT's timeout,
// which is a real window for a heater to stay energized. This phase moves
// the SSR write off the control task entirely: controlStep() only ever
// updates an "authorization" (an on/off decision, plus a short expiry
// timestamp); a separate, independent timer callback - not the control
// task, not even running in the same task at all - is the only thing that
// ever calls digitalWrite(SSR_PIN, ...), and it forces the pin off the
// instant the authorization goes stale, regardless of controlTask's health.
//
// SSR_WINDOW_MS: the time-proportional window. controlStep() converts the
// PID's 0-255 output into an on-time fraction of this window instead of the
// old fixed `output >= 127` threshold - REWRITE_PLAN.md's "convert the full
// PID output range to an on-time fraction." TODO(bench): 1s is a
// conservative starting point for a resistive heating element behind a
// solid-state relay (not a mechanical relay, so switching wear is not the
// constraint - thermal/electrical response of the actual heater is); tune
// against real hardware, same as every other *_INTERVAL constant in this
// file marked TODO(bench).
const unsigned long SSR_WINDOW_MS = 1000;

// SSR_AUTH_TIMEOUT_MS: how long an authorization stays valid without being
// refreshed. Must be comfortably larger than controlStep()'s own cycle time
// (nominally 10ms; Phase 5's diagnostics report the actual worst case once
// bench-measured - see PHASE5_NOTES.md) so ordinary cycle-to-cycle jitter
// never trips it, and comfortably smaller than any duration a energized
// heater staying on unsupervised should be tolerated for. 200ms is ~20x the
// nominal cycle time headroom; TODO(bench): revisit once Phase 5's real
// worst-case cycle time is measured.
const unsigned long SSR_AUTH_TIMEOUT_MS = 200;

// How often the deadman itself re-checks the authorization and drives the
// pin. Independent of, and much more frequent than, SSR_AUTH_TIMEOUT_MS -
// this is the deadman's own polling rate, not the staleness budget.
const unsigned long SSR_ENFORCE_INTERVAL_MS = 20;

// ssrAuthorizedOn/ssrAuthorizedUntilMs: written only by controlStep()
// (controlTask), read only by ssrDeadmanCallback() (a separate esp_timer
// task - see initSsrDeadman()). No queue/mutex: both are single-word types
// (bool, unsigned long - 32-bit on this target), atomic to read/write on
// this hardware, the same reasoning Phase 5 review applied to `offset`/
// `acDetected`-class globals. ssrAuthorizedOn is written before
// ssrAuthorizedUntilMs on every refresh (see controlStep()) so a reader can
// at worst see a stale on/off value paired with a fresh deadline for one
// SSR_ENFORCE_INTERVAL_MS tick - bounded, self-correcting next tick, and
// never weakens the actual safety property (an expired deadline always
// means off, regardless of ssrAuthorizedOn's value).
volatile bool ssrAuthorizedOn = false;
volatile unsigned long ssrAuthorizedUntilMs = 0;

esp_timer_handle_t ssrDeadmanTimer = nullptr;

// The deadman itself. Runs in the esp_timer service task (ESP_TIMER_TASK
// dispatch, matching this file's existing buzzerTimer pattern) - a real,
// separate FreeRTOS task at a high fixed priority (ESP_TASK_TIMER_PRIO),
// not the control task, so a wedged/starved/crashed control task cannot
// prevent this from running. True ISR dispatch (ESP_TIMER_ISR) was
// considered and deliberately not used: it adds real interrupt-context
// constraints (no blocking calls, careful use of ISR-safe APIs) for a
// narrower additional guarantee than it might first appear - an ESP32
// flash write/erase (e.g. during OTA) disables interrupts on both cores for
// its duration regardless of dispatch method (see docs/PHASE5_NOTES.md §2),
// so ISR dispatch would not protect against that specific, already-known,
// already-documented gap either. Task dispatch at high priority is the
// better cost/benefit here and is easier to reason about correctly without
// hardware to test interrupt-context code against.
//
// Wraparound-safe comparison: `(long)(ssrAuthorizedUntilMs - now) > 0` -
// this is the standard idiom for "is this unsigned-millis() deadline still
// in the future," distinct from this file's usual `now - lastX >= INTERVAL`
// idiom (which answers "has at least this much time elapsed" - a different
// question, also wraparound-safe, but the wrong shape for a deadline check).
void ssrDeadmanCallback(void* arg) {
  unsigned long now = millis();
  bool authorized = (long)(ssrAuthorizedUntilMs - now) > 0;
  digitalWrite(SSR_PIN, (authorized && ssrAuthorizedOn) ? HIGH : LOW);
}

bool initSsrDeadman() {
  digitalWrite(SSR_PIN, LOW);  // known-safe default before the deadman is even running
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
  if (sendControlCommand(cmd)) {
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
  // directly: this always returns the latest snapshot controlTask has
  // published, never a partial one, and never blocks this handler. Falls
  // back to building one directly only if the queue isn't ready yet (e.g. a
  // request arriving before controlStep() has run even once, or queue
  // creation failed in setup()).
  //
  // Before Phase 5, this handler ran in server.handleClient(), which loop()
  // called immediately before controlStep() every iteration, so an /adjust
  // request handled this same iteration was already guaranteed to be
  // drained and reflected here by construction. That ordering guarantee is
  // gone now that controlTask runs independently (found in review: the
  // comment here previously still asserted it) - a request landing within
  // the same ~10 ms window as the edit it's meant to reflect can read the
  // previous cycle's snapshot. Relative-delta commands (SET_PREINFTIME etc.)
  // self-correct on the next poll rather than compounding, since
  // applyControlCommand() always applies against the current base at drain
  // time (see docs/PHASE4_NOTES.md §2) - a UI showing one stale read is a
  // display lag, not a lost or duplicated edit.
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
  // snap.elapsedShotTimeMs, not the raw `actime` global - found in review
  // (Phase 5): `actime` is written only inside controlTask's `if
  // (acDetected)` branch and is never reset when a shot ends, so reading it
  // directly here was both a live cross-task read of a controlTask-owned
  // value (undefined relative to whatever snap/shotState this response
  // otherwise reports) and stale forever after the last shot's end -
  // snap.elapsedShotTimeMs is already the same-cycle-consistent, correctly
  // zeroed (buildTelemetrySnapshot(): `acDetected ? elapsedTime : 0`)
  // equivalent already sitting in the snapshot this handler already peeked.
  doc["actime"] = snap.elapsedShotTimeMs / 1000;
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
// resolves this against fault/mode priority and hands the result to the
// independent SSR deadman as a time-proportional authorization (Phase 6) -
// it does not write the pin directly any more either; see
// ssrDeadmanCallback().
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
      heaterDemand = 0;
      return;
    }
    faultClearStreak++;
    if (faultClearStreak >= FAULT_CLEAR_STABLE_READINGS) {
      currentFault = FaultCode::NONE;
    }

    myPID.Compute();

    // Phase 6: the full PID output range (0-255) is now the demand, not a
    // `output >= 127` binary threshold - controlStep() converts this into a
    // time-proportional on/off authorization instead of a plain digital
    // level. See "Phase 6: time-proportional SSR output" below.
    heaterDemand = output;

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

  // Only update after boost every xxx ms. `>=`, not `>` - found in review
  // (Phase 5): this function's caller now runs on an exact 10 ms grid
  // (controlTaskEntry()), so `now` advances in clean 10 ms steps relative to
  // DimlastUpdate; a strict `>` against PRESS_INTERVAL=50 is never true at
  // exactly +50 ms, so the gate only fired at +60 ms - silently stretching
  // this to a 60 ms cadence (and PUMP_ADJUST_INTERVAL's nested gate, only
  // reachable from here, to 240 ms instead of 200) instead of the intended
  // 50 ms. Previously loop()'s irregular, sub-millisecond cadence meant
  // `now` almost never landed on the exact boundary, so this was latent.
  if (now - DimlastUpdate >= PRESS_INTERVAL) {
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

    // Every field below is now sent through commandQueue and applied only
    // by applyControlCommand() inside controlTask - none of Kp/Ki/Kd/
    // myPID.SetTunings()/offset/steamSetpoint/setpoint/mode are written
    // directly from this handler (loopTask) any more (Phase 5 review; see
    // docs/PHASE5_NOTES.md). Before this fix, all of them were: harmless
    // with a single task (Phase 0-4), but a real cross-task race once
    // controlTask genuinely runs concurrently (Phase 5) - Kp/Ki/Kd feed
    // myPID.SetTunings(), which can tear against a concurrent
    // myPID.Compute(); setpoint/steamSetpoint are non-atomic doubles read
    // directly by the PID and by applyControlCommand()'s clamp.
    //
    // A field the client's JSON omits is simply not sent - no command at
    // all, not a "resend the current value" no-op - which also removes the
    // need to ever read pendingSettings.setpoint/PIDonly back out of
    // control-owned state from this handler: the previous fallback
    // (`doc["setpoint"] | (pendingSettings.setpoint - offset)`) read a
    // struct controlTask can be mid-write to, and a torn double there could
    // reach applyControlCommand()'s constrain() as NaN (constrain()'s
    // comparisons all fail on NaN, so the clamp would silently pass a NaN
    // setpoint through), permanently stalling the heater with no fault
    // raised. Omitting the command instead of reconstructing the current
    // value sidesteps that read entirely.
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

    // Also routed through the queue (Phase 4) rather than writing PIDonly/
    // currentMode directly: this is safety-critical state (the actuator
    // output-priority resolution in controlStep() reads currentMode - see
    // docs/PHASE3_NOTES.md "Review round 1"), so it goes through the same
    // control-owner-validated path as everything else instead of being a
    // direct cross-context write.
    bool modeOk = true;
    if (!doc["PIDonly"].isNull()) {
      ControlCommand modeCmd;
      modeCmd.type = ControlCommand::Type::SET_MODE;
      modeCmd.mode = doc["PIDonly"] ? OperatingMode::TEMP_ONLY : OperatingMode::NORMAL;
      modeOk = sendControlCommand(modeCmd);
    }

    Serial.println("Config saved");

    // Found in review (Phase 4): the SD write above already committed the
    // new config.json by this point (config.json is a separate, unqueued
    // persistence path this phase doesn't change - see PHASE4_NOTES.md),
    // but the in-memory state only takes effect if every send above
    // actually succeeded. Report a failure here rather than answering
    // "Saved" when the running machine and the file it just wrote can
    // disagree - the same 503-on-full-queue behaviour handleAdjust() already
    // has, extended to the one handler that previously had no way to signal
    // it at all.
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
  // Phase 6, docs/REWRITE_PLAN.md "Clear the current authorization
  // immediately on ... OTA start": forces the SSR deadman's authorization
  // to read as already-expired, independent of whether controlTask keeps
  // running through the OTA session (Phase 5's notes already document that
  // a flash erase/program can stall it regardless). The deadman - running
  // on its own SSR_ENFORCE_INTERVAL_MS cadence, a separate task - picks
  // this up and forces SSR_PIN off within one tick, not waiting for
  // SSR_AUTH_TIMEOUT_MS to elapse on its own.
  ArduinoOTA.onStart([]() {
    ssrAuthorizedUntilMs = 0;
  });
  ArduinoOTA.begin(); // Start OTA service

  // === Web Routes ===
  setupServerRoutes();

  // === Start Server ===
  server.begin();
  Serial.println("HTTP server started.");

    //Dimmer Setup
  pinMode(SSR_PIN, OUTPUT);
  // Started as early as reasonably possible - definitely before the control
  // task (below) ever runs a cycle - so the deadman is already enforcing a
  // safe (off) default rather than the pin being in whatever state
  // pinMode(OUTPUT) alone leaves it in (Phase 6).
  if (!initSsrDeadman()) {
    Serial.println("Failed to create SSR deadman timer - restarting");
    delay(100);
    ESP.restart();
  }
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

  // Started last, after everything controlStep() touches (queues, Dimmer,
  // PID, SD-loaded config) is already initialized above (Phase 5,
  // docs/REWRITE_PLAN.md "Create the FreeRTOS control task"). From this
  // point on, controlStep() runs exclusively inside this task, on its own
  // 10 ms schedule - loop() no longer calls it directly (see loop() below).
  // Checks xTaskCreatePinnedToCore()'s own return value (pdPASS/pdFAIL),
  // not just whether controlTaskHandle got written - found in review: the
  // out-param is only populated on success by contract, so testing it for
  // nullptr instead of checking the return code worked only because the
  // handle happens to be a zero-initialized global.
  BaseType_t controlTaskCreated = xTaskCreatePinnedToCore(
    controlTaskEntry, "ControlTask", CONTROL_TASK_STACK_SIZE, nullptr,
    CONTROL_TASK_PRIORITY, &controlTaskHandle, CONTROL_TASK_CORE);
  if (controlTaskCreated != pdPASS) {
    // Found in review: this task is the only thing that ever calls
    // controlStep() (Phase 5) - if it fails to start, nothing would ever
    // read a sensor, evaluate a fault, or write an actuator again, while
    // the web server stays fully up and keeps answering /getValues from its
    // buildTelemetrySnapshot() fallback, i.e. the machine would look alive
    // and controlled while actually being neither. A log line and
    // continuing into loop() is not an acceptable outcome for that failure
    // mode. Restarting is: it costs a reboot (same as any other startup
    // failure this file already treats as fatal-enough to be worth calling
    // out loudly), and gives a heap-exhaustion-at-boot condition a chance to
    // clear rather than running the machine with no control loop at all.
    Serial.println("Failed to create control task - restarting");
    delay(100);  // let the Serial line above actually go out before reset
    ESP.restart();
  }
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
// heaterDemand/pumpDemand or other control state. light.setBrightness(...)
// is still called directly, exactly once, at the very end of this function.
// The heater path is different since Phase 6: this function only ever
// updates the SSR authorization (ssrAuthorizedOn/ssrAuthorizedUntilMs) - it
// does not call digitalWrite(SSR_PIN, ...) itself any more; the independent
// deadman (ssrDeadmanCallback()) is the only thing that does. Runs inside
// its own dedicated task (Phase 5, controlTaskEntry()), not inside loop() -
// cross-boundary data to/from the web side flows exclusively through
// commandQueue/telemetryQueue (Phase 4); the SSR deadman is a second,
// separate cross-task boundary this function talks to, deliberately kept
// outside that same queue mechanism since it needs to be readable by a
// task that must keep working even if commandQueue's normal consumer
// (this function's own task) is the one that's wedged.
//
// Takes `now` rather than calling millis() itself, matching the
// controlStep(now) signature docs/REWRITE_PLAN.md names, and the periodic-
// cycle pattern controlTaskEntry() uses (one now per cycle, sampled by the
// caller, rather than several slightly-drifting millis() reads taken at
// different points in the same logical cycle). GetPressure(),
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

  // --- Output priority resolution (docs/REWRITE_PLAN.md "Apply output
  // priority: fault/safety off, mode restrictions, state-machine demand,
  // controller output") - the pump write is still the only actuator write
  // for the pump; the heater path now hands off to the independent SSR
  // deadman instead of writing SSR_PIN itself (Phase 6, see below). ---
  double heaterOut = heaterDemand;
  int pumpOut = pumpDemand;

  // 1. Fault/safety off - highest priority, overrides everything above.
  // Previously (Phase 0-2) a temperature fault only forced the SSR off,
  // synchronously inside runPID(); the pump was never coupled to it, so a
  // fault occurring mid-shot left the pump running exactly as if nothing
  // were wrong. This is the fix: any fault now forces both actuators off,
  // every cycle for as long as it persists, regardless of what the shot
  // state machine or PID computed.
  if (faulted) {
    heaterOut = 0;
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
  //
  // A mode flip mid-shot also needs the shot itself ended, not just this
  // cycle's pump output masked - found in review (Phase 4): SET_MODE
  // reaches here through the queue-drain loop above, so the shot-phase
  // branches earlier in this same function already ran and can keep
  // updating pumpDemand every cycle the mask stays up (e.g. currentPressure
  // reads low with no pump running, so the branch re-arms pumppower = 255).
  // Without calling endShot(), the moment the mode flips back to NORMAL
  // mid-shot the pump slams to whatever wound up underneath the mask -
  // the exact masked-output windup/slam bug Phase 3's review fixed for
  // faults via endShot(); this is the same fix for the new SET_MODE path.
  if (currentMode == OperatingMode::TEMP_ONLY) {
    pumpOut = 0;
    if (acDetected) endShot();
  }

  // 3. Whatever remains is the state-machine demand, already resolved
  // through the pressure-regulation controller output above. Clamped
  // defensively - light.setBrightness() takes a uint8_t, so an
  // out-of-range int would silently wrap (e.g. -1 becomes 255, full
  // power) rather than fail loudly; nothing produces an out-of-range
  // value today, but this is the one choke point all pump output passes
  // through, which is exactly where that guarantee belongs.
  pumpOut = constrain(pumpOut, 0, 255);
  heaterOut = constrain(heaterOut, 0, 255);

  // Time-proportional SSR authorization (Phase 6, docs/REWRITE_PLAN.md
  // "Convert the full PID output range to an on-time fraction instead of
  // switching at a fixed value of 127"). heaterOut is 0 exactly when fault/
  // safety-off applied above, or when the PID is genuinely asking for no
  // heat - either way, force off immediately and reset the window, rather
  // than letting a window's on-time (locked in earlier, before the fault)
  // keep authorizing heat for the rest of that window. This is the same
  // "abort, don't mask" principle Phase 3 applied to the shot state
  // machine (endShot()), extended to the window itself: a fault must
  // interrupt a window in progress, not just get silently absorbed by it.
  bool desiredHeaterOn;
  if (heaterOut <= 0) {
    desiredHeaterOn = false;
    ssrWindowStartMs = now;
    ssrWindowOnTimeMs = 0;
  } else {
    // Sample/lock the on-time once per window, at the window's start, not
    // continuously recomputed - standard practice for time-proportional
    // control (avoids the on/off decision changing mid-window as the PID
    // output drifts cycle to cycle, which would fragment what's supposed
    // to be one contiguous on-pulse into several shorter ones).
    if (now - ssrWindowStartMs >= SSR_WINDOW_MS) {
      ssrWindowStartMs = now;
      ssrWindowOnTimeMs = (unsigned long)((heaterOut / 255.0) * SSR_WINDOW_MS);
    }
    desiredHeaterOn = (now - ssrWindowStartMs) < ssrWindowOnTimeMs;
  }

  // Never re-arm before this cycle's safety checks have passed
  // (REWRITE_PLAN.md) - this line is reached only after the fault/mode
  // resolution above has already run for this cycle. Refreshing the
  // authorization every cycle (10ms), not just once per window, means a
  // wedged control task is caught within SSR_AUTH_TIMEOUT_MS (200ms) of its
  // last successful cycle, not up to a full SSR_WINDOW_MS (1000ms) later -
  // strictly stronger than "refresh once per window." ssrAuthorizedOn is
  // written before ssrAuthorizedUntilMs - see the comment on those globals
  // for why the order matters.
  resolvedHeaterOn = desiredHeaterOn;
  resolvedPumpPower = pumpOut;
  ssrAuthorizedOn = desiredHeaterOn;
  ssrAuthorizedUntilMs = now + SSR_AUTH_TIMEOUT_MS;
  light.setBrightness(pumpOut);

  // Publish telemetry for this cycle (Phase 4). xQueueOverwrite() never
  // blocks and never fails - it always succeeds by definition on a
  // length-1 queue, replacing whatever was there. Placed after the
  // actuator writes so resolvedHeaterOn/resolvedPumpPower (read inside
  // buildTelemetrySnapshot()) reflect what was actually just written, not
  // the previous cycle's values.
  if (telemetryQueue != nullptr) {
    TelemetrySnapshot snap = buildTelemetrySnapshot(now);
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

// controlStep() no longer runs here (Phase 5) - it runs in its own task
// (controlTaskEntry(), created in setup()) on a fixed 10 ms schedule, so its
// timing no longer depends on how long server.handleClient()/
// ArduinoOTA.handle()/SD access take on any given iteration of this loop.
void loop() {
  ArduinoOTA.handle(); // Handle OTA updates
  server.handleClient();

  // Reports controlTask's diagnostics counters (Phase 5) - deliberately
  // printed from here, not from inside controlTaskEntry() itself: found in
  // review that a Serial.printf() long enough to hit Print::printf()'s
  // malloc() fallback (its stack buffer is 64 bytes; this line is longer),
  // and Serial.write()'s potential to block once the UART TX FIFO fills,
  // both belonged nowhere near the highest-priority, safety-critical task.
  // loop() has no such constraint. controlTaskWorstCycleUs is reset after
  // each report so the number is a per-5s-window worst case, not a single
  // early spike that then reads as "still there" for the rest of uptime.
  static unsigned long lastReportMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastReportMs >= 5000) {
    lastReportMs = nowMs;
    Serial.printf(
      "controlTask: worst cycle %lu us, deadline misses %lu, stack headroom %u bytes\n",
      controlTaskWorstCycleUs, controlTaskDeadlineMisses,
      (unsigned)uxTaskGetStackHighWaterMark(controlTaskHandle));
    controlTaskWorstCycleUs = 0;
  }
}
