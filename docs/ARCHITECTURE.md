# Architecture reference

A single-page summary of the execution contexts, cross-task boundaries and
timing this firmware relies on, as of Phase 7 (`v2.0.6-beta`). Everything
here is already documented inline in `Discreet.ino`, phase by phase, as each
piece was introduced - this page exists so a new contributor (or a bench
tester) can get the whole picture without reading 2000+ lines of comments in
commit order first. Where this doc and `Discreet.ino`'s own comments ever
disagree, trust the code and its inline comments; this is a map, not the
territory.

## Execution contexts

Three independent FreeRTOS contexts touch this firmware's state. Nothing
outside these three ever runs firmware code.

| Context | Created | Core | Priority | Runs |
|---|---|---|---|---|
| `loopTask` (Arduino) | by the Arduino core, before `setup()` returns | 1 (`CONFIG_ARDUINO_RUNNING_CORE`) | 1 (hardcoded by the Arduino core) | `setup()` once, then `loop()` forever: `ArduinoOTA.handle()`, `server.handleClient()`, the 5s diagnostics report |
| `controlTask` | `setup()`, via `xTaskCreatePinnedToCore(controlTaskEntry, ...)` (Phase 5) | 1 (`CONTROL_TASK_CORE`) | 5 (`CONTROL_TASK_PRIORITY`) | `controlTaskEntry()`: `controlStep()` on a fixed 10ms period (`CONTROL_TASK_PERIOD_TICKS`), via `xTaskDelayUntil()` |
| esp_timer service task | internal to the `esp_timer` component, `ESP_TIMER_TASK` dispatch | unconfirmed (precompiled library - see `PHASE6_NOTES.md`) | `ESP_TASK_TIMER_PRIO` (22, arithmetically confirmed) | Every `esp_timer` callback using `ESP_TIMER_TASK` dispatch, serially, in one shared task: `ssrDeadmanCallback()` (every `SSR_ENFORCE_INTERVAL_MS`) and `buzzerTimerCallback()` (one-shot, re-armed per beep) |

`controlTask` and `loopTask` share the same core (1) but `controlTask`'s
higher priority means the scheduler preempts `loop()` the instant
`controlTask`'s 10ms deadline arrives, regardless of what `loop()` is doing.
The Arduino Wi-Fi event task (`arduino_events`) also runs on core 1 at a
higher priority still (~19) - normally idle, but see `PHASE7_NOTES.md` item
12 for why a Wi-Fi disconnect/reconnect storm is a real, unmeasured
contention risk worth a bench check, not an architectural guarantee.

**Only `controlTask` ever computes an actuator decision or writes an
actuator-facing value** (`heaterDemand`, `pumpDemand`, `ssrAuthorizedUntilMs`,
`resolvedPumpPower`, `light.setBrightness()` - all inside `controlStep()`).
`digitalWrite(SSR_PIN, ...)` has exactly two call sites: `initSsrDeadman()`
(a one-time `LOW` write, from `loopTask`/`setup()`, before the deadman timer
or `controlTask` even exist yet - the known-safe default while nothing else
is running) and `ssrDeadmanCallback()` (the esp_timer service task,
thereafter, forever). Once the deadman is running, it is the *only* thing
that ever writes that pin again - `controlStep()` only ever refreshes an
authorization deadline, never the pin itself, so a wedged `controlTask`
cannot leave the heater pin driven (see `PHASE6_NOTES.md`). The pump has no
equivalent second layer - `light.setBrightness()` is only ever called from
`controlTask`, so a wedged `controlTask` leaves the dimmer at its last
commanded level until the Task WDT recovers the system (`PHASE7_NOTES.md`
Gap 2).

## Cross-task boundaries

Four mechanisms cross between contexts - two queues, a small set of plain
word-sized globals, and one real mutex (the buzzer). Any new cross-task data
path should be one of the first three; the buzzer's mutex is not a pattern
to imitate (see its own note below), it is a pre-existing exception that
predates this rewrite.

### `commandQueue` (`loopTask` -> `controlTask`)

- `QueueHandle_t`, depth `COMMAND_QUEUE_DEPTH` (8), element type `ControlCommand`.
- Producers: `handleAdjust()` (`/adjust`) and the `/saveConfig` handler, both
  via `sendControlCommand()` - `xQueueSend()` with 0 ticks-to-wait, never
  blocks the web server. A full/uncreated queue returns `false`, which both
  call sites turn into an HTTP `503` rather than a silently dropped edit.
- Consumer: `controlStep()`'s queue-drain loop, at the very top of every
  10ms cycle - drains everything currently queued, in order, through
  `applyControlCommand()` (the sole "control owner," Phase 4) before that
  cycle's control decisions run.
- Schema (`ControlCommand`, `Discreet.ino` near the top): a `Type` enum
  tag plus a small union-by-convention of payload fields (`delta`, `value`,
  `mode`, `kp`/`ki`/`kd`) - which fields are meaningful depends on `type`;
  see the enum's own inline comments for the mapping. Every field this
  firmware treats as safety- or PID-relevant (brew/steam setpoint,
  preinf/bloom/pressure targets, mode, tunings, offset) is routed through
  this queue exclusively - nothing outside `applyControlCommand()` writes
  any of them directly (Phase 4, hardened further in Phase 5 review once a
  second task made the old direct writes a real race).

### `telemetryQueue` (`controlTask` -> `loopTask`)

- `QueueHandle_t`, depth 1, element type `TelemetrySnapshot`.
- Producer: `controlStep()`, once per cycle, via `xQueueOverwrite()` - always
  succeeds, always replaces whatever was there, so the publisher never
  blocks.
- Consumers: `handleGetValues()` (`/getValues`) via `xQueuePeek()` - never
  consumes, never blocks, always returns the latest complete snapshot (or
  falls back to calling `buildTelemetrySnapshot(millis())` directly if the
  queue isn't ready yet, e.g. before the first cycle has run).
- Schema (`TelemetrySnapshot`, `Discreet.ino` near the top): a complete,
  copyable view of everything the web side might want to show - mode,
  shot state, fault, temperature/pressure, both actuators' resolved and
  pre-resolution values, elapsed shot time, all three settings revisions,
  and a `timestampMs` for staleness detection (see `PHASE7_NOTES.md` item
  16 for the one thing this mechanism does *not* cover: `/temp`,
  `/pressure` and a few `/getValues` fields still read live globals
  directly rather than going through this snapshot - documented, not
  fixed, in that phase's notes).
- There is no `heaterOn` field in this struct (removed in Phase 8 - see the
  comment on `TelemetrySnapshot`'s declaration). It was written but never
  read once Phase 7 switched `handleGetValues()` to compute `heaterOn`
  directly from `ssrAuthorizedUntilMs` instead - a snapshot field is only
  ever as fresh as `controlTask`'s last successful publish, which defeats
  the point for the one field whose whole purpose is staying correct even
  when `controlTask` is unhealthy. See the plain-globals section below.

### Plain word-sized globals (bidirectional, no queue)

A small, explicit set of globals are shared without a queue or mutex,
because each is a single word (`bool`, `int`, or `unsigned long` - all
32-bit and atomic on this target):

| Global | Writer(s) | Readers | `volatile`? | Purpose |
|---|---|---|---|---|
| `ssrAuthorizedUntilMs` | `controlTask` (`controlStep()`) | esp_timer task (`ssrDeadmanCallback()`), both tasks (`buildTelemetrySnapshot()`, `handleGetValues()`) | yes | The SSR deadman's authorization deadline (Phase 6) |
| `otaInProgress` | `loopTask` (`ArduinoOTA` callbacks) | `controlTask` (`controlStep()`'s fault-priority chain) | yes | Latches for the whole OTA session; forces heater off |
| `ssrDeadmanLastRunMs` | esp_timer task (`ssrDeadmanCallback()`) | `loopTask` (`loop()`'s diagnostics report) | yes | Heartbeat only - not itself safety-load-bearing |
| `controlTaskWorstCycleUs`, `controlTaskDeadlineMisses` | both: `controlTask` sets them per-cycle, `loopTask` resets `controlTaskWorstCycleUs` to 0 after each 5s diagnostics report | `loopTask` (diagnostics report) | no | Bench/diagnostics counters (Phase 5) - the shared reset is a deliberate, accepted exception to "one writer," not an oversight: worst case is one report window's peak gets zeroed a cycle early, cosmetic for a diagnostics-only counter |

Every other control-relevant global (`offset`, `Kp`/`Ki`/`Kd`, `steamSetpoint`,
`setpoint`, `currentMode`, and the `activeSettings`/`shotSettings`/
`pendingSettings` structs) follows a **two-phase** ownership model, not a
single rule: during boot, `loadSDConfig()` (`loopTask`, from `setup()`,
before `controlTask` exists) writes all of them directly - safe, because
nothing else is running yet. Once `controlTask` starts, every one of them is
written **only** from inside `controlStep()`/`applyControlCommand()`
thereafter - `loopTask` never writes any of them again post-boot. `loopTask`
does still *read* a few of these directly rather than through
`telemetryQueue` - not only for display (`/temp`, `/pressure`, and the
handful of `/getValues` fields `PHASE7_NOTES.md` item 16 already names), but
also, in `/saveConfig`, to *construct* a new command's payload (`offset` is
added into a posted `setpoint`/`steamSetpoint` before it's queued) - a
narrower but real instance of the same class of gap: a `SET_OFFSET` command
queued in the same request as a `setpoint`/`steamSetpoint` edit can be
applied against a different `offset` than the one that payload was computed
from. A few of the read-only globals are `double`s, not atomic on this
target; both this and the `/saveConfig` case are known, documented residuals
(Phase 4 §4, Phase 5 review, `PHASE7_NOTES.md` item 16) - narrow enough that
closing them hasn't been judged worth a fifth cross-task mechanism, but
they're real, named gaps, not oversights.

### The buzzer (mutex, not a pattern to copy)

`queueBuzzer()`/`buzzerTimerCallback()` share `buzzerActive`, `buzzerPinOn`,
`buzzerBeepsRemaining`, `buzzerOnMs`, `buzzerOffMs` and `BUZZER_PIN` itself
behind `buzzerMutex`, a real `SemaphoreHandle_t` - the one place in this
firmware that takes an actual FreeRTOS mutex across tasks, predating this
rewrite. All three contexts touch it: `loopTask` (`startSD()`/`startWiFi()`
boot beeps, `waitForBuzzer()`), the esp_timer service task
(`buzzerTimerCallback()`, which also dispatches the SSR deadman - see
`SSR_ENFORCE_INTERVAL_MS`'s row below for why that callback's mutex wait is
bounded, not `portMAX_DELAY`), and `controlTask` itself (`steam()`, called
at the end of every `controlStep()` cycle, can call `queueBuzzer()` for the
steam-ready beep). Not a fourth instance of the plain-globals pattern above
- multi-field state needs real mutual exclusion, which is exactly why this
predates and sits outside that pattern rather than being folded into it.

## State model

Three scoped enums (`Discreet.ino`, declared together near the top of the
file) - part of the same schema `TelemetrySnapshot` publishes, so they
belong on this page as much as the queue that carries them does:

- **`OperatingMode { NORMAL, TEMP_ONLY }`** - set only via the
  `SET_MODE` command (`applyControlCommand()`), which is how `/saveConfig`'s
  `PIDonly` field takes effect at runtime. `TEMP_ONLY` disables shot
  detection and forces the pump off (`controlStep()`'s mode-restriction
  step, including ending an in-progress shot if the mode flips mid-shot).
- **`ShotState { IDLE, PREINFUSION, BLOOM, EXTRACTION, COMPLETE, FAULT }`** -
  a telemetry label, not a control input (see `acceptPreinftimeEdit()`'s own
  comment for why `!acDetected` is what control logic actually gates on,
  never this enum). Driven by `acDetected` + `actime` partitioning against
  `shotSettings.preinftime`/`bloomtime` inside `controlStep()`; `FAULT`
  overrides it from any state. `COMPLETE` is set by `endShot()` for exactly
  one `controlTask` cycle and is effectively unobservable over HTTP -
  `telemetryQueue` is depth 1 and the next cycle overwrites it with `IDLE`
  before any `/getValues` poll at UI rates could catch it.
- **`FaultCode { NONE, INVALID_TEMPERATURE }`** - set by `runPID()`'s
  sanity check (`isnan`/`MIN_PLAUSIBLE_TEMP_C`/`160`), cleared after
  `FAULT_CLEAR_STABLE_READINGS` consecutive good readings. One name, two
  different real conditions ("sensor is lying" and "boiler is genuinely too
  hot") - see the timing table below and `PHASE7_NOTES.md` Gap 5 for why
  that matters: the same debounce built for a flapping sensor also clears a
  genuine over-temperature trip, which is a real gap, not a rename waiting
  to happen.

**The settings latch** (`activeSettings`/`shotSettings`/`pendingSettings`,
all `ControlSettings` - `Discreet.ino`, declared with the enums above) is
the other major piece of cross-cutting state this firmware relies on, and
is easy to undersell as "just three globals": it's a three-stage
latch-and-promote design with its own revision counters
(`activeSettings.revision`/`shotSettings.revision`/`pendingSettings.revision`,
all published in `TelemetrySnapshot`). `activeSettings` is in effect
whenever no shot is running; `shotSettings` is latched from it once, at
shot start, and is the *only* one the shot-phase logic reads for the rest
of that shot - so a mid-shot edit (which only ever reaches
`pendingSettings`) cannot alter a shot already in progress. `endShot()`
promotes the entire `pendingSettings` struct to `activeSettings` on return
to `IDLE`, in one step, not field-by-field - see `REWRITE_PLAN.md`'s
"Settings lifecycle" for the original design intent this implements.

## Timing reference

| Constant | Value | Governs |
|---|---|---|
| `CONTROL_TASK_PERIOD_TICKS` | 10ms | `controlTask`'s fixed cycle period |
| `PRESS_INTERVAL` | 50ms | `GetPressure()`'s sampling gate |
| `PUMP_ADJUST_INTERVAL` | 200ms | `updatePumpRamp()`'s ramp-rate gate |
| `PID_INTERVAL` | 250ms | `runPID()`'s sampling gate (also `myPID.SetSampleTime(250)`) |
| `AC_OFF_DEBOUNCE_MS` / `AC_OFF_MIN_SAMPLES` | 300ms / 20 samples | Shot-end debounce (`TODO(bench)`, see `PHASE1_NOTES.md`) |
| `FAULT_CLEAR_STABLE_READINGS` | 3 readings (~750ms at `PID_INTERVAL`) | How long a temperature fault must read clean before *self-clearing* - designed to debounce a flapping sensor; `PHASE7_NOTES.md` Gap 5 is the finding that this is a real bug, not a feature, when the cause is a genuine over-temperature condition instead - it does **not** latch |
| `SSR_WINDOW_MS` | 1000ms | Time-proportional SSR on/off window (`TODO(bench)`) |
| `SSR_AUTH_TIMEOUT_MS` | 200ms | How long an SSR authorization stays valid unrefreshed |
| `SSR_ENFORCE_INTERVAL_MS` | 10ms | The deadman's own poll/enforce cadence, and the real ceiling on achievable SSR duty resolution (not the full 8-bit PID range - see `PHASE6_NOTES.md`) |
| `MIN_PLAUSIBLE_TEMP_C` | 5C | Lower sanity bound on `input` (Phase 7 - see `PHASE7_NOTES.md` item 5) |
| Task WDT timeout | 5s (`CONFIG_ESP_TASK_WDT_TIMEOUT_S`, this project's actual `esp32:esp32@2.0.17` sdkconfig - verified directly, not assumed) | System-wide recovery if `controlTask` (or any other watched task) hangs - confirmed to actually reboot on this target (`CONFIG_ESP_TASK_WDT_PANIC=y`, `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y`), not just log |

Worst-case SSR decay-to-off, accounting for the shared esp_timer dispatch
task also running the buzzer callback: `SSR_AUTH_TIMEOUT_MS` (200) + one
enforce tick (10) + one skipped buzzer tick (`BUZZER_MUTEX_WAIT_TICKS`, 20)
= ~230ms. See `PHASE7_NOTES.md` item 10 for the full derivation.

## Configuration persistence

`config.json` on the SD card is where the machine's own settings persist
(there is a second, unrelated persisted file - `/currentTheme.txt`, plus
whichever `.css` it names - for the front end's cosmetic theme selection;
see `handleApplyTheme()`/`getCurrentTheme()`, not covered further here).
`config.json`'s schema is **unchanged by this entire rewrite** (Phase 0
through Phase 7) - confirmed against the shipped front end
(`Discreet_Front_End/config.js`/`config.json`) and `loadSDConfig()`, which
still reads exactly these nine fields, the same set the pre-rewrite
baseline used. The `/saveConfig` write path itself is field-agnostic - it
persists whatever JSON the client posts verbatim (`serializeJson(doc,
file)`), with no schema check on write - so the nine-field contract is
really enforced by the two things that read the file back meaningfully
(`loadSDConfig()` at boot) and by what the shipped `config.js` chooses to
post, not by the write path itself:

```json
{
  "ssid": "string",
  "password": "string",
  "Kp": 100, "Ki": 6, "Kd": 60,
  "setpoint": 93,
  "offset": 9,
  "steamSetpoint": 135,
  "PIDonly": false
}
```

No migration step is needed for an existing `config.json` from a
pre-rewrite install - it will load without modification. What changed is
entirely on the firmware side of that boundary: these fields used to be
applied by direct global writes from whichever handler read them; as of
Phase 4/5 they're applied exclusively through `commandQueue` ->
`applyControlCommand()` (`SET_TUNINGS`, `SET_OFFSET`, `SET_BREW_SETPOINT_ABSOLUTE`,
`SET_STEAM_SETPOINT`, `SET_MODE` at runtime), and as of the Phase 7 review
fix, `loadSDConfig()` itself also calls `myPID.SetTunings()` and clamps
`setpoint` the same way that runtime path already did - both previously
missing at boot specifically (see `PHASE7_NOTES.md` "Review round 1" items
2-3). None of this changes what's stored on disk or how to read/write it.

## Where to look next

- **Why** each piece of this exists, in the order it was built, with the
  bugs found and fixed along the way: `PHASE1_NOTES.md` through
  `PHASE8_NOTES.md`, each with its own "Review round 1" section.
- **What still needs real hardware** before any of this should be trusted:
  [HARDWARE_TEST_PROCEDURE.md](HARDWARE_TEST_PROCEDURE.md) - one ordered
  runbook consolidating every bench item named across every phase's own
  notes (`PHASE1_NOTES.md` through `PHASE7_NOTES.md`), prioritized. Go
  there first; the individual phase notes are for *why* an item exists, not
  for the current to-do order.
- **The staged plan itself**, phase checklists and the full session log:
  `REWRITE_PLAN.md`.
