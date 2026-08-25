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
**Only the esp_timer service task ever calls `digitalWrite(SSR_PIN, ...)`**
(`ssrDeadmanCallback()`) - `controlStep()` only ever refreshes an
authorization deadline, never the pin itself, so a wedged `controlTask`
cannot leave the heater pin driven (see `PHASE6_NOTES.md`). The pump has no
equivalent second layer - `light.setBrightness()` is only ever called from
`controlTask`, so a wedged `controlTask` leaves the dimmer at its last
commanded level until the Task WDT recovers the system (`PHASE7_NOTES.md`
Gap 2).

## Cross-task boundaries

Exactly three mechanisms cross between contexts. Nothing else does - any new
cross-task data path should be one of these three, or a new instance of the
same "single word, volatile, no lock" pattern the last two use, not a fourth
shape.

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
- `heaterOn` inside the snapshot is derived fresh at `buildTelemetrySnapshot()`
  time, not cached - but a snapshot itself can still go stale if
  `controlTask` stops publishing new ones, which is why `handleGetValues()`
  as of Phase 7 recomputes `heaterOn` a second time, directly from
  `ssrAuthorizedUntilMs`, rather than trusting the queued copy - see that
  fix's own comment in `handleGetValues()`.

### Plain word-sized globals (bidirectional, no queue)

A small, explicit set of globals are shared without a queue or mutex,
because each is a single word (`bool`, `int`, or `unsigned long` - all
32-bit and atomic on this target) and each has exactly one writer:

| Global | Writer | Readers | Purpose |
|---|---|---|---|
| `ssrAuthorizedUntilMs` | `controlTask` (`controlStep()`) | esp_timer task (`ssrDeadmanCallback()`), both tasks (`buildTelemetrySnapshot()`, `handleGetValues()`) | The SSR deadman's authorization deadline (Phase 6) |
| `otaInProgress` | `loopTask` (`ArduinoOTA` callbacks) | `controlTask` (`controlStep()`'s fault-priority chain) | Latches for the whole OTA session; forces heater off |
| `ssrDeadmanLastRunMs` | esp_timer task (`ssrDeadmanCallback()`) | `loopTask` (`loop()`'s diagnostics report) | Heartbeat only - not itself safety-load-bearing |
| `controlTaskWorstCycleUs`, `controlTaskDeadlineMisses` | `controlTask` | `loopTask` (diagnostics report, which also resets the worst-cycle counter) | Bench/diagnostics counters (Phase 5) |

Every other global that both `loopTask` and `controlTask` can see (`offset`,
`Kp`/`Ki`/`Kd`, `steamSetpoint`, `setpoint`, `pumppower`, `currentPressure`,
`input`, and the `activeSettings`/`shotSettings`/`pendingSettings` structs)
is written **only** from inside `controlStep()`/`applyControlCommand()`
(i.e. only from `controlTask`) and only ever *read* from `loopTask` for
display (`/temp`, `/pressure`, and the handful of `/getValues` fields noted
above) - never written from `loopTask`. A few of these are `double`s, which
are not atomic on this target; the display-only reads of them are a known,
documented residual (Phase 4 §4, Phase 5 review, `PHASE7_NOTES.md` item 16)
- narrow enough (cosmetic UI values, not control decisions) that it's been
judged not worth a fourth cross-task mechanism to close, but it is a real,
named gap, not an oversight.

## Timing reference

| Constant | Value | Governs |
|---|---|---|
| `CONTROL_TASK_PERIOD_TICKS` | 10ms | `controlTask`'s fixed cycle period |
| `PRESS_INTERVAL` | 50ms | `GetPressure()`'s sampling gate |
| `PUMP_ADJUST_INTERVAL` | 200ms | `updatePumpRamp()`'s ramp-rate gate |
| `PID_INTERVAL` | 250ms | `runPID()`'s sampling gate (also `myPID.SetSampleTime(250)`) |
| `AC_OFF_DEBOUNCE_MS` / `AC_OFF_MIN_SAMPLES` | 300ms / 20 samples | Shot-end debounce (`TODO(bench)`, see `PHASE1_NOTES.md`) |
| `FAULT_CLEAR_STABLE_READINGS` | 3 readings (~750ms at `PID_INTERVAL`) | How long a temperature fault must read clean before clearing - see `PHASE7_NOTES.md` Gap 5 for why this is also, unintentionally, how long an over-temperature trip stays latched |
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

`config.json` on the SD card is the only persisted configuration this
firmware has. Its schema is **unchanged by this entire rewrite** (Phase
0 through Phase 7) - confirmed against the shipped front end
(`Discreet_Front_End/config.js`/`config.json`) and `loadSDConfig()`/the
`/saveConfig` handler, both of which still read and write exactly these
nine fields, the same set the pre-rewrite baseline used:

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
  `PHASE7_NOTES.md`, each with its own "Review round 1" section.
- **What still needs real hardware** before any of this should be trusted:
  `PHASE7_NOTES.md`'s bench checklist (the most current and complete one;
  it supersedes and cross-references the narrower bench items left in
  earlier phases' own notes).
- **The staged plan itself**, phase checklists and the full session log:
  `REWRITE_PLAN.md`.
