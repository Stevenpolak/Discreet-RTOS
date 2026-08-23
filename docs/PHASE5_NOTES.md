# Phase 5 notes — create the FreeRTOS control task

Date: 2026-08-23
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 5, applied on top of the
merged Phase 0-4 work (`v2.0.3-beta`, see [BASELINE.md](BASELINE.md),
[PHASE1_NOTES.md](PHASE1_NOTES.md), [PHASE2_NOTES.md](PHASE2_NOTES.md),
[PHASE3_NOTES.md](PHASE3_NOTES.md), [PHASE4_NOTES.md](PHASE4_NOTES.md)).

Goal per `REWRITE_PLAN.md`: move `controlStep()` into its own dedicated
FreeRTOS task, running on a fixed period, watched by the Task WDT
explicitly. **This is the phase where a second task actually starts
existing** - Phase 4 built the queue mechanism specifically so this move
would be a relocation, not a redesign, and that held: `controlStep()`'s own
body did not need to change.

## What changed

### 1. `controlTaskEntry()` — the new task, created in `setup()`

`xTaskCreatePinnedToCore()` is called as the very last thing in `setup()`,
after every subsystem `controlStep()` touches (queues, SD-loaded config,
Dimmer, PID) is already initialized. The task body is a fixed 10 ms loop
using `vTaskDelayUntil()` (so drift doesn't accumulate from the task's own
execution time), calling `controlStep(millis())` once per period - exactly
matching the signature and `now`-sampling convention `controlStep()` already
had from Phase 3/4.

`GetPressure()`/`runPID()`/`updatePumpRamp()` were not changed: they already
self-gate on `PRESS_INTERVAL` (50 ms) and `PID_INTERVAL` (250 ms) via their
own internal `millis()` checks. Calling `controlStep()` every 10 ms just
lets those existing gates fire on their intended cadence instead of on
whatever cadence `loop()` happened to reach `controlStep()` at - this is
what "schedule 50 ms pressure work and 250 ms temperature/PID work inside
the task" meant in practice; no restructuring of the gating logic was
needed.

### 2. Priority and core affinity — chosen deliberately, not left at defaults

- **Core 1 (APP_CPU)**, the same core Arduino's own `loopTask` runs on
  (`CONFIG_ARDUINO_RUNNING_CORE` defaults to 1), *not* core 0, where the
  Wi-Fi/BT driver's own tasks run at high priority (~23). Sharing a core
  with `loopTask` - not with the radio stack - is the point: this task only
  needs to preempt `server.handleClient()`/`ArduinoOTA.handle()`/SD access,
  never to contend with Wi-Fi's own real-time deadlines for CPU time.
- **Priority 5**, strictly above `loopTask`'s priority (hardcoded to 1 by
  the Arduino core), so the scheduler preempts `loop()` the instant this
  task's 10 ms deadline arrives, regardless of what `loop()` is in the
  middle of - this is the mechanism that makes control timing deterministic
  under service-side load, which is this phase's exit condition. Kept well
  below the Wi-Fi/BT driver tasks' priority so this task cannot starve the
  radio stack, even though it doesn't share a core with where those tasks
  usually run.
- **Stack size 4096 bytes** (ESP-IDF FreeRTOS specifies task stack size in
  bytes, not words, unlike vanilla FreeRTOS). `controlStep()` and everything
  it calls do no dynamic allocation and no `String` work; this is a
  conservative starting budget, not a measurement - see "Bench items" below.

### 3. Task Watchdog

`controlTaskEntry()` calls `esp_task_wdt_add(NULL)` once, at task start, to
explicitly subscribe itself - per `REWRITE_PLAN.md`, "do not rely on the
watched idle tasks": arduino-esp32's default TWDT setup watches the idle
tasks as a proxy for a runaway task starving them, which is a weaker,
indirect guarantee than watching this specific task. `esp_task_wdt_reset()`
is called once per loop iteration, immediately after `controlStep()`
returns - i.e. only after a complete cycle including the safety evaluation
and the actuator writes at the end of `controlStep()`, per the plan. If
`controlStep()` ever hangs, that line is simply never reached and the TWDT's
own timeout is what recovers the system, not any code in this task. Both
`esp_task_wdt_add()`'s and `esp_task_wdt_reset()`'s return values are
checked and logged on failure, per the plan's "check and handle the return
values" item.

The TWDT's own timeout was deliberately left at arduino-esp32's default
(`CONFIG_ESP_TASK_WDT_TIMEOUT_S` = 5 s) - not tuned in this phase. See
"Considered, not changed" below.

### 4. Diagnostics: worst cycle time, deadline misses, stack high-water mark

`controlTaskEntry()` tracks the worst observed `controlStep()` duration
(`micros()` before/after) and counts deadline misses (detected the standard
way for `vTaskDelayUntil()`: if the tick count already reached or passed the
next scheduled wake time *before* `vTaskDelayUntil()` is called, that call
will return immediately with no delay, which is the definition of a missed
period). Both, plus `uxTaskGetStackHighWaterMark(NULL)`, are printed to
Serial every 5 seconds.

This is deliberately Serial-only, not added to `TelemetrySnapshot`/
`/getValues`: it's bench instrumentation for tuning stack size and
confirming timing, not a value the front end or any HTTP consumer needs,
and keeping it off the HTTP surface keeps this phase's scope to the
control-task migration itself rather than growing the telemetry API.

### 5. `loop()`

`controlStep(millis())` was removed from `loop()`, which now only calls
`ArduinoOTA.handle()`/`server.handleClient()`. Zero-cross dimmer timing is
untouched - it already lived entirely in `dimmable_light`'s own interrupt
path (`activate_thyristors`), which this phase does not touch;
`controlStep()` only ever calls `light.setBrightness()` to set a target, the
same as before.

## What was intentionally *not* done in this phase

- **The Task WDT's own timeout was not tuned.** Left at arduino-esp32's
  default 5 s. A 10 ms-cadence control task hanging for up to 5 s before
  recovery is a long window relative to its own period, but changing the
  global TWDT init risks conflicting with arduino-esp32's own startup call
  to `esp_task_wdt_init()` and affects idle-task monitoring too - judged out
  of scope for "explicitly subscribe and feed this task," which is what the
  checklist asks for. A good target for bench tuning once real timing
  numbers exist.
- **No changes to `GetPressure()`/`runPID()`/`updatePumpRamp()`'s internal
  gating**, beyond what's already described in §1 - they didn't need any.
- **Phase 6's SSR deadman/time-proportional output work is explicitly a
  separate phase** per `REWRITE_PLAN.md`; not started here.

## Build verification

Compiled with the same toolchain as Phase 0-4 (`esp32:esp32@2.0.17`):

```
Sketch uses 907529 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51308 bytes (15%) of dynamic memory, leaving 276372 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +796 bytes flash / +16 bytes RAM versus
`v2.0.3-beta` (906,733 / 51,292) - the new task's own code, the
`esp_task_wdt_add()`/`esp_task_wdt_reset()` calls, and the diagnostics
counters/Serial report.

## Behaviour-preservation checklist

- [x] `controlStep()`'s own body is unchanged - verified by diff; this
      phase only changes what calls it and when.
- [x] `GetPressure()`/`runPID()`/`updatePumpRamp()`'s internal timing gates
      are unchanged and now fire on their intended cadence more reliably
      than before (previously gated by both their own interval check *and*
      how often `loop()` happened to reach `controlStep()`; now only the
      former, since the task's own 10 ms period is tighter than either
      gate).
- [ ] **New concurrency, not previously possible: a real second task now
      exists.** See "Considered, not changed - a new correctness question
      this phase raises" below - this is the one item from this phase that
      most needs independent review before merge.
- [ ] Bench re-test required, same hardware limitation as every prior
      phase. This phase specifically needs: confirm the Serial diagnostics
      report shows a worst-case cycle time comfortably under 10 ms and zero
      deadline misses under normal load; confirm stack high-water mark
      leaves comfortable headroom under `CONTROL_TASK_STACK_SIZE` (4096
      bytes) - reduce or grow it based on the measured number, don't leave
      it as a guess; stress Wi-Fi, `/adjust`/`/saveConfig` requests, SD
      access and an OTA session simultaneously with a simulated shot and
      confirm the diagnostics stay clean throughout; deliberately wedge the
      control task (e.g. a temporary infinite loop behind a debug flag) and
      confirm the Task WDT actually recovers the system and that outputs
      come up safe (off) after the resulting reboot, not stuck at whatever
      they were when the task wedged.

## Considered, not changed - a new correctness question this phase raises

Splitting `controlStep()` into a real second task means the handful of
globals Phase 4 §4 documented as "remain direct, deliberately" - `Kp`/`Ki`/
`Kd`/`offset`/`steamSetpoint`/`setpoint` - are, for the first time, read and
written from two genuinely concurrent tasks rather than from one task
calling itself through `loop()`. Phase 4's notes flagged this exact
question and deferred it here: "worth re-examining once Phase 5 makes these
genuinely concurrent reads."

Re-examined now that the concurrency is real:

- **`setpoint`/`steamSetpoint`** (both `double`, i.e. two 32-bit words on
  this target) are written directly from `/saveConfig` (`loopTask`) and
  read directly by `myPID.Compute()` inside `runPID()` (`controlTask`) via
  the pointer `myPID` was constructed with. A `double` write is not atomic
  on this hardware; a `controlTask` read landing between the two word
  writes of a `loopTask` update is a genuine torn read, not a theoretical
  one. This is the single most concrete new risk this phase introduces, and
  it feeds the PID output directly.
- **`myPID.SetTunings(Kp, Ki, Kd)`** is called from `/saveConfig`
  (`loopTask`) while `myPID.Compute()` runs on its own schedule inside
  `runPID()` (`controlTask`); `SetTunings()` writes several of `PID_v1`'s
  internal state fields non-atomically, and `Compute()` reads them.
  Same class of risk as above, one level removed from this file's own
  globals.
- **`offset`** (`int`, one word) is lower-risk - a single-word write is
  atomic on this hardware, so a reader gets either the old or the new
  value, never a torn one - but a reader on `controlTask` can still observe
  a stale value for up to one `loopTask` scheduling quantum, which matters
  because `offset` doubles as a safety-clamp bound in
  `applyControlCommand()`'s `constrain()` calls (Phase 4 §4 already flagged
  this double duty as worth revisiting).

Not fixed in this PR: closing this properly means either routing these
values through `commandQueue` the same way brew setpoint/mode already are
(a real, if small, extension of Phase 4's command set), or wrapping the
handful of read/write sites in a critical section - both are more than
"create the FreeRTOS control task" asks for, and neither should be guessed
at ahead of independent review confirming severity and the preferred fix
shape. Written down here explicitly so the review has a concrete, named
target rather than needing to rediscover it.
