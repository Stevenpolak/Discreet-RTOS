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
body did not need to change. What the real concurrency *did* expose is
almost the entire subject of "Review round 1" below.

## What changed

### 1. `controlTaskEntry()` — the new task, created in `setup()`

`xTaskCreatePinnedToCore()` is called as the very last thing in `setup()`,
after every subsystem `controlStep()` touches (queues, SD-loaded config,
Dimmer, PID) is already initialized. The task body is a fixed 10 ms loop
using `xTaskDelayUntil()` (so drift doesn't accumulate from the task's own
execution time), calling `controlStep(now)` once per period - `now` is
sampled once per cycle and reused, matching the convention `controlStep()`
already established in Phase 3/4.

`GetPressure()`/`runPID()`/`updatePumpRamp()` were not restructured: they
already self-gate on `PRESS_INTERVAL` (50 ms) and `PID_INTERVAL` (250 ms)
via their own internal `millis()` checks. Calling `controlStep()` every
10 ms just lets those existing gates fire on their intended cadence instead
of on whatever cadence `loop()` happened to reach `controlStep()` at - see
"Review round 1" for one place (`updatePumpRamp()`) where the *exactness* of
the new 10 ms grid interacted badly with a pre-existing strict-inequality
gate.

### 2. Priority and core affinity — chosen deliberately, verified against this project's actual build, not assumed

- **Core 1 (APP_CPU)**, the same core Arduino's own `loopTask` runs on
  (`CONFIG_ARDUINO_RUNNING_CORE=1` for this target). Verified during review
  against this project's actual `esp32:esp32@2.0.17` `sdkconfig`, not
  assumed from general ESP32 lore: lwIP's `tcpip_thread` - which does the
  actual per-packet work behind `server.handleClient()` - is pinned to
  **core 0** (`CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y`), so core 1 really is
  free of the one task that would matter most for contention. The Arduino
  event task (`arduino_events`) does run on core 1 at a high priority
  (`ESP_TASKD_EVENT_PRIO`, ~19), but it's idle except for infrequent Wi-Fi/
  system events, not a per-request cost.
- **Priority 5**, strictly above `loopTask`'s priority (hardcoded to 1 by
  the Arduino core), so the scheduler preempts `loop()` the instant this
  task's 10 ms deadline arrives. Kept well below `arduino_events`/lwIP/
  Wi-Fi-driver priorities (~18-23) so this task cannot starve them.
- **A known, explicitly-not-worked-around gap**: an OTA write's flash
  erase/program calls disable interrupts and caches on **both** cores for
  their duration - an ESP32 SPI-flash/XIP hardware constraint, not a
  scheduling choice. No core or priority assignment prevents this task from
  being stalled during an OTA upload; see the bench checklist below.
- **Stack size 4096 bytes** (ESP-IDF FreeRTOS specifies task stack size in
  bytes, not words, unlike vanilla FreeRTOS). `controlStep()` and everything
  it calls do no dynamic allocation and no `String` work; this is a
  conservative starting budget, not a measurement - see "Bench items" below.

### 3. Task Watchdog

`controlTaskEntry()` calls `esp_task_wdt_add(NULL)` once, at task start, to
explicitly subscribe itself - per `REWRITE_PLAN.md`, "do not rely on the
watched idle tasks." `esp_task_wdt_reset()` is called once per loop
iteration, immediately after `controlStep()` returns - only after a
complete cycle including the safety evaluation and the actuator writes.
Both calls' return values are checked; a failed `esp_task_wdt_add()` is
logged once (not per-cycle - see "Review round 1", finding 3) and disables
the per-cycle reset call for the rest of this task's life, since there is
nothing to feed once unsubscribed.

The TWDT's own timeout was deliberately left at arduino-esp32's default
(`CONFIG_ESP_TASK_WDT_TIMEOUT_S` = 5 s) - not tuned in this phase; see
"Considered, not changed" below.

### 4. Diagnostics: worst cycle time, deadline misses, stack high-water mark

`controlTaskEntry()` tracks the worst observed `controlStep()` duration
(`micros()` before/after) and counts deadline misses using
`xTaskDelayUntil()`'s own return value (`pdFALSE` means this cycle's work
alone consumed a full period or more - see "Review round 1", finding 4).
Both counters, plus `uxTaskGetStackHighWaterMark()`, are printed to Serial
every 5 seconds - but the print itself happens from **`loop()`**, not from
inside the control task; see "Review round 1", finding 6, for why that
matters and isn't just a style choice.

This is deliberately Serial-only, not added to `TelemetrySnapshot`/
`/getValues`: it's bench instrumentation for tuning stack size and
confirming timing, not a value the front end or any HTTP consumer needs.

### 5. `loop()`

`controlStep(millis())` was removed from `loop()`, which now calls
`ArduinoOTA.handle()`/`server.handleClient()` and prints the diagnostics
report on its own 5 s gate (see §4). Zero-cross dimmer timing is untouched -
it already lived entirely in `dimmable_light`'s own interrupt path, which
this phase does not touch; `controlStep()` only ever calls
`light.setBrightness()` to set a target, the same as before.

## What was intentionally *not* done in this phase

- **The Task WDT's own timeout was not tuned.** Left at arduino-esp32's
  default 5 s. Changing the global TWDT init risks conflicting with
  arduino-esp32's own startup call to `esp_task_wdt_init()` and affects
  idle-task monitoring too - judged out of scope for "explicitly subscribe
  and feed this task." A good target for bench tuning once real timing
  numbers exist.
- **Phase 6's SSR deadman/time-proportional output work is explicitly a
  separate phase** per `REWRITE_PLAN.md`; not started here.

## Build verification

Compiled with the same toolchain as Phase 0-4 (`esp32:esp32@2.0.17`), after
the review-round-1 fixes below:

```
Sketch uses 908053 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51308 bytes (15%) of dynamic memory, leaving 276372 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +1,320 bytes flash / +16 bytes RAM versus
`v2.0.3-beta` (906,733 / 51,292) - the new task's own code, three new
`ControlCommand` types and their `applyControlCommand()` cases, the
restructured `/saveConfig` handler, and the diagnostics counters/report.
`ControlCommand` growing by three `double` fields also adds ~192 bytes to
`commandQueue`'s heap allocation (depth 8 × 24 bytes) at runtime - not
visible in the linker's static RAM figure above, which only covers
statically-allocated globals.

## Behaviour-preservation checklist

- [x] `controlStep()`'s own body is unchanged - verified by diff; this
      phase only changes what calls it and when.
- [x] `GetPressure()`/`runPID()`'s internal timing gates are unchanged and
      fire on their intended cadence. `updatePumpRamp()`'s gate needed a
      one-character fix (`>` to `>=`) to actually get its intended cadence
      on the new exact grid - see "Review round 1", finding 5.
- [x] Every control-relevant write that could reach a genuinely concurrent
      `controlTask` from `loopTask` (Kp/Ki/Kd + `myPID.SetTunings()`,
      `offset`, `steamSetpoint`, brew setpoint, mode) now goes through
      `commandQueue`, same as brew setpoint/mode already did from Phase 4 -
      see "Review round 1", finding 1.
- [ ] Bench re-test required, same hardware limitation as every prior
      phase. This phase specifically needs, in priority order:
      1. **Zero-cross sampling under real 50 Hz mains** - see "Considered,
         not changed" below; this is the one finding in this phase judged
         too hardware-dependent to fix blind, and should be the first thing
         verified.
      2. Confirm the Serial diagnostics report shows a worst-case cycle
         time comfortably under 10 ms and zero deadline misses under normal
         load.
      3. Confirm stack high-water mark leaves comfortable headroom under
         `CONTROL_TASK_STACK_SIZE` (4096 bytes).
      4. Stress Wi-Fi, `/adjust`/`/saveConfig` requests, SD access and an
         OTA session simultaneously with a simulated shot; confirm the
         diagnostics stay clean under everything except OTA specifically
         (an OTA-induced deadline miss is expected per §2's flash-erase
         note, not a bug to chase).
      5. Deliberately wedge the control task and confirm the Task WDT
         recovers the system and outputs come up safe (off) after reboot.

## Review round 1: findings and fixes

7-angle independent review (plus one gap self-identified while writing
`PHASE5_NOTES.md`'s first draft: the concurrency question in finding 1
below was already flagged there as "considered, not changed" before review
ran, and was independently confirmed - and judged to belong in this PR, not
deferred further - by four of the seven angles).

### Fixed

1. **Kp/Ki/Kd (+ `myPID.SetTunings()`), `offset` and `steamSetpoint` were
   still being written directly from `/saveConfig` (`loopTask`) while read
   from the now-genuinely-concurrent `controlTask`.** This is real
   concurrency for the first time in this project - Phase 0-4 had one task
   calling itself through `loop()`, where none of this was a race. A
   `double` write on this hardware is two non-atomic word stores;
   `myPID.SetTunings()` writes several of `PID_v1`'s internal fields the
   same way while `myPID.Compute()` (inside `runPID()`, inside `controlTask`)
   reads them. The self-identified draft of this document already named
   this "the single most concrete new risk this phase introduces" but left
   it unfixed pending review; independent review agreed it belongs in this
   PR, not a follow-up, since this PR is what creates the exposure and the
   infrastructure to close it (`commandQueue`, `sendControlCommand()`) was
   already built by Phase 4 for exactly this purpose. Fixed by adding three
   `ControlCommand` types - `SET_TUNINGS`, `SET_STEAM_SETPOINT`,
   `SET_OFFSET` - applied only inside `applyControlCommand()` (the control
   owner, running in `controlTask`), so none of these globals are ever
   written from `loopTask` again. No new range-checking was added to
   `steamSetpoint`/`offset` beyond what they already had (none) - the fix
   here is the write's *location*, not new validation, which would be a
   separate, undiscussed behaviour change.

   This also closed a second, related gap in the same handler: `/saveConfig`
   used to *read* `pendingSettings.setpoint` (a `controlTask`-written
   `double`) and `PIDonly` as fallback values whenever the client's JSON
   omitted `"setpoint"`/`"PIDonly"`, to reconstruct a "no-op" command to
   resend. That read was itself racy, and review traced a concrete failure
   mode through it: a torn `double` read could reach
   `applyControlCommand()`'s `constrain()` as `NaN`, which passes every
   `constrain()` comparison unchanged (`NaN` fails both bounds checks),
   silently setting `setpoint` to `NaN` with no fault raised - the heater
   would stop responding with no visible cause. Fixed by not reconstructing
   a value at all: if the client's JSON omits a field, `/saveConfig` simply
   doesn't send a command for it, rather than resending the current value.
   This also fixed a latent, unrelated pre-existing bug in the same
   fallback expression for `steamSetpoint` (`(doc["steamSetpoint"] |
   steamSetpoint) + offset` re-added `offset` to an already-offset-adjusted
   value on every omission, silently drifting the steam target upward on
   repeated saves) - not something this phase introduced, but removed as a
   side effect of removing the fallback read entirely.

2. **Control-task creation failure was logged, then the firmware ran with
   no control loop and nothing watching it.** `xTaskCreatePinnedToCore()`'s
   return value was discarded in favor of checking whether the out-param
   handle was still `nullptr` - which only worked because the handle
   happens to be a zero-initialized global, not because of anything the API
   guarantees. More importantly: this task is the *only* thing that ever
   calls `controlStep()` from this phase on. If creation failed (e.g. heap
   exhaustion at the end of `setup()`, after Wi-Fi/SD/OTA/web-server
   allocations), nothing would ever read a sensor, evaluate a fault, or
   write an actuator again - while `/getValues` keeps answering 200 from
   its `buildTelemetrySnapshot()` fallback, so the machine looks alive and
   controlled while being neither, and the Task WDT never catches it either
   (it's watching a task that was never created). Fixed by checking the
   actual return code and calling `ESP.restart()` on failure instead of
   continuing into `loop()`.

3. **A failed `esp_task_wdt_add()` led to an unthrottled, unrate-limited
   `Serial.printf()` on every single 10 ms cycle, forever.** If subscription
   fails, `esp_task_wdt_reset()` fails the same way on every subsequent
   call (there's nothing to feed) - and the original code logged that
   failure unconditionally, every cycle: ~100 log lines/second, using a
   meaningful fraction of the UART link and adding real per-cycle overhead
   to the one task that must never be delayed. Fixed by latching the
   subscription outcome in `controlTaskWdtSubscribed`: `esp_task_wdt_reset()`
   is only called (and its own failure only logged) while still subscribed.

4. **Deadline-miss detection re-derived, via a second `xTaskGetTickCount()`
   call and manual arithmetic, exactly what `xTaskDelayUntil()`'s own return
   value already reports.** `xTaskDelayUntil()` returns `pdFALSE` precisely
   when the wake time was already due or past - i.e. a missed deadline - by
   definition. The original code computed the same condition a second, less
   precise way (whole-tick resolution vs. the delay call's own timing) and
   called `vTaskDelayUntil()` (the void-returning wrapper) instead of using
   the answer already available. Fixed by switching to `xTaskDelayUntil()`
   and checking its return value directly - one fewer syscall per cycle, and
   exact instead of approximate.

5. **`updatePumpRamp()`'s pressure-ramp gate used strict `>`, which never
   fires at exactly `PRESS_INTERVAL` (50 ms) on the new exact-10 ms grid.**
   `controlStep()` now runs on a fixed cadence, so `updatePumpRamp()`'s own
   `now = millis()` advances in clean ~10 ms steps relative to
   `DimlastUpdate`; `now - DimlastUpdate > 50` is never true at exactly
   +50 ms, so the gate didn't actually fire until +60 ms - a 20% cadence
   stretch, and (since `PUMP_ADJUST_INTERVAL`'s nested gate is only
   reachable from inside this one) the same stretch to 240 ms instead of
   200 ms for the pump-power ramp rate. Previously, `loop()`'s irregular,
   sub-millisecond cadence meant `now` almost never landed exactly on the
   50 ms boundary, so this was latent, not previously observable. Fixed by
   changing `>` to `>=`, matching every other interval gate in this file.

6. **The diagnostics report ran inside the control task itself, where it
   both perturbed the numbers it was reporting and was excluded from them.**
   Three separate, compounding problems, all from the same root cause:
   - `Print::printf()` uses a 64-byte stack buffer and falls back to
     `malloc()` + a second `vsnprintf()` for anything longer (verified
     against this project's actual `Print.cpp`); the report string is
     longer than that. A `malloc`/`free` pair on a real-time task's own
     stack, every 5 seconds, directly contradicted this same PR's own
     stack-sizing rationale ("no dynamic allocation").
   - `Serial.write()` can block once the UART TX FIFO fills, and shares a
     mutex with any other `Serial` output (including from `loopTask`) -
     another way the highest-priority task in the system could stall on a
     lower-priority one.
   - The measurement window (`cycleUs`) closed *before* the WDT feed, the
     deadline check and the report itself - so the report's own cost,
     whatever it was, was systematically invisible to the very metric it
     was printing.

   Fixed by moving the `Serial.printf()` call to `loop()`, which has none of
   these constraints; `controlTaskEntry()` only updates the counters now.
   `controlTaskWorstCycleUs` is also reset after each report (it wasn't
   before), so the printed number is a per-5s-window worst case instead of
   a single early spike that then reads as "still there" for the rest of
   uptime.

7. **Stack high-water mark was reported as "words"; ESP-IDF's
   `uxTaskGetStackHighWaterMark()` returns bytes, matching the bytes-based
   stack size this PR's own comment already correctly used for
   `CONTROL_TASK_STACK_SIZE`.** A 4x overstatement of real headroom if read
   as documented. Fixed by correcting the label; also fixed a second bug
   introduced by relocating the print to `loop()` (finding 6): calling
   `uxTaskGetStackHighWaterMark(NULL)` from `loop()` would report
   `loopTask`'s headroom, not `controlTask`'s - now passes
   `controlTaskHandle` explicitly.

8. **`handleGetValues()` read the raw `actime` global directly, unsynchronized
   against `controlTask`, and the value it read is stale by construction
   after any shot ends** (`actime` is only ever written inside `controlStep()`'s
   `if (acDetected)` branch and is never reset to 0 when a shot ends, unlike
   `TelemetrySnapshot.elapsedShotTimeMs`, which already does the right thing:
   `acDetected ? elapsedTime : 0`, published atomically with the rest of the
   snapshot this handler already peeks). Fixed by sourcing `actime` from
   `snap.elapsedShotTimeMs / 1000` instead of the raw global - removes the
   unsynchronized read and the staleness bug in the same change, using data
   already sitting in the snapshot this handler already reads.

9. **A duplicate forward-declaration block, with an inaccurate justification
   copied from the file's real one.** `void controlStep(unsigned long now);`
   was declared a second time, 300 lines from the existing "forward
   declarations" block this file already has for exactly this situation
   (and citing reasons - scoped enums, reference params - that don't apply
   to `controlStep()`'s plain signature). Fixed by moving the declaration
   into the existing block and correcting the comment to state the real
   reason (ctags-based prototype generation empirically doesn't reach
   across this particular gap either, confirmed by a failed build without
   it) instead of implying it matched the existing block's stated cases.

### Considered, not changed - needs bench verification, not a code fix guessed at blind

- **`digitalRead(syncPin)` sampling may alias against 50 Hz mains now that
  `controlStep()` runs on an exact, fixed 10 ms period.** Two independent
  review angles found this from different code paths. Mechanism: mains
  zero-crossings occur every 10 ms on 50 Hz supply (this project targets
  the Netherlands) - identical to `CONTROL_TASK_PERIOD_TICKS`. Before this
  phase, `loop()`'s irregular, sub-millisecond cadence meant successive
  `digitalRead(syncPin)` samples landed at effectively random phase
  relative to the mains waveform; `vTaskDelayUntil()`/`xTaskDelayUntil()`
  now locks the sample instant to one fixed phase point per boot, which
  only moves as the FreeRTOS tick and the AC grid frequency drift apart
  (independent, uncorrelated oscillators - a slow beat, not a permanent
  lock, but potentially minutes-long at a given phase). If that phase
  happens to land inside the zero-cross pulse window, `acOffSamples` could
  accumulate falsely and `endShot()` could fire mid-extraction with the
  paddle still engaged (`Discreet.ino`'s AC-off debounce path); if it lands
  outside the window, `acDetected` might never latch and a shot could fail
  to start. The AC-detect (shot-start) path is the more exposed of the two:
  it decides on a **single** sample with no debounce at all, unlike AC-off
  detection, which already requires `AC_OFF_MIN_SAMPLES` (20) consecutive
  samples over `AC_OFF_DEBOUNCE_MS` (300 ms) - though that debounce's
  margin is also thinner now (20 samples now costs exactly 200 ms of the
  300 ms budget at the fixed 100 Hz sample rate, versus a few ms when
  `loop()` ran at kHz rates).

  **Not fixed here.** Every code-level mitigation considered has a real
  cost or an unverifiable premise without real 50 Hz mains to test against:
  changing the task period to a value that isn't a mains harmonic only
  converts a possible permanent lock into a possible long transient lock,
  not a guarantee; sampling `syncPin` multiple times per cycle inside
  `controlStep()` adds deliberate blocking ahead of the safety-critical
  actuator write, which this project has been explicit about avoiding since
  Phase 3 (`steam()`'s reordering, `docs/PHASE3_NOTES.md`); and without
  knowing the actual zero-cross detector circuit's pulse width, any specific
  numeric choice is a guess this project's own discipline (see
  `docs/PHASE1_NOTES.md`'s `TODO(bench)` markers on these exact constants)
  says shouldn't be made blind. Written down here as the **first thing to
  verify on real hardware** before trusting this phase's shot-detection
  behavior, ahead of the timing/stack items every prior phase has also
  needed bench time for.

### Documented, not fixed - deliberately out of this phase's scope

- **`Kp`/`Ki`/`Kd`/`steamSetpoint`/`offset` are still read directly (not via
  `telemetryQueue`) by `handleGetValues()`/`handleTemp()` for display.**
  This phase fixed the *write*-side race (finding 1) by making
  `controlTask` their exclusive writer; these remaining reads are
  display-only, can still tear, and will show an occasionally-wrong value
  for exactly one HTTP response that self-corrects on the next poll - no
  actuator or safety consequence, the same category Phase 4 §4 already
  accepted for these same fields. Routing them through
  `TelemetrySnapshot`/`telemetryQueue` too is a reasonable follow-up but is
  a `TelemetrySnapshot`-widening change beyond "fix the concurrency race,"
  not done here.
- **`brewTemp` is declared, read in `handleGetValues()`, and never written
  anywhere in this file** (found in review) - pre-existing, not something
  this phase touched or introduced; `/getValues` has always shipped an
  empty string for it. Left alone as unrelated cleanup, worth a one-line
  removal whenever a future phase is next in this area.
