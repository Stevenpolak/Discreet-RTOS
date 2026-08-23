# Phase 6 notes — time-proportional SSR output

Date: 2026-08-23
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 6, applied on top of the
merged Phase 0-5 work (`v2.0.4-beta`, see [BASELINE.md](BASELINE.md) through
[PHASE5_NOTES.md](PHASE5_NOTES.md)).

Goal per `REWRITE_PLAN.md`: use the PID's full 0-255 output resolution
instead of a fixed `output >= 127` on/off threshold, and give the heater
output a deadman - "a short-lived authorization, not a persistent level" -
enforced independently of the control task, so a wedged/stalled control
task cannot leave the SSR continuously energized.

**This section describes the design after review round 1's fixes.** The
review found several real defects in the first draft, some safety-relevant;
see "Review round 1" below for what changed and why. Read this section as
the final design, not the starting point.

## What changed

### 1. `heaterDemand`: bool → the raw PID output (0-255)

`runPID()` used to reduce `myPID.Compute()`'s 0-255 `output` to a single
bit (`heaterDemand = (output >= 127)`) before `controlStep()` ever saw it.
`heaterDemand` is now the PID output itself; `controlStep()` is where the
on/off decision is made, using the full resolution, not a fixed threshold.

### 2. Time-proportional window

`controlStep()` converts `heaterOut` (the fault/OTA/mode-resolved demand)
into an on-time fraction of a fixed `SSR_WINDOW_MS` (1000ms) window,
sampled once at the start of each window and held fixed for its duration
(standard time-proportional-control practice - avoids fragmenting one
on-pulse into several as the PID output drifts cycle to cycle). The window
itself keeps rolling on its own schedule regardless of demand; a separate,
NaN-safe gate (`heaterOut > 0`) is what actually decides whether heat is
authorized on any given cycle. See "Review round 1" #1 for why an earlier
version that reset the window on every zero-demand cycle was wrong, and
why decoupling the gate from the window bookkeeping fixes it.

### 3. The independent deadman

`controlStep()` no longer calls `digitalWrite(SSR_PIN, ...)` at all. It
only ever writes one global - `ssrAuthorizedUntilMs`, a single deadline:
`now + SSR_AUTH_TIMEOUT_MS` while heat should be on, or `now` (not a fixed
sentinel - see "Review round 1" #3) while it shouldn't. Refreshed every
`controlStep()` cycle (10ms), not just once per window.

`ssrDeadmanCallback()` is the *only* thing that ever writes `SSR_PIN`. It
runs as its own `esp_timer` (`ESP_TIMER_TASK` dispatch, matching this
file's existing `buzzerTimer` pattern), a genuinely separate FreeRTOS task,
at a fixed priority arithmetically confirmed as 22 on this target
(`ESP_TASK_TIMER_PRIO` = `configMAX_PRIORITIES`(25) - 3), well above
`controlTask`'s 5 - on its own `SSR_ENFORCE_INTERVAL_MS` (10ms) period,
independent of whether `controlTask` is still running. Each tick: if the
deadline is still in the future, drive the pin high; otherwise (expired,
or never set) drive it low. A control task that stops refreshing the
authorization - wedged, crashed, starved - loses heater authority within
`SSR_AUTH_TIMEOUT_MS` (200ms) of its last successful cycle, independent of
the Task WDT's own (5s default) recovery window.

`ssrAuthorizedUntilMs` is a plain global, not routed through a queue: a
single word (`unsigned long`), atomic to read/write on this hardware,
following the same reasoning Phase 5's review applied to `offset`/
`acDetected`-class globals. Collapsing what was originally two fields
(a separate on/off bool plus the deadline) into this one removed a
load-bearing but compiler-unenforced write-ordering requirement - see
"Review round 1" #2.

**Why task dispatch, not ISR dispatch:** `esp_timer` supports a true
interrupt-context dispatch mode (`ESP_TIMER_ISR`), which would be immune to
FreeRTOS task scheduling entirely. Considered and not used: it adds real
interrupt-context constraints (no blocking calls, careful use of ISR-safe
APIs) that are hard to get right without hardware to test against, for a
narrower additional guarantee than it first appears - an ESP32 flash write/
erase (e.g. during OTA) disables interrupts on *both* cores for its
duration regardless of dispatch method (`docs/PHASE5_NOTES.md` §2), so ISR
dispatch would not protect against that specific, already-documented gap
either. Task dispatch at a high fixed priority was judged the better
cost/benefit - though review found (and fixed) a real consequence of that
choice: this task is shared with the pre-existing `buzzerTimerCallback()`,
which could previously block it indefinitely. See "Review round 1" #4.

### 4. Clearing the authorization on OTA start

A latched `otaInProgress` flag (not a one-shot write to
`ssrAuthorizedUntilMs` - see "Review round 1" #5 for why that doesn't
actually work) is set by `ArduinoOTA.onStart()` and cleared by both
`onEnd()` (success) and `onError()` (abort/failure, so a failed OTA
doesn't permanently disable the heater). `controlStep()` checks it every
cycle, in the same fault-priority resolution that already forces
`heaterOut` to 0 on a fault - so the heater is held off for the *whole*
OTA session, not just the instant it starts.

### 5. Boot-time default

`pinMode(SSR_PIN, OUTPUT)` and `initSsrDeadman()` are now the very first
hardware actions in `setup()`, before even the initial boot delay - not
after WiFi/SD/OTA setup, which on a real network can take tens of seconds.
See "Review round 1" #6 for why this matters (GPIO13/MTCK floats on any
reset that doesn't power-cycle the board). `initSsrDeadman()` writes
`SSR_PIN` low *before* creating or starting the timer, so the pin is in a
known-safe state even in the few instructions before the deadman's first
tick.

## What was intentionally *not* done in this phase

- **PID retuning.** `REWRITE_PLAN.md` explicitly says to validate heater
  response and retune only if measurements show it's necessary - not done
  here, no bench data exists yet. The existing `Kp`/`Ki`/`Kd` defaults are
  unchanged; time-proportional control with the same tunings may behave
  differently from the old binary threshold - this is exactly the kind of
  thing the plan's own "validate ... only if necessary" guidance is for.
- **`SSR_WINDOW_MS`/`SSR_AUTH_TIMEOUT_MS` tuning.** Both are `TODO(bench)`
  judgement calls, matching this file's existing convention for constants
  that need real hardware to validate.
- **The pump/dimmer path is completely untouched**, including during OTA -
  `REWRITE_PLAN.md` titles this phase "time-proportional *SSR* output," and
  gating the pump on OTA start would be a second, undiscussed behaviour
  change beyond what the plan asked for. See "Review round 1" #5.
- **The Task WDT's own timeout is still untouched** (Phase 5 left this at
  arduino-esp32's 5s default) - this phase adds a second, independent,
  faster (200ms) safety net specifically for the heater, but does not
  change the WDT's role of recovering the whole system from a genuinely
  wedged control task.
- **True hardware-only (ISR/timer-peripheral) enforcement.** The deadman is
  a genuinely separate FreeRTOS task, not the control task, but it is still
  a software mechanism running under the same scheduler - see "Review round
  1" #7 for an honest accounting of what that does and doesn't protect
  against, and why a stronger mechanism wasn't attempted here.
- **`SSR_ENFORCE_INTERVAL_MS`'s effect on achievable output resolution was
  not fully solved**, only improved (20ms → 10ms) and honestly documented -
  see "Review round 1" #8. A thermal system's time constant (seconds+)
  likely makes this immaterial, but that's a bench question, not something
  asserted here.

## Build verification

Compiled with the same toolchain as Phase 0-5 (`esp32:esp32@2.0.17`), after
the review-round-1 fixes below:

```
Sketch uses 909177 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51340 bytes (15%) of dynamic memory, leaving 276340 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +540 bytes flash versus the pre-review-fixes
commit (908,637), RAM unchanged (51,340) - net of removing `ssrAuthorizedOn`
and adding `otaInProgress`/`ssrDeadmanLastRunMs`, plus the `static_assert`
and integer-math changes. +1,124 bytes flash / +32 bytes RAM versus
`v2.0.4-beta` (908,053 / 51,308) overall.

## Behaviour-preservation checklist

- [x] `heaterDemand` carries the full PID resolution, not a fixed threshold.
- [x] Every window authorization expires automatically without a refresh -
      `ssrDeadmanCallback()` defaults to off whenever `ssrAuthorizedUntilMs`
      is not in the future, unconditionally, using a wraparound-safe
      comparison against `now` rather than a fixed sentinel.
- [x] The control task re-arms/refreshes every successful cycle (10ms), not
      just once per window (1000ms).
- [x] Expiry is enforced in a path independent of the control task, with a
      heartbeat (`ssrDeadmanLastRunMs`) so the *absence* of that path
      running is itself observable.
- [x] The authorization is never re-armed before this cycle's fault/OTA/mode
      resolution has run.
- [x] The authorization clears immediately on fault (heaterOut forced to 0,
      NaN-safe), mode transition (unchanged coupling from prior phases), and
      OTA start, held for the *whole* session via a latched flag, not a
      one-shot write.
- [x] A stale timestamp, missed refresh, or an authorization that was never
      set all map to off through the same single comparison.
- [ ] **Bench re-test required, same hardware limitation as every prior
      phase.** This phase specifically needs:
      1. Confirm actual heater behavior under time-proportional control -
         does the existing PID tuning produce reasonable window duty
         cycles, or does it need retuning?
      2. **Validate the deadman itself**, per `REWRITE_PLAN.md`:
         deliberately stop `controlTask` from refreshing the authorization
         and measure the actual time until `SSR_PIN` goes low.
      3. Confirm `SSR_WINDOW_MS`/`SSR_AUTH_TIMEOUT_MS`/`SSR_ENFORCE_INTERVAL_MS`
         are appropriate for the actual heater/SSR hardware.
      4. Repeat Phase 5's OTA-during-a-simulated-shot stress test and
         confirm the SSR authorization is held off for the *entire* OTA
         session now, not just eventually timing out.
      5. Confirm the low-duty/high-duty quantization described in "Review
         round 1" #8 is or isn't visible/significant on real hardware.

## Review round 1: findings and fixes

7-angle independent review found the first draft had several real defects,
some safety-relevant, concentrated in the window-reset logic and the OTA
hook. All were fixed before merge; none were deferred as "considered, not
changed" the way Phase 5 deferred its mains-sampling question - every
finding here had a code-level fix that didn't require bench hardware to
apply with confidence.

### Fixed

1. **The window-reset branch could starve the heater indefinitely.** The
   first draft reset `ssrWindowStartMs`/zeroed `ssrWindowOnTimeMs` on *every*
   cycle `heaterOut` was 0 - modeled on "abort, don't mask," by analogy with
   `endShot()`. But `heaterOut` reaching exactly 0 is routine PID behaviour
   at/above setpoint (`SetOutputLimits(0,255)` saturates there), not only a
   fault condition. Resetting the window every such cycle meant that on the
   very next cycle demand returned, the window-rollover gate (`now -
   ssrWindowStartMs >= SSR_WINDOW_MS`) was never true (only ~10ms had
   passed), so the on-time was never recomputed and stayed 0 - the heater
   got authorized for **0% of the next full second**, every time. Three
   independent review angles traced this to a worse case: if the PID output
   touches 0 more than once per second (plausible hunting behaviour near
   setpoint), the window-rollover gate could never fire again at all,
   permanently freezing `ssrWindowOnTimeMs` at 0 while the PID integral
   wound up against a heater silently delivering zero power - with no fault
   raised and telemetry reporting normally. Fixed by decoupling the two
   questions: the window rolls on its own fixed schedule regardless of
   demand (so it always eventually samples a fresh on-time), and a separate
   gate (`heaterOut > 0`) decides per-cycle whether heat is authorized -
   which still forces off on the very same cycle a fault is detected,
   without needing to also destroy the window's bookkeeping to do it.

2. **The original two-field authorization (`ssrAuthorizedOn` + a deadline)
   had a write-ordering requirement nothing enforced.** The design relied on
   `ssrAuthorizedOn` being written before `ssrAuthorizedUntilMs` so a torn
   cross-task read could only ever see a stale-but-safe combination -
   correct, but unenforceable by the compiler, and one accidental reorder of
   two adjacent lines away from being wrong. Collapsed to the single
   `ssrAuthorizedUntilMs` deadline described in §3: "on until T" and "off"
   (not in the future) are both fully expressed by one value, so there is no
   second field to get out of sync with the first.

3. **The OTA-start "clear" used `0` as an "already expired" sentinel, which
   is not wraparound-safe.** `ssrDeadmanCallback()`'s comparison,
   `(long)(ssrAuthorizedUntilMs - now) > 0`, is only valid for deadlines
   within ±2^31ms of `now`. With the deadline set to a literal `0`, once
   `now` exceeds 2^31ms (~24.86 days of uptime - not unusual for an always-on
   WiFi appliance), the unsigned subtraction wraps and the signed cast reads
   as **positive** - "authorized approximately 24 days from now." An OTA
   started after that much uptime, with the heater already authorized on,
   could have latched the deadman into holding `SSR_PIN` high indefinitely
   if `controlTask` then stalled during the flash write - the exact
   scenario the hook exists to protect against, inverted into its opposite.
   Fixed as a consequence of #2 and #5: `controlStep()` now only ever writes
   `now` (never a fixed literal) as the "off" case, which is always
   wraparound-safe relative to itself, and the OTA hook no longer touches
   `ssrAuthorizedUntilMs` directly at all (see #5).

4. **The deadman shared the esp_timer service task with the pre-existing
   buzzer, which could block it indefinitely.** `esp_timer`'s `ESP_TIMER_TASK`
   dispatch runs every callback using that mode serially, in one task.
   `buzzerTimerCallback()` opened with `xSemaphoreTake(buzzerMutex,
   portMAX_DELAY)`, and `queueBuzzer()` (reachable from `controlStep()` via
   `steam()`) takes the same mutex. Three independent review angles traced
   the same failure mode: if `controlTask` wedged, crashed, or was deleted
   while holding `buzzerMutex` inside `queueBuzzer()`, and the buzzer's
   one-shot timer then fired, `buzzerTimerCallback()` would block forever
   waiting for a mutex nothing would ever release - and because `esp_timer`
   dispatches serially, `ssrDeadmanCallback()` would never run again either,
   directly defeating this phase's central claim ("independent of the
   control task") in exactly the scenario it exists for. Fixed by bounding
   `buzzerTimerCallback()`'s wait (`BUZZER_MUTEX_WAIT_TICKS`, 20ms) instead
   of `portMAX_DELAY`; on timeout it simply skips that tick's work rather
   than touching buzzer state without the mutex. `queueBuzzer()`'s own wait
   (called from `controlTask`/`setup()`, never from the shared esp_timer
   task) was deliberately left unbounded - a stall there is exactly the
   "controlTask wedged" case the deadman is designed to catch, not a threat
   to the deadman's own task.

5. **The OTA-start authorization clear was a no-op in the case that
   mattered.** `ArduinoOTA.onStart()` wrote `ssrAuthorizedUntilMs = 0`
   directly (from `loopTask`), but nothing stopped `controlTask` from
   continuing to run normally during an OTA upload (network transfer, not
   yet the flash write) - and `controlStep()` unconditionally re-authorizes
   every 10ms. Four independent review angles confirmed: the cleared
   deadline survived at most one control cycle before being overwritten
   with a fresh authorization, so the heater kept heating normally through
   an entire OTA session unless/until the flash write itself happened to
   stall `controlTask` - i.e. exactly the behaviour that would have occurred
   with no hook at all. Fixed with the latched `otaInProgress` flag
   described in §4: `controlStep()`'s own priority-resolution chain now
   honors it every cycle, the same shape as the pre-existing `faulted`
   check, rather than a one-shot write to derived state that the next cycle
   simply overwrites.

6. **`SSR_PIN` was left floating for the majority of boot.** `pinMode`/
   `initSsrDeadman()` originally ran after `loadSDConfig()`, `startWiFi()`
   (which retries for up to ~20s), `startSD()`, and `ArduinoOTA.begin()` -
   tens of seconds into `setup()`. GPIO13 (this board's `SSR_PIN`) is also
   MTCK and reverts to a floating input on any reset that doesn't
   power-cycle the board - which includes every recovery path this project
   has added: the Task WDT (Phase 5), and both `ESP.restart()` calls this
   file now has. A warm reset while the heater was energized would have left
   the pin undriven, with no deadman yet running to help, for that entire
   window. Fixed by moving `pinMode(SSR_PIN, OUTPUT)`/`initSsrDeadman()` to
   the very first hardware actions in `setup()`, before even the initial
   boot delay.

7. **A `heaterOut <= 0` check is not NaN-safe, and silently defaults to the
   *wrong* direction.** Every comparison against NaN is false under IEEE
   754, so `heaterOut <= 0` being false for NaN would let a non-finite PID
   output fall through to the branch that authorizes heat - the opposite of
   this file's established fail-safe convention (`runPID()`'s own sensor
   check treats any non-finite reading as an immediate fault). Not
   reachable through any currently-validated path (`PID_v1` clamps its own
   output, and JSON can't encode NaN), but `SET_TUNINGS`/`SET_OFFSET`
   (Phase 5) are explicitly unvalidated by design, so a corrupted or
   hand-edited config value could in principle reach this. Fixed by
   inverting the gate to `heaterOut > 0`, which is false (correctly
   "no heat") for NaN.

8. **Output resolution and worst-case fault-to-pin latency were both worse
   than the phase's framing implied, and not disclosed.** `SSR_ENFORCE_
   INTERVAL_MS` (originally 20ms) is not just the deadman's poll rate; it's
   also the actual ceiling on achievable on-time resolution (255 PID levels
   collapse onto `SSR_WINDOW_MS / SSR_ENFORCE_INTERVAL_MS` = 50 achievable
   duty steps, not the full 8-bit range this phase's own checklist item
   claims), and it sets the worst-case latency between a fault being
   detected and the physical pin actually going low (previously 0ms,
   synchronous within `controlStep()`; now bounded by this constant plus
   task-dispatch jitter). Tightened from 20ms to 10ms (matching
   `controlStep()`'s own cycle time) as a cheap, well-justified improvement
   to both, and the resolution ceiling is now stated honestly above (§3)
   and in `REWRITE_PLAN.md`'s checklist rather than left implicit. Not fully
   solved - see "What was intentionally not done."

### Considered, not changed

- **`initSsrDeadman()`/`initBuzzer()` share near-identical `esp_timer`
  creation boilerplate**, and could share a small helper. Left as
  duplication (matching the buzzer's own existing hand-written pattern) -
  a real, low-severity cleanup opportunity, not done here to keep this PR's
  diff focused on the safety-relevant fixes above.
- **Restarting on `initSsrDeadman()` failure, rather than logging and
  continuing** (the convention `initBuzzer()`/queue creation use): a failed
  deadman is fail-safe in itself (`SSR_PIN` is already driven low before
  either `esp_timer` call is attempted), unlike a failed control-task
  creation, which is unsafe-and-invisible (Phase 5's rationale for
  restarting there). Kept the restart anyway: a machine that can never heat
  again until reboot is as non-functional as one with no control loop, and
  a third convention (log-and-continue specifically for actuator-safety
  infrastructure) seemed worse than being consistent with Phase 5's choice.
- **True hardware-only enforcement** (a dedicated hardware timer-group
  peripheral with its own ISR, entirely outside FreeRTOS scheduling) would
  be a stronger guarantee than a task-dispatch `esp_timer`, which is still,
  ultimately, a software mechanism the scheduler must run. Not attempted:
  meaningfully more code and interrupt-context risk to get right without
  hardware to validate against, for a gap (the scheduler itself failing to
  schedule anything) that would likely take down the whole system anyway,
  not just this one safety path.
- **The esp_timer service task's core affinity was not verified** the way
  Phase 5 verified `controlTask`'s against this project's actual
  `sdkconfig` - its priority was confirmed by header arithmetic
  (`ESP_TASK_TIMER_PRIO` = 22), but the task itself is created inside a
  precompiled library (`libesp_timer.a`) this environment has no source
  access to, so its core placement is asserted from general ESP-IDF
  knowledge, not confirmed the rigorous way. Worth confirming on real
  hardware or from IDF source directly in a future pass.
