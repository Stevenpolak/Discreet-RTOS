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

## What changed

### 1. `heaterDemand`: bool → the raw PID output (0-255)

`runPID()` used to reduce `myPID.Compute()`'s 0-255 `output` to a single
bit (`heaterDemand = (output >= 127)`) before `controlStep()` ever saw it.
`heaterDemand` is now the PID output itself; `controlStep()` is where the
on/off decision is made, using the full resolution, not a fixed threshold.

### 2. Time-proportional window

`controlStep()` converts `heaterOut` (the fault/mode-resolved demand) into
an on-time fraction of a fixed `SSR_WINDOW_MS` (1000ms) window: `onTimeMs =
(heaterOut / 255.0) * SSR_WINDOW_MS`, sampled once at the start of each
window and held fixed for its duration (standard time-proportional-control
practice - avoids fragmenting one on-pulse into several as the PID output
drifts cycle to cycle). `heaterOut == 0` (fault, safety-off, or the PID
genuinely asking for no heat) forces the window to reset immediately rather
than letting an already-locked-in on-time keep authorizing heat for the
rest of that window - the same "abort, don't mask" principle Phase 3
applied to the shot state machine, applied here to the window itself.

### 3. The independent deadman

`controlStep()` no longer calls `digitalWrite(SSR_PIN, ...)` at all. It
only ever writes two globals - `ssrAuthorizedOn` (the on/off decision above)
and `ssrAuthorizedUntilMs` (`now + SSR_AUTH_TIMEOUT_MS`, refreshed every
cycle, i.e. every 10ms, not just once per window) - the "authorization."

`ssrDeadmanCallback()` is the *only* thing that ever writes `SSR_PIN`. It
runs as its own `esp_timer` (`ESP_TIMER_TASK` dispatch, matching this
file's existing `buzzerTimer` pattern), a genuinely separate FreeRTOS task
at a fixed high priority, on its own `SSR_ENFORCE_INTERVAL_MS` (20ms)
period - entirely independent of whether `controlTask` is still running.
Each tick: if the authorization hasn't expired, drive the pin to the
authorized value; if it has (or was never set), drive it low. A control
task that stops refreshing the authorization - wedged, crashed, starved -
loses heater authority within `SSR_AUTH_TIMEOUT_MS` (200ms) of its last
successful cycle, independent of the Task WDT's own (5s default) recovery
window.

`ssrAuthorizedOn`/`ssrAuthorizedUntilMs` are plain globals, not routed
through a queue: both are single-word types (`bool`, `unsigned long`),
atomic to read/write on this hardware, following the same reasoning
Phase 5's review applied to `offset`/`acDetected`-class globals.
`ssrAuthorizedOn` is written before `ssrAuthorizedUntilMs` on every refresh
so a reader can at worst see a stale on/off value paired with a fresh
deadline for one `SSR_ENFORCE_INTERVAL_MS` tick - bounded, self-correcting,
and never weakens the actual safety property (an expired deadline always
means off, regardless of `ssrAuthorizedOn`).

**Why task dispatch, not ISR dispatch:** `esp_timer` supports a true
interrupt-context dispatch mode (`ESP_TIMER_ISR`), which would be immune to
FreeRTOS task scheduling entirely. Considered and not used: it adds real
interrupt-context constraints (no blocking calls, careful use of ISR-safe
APIs) that are hard to get right without hardware to test against, for a
narrower additional guarantee than it first appears - an ESP32 flash write/
erase (e.g. during OTA) disables interrupts on *both* cores for its
duration regardless of dispatch method (`docs/PHASE5_NOTES.md` §2), so ISR
dispatch would not protect against that specific, already-documented gap
either. Task dispatch at a high fixed priority (`ESP_TASK_TIMER_PRIO`, well
above `controlTask`'s priority 5) was judged the better cost/benefit.

### 4. Clearing the authorization on OTA start

`ArduinoOTA.onStart()` now sets `ssrAuthorizedUntilMs = 0`, forcing the
deadman to treat the authorization as already-expired the instant an OTA
session begins - independent of whether `controlTask` keeps running through
it (Phase 5's notes already document that a flash erase/program can stall
it regardless of this).

### 5. Boot-time default

`initSsrDeadman()` (called from `setup()`, right after `pinMode(SSR_PIN,
OUTPUT)`, before anything else that could plausibly want the heater on)
writes `SSR_PIN` low *before* creating or starting the timer, so the pin is
in a known-safe state even in the narrow window before the deadman's first
tick. `ssrAuthorizedUntilMs`'s zero-initialized default also already reads
as "expired" the moment `millis()` is nonzero, which is true well before
`initSsrDeadman()` even runs - belt and suspenders, not strictly required
by the initialization order, but cheap and explicit.

## What was intentionally *not* done in this phase

- **PID retuning.** `REWRITE_PLAN.md` explicitly says to validate heater
  response and retune only if measurements show it's necessary - not done
  here, no bench data exists yet. The existing `Kp`/`Ki`/`Kd` defaults are
  unchanged; time-proportional control with the same tunings may behave
  differently from the old binary threshold (in particular, near the
  setpoint the heater will now cycle at fractional duty within each 1s
  window instead of chattering fully on/off at the 127 boundary) - this is
  exactly the kind of thing the plan's own "validate ... only if necessary"
  guidance is for.
- **`SSR_WINDOW_MS`/`SSR_AUTH_TIMEOUT_MS` tuning.** Both are `TODO(bench)`
  judgement calls, matching this file's existing convention for constants
  that need real hardware to validate (`AC_OFF_DEBOUNCE_MS` etc., see
  `docs/PHASE1_NOTES.md`).
- **The pump/dimmer path is completely untouched.** `REWRITE_PLAN.md`
  titles this phase "time-proportional *SSR* output" - the pump's own ramp
  logic (`updatePumpRamp()`, `light.setBrightness()`) was never in scope.
- **The Task WDT's own timeout is still untouched** (Phase 5 left this at
  arduino-esp32's 5s default) - this phase adds a second, independent,
  faster (200ms) safety net specifically for the heater, but does not
  change the WDT's role of recovering the whole system from a genuinely
  wedged control task.

## Build verification

Compiled with the same toolchain as Phase 0-5 (`esp32:esp32@2.0.17`):

```
Sketch uses 908637 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51340 bytes (15%) of dynamic memory, leaving 276340 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +584 bytes flash / +32 bytes RAM versus
`v2.0.4-beta` (908,053 / 51,308) - the new deadman timer, its callback, and
the window-tracking state.

## Behaviour-preservation checklist

- [x] `heaterDemand` carries the full PID resolution, not a fixed threshold
      - `runPID()` now assigns `output` directly.
- [x] Every window authorization expires automatically without a refresh -
      `ssrDeadmanCallback()` defaults to off whenever `ssrAuthorizedUntilMs`
      has passed, unconditionally.
- [x] The control task re-arms/refreshes every successful cycle (10ms), not
      just once per window (1000ms) - strictly more frequent than the
      plan's literal ask.
- [x] Expiry is enforced in a path independent of the control task -
      `ssrDeadmanCallback()` runs in the `esp_timer` service task, a
      separate FreeRTOS task from `controlTask`.
- [x] The authorization is never re-armed before this cycle's fault/mode
      resolution has run - the refresh happens at the same point in
      `controlStep()` the old direct `digitalWrite(SSR_PIN, ...)` used to.
- [x] The authorization clears immediately on fault (heaterOut forced to 0
      by the existing fault-priority resolution → window reset → off),
      mode transition (temp-only never sets a nonzero heaterDemand path
      differently than normal mode - this phase didn't change that
      coupling), and OTA start (`ArduinoOTA.onStart()`, §4).
- [x] A stale timestamp, missed refresh, or an authorization that was never
      set (`ssrAuthorizedUntilMs` at its zero-initialized default) all map
      to off through the same single check in `ssrDeadmanCallback()`.
- [ ] **Bench re-test required, same hardware limitation as every prior
      phase.** This phase specifically needs:
      1. Confirm actual heater behavior under time-proportional control -
         does the existing PID tuning produce reasonable window duty
         cycles, or does it need retuning (see "What was intentionally not
         done")?
      2. **Validate the deadman itself**, per `REWRITE_PLAN.md`:
         deliberately stop `controlTask` from refreshing the authorization
         (e.g. a temporary debug hook that stops calling `controlStep()`)
         and measure the actual time until `SSR_PIN` goes low - should be
         close to `SSR_AUTH_TIMEOUT_MS` + one `SSR_ENFORCE_INTERVAL_MS`
         tick, not the Task WDT's 5s.
      3. Confirm `SSR_WINDOW_MS`/`SSR_AUTH_TIMEOUT_MS` are appropriate for
         the actual heater/SSR hardware - both are `TODO(bench)`.
      4. Repeat Phase 5's OTA-during-a-simulated-shot stress test and
         confirm the SSR authorization is cleared immediately at OTA start,
         not just eventually timing out.
