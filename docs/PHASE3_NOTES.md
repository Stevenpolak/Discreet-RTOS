# Phase 3 notes — centralize control ownership

Date: 2026-08-23
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 3, applied on top of the
merged Phase 0-2 work (`v2.0.1-beta`, see [BASELINE.md](BASELINE.md),
[PHASE1_NOTES.md](PHASE1_NOTES.md), [PHASE2_NOTES.md](PHASE2_NOTES.md)).

Goal per `REWRITE_PLAN.md`: a single function owns all control decisions
and both actuator outputs, **while still running in the original execution
context**. No FreeRTOS task, queue, or task boundary yet — that's Phase 4/5.

## What changed

### 1. `controlStep(now)` — the single actuator-writing boundary

Before this PR, `digitalWrite(SSR_PIN, ...)` was called from 2 places
(`runPID()`) and `light.setBrightness(...)` from 8 places (`SetPump()` and
six sites across the pre-infusion/bloom/extraction branches in `loop()`).
Every function that used to write an actuator directly now only sets one
of two module-level demand variables instead:

- `heaterDemand` (bool) — set by `runPID()`, which still owns the PID
  computation and the temperature-fault check exactly as before, just no
  longer writes `SSR_PIN` itself.
- `pumpDemand` (int, 0-255) — set by the shot-phase branches (directly, for
  the immediate full-power cases) and by `updatePumpRamp()` (renamed from
  `SetPump()`, since it no longer sets anything actuator-facing — it's the
  pressure-regulation ramp, unchanged internally).

`controlStep(now)` runs the existing sequence in the existing order
(`GetPressure()`, `runPID()`, `steam()`, AC detection, the shot state
machine, the AC-off debounce) exactly as `loop()` did before, then adds one
new block at the end: output-priority resolution, followed by exactly one
`digitalWrite(SSR_PIN, ...)` and one `light.setBrightness(...)` call. Those
two lines are now the *only* places in the entire file that touch either
actuator — verified by grepping the whole file for both call patterns.

`loop()` is now three lines: `ArduinoOTA.handle()`, `server.handleClient()`,
`controlStep(millis())`.

### 2. Why `controlStep(now)` takes a timestamp parameter

`REWRITE_PLAN.md` names this boundary `controlStep(now)` specifically, and
that shape matters for Phase 5: the dedicated FreeRTOS task will call this
once per `vTaskDelayUntil()` cycle with a single sampled `now`, not let the
function re-read `millis()` at several different points within one logical
cycle. This PR moves `controlStep()`'s own direct time arithmetic (the
AC-detect timestamp, the AC-off debounce) onto the passed-in `now`, ahead
of when it's actually needed for scheduling — this is pure preparation, not
required for Phase 3's own exit condition, but free to do now since it's
still just a single value threaded through the same code path.

**Not changed:** `GetPressure()`, `runPID()` and `updatePumpRamp()` still
call `millis()` themselves for their own internal interval gates
(`PRESS_INTERVAL`, `PID_INTERVAL`, `PUMP_ADJUST_INTERVAL`). Threading `now`
through those too would touch more call sites for a Phase 3 checklist that
only names the top-level boundary, and changing their internal gating
without a bench to verify against felt like unnecessary risk for this pass.

### 3. Output priority resolution — the actual new behaviour

This is where Phase 3 does more than relabel existing code. Per
`REWRITE_PLAN.md`: "Apply output priority: fault/safety off, mode
restrictions, state-machine demand, controller output."

```cpp
bool heaterOut = heaterDemand;
int pumpOut = pumpDemand;

if (currentFault != FaultCode::NONE) {
  heaterOut = false;
  pumpOut = 0;
}

if (PIDonly) {
  pumpOut = 0;
}

digitalWrite(SSR_PIN, heaterOut ? HIGH : LOW);
light.setBrightness(pumpOut);
```

**Fault/safety off (checklist: "verify that fault handling always forces
safe outputs").** Before this PR, a temperature fault only ever forced the
SSR off — that logic lived inside `runPID()` and had no coupling to the
pump at all. If a fault occurred while a shot happened to be in progress,
the pump kept running on the shot state machine's demand as if nothing
were wrong; there was no code path connecting `currentFault` to
`pumpDemand`. This is the actual fix: the fault check is now the first
thing evaluated after all demands are computed, and it unconditionally
overrides both outputs, every single `controlStep()` cycle for as long as
the fault persists — not just once when first detected. Nothing after this
block in the function can un-set it.

**Mode restrictions (checklist: "verify that temp-only cannot enable the
pump").** This one already held structurally before this PR, as a side
effect of AC detection being gated on `!PIDonly` (`if (digitalRead(syncPin)
== LOW && !acDetected && !PIDonly)`) — a shot can never start while in
temp-only mode, so `pumpDemand` could never become nonzero that way. But
that's an *indirect* guarantee with one gap: nothing stopped `PIDonly` from
being flipped to `true` via `/saveConfig` while a shot was **already**
in progress (`acDetected` already `true` from before the mode change) — the
running shot would just continue, pump and all, ignoring the newly-set
mode. The explicit `if (PIDonly) pumpOut = 0;` layer closes that gap: even
mid-shot, flipping to temp-only now forces the pump off on the very next
cycle. This is a genuine, safety-positive behaviour change, not just
restating what already held.

### 4. "Route web changes through validated requests instead of direct
global-variable or actuator writes"

Already satisfied going into this PR: Phase 2's `ControlCommand` /
`applyControlCommand()` already routes every settings change from
`handleAdjust()` and `/saveConfig` through one validated path, and Pause/
Resume (the only web-triggered direct actuator writes that ever existed)
were removed in that same phase. Verified for this PR by grepping every
assignment to `pumppower`, `pumpDemand` and `heaterDemand`: all of them are
inside `controlStep()`'s own call chain (`runPID()`, `updatePumpRamp()`,
the shot-phase branches) except the two `pumppower = ...` writes inside
`applyControlCommand()`'s idle-edit preview (a pre-existing quirk from the
original firmware, `if (idle) pumppower = basePumpPowerForSetpoint(...)`) -
those only ever update the *display* value; they cannot reach an actuator,
since `pumpDemand` defaults to `0` whenever idle and is only ever set
nonzero from inside the `if (acDetected)` shot-phase branch, which can't
run while idle.

## What was intentionally *not* done in this phase

- No FreeRTOS task, queue, or `vTaskDelayUntil()` periodic cycle — Phase 3's
  own exit condition explicitly keeps this in the original (single `loop()`)
  execution context.
- `GetPressure()`/`runPID()`/`updatePumpRamp()`'s internal timing gates were
  not threaded onto `now` — see §2.
- Kp/Ki/Kd/offset/steamSetpoint/PIDonly still bypass `ControlCommand` and
  are written directly from `/saveConfig` — an intentional Phase 2 scope
  decision (they are not actuator writes and were never in scope for the
  settings latch), not revisited here.
- No new fault *detection* (invalid pressure, over-temperature independent
  of the sensor-sanity bound, time-outs, watchdog) — those remain the same
  open gaps recorded in `BASELINE.md`. Phase 3 only changed what happens to
  actuators once a fault (as already detected) is active.
- The `FAULT` shot-state label introduced in Phase 2 is now backed by real
  actuator behaviour for the first time - Phase 2's "observability only"
  caveat no longer applies.

## Build verification

Compiled with the same toolchain as Phase 0-2 (`esp32:esp32@2.0.17`):

```
Sketch uses 905333 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51268 bytes (15%) of dynamic memory, leaving 276412 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +44 bytes flash / +8 bytes RAM versus the
merged Phase 0-2 state (`v2.0.1-beta`: 905,289 bytes flash, 51,260 bytes
RAM) - consistent with consolidating several actuator-write call sites down
to two, offset by the new priority-resolution block.

## Behaviour-preservation checklist

- [x] Every pump-control branch computes the exact same `pumppower`/
      `PressureTarget` values, from the exact same conditions, in the exact
      same order as before - only the final actuator write moved.
- [x] `runPID()`'s PID computation and temperature-fault detection are
      unchanged; only the SSR write moved out of it.
- [x] `updatePumpRamp()`'s pressure-regulation ramp (the ±1/-2 adjustment,
      the overpressure cut, the two interval gates) is unchanged; only the
      dimmer write moved out of it.
- [x] Verified by grep: exactly one `digitalWrite(SSR_PIN, ...)` and one
      `light.setBrightness(...)` call site exist in the whole file, both
      inside `controlStep()`.
- [ ] **Intentional behaviour change:** a fault now forces the pump off too
      (previously only the heater), and does so continuously for as long as
      the fault persists rather than only inside the ~250ms-gated check
      that detected it (§3 above).
- [ ] **Intentional behaviour change:** flipping to temp-only mid-shot now
      forces the pump off on the next cycle, closing a gap that existed
      since the original baseline (§3 above).
- [ ] Bench re-test still required, same hardware limitation as Phase 0-2.
      This phase adds its own bench items: confirm normal shot behaviour is
      pixel-for-pixel unchanged (same pump timing, same pressure curve);
      confirm a temperature fault mid-shot actually cuts the pump, not just
      the heater; confirm flipping `PIDonly` mid-shot actually cuts the
      pump.
