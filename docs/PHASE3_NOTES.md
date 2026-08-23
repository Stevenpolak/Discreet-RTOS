# Phase 3 notes — centralize control ownership

Date: 2026-08-23
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 3, applied on top of the
merged Phase 0-2 work (`v2.0.1-beta`, see [BASELINE.md](BASELINE.md),
[PHASE1_NOTES.md](PHASE1_NOTES.md), [PHASE2_NOTES.md](PHASE2_NOTES.md)).

Goal per `REWRITE_PLAN.md`: a single function owns all control decisions
and both actuator outputs, **while still running in the original execution
context**. No FreeRTOS task, queue, or task boundary yet — that's Phase 4/5.

**This branch went through the same independent review process as Phase
0-2 before being proposed for merge; see "Review round 1" below.** The
first version had several real problems, the most severe self-inflicted:
before dispatching the review, re-reading my own diff turned up a bug
where the pump would have been driven mostly off during a shot instead of
holding its ramped power. The review then found that neither a fault nor a
mid-shot mode change actually stopped the shot state machine underneath
the masked outputs, which both let pump demand silently wind up while
masked (producing a full-power slam the instant the mask cleared) and
reintroduced the exact "fault freezes settings edits" bug Phase 2 had
already fixed once, through a different path. All are fixed below.

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
restrictions, state-machine demand, controller output." The version
described here is the post-review-round one; see "Review round 1" below
for what the first version got wrong.

```cpp
bool faulted = (currentFault != FaultCode::NONE);
// ... AC detection is also gated on !faulted and currentMode ...

if (faulted) {
  if (acDetected) endShot();
  currentShotState = ShotState::FAULT;
}

bool heaterOut = heaterDemand;
int pumpOut = pumpDemand;

if (faulted) {
  heaterOut = false;
  pumpOut = 0;
}

if (currentMode == OperatingMode::TEMP_ONLY) {
  pumpOut = 0;
}

pumpOut = constrain(pumpOut, 0, 255);
resolvedHeaterOn = heaterOut;
resolvedPumpPower = pumpOut;
digitalWrite(SSR_PIN, heaterOut ? HIGH : LOW);
light.setBrightness(pumpOut);
```

**Fault/safety off (checklist: "verify that fault handling always forces
safe outputs").** Before this PR, a temperature fault only ever forced the
SSR off — that logic lived inside `runPID()` and had no coupling to the
pump at all. If a fault occurred while a shot happened to be in progress,
the pump kept running on the shot state machine's demand as if nothing
were wrong; there was no code path connecting `currentFault` to
`pumpDemand`. The fault check is the first override evaluated after all
demands are computed, and unconditionally forces both outputs off, every
single `controlStep()` cycle for as long as the fault persists — not just
once when first detected. **A fault also calls `endShot()`** if a shot was
in progress: found in review that masking the output alone let the shot
state machine keep running blind, which caused both the pump-demand
wind-up described in "Review round 1" below and a reintroduction of a bug
Phase 2 already fixed once (a fault freezing settings edits). `endShot()`
resets the shot exactly like a normal AC-off does — see its own comment in
`Discreet.ino`.

**Mode restrictions (checklist: "verify that temp-only cannot enable the
pump").** This one already held structurally before this PR, as a side
effect of AC detection being gated on the mode (a shot can never start
while in temp-only mode, so `pumpDemand` could never become nonzero that
way). But that's an *indirect* guarantee with one gap: nothing stopped the
mode from being flipped to temp-only via `/saveConfig` while a shot was
**already** in progress — the running shot would just continue, pump and
all, ignoring the newly-set mode. The explicit `pumpOut = 0` override
closes that gap: even mid-shot, flipping to temp-only now forces the pump
off on the very next cycle. This reads `currentMode` (the Phase 2 enum),
not the raw `PIDonly` bool it mirrors — the first version of this PR read
`PIDonly` directly; see "Review round 1" for why that was wrong for
safety-critical code specifically.

**Defensive clamp and resolved-output telemetry**, both added in review:
`pumpOut` is `constrain()`-ed to `0..255` immediately before it narrows to
`light.setBrightness()`'s `uint8_t` parameter — nothing produces an
out-of-range value today, but this is now the one place all pump output
funnels through, which is exactly where that bound belongs. `resolvedHeaterOn`/
`resolvedPumpPower` record what was actually written each cycle, separately
from `heaterDemand`/`pumpDemand` (what was *asked for* before the
overrides above), and are published through `TelemetrySnapshot`/
`/getValues` — see "Review round 1" finding 6 below.

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

## Review round 1: findings and fixes

Before being proposed for merge, this branch was reviewed by an independent
model (Opus) across 7 angles, following the same process as Phase 0-2. One
severe bug was found and fixed by re-reading the diff before the review
even started; the review then found six more, all fixed:

1. **(fixed, found before dispatching the review, most severe)** An
   unconditional `pumpDemand = 0;` at the top of every `controlStep()`
   cycle defeated `updatePumpRamp()`'s own gating: `updatePumpRamp()` only
   re-asserts `pumpDemand` on the ~50ms-gated iterations, so on every
   iteration in between (the overwhelming majority, since `loop()` runs far
   faster than 50ms) the reset would have commanded the pump off. The pump
   would have run mostly-off with brief full-power corrections for the
   entire steady-state portion of every shot. Fixed by removing the reset -
   `pumpDemand` now persists between iterations exactly like the
   pre-Phase-3 `SetPump()` did by simply not writing the actuator on
   ungated iterations; see the comment above the `if (acDetected)` block.
2. **(fixed)** Neither a fault nor a mid-shot mode change to temp-only
   actually stopped the shot state machine - both only masked the final
   actuator write. With the pump actually off, pressure would collapse,
   which drove the low-pressure boost branch to re-latch `pumpDemand` to
   255 within a cycle or two; the instant the mask cleared, the pump
   slammed to full power with no ramp. Fixed by having a fault call
   `endShot()` (§3 above) when a shot is in progress, rather than only
   overriding the write.
3. **(fixed, same root cause as finding 2)** Because a fault didn't clear
   `acDetected`, the Phase 2 settings-lifecycle idle gate (`!acDetected`) —
   fixed once already, specifically to stop a fault from freezing settings
   edits (`PHASE2_NOTES.md` "Review round 1", finding 1) — read "mid-shot"
   again during any fault, through this different path. `endShot()` fixes
   this as a side effect of finding 2's fix.
4. **(fixed)** `steam()` (which can call `queueBuzzer()`, an unbounded
   `xSemaphoreTake(buzzerMutex, portMAX_DELAY)`) ran between `runPID()`
   detecting a fault and `controlStep()`'s actuator write. A stalled or
   starved buzzer timer task could therefore delay turning the heater off
   during exactly the condition meant to force it off immediately. Fixed
   by moving the `steam()` call to after the actuator writes - it doesn't
   feed anything the shot-phase logic reads, so nothing else changes.
5. **(fixed)** `currentFault` had no hysteresis: one bad thermocouple
   sample set it, the very next good sample cleared it. Coupling that
   directly to the pump (this phase's whole point) meant an intermittent
   sensor - a real, common failure mode - could chatter the pump on and off
   every `PID_INTERVAL`. Fixed with `FAULT_CLEAR_STABLE_READINGS` (3): the
   fault still sets immediately on any bad reading (fast in the safe
   direction), but requires 3 consecutive good readings before clearing.
6. **(fixed)** The mode-restriction check read the raw `PIDonly` bool
   instead of the Phase 2 `currentMode` enum designated as mode's source of
   truth - the two are kept in sync today, but nothing enforced that, and
   this is now safety-critical code. Also added: telemetry only ever
   published the pre-resolution demand (`pumppower`), never what was
   actually written to either actuator, so a bench operator had no way to
   confirm a fault or mode restriction actually took effect - added
   `resolvedHeaterOn`/`resolvedPumpPower` to `TelemetrySnapshot` and
   `/getValues`.
7. **(fixed, defense-in-depth)** No range clamp existed at the single
   actuator-write choke point before narrowing to `light.setBrightness()`'s
   `uint8_t` parameter - currently unreachable (nothing produces an
   out-of-range value today) but this is now the one place all pump output
   funnels through, which is exactly where that bound belongs. Added
   `constrain(pumpOut, 0, 255)`.

Also, as a consequence of fix 2/3: without `endShot()`'s early-out, a
fault occurring while the paddle stayed engaged would re-detect AC and
start (then immediately self-abort) a fresh shot on every single cycle for
as long as the fault persisted - harmless, but pointless churn. Added an
explicit `!faulted` term to the AC-detection gate so no shot activity is
attempted at all while a fault is active; it resumes automatically once the
fault clears and the paddle is still engaged.

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
  open gaps recorded in `BASELINE.md`. This phase changed what happens to
  actuators (and now the shot state machine) once a fault is active, and
  added hysteresis to clearing an already-detected fault; it did not add
  any new fault type.
- The `FAULT` shot-state label introduced in Phase 2 is now backed by real
  actuator behaviour, and now also aborts an in-progress shot - Phase 2's
  "observability only" caveat no longer applies at all.
- Cleanup opportunities the review also surfaced but that were left alone
  as pure structure, not correctness (re-verified as genuinely required,
  not just untidy): `pumppower` and `pumpDemand` remain two separate
  variables - collapsing them would break the overpressure cut, which needs
  to zero the actuator output without destroying the ramp's internal state.
  The five `pumppower = X; pumpDemand = pumppower;` pairs in the shot-phase
  branches were not collapsed into a small helper; a worthwhile follow-up,
  not done here to keep this fix round focused on correctness.

## Build verification

Compiled with the same toolchain as Phase 0-2 (`esp32:esp32@2.0.17`), after
the review-round fixes:

```
Sketch uses 905621 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51284 bytes (15%) of dynamic memory, leaving 276396 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +332 bytes flash / +24 bytes RAM versus the
merged Phase 0-2 state (`v2.0.1-beta`: 905,289 bytes flash, 51,260 bytes
RAM).

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
- [x] `pumpDemand` correctly holds its last-ramped value between
      `updatePumpRamp()`'s gated ticks rather than being reset every cycle
      (Review round 1, finding 1 - the most severe issue found).
- [x] A fault or a mid-shot mode-restriction can no longer leave demand
      values winding up unseen underneath a masked output, and cannot
      produce a full-power slam once the mask clears (Review round 1,
      finding 2).
- [x] A fault no longer freezes settings edits by leaving `acDetected`
      true (Review round 1, finding 3).
- [x] No blocking call sits between fault detection and the actuator write
      that's supposed to respond to it (Review round 1, finding 4).
- [x] An intermittent sensor cannot chatter the pump via unhysteresised
      fault clearing (Review round 1, finding 5).
- [x] The mode-restriction override cannot silently stop enforcing itself
      if `currentMode` and `PIDonly` ever diverge (Review round 1,
      finding 6).
- [ ] **Intentional behaviour change:** a fault now forces the pump off too
      (previously only the heater), continuously for as long as the fault
      persists, and aborts a shot in progress rather than leaving it
      running underneath the masked output (§3 above; Review round 1,
      findings 2-3).
- [ ] **Intentional behaviour change:** flipping to temp-only mid-shot now
      forces the pump off on the next cycle, closing a gap that existed
      since the original baseline (§3 above).
- [ ] **Intentional behaviour change:** clearing a temperature fault now
      requires 3 consecutive valid readings (~750ms) instead of 1 (~250ms);
      setting the fault is unchanged - still immediate on any single bad
      reading (Review round 1, finding 5).
- [ ] Bench re-test still required, same hardware limitation as Phase 0-2.
      This phase adds its own bench items: confirm normal shot behaviour is
      pixel-for-pixel unchanged (same pump timing, same pressure curve);
      confirm a temperature fault mid-shot actually cuts the pump and ends
      the shot, not just the heater; confirm the pump does not slam to full
      power when a fault clears mid-shot-abort-and-restart; confirm
      flipping the mode mid-shot actually cuts the pump; confirm
      `heaterOn`/`resolvedPumpPower` in `/getValues` track reality during
      a fault.
