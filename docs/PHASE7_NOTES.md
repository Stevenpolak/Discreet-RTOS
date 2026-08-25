# Phase 7 — Verification

REWRITE_PLAN.md frames this phase as a hardware/bench checklist with a single
exit condition: "the hardware/bench checklist passes and results are
recorded." This environment has no physical machine to run that checklist
against - the same constraint every prior phase's own bench items have run
into (see `BASELINE.md` through `PHASE6_NOTES.md`). What this phase actually
delivers instead is a rigorous **code-level verification pass**: for each of
the 16 checklist items, either (a) cite the exact code that guarantees it and
explain why the guarantee holds regardless of hardware, (b) identify that the
item is inherently a physical measurement no amount of source reading can
substitute for, or (c) - the outcome this pass exists to catch - identify that
the current code does **not** actually satisfy the item as written.

Two real gaps came out of (c). Neither is fixed in this PR; both are
documented below with the reasoning for leaving them as named, prioritized
follow-ups rather than guessing at a fix, matching how Phase 5 handled the
mains zero-cross question and Phase 6 handled deadman-timing validation.

The Phase 7 exit condition is **not met** by this PR. `REWRITE_PLAN.md`'s
Phase 7 checklist stays unchecked; this phase changes no `Discreet.ino`
behavior on its own (see "Review round 1" below for what the independent
review pass found and whether anything was fixed as a result).

## Per-item verification

| # | Checklist item | Verdict | Basis |
|---|---|---|---|
| 1 | Temp-only heats and regulates temperature while shot logic and pump remain inactive | Verified in code | AC-detect is gated on `currentMode != OperatingMode::TEMP_ONLY` (`Discreet.ino` controlStep(), the AC-detect block), so `acDetected` can never become true in `TEMP_ONLY` - the entire shot-phase branch never runs. The independent mode-restriction layer (output priority resolution, step 2) also forces `pumpOut = 0` and calls `endShot()` if a mode flip to `TEMP_ONLY` happens mid-shot. `runPID()` is unaffected by mode, so heating/regulation continues normally. Two independent code paths enforce this, not one - see Phase 3's original design and Phase 4 review for why. |
| 2 | Normal shot detection enters and exits every intended phase correctly | Verified in code | The `PREINFUSION -> BLOOM -> EXTRACTION -> COMPLETE -> IDLE` sequence in `controlStep()` partitions cleanly on `actime` against `shotSettings.preinftime`/`bloomtime` with no gap or overlap between the `if`/`else if`/`else` branches; `endShot()` is the single exit point (AC-off debounce or fault) and unconditionally sets `COMPLETE`, with the next cycle's `else` branch reporting `IDLE`. Unchanged in shape since Phase 2/3 review already exercised this logic. |
| 3 | Pre-infusion, bloom and extraction timing use wall-clock time | Verified in code | `elapsedTime = now - acDetectedTime; actime = elapsedTime / 1000;` - `now` is the single `millis()` sample `controlTaskEntry()` takes once per fixed 10ms cycle, not a loop-iteration count. This was the specific defect Phase 1 fixed in the original baseline. |
| 4 | Pressure setpoint and pump output behave correctly at phase boundaries | Verified in code | PRE-INFUSION targets `PrePressureSetpoint`, EXTRACTION targets `shotSettings.pressuresetpoint` (latched at shot start, immune to mid-shot edits - see item 11); each phase's `pumpPowerSetPreinf`/`pumpPowerSetExtraction` latch runs its one-time `basePumpPowerForSetpoint()` call exactly once per phase, and both latches reset (`= false`) whenever the low-pressure boost branch re-arms, so a boundary re-entry (e.g. pressure dropping again mid-extraction) recomputes correctly rather than reusing a stale base. BLOOM forces `pumppower = 0` unconditionally. The actual magnitude of `basePumpPowerForSetpoint()`'s table entries against a real boiler/pump is a bench-tuning question, not a logic question - see the bench checklist below. |
| 5 | Invalid temperature sensor forces heater and pump safe | Verified in code | `runPID()`: `isnan(input) \|\| input < 0 \|\| input > 160` sets `currentFault = FaultCode::INVALID_TEMPERATURE` and returns before computing PID output. `controlStep()` reads `faulted = (currentFault != FaultCode::NONE)` once, right after `runPID()`, and uses it to force `heaterOut = 0; pumpOut = 0;` unconditionally, plus `endShot()` if a shot was in progress. No debounce on setting the fault (fast to trip); `FAULT_CLEAR_STABLE_READINGS` (3) debounces only the *clearing* path, so an intermittent sensor can't chatter the actuators back on. |
| 6 | Invalid pressure sensor prevents unsafe pump regulation | **Gap - not met** | See "Gap 1" below. There is no `FaultCode` (or any other mechanism) for an invalid pressure reading; `GetPressure()` accepts whatever `analogRead()` returns with no validity check at all. |
| 7 | Over-temperature and all time-outs force safe outputs | Verified in code | Over-temperature is the `input > 160` arm of the same check as item 5 (both share `FaultCode::INVALID_TEMPERATURE` - a naming overlap, not a functional gap; see "Considered, not changed" below) and forces the same safe outputs. "Time-outs": the SSR authorization timeout (`SSR_AUTH_TIMEOUT_MS`, Phase 6) forces the heater pin off independently of `controlStep()`'s health; the Task WDT (Phase 5) recovers a genuinely wedged control task via a full reset. Both were verified in their own phases' review and are unchanged here. |
| 8 | Reboot, task failure and watchdog reset leave heater and pump off | Partially verified - see Gap 2 | **Heater**: `SSR_PIN` is set `OUTPUT`/`LOW` as the literal first hardware action in `setup()`, before even the boot delay (Phase 6 review), and the independent SSR deadman (a separate `esp_timer` task, not `controlTask`) forces it off within `SSR_AUTH_TIMEOUT_MS` (200ms) of the last successful `controlStep()` cycle regardless of whether `controlTask` is wedged, crashed, or never started. This holds across every failure mode named in this item. **Pump**: verified safe across reboot/cold-boot specifically (see "Considered, not changed" below - the thyristor gate-fire mechanism itself, not an explicit safe-default write, is why), but a *wedged-but-still-running* `controlTask` is not equivalently covered - see Gap 2. |
| 9 | A deliberately wedged control task is detected because that task is explicitly subscribed to the Task WDT | Verified in code | `controlTaskEntry()` calls `esp_task_wdt_add(NULL)` and checks the return value; `esp_task_wdt_reset()` is called only after `controlStep()` returns, so a genuine hang never reaches the feed. Actually observing the reset requires deliberately wedging the task on real hardware - see the bench checklist below. |
| 10 | Loss of SSR refresh makes the heater output decay to off within the documented maximum window | Verified in code | `ssrDeadmanCallback()` runs every `SSR_ENFORCE_INTERVAL_MS` (10ms) and writes `SSR_PIN` low the instant `(long)(ssrAuthorizedUntilMs - now) > 0` is false - i.e. within one enforce tick of `SSR_AUTH_TIMEOUT_MS` (200ms) after the last refresh, ~210ms worst case. The actual electrical/thermal decay of the heating element itself is a bench measurement, not a code question. |
| 11 | Mid-shot settings edits do not change the active shot and the latest valid pending revision becomes active after return to `IDLE` | Verified in code | `shotSettings` is latched from `activeSettings` once, at shot start, and is the only settings struct the shot-phase branches read - a mid-shot edit only ever reaches `pendingSettings` (see `acceptPreinftimeEdit()`/etc., all gated on `!acDetected` for the immediate-apply path). `endShot()` promotes the *entire* `pendingSettings` struct (`activeSettings = pendingSettings;`), not just the last-edited field, so a burst of edits to different fields during the same shot all land correctly on return to idle. |
| 12 | Wi-Fi loss does not disturb control timing | Verified in code (architecturally) | `controlTask` runs on its own FreeRTOS task (core 1, priority 5, Phase 5), driven by `vTaskDelayUntil()` against a fixed period; it has no dependency on `WiFi`/`server`/`ArduinoOTA` state, all of which live in `loopTask` only. A Wi-Fi drop cannot block or delay `controlStep()`. |
| 13 | Missing/failing SD card does not disturb control timing | Verified in code (architecturally) | Every `SD.*` call in this file is reached from `loop()`-side handlers (`handleFileRequest()`, `/getConfig`, `/saveConfig`, `startSD()`/`loadSDConfig()` at boot) - none are reachable from `controlStep()` or anything it calls. Same task-isolation argument as item 12. |
| 14 | Large web requests/uploads do not disturb control timing | Verified in code (architecturally) | `handleUpload()`/`server.handleClient()` run entirely in `loopTask`; `controlTask`'s only contact with that side is `commandQueue`/`telemetryQueue`, both non-blocking on the `controlTask` side (`xQueueReceive(..., 0)`, `xQueueOverwrite()`). |
| 15 | OTA start either refuses while unsafe or transitions the machine to a documented safe state | Verified in code | Does not refuse - transitions to a documented safe state instead (the item's "or" is satisfied by either). `ArduinoOTA.onStart()` latches `otaInProgress`, which `controlStep()`'s fault-priority chain checks every cycle for the whole OTA session and forces `heaterOut = 0` accordingly (Phase 6 review fixed this from a one-shot no-op to a real per-cycle latch). Pump is deliberately left unmasked during OTA - documented, not accidental; see Phase 6 review notes. |
| 16 | Telemetry remains coherent and stale snapshots are detectable | Verified in code | `TelemetrySnapshot.timestampMs` is set from the same `now` used to build the rest of the snapshot (never a mixed-cycle read) and is exposed to the web side as `snapshotTimestampMs` in `/getValues`. A caller can detect staleness by comparing it against wall-clock time; nothing currently does that comparison automatically (no internal self-check raises a fault on a stale snapshot) - the item asks that staleness be *detectable*, which the field satisfies, not that it be auto-flagged. |

## Gap 1 — no invalid-pressure-sensor detection (checklist item 6)

`GetPressure()` computes `currentPressure` directly from `analogRead(pressurepin)`
with no validity check at all:

```cpp
int raw = analogRead(pressurepin);
currentPressure = (raw * (maxPressure / 4095.0));
```

Unlike the thermocouple path (`isnan(input) || input < 0 || input > 160`,
verified for item 5), there is no equivalent check here, and no
`FaultCode` for it - `FaultCode` only has `NONE`/`INVALID_TEMPERATURE`.

**Why this isn't fixed in this PR, rather than just adding a bounds check:**
`analogRead()` on this target always returns an in-range 12-bit value
(0-4095) - there is no "impossible" raw value the way `NAN` is impossible for
a valid `MAX6675::readCelsius()` result. A disconnected or shorted analog
sensor typically reads as a plausible-looking value *within* that same
0-4095 range (near 0 for a broken wire to ground, near 4095 for a short to
supply) - both of which overlap with genuinely valid low/high pressure
readings during normal operation. Some ratiometric pressure transducers
signal a fault condition with a "live zero" convention (output pinned below
or above the sensor's own working band, e.g. under 0.5V/over 4.5V on a
0.5-4.5V active range within a 0-5V supply) - if this project's actual sensor
follows that convention, real fault thresholds could be derived from its
datasheet. Nothing in this codebase currently documents which sensor is
fitted or its output convention, and guessing thresholds without that
information would be exactly the kind of blind, hardware-uninformed fix this
project has consistently avoided (see Phase 5's mains zero-cross item, left
undone for the same reason).

**Recommendation**: confirm the fitted pressure sensor's part number/output
convention, then either (a) add a bounds check against its documented fault
band if it has one, or (b) accept that this sensor has no self-reporting
fault signal and document that explicitly rather than leaving it silently
implied to be covered. Either way this is a bench/datasheet item, not a code
item, until that information exists.

## Gap 2 — no independent deadman for the pump (checklist item 8, partial)

The heater has two independent layers of protection against a wedged
`controlTask`: the Task WDT (system-wide, ~5s recovery via reset, Phase 5)
and the SSR deadman (heater-specific, ~200ms decay to off, runs in a
separate `esp_timer` task, Phase 6). The pump has only the first. If
`controlTask` wedges while `pumpDemand`/the last `light.setBrightness()`
call left the pump at a nonzero level, the pump keeps running at that level,
undisturbed, until the Task WDT's own timeout recovers the system - up to
several seconds, not the ~200ms the heater gets.

**Why this isn't fixed in this PR**: closing this gap the same way Phase 6
closed it for the heater means adding a second, genuinely independent
`esp_timer`-based deadman - a real, nontrivial piece of new safety
infrastructure (a second authorization field, a second timer, its own
enforce interval, the same wraparound/NaN-safety reasoning Phase 6 had to
work through) - not a verification-phase fix. It's also lower severity than
the heater case it mirrors: a stuck-on pump moves water, not heat - the
worst case is an overrun/overflow bounded by the Task WDT's existing ~5s
ceiling, not a burn/fire risk. Given that, this is flagged as a real,
understood gap and a candidate for its own follow-up phase, not added here
unasked, matching this session's practice of not expanding a phase's scope
past what was explicitly directed.

**Recommendation**: if closing this gap is wanted, it's a small, well-
understood addition - clone the SSR deadman pattern (`ssrAuthorizedUntilMs`,
`ssrDeadmanCallback()`, `initSsrDeadman()`) for the pump/dimmer pin. Worth
scoping as its own phase (or folded into Phase 8 cleanup) rather than
decided unilaterally here.

## Considered, not changed

**Pump/dimmer pin has no explicit early safe-default, unlike `SSR_PIN`.**
Checked the `Dimmable_Light_for_Arduino` library source
(`thyristor.cpp`) directly rather than assuming symmetry with the SSR fix:
`Thyristor::begin()` calls `pinMode(pin, OUTPUT)`, and the pin is only ever
driven `HIGH` for a brief gate-fire pulse timed against the zero-cross
interrupt, returning `LOW` immediately after - a triac needs an active,
recurring gate pulse to keep conducting past each zero-crossing. A floating
or undriven pin at boot (before `DimmableLight::begin()` runs, tens of
seconds into `setup()` today) produces no pulses at all, so the triac simply
never fires - unlike `SSR_PIN`, which Phase 6 found could plausibly read as
a sustained logic `HIGH` on a level-triggered drive if left floating during
that same window. The two pins fail differently by nature, not just by
omission; no equivalent fix is needed here.

**`FaultCode::INVALID_TEMPERATURE` also covers genuine over-temperature.**
Checklist item 7 groups these together and both are handled by the same
code path; the enum name conflates "sensor is lying" with "sensor is
reporting a real, dangerous value" under one label. Purely a naming/
documentation nit with no behavioral effect - a rename is a one-line,
zero-risk change but out of scope for a phase whose job is verifying
existing behavior, not touching it.

## Bench checklist — physical verification still required

Everything marked "Verified in code" above is a logic guarantee, not a
measurement. The following still need a real machine before Phase 7's own
exit condition can be considered met:

- Items 1-4: run an actual temp-only session and a full shot; confirm phase
  transitions, pump behavior and boundary timing feel correct against a real
  boiler/pump, not just against the code's own bookkeeping.
- Item 5: disconnect/short the thermocouple mid-idle and mid-shot; confirm
  both actuators cut immediately and recover after 3 stable readings.
- Item 6: blocked on Gap 1 above - identify the fitted pressure sensor first.
- Item 7: force `input` above 160 (or disconnect) and confirm safe outputs;
  confirm the SSR deadman and Task WDT timeouts as their own bench items
  (already flagged in `PHASE5_NOTES.md`/`PHASE6_NOTES.md`).
- Item 8: cold boot, and a warm reset via the Task WDT specifically (not just
  power-cycle) - confirm both actuators come up/stay off. Gap 2 above means
  the pump case should be tested with a genuine `controlTask` wedge, not just
  a clean reboot, to see the real recovery window in practice.
- Item 9: deliberately wedge `controlTask` (e.g. an infinite loop behind a
  debug flag) and confirm the watchdog reset actually fires and recovers.
- Item 10: with the machine heating, kill `controlTask` (or the whole
  `esp_timer` service, to test the heartbeat in `loop()`'s diagnostics) and
  time the SSR's actual physical decay.
- Item 11: edit settings mid-shot across several fields, confirm none affect
  the running shot, confirm all take effect together on the next shot.
- Items 12-14: pull Wi-Fi, pull the SD card, and push a large upload, each
  while a shot is running; watch `controlTask`'s diagnostics report in
  `loop()` (worst cycle time, deadline misses) for any disturbance.
- Item 15: trigger an OTA update mid-idle and mid-shot; confirm the heater
  authorization actually drops for the whole transfer, not just the start.
- Item 16: compare `snapshotTimestampMs` against wall-clock time under normal
  operation and while deliberately wedging `controlTask`, to confirm a stale
  snapshot is actually visible in the field, not just present as a number.

This list is additive to - not a replacement for - the bench items already
outstanding from Phase 0 through Phase 6 (mains zero-cross sampling and SSR
deadman timing chief among them; see their own notes files).
