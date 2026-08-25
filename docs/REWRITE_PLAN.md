# Two-task rewrite plan

Status: awaiting required hardware validation  
Current phase: Phase 0/1/2/3/4/5/6/7 merged to main and tagged (`v2.0.0-alpha` through `v2.0.6-beta`) - none of Phase 0-7's exit conditions are bench-verified yet; Phase 7's own verification pass identified two priority items ahead of Phase 5's zero-cross sampling question and Phase 6's deadman timing validation: the pressure sensor's fault-signaling convention (blocks closing Phase 7's Gap 1) and whether a genuine over-temperature condition is reachable in practice (Gap 5) - see docs/PHASE7_NOTES.md for the full prioritized bench checklist, and docs/BASELINE.md through docs/PHASE6_NOTES.md for everything still outstanding from earlier phases  
Architecture target: one Arduino service loop plus one dedicated FreeRTOS control task

This checklist is designed for work spread across multiple sessions. Finish and document one bounded step at a time; do not combine the architecture migration with unrelated behaviour changes.

## Target architecture

### Arduino `loop()`: service side

Owns work that may be delayed briefly without affecting machine control:

- Wi-Fi, mDNS and the web server
- OTA handling
- SD-card file management and uploads
- Timer-driven, non-blocking buzzer sequencing
- Validation of requested setting changes
- Sending commands/settings to the control task
- Reading the latest immutable telemetry snapshot

The service side must never write directly to the pump dimmer or heater SSR.

### FreeRTOS control task: coffee and safety side

Is the sole owner of time-critical machine behaviour and final actuator outputs:

- shot detection and shot state machine
- pre-infusion, bloom and extraction timing
- pressure sampling and pump regulation
- temperature sampling and PID
- temp-only operation
- sensor validation, over-temperature protection and time-outs
- pump/dimmer and heater/SSR output

The task should use a 10 ms periodic base cycle with `vTaskDelayUntil()`. Slower activities are scheduled inside that single task:

- every 10 ms: shot detection, state transitions and safety checks
- every 50 ms: pressure measurement and pump regulation
- every 250 ms: temperature measurement and PID calculation
- every cycle: apply safety overrides before actuator outputs

The dimmer library's zero-cross phase timing remains interrupt-driven and is not moved into this control loop.

## Operating model

Use a shot state such as:

`IDLE -> PREINFUSION -> BLOOM -> EXTRACTION -> COMPLETE`

`FAULT` may be entered from any state and has priority over all normal outputs.

Temp-only (currently represented by `PIDonly`) is retained as a separate operating mode, not a shot state:

- temperature PID remains active
- shot detection/state stays idle
- pump output remains disabled
- heater safety checks remain active

The current Pause/Resume feature is intentionally removed. There is no `PAUSED` state and no ambiguous question about whether shot time should freeze.

## Communication between tasks

### Commands and settings

Use a bounded FreeRTOS queue from the service loop to the control task. Commands should be small value-only structs and should include everything needed to validate and apply the change. Do not pass `String` objects, raw pointers or references to mutable web/JSON data.

Only the control task commits settings that affect control behaviour. Acknowledge accepted/rejected changes through telemetry or a small response mechanism.

### Settings lifecycle

Control and profile settings are frozen for the duration of a shot.

- Keep separate `activeSettings` and `pendingSettings` value snapshots with revision numbers.
- When a shot starts, latch the complete active settings snapshot into `shotSettings`.
- Every shot phase uses only `shotSettings`; web edits never mutate it.
- A valid edit received during `PREINFUSION`, `BLOOM`, `EXTRACTION` or `COMPLETE` updates `pendingSettings` only.
- After the shot returns to `IDLE`, atomically promote the latest pending revision to `activeSettings`.
- Telemetry reports active, shot and pending revision numbers so the UI can show that a change is queued.
- Safety limits, fault actions and emergency shutdown commands are not deferred and always take effect immediately.

This means that changing brew temperature, pressure or a phase duration mid-shot cannot alter the shot already in progress; the latest valid change becomes active for the next shot.

### Telemetry

Use a queue of length 1 containing a value-only `TelemetrySnapshot`.

- control task publishes with `xQueueOverwrite()`
- service loop reads the latest value with `xQueuePeek()`
- neither side waits for the other
- include a sequence number or timestamp so stale data is detectable
- copy complete snapshots; do not expose shared mutable fields

Candidate fields include mode, shot state, temperature, pressure, elapsed shot time, phase time, PID output, pump power, fault code, settings revision and snapshot timestamp.

## Rewrite phases

### Phase 0 — Establish a safe baseline

- [x] Record the exact imported upstream commit/tag. See [BASELINE.md](BASELINE.md).
- [x] Build the current firmware unchanged. Compiles cleanly against `esp32:esp32@2.0.17`; see [BASELINE.md](BASELINE.md).
- [x] Record ESP32 board/core version and all library versions. See [BASELINE.md](BASELINE.md).
- [ ] Record pin mapping and hardware variants used for testing. Pin mapping is documented in [BASELINE.md](BASELINE.md), but no physical test variant has been identified yet.
- [x] Document safe boot outputs: heater off and pump off until initialized. Documented from source; **not yet bench-verified** — see [BASELINE.md](BASELINE.md) "Safe boot outputs".
- [x] Document fault behaviour for invalid temperature, invalid pressure, over-temperature, time-out and watchdog reset. See [BASELINE.md](BASELINE.md) "Fault behaviour"; several gaps identified and carried forward, not fixed in Phase 0.
- [x] Create a baseline tag or branch. Tag `baseline-phase0`.
- [ ] Capture a short test record of current temp-only and normal-shot behaviour. **Not done** — no physical hardware was available in this environment. This remains open and must be completed on a bench before Phase 1+ firmware runs on real hardware.

Exit condition: **partially met**. The unmodified baseline builds and its essential behaviour is documented from source, but the bench test record is an open item, not a completed one — see [BASELINE.md](BASELINE.md) for the full list of open risks.

### Phase 1 — Remove blocking and loop-count timing

- [x] Replace buzzer delays with a timer-driven non-blocking sequencer. See [PHASE1_NOTES.md](PHASE1_NOTES.md). Pattern edges are now driven by an ESP one-shot timer independent of `loop()`, so a blocking web/SD/OTA call can no longer stretch or hold a beep — closing a risk an earlier, simpler design could only document.
- [x] Replace loop-count-based timing such as `offcount` with elapsed time or a timestamp derived from the relevant signal. See [PHASE1_NOTES.md](PHASE1_NOTES.md). An independent review caught a real regression in the first version (elapsed time alone let a `loop()` stall spuriously end a live shot); fixed by requiring a minimum observed-sample count alongside elapsed time. Also found and fixed the same defect in `SetPump()`'s `callCount`, which the original pass had missed. Debounce constants still need bench confirmation.
- [x] Audit web, SD and OTA paths for blocking calls that could affect migration. See [PHASE1_NOTES.md](PHASE1_NOTES.md); documented, not changed — deferred to later phases.
- [ ] Keep machine behaviour unchanged in this phase. The intended equivalence is documented in [PHASE1_NOTES.md](PHASE1_NOTES.md) "Behaviour-preservation checklist", with two small, deliberately-scoped exceptions (an explicit pump-off on shot-end, and the buzzer's edges no longer depending on loop latency) — but this cannot be checked off before bench confirmation. In particular, `AC_OFF_DEBOUNCE_MS`/`AC_OFF_MIN_SAMPLES` are judgement calls, not measured equivalents of the old 100-loop-iteration debounce.
- [ ] Re-run the baseline tests. **Not done** — same hardware limitation as Phase 0.

Exit condition: **implementation complete, not yet met overall**. Two rounds
of independent review found and fixed a real regression (the AC-off
debounce could be satisfied by a `loop()` stall) and a design gap (the
buzzer could stick on during a stall); both are now fixed, and no control
behaviour depends on loop iteration speed. Phase 2 still should not start
until the Phase 0/1 bench checklist below is actually recorded: temp-only
and normal-shot behaviour, safe-boot output measurement, buzzer sound
check, and `AC_OFF_DEBOUNCE_MS`/`AC_OFF_MIN_SAMPLES` tuning.

### Phase 2 — Introduce explicit state and data models

**Started ahead of the Phase 0/1 bench checklist being recorded, on
explicit direction.** Per this document's own rule 4 ("do not start a
phase until the previous exit condition is met or the exception is written
down"), that exception is: no physical hardware has been available in any
session so far, the bench items were already blocked before this phase
started, and Phase 2 is pure code/data-model structure that does not touch
timing-sensitive or actuator-priority behaviour beyond what's noted below -
judged low-risk enough to proceed with in software while the bench
checklist remains outstanding. It does not reduce the need to complete
that checklist before any of this runs on real hardware.

- [x] Add `OperatingMode` with at least normal brew and temp-only. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §1 - kept as a mirror of the existing `PIDonly` bool rather than replacing it, to avoid auditing every `PIDonly` read without hardware to verify against.
- [x] Add `ShotState`: `IDLE`, `PREINFUSION`, `BLOOM`, `EXTRACTION`, `COMPLETE`, `FAULT`. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §1-2. `FAULT` priority is observability-only in this phase - it does not yet override actuator outputs; see the explicit scope note in §2.
- [x] Remove Pause/Resume endpoints, variables and UI assumptions. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §3. No UI assumption existed to remove (checked the SD-card front-end archive).
- [x] Define value-only `ControlSettings`, `ControlCommand` and `TelemetrySnapshot` structs. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §4 - all three are actually used (`ControlCommand` by `handleAdjust()`, `TelemetrySnapshot` by `handleGetValues()`), not just defined.
- [x] Define allowed state transitions and entry/exit actions. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §1, §5 - shot-start and shot-end entry actions (settings latch/promotion, brew-setpoint mirror) are implemented; transitions are labels on the existing, unchanged phase conditions.
- [x] Define settings ranges, units and revision handling. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §4-5 - ranges match the original `constrain()` calls; each `ControlSettings` instance carries a revision counter.
- [x] Define `activeSettings`, immutable per-shot `shotSettings` and latest-value `pendingSettings`. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §5.
- [x] Define the atomic `pendingSettings -> activeSettings` promotion on return to `IDLE`. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §5. This is a genuine, intentional behaviour change (mid-shot edits no longer take effect immediately) - explicitly scoped here by this document's own "Settings lifecycle" section.
- [x] Add human-readable fault codes. See [PHASE2_NOTES.md](PHASE2_NOTES.md) §1 - `toString()` for all three new enums, not just `FaultCode`.

Exit condition: **met at the code/data-model level** - modes, states,
settings, commands and telemetry have explicit definitions, still running
synchronously inside the single `loop()` with no second task. Not yet
bench-verified (same outstanding item as Phase 0/1); see
[PHASE2_NOTES.md](PHASE2_NOTES.md) "Behaviour-preservation checklist" for
what specifically still needs a bench pass, including the two intentional
behaviour changes this phase introduces (settings latch, Pause/Resume
removal).

### Phase 3 — Centralize control ownership

**Started ahead of the outstanding Phase 0/1/2 bench checklist, on the same
explicit-direction basis recorded under Phase 2 above.** This phase moves
actuator-write ownership, which is exactly the kind of change that most
needs bench verification before real use - the exit condition below is
met at the code level only.

- [x] Move coffee logic behind one `controlStep(now)` boundary. See [PHASE3_NOTES.md](PHASE3_NOTES.md) §1-2.
- [x] Make this boundary the only writer of pump/dimmer and heater/SSR outputs. See [PHASE3_NOTES.md](PHASE3_NOTES.md) §1 - verified by grep, exactly one write site per actuator, both inside `controlStep()`.
- [x] Route web changes through validated requests instead of direct global-variable or actuator writes. Already satisfied by Phase 2's `ControlCommand`/`applyControlCommand()`; re-verified for this phase in [PHASE3_NOTES.md](PHASE3_NOTES.md) §4.
- [x] Apply output priority: fault/safety off, mode restrictions, state-machine demand, controller output. See [PHASE3_NOTES.md](PHASE3_NOTES.md) §3.
- [x] Verify that temp-only cannot enable the pump. See [PHASE3_NOTES.md](PHASE3_NOTES.md) §3 - now enforced by two independent mechanisms (AC-detect gating and an explicit mode-restriction override), closing a gap where flipping to temp-only mid-shot previously would not have stopped the pump.
- [x] Verify that fault handling always forces safe outputs. See [PHASE3_NOTES.md](PHASE3_NOTES.md) §3 - the pump was never coupled to a temperature fault before this phase; it is now, unconditionally, every cycle.

Exit condition: **met at the code level.** A single function
(`controlStep(now)`) owns all control decisions and both actuator writes,
still running synchronously inside the original `loop()`. Not yet
bench-verified - see [PHASE3_NOTES.md](PHASE3_NOTES.md) "Behaviour-
preservation checklist" for what a bench pass needs to confirm, including
the two new safety-positive behaviour changes this phase introduces.

### Phase 4 — Add safe task communication

**Started ahead of the outstanding Phase 0-3 bench checklist, on the same
explicit-direction basis as Phases 2-3.** No FreeRTOS task exists yet -
`commandQueue`/`telemetryQueue` have one producer and one consumer today,
both the same single Arduino task - so this phase is lower actuator risk
than Phase 3 (no actuator-write logic changes), but see
[PHASE4_NOTES.md](PHASE4_NOTES.md) for a real caveat: the reasoning that
makes `ControlCommand`'s relative-delta shape safe today may not survive
Phase 5 unchanged.

- [x] Create the bounded command/settings queue. `commandQueue`, depth 8. See [PHASE4_NOTES.md](PHASE4_NOTES.md) §1.
- [x] Create the length-1 telemetry queue. `telemetryQueue`. See [PHASE4_NOTES.md](PHASE4_NOTES.md) §3.
- [x] Publish telemetry with `xQueueOverwrite()`. Once per `controlStep()` cycle, after the actuator writes.
- [x] Read telemetry with `xQueuePeek()`. `handleGetValues()`, with a direct-build fallback if the queue isn't ready.
- [x] Ensure queue operations used by either task are non-blocking or tightly bounded. All sends/receives/peeks use `0` ticks-to-wait; none can block.
- [x] Reject unsafe or invalid settings inside the control owner. `applyControlCommand()`'s `constrain()` calls are the only validation, reachable only from the queue-drain loop, never directly from a handler. Review round 1 found `SET_BREW_SETPOINT_ABSOLUTE` (the one new command type this phase adds) had no `constrain()` unlike every sibling case - fixed. See [PHASE4_NOTES.md](PHASE4_NOTES.md) "Review round 1."
- [x] Queue valid mid-shot edits as pending without changing `shotSettings`. Unchanged - the settings-lifecycle latch itself (Phase 2) doesn't change in this phase, only how commands reach `applyControlCommand()`.
- [x] Coalesce repeated edits so the latest complete pending revision wins. Achieved without separate dedup logic - see [PHASE4_NOTES.md](PHASE4_NOTES.md) §2 for why draining in FIFO order and reading the current base at apply time already gives this property.
- [ ] Test command bursts and slow/unavailable web clients. Not bench-tested (no hardware); the failure mode (an explicit `503` on a full queue, never a silent drop, now also true for `/saveConfig` after review round 1) is implemented at the firmware boundary - but the shipped front end doesn't check the response status, so the `503` is currently invisible in the UI. See [PHASE4_NOTES.md](PHASE4_NOTES.md)'s checklist and "Review round 1."
- [x] Confirm no shared mutable control globals remain. `TelemetrySnapshot` widened (`displaySetpoint`/`pendingPreinftime`/`pendingBloomtime`/`pendingPressuresetpoint`) so `handleGetValues()` no longer reads `pendingSettings`/`setpoint`/`steamRequested` directly; `/saveConfig`'s direct writes to `PIDonly`/`currentMode`/brew setpoint replaced with queued commands. `Kp`/`Ki`/`Kd`/`offset`/`steamSetpoint` remain direct, deliberately - see [PHASE4_NOTES.md](PHASE4_NOTES.md) §4. Review round 1 also found `handleTemp()` still reads live `setpoint` directly and `offset` doubles as a safety-clamp bound; documented as narrower pre-existing residuals rather than reintroductions - see [PHASE4_NOTES.md](PHASE4_NOTES.md) "Review round 1."

Exit condition: **met, with one caveat written down rather than
resolved** - all cross-boundary data (settings edits, mode changes,
telemetry) is copied through defined queue messages. The one thing not
fully settled: whether `ControlCommand`'s relative-delta shape survives
Phase 5 unchanged depends on that phase's own draining cadence - see
[PHASE4_NOTES.md](PHASE4_NOTES.md) "What was intentionally not done."

### Phase 5 — Create the FreeRTOS control task

- [x] Move `controlStep()` into one dedicated task. `controlTaskEntry()`, created in `setup()` via `xTaskCreatePinnedToCore()`. See [PHASE5_NOTES.md](PHASE5_NOTES.md) §1.
- [x] Start with a 10 ms period using `vTaskDelayUntil()`. `CONTROL_TASK_PERIOD_TICKS = pdMS_TO_TICKS(10)`, drained via `xTaskDelayUntil()` (review round 1: its own return value is the deadline-miss signal, see [PHASE5_NOTES.md](PHASE5_NOTES.md) "Review round 1" #4).
- [x] Schedule 50 ms pressure work and 250 ms temperature/PID work inside the task. Unchanged from Phase 0-4: `GetPressure()`/`runPID()` already self-gate on `PRESS_INTERVAL`/`PID_INTERVAL` via their own `millis()` checks; the 10 ms task period just lets those gates fire on their intended cadence. See [PHASE5_NOTES.md](PHASE5_NOTES.md) §1.
- [x] Keep zero-cross dimmer timing in the library interrupt path. Untouched - `dimmable_light`'s own ISR path is unaffected; `controlStep()` still only calls `light.setBrightness()`.
- [x] Choose task priority, core affinity and stack size deliberately; document the reason. Core 1 (same as `loopTask`), priority 5 (above `loopTask`'s 1, below `arduino_events`/lwIP/Wi-Fi-driver's ~18-23), 4096-byte stack. Review round 1 verified the core-placement rationale against this project's actual `sdkconfig` rather than leaving it assumed (lwIP's `tcpip_thread` is confirmed pinned to core 0). See [PHASE5_NOTES.md](PHASE5_NOTES.md) §2.
- [x] Explicitly subscribe the control task to the ESP Task Watchdog with `esp_task_wdt_add(NULL)` from inside that task; do not rely on the watched idle tasks. See [PHASE5_NOTES.md](PHASE5_NOTES.md) §3.
- [x] Feed the Task WDT with `esp_task_wdt_reset()` only after a complete successful control cycle, including safety evaluation and actuator/deadman refresh. Called immediately after `controlStep()` returns, never before; review round 1 fixed a failed-subscription case that had been logging unconditionally at 100 Hz.
- [x] Check and handle the return values of the Task WDT API calls. Both `esp_task_wdt_add()` and `esp_task_wdt_reset()` are checked and logged; task-creation failure itself now forces `ESP.restart()` rather than continuing with no control loop at all (review round 1).
- [ ] Measure worst observed cycle time and deadline misses. Instrumented (`controlTaskWorstCycleUs`/`controlTaskDeadlineMisses`, reported from `loop()` every 5s after review round 1 moved the report out of the real-time task itself) but not bench-measured - no hardware. See [PHASE5_NOTES.md](PHASE5_NOTES.md) §4 and its bench checklist.
- [ ] Measure stack high-water mark. Instrumented (`uxTaskGetStackHighWaterMark(controlTaskHandle)`, same report) but not bench-measured.
- [ ] Verify that deliberately wedging the control task triggers the intended watchdog recovery and leaves outputs safe. Not bench-tested.
- [ ] Stress Wi-Fi, web requests, SD access and OTA preparation during a simulated/bench shot. Not bench-tested; an OTA-induced deadline miss is expected regardless of core placement (both cores are halted during a flash erase/program - an ESP32 hardware constraint, see [PHASE5_NOTES.md](PHASE5_NOTES.md) §2) and should not be chased as a bug.

Exit condition: control timing remains deterministic under service-side load, the control task itself is watched by the Task WDT, and no deadline or stack problems are observed. **Partially met**: the mechanism (dedicated task, higher priority than `loopTask`, explicit watchdog subscription, verified core placement) is implemented in [PHASE5_NOTES.md](PHASE5_NOTES.md), but the actual timing/stack/watchdog-recovery numbers this exit condition asks for require bench hardware not available in this environment. Real task concurrency exists for the first time as of this phase; the write-side race Phase 4 §4 flagged as "worth re-examining once Phase 5 makes these genuinely concurrent" (`setpoint`/`steamSetpoint`/PID tunings/`offset`) was found real by independent review and fixed in review round 1 - all four now go through `commandQueue` exclusively. One item was deliberately left for bench verification rather than a blind code fix: `digitalRead(syncPin)`'s AC-detect/AC-off sampling may alias against 50 Hz mains now that the control task samples on an exact, fixed 10 ms grid - see [PHASE5_NOTES.md](PHASE5_NOTES.md) "Considered, not changed," the single highest-priority bench item this phase leaves outstanding.

### Phase 6 — Consider time-proportional SSR output separately

This is a useful behaviour improvement but should not be mixed into the task migration. Its interface should already be designed as a deadman: heater output is a short-lived authorization, not a persistent level.

- [x] Define a safe SSR time window appropriate for the heater and SSR. `SSR_WINDOW_MS = 1000` - a starting point, marked `TODO(bench)`. See [PHASE6_NOTES.md](PHASE6_NOTES.md) §2.
- [x] Convert the full PID output range to an on-time fraction instead of switching at a fixed value of 127. `heaterDemand` now carries `output` (0-255) directly; `controlStep()` derives an on-time fraction of `SSR_WINDOW_MS` from it. Note: achievable resolution is bounded by `SSR_ENFORCE_INTERVAL_MS` (10ms), not the full 8-bit range - 100 duty steps, not 255 - found in review; see [PHASE6_NOTES.md](PHASE6_NOTES.md) §1-2 and "Review round 1" #8.
- [x] Make every window authorization expire automatically at its deadline. `ssrDeadmanCallback()` defaults to off whenever `ssrAuthorizedUntilMs` is not in the future, unconditionally, using a wraparound-safe comparison - see [PHASE6_NOTES.md](PHASE6_NOTES.md) §3.
- [x] Require the control task to re-arm/refresh the next window after each successful control period. Refreshed every `controlStep()` cycle (10ms) - stricter than once per window (1000ms).
- [x] Enforce expiry in a timer/driver path that continues to turn the SSR off if the control task is wedged. `ssrDeadmanCallback()` runs in the `esp_timer` service task - a separate FreeRTOS task from `controlTask`, priority arithmetically confirmed as 22 vs. `controlTask`'s 5 - on its own fixed period, with a heartbeat so the deadman's own absence is observable. Review found and fixed a real gap: this task is shared with the pre-existing buzzer timer, which could previously block it indefinitely - see [PHASE6_NOTES.md](PHASE6_NOTES.md) "Review round 1" #4.
- [x] Never re-arm before current temperature data and all safety checks have passed. The refresh happens at the same point in `controlStep()` the old direct `digitalWrite(SSR_PIN, ...)` used to - after fault/OTA/mode resolution.
- [x] Clear the current authorization immediately on fault, invalid sensor, over-temperature, mode transition, OTA start or shutdown. Fault/invalid-sensor/over-temp force `heaterOut` to 0 (existing fault-priority resolution, now NaN-safe - review found the original comparison direction wasn't); OTA start is a latched flag `controlStep()` honors for the whole session, not a one-shot write - review found the original one-shot version was a no-op in the case that mattered. See [PHASE6_NOTES.md](PHASE6_NOTES.md) §4 and "Review round 1" #5/#7.
- [x] Ensure a stale timestamp, missed refresh or invalid authorization always maps to SSR off. Single check in `ssrDeadmanCallback()` covers all cases; review found and fixed a wraparound bug where a literal `0` "expired" sentinel would have silently meant "authorized" after ~24.9 days of uptime - see [PHASE6_NOTES.md](PHASE6_NOTES.md) "Review round 1" #3.
- [ ] Validate the deadman by deliberately stopping control-task refresh and measuring the maximum time until SSR off. Not bench-tested - no hardware.
- [ ] Validate heater response and retune PID only if measurements show it is necessary. Not done - no bench data exists yet to show it's necessary, per the plan's own guidance.
- [x] Keep this change in a separate commit or pull request. `phase6-time-proportional-ssr`, its own PR.

Exit condition: the SSR uses the available PID resolution (bounded by `SSR_ENFORCE_INTERVAL_MS`, honestly stated above), every authorization expires without refresh, and a stalled control task cannot leave the heater continuously energized. **Mechanism implemented, reviewed and reasoned through** in [PHASE6_NOTES.md](PHASE6_NOTES.md), including a review round that found and fixed a heater-starvation bug in the original window logic, an authorization-design fragility, a millis()-wraparound bug, a shared-task blocking hazard that could have defeated the whole deadman, and an OTA hook that didn't actually do anything; the actual maximum-time-to-off measurement this exit condition asks for still requires bench hardware not available in this environment.

### Phase 7 — Verification

A code-level verification pass against every item below - what the code
actually guarantees vs. what still needs a real machine - is recorded in
`docs/PHASE7_NOTES.md`. An independent 6-agent review of that pass found it
had overstated several verdicts and, more importantly, surfaced two severe
defects unrelated to any single checklist item (a stuck-low thermocouple
reading as a valid 0C and saturating the heater with no fault raised; PID
tunings loaded from `config.json` never actually reaching the running
controller) - both fixed in code this phase, along with three smaller
fixes. Five real, understood gaps remain deliberately undone rather than
guessed at (no invalid-pressure-sensor detection; no independent deadman for
the pump; OTA doesn't transition the pump/shot to a safe state; a
fault-triggered mid-shot restart can command full pump power into an
already-pressurized group; the over-temperature fault doesn't latch) - see
`PHASE7_NOTES.md`'s "Review round 1" for the full reasoning on each.
Checkboxes below stay unchecked: this phase's exit condition requires bench
results this environment cannot produce, and code-level verification is not
a substitute for it - see `PHASE7_NOTES.md` for which items are
logic-guaranteed today and exactly what to run on real hardware to close out
each one.

- [ ] Temp-only heats and regulates temperature while shot logic and pump remain inactive.
- [ ] Normal shot detection enters and exits every intended phase correctly.
- [ ] Pre-infusion, bloom and extraction timing use wall-clock time.
- [ ] Pressure setpoint and pump output behave correctly at phase boundaries.
- [ ] Invalid temperature sensor forces heater and pump safe.
- [ ] Invalid pressure sensor prevents unsafe pump regulation.
- [ ] Over-temperature and all time-outs force safe outputs.
- [ ] Reboot, task failure and watchdog reset leave heater and pump off.
- [ ] A deliberately wedged control task is detected because that task is explicitly subscribed to the Task WDT.
- [ ] Loss of SSR refresh makes the heater output decay to off within the documented maximum window.
- [ ] Mid-shot settings edits do not change the active shot and the latest valid pending revision becomes active after return to `IDLE`.
- [ ] Wi-Fi loss does not disturb control timing.
- [ ] Missing/failing SD card does not disturb control timing.
- [ ] Large web requests/uploads do not disturb control timing.
- [ ] OTA start either refuses while unsafe or transitions the machine to a documented safe state.
- [ ] Telemetry remains coherent and stale snapshots are detectable.

Exit condition: the hardware/bench checklist passes and results are recorded.

### Phase 8 — Cleanup and handoff

- [x] Remove obsolete globals and duplicate actuator paths. Audited every
      global declaration and every `digitalWrite(SSR_PIN, ...)`/
      `light.setBrightness()`/`digitalWrite(BUZZER_PIN, ...)` call site -
      actuator ownership was already fully centralized by Phase 3/6 (one
      write site per actuator, verified by grep, no duplication found).
      Two genuinely dead globals removed: `lastPrintTime` (declared,
      referenced nowhere else - superseded by `loop()`'s own local
      `lastReportMs` since Phase 5) and `brewTemp` (declared, never
      written anywhere, only ever read into a permanently-empty `/getValues`
      field - confirmed unread by the shipped front end before removing).
- [x] Document task boundaries, queue schemas and timing. See
      [ARCHITECTURE.md](ARCHITECTURE.md) - the three execution contexts,
      what may write which actuator/global from where, the two queues'
      schemas and producer/consumer contracts, and a consolidated timing
      constants table.
- [x] Document configuration migration, if any. See ARCHITECTURE.md
      "Configuration persistence" - `config.json`'s schema is unchanged by
      this entire rewrite (confirmed against the shipped front end and
      `loadSDConfig()`/`/saveConfig`); no migration is needed for an
      existing install's config file.
- [x] Add a concise hardware test procedure. See
      [HARDWARE_TEST_PROCEDURE.md](HARDWARE_TEST_PROCEDURE.md) - every bench
      item from every phase's own notes, consolidated into one ordered
      runbook, prioritized starting with the two items Phase 7 identified as
      highest-priority (pressure sensor fault convention, over-temperature
      reachability).
- [x] Update this checklist and the root README. This checklist, above; see
      the root [README.md](../README.md) for the corresponding update.
- [x] Decide whether changes should be proposed upstream as one or several
      focused pull requests. See "Upstream contribution" below - a
      recommendation, not an action taken by this phase; opening a PR
      against a third-party repository needs the user's own explicit
      go-ahead, not something to do unilaterally.

**Upstream contribution.** This repository is a development copy of
[Discreet-Coffee/Discreet](https://github.com/Discreet-Coffee/Discreet), a
real, separately-maintained open-source project (see the root README's
"Credits and license"). Recommendation: **several small, independently
-reviewable PRs, not one large one**, roughly in this order, each of which
already exists as its own reviewed, tagged commit range in this repo's
history and could be cherry-picked/rebased onto upstream's `main` largely
as-is:

1. Phase 1's non-blocking buzzer and loop-count-timing fixes (small,
   behavior-preserving, easiest to review and merge independently).
2. Phase 3's actuator-ownership centralization and fault/mode priority
   resolution (the single-write-site guarantee and the fault-forces-pump
   -off fix - a real bug fix upstream likely wants regardless of the rest
   of this rewrite).
3. The two-task architecture itself (Phases 2, 4, 5, 6 together - these
   depend on each other and don't split further cleanly).
4. Phase 7's five bug fixes (stuck-low thermocouple, PID tunings never
   applied, unclamped boot setpoint, stale `heaterOn` telemetry) - genuinely
   independent of the architecture change, since all five are real bugs in
   logic upstream also has today, not specific to this rewrite's new
   structure. Worth proposing on their own, soonest, regardless of whether
   upstream has any interest in the broader two-task rewrite.

Not recommended for upstream in their current state: anything still marked
as a documented, un-bench-verified gap in `PHASE7_NOTES.md` - those need
this repo's own hardware validation first, not a second team's.

### Phase 9 — Physical controls: power/sleep, steam button, status LEDs

Net-new functionality, not part of the task-migration architecture itself - sequenced after Phase 7/8 because it's built on top of `OperatingMode`/`ControlCommand`/`controlStep()`'s priority-resolution chain and should land on a verified base, not a moving one. Two physical momentary buttons and two status LEDs:

```
#define SLEEP_BUTTON_PIN 27  // toggles a new OFF state
#define STEAM_BUTTON_PIN 14  // toggles steam mode, mirrors the existing web START_STEAM/STOP_STEAM path
#define POWER_LED_PIN    16  // solid=awake, pulsing=asleep, off=OFF - PENDING PSRAM CHECK, see below
#define STEAM_LED_PIN    17  // reflects steamRequested - PENDING PSRAM CHECK, see below
```

**Hardware pre-check, before wiring anything:** confirm whether the physical ESP32 module has PSRAM (WROVER-class). If it does, GPIO16/17 are wired internally to the PSRAM chip and unusable for anything else - fall back to GPIO2 and GPIO15 for the two LEDs instead (both are boot-strapping pins, but low-risk for an LED specifically: a current-limited LED load doesn't typically violate strapping requirements the way a button input or an actuator sharing the same pin could, and a brief flash at boot is cosmetic, not functional - still worth a bench check at boot before trusting it). GPIO27/14 (the buttons) have no such caveat and are confirmed free against every pin this project already uses (SSR 13, dimmer 18, sync 19, thermocouple 21/22/23, SD 25/26/32/33, pressure 34, buzzer 4). GPIO14 is one of the four JTAG pins (MTMS) - harmless for a plain debounced input with JTAG unused, the same category of caveat this project already lives with on GPIO13 (SSR, MTCK).

**Design, confirmed with the user before drafting this checklist:**
- A new `PowerState` enum (`AWAKE`/`SLEEPING`/`OFF`), independent of `OperatingMode` - power/attentiveness is a different axis from brew-mode.
- **OFF** and **SLEEP** are two distinct states, not the same state reached two ways. `SLEEP_BUTTON_PIN` toggles `OFF` directly (manual only - press to fully disable, press again to wake straight to `AWAKE`); a 30-minute idle timeout is the *only* path into `SLEEPING`. `OFF` forces the heater and pump off through the same priority-resolution chain `faulted`/`otaInProgress` already use (Phase 3/6), and also suppresses AC-detect/shot-start the same way `TEMP_ONLY` already does, so pulling the lever while off doesn't try to start a shot underneath a forced-off heater. WiFi/web server/OTA stay up throughout - "keeps the board alive" per the original ask.
- **SLEEP** (auto, 30 min idle) does not disable the control loop - it changes the target: setpoint falls back to 55°C (offset-adjusted, same representation `setpoint` already uses) and steam mode is turned off if active, using the exact same setpoint-swap shape `START_STEAM`/`STOP_STEAM` already have in `applyControlCommand()`. The PID keeps running normally at the new, lower target - this is a standby setback, not a shutdown.
- **Idle timer reset**: any interaction counts, not just physical buttons - web `/adjust`, `/saveConfig`, and steam start/stop all reset it too. Free to implement with zero new plumbing: `applyControlCommand()` is already the single point every one of those passes through (Phase 4's "control owner" design), so it can update `lastActivityMs` once, centrally, rather than needing every caller to remember to.
- **Wake trigger**: physical buttons only (`SLEEP_BUTTON_PIN` or `STEAM_BUTTON_PIN`) - a touch on either restores `setpoint` to `activeSettings.setpoint` (the last real brew target, not a hardcoded value) and returns to `AWAKE`. A web/app command arriving while asleep does not itself wake the machine.
- LED pulsing ("MacBook-style breathing") driven from the existing 10ms control cycle via LEDC PWM, not a separate timer/task - cosmetic, not safety-critical, so (unlike the Phase 6 SSR deadman) there's no reason to pay for independence from `controlTask`'s own health.

**Checklist:**

- [ ] Confirm PSRAM presence on the actual board; finalize LED pin assignment (16/17, or the GPIO2/15 fallback) before wiring.
- [ ] Add `PowerState` (`AWAKE`/`SLEEPING`/`OFF`) as a new enum, and a global tracking it, alongside the existing `OperatingMode`/`ShotState`/`FaultCode` enums.
- [ ] Read `SLEEP_BUTTON_PIN`/`STEAM_BUTTON_PIN` inside `controlStep()`, next to the existing `syncPin` read - not through `commandQueue` (they're not HTTP-originated). Debounce with the same edge-triggered, multi-consistent-sample idiom `AC_OFF_MIN_SAMPLES`/`AC_OFF_DEBOUNCE_MS` already establish, not a new pattern; mark the debounce constants `TODO(bench)` like every other timing constant in this file.
- [ ] `SLEEP_BUTTON_PIN` toggles `PowerState` between `OFF` and `AWAKE` directly (an edge-triggered action, once per physical press, never through `SLEEPING`).
- [ ] `STEAM_BUTTON_PIN` toggles steam mode by constructing the same `START_STEAM`/`STOP_STEAM` `ControlCommand` the web UI's `/adjust` already sends - reuse the existing command, don't add a parallel path into `steamRequested`.
- [ ] Extend `controlStep()`'s fault-priority resolution (where `faulted`/`otaInProgress` already force `heaterOut`/`pumpOut` off) to also force off when `powerState == PowerState::OFF`.
- [ ] Extend the AC-detect gate (currently `currentMode != OperatingMode::TEMP_ONLY`) to also require `powerState != PowerState::OFF`, so a shot can't start while off.
- [ ] `lastActivityMs`, updated once inside `applyControlCommand()` (covers every web-originated command) and once on each physical button press; a 30-minute-elapsed check inside `controlStep()` transitions `AWAKE → SLEEPING` when idle, using the setpoint-swap shape described above.
- [ ] Either physical button, while `SLEEPING`, transitions back to `AWAKE`: restore `setpoint` from `activeSettings.setpoint`, reset `lastActivityMs`.
- [ ] `POWER_LED_PIN`: solid while `AWAKE`, breathing-pulse (LEDC PWM, smooth duty ramp over roughly a 2-3s period - `TODO(bench)` for the exact feel) while `SLEEPING`, off while `OFF`.
- [ ] `STEAM_LED_PIN`: reflects `steamRequested`. Open refinement to confirm with the user, not assumed: blink while warming up (`steamRequested` true, `steaming` still false) and go solid once actually at temperature (`steaming` true) - mirrors the existing buzzer "steam ready" cue - versus a plain on/off tied only to `steamRequested`.
- [ ] Decide and document whether `OFF` survives a reboot/power cycle, or the machine always comes back `AWAKE` after power is restored - not resolved yet, worth a deliberate answer rather than an accidental default.
- [ ] Bench-test debounce behaviour, LED pulse feel, and the full state graph (`AWAKE → SLEEPING → AWAKE`, `AWAKE → OFF → AWAKE`, and that `OFF` cannot be entered from `SLEEPING` by anything other than the explicit button toggle) on real hardware.

Exit condition: both buttons are debounced and reliable, `OFF` fully disables actuators and shot-start while keeping the board reachable over WiFi, the 30-minute idle setback and physical-button wake behave as specified, and both LEDs correctly reflect `PowerState`/steam status including the pulsing effect.

## Session handoff protocol

At the end of every work session, update this file with:

- date and bounded session goal
- branch and final commit hash
- files changed
- tests run and their results
- known risks or unverified assumptions
- exact next step for the following session

Rules for incremental work:

1. Work on one phase or one clearly bounded subtask per branch/commit.
2. Keep refactors separate from behaviour changes.
3. Never leave actuator ownership split between the old loop and the new control task.
4. Do not start a phase until the previous exit condition is met or the exception is written down.
5. Finish each session with code that builds, or clearly mark the branch as work-in-progress and record the last known-good commit.

## Session log

| Date | Phase | Branch/commit | Result | Next step |
|---|---|---|---|---|
| 2026-08-19 | Planning | Initial documentation | Two-task architecture and staged plan recorded | Start Phase 0 and capture the imported baseline |
| 2026-08-19 | Planning | Documentation update | Added explicit Task WDT subscription, SSR deadman contract and latched mid-shot settings | Carry these contracts into Phase 0 safety criteria |
| 2026-08-19 | Phase 0 + 1 | `phase0-1-baseline-and-deblocking`, see PR | Recorded imported commit (`b6ffbae`), pin mapping, safe-boot/fault-behaviour gaps and a `baseline-phase0` tag; confirmed the unmodified baseline only builds against `esp32:esp32@2.0.17` (3.x breaks `dimmable_light`'s timer API usage). Replaced the blocking buzzer and the loop-count `offcount` shot-end debounce with non-blocking/timestamp-driven equivalents; audited remaining blocking calls in web/SD/OTA paths and documented rather than changed them. Both baseline and modified firmware compile cleanly. **No physical hardware was available in this environment**, so the bench test record (temp-only and normal-shot behaviour, safe-boot measurement, `AC_OFF_DEBOUNCE_MS` validation) is an open item, not completed. | Bench-verify this PR's behaviour-preservation claims on real hardware, then start Phase 2 (explicit `OperatingMode`/`ShotState`/settings structs) |
| 2026-08-19 | Phase 0 + 1 review fixes | `phase0-1-baseline-and-deblocking`, see PR | Two independent review efforts landed on this branch and were merged together. One (7-angle model review, verified) found a real regression: the elapsed-time-only AC-off debounce could be satisfied by one sample right after a `loop()` stall, spuriously ending a live shot and restarting it at full pump power; fixed by also requiring a minimum count of actually-observed samples, by explicitly zeroing the pump on shot-end, and by fixing the same loop-count-timing defect in `SetPump()`'s `callCount` (missed by the original Phase 1 pass). The other moved the buzzer's pattern edges onto an ESP one-shot timer independent of `loop()`, closing the "a blocking web/SD/OTA call can stretch or hold a beep" risk that the first pass could only document; it also corrected the actuator-ownership baseline (`/adjust`'s legacy `Pause`/`Resume` write the pump directly) and expanded the blocking-call audit table. Firmware compiles cleanly after merging both. Full detail in `PHASE1_NOTES.md` "Review round 1". | Get sign-off on the merged state, then bench-test temp-only and a normal shot, measure safe boot outputs, verify buzzer patterns, and tune `AC_OFF_DEBOUNCE_MS`/`AC_OFF_MIN_SAMPLES` before starting Phase 2 |
| 2026-08-23 | Phase 0 + 1 tagged, merged | `main`, tag `v2.0.0-alpha` | PR merged to `main` after the review fixes above; tagged and published as a pre-release with notes framing it as the first of several rewrite waves. Bench checklist still outstanding at this point. | Bench-test before Phase 2, or proceed with Phase 2 code/data-model work in parallel per explicit direction (see next row) |
| 2026-08-23 | Phase 2 | `phase2-state-and-data-models`, see PR | Started ahead of the outstanding bench checklist, on explicit direction (see the exception note under Phase 2 above). Added `OperatingMode`/`ShotState`/`FaultCode` enums with `toString()`; `ControlSettings`/`ControlCommand`/`TelemetrySnapshot` value-only structs, all three actually used (not just defined); the `activeSettings`/`shotSettings`/`pendingSettings` latch-and-promote lifecycle for brew setpoint, pressure target and phase durations (a genuine, intentionally-scoped behaviour change - mid-shot edits no longer take effect until the shot returns to `IDLE`); removed Pause/Resume (no UI referenced it). `FAULT` state is observability-only in this phase - it does not yet override actuator outputs, which stays Phase 3's job. Found and fixed a narrow pre-existing `setpoint`/`setpointBoot` offset inconsistency while consolidating them. Also found that the new code broke Arduino's automatic function-prototype generation for the rest of the file; fixed with explicit forward declarations. Compiles cleanly: 905,149 bytes flash (+1,516 vs `v2.0.0-alpha`), 51,260 bytes RAM (+48). Full detail in `PHASE2_NOTES.md`. | Independent model review of this PR, then bench-test everything still outstanding from Phase 0/1/2 before Phase 3 |
| 2026-08-23 | Phase 2 review fixes, tagged, merged | `main`, tag `v2.0.1-beta` | Independent 7-angle review found the settings-lifecycle idle/mid-shot gate used `currentShotState == ShotState::IDLE` (a telemetry label) instead of the real "shot in progress" signal - the `FAULT` override could freeze every settings edit indefinitely while genuinely idle, and a one-iteration `COMPLETE` window could drop an edit for a full extra shot cycle. Fixed by gating on `!acDetected` everywhere. Same root cause let a mid-shot `pumppower` side-effect bypass the settings latch entirely (real pressure transient during a "protected" shot) - fixed by gating it on the same corrected check. A separate bug let pulling the machine mid-warm-up silently cancel a steam request (the new setpoint-mirror writes guarded on `steaming`, a temperature-hysteresis flag, not an actual "steam requested" flag) - fixed with a dedicated `steamRequested` flag, which also fixed a `/getValues`-vs-`/temp` disagreement during steaming and a related contamination risk in `/saveConfig`'s fallback. Compiles cleanly: 905,289 bytes flash (+140 vs the pre-fix commit), 51,260 bytes RAM (unchanged). Full detail in `PHASE2_NOTES.md` "Review round 1". PR merged to `main`; tagged and published as `v2.0.1-beta`. | Bench-test everything outstanding from Phase 0/1/2 (see `BASELINE.md`, `PHASE1_NOTES.md`, `PHASE2_NOTES.md`) before starting Phase 3 |
| 2026-08-23 | Phase 3 | `phase3-centralize-control-ownership`, see PR | Started ahead of the outstanding bench checklist, on the same explicit-direction basis as Phase 2. Introduced `controlStep(now)` as the sole writer of both actuators: `runPID()` and the renamed `updatePumpRamp()` (was `SetPump()`) now only set `heaterDemand`/`pumpDemand`, and the six direct `light.setBrightness()` call sites in the shot-phase branches were replaced with `pumpDemand` assignments - collapsing what was 2 SSR write sites and 8 dimmer write sites down to exactly one of each, verified by grep. Added the output-priority resolution the plan calls for: a temperature fault now forces the pump off too (previously only the heater, with no coupling between a fault and the pump at all - a real gap this closes), continuously for as long as the fault persists; and an explicit mode-restriction layer forces the pump off in temp-only mode even if flipped there mid-shot, closing a second gap that existed since the original baseline. Both are genuine, intentional, safety-positive behaviour changes. `controlStep()` takes `now` as a parameter (matching the plan's own naming) and uses it for its own direct timestamp arithmetic, ahead of Phase 5's periodic-task needs. Compiles cleanly: 905,333 bytes flash (+44 vs `v2.0.1-beta`), 51,268 bytes RAM (+8). Full detail in `PHASE3_NOTES.md`. | Independent model review of this PR, then bench-test everything still outstanding from Phase 0-3 before Phase 4 |
| 2026-08-23 | Phase 3 review fixes | `phase3-centralize-control-ownership`, see PR | Before dispatching the independent review, re-reading the diff found the most severe issue directly: an unconditional `pumpDemand = 0;` every cycle would have defeated `updatePumpRamp()`'s own ~50ms gate, driving the pump mostly off during the steady-state portion of every shot - fixed by removing the reset (matches the pre-Phase-3 `SetPump()` behaviour of simply not writing on ungated iterations). The subsequent 7-angle review found six more, all fixed: neither a fault nor a mid-shot mode change actually stopped the shot state machine, only the final write, so demand could wind up unseen underneath a masked pump and then slam to full power the instant the mask cleared - fixed with a new `endShot()` helper that a fault (or the normal AC-off debounce) both call, aborting the shot rather than leaving it running blind; that same fix closed a reintroduction of the Phase 2 "fault freezes settings edits" bug, since `endShot()` clears `acDetected`. `steam()` (which can block on an unbounded mutex via `queueBuzzer()`) was moved to run after the actuator writes instead of before, so nothing can delay the safety-critical write once a fault is detected. Fault-clearing gained hysteresis (3 consecutive good readings) so an intermittent sensor can't chatter the pump. The mode-restriction check was switched from the raw `PIDonly` bool to the Phase 2 `currentMode` enum designated as the source of truth. Added a defensive `constrain()` clamp at the single actuator-write choke point, and `resolvedHeaterOn`/`resolvedPumpPower` telemetry fields so a bench operator can actually observe a fault or mode restriction taking effect, rather than only seeing the pre-resolution demand. Compiles cleanly: 905,621 bytes flash (+332 vs the pre-fix commit), 51,284 bytes RAM (+24). Full detail in `PHASE3_NOTES.md` "Review round 1". | Get sign-off on the fixes, merge and tag, then bench-test everything outstanding from Phase 0-3 (see `BASELINE.md`, `PHASE1_NOTES.md`, `PHASE2_NOTES.md`, `PHASE3_NOTES.md`) before Phase 4 |
| 2026-08-23 | Phase 3 tagged, merged | `main`, tag `v2.0.2-beta` | PR merged to `main` after the review fixes above; tagged and published as a pre-release. Bench checklist still outstanding at this point, now covering Phase 0-3. | Bench-test everything outstanding (see `BASELINE.md`, `PHASE1_NOTES.md`, `PHASE2_NOTES.md`, `PHASE3_NOTES.md`), or proceed with Phase 4 (safe task communication - queues) per the same explicit-direction basis as Phases 2-3, if directed |
| 2026-08-23 | Phase 4 | `phase4-task-communication`, see PR | Started ahead of the outstanding Phase 0-3 bench checklist, on the same explicit-direction basis as Phases 2-3. Added `commandQueue` (bounded, depth 8) and `telemetryQueue` (length 1) - `handleAdjust()` and `/saveConfig` no longer call `applyControlCommand()`/`acceptBrewSetpointEdit()` or write `PIDonly`/`currentMode` directly; they only construct a `ControlCommand` and `xQueueSend()` it, non-blocking. `applyControlCommand()` (the "control owner") is now called from exactly one place: a queue-drain loop at the top of `controlStep()`, applying every currently-queued command in FIFO order before that cycle's control decisions run - this achieves "latest complete pending revision wins" coalescing for free, since each command reads the current settings base at apply time rather than at submission time, without needing separate dedup logic (see `PHASE4_NOTES.md` for why, and for the one caveat: this reasoning is specific to a single-task drain cadence and may need revisiting once Phase 5 actually splits the tasks apart). Added two new `ControlCommand` types (`SET_BREW_SETPOINT_ABSOLUTE`, `SET_MODE`) to close direct-global-write gaps in `/saveConfig`, including one the Phase 3 review had flagged and deferred (the safety-critical mode check reading a value written with no validation path). `TelemetrySnapshot` widened with `displaySetpoint`/`pendingPreinftime`/`pendingBloomtime`/`pendingPressuresetpoint` so `handleGetValues()` reads everything through `xQueuePeek()` instead of reaching into `pendingSettings`/`setpoint`/`steamRequested` directly. Compiles cleanly: 906,661 bytes flash (+1,040 vs `v2.0.2-beta`), 51,292 bytes RAM (+8). Full detail in `PHASE4_NOTES.md`. | Independent model review of this PR, then bench-test everything still outstanding from Phase 0-4 before Phase 5 |
| 2026-08-23 | Phase 4 review fixes | `phase4-task-communication`, see PR | 7-angle independent review (one gap self-identified beforehand, independently confirmed by four of the angles). Fixed three real issues: `/saveConfig` discarded both `xQueueSend()` return values and always answered "Saved," so a full/uncreated `commandQueue` could leave the just-written `config.json` and the running machine disagreeing with no client-visible signal - fixed by extracting a shared `sendControlCommand()` helper (also now used by `handleAdjust()`) and answering `503` if either send fails, matching the existing `/adjust` behaviour. A mode change to temp-only mid-shot masked the pump's output every cycle but never called `endShot()`, so the shot state machine kept running underneath the mask and could re-arm full pump power, slamming to it the moment the mode flipped back - the same masked-output windup/slam class Phase 3's review fixed for faults, now fixed the same way (`endShot()`) for this new `SET_MODE` path. `SET_BREW_SETPOINT_ABSOLUTE` was the one command type this phase newly routes through the control owner and the one left without a `constrain()` clamp, unlike every sibling case - fixed by clamping it identically. The efficiency angle disassembled the actual ESP32 core objects on this machine and found `buildTelemetrySnapshot()` called `millis()` itself (a ~250-400 cycle 64-bit-divide-backed call) despite `controlStep()` already sampling `now` for exactly this purpose - the single largest new per-cycle cost this phase added - fixed by threading `now` through instead. Considered and deliberately not done: gating the telemetry publish to an interval, which would trade away the same-cycle fault/mode-restriction visibility Phase 3 added those telemetry fields for. Also documented, not fixed (out of this phase's scope): the shipped front end doesn't check the HTTP response status, so the `503` failure mode is currently invisible in the UI; a couple of narrower pre-existing direct-global-read residuals beyond the ones already documented in §4. Compiles cleanly: 906,733 bytes flash (+72 vs the pre-review-fixes commit, net of the `millis()` removal), 51,292 bytes RAM (unchanged). Full detail in `PHASE4_NOTES.md` "Review round 1." | Get sign-off on the fixes, merge and tag, then bench-test everything outstanding from Phase 0-4 (see `BASELINE.md`, `PHASE1_NOTES.md`, `PHASE2_NOTES.md`, `PHASE3_NOTES.md`, `PHASE4_NOTES.md`) before Phase 5 |
| 2026-08-23 | Phase 4 tagged, merged | `main`, tag `v2.0.3-beta` | PR merged to `main` after the review fixes above; tagged and published as a pre-release, directed as the immediate prerequisite for starting Phase 5. Bench checklist still outstanding at this point, now covering Phase 0-4. | Proceed with Phase 5 (create the FreeRTOS control task), per explicit direction |
| 2026-08-23 | Phase 5 | `phase5-control-task`, see PR | Started immediately after the Phase 4 merge/tag above, on explicit direction. Moved `controlStep()` into a new dedicated task (`controlTaskEntry()`, created last in `setup()` via `xTaskCreatePinnedToCore()`), running a fixed 10 ms period; `controlStep()`'s own body did not need to change - `GetPressure()`/`runPID()` already self-gated on `PRESS_INTERVAL`/`PID_INTERVAL` via their own `millis()` checks, so this really was the relocation Phase 4 set up for, not a redesign. Chose and documented core 1 at priority 5, and a 4096-byte stack. Explicitly subscribed the task to the Task WDT, feeding it only after `controlStep()` returns. Added Serial-only diagnostics. `loop()` no longer calls `controlStep()` directly. While implementing, re-examined the Phase 4 §4 "worth re-examining once Phase 5 makes these genuinely concurrent" note and found it real: `setpoint`/`steamSetpoint`/PID tunings were being written from `loopTask` while read from the genuinely concurrent `controlTask` for the first time - written down as a named target for review rather than guessed at blind. Compiles cleanly: 907,529 bytes flash (+796 vs `v2.0.3-beta`), 51,308 bytes RAM (+16). Full detail in `PHASE5_NOTES.md`. | Independent model review of this PR - in particular the self-identified concurrency question - then apply fixes |
| 2026-08-23 | Phase 5 review fixes | `phase5-control-task`, see PR | 7-angle independent review confirmed the self-identified concurrency question from four separate angles and judged it belonged in this PR (the infrastructure to fix it - `commandQueue`/`sendControlCommand()` - was already built by Phase 4). Fixed by adding three `ControlCommand` types (`SET_TUNINGS`, `SET_STEAM_SETPOINT`, `SET_OFFSET`) so Kp/Ki/Kd/`myPID.SetTunings()`/`offset`/`steamSetpoint` are now written only inside `applyControlCommand()` (`controlTask`), never from `/saveConfig` (`loopTask`) directly; also removed `/saveConfig`'s racy fallback reads of `pendingSettings.setpoint`/`PIDonly` entirely by simply not sending a command for a field the client's JSON omits, rather than reconstructing "the current value" from control-owned state - review traced a concrete path from the old fallback's torn read to a silent `NaN` setpoint with no fault raised. Also fixed: control-task creation failure now forces `ESP.restart()` instead of logging and continuing with no control loop and nothing watching it (the machine would have looked alive via `/getValues`'s fallback while being neither sensed nor actuated); a failed Task WDT subscription no longer floods Serial at 100 Hz forever; deadline-miss detection now uses `xTaskDelayUntil()`'s own return value instead of re-deriving the same condition less precisely; `updatePumpRamp()`'s pressure-ramp gate used strict `>`, which never fires at exactly 50 ms once sampled on the new exact 10 ms grid - stretched its cadence to 60 ms (and the nested pump-ramp-rate gate to 240 ms instead of 200) until fixed to `>=`; the diagnostics `Serial.printf()` moved out of the control task into `loop()` (it was long enough to hit `Print::printf()`'s malloc fallback and could block on the UART, both inside the highest-priority task, and was excluded from its own cycle-time measurement by running after the mark), with the worst-cycle counter now reset per report window instead of latching a single early spike forever; stack headroom was mislabeled "words" instead of bytes and would have been read 4x too generously; `handleGetValues()`'s `actime` field now sources `snap.elapsedShotTimeMs` instead of a raw, never-reset, unsynchronized global. One finding was deliberately left for bench verification rather than a blind code fix: two independent review angles found that `digitalRead(syncPin)`'s AC-detect/AC-off sampling may alias against 50 Hz mains now that the control task samples on an exact, fixed 10 ms grid (mains zero-crossings also recur every 10 ms) - every code-level mitigation considered either only reduces rather than eliminates the risk or requires circuit knowledge not available without hardware; flagged as the first bench item to verify, ahead of timing/stack. Compiles cleanly: 908,053 bytes flash (+524 vs the pre-fix commit), 51,308 bytes RAM (unchanged). Full detail in `PHASE5_NOTES.md` "Review round 1." | Get sign-off on the fixes, merge and tag, then bench-test everything outstanding from Phase 0-5 (see `BASELINE.md` through `PHASE5_NOTES.md`) before Phase 6 - the mains zero-cross sampling question first |
| 2026-08-23 | Phase 5 tagged, merged | `main`, tag `v2.0.4-beta` | PR merged to `main` after the review fixes above; tagged and published as a pre-release, directed as part of the same explicit instruction that started this phase ("go ahead ... code review with Opus and merge just like the others"). Bench checklist still outstanding at this point, now covering Phase 0-5, with the zero-cross sampling question flagged as the first thing to check before trusting shot detection on real mains. | Bench-test everything outstanding (see `BASELINE.md` through `PHASE5_NOTES.md`), zero-cross sampling first, or proceed with Phase 6 (time-proportional SSR output) if directed |
| 2026-08-23 | Phase 6 | `phase6-time-proportional-ssr`, see PR | Started immediately after Phase 5's merge/tag, on explicit direction. `heaterDemand` changed from a `bool` (`output >= 127` threshold) to the raw PID output (0-255); `controlStep()` converts it into an on-time fraction of a fixed `SSR_WINDOW_MS` (1000ms) window, sampled once per window and reset immediately (not just masked) whenever `heaterOut` is 0, the same "abort, don't mask" principle Phase 3 applied to the shot state machine. `controlStep()` no longer writes `SSR_PIN` directly at all - it only refreshes an "authorization" (`ssrAuthorizedOn`/`ssrAuthorizedUntilMs`, plain word-sized globals, no queue needed) every cycle (10ms). A new, genuinely independent `esp_timer` task (`ssrDeadmanCallback()`, `ESP_TIMER_TASK` dispatch matching the existing `buzzerTimer` pattern, its own fixed `SSR_ENFORCE_INTERVAL_MS` = 20ms period) is now the *only* thing that writes `SSR_PIN` - it forces the pin off within `SSR_AUTH_TIMEOUT_MS` (200ms) of the control task's last successful cycle if the authorization isn't refreshed, independent of the Task WDT's own 5s recovery window and of whether `controlTask` is wedged, crashed, or starved. `ArduinoOTA.onStart()` explicitly zeroes the authorization so OTA sessions clear heater authority immediately rather than waiting for the timeout. True ISR dispatch was considered and deliberately not used - reasoned through in `PHASE6_NOTES.md` §3, since an ESP32 flash write already disables interrupts on both cores regardless of dispatch method (a known Phase 5 finding), so it wouldn't add protection against that specific case anyway. PID retuning explicitly not attempted, per the plan's own "only if measurements show it is necessary" guidance - no bench data exists yet. Compiles cleanly: 908,637 bytes flash (+584 vs `v2.0.4-beta`), 51,340 bytes RAM (+32). Full detail in `PHASE6_NOTES.md`. | Independent model review of this PR, then apply fixes |
| 2026-08-23 | Phase 6 review fixes | `phase6-time-proportional-ssr`, see PR | 7-angle independent review found the first draft's window-reset logic could starve the heater indefinitely: resetting the window on every `heaterOut<=0` cycle (modeled on `endShot()`'s "abort don't mask") meant the on-time was never recomputed if demand returned within the same ~10ms, because the rollover gate needs a full `SSR_WINDOW_MS` to elapse - three angles traced this to a worst case where PID output touching 0 more than once a second (routine hunting near setpoint) could permanently freeze the heater at 0% duty with no fault raised. Fixed by decoupling the window's own schedule (rolls regardless of demand) from the authorization gate (`heaterOut > 0`, checked separately). Also fixed: the original two-field authorization (`ssrAuthorizedOn` bool + a deadline) had an unenforced write-ordering requirement - collapsed to a single deadline field, where "off" is `now`, not the literal `0` the original OTA-clear hook used - review found `0` silently reads as "authorized ~24 days from now" once `millis()` wraps past 2^31, a real latch-on risk after ~24.9 days of uptime. The deadman shared the esp_timer service task with the pre-existing buzzer timer, whose unbounded mutex wait could block the deadman indefinitely if `controlTask` wedged while holding the same mutex - found by three angles as directly defeating this phase's central "independent of the control task" claim; fixed by bounding that wait. The OTA-start clear itself was a no-op in the case that mattered - a one-shot write `controlStep()` re-armed within one 10ms cycle - fixed with a latched `otaInProgress` flag the fault-priority chain now honors for the whole session. A `heaterOut <= 0` gate is not NaN-safe (every NaN comparison is false, so it would have let a non-finite PID output fall through to the *heat* branch); inverted to `heaterOut > 0`. `SSR_PIN` was left floating for the majority of boot (tens of seconds, behind WiFi/SD/OTA setup) on a pin that reverts to floating on any warm reset; moved `pinMode`/`initSsrDeadman()` to the very start of `setup()`. Tightened `SSR_ENFORCE_INTERVAL_MS` from 20ms to 10ms and documented honestly that it - not the full 8-bit PID range - is what actually bounds achievable output resolution and fault-to-pin latency. Added a deadman heartbeat so the esp_timer task's own absence would be observable. Considered and not changed: sharing a small `esp_timer`-creation helper between the buzzer and the deadman (cosmetic); restarting rather than logging on deadman init failure (kept, for consistency with Phase 5's control-task-creation precedent); true ISR/hardware-only enforcement (more risk than this environment could validate, and wouldn't survive an OTA flash write regardless); the esp_timer task's core affinity, whose priority was confirmed by header arithmetic (22) but whose core placement lives in a precompiled library this environment can't inspect. Compiles cleanly: 909,177 bytes flash (+540 vs the pre-fix commit), 51,340 bytes RAM (unchanged). Full detail in `PHASE6_NOTES.md` "Review round 1." | Get sign-off on the fixes, merge and tag, then bench-test everything outstanding from Phase 0-6 before Phase 7 - zero-cross sampling (Phase 5) and deadman validation (this phase) both need real hardware and neither has been checked yet |
| 2026-08-23 | Phase 6 tagged, merged | `main`, tag `v2.0.5-beta` | PR merged to `main` after the review fixes above; tagged and published as a pre-release, directed as part of the same explicit instruction that started this phase. Bench checklist still outstanding at this point, now covering Phase 0-6, with zero-cross sampling (Phase 5) and deadman timing validation (this phase) both flagged as needing real hardware before either should be trusted. | Bench-test everything outstanding (see `BASELINE.md` through `PHASE6_NOTES.md`), or proceed with Phase 7 (verification) if directed |
| 2026-08-25 | Phase 7 | `phase7-verification`, see PR | Started on explicit direction. Phase 7 is itself a hardware/bench checklist with no code to write - this environment has no physical machine, so the PR instead did a rigorous code-level verification pass against all 16 checklist items: cite the exact logic guaranteeing each one, or identify it as inherently a physical measurement, or - the outcome this pass exists to catch - identify that the code doesn't actually satisfy it. Found two real gaps this way and documented both rather than guessing at a fix: no invalid-pressure-sensor detection (no equivalent of the thermocouple's NaN/range check exists for the analog pressure input, and the ADC path can't naturally surface "invalid" the way MAX6675 can); no independent deadman for the pump the way Phase 6 built one for the heater (the pump only has the Task WDT as a backstop for a wedged control task). Docs-only; no `Discreet.ino` behavior changed. Full detail in `PHASE7_NOTES.md`. | Independent review of this pass (auditing the verification claims themselves, not a traditional diff), then apply fixes |
| 2026-08-25 | Phase 7 review fixes | `phase7-verification`, see PR | 6-agent independent review (items 1-8 and 9-16 each fresh-audited against the code rather than against the notes, a hunt for a further gap, an altitude challenge on the two "document, don't fix" calls, a diff-scan for citation accuracy, and a front-end cross-check) found the first draft had overstated several verdicts and, more significantly, surfaced two severe defects unrelated to any single checklist item - fixed: `loadSDConfig()` set the `Kp`/`Ki`/`Kd` globals from `config.json` but never called `myPID.SetTunings()` (verified against `PID_v1.cpp`: the PID's internal tunings are captured once at construction from the compile-time defaults and never re-read after), so every boot silently regulated on 48/8/50 regardless of the saved config while `/getValues` reported the unused config values; and a stuck-low MAX6675 interface (verified against `max6675.cpp`) reads back a valid, unflagged `0.00°C` - `isnan()` only catches the chip's own open-thermocouple bit, and the existing `input < 0` guard was dead code - saturating the heater to 100% duty with no fault raised and no other backstop. Also fixed: `/saveConfig`'s setpoint clamp didn't survive a reboot; `heaterOn` over `/getValues` could report stale "on" forever after a control-task wedge; a stale comment describing pre-Phase-5 ordering. One reviewer's claim that the Task WDT only logs and doesn't reboot on this target was checked directly against the real `esp32:esp32@2.0.17` sdkconfig (`CONFIG_ESP_TASK_WDT_PANIC=y`, `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y`) and refuted, matching Phase 5's own precedent for verifying ESP-IDF internals rather than assuming. Three more real gaps documented, not fixed, each requiring a decision this environment can't make blindly: OTA doesn't transition the pump/shot to a safe state (only the heater is masked, and an OTA flash write stalls the control task entirely); a fault-triggered mid-shot restart can command full pump power into an already-pressurized group; the over-temperature fault doesn't latch (self-clears the same way an intermittent-sensor debounce does). Compiles cleanly: 909,345 bytes flash (+168 vs the pre-review commit), 51,340 bytes RAM (unchanged). Full detail in `PHASE7_NOTES.md` "Review round 1." | Get sign-off on the fixes, merge and tag, then bench-test everything outstanding from Phase 0-7 - the pressure sensor's fault convention and whether a genuine over-temperature condition is reachable both take priority over the earlier zero-cross/deadman-timing items |
| 2026-08-25 | Phase 7 tagged, merged | `main`, tag `v2.0.6-beta` | PR merged to `main` after the review fixes above; tagged and published as a pre-release, directed as part of the same explicit instruction that started this phase. Phase 7's own exit condition (hardware/bench checklist passes) is explicitly not met by this tag - this environment has no physical machine to run it against. Bench checklist still outstanding, now covering Phase 0-7, with the two Phase-7-identified gaps (pressure sensor fault convention, over-temperature reachability) flagged as the new top priority. | Bench-test everything outstanding (see `BASELINE.md` through `PHASE7_NOTES.md`), or proceed with Phase 8 (cleanup and handoff) if directed |
