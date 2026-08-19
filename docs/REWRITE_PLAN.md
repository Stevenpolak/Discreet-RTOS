# Two-task rewrite plan

Status: planning  
Current phase: Phase 0 — baseline and safety  
Architecture target: one Arduino service loop plus one dedicated FreeRTOS control task

This checklist is designed for work spread across multiple sessions. Finish and document one bounded step at a time; do not combine the architecture migration with unrelated behaviour changes.

## Target architecture

### Arduino `loop()`: service side

Owns work that may be delayed briefly without affecting machine control:

- Wi-Fi, mDNS and the web server
- OTA handling
- SD-card file management and uploads
- Non-blocking buzzer sequencing
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

- [ ] Record the exact imported upstream commit/tag.
- [ ] Build the current firmware unchanged.
- [ ] Record ESP32 board/core version and all library versions.
- [ ] Record pin mapping and hardware variants used for testing.
- [ ] Document safe boot outputs: heater off and pump off until initialized.
- [ ] Document fault behaviour for invalid temperature, invalid pressure, over-temperature, time-out and watchdog reset.
- [ ] Create a baseline tag or branch.
- [ ] Capture a short test record of current temp-only and normal-shot behaviour.

Exit condition: the unmodified baseline builds and its essential behaviour is recorded.

### Phase 1 — Remove blocking and loop-count timing

- [ ] Replace buzzer delays with a timestamp-driven non-blocking sequencer.
- [ ] Replace loop-count-based timing such as `offcount` with elapsed time or a timestamp derived from the relevant signal.
- [ ] Audit web, SD and OTA paths for blocking calls that could affect migration.
- [ ] Keep machine behaviour unchanged in this phase.
- [ ] Re-run the baseline tests.

Exit condition: the current single-loop firmware remains functionally equivalent but no control behaviour depends on loop iteration speed.

### Phase 2 — Introduce explicit state and data models

- [ ] Add `OperatingMode` with at least normal brew and temp-only.
- [ ] Add `ShotState`: `IDLE`, `PREINFUSION`, `BLOOM`, `EXTRACTION`, `COMPLETE`, `FAULT`.
- [ ] Remove Pause/Resume endpoints, variables and UI assumptions.
- [ ] Define value-only `ControlSettings`, `ControlCommand` and `TelemetrySnapshot` structs.
- [ ] Define allowed state transitions and entry/exit actions.
- [ ] Define settings ranges, units and revision handling.
- [ ] Add human-readable fault codes.

Exit condition: modes, states, settings, commands and telemetry have explicit definitions without yet requiring a second task.

### Phase 3 — Centralize control ownership

- [ ] Move coffee logic behind one `controlStep(now)` boundary.
- [ ] Make this boundary the only writer of pump/dimmer and heater/SSR outputs.
- [ ] Route web changes through validated requests instead of direct global-variable or actuator writes.
- [ ] Apply output priority: fault/safety off, mode restrictions, state-machine demand, controller output.
- [ ] Verify that temp-only cannot enable the pump.
- [ ] Verify that fault handling always forces safe outputs.

Exit condition: a single function owns all control decisions and actuator writes while still running in the original execution context.

### Phase 4 — Add safe task communication

- [ ] Create the bounded command/settings queue.
- [ ] Create the length-1 telemetry queue.
- [ ] Publish telemetry with `xQueueOverwrite()`.
- [ ] Read telemetry with `xQueuePeek()`.
- [ ] Ensure queue operations used by either task are non-blocking or tightly bounded.
- [ ] Reject unsafe or invalid settings inside the control owner.
- [ ] Test command bursts and slow/unavailable web clients.
- [ ] Confirm no shared mutable control globals remain.

Exit condition: all cross-boundary data is copied through defined messages.

### Phase 5 — Create the FreeRTOS control task

- [ ] Move `controlStep()` into one dedicated task.
- [ ] Start with a 10 ms period using `vTaskDelayUntil()`.
- [ ] Schedule 50 ms pressure work and 250 ms temperature/PID work inside the task.
- [ ] Keep zero-cross dimmer timing in the library interrupt path.
- [ ] Choose task priority, core affinity and stack size deliberately; document the reason.
- [ ] Measure worst observed cycle time and deadline misses.
- [ ] Measure stack high-water mark.
- [ ] Verify watchdog behaviour and safe recovery.
- [ ] Stress Wi-Fi, web requests, SD access and OTA preparation during a simulated/bench shot.

Exit condition: control timing remains deterministic under service-side load and no deadline or stack problems are observed.

### Phase 6 — Consider time-proportional SSR output separately

This is a useful behaviour improvement but should not be mixed into the task migration.

- [ ] Define a safe SSR time window appropriate for the heater and SSR.
- [ ] Convert the full PID output range to an on-time fraction instead of switching at a fixed value of 127.
- [ ] Ensure over-temperature, invalid sensor and fault states override the window immediately.
- [ ] Validate heater response and retune PID only if measurements show it is necessary.
- [ ] Keep this change in a separate commit or pull request.

Exit condition: the SSR uses the available PID resolution and safety-off behaviour is unchanged or stronger.

### Phase 7 — Verification

- [ ] Temp-only heats and regulates temperature while shot logic and pump remain inactive.
- [ ] Normal shot detection enters and exits every intended phase correctly.
- [ ] Pre-infusion, bloom and extraction timing use wall-clock time.
- [ ] Pressure setpoint and pump output behave correctly at phase boundaries.
- [ ] Invalid temperature sensor forces heater and pump safe.
- [ ] Invalid pressure sensor prevents unsafe pump regulation.
- [ ] Over-temperature and all time-outs force safe outputs.
- [ ] Reboot, task failure and watchdog reset leave heater and pump off.
- [ ] Wi-Fi loss does not disturb control timing.
- [ ] Missing/failing SD card does not disturb control timing.
- [ ] Large web requests/uploads do not disturb control timing.
- [ ] OTA start either refuses while unsafe or transitions the machine to a documented safe state.
- [ ] Telemetry remains coherent and stale snapshots are detectable.

Exit condition: the hardware/bench checklist passes and results are recorded.

### Phase 8 — Cleanup and handoff

- [ ] Remove obsolete globals and duplicate actuator paths.
- [ ] Document task boundaries, queue schemas and timing.
- [ ] Document configuration migration, if any.
- [ ] Add a concise hardware test procedure.
- [ ] Update this checklist and the root README.
- [ ] Decide whether changes should be proposed upstream as one or several focused pull requests.

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
