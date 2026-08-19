# Two-task rewrite plan

Status: in progress  
Current phase: Phase 0/1 complete pending hardware bench verification — see [BASELINE.md](BASELINE.md) and [PHASE1_NOTES.md](PHASE1_NOTES.md); next up is Phase 2  
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
- [x] Record pin mapping and hardware variants used for testing. See [BASELINE.md](BASELINE.md).
- [x] Document safe boot outputs: heater off and pump off until initialized. Documented from source; **not yet bench-verified** — see [BASELINE.md](BASELINE.md) "Safe boot outputs".
- [x] Document fault behaviour for invalid temperature, invalid pressure, over-temperature, time-out and watchdog reset. See [BASELINE.md](BASELINE.md) "Fault behaviour"; several gaps identified and carried forward, not fixed in Phase 0.
- [x] Create a baseline tag or branch. Tag `baseline-phase0`.
- [ ] Capture a short test record of current temp-only and normal-shot behaviour. **Not done** — no physical hardware was available in this environment. This remains open and must be completed on a bench before Phase 1+ firmware runs on real hardware.

Exit condition: **partially met**. The unmodified baseline builds and its essential behaviour is documented from source, but the bench test record is an open item, not a completed one — see [BASELINE.md](BASELINE.md) for the full list of open risks.

### Phase 1 — Remove blocking and loop-count timing

- [x] Replace buzzer delays with a timestamp-driven non-blocking sequencer. See [PHASE1_NOTES.md](PHASE1_NOTES.md).
- [x] Replace loop-count-based timing such as `offcount` with elapsed time or a timestamp derived from the relevant signal. See [PHASE1_NOTES.md](PHASE1_NOTES.md); the new debounce constant needs bench confirmation.
- [x] Audit web, SD and OTA paths for blocking calls that could affect migration. See [PHASE1_NOTES.md](PHASE1_NOTES.md); documented, not changed — deferred to later phases.
- [x] Keep machine behaviour unchanged in this phase. Preserved by construction (see [PHASE1_NOTES.md](PHASE1_NOTES.md) "Behaviour-preservation checklist"); not yet bench-confirmed.
- [ ] Re-run the baseline tests. **Not done** — same hardware limitation as Phase 0.

Exit condition: **partially met**. No control behaviour depends on loop iteration speed any more, and the firmware builds; bench re-testing is still required before deployment.

### Phase 2 — Introduce explicit state and data models

- [ ] Add `OperatingMode` with at least normal brew and temp-only.
- [ ] Add `ShotState`: `IDLE`, `PREINFUSION`, `BLOOM`, `EXTRACTION`, `COMPLETE`, `FAULT`.
- [ ] Remove Pause/Resume endpoints, variables and UI assumptions.
- [ ] Define value-only `ControlSettings`, `ControlCommand` and `TelemetrySnapshot` structs.
- [ ] Define allowed state transitions and entry/exit actions.
- [ ] Define settings ranges, units and revision handling.
- [ ] Define `activeSettings`, immutable per-shot `shotSettings` and latest-value `pendingSettings`.
- [ ] Define the atomic `pendingSettings -> activeSettings` promotion on return to `IDLE`.
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
- [ ] Queue valid mid-shot edits as pending without changing `shotSettings`.
- [ ] Coalesce repeated edits so the latest complete pending revision wins.
- [ ] Test command bursts and slow/unavailable web clients.
- [ ] Confirm no shared mutable control globals remain.

Exit condition: all cross-boundary data is copied through defined messages.

### Phase 5 — Create the FreeRTOS control task

- [ ] Move `controlStep()` into one dedicated task.
- [ ] Start with a 10 ms period using `vTaskDelayUntil()`.
- [ ] Schedule 50 ms pressure work and 250 ms temperature/PID work inside the task.
- [ ] Keep zero-cross dimmer timing in the library interrupt path.
- [ ] Choose task priority, core affinity and stack size deliberately; document the reason.
- [ ] Explicitly subscribe the control task to the ESP Task Watchdog with `esp_task_wdt_add(NULL)` from inside that task; do not rely on the watched idle tasks.
- [ ] Feed the Task WDT with `esp_task_wdt_reset()` only after a complete successful control cycle, including safety evaluation and actuator/deadman refresh.
- [ ] Check and handle the return values of the Task WDT API calls.
- [ ] Measure worst observed cycle time and deadline misses.
- [ ] Measure stack high-water mark.
- [ ] Verify that deliberately wedging the control task triggers the intended watchdog recovery and leaves outputs safe.
- [ ] Stress Wi-Fi, web requests, SD access and OTA preparation during a simulated/bench shot.

Exit condition: control timing remains deterministic under service-side load, the control task itself is watched by the Task WDT, and no deadline or stack problems are observed.

### Phase 6 — Consider time-proportional SSR output separately

This is a useful behaviour improvement but should not be mixed into the task migration. Its interface should already be designed as a deadman: heater output is a short-lived authorization, not a persistent level.

- [ ] Define a safe SSR time window appropriate for the heater and SSR.
- [ ] Convert the full PID output range to an on-time fraction instead of switching at a fixed value of 127.
- [ ] Make every window authorization expire automatically at its deadline.
- [ ] Require the control task to re-arm/refresh the next window after each successful control period.
- [ ] Enforce expiry in a timer/driver path that continues to turn the SSR off if the control task is wedged.
- [ ] Never re-arm before current temperature data and all safety checks have passed.
- [ ] Clear the current authorization immediately on fault, invalid sensor, over-temperature, mode transition, OTA start or shutdown.
- [ ] Ensure a stale timestamp, missed refresh or invalid authorization always maps to SSR off.
- [ ] Validate the deadman by deliberately stopping control-task refresh and measuring the maximum time until SSR off.
- [ ] Validate heater response and retune PID only if measurements show it is necessary.
- [ ] Keep this change in a separate commit or pull request.

Exit condition: the SSR uses the available PID resolution, every authorization expires without refresh, and a stalled control task cannot leave the heater continuously energized.

### Phase 7 — Verification

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
| 2026-08-19 | Planning | Documentation update | Added explicit Task WDT subscription, SSR deadman contract and latched mid-shot settings | Carry these contracts into Phase 0 safety criteria |
| 2026-08-19 | Phase 0 + 1 | `phase0-1-baseline-and-deblocking`, see PR | Recorded imported commit (`b6ffbae`), pin mapping, safe-boot/fault-behaviour gaps and a `baseline-phase0` tag; confirmed the unmodified baseline only builds against `esp32:esp32@2.0.17` (3.x breaks `dimmable_light`'s timer API usage). Replaced the blocking buzzer and the loop-count `offcount` shot-end debounce with non-blocking/timestamp-driven equivalents; audited remaining blocking calls in web/SD/OTA paths and documented rather than changed them. Both baseline and modified firmware compile cleanly. **No physical hardware was available in this environment**, so the bench test record (temp-only and normal-shot behaviour, safe-boot measurement, `AC_OFF_DEBOUNCE_MS` validation) is an open item, not completed. | Bench-verify this PR's behaviour-preservation claims on real hardware, then start Phase 2 (explicit `OperatingMode`/`ShotState`/settings structs) |
