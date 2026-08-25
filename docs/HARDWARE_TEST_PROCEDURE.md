# Hardware test procedure

Every phase of this rewrite (Phase 0 through Phase 8) was implemented and
reviewed without access to physical hardware. Nothing in `v2.0.6-beta` (or
this phase) has been bench-verified. This is a single, ordered procedure
consolidating every outstanding bench item named across every phase's own
notes into one runbook - what to do, in what order, and what a pass/fail
result looks like. It supersedes running each phase's bench checklist
separately; the phase notes remain the place to go for *why* each item
exists and what code backs it.

**Do not power a real boiler/pump from this firmware before completing at
least Group 1.** Bench with the heater/pump disconnected or current-limited
where practical until sensor-fault and over-temperature behavior are
confirmed.

**Query `/getValues` directly (curl or browser devtools) for every step
below that says "confirm" a fault, mode restriction, or actuator state -
never trust the shipped dashboard for this.** `PHASE7_NOTES.md`'s front-end
observability note found the dashboard reads `pumppower` (the
pre-resolution demand) instead of `resolvedPumpPower`, never surfaces
`fault`/`mode`/`shotState` at all, and stamps its chart with the browser's
own clock rather than `snapshotTimestampMs` - so a frozen, stale snapshot
renders as if it were live. This isn't a one-off caveat on a single step; it
applies everywhere below that asks you to confirm firmware state, not just
step 19.

## Group 1 — before connecting mains-powered hardware at all

1. **Identify the pressure sensor's fault-signaling convention.** Find its
   datasheet/part number. Does it use a "live zero" band (output pinned
   below/above its normal working range on a fault)? Record the answer in
   `PHASE7_NOTES.md` Gap 1, whichever way it goes. This is a datasheet/bench
   item, not a code item - `PHASE7_NOTES.md` Gap 1 already established that
   a bounds check on the pressure reading is provably dead code without it
   (the raw ADC value is already mathematically confined to a range that
   overlaps valid readings) - but it's still the first thing to resolve,
   since Gap 5 (below) is the only other item ranked at the same priority.
2. **Confirm a genuine over-temperature condition is reachable in
   practice**, given this machine's actual heater sizing and boiler thermal
   mass. If it is, decide (before wiring the boiler) what "latched until
   cleared" should mean for `FaultCode::INVALID_TEMPERATURE` when the cause
   is genuine over-temperature rather than a flapping sensor - see
   `PHASE7_NOTES.md` Gap 5. Ranked alongside item 1, not below it.
3. **Safe boot outputs.** Power on with a multimeter/logic probe on
   `SSR_PIN` (13) and the dimmer gate line (18). Confirm both read off
   through the entire boot sequence, including the ~2s initial delay and
   Wi-Fi/SD retry loops. Repeat across a cold power-cycle and a warm
   `ESP.restart()` (trigger one from Serial or by unplugging/replugging SD
   mid-read).
4. **Thermocouple fault paths.** With the boiler cold: disconnect the
   thermocouple lead (open-circuit) and confirm the fault trips and both
   actuators go safe. Separately, short or otherwise force a stuck-low read
   (simulating lost sensor power) and confirm `MIN_PLAUSIBLE_TEMP_C` (Phase 7)
   actually catches it - this is the one fault path this rewrite added
   without being able to test it. Reconnect and confirm the fault clears
   after 3 good readings (~750ms) and heat resumes.
5. **Over-temperature trip.** With the boiler powered and a way to safely
   drive `input` toward 160C (or a controlled heat source), confirm the
   fault trips at 160C. Given Gap 5, also confirm and document what actually
   happens next - does it re-trip repeatedly, and is that acceptable for
   this specific machine, pending whatever Gap 5's own decision (item 2
   above) settles on?
6. **Heater response and PID tuning** (`PHASE6_NOTES.md` bench item 1;
   `REWRITE_PLAN.md`'s own unchecked Phase 6 box: "retune PID only if
   measurements show it is necessary"). With the boiler heating normally
   under closed-loop control, observe actual temperature response -
   overshoot, settle time, steady-state ripple. This is now more load
   -bearing than when Phase 6 first listed it: Phase 7's review found
   `loadSDConfig()` never applied `config.json`'s tunings before this
   rewrite (fixed - see `PHASE7_NOTES.md` "Review round 1" item 2), meaning
   **no one has ever observed this machine actually running on the gains it
   reports running.** Only retune if this observation shows a real problem,
   per the plan's own guidance - do not retune blind.

## Group 2 — shot detection and timing (needs mains AC on `syncPin`)

7. **Mains zero-cross aliasing** (`PHASE5_NOTES.md`, ranked just behind
   Group 1's two items). `controlTask` samples `syncPin` on an exact 10ms
   grid, which can alias against 50Hz mains zero-crossings (also ~10ms
   apart). Run several shot start/stop cycles and confirm AC detection and
   AC-off debounce both fire reliably, not just once. If detection is
   flaky or the debounce fires early/late, this needs a hardware-level fix
   (see that phase's "Considered, not changed" section for what was ruled
   out and why).
8. **Temp-only mode.** Confirm the boiler heats and regulates while shot
   logic and pump both stay inactive throughout.
9. **A full normal shot.** Confirm pre-infusion, bloom and extraction all
   enter and exit at the right wall-clock boundaries, and that pump output
   matches each phase's target. Also confirm `basePumpPowerForSetpoint()`'s
   table produces sensible pump levels against the real pump/boiler -
   `PHASE7_NOTES.md` item 4 calls its magnitude "a bench-tuning question,"
   not yet checked.
10. **`AC_OFF_DEBOUNCE_MS`/`AC_OFF_MIN_SAMPLES` tuning** (`TODO(bench)`
    since Phase 1; re-scoped by Phase 5). The original Phase 1 concern was
    brief zero-cross artifacts on `syncPin` false-triggering an early shot
    end - not a pressure or timing-side issue. Phase 5's control-task move
    changed what's actually being risked: `syncPin` is now sampled on a
    fixed 100Hz grid (`controlTask`'s 10ms cycle) instead of whatever rate
    `loop()` happened to reach it at, so `AC_OFF_MIN_SAMPLES` (20) now
    consumes a full 200ms of the 300ms `AC_OFF_DEBOUNCE_MS` budget, where it
    used to complete in a few milliseconds at `loop()`'s old, much faster
    rate. Confirm a real shot-end is still detected promptly under the new
    timing, and that no ordinary zero-cross artifact ends a shot early.
11. **Fault-triggered restart into a pressurized group** (`PHASE7_NOTES.md`
    Gap 4). With the paddle held continuously, force a transient thermocouple
    fault mid-extraction (see step 4) and observe what the pump actually does
    on the immediate restart - does the group stay pressurized through the
    fault, and does the pump's 500ms boost-to-255 behavior on restart cause
    a problem in practice? This determines whether Gap 4 needs a real fix.

## Group 3 — the independent safety mechanisms

12. **SSR deadman timing** (`PHASE6_NOTES.md` bench item 2). With the heater
    authorized on, kill `controlTask` (a debug build with an intentional
    infinite loop behind a flag is easiest) and time how long `SSR_PIN`
    stays driven before the deadman forces it off. Expect ~230ms (see
    `ARCHITECTURE.md`'s timing table for the full derivation); confirm it's
    not meaningfully longer. **Also record which core the esp_timer service
    task actually lands on** (e.g. via `xTaskGetAffinity()`/a debug log in
    `ssrDeadmanCallback()`) - `PHASE6_NOTES.md` documents this was never
    confirmed (it lives inside a precompiled library), and `PHASE7_NOTES.md`
    item 10 makes the ~230ms bound conditional on this task not sharing a
    core with the higher-priority Wi-Fi driver task; an unexpectedly long
    decay time is the first thing this measurement would explain.
13. **SSR time-proportional constants** (`PHASE6_NOTES.md` bench item 3;
    still marked `TODO(bench)` at `SSR_WINDOW_MS`'s own declaration).
    Confirm `SSR_WINDOW_MS` (1000ms) and the achievable duty-cycle
    resolution it implies (`SSR_ENFORCE_INTERVAL_MS` bounds this to ~100
    steps, not the full 8-bit PID range - see `PHASE6_NOTES.md`) are
    appropriate for the actual heating element and SSR fitted - this is a
    thermal/electrical response question about the real hardware, not
    something derivable from the code. Also visually/thermally confirm the
    low- and high-duty quantization (`PHASE6_NOTES.md` bench item 5) isn't
    causing visible temperature ripple at the achievable step size.
14. **Task WDT recovery.** With the same wedged-task setup as step 12,
    confirm the system actually reboots within the WDT's 5s timeout (this
    was verified at the config level against this project's `sdkconfig` in
    Phase 7 - this step confirms it in practice). **Do not expect both
    actuators to be safe throughout the hang** - this is `PHASE7_NOTES.md`
    Gap 2, by design as things stand today: the heater goes safe via the
    deadman within ~230ms regardless (step 12), but the pump has no
    independent deadman and is expected to stay frozen at whatever level it
    was last commanded to for the full ~5s until the reboot recovers it.
    Record how long the pump actually stayed driven - this is the
    measurement that would confirm or correct Gap 2's severity assessment.
15. **SSR deadman heartbeat.** Separately, if possible, stall the esp_timer
    service task itself (not just `controlTask`) and confirm `loop()`'s
    diagnostics report actually surfaces the "deadman has not run" warning.

## Group 4 — service-side load and isolation

16. **Establish a baseline first.** Before any of the load tests below, let
    the machine idle (no shot, no Wi-Fi/SD/OTA activity beyond normal
    polling) and record `controlTask`'s worst cycle time and deadline-miss
    count from `loop()`'s 5s diagnostics report over several minutes.
    `PHASE5_NOTES.md` bench item 2 asks for "comfortably under 10ms and
    zero deadline misses" - without this number first, none of steps 17-19
    below have anything to compare against.
17. **Wi-Fi loss mid-shot**, not just while idle - pull the AP and watch
    `controlTask`'s diagnostics against the step 16 baseline for any
    disturbance during the reconnect churn. (Phase 7 review downgraded this
    from "architecturally guaranteed" to "needs a bench check" - see
    `PHASE7_NOTES.md` item 12.)
18. **Missing SD card at boot**, ideally combined with no saved Wi-Fi
    credentials, to see the actual worst-case boot delay before
    `controlTask` even exists (Phase 7 found this could be 30-40s in the
    worst case - outputs stay safe throughout, since the SSR deadman starts
    before any of this, but confirm it).
19. **A large upload mid-shot** (`/upload`) - confirm no disturbance to
    control timing against the step 16 baseline.
20. **OTA update, both mid-idle and mid-shot.** Confirm the heater
    authorization actually drops for the whole transfer. Mid-shot,
    confirm and document what the pump actually does during the transfer,
    given Gap 3 (`PHASE7_NOTES.md`) - it is not expected to be regulated
    correctly; the point of this test is to see exactly what happens so
    Gap 3's fix can be scoped.
21. **Combined load** (`PHASE5_NOTES.md` bench item 4 - the case most
    likely to actually threaten a 10ms deadline, since steps 17-20 above
    only test each source of load in isolation). During a real shot,
    simultaneously: churn Wi-Fi reconnects, send a burst of `/adjust`/
    `/saveConfig` requests, access the SD card, and start an OTA update.
    Watch `controlTask`'s diagnostics throughout. **A deadline miss caused
    specifically by the OTA flash write is expected** (an ESP32 hardware
    constraint - flash erase/program disables interrupts and caches on both
    cores - documented in `PHASE5_NOTES.md`, not a bug to chase); anything
    beyond that is a real finding.

## Group 5 — everything else

22. **Mid-shot settings edits.** Change preinfusion time, bloom time,
    pressure setpoint and brew setpoint mid-shot; confirm none affect the
    running shot and all take effect together on the next one.
23. **Telemetry staleness.** Compare `snapshotTimestampMs` (via `/getValues`
    directly, per this file's preamble) against wall-clock time, both under
    normal operation and while deliberately wedging `controlTask`.
24. **Stack headroom.** Read `controlTask`'s stack high-water mark from the
    5s diagnostics report after some real running time; confirm it's not
    trending toward exhaustion (Phase 5's 4096-byte budget was a starting
    estimate, not a measurement).
25. **Buzzer patterns.** Confirm boot/error/steam-ready beep patterns sound
    correct and aren't audibly delayed by concurrent web/SD activity.

## Recording results

Record pass/fail and any measured numbers (SSR decay time, WDT recovery
time, pump-frozen duration from step 14, baseline cycle time, stack
headroom, boot delay) back into this file or a dated copy of it, and update
`REWRITE_PLAN.md`'s "Current phase" line and the relevant phase's own "Exit
condition" once its items are confirmed. None of Phase 0-8's exit
conditions can be marked met until this procedure has actually been run.
