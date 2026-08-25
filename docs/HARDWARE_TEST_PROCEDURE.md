# Hardware test procedure

Every phase of this rewrite (Phase 0 through Phase 7) was implemented and
reviewed without access to physical hardware. Nothing in `v2.0.6-beta` has
been bench-verified. This is a single, ordered procedure consolidating every
outstanding bench item from every phase's own notes into one runbook - what
to do, in what order, and what a pass/fail result looks like. It supersedes
running each phase's bench checklist separately; the phase notes remain the
place to go for *why* each item exists and what code backs it.

**Do not power a real boiler/pump from this firmware before completing at
least Group 1.** Bench with the heater/pump disconnected or current-limited
where practical until sensor-fault and over-temperature behavior are
confirmed.

## Group 1 — before connecting mains-powered hardware at all

1. **Identify the pressure sensor's fault-signaling convention.** Find its
   datasheet/part number. Does it use a "live zero" band (output pinned
   below/above its normal working range on a fault)? Record the answer in
   `PHASE7_NOTES.md` Gap 1, whichever way it goes - this blocks adding real
   invalid-pressure-sensor detection, the single highest-priority code gap
   this rewrite has found.
2. **Confirm a genuine over-temperature condition is reachable in
   practice**, given this machine's actual heater sizing and boiler thermal
   mass. If it is, decide (before wiring the boiler) what "latched until
   cleared" should mean for `FaultCode::INVALID_TEMPERATURE` when the cause
   is genuine over-temperature rather than a flapping sensor - see
   `PHASE7_NOTES.md` Gap 5.
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

## Group 2 — shot detection and timing (needs mains AC on `syncPin`)

6. **Mains zero-cross aliasing** (`PHASE5_NOTES.md`, the single highest
   -priority item that phase left outstanding). `controlTask` samples
   `syncPin` on an exact 10ms grid, which can alias against 50Hz mains
   zero-crossings (also ~10ms apart). Run several shot start/stop cycles
   and confirm AC detection and AC-off debounce both fire reliably, not
   just once. If detection is flaky or the debounce fires early/late,
   this needs a hardware-level fix (see that phase's "Considered, not
   changed" section for what was ruled out and why).
7. **Temp-only mode.** Confirm the boiler heats and regulates while shot
   logic and pump both stay inactive throughout.
8. **A full normal shot.** Confirm pre-infusion, bloom and extraction all
   enter and exit at the right wall-clock boundaries, and that pump output
   matches each phase's target.
9. **`AC_OFF_DEBOUNCE_MS`/`AC_OFF_MIN_SAMPLES` tuning** (`TODO(bench)` since
   Phase 1). Confirm a real shot-end is detected promptly and that no
   ordinary in-shot condition (a brief pressure dip, a `loop()` stall from
   an SD/web operation) ends a shot early.
10. **Fault-triggered restart into a pressurized group** (`PHASE7_NOTES.md`
    Gap 4). With the paddle held continuously, force a transient thermocouple
    fault mid-extraction (see step 4) and observe what the pump actually does
    on the immediate restart - does the group stay pressurized through the
    fault, and does the pump's 500ms boost-to-255 behavior on restart cause
    a problem in practice? This determines whether Gap 4 needs a real fix.

## Group 3 — the independent safety mechanisms

11. **SSR deadman timing** (`PHASE6_NOTES.md`, this rewrite's other
    highest-priority outstanding item). With the heater authorized on, kill
    `controlTask` (a debug build with an intentional infinite loop behind a
    flag is easiest) and time how long `SSR_PIN` stays driven before the
    deadman forces it off. Expect ~230ms (see `ARCHITECTURE.md`'s timing
    table for the full derivation); confirm it's not meaningfully longer.
12. **Task WDT recovery.** With the same wedged-task setup, confirm the
    system actually reboots within the WDT's 5s timeout (this was verified
    at the config level against this project's `sdkconfig` in Phase 7 - this
    step confirms it in practice) and that both actuators are safe
    throughout the hang and the reboot.
13. **SSR deadman heartbeat.** Separately, if possible, stall the esp_timer
    service task itself (not just `controlTask`) and confirm `loop()`'s
    diagnostics report actually surfaces the "deadman has not run" warning.

## Group 4 — service-side load and isolation

14. **Wi-Fi loss mid-shot**, not just while idle - pull the AP and watch
    `controlTask`'s diagnostics (worst cycle time, deadline misses) for any
    disturbance during the reconnect churn. (Phase 7 review downgraded this
    from "architecturally guaranteed" to "needs a bench check" - see
    `PHASE7_NOTES.md` item 12.)
15. **Missing SD card at boot**, ideally combined with no saved Wi-Fi
    credentials, to see the actual worst-case boot delay before
    `controlTask` even exists (Phase 7 found this could be 30-40s in the
    worst case - outputs stay safe throughout, since the SSR deadman starts
    before any of this, but confirm it).
16. **A large upload mid-shot** (`/upload`) - confirm no disturbance to
    control timing.
17. **OTA update, both mid-idle and mid-shot.** Confirm the heater
    authorization actually drops for the whole transfer. Mid-shot,
    confirm and document what the pump actually does during the transfer,
    given Gap 3 (`PHASE7_NOTES.md`) - it is not expected to be regulated
    correctly; the point of this test is to see exactly what happens so
    Gap 3's fix can be scoped.

## Group 5 — everything else

18. **Mid-shot settings edits.** Change preinfusion time, bloom time,
    pressure setpoint and brew setpoint mid-shot; confirm none affect the
    running shot and all take effect together on the next one.
19. **Telemetry staleness.** Query `/getValues` directly (not the shipped
    dashboard - see `PHASE7_NOTES.md`'s front-end note for why the
    dashboard can't be trusted for this) and compare `snapshotTimestampMs`
    against wall-clock time, both under normal operation and while
    deliberately wedging `controlTask`.
20. **Stack headroom.** Read `controlTask`'s stack high-water mark from the
    5s diagnostics report after some real running time; confirm it's not
    trending toward exhaustion (Phase 5's 4096-byte budget was a starting
    estimate, not a measurement).
21. **Buzzer patterns.** Confirm boot/error/steam-ready beep patterns sound
    correct and aren't audibly delayed by concurrent web/SD activity.

## Recording results

Record pass/fail and any measured numbers (SSR decay time, WDT recovery
time, stack headroom, boot delay) back into this file or a dated copy of
it, and update `REWRITE_PLAN.md`'s "Current phase" line and the relevant
phase's own "Exit condition" once its items are confirmed. None of Phase
0-7's exit conditions can be marked met until this procedure has actually
been run.
