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

An independent 7-angle review (six parallel passes: items 1-8 audited fresh
against the code, items 9-16 audited fresh against the code, a hunt for a
further undocumented gap, an altitude/depth challenge on the first draft's
two "document, don't fix" calls, a literal diff-scan for citation accuracy, and
a front-end cross-check) found the first draft of this pass overstated several
verdicts and, more importantly, surfaced two severe defects unrelated to any
single checklist item. See "Review round 1" below for what was found, what got
fixed, and what was deliberately left as a documented, prioritized gap rather
than guessed at.

The Phase 7 exit condition is **not met** by this PR - checkboxes in
`REWRITE_PLAN.md`'s Phase 7 checklist stay unchecked. This phase does change
`Discreet.ino` behavior (five small, targeted fixes - see "Review round 1"),
unlike the first draft, which was documentation-only.

## Per-item verification

| # | Checklist item | Verdict | Basis |
|---|---|---|---|
| 1 | Temp-only heats and regulates temperature while shot logic and pump remain inactive | Verified in code | AC-detect is gated on `currentMode != OperatingMode::TEMP_ONLY` (`Discreet.ino` controlStep(), the AC-detect block), so `acDetected` can never become true in `TEMP_ONLY` - the entire shot-phase branch never runs. The independent mode-restriction layer (output priority resolution, step 2) also forces `pumpOut = 0` and calls `endShot()` if a mode flip to `TEMP_ONLY` happens mid-shot. `runPID()` is unaffected by mode, so heating/regulation continues normally. |
| 2 | Normal shot detection enters and exits every intended phase correctly | Verified in code for partitioning; **detection itself is blocked on the already-known mains-aliasing bench item** | The `PREINFUSION -> BLOOM -> EXTRACTION -> COMPLETE -> IDLE` sequence in `controlStep()` partitions cleanly on `actime`, with `endShot()` as the single exit point (three call sites: the AC-off debounce, the fault override, and the `TEMP_ONLY` mode-restriction path - all three converge on the same helper). But entry/exit both hinge on a single `digitalRead(syncPin)` per 10ms control cycle, and Phase 5's own review (`PHASE5_NOTES.md`) already flagged that this exact 10ms grid can alias against 50Hz mains zero-crossings - "the single highest-priority bench item this phase leaves outstanding," per `REWRITE_PLAN.md`. That item is not re-litigated here, just correctly cross-referenced: this phase's own verification can confirm the state machine's *shape* is correct, not that detection itself is reliable on real mains. Separately: `ShotState::COMPLETE` is effectively unobservable from the web side today - `telemetryQueue` is depth 1 and the very next 10ms cycle overwrites it with `IDLE`, so no `/getValues` poll at UI rates can ever catch it (a pre-existing, harmless side effect of Phase 5 decoupling `controlTask` from `loop()`, not a new defect - see the fixed stale comment on `acceptPreinftimeEdit()` for where this used to be described incorrectly). |
| 3 | Pre-infusion, bloom and extraction timing use wall-clock time | Verified in code | `elapsedTime = now - acDetectedTime; actime = elapsedTime / 1000;` - `now` is the single `millis()` sample `controlTaskEntry()` takes once per fixed 10ms cycle, not a loop-iteration count. Correction from the first draft: this was **not** a Phase 1 fix - `git show` on the pre-rewrite baseline shows this exact wall-clock computation already present, unchanged, before this rewrite began. Phase 1 fixed two different loop-count defects (the AC-off debounce's `offcount` and the pump ramp's `callCount`) - see `PHASE1_NOTES.md`. |
| 4 | Pressure setpoint and pump output behave correctly at phase boundaries | Verified in code | PRE-INFUSION targets `PrePressureSetpoint`, EXTRACTION targets `shotSettings.pressuresetpoint` (latched at shot start, immune to mid-shot edits - see item 11); each phase's `pumpPowerSetPreinf`/`pumpPowerSetExtraction` latch runs its one-time `basePumpPowerForSetpoint()` call exactly once per phase and resets whenever the low-pressure boost branch re-arms. BLOOM forces `pumppower = 0` unconditionally. The magnitude of `basePumpPowerForSetpoint()`'s table against a real boiler/pump is a bench-tuning question - see the bench checklist below. See also Gap 4 below for one specific boundary interaction (a fault-triggered mid-shot restart) that behaves correctly by the same logic but is worth flagging as a distinct, higher-risk case. |
| 5 | Invalid temperature sensor forces heater and pump safe | **Fixed - was a real gap, not "verified" as the first draft claimed** | See "Review round 1" item 1. `isnan(input)` only catches the MAX6675's own explicit open-thermocouple bit; a different link failure (lost sensor power, a stuck-low `thermoDO`/GPIO21 with no internal pull) reads back as a numerically valid, unflagged `0.00`. `input < 0` was dead code (`MAX6675::readCelsius()` can never return negative), so nothing caught this. Fixed by replacing the dead check with `MIN_PLAUSIBLE_TEMP_C` (5), mirroring the existing `> 160` bound's own "physically implausible for this appliance" reasoning rather than guessing at the sensor's fault-signaling convention. Residual, inherent limitation worth stating plainly: a fault that happens to land on a *plausible* wrong value (not too low, not too high, not NaN) is fundamentally undetectable without redundant sensing - true of any single-sensor system, not something this phase (or any code change) can close. |
| 6 | Invalid pressure sensor prevents unsafe pump regulation | **Gap - not met, more severe than first stated** | See Gap 1 below. There is no `FaultCode` (or any other mechanism) for an invalid pressure reading, and unlike Gap 5, no honest fix is available without knowing the fitted sensor's fault-signaling convention. |
| 7 | Over-temperature and all time-outs force safe outputs | Verified for the immediate response; **the over-temperature trip itself does not latch (Gap 5)** | Over-temperature (`input > 160`) forces the same immediate safe outputs as item 5. "Time-outs": the SSR authorization timeout (Phase 6) forces the heater pin off independently of `controlStep()`'s health (see item 10); the Task WDT (Phase 5) recovers a genuinely wedged control task via a full reset - **this was explicitly re-verified against this project's actual compiled sdkconfig** (`CONFIG_ESP_TASK_WDT_PANIC=y`, `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y` in `esp32:esp32@2.0.17`'s `esp32/sdkconfig`, matching Phase 5's own precedent of checking real build config rather than assuming), refuting a plausible-sounding but incorrect claim from one review pass that the TWDT only logs and continues on this target - it does not; it reboots. What's *not* verified: `FAULT_CLEAR_STABLE_READINGS` (3 consecutive good readings, ~750ms) was designed to debounce an *intermittent sensor*, but it applies identically to a *genuine* over-temperature event - the fault clears the moment the boiler cools 750ms worth, heat resumes, and it can climb back to 160 and re-trip indefinitely, turning what should be a latched safety cutout into a crude thermostat at the fault boundary. See Gap 5. |
| 8 | Reboot, task failure and watchdog reset leave heater and pump off | Verified in code, including the pump case | **Heater**: `SSR_PIN` is set `OUTPUT`/`LOW` among the first actions in `setup()` (immediately after `Serial.begin()`, before the boot delay - Phase 6 review), and the independent SSR deadman forces it off within `SSR_AUTH_TIMEOUT_MS` of the last successful `controlStep()` cycle regardless of task health. **Pump**: re-examined more carefully than the first draft, which cited the wrong library function (`Thyristor::begin()` does not touch the output pin - that's the constructor, `thyristor.cpp:711-713`; `DimmableLight light(thyristorPin)` is a file-scope global, so its constructor runs at C++ static-init time, before `setup()` even starts). The window that actually matters is between `DimmableLight::begin()` (which arms the timer/ISR machinery) and the first real `light.setBrightness()` call, which can only happen once `controlTask` starts, several seconds later. Checked directly: `Thyristor::begin()` does **not** attach the zero-cross interrupt (`attachInterrupt()` for `zero_cross_int` only happens inside `setDelay()`, gated behind `#ifdef MONITOR_FREQUENCY` at `begin()`-time otherwise, which this project does not define) - so no gate-fire logic can run at all until the first `setDelay()` call, which is exactly the first `setBrightness()` call. There is no unguarded window. |
| 9 | A deliberately wedged control task is detected because that task is explicitly subscribed to the Task WDT | Verified in code | `controlTaskEntry()` calls `esp_task_wdt_add(NULL)` and checks the return value; `esp_task_wdt_reset()` is called only after `controlStep()` returns, so a genuine hang never reaches the feed. See item 7 above for the direct sdkconfig verification that a trip actually reboots this build, not just logs. Actually observing the reset requires deliberately wedging the task on real hardware - see the bench checklist below. |
| 10 | Loss of SSR refresh makes the heater output decay to off within the documented maximum window | Verified in code, bound corrected | `ssrDeadmanCallback()` runs every `SSR_ENFORCE_INTERVAL_MS` (10ms). First-draft bound (~210ms) missed that this callback shares its dispatch task with `buzzerTimerCallback()`, which can hold `buzzerMutex` for up to `BUZZER_MUTEX_WAIT_TICKS` (20ms) before giving up - both run under the same serial `ESP_TIMER_TASK` dispatch (`Discreet.ino`'s own comment on this, `ssrDeadmanTimer`'s declaration). Corrected worst case: `SSR_AUTH_TIMEOUT_MS` (200) + one enforce tick (10) + one skipped buzzer tick (20) = **~230ms**, not 210. Also conditional on the esp_timer service task actually being scheduled promptly - `PHASE6_NOTES.md` already documents that its core affinity was never confirmed, and it shares whatever core it's on with the Wi-Fi driver task at a higher priority. |
| 11 | Mid-shot settings edits do not change the active shot and the latest valid pending revision becomes active after return to `IDLE` | Verified in code | `shotSettings` is latched from `activeSettings` once, at shot start, and is the only settings struct the shot-phase branches read - a mid-shot edit only ever reaches `pendingSettings`. `endShot()` promotes the *entire* `pendingSettings` struct, not just the last-edited field, and runs on all three exit paths (AC-off, fault, mode restriction). One narrow note, not counted as a defect: `START_STEAM`/`STOP_STEAM` write the live `setpoint` unconditionally, including mid-shot - outside the four fields this item's "latched settings" language covers (brew temp, pressure target, phase durations), so not in scope for this item, but worth knowing the steam/brew target swap is not itself latch-protected. |
| 12 | Wi-Fi loss does not disturb control timing | **Downgraded to bench-required** - not fully "architecturally guaranteed" as first stated | `controlTask` has no *direct* dependency on `WiFi`/`server`/`ArduinoOTA` state - that part holds. But the first draft's "cannot" was too strong: `Discreet.ino`'s own comment on `CONTROL_TASK_CORE` (Phase 5) already documents that arduino-esp32's `arduino_events` task shares core 1 with `controlTask`, at a priority (~19) well above `controlTask`'s (5), and is idle "except for infrequent Wi-Fi/system events." A Wi-Fi *loss* is exactly the event that makes it non-idle: with the default auto-reconnect behavior (nothing in this file disables it), a disconnect drives that same high-priority task through reconnect work, repeatedly, on `controlTask`'s own core. Whether that measurably delays a 10ms cycle or the 200ms SSR authorization window is unmeasured - this needs a bench run, not just the architectural argument. |
| 13 | Missing/failing SD card does not disturb control timing | Verified for steady-state; **boot-time caveat added** | Once running, every `SD.*` call is reached only from `loop()`-side handlers, never from `controlStep()` or anything it calls - same isolation argument as item 12's steady-state case. Caveat: `controlTask` is created *last* in `setup()`, after `loadSDConfig()`/`startSD()`/`startWiFi()` - a worst case with no SD card and no saved Wi-Fi credentials can spend 30-40s in blocking retry/beep loops before any control task exists at all. Outputs stay safe throughout (the SSR deadman is started first, before any of this), so this is an availability gap, not a safety one - but "does not disturb control timing" is inaccurate for that boot window, where there is no control timing yet to disturb. |
| 14 | Large web requests/uploads do not disturb control timing | Verified in code (architecturally) | `handleUpload()`/`server.handleClient()` run entirely in `loopTask`; `controlTask`'s only contact with that side is `commandQueue`/`telemetryQueue`, both non-blocking on the `controlTask` side. The item 10/12 caveats about the esp_timer task's scheduling and Wi-Fi-event contention apply here too but are not specific to this item. |
| 15 | OTA start either refuses while unsafe or transitions the machine to a documented safe state | **Downgraded to gap** - see Gap 3 | Does not refuse, and does not fully transition to a safe state either: `otaInProgress` forces the heater off, but the pump is deliberately left unmasked, and - more significantly - Phase 5's own note on `CONTROL_TASK_CORE` already documents that an OTA flash erase/program disables interrupts and caches on both cores for its duration, meaning `controlTask` itself stalls during the transfer. A stalled control task means the pump is frozen at whatever level it was last commanded to, with no AC-off debounce sampling and no shot-end detection, for the length of the OTA transfer - not "regulation is skipped," but "the pump can run unattended, at whatever level, for the whole upload." |
| 16 | Telemetry remains coherent and stale snapshots are detectable | **Partially fixed; residual gaps documented** | `heaterOn` in `/getValues` is now computed fresh from `ssrAuthorizedUntilMs` directly (see "Review round 1" item 4) rather than read from the queued snapshot, so it can no longer report stale "on" after the deadman has already taken over - this was a real gap in the first draft's claim, now closed. Two things remain, documented rather than fixed here: (a) `/temp` and `/pressure` read live control-owned globals directly from `loopTask`, bypassing the snapshot mechanism entirely - a `/saveConfig` offset change landing between `xQueuePeek()` and the point `/getValues` uses the live `offset` global to derive `displaySetpoint` can produce a `/getValues` response computed against two different offsets in the same reply; (b) `Kp`/`Ki`/`Kd`/`steamSetpoint`/`brewTemp` in `/getValues` are likewise read live rather than from the snapshot (pre-existing, noted in Phase 4/5 notes already). Neither is new to this phase and neither is safety-critical (display-only), but "coherent" oversold both. `snapshotTimestampMs` staleness detection itself is real and correctly wired, but see the front-end note below for why it isn't actually usable as a bench signal today. |

## Review round 1

Six parallel Opus review passes (items 1-8 fresh-audited against the code,
items 9-16 fresh-audited against the code, a hunt for a further gap beyond
the two already documented, an altitude/depth challenge on the original two
"document, don't fix" calls, a literal diff-scan for citation accuracy, and a
front-end cross-check) returned a large, well-substantiated set of findings.
Each factual claim below was independently re-verified against the actual
library/SDK source on disk before being accepted - the same discipline
Phase 5 established for ESP-IDF internals claims - and at least one plausible
-sounding but incorrect claim (that the Task WDT only logs and doesn't
reboot on this target) was caught and refuted this way; see item 7/9 above.

### Fixed

1. **MAX6675 stuck-low reads as a valid 0.00C, not a fault (item 5).**
   `isnan(input)` only catches the chip's own open-thermocouple bit; a
   different link failure (verified against `max6675.cpp`: any SPI/power
   fault that reads back all-zeros returns exactly `0.0`, not `NAN`) passed
   every existing check, including the now-dead `input < 0` (`readCelsius()`
   can never return negative). Left unfixed, this saturates the heater to
   100% duty indefinitely with no fault raised and no other backstop - the
   deadman and Task WDT both protect against a wedged/crashed task, not a
   healthy task computing a wrong answer from bad data. Fixed by replacing
   the dead check with `MIN_PLAUSIBLE_TEMP_C` (5), reusing the existing
   `input > 160` bound's own "physically implausible for this appliance"
   reasoning - not a guess at this specific sensor's fault convention, which
   is exactly the distinction that kept Gap 1 (pressure) from getting the
   same treatment; see Gap 1 for why that one is different.

2. **`loadSDConfig()` never applied the tunings it loaded (unrelated to any
   single checklist item, found while verifying item 5's neighborhood).**
   `Kp`/`Ki`/`Kd` were assigned to the *globals* from `config.json`, but
   `myPID.SetTunings()` was never called again after the `PID` object's own
   constructor captured the compile-time defaults (48/8/50) at static-init
   time - verified against `PID_v1.cpp`: the constructor's tunings are
   copied into private state, and nothing re-reads the globals afterward
   except the `SET_TUNINGS` queued command (`applyControlCommand()`), which
   only ever ran if a user happened to resubmit the config page after boot.
   Every boot silently regulated on the hardcoded defaults instead of
   `config.json`'s values (or the no-config fallback's 80/6/55), while
   `/getValues` reported the unused globals the whole time - a bench tuning
   session conducted through the config page was reading one gain set while
   the controller ran another. Fixed with one `myPID.SetTunings(Kp, Ki,
   Kd);` call in `loadSDConfig()`, after the globals are set.

3. **`/saveConfig`'s setpoint clamp doesn't survive a reboot (found in the
   same neighborhood).** The live-session clamp
   (`applyControlCommand()`'s `SET_BREW_SETPOINT_ABSOLUTE` case) only
   bounds the in-memory value; `/saveConfig` persists the client's raw JSON
   to `config.json` *before* that command is even sent, and `loadSDConfig()`
   applies whatever's in the file back with no clamp at all. A single
   out-of-range save (a UI typo, since neither `config.html` nor `config.js`
   enforce min/max) is invisible in the moment - the running machine still
   clamps - but becomes the unclamped boot-time PID target on every
   subsequent reboot, indefinitely, until saved over again. Fixed by
   applying the identical `constrain(setpoint, 10 + offset, 96 + offset)`
   bound in `loadSDConfig()` - the same bound already established
   elsewhere, not a new judgement call. `steamSetpoint`/`offset` are
   deliberately left unclamped here, matching their existing total lack of
   validation everywhere else in this file (Phase 5 review already noted
   this as deliberate) - clamping only one of the three inconsistently,
   without first deciding real bounds for the other two, would be a
   separate, undiscussed change.

4. **`heaterOn` served over HTTP could report stale "on" forever after a
   control-task wedge (item 16).** `handleGetValues()` read `snap.heaterOn`
   from a single `xQueuePeek()` near the top of the handler - the last
   snapshot `controlStep()` published. If `controlTask` ever wedges, that
   snapshot simply stops updating, and this handler would keep serving
   whatever `heaterOn` value it last saw, forever - directly contradicting
   the guarantee `ssrAuthorizedUntilMs`'s own declaration comment promises
   ("telemetry can never show 'on' once the deadman has actually taken
   over"), which holds for `buildTelemetrySnapshot()`'s own computation but
   not for a reader of its stale output. Fixed by computing `heaterOn`
   directly from `ssrAuthorizedUntilMs` in `handleGetValues()` itself, the
   same wraparound-safe idiom `ssrDeadmanCallback()`/`buildTelemetrySnapshot()`
   already use, reading the same always-fresh, single-word global instead of
   the queue. `resolvedPumpPower` has no equivalent fix available - the pump
   has no independent deadman (Gap 2), so it is only ever as fresh as
   `controlTask`'s last successful cycle regardless of how it's read.

5. **A stale comment mis-described the current architecture.** The comment
   on `acceptPreinftimeEdit()` still said the FAULT override runs "at the
   end of `loop()`" and that `COMPLETE` survives "into the next iteration's
   `server.handleClient()` (line ~1006)" - both true before Phase 5, neither
   true after it (`controlStep()` runs in its own task now; line ~1006
   today is inside an unrelated handler). The underlying reasoning the
   comment exists to explain (prefer `!acDetected` over `currentShotState`
   as the idle gate) is still correct and was kept; only the stale
   mechanism description was corrected, along with a note that `COMPLETE`
   is now effectively unobservable from the web side at all (queue depth 1,
   overwritten within one 10ms cycle) - itself a harmless side effect of
   Phase 5, not a new defect.

Compiles cleanly after all five fixes: 909,345 bytes flash (+168 vs
`v2.0.5-beta`), 51,340 bytes RAM (unchanged).

### Documented, not fixed - real gaps requiring a decision this environment can't make blindly

**Gap 1 - no invalid-pressure-sensor detection (checklist item 6).**
`GetPressure()` computes `currentPressure` directly from
`analogRead(pressurepin)` with no validity check:

```cpp
int raw = analogRead(pressurepin);
currentPressure = (raw * (maxPressure / 4095.0));
```

Unlike temperature (Gap-1-equivalent already fixed above, item 5),
`analogRead()` on this target always returns an in-range 12-bit value - there
is no impossible raw value the way `NAN` is for a valid MAX6675 word, and
`currentPressure` is already mathematically confined to `[0, maxPressure]`
by this formula alone, so a bounds check here is provably dead code, not a
missing safeguard - re-verified during review, correcting the first draft's
framing, which floated a bounds check as a live option. A disconnected or
shorted sensor reads as a plausible in-range value overlapping genuinely
valid readings in both directions, and the two directions are **not
symmetric in severity**, confirmed by tracing both branches of the shot-phase
logic:

- **Fail-high** (short to supply, raw approx 4095 to 12 bar) is self-limiting:
  `updatePumpRamp()` cuts `pumpDemand` at `currentPressure > PressureTarget +
  0.3`.
- **Fail-low** (broken wire, raw approx 0 to 0 bar) is the opposite of
  self-limiting: both the pre-infusion boost branch
  (`currentPressure <= PrePressureSetpoint - 1`) and the extraction boost
  branch (`currentPressure < shotSettings.pressuresetpoint - 2`) read a
  stuck 0 as "pressure is low," latch `pumppower = 255`, and clear the
  `pumpPowerSet*` flag that would otherwise let `updatePumpRamp()` - and its
  overpressure cut - ever run. A dead sensor pins the pump at full power,
  completely unregulated, for the entire shot, while telemetry reports a
  calm 0.00 bar.

**Why this isn't fixed here**: there is no honest fix at any price without
knowing the fitted sensor's fault-signaling convention (some ratiometric
transducers use a "live zero" band - e.g. below 0.5V/above 4.5V on a
0.5-4.5V active range - to signal a fault; this repo documents neither the
part fitted nor whether it has such a convention). Guessing thresholds would
be exactly the kind of hardware-uninformed fix this project has consistently
avoided (see Phase 5's mains zero-cross item). **Recommendation**: identify
the fitted sensor and its output convention first; this is a datasheet/bench
item, not a code item, and given the fail-low severity just established, it
should be the *first* thing checked, ahead of the mains-aliasing and deadman
-timing bench items already queued from Phase 5/6.

**Gap 2 - no independent deadman for the pump (checklist item 8, and the
severity claim in Gap 2 of the first draft).** The heater has two
independent layers against a wedged `controlTask`: the Task WDT (system-wide,
confirmed via direct sdkconfig verification to actually reboot this build -
see item 7 above - within its 5s timeout) and the SSR deadman
(heater-specific, ~230ms decay to off, Phase 6). The pump has only the
first: if `controlTask` wedges while the dimmer was at a nonzero level, it
stays there until the Task WDT reboots the system.

The first draft argued this should be deferred because building a second
deadman is "real new safety infrastructure" - review found that reasoning
wrong on its own terms: cloning the mechanical pattern is cheap (the
existing SSR deadman's `esp_timer` callback already ticks every 10ms; adding
a `pumpAuthorizedUntilMs` next to the existing one and a stale-check in the
same callback is a small addition, not a second subsystem). The actual
reason to defer is different, and more specific: doing so means calling into
the `Dimmable_Light_for_Arduino` library (`light.setBrightness(0)` -
directly writing the pin, as the first draft's recommendation suggested,
would not work, since the library's own zero-cross ISR re-drives the gate
pin every half-cycle independent of `controlTask`) from the `esp_timer`
service task, a different task than the one (`controlTask`) that normally
calls it. Verified directly against `thyristor.cpp`: `Thyristor::setDelay()`
mutates shared state (`thyristors[]`, `posIntoArray`, `delay`,
`newDelayValues`) guarded only by a plain `bool updatingStruct` that is
never checked at entry - not a lock, just a comment-documented convention
the library's own author states is safe "w.r.t. the ISR," which is not the
same claim as "safe w.r.t. a second concurrent task." With `nThyristors == 1`
(one `DimmableLight` in this project) the array-reorder loops are provably
inert regardless of concurrency, which makes an actual corruption unlikely
in practice - but "probably fine because there's only one light" is exactly
the kind of unverified-in-this-environment claim Phase 6 refused to accept
for the SSR deadman's own core-affinity question, and this library isn't
vendored with tests this project can run to check it. A mitigating factor
the first draft's severity argument missed: the pump only fires through the
triac off mains AC that the same zero-cross detector watches, which is only
present while the physical brew switch is engaged - so a wedged-pump
scenario cannot outlast the operator's own hand on the switch, and is
already bounded by the Task WDT's confirmed reboot regardless.

**Recommendation**: still worth its own follow-up (mirroring Gap 5's
over-temp-latching item in spirit: real, understood, not urgent enough to
guess at inline) - but scope it explicitly as "call `light.setBrightness(0)`
from a task that isn't `controlTask`, and either prove `Thyristor::setDelay()`
tolerates that or ask the library's maintainer," not as "build a second SSR
-shaped deadman."

**Gap 3 - OTA does not transition the pump/shot to a documented safe state
(checklist item 15, downgraded from "verified").** `otaInProgress` forces the
heater off for the OTA session, but the pump is deliberately left unmasked -
and Phase 5's own notes already record that an OTA flash erase/program
disables interrupts and caches on both cores for its duration, meaning
`controlTask` itself stalls while the transfer runs. A stalled control task
during OTA does not mean "the pump keeps regulating unsupervised" (the
original framing) - it means the pump is frozen at whatever level it was
last commanded to, with no AC-off debounce sampling and no shot-end
detection, for the length of the transfer. **Why this isn't fixed here**:
closing it well requires a real design decision (should OTA simply be
refused while `acDetected` is true? should the pump be forced off the same
way the heater is, even though nothing currently couples pump safety to OTA
state? is a mid-shot OTA update even a scenario worth supporting at all?),
not a one-line mechanical fix. **Recommendation**: decide the intended
behavior (most conservative: refuse `ArduinoOTA` start while `acDetected` is
true, matching "OTA start either refuses while unsafe" - the checklist's own
first alternative, not currently implemented either) before the next phase
that touches OTA.

**Gap 4 - the pressure-blanking window can command full pump power into an
already-pressurized group on a fault-triggered restart (checklist item 4,
noted as an edge case, not a defect in the core logic).**
`GetPressure()` forces `currentPressure = 0` for the first 500ms of every
shot (`elapsedTime < 500`) - intentional, and safe, for a *normal* shot
start into a group that has just depressurized. But a fault-triggered
restart (paddle held continuously through a transient fault: `endShot()`
clears `acDetected`, the fault clears after `FAULT_CLEAR_STABLE_READINGS`
~750ms later, and `acDetected` immediately re-triggers on the very next
cycle because the paddle never released) re-enters pre-infusion the same
way, with the same 500ms pressure-blank - except the group may still be at
or near its previous operating pressure, not depressurized. The pre-infusion
boost branch reads the forced 0 as "pressure is low" and commands
`pumppower = 255` for up to 500ms into a group that might already be at
several bar. **Why this isn't fixed here**: whether the group actually stays
pressurized through a fault (this depends on solenoid/valve behavior not
controlled by this firmware at all, as far as this file shows) is a
hardware question this environment can't answer, and any code fix (skip the
blank window on a restart? read live pressure immediately instead of
blanking?) depends on that answer. **Recommendation**: bench-check whether
the group actually holds pressure across a fault-and-recover cycle with the
paddle held; if it does, this needs a real fix, scoped after that answer is
known.

**Gap 5 - the over-temperature fault does not latch (checklist item 7).**
`FAULT_CLEAR_STABLE_READINGS` (3 consecutive good readings, ~750ms) was
designed to debounce an *intermittent sensor* (a loose crimp flapping
between valid and invalid reads - see the comment on this constant's own
declaration) and applies identically to a *genuine* over-temperature event.
A real over-temperature trip therefore self-clears the moment the boiler
cools 750ms worth, heat resumes, and the cycle can repeat indefinitely -
turning what should be a latched safety cutout into a crude thermostat
pinned at the fault boundary, rather than a condition requiring a deliberate
reset. **Why this isn't fixed here**: latching correctly requires a real UX
decision this environment can't make blindly (latch until physical reboot
only? until a new explicit "clear fault" action? for how long, if
time-based?) - not something to bolt on without deciding what "cleared"
should mean for this specific failure mode, as distinct from a flapping
sensor. **Recommendation**: given severity (a genuine thermal excursion
cycling indefinitely rather than shutting down), this is worth prioritizing
alongside Gap 1 - both are heater-adjacent and both were found, not assumed.

### Front-end observability note (not a firmware defect, but affects how the bench checklist below can actually be run)

The front-end cross-check pass found the shipped dashboard (`Discreet_Front_End/`)
undermines the bench checklist's own usability in a few ways worth knowing
before running it:

- `scripts.js` stamps every chart point with the *browser's* `Date.now()`,
  not `snapshotTimestampMs` - nothing in the front end reads that field at
  all. If `controlTask` wedges and telemetry freezes, the chart keeps
  scrolling with fresh timestamps against flat lines: a stale snapshot
  renders as live, not as stale. `snapshotTimestampMs`'s staleness-detection
  value (item 16) is real, but only via a direct `/getValues` query - not
  through the shipped UI.
- The dashboard displays `pumppower` (`snap.pumpPower`, the pre-resolution
  demand), not `resolvedPumpPower` (post fault/mode resolution) - the exact
  field Phase 3 added specifically so a bench operator could see a fault or
  mode restriction actually take effect. `fault`/`mode`/`shotState` are
  likewise emitted by the firmware and never read anywhere in the front end.
- Practical consequence for the bench checklist below: several items ("confirm
  safe outputs," "confirm the heater authorization actually drops") are not
  actually observable through the shipped dashboard - they require querying
  `/getValues` directly (curl or browser devtools), not just watching the
  page.

Not fixed here (front-end code, outside this PR's scope), but worth a line
in whichever phase next touches the front end.

## Bench checklist — physical verification still required

Everything marked "Verified in code" above is a logic guarantee, not a
measurement. The following still need a real machine before Phase 7's own
exit condition can be considered met - reordered from the first draft to put
the two heater-adjacent gaps (1 and 5) first, given their severity:

- **Gap 1 first**: identify the fitted pressure sensor and its output
  convention before anything else - it blocks item 6 and materially changes
  the pump-safety picture for pre-infusion/extraction.
- **Gap 5 alongside it**: confirm whether a genuine over-temperature
  condition is reachable in practice (heater sizing, boiler thermal mass)
  and decide the intended latching behavior before relying on the 160C trip
  as a real safety cutout.
- Items 1-4: run an actual temp-only session and a full shot; confirm phase
  transitions, pump behavior and boundary timing feel correct against a real
  boiler/pump. Query `/getValues` directly, not the dashboard - see the
  front-end note above.
- Item 5: disconnect/short the thermocouple mid-idle and mid-shot; confirm
  both actuators cut immediately and recover after 3 stable readings. Also
  specifically test a stuck-low failure (not just an open lead) now that
  `MIN_PLAUSIBLE_TEMP_C` exists, to confirm it actually trips in practice.
- Item 7: force `input` above 160 and confirm the fault trips - and, given
  Gap 5, confirm (and document) that it currently does *not* stay latched.
- Item 8: cold boot, and a warm reset via the Task WDT specifically (not just
  power-cycle) - confirm both actuators come up/stay off.
- Item 9: deliberately wedge `controlTask` (e.g. an infinite loop behind a
  debug flag) and confirm the watchdog reset actually fires and reboots -
  this is now expected to reboot (re-verified via sdkconfig, see item 7),
  not just log.
- Item 10: with the machine heating, kill `controlTask` and time the SSR's
  actual physical decay against the corrected ~230ms bound.
- Item 11: edit settings mid-shot across several fields, confirm none affect
  the running shot, confirm all take effect together on the next shot.
- Item 12: pull Wi-Fi specifically *during* a shot (not just while idle) and
  watch `controlTask`'s diagnostics report for any disturbance during the
  reconnect churn - this is no longer assumed safe, per the corrected
  verdict above.
- Items 13-14: boot with no SD card and no saved Wi-Fi credentials
  simultaneously, and time how long it takes before `controlTask` exists at
  all; separately, push a large upload mid-shot and watch for disturbance.
- Item 15: trigger an OTA update mid-idle and mid-shot; given Gap 3, expect
  and document what actually happens to the pump/shot, not just the heater.
- Item 16: compare `snapshotTimestampMs` against wall-clock time via direct
  `/getValues` queries (not the dashboard) under normal operation and while
  deliberately wedging `controlTask`.
- Gaps 2 and 4: no bench action needed until their own recommended follow-up
  work is scoped (see each gap's own recommendation above).

This list is additive to - not a replacement for - the bench items already
outstanding from Phase 0 through Phase 6 (mains zero-cross sampling and SSR
deadman timing chief among them; see their own notes files).
