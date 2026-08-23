# Phase 1 notes — remove blocking and loop-count timing

Date: 2026-08-19
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 1, applied on top of the
Phase 0 baseline (`baseline-phase0` tag, see [BASELINE.md](BASELINE.md)).

Goal: keep machine behaviour unchanged while removing the places control
timing depended on `delay()` or on loop-iteration counts rather than
wall-clock time. No application-owned FreeRTOS task, state model, or
actuator-ownership change happens in this phase — that is Phases 2–5.

**This branch went through two rounds of independent review before being
proposed for merge; see "Review round 1: findings and fixes" below.** The
first version had a real regression in the AC-off debounce (it could end a
live shot mid-extraction and restart it at full pump power, under exactly
the kind of `loop()` stall this phase is trying to make control timing
independent of), left `SetPump()`'s `callCount` unaddressed despite the
Phase 1 exit condition claiming otherwise, and left one documented,
unresolved residual risk in the buzzer sequencer. All three are fixed in
the version described below — the buzzer fix in particular came from a
second, independently-authored change that arrived on this branch while the
debounce/`callCount` fixes were being made, and the two were merged
together rather than one overwriting the other.

## 1. Buzzer: `delay()`-based blocking → timer-driven sequencer

**Before:** `beepBuzzer(times, beepDuration, pauseDuration)` toggled
`BUZZER_PIN` in a `for` loop using `delay()` between edges. Every call
blocked whichever code path invoked it for the full pattern duration
(e.g. `beepBuzzer(3,100,100)` blocks ~600 ms).

**After:** `queueBuzzer()` starts a small state machine whose edges are
driven by an ESP one-shot timer (`esp_timer_create` / `esp_timer_start_once`,
dispatched on the ESP timer service task). The timer callback advances the
pattern independently of Arduino `loop()`, guarded by a mutex shared with
`queueBuzzer()`/`waitForBuzzer()` since state is now touched from two
different task contexts. Consequently a slow HTTP, SD or OTA call cannot
stretch an ON pulse or leave the pin stuck asserted — this closes what an
earlier, simpler non-blocking design (a state machine serviced once per
`loop()` iteration) could not: that version needed `loop()` to actually run
to advance a beep, so a stall elsewhere could hold the pin on for the whole
stall. This does not create a third application-owned task; it uses a
platform service (`esp_timer`) that already exists. `waitForBuzzer()` only
polls completion during setup, to retain the original boot sequence.

**Call sites**, and why each is safe to leave as-is or was changed:

- `startSD()` (called from `loadSDConfig()` and again directly in `setup()`)
  and `startWiFi()`: both only ever run during `setup()`, before `loop()`
  starts. Nothing else needs to run concurrently at that point, so these now
  call `queueBuzzer()` followed by `waitForBuzzer()` — a blocking wait, but
  the timer keeps driving the pattern in the background rather than a
  duplicate timing implementation. Boot-time beep timing is unchanged from
  the original behaviour.
- `steam()`: called from every `loop()` iteration. This is the call site
  that mattered — the original blocking `beepBuzzer(3,100,100)` here could
  stall `GetPressure()`, `runPID()`, `SetPump()` and AC/shot-state handling
  for ~600 ms if steam threshold was crossed while other control logic
  needed to run. It now calls `queueBuzzer()` only; timer callbacks advance
  the pattern without blocking coffee logic and without depending on loop
  latency.

`queueBuzzer()` also now rejects a new pattern outright while one is already
playing (checked under the mutex), matching the original blocking
function's implicit "cannot overlap itself" contract, rather than silently
truncating an in-flight pattern.

## 2. Shot-end detection: loop-count `offcount` → elapsed-time debounce

**Before:**

```cpp
if (digitalRead(syncPin) == HIGH) offcount++;
else offcount = 0;
if (offcount >= 100) { /* end shot */ }
```

`syncPin` serves two roles: it is the dimmer library's zero-cross interrupt
source, and it is also polled directly here as a proxy for "is AC/the shot
switch on". While AC is present it toggles at line frequency; when the shot
switch is released it settles. Debouncing this over 100 *loop iterations*
means the real-world debounce time is whatever 100 iterations of `loop()`
happen to take — which varies with `server.handleClient()` load (SD
streaming, uploads), OTA activity, and everything else sharing the loop.
This is precisely the "loop-count-based timing" `REWRITE_PLAN.md` Phase 1
calls out.

**After (first version, reverted — see "Review round 1" below):** a version
using elapsed time alone (`millis() - acOffSince >= AC_OFF_DEBOUNCE_MS`) was
tried first and turned out to be a real regression, not just an unmeasured
constant: `millis()` keeps advancing even while `loop()` is stalled, so a
single sample taken right after a stall could satisfy a pure time check
immediately — something the old iteration counter could never do, since it
only advanced once per sample actually taken. See "Review round 1 findings
and fixes" below for the full failure scenario.

**After (current):**

```cpp
if (digitalRead(syncPin) == HIGH) {
  if (!acOffPending) {
    acOffPending = true;
    acOffSince = millis();
    acOffSamples = 0;
  }
  acOffSamples++;
} else {
  acOffPending = false;
  acOffSamples = 0;
}

if (acOffPending && acOffSamples >= AC_OFF_MIN_SAMPLES &&
    millis() - acOffSince >= AC_OFF_DEBOUNCE_MS) {
  /* end shot, light.setBrightness(0) */
}
```

Same reset-on-any-LOW-reading semantics as the original (a single "on"
sample immediately cancels the pending off-debounce). Shot-end now requires
**both**:

- `AC_OFF_DEBOUNCE_MS` (300 ms) of elapsed wall-clock time since the first
  HIGH sample, so it no longer depends on loop speed under normal operation, and
- `AC_OFF_MIN_SAMPLES` (20) actual HIGH samples taken during that window, so
  a `loop()` stall spanned by only one or two samples cannot satisfy the
  debounce on its own — `loop()` has to actually keep running and keep
  reading `syncPin` as HIGH throughout, much closer to what the old
  100-consecutive-samples counter guaranteed.

The shot-end branch now also calls `light.setBrightness(0)` explicitly
(neither the old nor the first version of this code did — see finding 7 in
"Review round 1" below).

Both `AC_OFF_DEBOUNCE_MS` and `AC_OFF_MIN_SAMPLES` are still judgement
calls, not measured equivalents of the old 100-iteration debounce — this
environment has no bench to measure actual `loop()` iteration timing under
load, and the old behaviour's real-world duration was itself undocumented
and load-dependent. **This needs bench verification before relying on it**:
confirm a normal shot still ends promptly when the paddle/switch is
released, and that brief zero-cross artefacts don't false-trigger an early
end. If bench testing shows different values are needed, change the two
constants — the surrounding logic does not need to change.

An architectural alternative worth considering in a later phase: derive
off-time from the `dimmable_light` library's existing zero-cross interrupt
instead of polling `syncPin` from `loop()`. That would remove the
dependency on a bench-tuned constant entirely. Not done here to keep this
PR's scope to timing-mechanism changes only, not new signal sources.

## 3. Pump-power ramp cadence: `SetPump()`'s `callCount` → elapsed-time interval

Found during review, not part of the original scope for this PR but the
same defect class as `offcount`, so fixed here rather than deferred (see
finding 3 in "Review round 1" below).

**Before:** `SetPump()` is gated by `millis() - DimlastUpdate > PRESS_INTERVAL`
(correctly elapsed-time-based), but *inside* that gate a `static int
callCount` counted qualifying calls and only adjusted `pumppower` every 4th
one — intending roughly `4 × PRESS_INTERVAL` = 200 ms between adjustments.
That only holds if `SetPump()` is entered at least every `PRESS_INTERVAL`;
under a slow/stalled `loop()`, each entry still passes the outer gate (since
far more than `PRESS_INTERVAL` has elapsed), so `callCount` still only
advances by 1 per call — 4 calls could span far more than 200 ms.

**After:** a dedicated `lastPumpAdjust` timestamp and `PUMP_ADJUST_INTERVAL`
(200 ms) constant replace the counter, matching the same
`now - last >= interval` idiom already used for `PRESS_INTERVAL` and
`PID_INTERVAL` elsewhere in the file. The overpressure cut
(`currentPressure > PressureTarget + 0.3`) is unchanged and still runs on
every pass of the outer `PRESS_INTERVAL` gate, as before.

## 4. Audit of other blocking calls in web/SD/OTA paths

Per the Phase 1 checklist item "audit ... for blocking calls that could
affect migration" — these are documented, not changed, in this phase. They
remain blocking in the current single-loop architecture; each is flagged
with where it is expected to actually be addressed.

| Location | Blocking behaviour | Addressed in |
|---|---|---|
| `handleApplyTheme()` | Byte-by-byte `while (sourceFile.available()) targetFile.write(sourceFile.read());` file copy over SD SPI | Not addressed here; theme files are small, but this is a synchronous SD copy inside an HTTP handler. Candidate for a chunked/non-blocking copy if Phase 5 bench testing shows it disturbs control timing. |
| `handleFileRequest()`, `/` route, `/getConfig` | `server.streamFile()` over SD SPI | Same — inherent to serving files from SD synchronously within `server.handleClient()`. `REWRITE_PLAN.md` Phase 7 explicitly requires verifying "large web requests/uploads do not disturb control timing" once control moves to its own task; until then this only matters because it currently shares the same loop as coffee control. |
| `handleUpload()` | Per-chunk `uploadFile.write()` over SD SPI, called synchronously as HTTP body arrives | Same as above; flagged in `REWRITE_PLAN.md` Phase 7 verification list already. |
| `handleListFiles()` | Synchronously walks the SD root and builds one in-memory JSON string | Move remains on the service side; stress-test a full card in Phase 7. |
| `handleDelete()` | Synchronous `SD.exists()` and `SD.remove()` inside the request handler | Move remains on the service side; verify SD failures cannot affect control timing. |
| `getCurrentTheme()` | Synchronous SD open plus `File.readString()` | Move remains on the service side; it must never be called from the control task. |
| `/saveConfig` | Parses the complete request, removes and rewrites `config.json`, then mutates live PID/config globals | The SD work remains on the service side. Phase 3 must replace direct live-global mutation with validated requests; Phase 4 then carries copied settings through the command queue. |
| `ArduinoOTA.handle()` | Blocks for extended periods while an OTA transfer is in progress (core library behaviour, not this firmware) | `REWRITE_PLAN.md` Phase 7 requires OTA to either refuse while unsafe or force a documented safe state; not solved by Phase 1. |
| `startSD()` / `loadSDConfig()` / `startWiFi()` | `SPI.begin/end`, `SD.begin`, blocking Wi-Fi connect loop (`delay(500)` up to 20 s), boot delays (`delay(2000)` ×2) | Setup-time only, before `loop()` starts and before any actuator can be driven; out of scope for Phase 1 by the plan's own framing (control timing, not boot timing). |

No change was made to any of the above in this PR. They are recorded so
later phases (moving control off `loop()` entirely) address them
deliberately rather than being rediscovered mid-migration.

## Review round 1: findings and fixes

Before being proposed for merge, this branch was reviewed by an independent
model (Opus) across 7 angles (correctness scan, removed-behaviour audit,
cross-file trace, reuse, simplification, efficiency, altitude) with a
verification pass on every candidate. 10 findings were reported. A second,
independently-authored change also landed on this branch around the same
time addressing the buzzer's residual risk directly (§1); the two efforts
are reconciled below rather than one overwriting the other.

1. **(fixed, was the most severe finding)** The AC-off debounce's first
   version used elapsed time alone and could be satisfied by a single
   sample taken right after a `loop()` stall, even though AC was present
   throughout the stall — ending a live shot mid-extraction with no
   `light.setBrightness(0)`, then immediately re-detecting AC and
   restarting the shot from pre-infusion at `pumppower = 255`. Fixed by
   requiring both elapsed time and a minimum count of actually-observed
   samples (§2 above).
2. **(fixed, superseded by a better design)** `queueBuzzer()` could leave
   `BUZZER_PIN` asserted for the duration of any blocking call that ran
   before the buzzer's state was next advanced. Originally documented here
   as an accepted, undosable-within-this-phase risk. A second change
   replaced the `loop()`-serviced state machine with the ESP-timer-driven
   design in §1, which advances pattern edges independently of `loop()`
   entirely and closes this risk rather than just documenting it.
3. **(fixed)** `SetPump()`'s `callCount` was the same loop-count-timing
   defect class as `offcount`, left unaudited. Fixed in §3 above.
4. **(fixed)** `queueBuzzer()` had no guard against a non-positive
   duration, which would cast to a very large duration and could hang
   `waitForBuzzer()` forever if hit from a setup-time call. The final
   timer-driven `queueBuzzer()` (§1) validates all of `times`,
   `beepDurationMs` and `pauseDurationMs` before arming the timer. Currently
   unreachable from any real call site (all pass positive literals); fixed
   as a latent landmine for future callers.
5. **(fixed)** The `acOffSince == 0` sentinel collided with a legitimate
   `millis() == 0` value at the ~49.7-day rollover. Replaced with a
   dedicated `acOffPending` bool (§2 above); low real-world impact
   (self-healed within one iteration) but cheap to remove entirely.
6. **(superseded)** An earlier fix collapsed a redundant `buzzerActive`
   flag out of the `loop()`-serviced buzzer state machine. That state
   machine was itself superseded by the timer-driven design in §1, which
   reintroduces a `buzzerActive` flag — there it is load-bearing: it is the
   guard, checked under a mutex, that both the timer callback and
   `queueBuzzer()` use to coordinate across the two task contexts. No
   redundant state remains in the final design.
7. **(fixed)** Neither the old nor the first version of this PR called
   `light.setBrightness(0)` on shot end — a pre-existing gap, not
   introduced by this PR, but one the original debounce regression (finding
   1) made much more likely to actually be hit. Added explicitly in §2
   above as a small, deliberately-scoped exception to "behaviour
   preservation only," because it directly mitigates the consequence of
   finding 1 and because leaving a known, freshly-identified pump-off gap
   in place once it was found would be irresponsible for actuator-adjacent
   code.
8. **(fixed by the same design change as finding 2)** `queueBuzzer()`
   originally had no re-entrancy guard against being called while a pattern
   was already playing — documented as latent/unreachable rather than
   fixed. The timer-driven `queueBuzzer()` (§1) now explicitly rejects a
   new pattern while one is active, matching the original blocking
   function's implicit contract.
9. **(not fixed, architectural note for later phases)** The AC-off debounce
   is a polled-elapsed-time mechanism; `REWRITE_PLAN.md`'s Phase 1 item also
   allowed deriving off-time from the signal itself (the `dimmable_light`
   library's existing zero-cross interrupt), which the buzzer fix (finding
   2) demonstrates is a viable pattern in this codebase. That would remove
   the dependency on bench-tuned debounce constants entirely and is likely
   closer to what Phase 3+ needs anyway — flagged as a candidate for
   revisiting there rather than expanding this PR's scope now.
10. **(not fixed, low severity, pre-existing)** The actuator-ownership
    baseline understated: `SSR_PIN` is written only by `runPID()`, but pump
    output is written both from the shot state machine and directly from
    the legacy `/adjust` `Pause`/`Resume` commands (`handleAdjust()`). This
    is an existing split-ownership gap, not introduced by this PR — Phase 2
    removes Pause/Resume entirely and Phase 3 makes the control boundary
    the sole actuator writer. Corrected in `BASELINE.md`.

## Behaviour-preservation checklist

- [x] The programmed buzzer durations and final trailing pause match the
      old sequence, and pattern edges no longer depend on `loop()` latency
      at all — a blocking web/SD/OTA call can no longer stretch or hold a
      beep (review round 1, findings 2 and 8).
- [x] Shot-end reset-on-any-"on"-reading semantics preserved exactly.
- [x] Shot-end debounce no longer trusts elapsed time alone; requires
      actually-observed samples too, so a `loop()` stall cannot end a live
      shot on its own (review round 1, finding 1).
- [x] Shot-end now explicitly commands the pump off (review round 1,
      finding 7) — a small, deliberate exception to pure behaviour
      preservation, made because the debounce fix made the old silent gap
      more consequential if left in place.
- [x] `SetPump()`'s pump-power ramp cadence no longer depends on how often
      `SetPump()` happens to be entered (review round 1, finding 3).
- [x] No application-owned FreeRTOS task, queue, state enum, or
      actuator-ownership change in this PR (the buzzer uses the ESP timer
      service, a platform facility, not a task this application owns).
- [ ] Confirm buzzer sound/timing and all machine behaviour on hardware;
      code review and compilation alone do not establish behavioural
      equivalence.
- [ ] Bench re-test of temp-only and a normal shot (see `BASELINE.md` —
      not performed in this environment; required before this firmware runs
      on real hardware, and again after `AC_OFF_DEBOUNCE_MS`/
      `AC_OFF_MIN_SAMPLES` are validated).

## Build verification

Compiled with the same toolchain as the baseline (`esp32:esp32@2.0.17`; see
`BASELINE.md` for why that core version rather than the newest 3.3.11 was
used), after merging both rounds of fixes:

```
Sketch uses 903633 bytes (68%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51212 bytes (15%) of dynamic memory, leaving 276468 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. Compared with the unmodified baseline
(`baseline-phase0`: 902957 bytes flash, 51188 bytes RAM — see
`BASELINE.md`), this adds 676 bytes of flash and 24 bytes of RAM: the
timer-driven buzzer sequencer, the dual-gate AC-off debounce, and the
elapsed-time pump-ramp cadence.
