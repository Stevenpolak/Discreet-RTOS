# Phase 1 notes — remove blocking and loop-count timing

Date: 2026-08-19
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 1, applied on top of the
Phase 0 baseline (`baseline-phase0` tag, see [BASELINE.md](BASELINE.md)).

Goal: keep machine behaviour unchanged while removing the places control
timing depended on `delay()` or on loop-iteration counts rather than
wall-clock time. No FreeRTOS task, state model, or actuator-ownership change
happens in this phase — that is Phases 2–5.

**This PR went through an independent review round before being proposed for
merge; see "Review round 1 findings and fixes" below.** The first version of
this change had a real regression in the AC-off debounce (it could end a
live shot mid-extraction and restart it at full pump power, under exactly
the kind of `loop()` stall this phase is trying to make control timing
independent of) and left one other loop-count timing dependency
(`SetPump()`'s `callCount`) unaddressed despite the Phase 1 exit condition
claiming otherwise. Both are fixed in the version described below.

## 1. Buzzer: `delay()`-based blocking → non-blocking sequencer

**Before:** `beepBuzzer(times, beepDuration, pauseDuration)` toggled
`BUZZER_PIN` in a `for` loop using `delay()` between edges. Every call
blocked whichever code path invoked it for the full pattern duration
(e.g. `beepBuzzer(3,100,100)` blocks ~600 ms).

**After:** a small state machine (`queueBuzzer()` / `serviceBuzzer()` /
`waitForBuzzer()`) tracks the current beep phase against `millis()`.
`serviceBuzzer()` is called once per `loop()` iteration and advances the
pattern by at most one phase transition per call; it never blocks.

**Call sites**, and why each is safe to leave as-is or was changed:

- `startSD()` (called from `loadSDConfig()` and again directly in `setup()`)
  and `startWiFi()`: both only ever run during `setup()`, before `loop()`
  starts. Nothing else needs to run concurrently at that point, so these now
  call `queueBuzzer()` followed by `waitForBuzzer()` — a blocking wait, but
  one built on the same non-blocking engine rather than a second, duplicate
  timing implementation. Boot-time beep timing is unchanged from the
  original behaviour.
- `steam()`: called from every `loop()` iteration. This is the call site
  that mattered — the original blocking `beepBuzzer(3,100,100)` here could
  stall `GetPressure()`, `runPID()`, `SetPump()` and AC/shot-state handling
  for ~600 ms if steam threshold was crossed while other control logic
  needed to run. It now calls `queueBuzzer()` only; `serviceBuzzer()` in
  `loop()` advances the pattern without blocking anything else.

**Known residual risk, not fully closed by this phase:** `serviceBuzzer()`
can only advance a phase when it is actually called. If a *later* `loop()`
iteration blocks in one of the still-blocking paths in §3 below (SD
streaming, upload, OTA) while a pattern is mid-flight, the pin holds
whatever level it was last set to for the entire stall — e.g. a queued
"beep" can sound continuously for seconds instead of the intended 100 ms.
This cannot be fixed inside the buzzer engine itself without either
reintroducing blocking (defeating the point of this change) or removing the
underlying blocking calls (Phase 3+). It is a real, audible behaviour change
from the baseline, but it is a sound-quality issue only — `BUZZER_PIN` is
not shared with any actuator, so it cannot affect heater or pump output.
Documented here rather than silently claimed as fully preserved.

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
| `ArduinoOTA.handle()` | Blocks for extended periods while an OTA transfer is in progress (core library behaviour, not this firmware) | `REWRITE_PLAN.md` Phase 7 requires OTA to either refuse while unsafe or force a documented safe state; not solved by Phase 1. |
| `startSD()` / `loadSDConfig()` / `startWiFi()` | `SPI.begin/end`, `SD.begin`, blocking Wi-Fi connect loop (`delay(500)` up to 20 s), boot delays (`delay(2000)` ×2) | Setup-time only, before `loop()` starts and before any actuator can be driven; out of scope for Phase 1 by the plan's own framing (control timing, not boot timing). |

No change was made to any of the above in this PR. They are recorded so
later phases (moving control off `loop()` entirely) address them
deliberately rather than being rediscovered mid-migration.

## Review round 1: findings and fixes

Before being proposed for merge, this PR was reviewed by an independent
model (Opus) across 7 angles (correctness scan, removed-behaviour audit,
cross-file trace, reuse, simplification, efficiency, altitude) with a
verification pass on every candidate. 10 findings were reported; the ones
that changed the code:

1. **(fixed, was the most severe finding)** The AC-off debounce's first
   version used elapsed time alone and could be satisfied by a single
   sample taken right after a `loop()` stall, even though AC was present
   throughout the stall — ending a live shot mid-extraction with no
   `light.setBrightness(0)`, then immediately re-detecting AC and
   restarting the shot from pre-infusion at `pumppower = 255`. Fixed by
   requiring both elapsed time and a minimum count of actually-observed
   samples (§2 above).
2. **(documented, not fully fixable in this phase)** `queueBuzzer()` can
   leave `BUZZER_PIN` asserted for the duration of any blocking call that
   runs before the next `serviceBuzzer()`. See the "known residual risk"
   note in §1 above.
3. **(fixed)** `SetPump()`'s `callCount` was the same loop-count-timing
   defect class as `offcount`, left unaudited. Fixed in §3 above.
4. **(fixed)** `queueBuzzer()` had no guard against a non-positive
   duration, which would cast to a ~49-day value and could hang
   `waitForBuzzer()` forever if hit from a setup-time call. Now validated
   (`beepDurationMs <= 0 || pauseDurationMs < 0` is rejected). Currently
   unreachable from any real call site (all pass positive literals), fixed
   as a latent landmine for future callers.
5. **(fixed)** The `acOffSince == 0` sentinel collided with a legitimate
   `millis() == 0` value at the ~49.7-day rollover. Replaced with a
   dedicated `acOffPending` bool (§2 above); low real-world impact
   (self-healed within one iteration) but cheap to remove entirely.
6. **(fixed as a side effect of the §2 rewrite)** `buzzerActive` was
   redundant with `buzzerBeepsRemaining > 0`; collapsed to one fewer global
   with no behaviour change.
7. **(fixed)** Neither the old nor the first version of this PR called
   `light.setBrightness(0)` on shot end — a pre-existing gap, not
   introduced by this PR, but one the original debounce regression (finding
   1) made much more likely to actually be hit. Added explicitly in §2
   above as a small, deliberately-scoped exception to "behaviour
   preservation only," because it directly mitigates the consequence of
   finding 1 and because leaving a known, freshly-identified pump-off gap
   in place once it was found would be irresponsible for actuator-adjacent
   code.
8. **(not fixed, latent/low severity)** `queueBuzzer()` has no re-entrancy
   guard against being called while a pattern is already playing. Currently
   unreachable at all existing call sites; left as-is rather than adding
   unneeded complexity for a scenario nothing can currently trigger.
9. **(not fixed, architectural note for later phases)** `AC_OFF_DEBOUNCE_MS`
   is a polled-elapsed-time constant; `REWRITE_PLAN.md`'s Phase 1 item also
   allowed deriving off-time from the signal itself (the `dimmable_light`
   library's existing zero-cross interrupt). That would remove the
   dependency on a bench-tuned constant entirely and is likely closer to
   what Phase 3+ needs anyway — flagged as a candidate for revisiting there
   rather than expanding this PR's scope now.

## Behaviour-preservation checklist

- [x] Buzzer patterns play with the same on/off timing as before when
      `loop()` isn't stalled, and setup-time callers still wait for the
      pattern to finish, matching original boot timing. **Exception:** a
      pattern can be stretched or held on by a blocking call elsewhere in
      `loop()` — see the "known residual risk" note in §1. Sound-quality
      only; cannot affect heater/pump output.
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
- [x] No FreeRTOS task, queue, state enum, or actuator-ownership change in
      this PR.
- [ ] Bench re-test of temp-only and a normal shot (see `BASELINE.md` —
      not performed in this environment; required before this firmware runs
      on real hardware, and again after `AC_OFF_DEBOUNCE_MS`/
      `AC_OFF_MIN_SAMPLES` are validated).

## Build verification

Compiled with the same toolchain as the baseline (`esp32:esp32@2.0.17`; see
`BASELINE.md` for why that core version rather than the newest 3.3.11 was
used):

```
Sketch uses 903269 bytes (68%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51204 bytes (15%) of dynamic memory, leaving 276476 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. Flash/RAM usage is essentially unchanged
from the baseline (+312 bytes flash, +16 bytes RAM), consistent with the
buzzer state machine, the dual-gate AC-off debounce, and the elapsed-time
pump-ramp cadence added by this PR and its review-round fixes.
