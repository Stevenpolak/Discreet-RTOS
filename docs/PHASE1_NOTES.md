# Phase 1 notes — remove blocking and loop-count timing

Date: 2026-08-19
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 1, applied on top of the
Phase 0 baseline (`baseline-phase0` tag, see [BASELINE.md](BASELINE.md)).

Goal: keep machine behaviour unchanged while removing the two places control
timing depended on `delay()` or on loop-iteration counts rather than
wall-clock time. No application-owned FreeRTOS task, state model, or
actuator-ownership change happens in this phase — that is Phases 2–5.

## 1. Buzzer: `delay()`-based blocking → timer-driven sequencer

**Before:** `beepBuzzer(times, beepDuration, pauseDuration)` toggled
`BUZZER_PIN` in a `for` loop using `delay()` between edges. Every call
blocked whichever code path invoked it for the full pattern duration
(e.g. `beepBuzzer(3,100,100)` blocks ~600 ms).

**After:** `queueBuzzer()` starts a small state machine whose edges are driven
by an ESP one-shot timer. The timer callback runs through the ESP timer
service, independently of Arduino `loop()`. Consequently a slow HTTP, SD or
OTA call cannot stretch an ON pulse. This does not create a third
application-owned task; it uses a platform service that already exists.
`waitForBuzzer()` only polls completion during setup to retain the original
boot sequence.

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
  needed to run. It now calls `queueBuzzer()` only; timer callbacks advance
  the pattern without blocking coffee logic and without depending on loop
  latency.

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

**After:**

```cpp
if (digitalRead(syncPin) == HIGH) {
  if (acOffSince == 0) acOffSince = millis();
} else {
  acOffSince = 0;
}
if (acOffSince != 0 && millis() - acOffSince >= AC_OFF_DEBOUNCE_MS) {
  /* end shot */
}
```

Same reset-on-any-LOW-reading semantics as the original (a single "on"
sample immediately cancels the pending off-debounce), but the "how long has
it been continuously off" question is now answered with `millis()` instead
of an iteration count, so it no longer depends on loop speed.

`AC_OFF_DEBOUNCE_MS` is currently set to **300 ms**. This is a judgement
call, not a measured equivalence to the old 100-iteration debounce — this
environment has no bench to measure actual `loop()` iteration timing under
load, and the old behaviour's real-world duration was itself undocumented
and load-dependent. **This needs bench verification before relying on it**:
confirm a normal shot still ends promptly when the paddle/switch is
released, and that brief zero-cross artefacts don't false-trigger an early
end. If bench testing shows a different value is needed, change
`AC_OFF_DEBOUNCE_MS` alone — the surrounding logic does not need to change.

## 3. Audit of other blocking calls in web/SD/OTA paths

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

## Behaviour-preservation checklist

- [x] The programmed buzzer durations and final trailing pause match the old
      sequence, and edges no longer depend on `loop()` latency.
- [x] Shot-end reset-on-any-"on"-reading semantics preserved exactly.
- [x] No application-owned FreeRTOS task, queue, state enum, or
      actuator-ownership change in this PR.
- [ ] Confirm buzzer sound/timing and all machine behaviour on hardware; code
      review and compilation alone do not establish behavioural equivalence.
- [ ] Bench re-test of temp-only and a normal shot (see `BASELINE.md` —
      not performed in this environment; required before this firmware runs
      on real hardware, and again after `AC_OFF_DEBOUNCE_MS` is validated).

## Build verification

Compiled with the same toolchain as the baseline (`esp32:esp32@2.0.17`; see
`BASELINE.md` for why that core version rather than the newest 3.3.11 was
used):

```
Sketch uses 903605 bytes (68%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51212 bytes (15%) of dynamic memory, leaving 276468 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. Compared with the unmodified baseline, the
timer-driven sequencer and elapsed-time debounce add 648 bytes of flash and
24 bytes of RAM.
