# Phase 4 notes — add safe task communication

Date: 2026-08-23
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 4, applied on top of the
merged Phase 0-3 work (`v2.0.2-beta`, see [BASELINE.md](BASELINE.md),
[PHASE1_NOTES.md](PHASE1_NOTES.md), [PHASE2_NOTES.md](PHASE2_NOTES.md),
[PHASE3_NOTES.md](PHASE3_NOTES.md)).

Goal per `REWRITE_PLAN.md`: all cross-boundary data flows through defined
messages. **Still no second task.** Everything in this PR runs in the same
single Arduino task as before - `commandQueue`/`telemetryQueue` are real
FreeRTOS queues, but right now they have exactly one producer and one
consumer that happen to be the same task calling itself through `loop()`.
The point of this phase is that the *mechanism* is already exactly what
Phase 5's dedicated control task will need, so moving the control logic
into that task later is a relocation, not a redesign.

## What changed

### 1. `commandQueue` — the only channel from an HTTP handler into control state

Before this PR, `handleAdjust()` called `applyControlCommand()` directly,
and `/saveConfig` called `acceptBrewSetpointEdit()` directly and wrote
`PIDonly`/`currentMode` directly. All three now only ever construct a
`ControlCommand` and `xQueueSend()` it (`0` ticks-to-wait - never blocks
the web server). `applyControlCommand()` - the "control owner" per the
plan's own language - is now called from exactly one place:
`controlStep()`'s queue-drain loop, which runs before anything else that
cycle:

```cpp
ControlCommand cmd;
while (commandQueue != nullptr && xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
  applyControlCommand(cmd);
}
```

Verified by grep: `applyControlCommand()` and every `accept*Edit()`
function it calls have exactly one call site each, and none of them are
inside an HTTP handler.

**Two `ControlCommand` types were added** to close the gaps this exact
scope caused in earlier phases:

- `SET_BREW_SETPOINT_ABSOLUTE` (`double value`) - `/saveConfig` posts a
  complete target, not a delta like `/adjust` does, so it needed its own
  command shape rather than being forced through the delta-based
  `SET_BREW_SETPOINT`.
- `SET_MODE` (`OperatingMode mode`) - closes a gap the Phase 3 review
  flagged and left as a note for later: `currentMode` is safety-critical
  (`controlStep()`'s output-priority resolution reads it directly), but it
  was still being written straight from `/saveConfig` with no path through
  the control owner at all. `applyControlCommand()`'s `SET_MODE` case
  keeps the legacy `PIDonly` mirror in sync, matching how `PIDonly` is
  still what actually gets persisted to `config.json`.

### 2. Why no separate "coalesce repeated edits" logic was needed

`REWRITE_PLAN.md`'s Phase 4 checklist explicitly asks to "coalesce
repeated edits so the latest complete pending revision wins." The naive
reading suggests deduplicating the queue itself (e.g. collapsing three
queued `SET_PREINFTIME` deltas into one before applying). That's not what
this PR does, because it isn't needed: `applyControlCommand()` already
reads the *current* `pendingSettings`/`activeSettings` value at the moment
each command is actually applied (not at the moment it was submitted) -
see `bool idle = !acDetected; ... base = idle ? activeSettings.X :
pendingSettings.X;`. Draining the whole queue in FIFO order at the top of
every `controlStep()` cycle and applying each command against the
then-current base means three queued `+1` deltas simply accumulate to
`+3`, exactly as if a person had clicked the button three times and waited
for each click to land - which is precisely "the latest complete pending
revision wins," achieved as a consequence of *when* commands are read
rather than by deduplicating what's sitting in the queue.

This also directly answers a concern the Phase 3 review raised and
deferred: `ControlCommand`'s relative-`delta` shape looked incompatible
with coalescing at the time, because the reviewer was (correctly, at that
point) imagining a queue read by a task running at its own independent
cadence, where a delta computed against a stale base would be wrong. In
this single-task design, "the base" is always read fresh at apply time,
so the delta shape is fine. **This reasoning changes once Phase 5
actually splits the two tasks apart** - see "What was intentionally not
done" below.

### 3. `telemetryQueue` — the only channel out of control state to the web side

`buildTelemetrySnapshot()` (unchanged internally) is called once at the
end of every `controlStep()` cycle, after the actuator writes (so
`resolvedHeaterOn`/`resolvedPumpPower` reflect what was actually just
written, not the previous cycle's values), and published with
`xQueueOverwrite()` - which never blocks and never fails on a length-1
queue, by definition. `handleGetValues()` reads it with `xQueuePeek(...,
0)` instead of calling `buildTelemetrySnapshot()` directly, falling back
to a direct build only if the queue isn't ready yet (creation failed, or a
request arrives before `controlStep()` has run even once - practically
unreachable since `setup()` fully completes, including queue creation,
before `loop()` starts, but cheap to guard against).

**`TelemetrySnapshot` gained four fields**
(`displaySetpoint`/`pendingPreinftime`/`pendingBloomtime`/
`pendingPressuresetpoint`) specifically so `handleGetValues()` no longer
needs to read `pendingSettings`/`setpoint`/`steamRequested` directly -
those are exactly the "shared mutable control globals" this phase's
checklist item asks to confirm are gone from the web-handler side.
`displaySetpoint` carries the same `steamRequested ? setpoint :
pendingSettings.setpoint` resolution `handleGetValues()` has applied since
Phase 2, just computed once inside `buildTelemetrySnapshot()` now instead
of in the handler.

### 4. What still reads/writes a global directly, and why that's an
explicit, not accidental, boundary

- `Kp`/`Ki`/`Kd`/`offset`/`steamSetpoint` - written directly from
  `/saveConfig`, read directly in `handleGetValues()`/`handleTemp()`. This
  is the same scope decision Phase 2 made and Phase 3 reaffirmed: these
  are not in the settings-lifecycle latch, are not actuator-critical in
  the same sense as brew setpoint/pressure/phase durations, and were never
  asked to be revisited. Left alone here too, for the same reason.
- `actime`/`brewTemp` - legacy display-only fields, read directly in
  `handleGetValues()`. Neither affects control behaviour; not part of the
  state model this phase's checklist is protecting.
- `/saveConfig`'s fallback read of `doc["PIDonly"] | PIDonly` when
  constructing the `SET_MODE` command - a read of the current value to use
  as a default if the client's JSON omits the field, not a control
  decision itself; the actual mode change still goes through
  `applyControlCommand()`.

## What was intentionally *not* done in this phase

- **No FreeRTOS task.** `commandQueue`/`telemetryQueue` have one producer
  and one consumer today, both running in the same Arduino task. Creating
  the actual control task is Phase 5.
- **`ControlCommand`'s relative-delta shape was not changed to absolute
  values**, even though that was flagged as a Phase 3 review concern and
  even though `SET_BREW_SETPOINT_ABSOLUTE`/`SET_MODE` (added in this PR)
  already demonstrate the struct can carry an absolute value just fine.
  Reasoning: with a single task draining the queue immediately (see §2),
  the delta shape is provably correct today - changing it now would be
  speculative hardening against a race that doesn't exist yet, in the same
  PR that's supposed to be "just plumbing." Whether Phase 5's dedicated
  task drains the queue at the same effectively-immediate cadence this
  code does (in which case the reasoning in §2 still holds) or introduces
  real latency between submission and application (in which case a
  relative delta *does* become unsafe to coalesce and the type needs to
  change to absolute values first) is a question for Phase 5's own design,
  not something to guess at here.
- **No change to queue depth tuning or behaviour under sustained overload**
  beyond the 503 response on a full `commandQueue` - see "Bench items"
  below.
- **`Kp`/`Ki`/`Kd`/`offset`/`steamSetpoint`/`actime`/`brewTemp`** remain
  direct global reads/writes from the web-handler side - see §4 above.

## Build verification

Compiled with the same toolchain as Phase 0-3 (`esp32:esp32@2.0.17`), after
the review-round-1 fixes below:

```
Sketch uses 906733 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51292 bytes (15%) of dynamic memory, leaving 276388 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +1112 bytes flash / +8 bytes RAM versus the
merged Phase 0-3 state (`v2.0.2-beta`: 905,621 bytes flash, 51,284 bytes
RAM) - the two `xQueueCreate()`-backed queues, the two new `ControlCommand`
fields, four new `TelemetrySnapshot` fields, the `sendControlCommand()`
helper / `endShot()` call added in review, net of the code-size drop from
no longer calling `millis()` inside `buildTelemetrySnapshot()`.

## Behaviour-preservation checklist

- [x] `applyControlCommand()`'s validation/constrain logic is completely
      unchanged - only how it's invoked (from a queue-drain loop instead
      of directly from an HTTP handler) is new.
- [x] A normal `/adjust` request still applies within the same
      `controlStep()` cycle in practice: `handleClient()` runs immediately
      before `controlStep()` in `loop()`, so a command sent during this
      iteration's HTTP handling is drained and applied during this same
      iteration's `controlStep()` call, same effective latency as a direct
      call.
- [x] `/getValues`' response shape and field values are unchanged for
      every existing field - only the fields' *source* moved from direct
      global reads to a `TelemetrySnapshot` field.
- [ ] **New failure mode, not previously possible:** an `/adjust` request
      can now receive `503 Busy, try again` if `commandQueue` is full (a
      burst deeper than `COMMAND_QUEUE_DEPTH` = 8 queued-but-undrained
      commands - not reachable in practice given the drain happens every
      `loop()` iteration, but a real, testable behaviour change from the
      old handler, which never rejected a request). `/saveConfig` gained
      the same failure mode in review round 1 - see below.
- [ ] Bench re-test still required, same hardware limitation as every
      prior phase. This phase adds: confirm `/adjust` edits still apply
      with no perceptible added latency; confirm `/saveConfig`'s setpoint
      and mode changes still take effect (now via the queue); confirm
      `/getValues` still reflects reality throughout a shot, a fault, and
      a mode change; attempt a rapid-fire `/adjust` burst and confirm
      either normal application or an explicit `503`, never a silently
      dropped edit; confirm flipping to temp-only mid-shot now ends the
      shot outright (matching a fault) rather than just masking the pump
      for as long as the mode stays flipped.

## Review round 1: findings and fixes

Same independent multi-angle review as every prior phase. One gap was
already self-identified while writing the review prompts (the
`/saveConfig` `xQueueSend()` return-value gap below); the review
independently confirmed it from four separate angles and surfaced two more
real, safety-relevant issues.

### Fixed

1. **`/saveConfig` never checked `xQueueSend()`'s return value.** Both the
   setpoint and mode commands were sent with `if (commandQueue != nullptr)
   xQueueSend(...)`, discarding the result, and the handler always answered
   `200 "Saved"`. Because `/saveConfig` writes `config.json` to SD *before*
   queuing either command, a full or uncreated `commandQueue` meant the
   persisted file and the running machine could disagree - silently, with
   no client-visible signal - until the next reboot. `handleAdjust()`
   already had the correct pattern (503 on failure); `/saveConfig` didn't.
   Fixed by extracting `sendControlCommand()` - a single `commandQueue !=
   nullptr && xQueueSend(..., 0) == pdTRUE` check both handlers now share -
   and having `/saveConfig` answer `503` if either send fails, matching
   `handleAdjust()`. See `sendControlCommand()` and both call sites.

2. **A mode change to temp-only mid-shot masked the pump's output but never
   ended the shot**, unlike a fault. `controlStep()`'s output-priority
   resolution already forced `pumpOut = 0` while `currentMode ==
   TEMP_ONLY`, but the shot-phase branches earlier in the same function
   kept running underneath that mask - with the pump not actually turning,
   `currentPressure` reads low, which re-arms `pumppower = 255` on the next
   cycle. The moment the mode flipped back to `NORMAL` mid-shot, the pump
   would slam to whatever wound up under the mask. This is the same
   masked-output windup/slam bug Phase 3's review found and fixed for
   faults via `endShot()` - `SET_MODE` is a new path into that same
   priority-resolution code this phase adds, and needed the same fix.
   Fixed by calling `endShot()` alongside the existing `pumpOut = 0` when
   `currentMode == TEMP_ONLY` and `acDetected`, mirroring the fault
   handling immediately above it in `controlStep()`.

3. **`SET_BREW_SETPOINT_ABSOLUTE` had no `constrain()`**, unlike every
   other setpoint-bearing case in `applyControlCommand()`'s switch. This
   matched `/saveConfig`'s pre-existing behaviour (never range-checked
   before this phase either), but this phase is what routes it through
   `applyControlCommand()` and checks off REWRITE_PLAN.md's "reject unsafe
   or invalid settings inside the control owner" - the one command type
   that newly reaches the control owner was also the one left unvalidated.
   A client posting `{"setpoint": 500}` to `/saveConfig` would have set an
   unbounded heater target with no PID-side ceiling. Fixed by clamping to
   the same `10 + offset .. 96 + offset` range `SET_BREW_SETPOINT` uses.

4. **`buildTelemetrySnapshot()` called `millis()` itself** instead of
   taking `now` as a parameter, even though its only hot-path caller
   (`controlStep()`) already samples `now` at the top of the function for
   exactly this purpose - and `controlStep()`'s own doc comment already
   states that convention. The efficiency angle measured `millis()` on this
   target as a ~250-400 cycle call (a real hardware-timer read plus a
   software 64-bit divide, there being no hardware 64-bit divide on the
   LX6 core), making it the single largest new per-cycle cost this phase
   added - larger than either new queue operation. Fixed by giving
   `buildTelemetrySnapshot()` a `now` parameter; `controlStep()` passes the
   `now` it already has, and `handleGetValues()`'s direct-build fallback
   (which has no `now` of its own) passes `millis()` explicitly, so its
   behaviour is unchanged.

### Considered, not changed - would trade correctness for throughput

- **Gating the telemetry publish to a periodic interval** (matching
  `GetPressure()`'s `PRESS_INTERVAL` gate) instead of publishing every
  cycle would remove most of the remaining new per-cycle cost (an
  `xQueueReceive` on an empty queue, plus the `xQueueOverwrite`'s critical
  section - together on the order of 1-2 µs, small enough at 240 MHz to be
  a rounding error next to `server.handleClient()`/`ArduinoOTA.handle()`'s
  tens-of-µs socket polls, but real). Not done: `resolvedHeaterOn`/
  `resolvedPumpPower` are specifically published *every* cycle, right after
  the actuator writes, so a bench operator watching telemetry sees a fault
  or mode restriction take effect on the same cycle it happens - gating the
  publish would reintroduce exactly the staleness Phase 3's review added
  those two fields to eliminate. A throughput win here isn't worth trading
  away.

### Documented, not fixed - deliberately out of this phase's scope

- **The front end doesn't check the HTTP response status.**
  `Discreet_Front_End/scripts.js`'s `adjust()` and `config.js`'s save
  handler both proceed unconditionally after `fetch()` resolves, so the
  `503` this phase added (and just extended to `/saveConfig`) is invisible
  in the UI - a rejected edit looks identical to an accepted one. Real, but
  front-end behaviour is outside this phase's stated scope (queue
  plumbing on the firmware side); left as a known gap for whichever phase
  next touches the front end, rather than changing UI code inside a
  backend-plumbing PR.
- **A few more control-relevant globals are still read directly from
  HTTP-handler context** beyond the `Kp`/`Ki`/`Kd`/`offset`/`steamSetpoint`/
  `actime`/`brewTemp` exceptions §4 already names: `handleTemp()` reads the
  live `setpoint` global directly (one of the exact globals `handleGetValues()`
  no longer touches), and `offset` doubles as the safety clamp bound at the
  `constrain()` calls in `applyControlCommand()` in addition to being a
  display-only adjustment - both pre-existing, both single-task-safe today,
  both worth re-examining once Phase 5 makes these genuinely concurrent
  reads. The REWRITE_PLAN.md "confirm no shared mutable control globals
  remain" checkbox is left checked with this note added rather than
  unchecked, since the phase's actual scope (getting settings/mode edits
  and telemetry off direct handler access) is met - these are narrower,
  pre-existing residuals, not something this phase reintroduced.
- **`/saveConfig`'s SD write and its two queued commands are not atomic.**
  The file write happens first and always completes; the setpoint and mode
  commands are then two independent `xQueueSend()` calls, so it's possible
  (though not reachable today given the queue is drained every `loop()`
  iteration and can only ever hold at most 2 pending commands from this
  handler) for one to succeed and the other to fail. Not fixed here -
  making config persistence and control-state application a single atomic
  step is a larger redesign than this phase's "route existing writes
  through a queue" scope, and is better addressed once Phase 5's real task
  split forces a decision about config persistence ownership anyway.
