# Phase 2 notes — explicit state and data models

Date: 2026-08-23
Scope: [REWRITE_PLAN.md](REWRITE_PLAN.md) Phase 2, applied on top of the
merged Phase 0 + 1 work (`v2.0.0-alpha`, see [BASELINE.md](BASELINE.md) and
[PHASE1_NOTES.md](PHASE1_NOTES.md)).

Goal per `REWRITE_PLAN.md`: give modes, shot states, settings, commands and
telemetry explicit definitions, **without yet requiring a second task**.
This PR does not create a FreeRTOS task, a queue, or a single
`controlStep()` boundary — those are Phases 3-5. Everything here still runs
synchronously inside the single Arduino `loop()` and its HTTP handlers.

## What changed

### 1. `OperatingMode`, `ShotState`, `FaultCode`

Three scoped enums, each with a `toString()` for telemetry/logging:

- `OperatingMode { NORMAL, TEMP_ONLY }` — mirrors the existing `PIDonly`
  bool. `PIDonly` itself is **not removed**: it still gates AC detection in
  `loop()` (`!PIDonly` in the AC-detect condition) exactly as before.
  `currentMode` is kept in sync wherever `PIDonly` is assigned
  (`loadSDConfig()`, `/saveConfig`) so it's always an accurate mirror, but
  nothing currently branches on `currentMode` itself. Kept as a low-risk
  mirror rather than replacing `PIDonly` outright, since that would mean
  auditing every `PIDonly` read for behavioural equivalence without
  hardware to verify against.
- `ShotState { IDLE, PREINFUSION, BLOOM, EXTRACTION, COMPLETE, FAULT }` —
  unlike `OperatingMode`, this one is **not** a passive mirror for the shot
  phases: `currentShotState` is assigned inside the same pre-infusion /
  bloom / extraction branches that already existed, using the exact same
  conditions as before (now reading from `shotSettings`, see below). It
  accurately labels which branch executed each iteration without changing
  when transitions occur. `COMPLETE` is set for exactly one iteration, at
  the moment the AC-off debounce fires; the following iteration's `else`
  branch (`!acDetected`) reports `IDLE`. There is currently no dwell period
  in `COMPLETE` — none existed before Phase 2 either.
- `FaultCode { NONE, INVALID_TEMPERATURE }` — set/cleared in `runPID()` at
  the same point that already existed (the `isnan(input) || input < 0 ||
  input > 160` check). No new fault detection was added.

### 2. FAULT priority is observability-only in this phase

`REWRITE_PLAN.md`'s operating model says "FAULT may be entered from any
state and has priority over all normal outputs." This PR implements the
first half only: after the shot-phase branches run, `if (currentFault !=
FaultCode::NONE) currentShotState = ShotState::FAULT;` overrides the
*reported* state. **It does not touch actuator outputs** — the
pre-infusion/bloom/extraction branches above it still run their pump-control
logic exactly as before, regardless of fault status, and `runPID()` already
independently forces the SSR off on the same condition. Making FAULT
actually pre-empt pump control (and everything else) requires the single
`controlStep()` boundary and output-priority ordering that
`REWRITE_PLAN.md` puts in Phase 3 ("Apply output priority: fault/safety
off, mode restrictions, state-machine demand, controller output"). Doing
that here, ahead of schedule and without hardware to verify the interaction
of a temperature fault with an in-progress shot's pump behaviour, would be
exactly the kind of change this project's own safety framing warns against.
Recorded as a known, deliberate scope boundary, not an oversight.

### 3. Pause/Resume removed

`handleAdjust()`'s `"Pause"`/`"Resume"` branches (direct
`light.setBrightness()` calls) are gone, per `REWRITE_PLAN.md`'s operating
model ("There is no `PAUSED` state and no ambiguous question about whether
shot time should freeze") and the Phase 2 checklist. Checked the SD-card
front-end archive (`Discreet_Front_End/SD-Card.zip`) for any UI reference to
Pause/Resume — there is none, so no front-end change was needed. A request
with `var=Pause` or `var=Resume` now falls through to
`ControlCommand::Type::NONE` and is a no-op that still returns `200 OK`,
matching the old handler's behaviour for any other unrecognized `var`.

### 4. `ControlSettings`, `ControlCommand`, `TelemetrySnapshot`

Defined as value-only structs (`docs/REWRITE_PLAN.md`'s "Do not pass
`String` objects, raw pointers or references to mutable web/JSON data" -
none of these three types do).

**Scope decision on which fields are latched:** `REWRITE_PLAN.md`'s
"Settings lifecycle" section specifically says "changing brew temperature,
pressure or a phase duration mid-shot cannot alter the shot already in
progress." `ControlSettings` holds exactly those four fields: `setpoint`
(brew temperature), `pressuresetpoint` (pressure target), `preinftime` and
`bloomtime` (phase durations). `Kp`/`Ki`/`Kd`, `offset`, `steamSetpoint` and
`PIDonly` are not named there and remain plain, immediately-applied
globals, unchanged from Phase 1 - editing them still takes effect
instantly regardless of shot state, exactly as before this PR.

**`ControlCommand` is used, not just defined:** `handleAdjust()` now builds
one from the HTTP request and passes it to `applyControlCommand()`, which
validates (same `constrain()` ranges the old inline `if`/`else if` chain
used) and applies it. This is still fully synchronous - no queue, no
cross-task boundary - but it means Phase 3's "route web changes through
validated requests" already has a request object to route, rather than
needing another redefinition later.

**`TelemetrySnapshot` is used, not just defined:** `buildTelemetrySnapshot()`
is called from `handleGetValues()`, which now returns the existing fields
unchanged plus `mode`, `shotState`, `fault` (as strings) and the three
settings revision numbers, additively. Not yet published through the
length-1 overwrite queue `REWRITE_PLAN.md` describes for telemetry - that
needs a second task to publish into it (Phase 4/5).

### 5. Settings lifecycle: `activeSettings` / `shotSettings` / `pendingSettings`

This is the one genuinely new *behaviour* in this PR (Pause/Resume removal
being the other), not just a structural relabeling - `REWRITE_PLAN.md`
explicitly scopes it to Phase 2 ("Define the atomic `pendingSettings ->
activeSettings` promotion on return to `IDLE`").

- **While idle** (`currentShotState == ShotState::IDLE`), an accepted edit
  updates `activeSettings` immediately (and `pendingSettings` in lockstep,
  so they never disagree while idle) - there's no in-progress shot to
  protect, and the UI expects to see its own edit reflected right away
  (see the `updateLabels()` note below).
- **Mid-shot**, an accepted edit updates only `pendingSettings`;
  `shotSettings` (latched from `activeSettings` at shot start) and the
  running shot are unaffected.
- **At shot start** (`IDLE -> PREINFUSION`), `shotSettings = activeSettings`,
  plus the existing preinf-auto-bump-to-8-if-bloom-enabled quirk (now
  applied to `shotSettings.preinftime`, so it only affects the current
  shot's latched copy, not the saved setting).
- **At shot end** (the AC-off debounce firing), `activeSettings =
  pendingSettings` - the latest valid edit, whether made this shot or
  earlier, becomes active for the next shot.

This was previously undefined: the baseline had no distinction at all - a
mid-shot `/adjust` edit mutated the single live global immediately, which
is a real behaviour change this PR intentionally makes, per the plan's own
explicit scope.

**`setpoint` and the PID library binding:** `myPID` is constructed with a
pointer to `setpoint` (`PID myPID(&input, &output, &setpoint, ...)`), so it
can't be redirected to a struct field. `setpoint` stays as the one global
PID reads, but it is now only ever written at the specific points the
settings-lifecycle state machine changes what's "current": shot start,
shot end, an idle-time edit, and `steam`/`stopsteam`. `steaming`'s own
override (set by the automatic `steam()` threshold check) still takes
unconditional precedence at every one of those points, unchanged from
before.

**`setpointBoot` retired:** the old code kept `setpointBoot` in sync with
every `setpoint` edit specifically so `stopsteam` could restore "the last
user-set target" - which is exactly what `activeSettings.setpoint` now is
by construction, whenever not mid-shot. `stopsteam` now restores to
`activeSettings.setpoint` directly.

**Bug fix found while consolidating `setpointBoot`:** the old
`loadSDConfig()`'s no-config-file fallback branch set `setpoint = 93;`
(no offset added) but `setpointBoot = setpoint + offset;` (offset added) -
the two would disagree by `offset` whenever config.json was missing/
unreadable. Since both are being merged into one canonical value
(`activeSettings.setpoint`), the disagreement had to be resolved one way;
fixed by adding `offset` uniformly to `setpoint` in both branches. Only
reachable when config.json fails to load *and* steam mode is used - a
narrow, low-severity latent bug, not something this PR set out to fix, but
unavoidable to leave ambiguous once the two variables were merged.

### 6. `handleGetValues()` field semantics

`preinftime`/`bloomtime`/`pressuresetpoint`/`setpoint` in the JSON response
now come from **`pendingSettings`**, not `activeSettings` or
`shotSettings`. The front end's `scripts.js` calls `/getValues` via
`updateLabels()` immediately after every `/adjust` request and expects to
see that edit reflected right away, whether idle or mid-shot - reading
`pendingSettings` (the latest edit, promoted or not) satisfies that. While
idle, `pendingSettings == activeSettings`, so this is unchanged from Phase
1 in the common case; only mid-shot does it now show the pending edit
rather than a value that used to be mutated live with no latch at all.

## What was intentionally *not* done in this phase

- No FreeRTOS task, queue, or `controlStep()` boundary (Phases 3-5).
- No actual fault-priority actuator arbitration (Phase 3) - see §2 above.
- No new fault detection (invalid pressure, over-temperature, time-outs,
  watchdog) - these remain the same open gaps recorded in `BASELINE.md`.
- `Kp`/`Ki`/`Kd`/`offset`/`steamSetpoint`/`PIDonly` were not folded into the
  settings-lifecycle latch, per the plan's own wording (§4/§5 above).
- `GetPressure()`'s `!acDetected` gate was left reading `acDetected`
  directly rather than `currentShotState != ShotState::IDLE` - `acDetected`
  remains the actual internal driver; `ShotState` mirrors it for
  observability. Fully unifying internal control-flow onto `ShotState`
  alone is Phase 3's "single function owns all control decisions."

## A build-system finding, not a firmware bug

Adding the scoped enums / struct-with-reference-parameter code broke
Arduino's automatic function-prototype generation (ctags-based) for the
**entire rest of the file** - `basePumpPowerForSetpoint` and
`getContentType`, both defined after their first use and previously
auto-prototyped successfully in Phase 0/1, started failing to compile with
"not declared in this scope." Fixed by adding explicit forward
declarations for both near the top of the file, rather than depending on
Arduino's automatic prototyping (which this codebase now exceeds the
reliable feature set of). Anyone adding new out-of-order function calls in
later phases should add an explicit prototype rather than assume
auto-prototyping will handle it.

## Build verification

Compiled with the same toolchain as Phase 0/1 (`esp32:esp32@2.0.17`):

```
Sketch uses 905149 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 51260 bytes (15%) of dynamic memory, leaving 276420 bytes for local variables. Maximum is 327680 bytes.
```

No compiler errors or warnings. +1516 bytes flash / +48 bytes RAM versus
the merged Phase 0+1 state (`v2.0.0-alpha`: 903,633 bytes flash, 51,212
bytes RAM).

## Behaviour-preservation checklist

- [x] Pump-control branches (pre-infusion/bloom/extraction conditions and
      bodies) are unchanged except for reading `shotSettings.*` instead of
      the old live globals - same conditions, same actuator calls, same
      order.
- [x] AC-off debounce and buzzer sequencer (Phase 1) untouched.
- [x] `runPID()`'s SSR safety behaviour unchanged; only a `currentFault`
      assignment was added alongside the existing check.
- [ ] **Intentional behaviour change:** mid-shot settings edits no longer
      take effect until the shot returns to `IDLE` (§5 above) - this is
      what Phase 2 was scoped to introduce.
- [ ] **Intentional behaviour change:** `Pause`/`Resume` no longer do
      anything (§3 above) - explicitly required by the plan.
- [ ] Bench re-test still required, same hardware limitation as Phase 0/1
      (`BASELINE.md`). This phase adds new items to verify: mid-shot
      setting edits genuinely don't affect the running shot and correctly
      apply to the next one; `/getValues`' new fields read sensibly;
      `Pause`/`Resume` requests from an old client no longer do anything
      (harmless no-op, not an error).
