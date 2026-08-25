# Phase 8 — Cleanup and handoff

This phase's checklist (`REWRITE_PLAN.md`) is almost entirely documentation:
remove obsolete globals/duplicate actuator paths, document task boundaries/
queue schemas/timing, document configuration migration, add a hardware test
procedure, update the checklist and README, and decide on an upstream
contribution strategy. The first draft delivered all six items; a 4-agent
independent review found it had gotten two more genuinely obsolete pieces of
state wrong (missed, in the "obsolete globals" pass) and a substantial
number of factual/completeness problems across the two new reference docs
and the upstream recommendation - all fixed below.

## Review round 1

Four parallel Opus review passes: verify the two dead-global removals and
hunt for anything else obsolete; fact-check every concrete claim in the two
new docs (`ARCHITECTURE.md`, `HARDWARE_TEST_PROCEDURE.md`) against the
actual code and cited sources; audit both docs for completeness against
what Phase 7 and earlier phases actually left outstanding; and sanity-check
the upstream PR-splitting recommendation against real git history (`git
apply --check` against the actual `Discreet-Coffee/Discreet` upstream tree,
not just read against this repo's own history).

### Fixed - code

1. **`TelemetrySnapshot::heaterOn` was write-only.** Declared, written every
   cycle in `buildTelemetrySnapshot()`, read by nothing - its one consumer
   (`handleGetValues()`) was switched in Phase 7 to recompute `heaterOn`
   directly from `ssrAuthorizedUntilMs` instead, and the field was never
   removed at the time. Worse than ordinary dead code: the field's own
   declaration comment still described it as the snapshot's heater state,
   so the next person adding a `TelemetrySnapshot` consumer would have
   reached for it and silently reintroduced the exact staleness bug Phase 7
   fixed (a snapshot is only as fresh as `controlTask`'s last successful
   cycle; `ssrAuthorizedUntilMs` is not). Removed; the field's former
   location now has a comment pointing at `ssrAuthorizedUntilMs` instead.

2. **`PIDonly` duplicated `currentMode`, and its one live-session write was
   dead.** `PIDonly` was written in two places (`loadSDConfig()` at boot,
   and `applyControlCommand()`'s `SET_MODE` case at runtime) but read in
   exactly one (`loadSDConfig()`, immediately after its own boot-time
   write) - the runtime write was never read by anything. The comment
   justifying that runtime write claimed "PIDonly is still what gets
   persisted to config.json"; checked directly against the `/saveConfig`
   handler and found false - it persists the client's posted JSON verbatim
   (`serializeJson(doc, file)`) and never reads the `PIDonly` global at
   all. Fixed by removing the global entirely and scoping `PIDonly` to a
   local variable inside `loadSDConfig()`, the only place it's still
   meaningful; the dead runtime write and its incorrect justification were
   removed from `applyControlCommand()`.

Both were missed by the first draft's "obsolete globals" pass, which
checked every *declared* global but didn't separately check whether a
struct *field* (case 1) or a variable with more than one write site
(case 2, where the *existence* of a write can look like use even when
nothing reads it) was actually live. Compiles cleanly after both fixes:
909,097 bytes flash (-68 vs the pre-review commit), 51,324 bytes RAM
(unchanged).

### Fixed - `ARCHITECTURE.md`

Several claims were imprecise or, in two cases, actively contradicted by
the same document a few sections later:

- "Only the esp_timer service task ever calls `digitalWrite(SSR_PIN, ...)`"
  was inherited from `Discreet.ino`'s own comment, but is imprecise: there
  are two call sites, `initSsrDeadman()`'s one-time `LOW` write (from
  `loopTask`/`setup()`, before the deadman or `controlTask` exist) and
  `ssrDeadmanCallback()` thereafter. Corrected to describe both.
- "Exactly three mechanisms cross between contexts. Nothing else does" was
  wrong - the buzzer's mutex-protected state (`buzzerMutex` plus five
  fields plus `BUZZER_PIN` itself) is touched from all three contexts,
  including `controlTask` via `steam()`'s `queueBuzzer()` call at the end
  of every `controlStep()` cycle. This is the one place this firmware takes
  a real FreeRTOS mutex across tasks, and the doc's own ~230ms SSR-decay
  derivation depends on this boundary existing (the shared esp_timer
  dispatch task, bounded buzzer-mutex wait) - so the claim contradicted the
  doc's own later section. Added as a fourth, explicitly-not-a-pattern
  -to-copy mechanism.
- The plain-globals table's "each has exactly one writer" was false for its
  own last row (`loopTask` resets `controlTaskWorstCycleUs` after each
  report - the table already said so in its own "Purpose" column, which
  contradicted the prose above it), and two of the four rows are not
  actually `volatile` as the surrounding prose implied for all of them.
  Table corrected with an explicit `volatile?` column and the shared-reset
  case called out as a deliberate, low-consequence exception rather than a
  contradiction to gloss over.
- "written only from inside `controlStep()`/`applyControlCommand()`" for
  the control-relevant globals doesn't hold during boot - `loadSDConfig()`
  writes all of them directly from `loopTask`, safely, before `controlTask`
  exists. Corrected to describe the real two-phase model (direct boot-time
  writes, then runtime-exclusive via the control owner) instead of a single
  rule that the doc's own "Configuration persistence" section (a hundred
  lines later) already contradicted.
- "only ever *read* from `loopTask` for display" missed that `/saveConfig`
  reads the live `offset` global to *construct* a new command's payload
  (`setpointCmd.value = ... + offset`), not just to display something - a
  narrower instance of the same class of gap `PHASE7_NOTES.md` item 16
  already names for `/getValues`, but not previously written down for this
  site. Added.
- "`config.json` on the SD card is the only persisted configuration this
  firmware has" is false - `handleApplyTheme()`/`getCurrentTheme()`
  persist a selected theme via `/currentTheme.txt` and `/global.css`.
  Corrected to acknowledge it (out of scope to document further here).
- "`/saveConfig`... still read[s] and write[s] exactly these nine fields"
  overstated what the write path actually does - it persists whatever JSON
  the client posts, field-agnostic; the nine-field contract is enforced by
  `loadSDConfig()` (what's read back meaningfully) and by what the shipped
  `config.js` chooses to post, not by `/saveConfig` itself. Corrected.
- The `FAULT_CLEAR_STABLE_READINGS` timing-table row described ~750ms as
  "how long an over-temperature trip stays latched" - inverting Gap 5's own
  finding, which is titled "the over-temperature fault does not latch."
  Corrected to say "self-clears," with an explicit note that this is the
  bug, not a feature.
- Added a "State model" section (the `OperatingMode`/`ShotState`/
  `FaultCode` enums and what drives transitions between them) and expanded
  the settings-latch treatment beyond a passing parenthetical mention -
  both real gaps in a doc whose own stated purpose already covers the
  `TelemetrySnapshot` schema that publishes this state, and whose absence
  risked a new contributor mutating `activeSettings` mid-shot without
  realizing the latch exists.
- "Where to look next" claimed `PHASE7_NOTES.md`'s bench checklist "is the
  most current and complete one," directly contradicted by
  `HARDWARE_TEST_PROCEDURE.md` (written in this same PR) claiming to
  supersede *that*. Fixed to point at `HARDWARE_TEST_PROCEDURE.md` as the
  actual current one.

### Fixed - `HARDWARE_TEST_PROCEDURE.md`

- **Four Phase 6 bench items and two Phase 5 items had been dropped in the
  consolidation**, not just reworded: heater response/PID-tuning validation
  (more load-bearing than when Phase 6 first listed it, now that Phase 7
  found tunings were never actually applied at boot until this phase's own
  fix); the SSR time-proportional constants' appropriateness for the real
  heater/SSR (still marked `TODO(bench)` at the constant's own declaration);
  low/high duty-cycle quantization visibility; recording which core the
  esp_timer service task actually lands on (directly relevant to whether a
  bad SSR-decay measurement is explained by core contention); a pre-load
  -test timing baseline (without it, the load-disturbance steps have
  nothing to compare against); and the combined simultaneous-load case
  (Wi-Fi + web traffic + SD + OTA at once, the case most likely to actually
  threaten a 10ms deadline) with its "an OTA-induced miss is expected, not
  a bug" caveat, which had been dropped along with the item. All added back
  as their own steps.
- **A step read as contradicting a known gap instead of testing it.** The
  Task WDT recovery step said to confirm "both actuators are safe
  throughout the hang" - `PHASE7_NOTES.md` Gap 2 already establishes the
  pump has no independent deadman and is expected to stay frozen at its
  last commanded level for the full ~5s WDT window, by design as things
  stand. Fixed to state the actual expected behavior (heater safe via the
  deadman within ~230ms; pump frozen until reboot) and to ask the operator
  to record how long the pump actually stayed driven - turning it into the
  measurement that confirms or corrects Gap 2's severity assessment,
  instead of a test an honest operator could only fail.
- **The front-end-observability caveat was attached to one step instead of
  being a standing instruction.** `PHASE7_NOTES.md` documents that the
  shipped dashboard can't be trusted to show fault/mode/actuator state or
  telemetry freshness at all - relevant to roughly a third of this
  procedure's steps, not just the telemetry-staleness one it was originally
  attached to. Moved into the preamble as a standing instruction.
- **A step's cited concern didn't match its source, and was stale on top of
  that.** The AC-off debounce tuning step described "a brief pressure dip"
  as a risk to shot-end detection - `PHASE1_NOTES.md`'s actual concern was
  zero-cross artifacts on `syncPin`, unrelated to pressure. It also missed
  Phase 5's real, current version of the concern: at the new fixed 100Hz
  sampling rate, `AC_OFF_MIN_SAMPLES` (20) now consumes 200ms of the 300ms
  `AC_OFF_DEBOUNCE_MS` budget, versus a few milliseconds when `loop()` ran
  much faster. Corrected.
- **Two priority claims didn't match the notes they cited.** Step 1 called
  the pressure-sensor gap "the single highest-priority *code gap*" while
  `PHASE7_NOTES.md` Gap 1 explicitly says the opposite - a bounds check
  there is "provably dead code, not a missing safeguard," i.e. not a code
  gap at all, and Gap 5's own text ranks itself co-equal with Gap 1
  ("alongside," not below). The SSR-deadman-timing step called itself "this
  rewrite's other highest-priority outstanding item," but `PHASE6_NOTES.md`
  ranks it #2 of 5 in its own list and `PHASE7_NOTES.md` Gap 1 ranks it
  third overall, behind both the pressure-sensor and over-temperature
  items. Both corrected to match the actual established ranking.

### Fixed - the upstream-contribution recommendation (`REWRITE_PLAN.md`)

The independent review ran `git apply --check` against the real
`Discreet-Coffee/Discreet` upstream tree for each proposed candidate range,
not just read the diffs against this repo's own history, and found two real
problems:

- **Phase 3 is not standalone**, contrary to the first draft's ordering
  (Phase 3 proposed second, "before" the rest of the architecture). Phase
  3's fault/mode priority resolution is written directly against Phase 2's
  `OperatingMode`/`ShotState`/`FaultCode` enums, which don't exist upstream
  at all (upstream has only a bare `PIDonly` bool) - confirmed by a failed
  `git apply --check` and by reading the actual code dependency. Reordered:
  Phase 2 and Phase 3 are now proposed together, as one PR.
- **The fourth Phase 7 "bug fix" (stale `heaterOn` telemetry) is not
  portable to upstream at all**, contrary to the first draft's claim that
  all of Phase 7's fixes were "genuinely independent of the architecture
  change." That fix reads `ssrAuthorizedUntilMs` (introduced by Phase 6)
  and addresses a staleness mode caused by `telemetryQueue` (introduced by
  Phase 4) - neither exists upstream, confirmed by grep against the real
  upstream source. The other three Phase 7 fixes (stuck-low thermocouple,
  missing `SetTunings()` call, unclamped boot setpoint) were each confirmed
  to apply against upstream's real code as claimed. The recommendation now
  proposes only those three as an upstream-independent group, and folds the
  stale-`heaterOn` fix into the general "not recommended yet" category with
  the other still-open gaps (it isn't a gap, but it also isn't proposable
  on its own upstream, so it travels with the rest of the two-task
  architecture instead).

Also corrected: the section previously said "Phase 7's five bug fixes" while
naming four - the fifth item in `PHASE7_NOTES.md`'s "Review round 1" is a
stale-comment correction, not a bug fix, and shouldn't have been counted
alongside the other four when describing what's proposable upstream.

## Not changed

The Phase 8 checklist's own scope (six items, all documentation/cleanup, no
new safety-relevant code) was not expanded in response to review - none of
the findings above called for new firmware behavior, only for correcting
what was claimed about existing behavior and closing two small, genuinely
dead pieces of state. Nothing in this phase's review surfaced a new gap of
the kind `PHASE7_NOTES.md` documents (a real, un-bench-verified safety
question) - that phase's five gaps are unchanged and still the priority list
`HARDWARE_TEST_PROCEDURE.md` opens with.
