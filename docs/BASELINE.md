# Phase 0 baseline record

Date: 2026-08-19
Scope: Phase 0 — Establish a safe baseline (see [REWRITE_PLAN.md](REWRITE_PLAN.md))

This records the state of the imported firmware before any behavioural or
structural change. Everything here was produced by reading the firmware
source and its git history, plus a toolchain build check. **No physical
Discreet hardware was available in this environment**, so the bench items
(safe-boot measurement, fault-injection tests, temp-only/normal-shot test
record) could not be executed and are recorded as open items, not as
completed baseline data. They must be completed on a bench before any
Phase 1+ firmware is installed on a real machine.

## Imported upstream commit

- Upstream repository: [Discreet-Coffee/Discreet](https://github.com/Discreet-Coffee/Discreet)
- Imported at commit [`b6ffbae0d992e453d375a2f793ccef7c7783ccde`](https://github.com/Discreet-Coffee/Discreet/commit/b6ffbae0d992e453d375a2f793ccef7c7783ccde)
  — "13 Bar Max enabled to Clear Discharge Tube", authored 2026-06-25T05:00:19Z.
- This is one commit past upstream tag `V1.7.1` (which points at the parent
  commit `8c2d8b4`). No newer upstream tag exists as of this writing.
- `Discreet.ino` in this repository is byte-identical to upstream `b6ffbae`
  (`git diff b6ffbae HEAD -- Discreet.ino` is empty). All commits in this
  repository since import (`c189bd0` onward) are documentation-only; no
  firmware behaviour changed prior to the Phase 1 work in this same PR.
- Baseline tag: `baseline-phase0`, created at `aa69af4` (last commit before
  Phase 1 code changes; firmware content identical to the imported `b6ffbae`).

## Build verification

Toolchain used to verify the unmodified baseline compiles:

- `arduino-cli` 1.5.1 (Homebrew)
- Board core: `esp32:esp32` (Espressif's `arduino-esp32`), **version 2.0.17**.
- FQBN used: `esp32:esp32:esp32` (generic "ESP32 Dev Module"). The repository
  does not pin a board variant anywhere (no `platformio.ini`, no
  board-specific `#ifdef`s); **the exact board module used on the reference
  hardware has not been confirmed** and should be recorded here once known.
- **The core version matters and is not a free choice**: this firmware, as
  imported, does not compile against the current `esp32:esp32` core
  (3.3.11). `dimmable_light` 1.6.0 (the newest version resolvable through
  the Arduino Library Manager) calls the pre-3.0 timer API
  (`timerBegin(id, div, edge)`, `timerAlarmWrite`, `timerAlarmEnable`),
  which arduino-esp32 3.x replaced with a different signature
  (`timerBegin(frequency)`, no separate alarm-write/enable calls). Compiling
  against 3.3.11 fails with "too many arguments to function 'timerBegin'"
  and related errors in `dimmable_light`'s `hw_timer_esp32.cpp`. This is a
  **pre-existing baseline finding**, not something introduced by this PR —
  the unmodified `Discreet.ino` at `baseline-phase0` has the same result.
  `esp32:esp32@2.0.17` (the newest 2.0.x release) was used instead and
  builds cleanly. **Open item:** decide whether to keep the project pinned
  to an `esp32:esp32` 2.0.x core, or to update `dimmable_light` usage for
  3.x compatibility, before doing any further toolchain upgrades.
- Libraries: the repository has no dependency manifest or lockfile. The
  following were resolved via the Arduino Library Manager to match each
  `#include`; this is this session's best-effort reconstruction, not a
  pinned/verified-on-hardware set:

  | `#include` | Library installed | Version |
  |---|---|---|
  | `dimmable_light.h` | Dimmable Light for Arduino (fabianoriccardi) | 1.6.0 |
  | `max6675.h` | MAX6675 library (Adafruit) | 1.1.2 |
  | `PID_v1.h` | PID (Brett Beauregard) | 1.2.0 |
  | `ArduinoJson.h` | ArduinoJson (Benoit Blanchon) | 7.4.3 |
  | `WiFi.h`, `WebServer.h`, `SD.h`, `SPI.h`, `Wire.h`, `ArduinoOTA.h`, `ESPmDNS.h` | bundled with `esp32:esp32` core | 2.0.17 |

  Note: `#include "max6675.h"` (lowercase) only resolves against the
  Adafruit-style library, not the unrelated "MAX6675" library by Rob
  Tillaart, which installs as `MAX6675.h` (uppercase) and only matches on
  case-insensitive filesystems. The Adafruit library's class name
  (`MAX6675(CLK, CS, DO)`) also matches the constructor call in
  `Discreet.ino:45`.

  **Action for maintainers:** if the reference build historically used
  different library versions, record the actual versions used on hardware
  here instead of the reconstructed set above.

- Result: unmodified `Discreet.ino` (tag `baseline-phase0`) compiles cleanly
  against `esp32:esp32@2.0.17` with the library set above:

  ```
  Sketch uses 902957 bytes (68%) of program storage space. Maximum is 1310720 bytes.
  Global variables use 51188 bytes (15%) of dynamic memory, leaving 276492 bytes for local variables. Maximum is 327680 bytes.
  ```

  No compiler errors or warnings from `Discreet.ino` itself (the
  `dimmable_light`/core-version issue above is a dependency compatibility
  finding, not a warning from this project's own code).

## Pin mapping

From `#define`s in `Discreet.ino`:

| Signal | GPIO | Notes |
|---|---|---|
| `SD_CS` | 32 | SD card chip select |
| `SD_MOSI` | 25 | SD card SPI |
| `SD_MISO` | 26 | SD card SPI |
| `SD_SCK` | 33 | SD card SPI |
| `SSR_PIN` | 13 | Heater solid-state relay output |
| `syncPin` | 19 | Dimmer zero-cross sync input; also polled directly in `loop()` for AC/shot detection |
| `thyristorPin` | 18 | Dimmer thyristor (pump power) output |
| `thermoCS` | 22 | MAX6675 thermocouple amplifier, software SPI |
| `thermoCLK` | 23 | MAX6675 thermocouple amplifier, software SPI |
| `thermoDO` | 21 | MAX6675 thermocouple amplifier, software SPI |
| `pressurepin` | 34 | Analog pressure sensor input (ADC1, input-only pin) |
| `BUZZER_PIN` | 4 | Buzzer output |

## Hardware variant notes

- MCU: ESP32 (specific module/variant not recorded in the repository;
  confirm against the physical board before Phase 5 stack-size/watchdog
  tuning, since stack headroom and core count are variant-dependent).
- Thermocouple: MAX6675 K-type amplifier via software SPI.
- SD card: hardware SPI, shared bus is explicitly torn down
  (`SD.end(); SPI.end();`) and restarted around Wi-Fi bring-up
  (`loadSDConfig()` / `startSD()` in `setup()`), per an existing code comment
  that running SD and starting Wi-Fi concurrently breaks the SD card on this
  hardware.
- Dimmer: zero-cross detection + thyristor phase control via the
  `dimmable_light` library, sharing `syncPin` with the loop's own AC-presence
  polling (see Phase 1 notes).
- Pressure: single analog input, linear-scaled `raw * (maxPressure / 4095.0)`
  over the ESP32's 12-bit ADC range; no documented calibration procedure.
- SSR and buzzer: simple digital outputs.

## Safe boot outputs

From source reading only (bench measurement not performed):

- **Heater (`SSR_PIN`)**: `pinMode(SSR_PIN, OUTPUT)` runs in `setup()`, but
  no explicit `digitalWrite(SSR_PIN, LOW)` precedes or follows it anywhere
  in `setup()`. The first write to this pin happens inside `runPID()`,
  reached only once `loop()` starts, which explicitly drives it `LOW` when
  the temperature reading is invalid or applies the PID-derived level once a
  valid reading exists. Between reset and the first `runPID()` call — i.e.
  through all of `setup()` (SD, Wi-Fi connect up to 20 s, OTA/mDNS init, the
  two 2 s startup delays) — the pin's state depends on the ESP32 GPIO
  power-on-reset default rather than an explicit write in this firmware.
  **Open item:** confirm on the bench that `SSR_PIN` reads low throughout
  power-up and `setup()`.
- **Pump (`thyristorPin` via `DimmableLight`)**: `light.setBrightness()` is
  never called anywhere in `setup()`; the first call is inside the shot
  logic in `loop()`, gated behind AC detection. Whether the dimmer library
  itself defaults to zero output before any `setBrightness()` call, and
  what the thyristor line does before `DimmableLight::begin()` runs, was
  not verified here. **Open item:** confirm on the bench that the pump does
  not fire before initialization completes.
- Both actuators are exclusively driven from the `acDetected` shot branch or
  `runPID()`; no other code path writes to `SSR_PIN` or calls
  `light.setBrightness()`.

## Fault behaviour

From source reading only:

- **Invalid temperature**: `runPID()` treats `isnan(input)`, `input < 0`, or
  `input > 160` as invalid, forces `SSR_PIN` `LOW`, and returns before
  `myPID.Compute()`. Pump behaviour is unaffected by this check.
- **Invalid pressure**: `analogRead()` cannot return NaN, and there is no
  range/validity check on the raw pressure reading before it drives pump
  power in `SetPump()` / `basePumpPowerForSetpoint()`. **This is a gap in
  the current baseline**, not something Phase 0 changes; it is in scope for
  later phases per `REWRITE_PLAN.md` ("sensor validation ... over-temperature
  protection").
- **Over-temperature**: no cutoff independent of the active setpoint exists
  beyond the 160 °C sensor-sanity bound in `runPID()`, which is a
  sensor-fault check rather than an OTP tied to the current target
  temperature. Regulation above setpoint relies entirely on the PID's own
  output limits (`0–255`, mapped to SSR on/off at `output >= 127`). **Gap**,
  carried forward.
- **Time-outs**: Wi-Fi connect has a 20 s timeout in `startWiFi()` (falls
  back to offline operation; not a coffee-safety time-out). Shot-end
  detection previously used a 100-iteration loop counter (`offcount`),
  addressed by this PR's Phase 1 change — see `PHASE1_NOTES.md`. There is no
  independent maximum-shot-duration time-out anywhere in the baseline.
- **Watchdog**: no `esp_task_wdt_*` call exists anywhere in the baseline;
  the `loop()` task is not explicitly subscribed to the ESP Task Watchdog.
  Whether the Arduino core's default idle/loop-task watchdog behaviour
  covers this loop was not verified here — this is exactly the gap
  `REWRITE_PLAN.md` Phase 5 is scoped to close
  (`esp_task_wdt_add(NULL)` from the dedicated control task).

## Bench test record

**Not performed.** No physical Discreet hardware was available in this
environment. Before any Phase 1+ firmware goes on real hardware, a bench
session must record, at minimum:

- Temp-only (`PIDonly = true`) heats and regulates temperature with the
  pump and shot logic staying inactive.
- A normal shot progresses through pre-infusion → (optional bloom) →
  extraction and stops promptly when the brew switch/paddle is released.
- `SSR_PIN` and the pump output stay off from power-up through the end of
  `setup()`.

This is tracked as an open risk (see below), not a completed Phase 0 item.

## Open risks / unverified assumptions carried into later phases

1. No independent over-temperature protection beyond the 160 °C sensor-fault
   bound in `runPID()`.
2. No validity/range check on the raw pressure reading.
3. No maximum-shot-duration time-out independent of AC/paddle detection.
4. No explicit ESP Task Watchdog subscription anywhere in the baseline.
5. Safe-boot output state for `SSR_PIN` and the pump is inferred from
   silicon/library defaults, not verified against the actual hardware.
6. Exact board variant, core version, and library versions used on the
   physical reference hardware are not recorded upstream and have not been
   confirmed against the reconstructed toolchain in this document.
7. `AC_OFF_DEBOUNCE_MS` (introduced in this PR's Phase 1 change, see
   `PHASE1_NOTES.md`) approximates, but does not guarantee, the same
   real-world shot-end timing as the previous loop-count debounce.
8. The project has no dependency manifest, so it silently depends on getting
   a compatible core/library combination by chance. `dimmable_light` 1.6.0
   (latest on the Library Manager) does not build against `esp32:esp32` 3.x;
   this baseline only builds pinned to `esp32:esp32@2.0.17`. Any future
   toolchain upgrade needs to either keep pinning the 2.0.x core or update
   `dimmable_light` usage first.

Items 1–4 are pre-existing gaps in the imported baseline, not introduced by
this work; they are recorded here so later phases address them deliberately
(`REWRITE_PLAN.md` phases 2–5) rather than by accident.
