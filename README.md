# Discreet RTOS Refactor ☕

Private development repository for an experimental two-task FreeRTOS refactor of the Discreet espresso-machine controller.

> This project is derived from **[Discreet Coffee / Discreet](https://github.com/Discreet-Coffee/Discreet)**. All credit for the original project, firmware, hardware concept and web interface belongs to the Discreet Coffee project and its contributors. This repository is an independent development copy and does not imply endorsement by the upstream maintainers.

## Original project

[Discreet](https://github.com/Discreet-Coffee/Discreet) is an open-source ESP32-based espresso-machine controller, designed around the Gaggia Classic Pro and usable with other single-boiler machines. Its features include:

- PID temperature control
- pressure profiling and pump control
- pre-infusion and bloom timing
- a web UI served from SD card
- live telemetry and adjustable settings
- OTA firmware updates
- buzzer feedback

The imported source, tags and revision history have been retained. The original README is also preserved at [docs/UPSTREAM_README.md](docs/UPSTREAM_README.md).

## Goal of this development copy

The aim is to keep the current feature set while separating timing-critical coffee control from networking and file-service work. The design deliberately uses only two execution contexts:

1. Arduino `loop()` handles Wi-Fi, web server, OTA, SD-card management, a non-blocking buzzer, command submission and telemetry display.
2. One dedicated FreeRTOS control task owns shot detection, the shot state machine, pressure/pump control, temperature/PID, safety checks and final actuator outputs.

The proposed control task runs on a 10 ms base cycle. Within that single task:

- 10 ms: shot detection, state transitions and safety checks
- 50 ms: pressure sampling and pump regulation
- 250 ms: temperature sampling and PID
- every cycle: safety overrides before pump and heater outputs

Zero-cross phase control remains interrupt-driven by the dimmer library.

## Scope decisions

- **Temp-only is retained.** The current `PIDonly` behaviour becomes a separate operating mode: temperature regulation stays active while shot detection and pump output remain disabled.
- **Pause/Resume is removed.** It is intentionally not represented as a shot state.
- **Only the control task may write the pump/dimmer or heater/SSR.**
- Settings travel to the control task through a bounded command queue.
- Shot settings are latched at shot start. Valid mid-shot edits are held as the latest pending revision and applied atomically after the shot returns to `IDLE`; safety actions are always immediate.
- The control task publishes a complete latest-value telemetry snapshot through a length-1 queue using `xQueueOverwrite()`; the service side reads it with `xQueuePeek()`.
- The control task must explicitly subscribe itself to the ESP Task WDT with `esp_task_wdt_add(NULL)` and feed it only after a complete successful cycle.
- A time-proportional SSR output is worth evaluating as a separate behaviour change. It must act as a deadman: each short-lived heater authorization expires unless a healthy control cycle re-arms it.

## Roadmap and multi-session workflow

See **[the staged rewrite plan](docs/REWRITE_PLAN.md)** for phase checklists, exit conditions, verification items and the session handoff log.

The intended sequence is:

1. capture and test the unchanged baseline
2. remove blocking and loop-count timing
3. introduce explicit modes, states and value-only messages
4. centralize actuator ownership
5. add command and telemetry queues
6. move the control boundary into one FreeRTOS task
7. validate timing and safety under service-side load
8. consider the SSR improvement separately
9. clean up, document and decide how to feed focused changes upstream

## Safety status

This is experimental firmware for mains-powered heating and pumping hardware. It is not ready for installation merely because it builds. Any change must be bench-tested with safe boot outputs, sensor-fault handling, over-temperature protection, time-outs and watchdog recovery verified before use on a machine.

## Credits and license

Original project: **[Discreet Coffee / Discreet](https://github.com/Discreet-Coffee/Discreet)**  
Original project website/community: [discreetcoffee.co.uk](https://discreetcoffee.co.uk/)  
Original authors and contributors: the Discreet Coffee maintainers and all contributors recorded in the preserved Git history.

This derivative remains licensed under the **[GNU General Public License v3.0](LICENSE)**. The upstream license file is retained unchanged. When redistributing modified versions, preserve copyright and attribution notices, provide corresponding source as required by GPL-3.0, and clearly identify modifications.
