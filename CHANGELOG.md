# Changelog

## [Unreleased]


### Changed

- Moved first-order thermal physics and thermistor conversion into canonical lib-smoothie models with server API forwarding; ordinary V2 simulator builds now target C++17. Native product builds, isolated fleet/service fixtures and V2/V1 simulator regressions passed.

- V2 analog, PWM and SPI adapters now use canonical lib-smoothie source; native firmware, ten-board isolated feedback/service checks and V2/V1 simulator regressions verified the shared consumers.

- V2 firmware instrumentation now forwards to canonical lib-smoothie tooling; missing canonical staging tools fail before build-key generation.
- V2 deterministic timers, scheduler and HAL timer entry points now forward to canonical lib-smoothie source; shared C++ changes invalidate build fingerprints.
- V2 firmware-copy staging forwards to canonical lib-smoothie tooling; source keys cover shared tools.
- V2 pin and callback-context adapters now include canonical sibling lib-smoothie source; source keys cover shared headers.

### Fixed

- Reject unusable synthetic probe geometry before motion and prevent successful results after runtime geometry failure.
- LPC1768 static RAM reporting excludes the dynamic heap reservation and the overlapping linker-only stack dummy.
- V2 trace comparisons require initial positions on both sides and compare equivalent timer-period representations numerically.
- Step references retain and compare V2 initial motor positions and timer periods.
- STM32 firmware size analysis accounts for SRAM3 and SRAM4 and rejects allocated sections outside the supported memory map.
- Undriven SmoothieV2 inputs now honor the firmware's default pull-up and configured pull-down, while external pin levels take precedence.
- SmoothieV2 pins constructed with an input or output type apply the requested direction, including when a pin is reused.
- SmoothieV2 interrupt pins switch to input mode so simulated input providers remain visible.
- SmoothieV2 builds honor the selected development board and use its motor-driver SPI defaults.
- SmoothieV2 simulated GPIO interrupt callbacks use interrupt context for mutex operations.
- A replacement SmoothieV2 player task cannot release a mutex owned by an earlier player task.
- SmoothieV2 playback dispatches supported console commands from queued files.
- SmoothieV2 mutexes reject release by a different simulated execution context.
- SmoothieV2 runs console commands invoked by firmware modules and reports unhandled nested commands.
- SmoothieV2 reports firmware errors from queued and nested module commands even when their output stream is silent.
- SmoothieV2 motion checkers account for nonzero initial actuator positions used by nonlinear kinematics.
- Nonlinear SmoothieV2 motion checks compare emitted pulses with changes in actuator counters when starting positions are available.
- SmoothieV2 module workloads fail when the firmware reports a command error, including in additional workloads.
- The step checker reports firmware command errors instead of accepting a rejected motion as an empty successful plan.
- SmoothieV2 memory estimates retain firmware command-handler allocations when compiler inlining hides their firmware stack frames.
- SmoothieV2 staged Z-probe firmware rejects invalid leveling and calibration strategies and checks for missing strategies instead of dereferencing null pointers.
- Document the SmoothieV2 square-and-arc checker case and the planner failure observed in host simulation.
- The planner checker rejects deceleration points beyond the block's total duration.
- SmoothieV2 memory estimates exclude simulator-created timer instrumentation wrappers from the firmware heap.
- SmoothieV2 software-timer callbacks are serialized, preserving panel updates when timer-driven firmware operations overlap.
- SmoothieV2 timer events are rechecked after a step interrupt advances the scheduler, preventing a previously dispatched event from firing twice.
- The SmoothieV2 static wait scan recognizes registered command-handler paths when labeling calls in the same source file.
- SmoothieV2 module checks retain observed failures while warning that handler instrumentation is incomplete.
- SmoothieV2 file reset waits for the aborted player task to exit before restarting playback, allowing `M26` to work with the cooperative player.
- SmoothieV2 accepts disabled digital-readout configuration sections while still rejecting enabled readouts that the simulator cannot model.

### Added

- Connected shared synthetic thermal status and fault commands, with optional sensor lag and independent heater/sensor faults. Five thermal CLI tests, 127 core tests (three skips), nine mechanical tests and seven preserved V1 integration tests passed; native shared-source fault/alarm and ten-board feedback checks also passed on isolated Ethernet.


- Separate machine-model files support shared heater-to-thermistor feedback alone or alongside mechanics; three real firmware heater/mapping/motion tests and existing V2 regressions passed.

- Optional V2 `--machine-model` configuration from canonical lib-smoothie, with physical pulse-driven travel, endstops, plane probes, status and fault commands; six CLI tests plus existing V2 regressions passed.

- Canonical emitted-step observer connection carrying actual actuator, physical direction, enable and calibration state; default report behavior is preserved and mechanical runtime configuration remains pending.

- Shared Pin adapter support for removable input providers alongside the existing display provider; mechanical model integration remains pending.
- SmoothieV2 results and checker reports record inclusive simulated elapsed time for observed module broadcasts.
- SmoothieV2 results and checker reports record inclusive simulated elapsed time for recognized periodic timer callbacks.
- SmoothieV2 results and checker reports summarize completed player task delays and their elapsed simulated time separately from handler waits.
- SmoothieV2 workloads can inject repeated rising-edge interrupt pulses for devices such as filament encoders.
- SmoothieV2 accepts an initial thermistor ADC reading through the existing simulator command-line option.
- SmoothieV2 interrupt exploration can inject a nearly due step using a configurable time window.
- SmoothieV2 interrupt exploration models shared conveyor flag writes and counts interrupted writes that overwrite another flag.
- SmoothieV2 checker runs can use a separate uninstrumented simulator for memory estimates while collecting coverage with the main simulator.
- The SmoothieV2 command thread and cooperative player wait for a held mutex up to the requested timeout.
- The report shows partial SmoothieV2 handler elapsed simulated time with its measurement limits.
- SmoothieV2 workloads can drive simulated external button-box inputs by name.
- Document SmoothieV2 coupled-axis, delta and SCARA motion checks and the unequal-arm SCARA discrepancy observed in host simulation.
- The report displays recognized SmoothieV2 command-handler and periodic callback line coverage with explicit partial-coverage labels.
- An optional SmoothieV2 coverage build supplies compiler line and call counts for recognized command-handler and periodic timer methods to the module checker, with isolated counters for each checker run.
- SmoothieV2 interrupt exploration counts planner maximum-rate writes that occur after the simulated step interrupt starts the block.
- SmoothieV2 runs the firmware thermistor-list console command and instruments recognized static handlers defined in another source file.
- SmoothieV2 records inclusive simulated elapsed time for recognized command-handler methods and merges these observations across checker workloads.
- SmoothieV2 results count entries to methods registered as command handlers in recognized direct binding forms; this is partial coverage instrumentation.
- SmoothieV2 runs the ST7920 graphical-display driver with a serial graphics-memory model and simulator-only pixel assertions.
- SmoothieV2 results count completed module halt and command-context callbacks, without claiming full handler coverage or timing.
- SmoothieV2 results count command-handler invocations by G-code, M-code or console command; the module checker preserves and merges those partial counts.
- SmoothieV2 results and module-checker JSON count named periodic callbacks for recognized ticker registrations.
- SmoothieV2 command-handler counts include the callback's declaring class when recognizable, with other callback types marked unattributed.
- SmoothieV2 runs the electronic-leadscrew firmware and TM1638 panel driver with simulated buttons and display register assertions.
- SmoothieV2 runs lathe spindle-speed and synchronized-motion firmware with simulated index pulses, a 16-bit encoder counter and scheduled stop requests for manual mode.
- SmoothieV2 manual pulse generators use the firmware's quadrature decoder and support per-axis or shared handwheels with `M922` axis selection.
- SmoothieV2 runs the firmware digital readout and MAX7219 driver with individual or cascaded displays, decoding their serial pin transitions and supporting workload assertions on display registers.
- Experimental SmoothieV2 simulator: stages firmware source and runs its G-code parser, motion core, extruder, endstops, Z probe, thermistor and MAX31855 temperature control, switches, temperature-triggered switches, kill button and laser module against simulated timers and pins. Hardware PWM models channel allocation, frequency and duty cycle. Other modules and full checker parity are not yet implemented.
- SmoothieV2 workloads can set a MAX31855 temperature or simulate an open circuit or short using `;@sim thermocouple <chip-select-pin> <degrees-C|open|short-gnd|short-vcc>`.
- SmoothieV2 can run a filament switch, bulge input or encoder through the firmware's suspend handler and resume afterward. The staged firmware initializes a pointer that is otherwise uninitialized on the bulge path.
- SmoothieV2 can load the firmware's suspend and resume commands with a configured player even when filament detection is disabled.
- SmoothieV2 can run the firmware file player cooperatively through its command queue, with `play`, pause, resume, progress and abort controls. A configured SD-card directory supplies startup files and `M23`/`M32` selections.
- SmoothieV2 can inject step interrupts around the planner lock and report whether the target block started or protected updates occurred while it was ticking.
- SmoothieV2 workloads can schedule a physical input change with `;@sim pin-after <milliseconds> <pin> <level>`, including while firmware runs a homing move.
- SmoothieV2 allocation traces identify the STM32 heap, DTCM and SRAM1 pools. A linker-map parser and memory checker replay those pools separately when an STM32H745 firmware map and ELF are supplied; without them memory use is not measured.
- The firmware size checker can compare STM32H745 ELF files, separating flash, ITCM, DTCM, AXI SRAM, SRAM1 and SRAM2 and accounting for RAM sections' flash load images.
- The end-of-job position checker reads SmoothieV2 kinematics and actuator steps-per-millimetre from its sectioned configuration.
- The temperature fault checker recognizes SmoothieV2 temperature-control configuration.
- SmoothieV2 fault checks recognize section headers with trailing text accepted by the firmware.
- The module checker reports SmoothieV2 handler timing and coverage as not measured until the simulator supplies instrumentation.
- The SmoothieV2 static module scan flags `safe_sleep` and FreeRTOS task delays for context-sensitive review.
- SmoothieV2 runs explicitly configured TMC2590 and TMC2660 firmware drivers and motor-current commands against a simulated SPI register model, with optional register-write tracing. Compatible Trinamic motors can be slaved to a primary axis; an optional motor trace records their physical pulse positions separately.
- SmoothieV2 task delays advance simulated time and service timers; the simulated task tick count no longer wraps at the microsecond counter's rollover.
- SmoothieV2 can run the firmware's drilling cycles, including drilling and pecking moves with retracts.
- SmoothieV2 records `safe_sleep` calls observed during G-code dispatch with their command context; complete handler timing and coverage remain unmeasured.
- SmoothieV2 button-box inputs can queue G-code, M-code and SD-card file-selection actions through a bounded command queue; external button providers remain unsupported.
- SmoothieV2 workloads can assert an allocated digital pin's physical level with `;@sim expect-pin <pin> <level>`.
- SmoothieV2 workloads can assert the duty fraction of an allocated hardware PWM output with `;@sim expect-pwm <channel> <fraction>`.
- SmoothieV2 module runs reject a temperature configuration if any enabled controller fails to load.
- SmoothieV2 thermistor inputs now require distinct ADC1 channel names accepted by the firmware.
- SmoothieV2 workloads can set an individual thermistor input with `;@sim adc ADC1_0 <value>`.
- `run_checks_v2.sh` runs the available SmoothieV2 checks and generates a report, identifying unmeasured handler instrumentation.
- The SmoothieV2 memory checker loads configured modules before recording allocations.
- Event-ordered timer dispatch in the simulator, now the default: the step and slow tickers run in time order with ties broken by NVIC priority, `--fixed-tick` restores the old order. Results were identical on all workload cases and the 8 replayed PRs and the run time did not change. `tools/compare_dispatch.py` compares both modes; checkers pass extra simulator options through `SMOOTHIE_SIM_ARGS`.
- Fault injection checker `checkers/fault_check.py` with `cases/faults/`: an over-temperature reading while a heater target is set must halt the machine through the real TemperatureControl and refuse later moves; a control run must not halt. Shown in the report as "Fault injection".
- End-of-job position check in `gcode_to_steps` (`checkers/position_check.py`): the position integrated from the step and direction pin levels at every pulse must equal the motor step counter, the Robot's machine position and the G-code destination x steps/mm (catches lost steps, a wrong steps/mm and an inverted direction). The simulator result gains `pulse_position` and `machine_position`. Programs the parser cannot follow (`G20`, `G92`, `G28`, non-cartesian arms) are reported as not checked.
- RAM model (`checkers/memory_check.py`, `tools/lpc_memory_layout.py`): the simulator can log every allocation (`--alloc-trace`), and the checker replays it on the LPC1768 main SRAM heap and AHB0 pool built from a real ARM build's linker map and DWARF class sizes. The report gets a section with peak live bytes, fragmentation, largest free block, failed allocations and the room below the stack; without an ARM build it says "not measured". `overlay/libs/MemoryPool.h` marks pool allocations as AHB0. The idea and the heap model are adapted from the Carvera Firmware Simulator by Konstantin Tcepliaev (GPL version 3 or later); see [Acknowledgements](documentation/acknowledgements.md).
- README: one-line dependency install commands for nine Linux distributions, and the easily missed assumptions (GCC with a matching `gcov`, bash, GNU userland, network at first build, `patch` and `gh` for the PR replay).

## [0.1.0] - 2026-10-20

### Fixed
- The simulator no longer crashes on any console message of 64 characters or more (the module workload died on `M115`): upstream `StreamOutput::printf` reuses a consumed `va_list`, which is harmless on ARM but undefined on x86-64. A stage patch now gives the second `vsnprintf` its own copy.

### Added (releases and packages)
- Every `v*` tag publishes a GitHub release through `.github/workflows/release.yml`: `smoothie-sim-VERSION.tar.gz`, a `.deb`, an `.rpm`, `install.sh` and `SHA256SUMS`, plus a multi-architecture container image `ghcr.io/smoothieware/smoothie-sim` (amd64 and arm64). Pushes to `main` and pull requests run the same packaging and a full build-and-check smoke test without publishing.
- The new `smoothie-sim` command (`packaging/smoothie-sim`) wraps the installed tree: `smoothie-sim build` builds `./build/smoothie-sim` from a cached shallow clone of Smoothieware `edge` (or `SMOOTHIE_SRC`), `smoothie-sim check` runs the checkers and writes `./out/report.html`.
- `VERSION` file, `Dockerfile`, and `packaging/` scripts (`stage.sh` is the single install layout, `build-packages.sh`, `install.sh`, `release-notes.sh`). See [Installation](documentation/installation.md).
- Linux only. macOS is not supported: the firmware core is built with GNU g++ flags and gcov, which have not been tried with Apple's compiler.

### Added (module coverage stimuli)
- Workload stimulus lines: a `;@sim` comment in a workload G-code file is read by the simulator instead of the firmware. `pin <port.pin> <0|1>` sets the level of an input pin (panel buttons, SD card detect, bulge sensor), `pulses <port.pin> <count>` delivers rising edges to an interrupt pin (the `InterruptIn` shim now keeps the firmware's handler), `adc <raw>` changes the thermistor reading, `wait <ms>` lets simulated time pass with the main loop and idle running (second ticks, button polling).
- The modules case now enters the handlers the report listed as never run or partly run: `FilamentDetector::on_pin_rise` and the M404 to M407 and M601 paths, jam and bulge alarms; `TemperatureSwitch::on_gcode_received` (new armed switch with `arm_mcode`) and its heat-up branch; `MotorDriverControl` second tick and the M906 to M911 sub-codes; `Panel` select, back, up, down, second tick (external SD) and the public-data message. With this, all 24 event-registering module files are fully exercised.
- `modules_check.py --extra-run CONFIG WORKLOAD` (repeatable) runs another workload with its own config and merges its counters; used for a second panel type (`cases/modules/panel-config.txt`), because the up and down buttons only exist on the plain ST7565 panel and the back button only on the Reprap Discount one.
- `rules.json`: the M600 suspend that the filament detector's jam response starts (`Player::ON_MAIN_LOOP`) is an allowed long handler (it drains the queue with `wait_for_idle`).

### Added (logo)

- smoothie-sim logo (a chip with a pulse line, navy and orange) in `assets/`, at the top of the README, as the report's page icon and in its top bar.

### Changed (report design)
- The HTML report is redesigned as a single self-contained file: hero header with stat cards, sticky navigation with scroll-spy, light/dark theme toggle, Tabler icons (MIT, inlined, no network requests), restyled charts, sortable and filterable tables, copy buttons, and layouts that stack on phone width. Code: `checkers/report.py`, `checkers/report_ui.py`, `checkers/report_coverage.py`.
- Module coverage keeps its table, and gains a summary strip, filters and a "More info" drawer per module: handler functions entered or not with call counts, what each state means, copyable snippets to exercise a module, and links to the source.
- File names and `file:line` locations link to the file on GitHub. `run_checks.sh` pins the links to the firmware tree's git origin and commit; `SOURCE_URL` (a template with `{path}`) or `report.py --source-url` overrides it.

### Added (step traces)
- `checkers/step_trace.py`: `record` turns a simulator result into a compact text step trace (one line per planner block); `compare` checks a trace against a reference, exact for planned steps, emitted step counts, direction bits, direction changes and final positions, with a tolerance for block start, duration and entry/exit speed. Used by smoothie-gate to hold reference traces.

### Changed (reference comparison)
- With a reference tree, the report shows only findings new to the change. Blocking calls in the source, runtime waits and long handlers that the reference firmware already has are kept in collapsed "not caused by this change" sections. Runtime findings need `BASE_SIM`, a simulator built from the reference tree (new `--base-sim` option of `modules_check.py`, `BASE_SIM` variable of `run_checks.sh`).
- The live example report now compares the `pressure-advance` branch with upstream `edge`.

### Changed (defaults)
- With no `SMOOTHIE_SRC`, `make` clones the `edge` branch of the upstream Smoothieware repository into `fw/` (shallow) and builds from it; `SMOOTHIE_REPO` and `SMOOTHIE_BRANCH` override the source. `run_checks.sh` arguments are now optional and default to that build. The README quick start is now `make` then `./run_checks.sh`.
- The modules checker's "no base tree given" message now explains itself.

### Added (publication)
- Live example report at https://smoothie-sim-example.pages.dev/report.html (Smoothieware `edge` branch), linked from the README; `CLAUDE.md` says to keep it updated.
- `LICENSE` (GPL-2.0-or-later; `overlay/` keeps Smoothieware's GPL-3.0-or-later with its own `COPYING`), `CONTRIBUTING.md`, `.gitignore`, `documentation/` (published docs) and an example report image.
- The simulator builds against upstream Smoothieware `edge` again: the per-motor `Block` API (`is_active`/`axis`/`set_active`) is used only when the firmware's `Block.h` has it (`SIM_SPARSE_BLOCKS`, detected by the Makefile).

### Changed (publication)
- `stage_patch.py`, `stage_patch_race.py` and `replay_pr.sh` moved to `tools/`.
- `regress_prs.py` takes `--firmware CHECKOUT`. `replay_pr.sh` uses `JOBS` (default: all CPUs); `make_pr_trees.sh` takes the firmware checkout as a required argument.
- `evidence.json` key `jim` renamed `maintainer_comments`; wording about the maintainer made neutral.
- The GitHub Actions workflow moved out to the smoothie-gate project, which now owns CI and pull-request verification; smoothie-sim stays a self-contained simulator.
- README rewritten as a short front page; details moved to `documentation/`.
- README: jump-link labels match their headings, `Limits` link added, first-run commands explain the `-` argument and the build/output folders, PR-check example says which tree each argument is.

### Changed
- wait_for_idle is now an allowed call (rules.json allowed_calls): never reported statically or at runtime.
- Static scan also covers every file a PR changes (not only src/modules), so waits in src/libs and mbed/ are found.
- Module checker now builds and runs Laser, Spindle, MotorDriverControl and Panel; coverage is 24 of 24 modules.
### Added
- Race mode (RACE=1): step ISR preempts firmware inside Block flag read-modify-writes; options --race-window-us, --race-force-every; stage_patch_race.py, shim/sim_race.h, cases/race/.
- Simulator GPIO ports now remember written pin levels (stage patch on `libs/Pin.h`), so `StepperMotor::is_enabled()` changes and the PR 1227 enable delay is caught at runtime.
- `modules_check.py --static-only` (scan without a simulator, used for old PRs that no longer compile).
- `run_checks.sh` env vars `REPLAY` (add the old-PR replay table to the report) and `SIZE_TOOL_PREFIX` (empty uses host binutils, which read ARM ELF files).
- Report: static hits show function, context and applied rule; "partly exercised" coverage state.
- rules.json is a rules table: every call has a status (forbidden, allowed, ask-maintainer) and a reason; the report names the rule that applied. Short `wait_us` (2 us or less) and `safe_delay_*` are ask-maintainer.
- Static scan reports the function each wait is in and whether it is an event handler / on_idle, startup, or helper; severity follows status and context.
- Handler-duration check now fails (was a warning) unless the handler is on `long_handler_allow` in rules.json, each entry with a written justification.
- Module coverage uses gcov: a module counts as exercised only if every `on_*` handler function was entered; longer workload (`cases/modules/workload.gcode`).
- `EXTRA_FILES` in the Makefile stages files a PR changes outside `src/` (for example mbed/src/cpp/SWSPI.cpp); `replay_pr.sh` passes them.
- PR replay compatibility patch for PR 633 (`cases/pr-replay/633/compat/`) and a synthetic blocking-ADC case standing in for PR 564.
- `tools/regress_prs.py`: regression test replaying the old PRs; `cases/pr-replay/expected.json` records which are runtime-capable. The HTML report has an "Old PR replay" section (`report.py --replay`).
- safe_delay_us is recorded at runtime (stage patch), so safe_delay_ms shows up as ask-maintainer.
- Filament advance (pressure advance): `checkers/advance_check.py` with `cases/filament-advance/`. Runs the real planner and step ticker on a mixed print and a 100 mm move at 500/1000/3000 mm/s^2, and checks net extruder steps, reversals, durations, lead = K x cruise rate and end position. The simulator now also records per-motor direction changes and the advance fields (`advance_lead`, `flip_tick`, `decel_start_adjust`, `advance_motors`) of each block.
- PR replay: cases/pr-replay/ (evidence.json with the maintainer's quotes, per-PR config+workload) and replay_pr.sh, which builds a simulator from an old PR's tree and runs the module checker.
- x86 simulator of the Smoothieware core and modules, built from a copy of the firmware.
- Checkers: G-code to blocks, blocks to steps (and G-code to steps), modules (waits, interference), RAM and flash comparison.
- HTML report, GitHub job summary, optional PR comment, and a manual-dispatch GitHub Actions workflow.

### Fixed
- `size_check.py` ignores linker address markers (e.g. `__end_bss_RAM5`) in the top-symbol table.
- `report.py` no longer crashes on partly exercised modules.
