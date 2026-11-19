// Experimental command-line driver for the real SmoothieV2 motion core.
#include "sim_output.h"
#include "sim_runtime.h"
#include "sim_alloc.h"
#include "ConfigReader.h"
#include <smoothie/v2/mechanical_runtime.h>
#include <smoothie/v2/machine_config.h>
#include <smoothie/v2/thermal_runtime.h>
#include <smoothie/v2/thermal_config.h>
#include "Conveyor.h"
#include "Dispatcher.h"
#include "GCodeProcessor.h"
#include "MemoryPool.h"
#include "OutputStream.h"
#define private public
#include "Planner.h"
#undef private
#include "PlannerQueue.h"
#include "Block.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StepTicker.h"
#include "sim_v2_hooks.h"
#include "v2_scheduler.h"
#include "v2_task.h"
#include "v2_player_files.h"
#include "v2_max7219.h"
#include "v2_tm1638.h"
#include "v2_st7920.h"
#include "ST7920.h"
#include "TemperatureControl.h"
#include "TemperatureSwitch.h"
#include "Laser.h"
#include "Extruder.h"
#include "Endstops.h"
#include "ZProbe.h"
#include "KillButton.h"
#include "FilamentDetector.h"
#include "Player.h"
#include "CurrentControl.h"
#include "drillingcycles.h"
#include "buttonbox.h"
#include "max7219.h"
#include "dro.h"
#include "mpg.h"
#include "Lathe.h"
#include "TM1638.h"
#include "els.h"
#include "QuadEncoder.h"
#include "SlowTicker.h"
#include "FastTicker.h"
#include "Switch.h"
#include "Module.h"
#include "Pwm.h"
#include "Adc.h"
#include "Spi.h"
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <functional>
#include <memory>
#include <map>
#include <set>
#include <vector>

extern "C" uint32_t get_microseconds();
extern "C" void sim_v2_timer_advance_to(double);
extern "C" double sim_v2_timer_now_us();
Pin *fets_enable_pin = nullptr;
Pin *fets_power_enable_pin = nullptr;
uint8_t board_id = 0;
float get_voltage_monitor(const char *) { return 24.0F; }
extern "C" void print_to_all_consoles(const char *text) { if (text) SIM.serial_out += text; }
SimState SIM;
MemoryPool dtcm_pool(1), sram1_pool(2);
MemoryPool *_DTCMRAM = &dtcm_pool;
MemoryPool *_SRAM_1 = &sram1_pool;
static std::vector<std::function<void()>> startup_callbacks;
void register_startup(std::function<void()> callback) { startup_callbacks.push_back(std::move(callback)); }
void safe_sleep(uint32_t ms) {
    const double start = sim_v2_timer_now_us();
    sim_v2_timer_advance_to(start + ms * 1000.0);
    sim_v2_record_command_wait("safe_sleep", static_cast<uint64_t>(sim_v2_timer_now_us() - start));
}
static GCodeProcessor command_parser;
static bool stop_pending = false;
static bool nested_command_error = false;
static OutputStream *stop_target = nullptr;
static bool dispatch_with_stop(GCode &command, OutputStream &output)
{
    const bool manual_lathe = command.has_g() && command.get_code() == 33 && !command.has_arg('Z');
    if (manual_lathe && !stop_pending) return false;
    OutputStream *previous = stop_target;
    if (manual_lathe) stop_target = &output;
    const bool handled = Dispatcher::getInstance()->dispatch(command, output);
    stop_target = previous;
    return handled;
}
bool dispatch_line(OutputStream &output, const char *line)
{
    if (!line) return false;
    if (std::islower(static_cast<unsigned char>(line[0])) || line[0] == '$') {
        if (Dispatcher::getInstance()->dispatch(line, output)) return true;
        std::fprintf(stderr, "version 2 nested console command unhandled: %s\n", line);
        nested_command_error = true;
        return false;
    }
    GCodeProcessor::GCodes_t commands;
    if (!command_parser.parse(line, commands)) return false;
    for (auto &command : commands) {
        if ((command.has_g() || command.has_m()) && !dispatch_with_stop(command, output)) {
            std::fprintf(stderr, "version 2 nested G-code unhandled: %s\n", line);
            nested_command_error = true;
            return false;
        }
        if (command.has_error()) {
            const char *error = command.get_error_message();
            std::fprintf(stderr, "version 2 nested command failed: %s\n", error ? error : "unknown firmware error");
            nested_command_error = true;
            return false;
        }
    }
    return true;
}
struct QueuedCommand { std::string line; OutputStream *output; };
static std::deque<QueuedCommand> command_queue;
static bool unsupported_file_command(const GCode &command) {
    return command.has_m() && (command.get_code() == 22 || command.get_code() == 23 || command.get_code() == 32);
}
extern "C" bool send_message_queue(const char *line, OutputStream *output, bool) {
    if (!line || !output || std::strlen(line) >= 132) return false;
    while (command_queue.size() >= 10 && sim_v2_task_running()) sim_v2_task_yield(0);
    if (command_queue.size() >= 10) return false;
    command_queue.push_back({line, output});
    if (sim_v2_task_running()) sim_v2_task_yield(0);
    return true;
}
static bool drain_command_queue() {
    for (int count = 0; !command_queue.empty() && count < 10; ++count) {
        QueuedCommand queued = std::move(command_queue.front());
        command_queue.pop_front();
        const auto first = queued.line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || queued.line[first] == ';') continue;
        std::string text = queued.line.substr(first);
        if (std::islower(static_cast<unsigned char>(text[0])) || text[0] == '$') {
            if (!dispatch_line(*queued.output, text.c_str())) return false;
            continue;
        }
        if (text.compare(0, 4, "M23 ") == 0 || text.compare(0, 4, "M32 ") == 0) {
            if (!sim_v2_sd_root_ready()) return false;
            text[0] = 'm';
            if (!Dispatcher::getInstance()->dispatch(text.c_str(), *queued.output)) return false;
            continue;
        }
        GCodeProcessor::GCodes_t commands;
        if (!command_parser.parse(queued.line.c_str(), commands)) return false;
        bool found = false;
        for (auto &command : commands) {
            if (!command.has_g() && !command.has_m()) continue;
            if (unsupported_file_command(command)) return false;
            found = true;
            if (!dispatch_with_stop(command, *queued.output)) return false;
            if (command.has_error()) {
                const char *error = command.get_error_message();
                std::fprintf(stderr, "version 2 queued command failed: %s\n", error ? error : "unknown firmware error");
                return false;
            }
        }
        if (!found) return false;
    }
    return true;
}
extern "C" void rtc_get_datetime(char *buffer, size_t size) { if (size) buffer[0] = '\0'; }

static bool inject_blocks(std::istream &input, Planner *planner, Conveyor *conveyor)
{
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        int axes;
        if (!(fields >> axes) || axes < 1 || axes > Block::n_actuators) return false;
        uint32_t steps[k_max_actuators]{};
        uint32_t maximum = 0;
        for (int i = 0; i < axes; ++i) {
            long long value;
            if (!(fields >> value) || value < 0 || value > UINT32_MAX) return false;
            steps[i] = static_cast<uint32_t>(value);
            if (steps[i] > maximum) maximum = steps[i];
        }
        unsigned direction;
        float speed, rate, acceleration, distance, entry, exit;
        if (!(fields >> direction >> speed >> rate >> acceleration >> distance >> entry >> exit) ||
            !maximum || !std::isfinite(speed) || !std::isfinite(rate) || !std::isfinite(acceleration) ||
            !std::isfinite(distance) || !std::isfinite(entry) || !std::isfinite(exit) ||
            speed <= 0 || rate <= 0 || acceleration <= 0 || distance <= 0 || entry < 0 || exit < 0)
            return false;
        while (planner->queue->full()) {
            conveyor->check_queue(true);
            safe_sleep(10);
        }
        Block *block = planner->queue->get_head();
        block->clear();
        for (int i = 0; i < axes; ++i) {
            block->steps[i] = steps[i];
            block->direction_bits[i] = (direction >> i) & 1;
        }
        block->steps_event_count = maximum;
        block->nominal_speed = speed;
        block->nominal_rate = rate;
        block->acceleration = acceleration;
        block->millimeters = distance;
        block->entry_speed = entry;
        block->primary_axis = true;
        block->is_g123 = true;
        planner->calculate_trapezoid(block, entry, exit);
        block->ready();
        SIM.current_line = line_number;
        if (!planner->queue->queue_head()) return false;
        sim_v2_block_queued();
        conveyor->check_queue();
    }
    return !input.bad();
}

int main(int argc, char **argv)
{
    std::string machine_model_path, config_path, gcode_path, blocks_path, alloc_trace, spi_trace, motor_trace, sd_root, output_path = "sim-result.json";
    bool full_steps = false, with_modules = false;
    for (int i = 1; i < argc; ++i) {
        std::string option(argv[i]);
        if (option == "--full-steps") full_steps = true;
        else if (option == "--modules") with_modules = true;
        else if (option == "--no-steps") SIM.capture_steps = false;
        else if (option == "--race-force-every" && i + 1 < argc) {
            const char *argument = argv[++i];
            char *end = nullptr;
            const auto interval = std::strtoull(argument, &end, 10);
            if (!*argument || *argument == '-' || *end || !interval) {
                std::fprintf(stderr, "invalid race interrupt interval\n");
                return 2;
            }
            sim_v2_race_configure(interval);
        }
        else if (option == "--race-window-us" && i + 1 < argc) {
            const char *argument = argv[++i];
            char *end = nullptr;
            double window = std::strtod(argument, &end);
            if (!*argument || *end || !std::isfinite(window) || window < 0) {
                std::fprintf(stderr, "invalid race interrupt window\n");
                return 2;
            }
            sim_v2_race_window(window);
        }
        else if (option == "--adc-value" && i + 1 < argc) {
            const char *argument = argv[++i];
            char *end = nullptr;
            long value = std::strtol(argument, &end, 10);
            if (!*argument || *end || value < 0 || value > 4095) {
                std::fprintf(stderr, "invalid initial ADC value\n");
                return 2;
            }
            SIM.adc_value = static_cast<int>(value);
        }
        else if ((option == "--machine-model" || option == "--config" || option == "--gcode" || option == "--blocks-in" || option == "--out" || option == "--alloc-trace" || option == "--spi-trace" || option == "--motor-trace" || option == "--sd-root") && i + 1 < argc) {
            std::string value(argv[++i]);
            if (option == "--machine-model") machine_model_path = value;
            else if (option == "--config") config_path = value;
            else if (option == "--gcode") gcode_path = value;
            else if (option == "--blocks-in") blocks_path = value;
            else if (option == "--alloc-trace") alloc_trace = value;
            else if (option == "--spi-trace") spi_trace = value;
            else if (option == "--motor-trace") motor_trace = value;
            else if (option == "--sd-root") sd_root = value;
            else output_path = value;
        } else {
            std::fprintf(stderr, "unknown or incomplete option: %s\n", option.c_str());
            return 2;
        }
    }
    if (config_path.empty() || (gcode_path.empty() == blocks_path.empty())) {
        std::fprintf(stderr, "usage: smoothie-sim-v2 --config file (--gcode file | --blocks-in file) [--out file] [--full-steps] [--no-steps]\n");
        return 2;
    }
    if (!sd_root.empty() && !sim_v2_set_sd_root(sd_root.c_str())) {
        std::fprintf(stderr, "invalid simulated SD-card root\n");
        return 2;
    }
    std::ifstream config_file(config_path), input_file(gcode_path.empty() ? blocks_path : gcode_path);
    if (!config_file || !input_file) {
        std::fprintf(stderr, "cannot open configuration or motion input file\n");
        return 2;
    }
    if (!alloc_trace.empty()) sim_alloc_start();
    if (!spi_trace.empty()) SPI::enable_trace();
    if (!motor_trace.empty()) sim_v2_motor_trace_enable();
    ConfigReader config(config_file);
    ConfigReader::sub_section_map_t actuators;
    std::vector<std::pair<std::size_t, std::size_t>> requested_slaves;
    bool motor_drivers_enabled = false;
    if (config.get_sub_sections("actuator", actuators)) {
        const std::vector<std::string> names{"alpha", "beta", "gamma", "delta", "epsilon", "zeta"};
        std::set<std::string> slave_masters;
        for (const auto &actuator : actuators) {
            const std::string driver = config.get_string(actuator.second, "driver", "external");
            const std::string slave = config.get_string(actuator.second, "slaved_to", "");
            if (driver != "external" && driver != "tmc2590" && driver != "tmc2660") {
                std::fprintf(stderr, "unsupported version 2 actuator driver or slaving: %s\n", actuator.first.c_str());
                return 2;
            }
            if (!slave.empty() && slave != "none") {
                auto source = std::find(names.begin(), names.end(), actuator.first);
                auto target = std::find(names.begin(), names.begin() + 3, slave);
                auto master = actuators.find(slave);
                if (driver == "external" || source == names.end() || source < names.begin() + 3 ||
                    target == names.begin() + 3 || master == actuators.end() ||
                    config.get_string(master->second, "driver", "external") != driver ||
                    config.get_int(master->second, "microsteps", 32) != config.get_int(actuator.second, "microsteps", 32) ||
                    !slave_masters.insert(slave).second) {
                    std::fprintf(stderr, "invalid version 2 actuator slaving: %s\n", actuator.first.c_str());
                    return 2;
                }
                requested_slaves.emplace_back(target - names.begin(), source - names.begin());
            } else if (slave == "none" && driver == "external") {
                std::fprintf(stderr, "invalid version 2 actuator slaving: %s\n", actuator.first.c_str());
                return 2;
            }
            if (driver == "external") continue;
            ConfigReader::sub_section_map_t tmc;
            if (!config.get_sub_sections(driver.c_str(), tmc) || !tmc.count(actuator.first)) {
                std::fprintf(stderr, "missing version 2 actuator driver configuration: %s\n", actuator.first.c_str());
                return 2;
            }
            const auto common = tmc.find("common");
#ifdef BOARD_DEVEBOX
            constexpr int default_channel = 0;
#else
            constexpr int default_channel = 1;
#endif
            const int channel = common == tmc.end() ? default_channel : config.get_int(common->second, "spi_channel", default_channel);
            const char *fallback = "nc";
#ifdef BOARD_PRIME
            const std::map<std::string, std::string> default_cs{{"alpha", "PJ13"}, {"beta", "PG8"}, {"gamma", "PG7"}, {"delta", "PG6"}};
            const auto default_pin = default_cs.find(actuator.first);
            if (default_pin != default_cs.end()) fallback = default_pin->second.c_str();
#endif
            const std::string pin = config.get_string(tmc.at(actuator.first), "spi_cs_pin", fallback);
            if (!SPI::register_motor(channel, pin)) {
                std::fprintf(stderr, "invalid version 2 actuator driver SPI configuration: %s\n", actuator.first.c_str());
                return 2;
            }
            motor_drivers_enabled = true;
        }
    }
    auto *ticker = StepTicker::getInstance();
    auto *planner = Planner::getInstance();
    auto *conveyor = Conveyor::getInstance();
    auto *robot = Robot::createInstance();
    if (!planner->configure(config) || !conveyor->configure(config) || !robot->configure(config) ||
        !planner->initialize(robot->get_number_registered_motors())) {
        std::fprintf(stderr, "version 2 motion configuration failed\n");
        return 2;
    }
    for (const auto &pair : requested_slaves) {
        if (pair.first >= robot->actuators.size() || !robot->actuators[pair.first]->has_slave() ||
            robot->actuators[pair.first]->get_slave()->get_motor_id() != pair.second) {
            std::fprintf(stderr, "version 2 actuator slave was not configured\n");
            return 2;
        }
    }
    bool temperature_enabled = false, switches_enabled = false, temperature_switches_enabled = false, fast_required = false;
    bool player_enabled = false, button_box_enabled = false, dro_enabled = false, lathe_enabled = false, els_enabled = false;
    std::map<std::string, bool> external_buttons;
    V2Max7219 display_model;
    V2TM1638 panel_model;
    V2ST7920 graphics_model;
    ST7920 *graphics = nullptr;
    bool graphics_initialized = false;
    if (with_modules) {
        for (int i = 0; i < 2; ++i) {
            ConfigReader::section_map_t settings;
            const std::string section = std::string("pwm") + std::to_string(i + 1);
            config.get_section(section.c_str(), settings);
            int frequency = config.get_int(settings, "frequency", 10000);
            if (frequency <= 0 || !Pwm::setup(i, frequency)) {
                std::fprintf(stderr, "invalid version 2 PWM timer frequency\n");
                return 2;
            }
        }
        ConfigReader::sub_section_map_t manual_generators;
        if (config.get_sub_sections("mpg", manual_generators)) {
            size_t enabled_count = 0;
            bool shared = false;
            for (const auto &instance : manual_generators) {
                if (!config.get_bool(instance.second, "enable", false)) continue;
                ++enabled_count;
                shared |= instance.first == "shared";
            }
            if ((shared && enabled_count != 1) ||
                (enabled_count && (!MPG::create(config) || Module::lookup_group("mpg").size() != enabled_count))) {
                std::fprintf(stderr, "version 2 manual pulse generator configuration failed\n");
                return 2;
            }
        }
        ConfigReader::sub_section_map_t buttons;
        if (config.get_sub_sections("button box", buttons)) {
            std::set<int> button_pins;
            const auto common = buttons.find("common");
            if (common != buttons.end() && (config.get_int(common->second, "poll_frequency_hz", 20) <= 0 ||
                                             config.get_int(common->second, "poll_frequency_hz", 20) > 1000)) {
                std::fprintf(stderr, "invalid version 2 button box polling frequency\n");
                return 2;
            }
            for (const auto &button : buttons) {
                if (button.first == "common" || !config.get_bool(button.second, "enable", true)) continue;
                const std::string pin = config.get_string(button.second, "pin", "nc");
                const std::string press = config.get_string(button.second, "press", "");
                const std::string release = config.get_string(button.second, "release", "");
                const int pin_index = pin == "external" ? -1 : Pin::external_pin_index(pin);
                if ((pin != "external" && (pin_index < 0 || Pin::has_external_pin(pin) || !button_pins.insert(pin_index).second)) ||
                    press.empty() || press.size() >= 132 || release.size() >= 132) {
                    std::fprintf(stderr, "invalid or unsupported version 2 button box input: %s\n", button.first.c_str());
                    return 2;
                }
                if (pin == "external") external_buttons.emplace(button.first, false);
                button_box_enabled = true;
            }
            if (button_box_enabled && (!ButtonBox::create(config) || !Module::lookup("buttonbox"))) {
                std::fprintf(stderr, "version 2 button box configuration failed\n");
                return 2;
            }
            for (const auto &button : buttons) {
                if (button.first == "common" || !config.get_bool(button.second, "enable", true)) continue;
                const std::string pin = config.get_string(button.second, "pin", "nc");
                if (pin == "external") {
                    auto *box = static_cast<ButtonBox *>(Module::lookup("buttonbox"));
                    if (!box->set_cb_fnc(button.first.c_str(), [&external_buttons](const char *name) {
                        auto state = external_buttons.find(name);
                        return state != external_buttons.end() && state->second;
                    })) {
                        std::fprintf(stderr, "version 2 external button was not configured: %s\n", button.first.c_str());
                        return 2;
                    }
                } else if (!Pin::has_external_pin(pin)) {
                    std::fprintf(stderr, "version 2 button box input was not configured: %s\n", button.first.c_str());
                    return 2;
                }
            }
        }
        const char *singletons[] = {"network"};
        for (const char *name : singletons) {
            ConfigReader::section_map_t section;
            if (config.get_section(name, section) && config.get_bool(section, "enable", false)) {
                std::fprintf(stderr, "unsupported version 2 module: %s\n", name);
                return 2;
            }
        }
        ConfigReader::section_map_t graphics_config;
        if (config.get_section("st7920", graphics_config) && config.get_bool(graphics_config, "enable", false)) {
            if (!ST7920::create(config) || !(graphics = static_cast<ST7920 *>(Module::lookup("st7920"))) ||
                !graphics_model.configure(config.get_string(graphics_config, "clk", "nc"),
                                          config.get_string(graphics_config, "mosi", "nc"))) {
                std::fprintf(stderr, "version 2 graphical display configuration failed\n");
                return 2;
            }
        }
        ConfigReader::section_map_t lathe_config;
        if (config.get_section("lathe", lathe_config) && config.get_bool(lathe_config, "enable", false)) {
            if (!Lathe::create(config) || !Module::lookup("Lathe")) {
                std::fprintf(stderr, "version 2 lathe configuration failed\n");
                return 2;
            }
            lathe_enabled = true;
        }
        ConfigReader::section_map_t panel_config, els_config;
        const bool panel_enabled = config.get_section("tm1638", panel_config) && config.get_bool(panel_config, "enable", false);
        if (panel_enabled && (!TM1638::create(config) || !Module::lookup("tm1638") ||
                              !panel_model.configure(config.get_string(panel_config, "clock_pin", "nc"),
                                                     config.get_string(panel_config, "data_pin", "nc"),
                                                     config.get_string(panel_config, "strobe_pin", "nc")))) {
            std::fprintf(stderr, "version 2 panel configuration failed\n");
            return 2;
        }
        if (config.get_section("els", els_config) && config.get_bool(els_config, "enable", false)) {
            if (!panel_enabled || !lathe_enabled || !ELS::create(config) || !Module::lookup("ELS")) {
                std::fprintf(stderr, "version 2 electronic leadscrew configuration failed\n");
                return 2;
            }
            els_enabled = true;
        }
        ConfigReader::section_map_t drilling;
        if (config.get_section("drillingcycles", drilling) && config.get_bool(drilling, "enable", false) &&
            (!Drillingcycles::create(config) || !Module::lookup("Drillingcycles"))) {
            std::fprintf(stderr, "version 2 drilling cycle configuration failed\n");
            return 2;
        }
        ConfigReader::sub_section_map_t currents;
        if (config.get_sub_sections("current control", currents)) {
            bool enabled = false;
            for (const auto &current : currents) {
                if (config.get_float(current.second, "current", -1) <= 0) continue;
                enabled = true;
                auto actuator = actuators.find(current.first);
                if (actuator == actuators.end() || std::string(config.get_string(actuator->second, "driver", "external")) == "external") {
                    std::fprintf(stderr, "unsupported version 2 current control without internal actuator driver: %s\n", current.first.c_str());
                    return 2;
                }
            }
            if (enabled && (!CurrentControl::create(config) || !Module::lookup("currentcontrol"))) {
                std::fprintf(stderr, "version 2 current control configuration failed\n");
                return 2;
            }
        }
        ConfigReader::sub_section_map_t controls;
        ConfigReader::section_map_t filament, player;
        const bool filament_enabled = config.get_section("filament detector", filament) && config.get_bool(filament, "enable", false);
        const bool player_configured = config.get_section("player", player);
        if (player_configured || filament_enabled) {
            if (config.get_bool(player, "on_boot_gcode_enable", true) && sd_root.empty()) {
                std::fprintf(stderr, "version 2 startup playback needs --sd-root or disable on_boot_gcode_enable\n");
                return 2;
            }
            if (!Player::create(config) || !Module::lookup("player")) {
                std::fprintf(stderr, "version 2 player configuration failed\n");
                return 2;
            }
            player_enabled = true;
        }
        if (filament_enabled) {
            if (!FilamentDetector::create(config) || !Module::lookup("FilamentDetector")) {
                std::fprintf(stderr, "version 2 filament detector configuration failed\n");
                return 2;
            }
            switches_enabled = true;
        }
        ConfigReader::section_map_t kill_button;
        if (config.get_section("kill button", kill_button) && config.get_bool(kill_button, "enable", false)) {
            if (!KillButton::create(config) || !Module::lookup("killbutton")) {
                std::fprintf(stderr, "version 2 kill button configuration failed\n");
                return 2;
            }
            switches_enabled = true;
        }
        ConfigReader::section_map_t probe;
        if (config.get_section("zprobe", probe) && config.get_bool(probe, "enable", false)) {
            if (!ZProbe::create(config) || !Module::lookup("zprobe")) {
                std::fprintf(stderr, "version 2 probe configuration failed\n");
                return 2;
            }
            fast_required = true;
        }
        ConfigReader::sub_section_map_t endstops;
        if (config.get_sub_sections("endstops", endstops)) {
            std::size_t enabled_count = 0;
            for (const auto &instance : endstops)
                if (instance.first != "common") enabled_count += config.get_bool(instance.second, "enable", false);
            if (enabled_count && (!Endstops::create(config) || !Module::lookup("endstops"))) {
                std::fprintf(stderr, "version 2 endstop configuration failed\n");
                return 2;
            }
            switches_enabled |= enabled_count != 0;
        }
        ConfigReader::sub_section_map_t extruders;
        if (config.get_sub_sections("extruder", extruders)) {
            std::size_t enabled_count = 0;
            for (const auto &instance : extruders)
                enabled_count += config.get_bool(instance.second, "enable", false);
            if (enabled_count) {
                bool configured;
                {
                    Extruder loader("extruder loader");
                    configured = loader.configure(config);
                }
                if (!configured || Module::lookup_group("extruder").size() != enabled_count) {
                    std::fprintf(stderr, "version 2 extruder configuration failed\n");
                    return 2;
                }
            }
        }
        std::size_t temperature_count = 0;
        if (config.get_sub_sections("temperature control", controls)) {
            for (const auto &control : controls) {
                if (!config.get_bool(control.second, "enable", false)) continue;
                ++temperature_count;
                if (std::string(config.get_string(control.second, "sensor", "thermistor")) == "max31855" &&
                    !SPI::register_thermocouple(config.get_int(control.second, "spi_channel", -1),
                                                config.get_string(control.second, "spi_select_pin", "nc"))) {
                    std::fprintf(stderr, "version 2 thermocouple configuration failed\n");
                    return 2;
                }
            }
        }
        temperature_enabled = temperature_count != 0;
        if (temperature_enabled && (!TemperatureControl::load_controls(config) ||
                                    Module::lookup_group("temperature control").size() != temperature_count)) {
            std::fprintf(stderr, "version 2 temperature configuration failed\n");
            return 2;
        }
        fast_required |= temperature_enabled;
        ConfigReader::sub_section_map_t switches;
        if (config.get_sub_sections("switch", switches)) {
            std::size_t enabled_count = 0;
            for (const auto &instance : switches) {
                const auto &settings = instance.second;
                if (!config.get_bool(settings, "enable", false)) continue;
                ++enabled_count;
                std::string type = config.get_string(settings, "output_type", "");
                fast_required |= type == "sigmadeltapwm";
            }
            if (enabled_count && (!Switch::load_switches(config) || Module::lookup_group("switch").size() != enabled_count)) {
                std::fprintf(stderr, "version 2 switch configuration failed\n");
                return 2;
            }
            switches_enabled |= enabled_count != 0;
        }
        ConfigReader::sub_section_map_t temperature_switches;
        if (config.get_sub_sections("temperature switch", temperature_switches)) {
            std::size_t enabled_count = 0;
            for (const auto &instance : temperature_switches)
                enabled_count += config.get_bool(instance.second, "enable", false);
            if (enabled_count && (!TemperatureSwitch::load(config) || Module::lookup_group("temperature switch").size() != enabled_count)) {
                std::fprintf(stderr, "version 2 temperature switch configuration failed\n");
                return 2;
            }
            temperature_switches_enabled = enabled_count != 0;
        }
        ConfigReader::section_map_t display_config;
        const bool display_enabled = config.get_section("max7219", display_config) && config.get_bool(display_config, "enable", false);
        ConfigReader::sub_section_map_t dro_config;
        std::vector<std::string> display_selects;
        const int cascaded = display_enabled ? config.get_int(display_config, "cascaded", 0) : 0;
        if (cascaded < 0 || cascaded == 1 || cascaded > 16) {
            std::fprintf(stderr, "invalid version 2 display cascade\n");
            return 2;
        }
        if (config.get_sub_sections("dro", dro_config)) {
            const auto common = dro_config.find("common");
            const bool enabled = common == dro_config.end() || config.get_bool(common->second, "enable", true);
            const int frequency = common == dro_config.end() ? 10 : config.get_int(common->second, "poll_frequency_hz", 10);
            if (enabled && (frequency <= 0 || frequency > 100)) {
                std::fprintf(stderr, "invalid version 2 digital readout polling frequency\n");
                return 2;
            }
            std::set<int> display_ids;
            for (const auto &axis : dro_config) {
                if (!enabled || axis.first == "common" || !config.get_bool(axis.second, "enable", true)) continue;
                if (axis.first.size() != 1 || std::string("xyzabc").find(axis.first) == std::string::npos || !display_enabled) {
                    std::fprintf(stderr, "unsupported version 2 module: dro.%s\n", axis.first.c_str());
                    return 2;
                }
                dro_enabled = true;
                std::string select = config.get_string(axis.second, "id", "");
                if (select.empty()) select = config.get_string(axis.second, "cs_pin", "nc");
                if (!cascaded) {
                    display_selects.push_back(select);
                } else {
                    char *end = nullptr;
                    const long id = std::strtol(select.c_str(), &end, 10);
                    if (select.empty() || !std::all_of(select.begin(), select.end(), [](unsigned char c) { return std::isdigit(c); }) ||
                        *end || id < 0 || id >= cascaded || !display_ids.insert(static_cast<int>(id)).second) {
                        std::fprintf(stderr, "invalid version 2 digital readout display id: %s\n", axis.first.c_str());
                        return 2;
                    }
                }
            }
        }
        if (display_enabled && (!MAX7219::create(config) || !Module::lookup("max7219"))) {
            std::fprintf(stderr, "version 2 display configuration failed\n");
            return 2;
        }
        if (cascaded) display_selects.push_back(config.get_string(display_config, "cs", "nc"));
        if (!display_selects.empty() && !display_model.configure(config.get_string(display_config, "clk", "nc"),
                                                                 config.get_string(display_config, "mosi", "nc"), display_selects, cascaded)) {
            std::fprintf(stderr, "invalid version 2 display pins\n");
            return 2;
        }
        if (dro_enabled && (!DRO::create(config) || !Module::lookup("DRO"))) {
            std::fprintf(stderr, "version 2 digital readout configuration failed\n");
            return 2;
        }
        for (auto &callback : startup_callbacks) callback();
        for (auto *module : Module::lookup_group("temperature switch")) {
            if (!static_cast<TemperatureSwitch *>(module)->is_valid()) {
                std::fprintf(stderr, "version 2 temperature switch output is invalid\n");
                return 2;
            }
        }
        ConfigReader::section_map_t laser;
        if (config.get_section("laser", laser) && config.get_bool(laser, "enable", false)) {
            if (config.get_int(laser, "proportional_power_frequency", 10000) <= 0 ||
                !Laser::create(config) || !Module::lookup("laser")) {
                std::fprintf(stderr, "version 2 laser configuration failed\n");
                return 2;
            }
            const std::string laser_pin = config.get_string(laser, "pwm_pin", "nc");
            uint32_t pwm_frequency = 0;
            for (int timer = 0; timer < 2; ++timer)
                for (int channel = 0; channel < 4; ++channel) {
                    Pwm *pin = Pwm::get_allocation(timer, channel);
                    if (pin && pin->to_string() == laser_pin) pwm_frequency = pin->get_frequency();
                }
            fast_required |= std::min<uint32_t>(pwm_frequency, config.get_int(laser, "proportional_power_frequency", 10000)) >= FastTicker::get_min_frequency();
            switches_enabled = true;
        }
    }
    smoothie::v2::MechanicalRuntime mechanics;
    smoothie::v2::ThermalRuntime thermal;
    bool mechanics_enabled = false, thermal_enabled = false, thermal_faulted = false;
    struct ThermalTimer {
        int token = -1;
        ~ThermalTimer() { if (token >= 0) sim_v2_scheduler().detach(token); }
    } thermal_timer;
    if (!machine_model_path.empty()) {
        std::ifstream model_file(machine_model_path);
        if (!model_file) {
            std::fprintf(stderr, "cannot open mechanical/thermal model configuration\n");
            return 2;
        }
        const auto model = smoothie::v2::read_machine_configuration(model_file);
        if (!model.error.empty()) {
            std::fprintf(stderr, "mechanical/thermal configuration failed: %s\n",model.error.c_str());
            return 2;
        }
        if (model.has_mechanics) {
            std::string error;
            if (!mechanics.start(model.mechanics, *robot, config, error)) {
                std::fprintf(stderr, "mechanical configuration failed: %s\n", error.c_str());
                return 2;
            }
            mechanics_enabled = true;
            Dispatcher::getInstance()->add_handler("sim-model", [&mechanics](std::string& arguments, OutputStream& output) {
                const auto response = mechanics.command(arguments);
                output.printf("%s", response.c_str());
                return true;
            });
        }
        if (!model.thermal.empty()) {
            const auto mapping_error = smoothie::v2::validate_thermal_bindings(config,model.thermal);
            if (!mapping_error.empty()) {
                std::fprintf(stderr,"thermal configuration failed: %s\n",mapping_error.c_str());
                return 2;
            }
            if (!thermal.start(model.thermal, [] { return sim_v2_timer_now_us()/1000000.0; })) {
                std::fprintf(stderr, "thermal configuration failed: check allocated heater outputs and ADC channels\n");
                return 2;
            }
            thermal_enabled = true;
            Dispatcher::getInstance()->add_handler("sim-thermal", [&thermal](std::string& arguments, OutputStream& output) {
                const auto response = thermal.command(arguments);
                if (!thermal.healthy()) Module::broadcast_halt(true);
                output.printf("%s",response.c_str());
                return true;
            });
            thermal_timer.token = sim_v2_scheduler().attach(100, [&] {
                if (!thermal.advance(sim_v2_timer_now_us()/1000000.0) && !thermal_faulted) {
                    thermal_faulted = true;
                    Module::broadcast_halt(true);
                }
            });
            if (thermal_timer.token < 0) {
                std::fprintf(stderr,"thermal feedback timer failed to start\n");
                return 2;
            }
        }
    }
    conveyor->start();
    if (!ticker->start()) {
        std::fprintf(stderr, "version 2 step timer failed to start\n");
        return 2;
    }
    if ((temperature_enabled || switches_enabled || temperature_switches_enabled || motor_drivers_enabled || button_box_enabled || dro_enabled || lathe_enabled || els_enabled) && !SlowTicker::getInstance()->start()) {
        std::fprintf(stderr, "version 2 slow module timer failed to start\n");
        return 2;
    }
    if (fast_required && !FastTicker::getInstance()->start()) {
        std::fprintf(stderr, "version 2 module timers failed to start\n");
        return 2;
    }
    SIM.step_period_us = 1000000.0 / ticker->get_frequency();
    SimMachineSnapshot machine;
    for (auto *motor : robot->actuators) machine.initial_steps.push_back(motor->get_current_step_position());
    std::ostringstream serial;
    OutputStream output(&serial);
    stop_target = &output;
    if (player_enabled) {
        Module::broadcast_in_commmand_ctx(true);
        sim_v2_task_step();
        if (!drain_command_queue()) {
            std::fprintf(stderr, "unhandled version 2 startup command\n");
            return 2;
        }
    }
    std::string line;
    if (!blocks_path.empty() && !inject_blocks(input_file, planner, conveyor)) {
        std::fprintf(stderr, "invalid block input\n");
        return 2;
    }
    while (blocks_path.empty() && std::getline(input_file, line)) {
        ++SIM.current_line;
        if (line.compare(0, 5, ";@sim") == 0) {
            std::istringstream directive(line.substr(5));
            std::string verb;
            int value;
            if (!(directive >> verb)) {
                std::fprintf(stderr, "invalid simulation directive at line %d\n", SIM.current_line);
                return 2;
            }
            if (verb == "button") {
                std::string name, trailing;
                if (!(directive >> name >> value) || (value != 0 && value != 1) || (directive >> trailing) ||
                    external_buttons.find(name) == external_buttons.end()) {
                    std::fprintf(stderr, "invalid external button directive at line %d\n", SIM.current_line);
                    return 2;
                }
                external_buttons[name] = value != 0;
                continue;
            }
            if (verb == "graphics-init" || verb == "graphics-refresh") {
                if (!graphics) { std::fprintf(stderr, "graphical display is not configured\n"); return 2; }
                if (verb == "graphics-init") { graphics->initDisplay(); graphics_initialized = true; }
                else if (graphics_initialized) graphics->refresh();
                else { std::fprintf(stderr, "graphical display is not initialized\n"); return 2; }
                continue;
            }
            if (verb == "graphics-pixel" || verb == "expect-graphics") {
                unsigned x, y, expected;
                if (!(directive >> x >> y >> expected) || x >= 128 || y >= 64 || expected > 1 || !graphics || !graphics_initialized) {
                    std::fprintf(stderr, "invalid graphical display directive at line %d\n", SIM.current_line);
                    return 2;
                }
                if (verb == "graphics-pixel") graphics->pixel(x, y, expected);
                else {
                    bool actual;
                    if (!graphics_model.pixel(x, y, actual) || actual != (expected != 0)) {
                        std::fprintf(stderr, "graphical display expectation failed at line %d\n", SIM.current_line);
                        return 2;
                    }
                }
                continue;
            }
            if (verb == "expect-display") {
                std::string select;
                unsigned device, reg, expected, actual;
                if (!(directive >> select >> device >> reg >> expected) || expected > 255 ||
                    !display_model.read(select, device, reg, actual) || actual != expected) {
                    std::fprintf(stderr, "simulated display expectation failed at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "expect-tm1638") {
                unsigned address, expected, actual;
                if (!(directive >> address >> expected) || expected > 255 ||
                    !panel_model.read_register(address, actual) || actual != expected) {
                    std::fprintf(stderr, "simulated panel expectation failed at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "tm-buttons") {
                if (!(directive >> value) || value < 0 || !panel_model.set_buttons(value)) {
                    std::fprintf(stderr, "invalid simulated panel buttons at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "encoder") {
                if (!(directive >> value) || !sim_v2_encoder_advance(value)) {
                    std::fprintf(stderr, "invalid simulated encoder input at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "encoder-after" || verb == "stop-after") {
                int delay, counts = 0;
                if (!(directive >> delay) || delay <= 0 ||
                    (verb == "stop-after" && stop_pending) ||
                    (verb == "encoder-after" && (!(directive >> counts) || !sim_v2_encoder_advance(0)))) {
                    std::fprintf(stderr, "invalid scheduled lathe input at line %d\n", SIM.current_line);
                    return 2;
                }
                auto event = std::make_shared<int>(-1);
                const bool encoder = verb == "encoder-after";
                *event = sim_v2_scheduler().attach_period(delay * 1000.0, [event, encoder, counts] {
                    if (encoder) sim_v2_encoder_advance(counts);
                    else { stop_pending = false; if (stop_target) stop_target->set_stop_request(true); }
                    sim_v2_scheduler().detach(*event);
                });
                if (*event < 0) return 2;
                if (!encoder) stop_pending = true;
                continue;
            }
            if (verb == "expect-pwm") {
                std::string pin;
                double expected;
                Pwm *actual = nullptr;
                if (directive >> pin >> expected && pin.size() == 6) {
                    for (char &c : pin) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
                    if (pin.compare(0, 3, "PWM") == 0 && pin[4] == '_' &&
                        pin[3] >= '1' && pin[3] <= '2' && pin[5] >= '1' && pin[5] <= '4')
                        actual = Pwm::get_allocation(pin[3] - '1', pin[5] - '1');
                }
                if (!actual || !std::isfinite(expected) || expected < 0 || expected > 1 ||
                    std::fabs(actual->get() - expected) > 0.0001) {
                    std::fprintf(stderr, "simulated PWM expectation failed at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "expect-pin") {
                std::string pin;
                bool actual;
                if (!(directive >> pin >> value) || (value != 0 && value != 1) ||
                    !Pin::external_level(pin, actual) || actual != static_cast<bool>(value)) {
                    std::fprintf(stderr, "simulated pin expectation failed at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "pulses") {
                std::string pin, trailing;
                if (!(directive >> pin >> value) || (directive >> trailing) ||
                    !Pin::emit_external_pulses(pin, value)) {
                    std::fprintf(stderr, "invalid simulated pulses at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "pin") {
                std::string pin;
                if (!(directive >> pin >> value) || (value != 0 && value != 1) || !Pin::set_external_level(pin, value)) {
                    std::fprintf(stderr, "invalid simulated input pin at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "pin-after") {
                int delay, level;
                std::string pin;
                if (!(directive >> delay >> pin >> level) || delay <= 0 || (level != 0 && level != 1) || !Pin::has_external_pin(pin)) {
                    std::fprintf(stderr, "invalid scheduled input pin at line %d\n", SIM.current_line);
                    return 2;
                }
                auto event = std::make_shared<int>(-1);
                *event = sim_v2_scheduler().attach_period(delay * 1000.0, [event, pin, level] {
                    Pin::set_external_level(pin, level);
                    sim_v2_scheduler().detach(*event);
                });
                if (*event < 0) return 2;
                continue;
            }
            if (verb == "adc") {
                std::string token;
                if (!(directive >> token)) {
                    std::fprintf(stderr, "invalid ADC directive at line %d\n", SIM.current_line);
                    return 2;
                }
                std::istringstream number(token);
                char trailing;
                if (number >> value && !(number >> trailing) && value >= 0 && value <= 4095) {
                    SIM.adc_value = value;
                    Adc::clear_external_values();
                } else if (!(directive >> value) || !Adc::set_external_value(token, value)) {
                    std::fprintf(stderr, "invalid ADC directive at line %d\n", SIM.current_line);
                    return 2;
                }
                continue;
            }
            if (verb == "thermocouple") {
                std::string select_pin, reading;
                double temperature;
                if (!(directive >> select_pin >> reading)) {
                    std::fprintf(stderr, "invalid thermocouple directive at line %d\n", SIM.current_line);
                    return 2;
                }
                std::istringstream number(reading);
                char trailing;
                if (number >> temperature && !(number >> trailing)) {
                    if (SPI::set_thermocouple(select_pin, temperature)) continue;
                } else if (SPI::set_thermocouple_fault(select_pin, reading)) continue;
                std::fprintf(stderr, "invalid thermocouple directive at line %d\n", SIM.current_line);
                return 2;
            }
            if (!(directive >> value) || value < 0) {
                std::fprintf(stderr, "invalid simulation directive at line %d\n", SIM.current_line);
                return 2;
            }
            if (verb == "wait") {
                while (value > 0) {
                    int elapsed = value < 10 ? value : 10;
                    conveyor->check_queue();
                    safe_sleep(elapsed);
                    Module::broadcast_in_commmand_ctx(true);
                    sim_v2_task_step();
                    if (!drain_command_queue()) {
                        std::fprintf(stderr, "unhandled version 2 queued command\n");
                        return 2;
                    }
                    value -= elapsed;
                }
            } else {
                std::fprintf(stderr, "unsupported simulation directive at line %d\n", SIM.current_line);
                return 2;
            }
            continue;
        }
        const auto command_end = line.find_first_of(" \t");
        const std::string console_command = line.substr(0, command_end);
        const bool sd_command = (console_command == "M23" || console_command == "M32") && command_end != std::string::npos;
        const bool player_command = player_enabled && (console_command == "play" || console_command == "progress" ||
                                    console_command == "abort" || console_command == "suspend" || console_command == "resume" || sd_command);
        if ((mechanics_enabled && console_command == "sim-model") || (thermal_enabled && console_command == "sim-thermal") || player_command || (lathe_enabled && console_command == "rpm") || console_command == "thermistors") {
            if (sd_command && sd_root.empty()) {
                std::fprintf(stderr, "simulated SD-card root is required at line %d\n", SIM.current_line);
                return 2;
            }
            std::string dispatch_command = line;
            if (sd_command) dispatch_command[0] = 'm';
            if (!Dispatcher::getInstance()->dispatch(dispatch_command.c_str(), output)) {
                std::fprintf(stderr, "unhandled console command at line %d\n", SIM.current_line);
                return 2;
            }
            SIM.serial_out += serial.str();
            serial.str("");
            serial.clear();
            conveyor->check_queue();
            Module::broadcast_in_commmand_ctx(true);
            sim_v2_task_step();
            if (!drain_command_queue()) {
                std::fprintf(stderr, "unhandled version 2 queued command\n");
                return 2;
            }
            continue;
        }
        GCodeProcessor::GCodes_t commands;
        if (!command_parser.parse(line.c_str(), commands)) {
            std::fprintf(stderr, "invalid G-code at line %d\n", SIM.current_line);
            return 2;
        }
        for (auto &command : commands) {
            if (!command.has_g() && !command.has_m()) continue;
            if (lathe_enabled && command.has_g() && command.get_code() == 33 && !command.has_arg('Z') && !stop_pending) {
                std::fprintf(stderr, "version 2 manual lathe mode needs a scheduled stop at line %d\n", SIM.current_line);
                return 2;
            }
            if (player_enabled && unsupported_file_command(command)) {
                std::fprintf(stderr, "version 2 file playback is unsupported at line %d\n", SIM.current_line);
                return 2;
            }
            if (!dispatch_with_stop(command, output)) {
                std::fprintf(stderr, "unhandled G-code at line %d\n", SIM.current_line);
                return 2;
            }
            SIM.serial_out += serial.str();
            serial.str("");
            serial.clear();
        }
        conveyor->check_queue();
        Module::broadcast_in_commmand_ctx(true);
        sim_v2_task_step();
        if (!drain_command_queue()) {
            std::fprintf(stderr, "unhandled version 2 queued command\n");
            return 2;
        }
    }
    if (input_file.bad()) {
        std::fprintf(stderr, "error reading G-code file\n");
        return 2;
    }
    SIM.current_line = 0;
    bool queue_settled = false;
    for (int count = 0; count < 100000; ++count) {
        sim_v2_task_step();
        if (!drain_command_queue()) {
            std::fprintf(stderr, "unhandled version 2 queued command\n");
            return 2;
        }
        conveyor->check_queue();
        Module::broadcast_in_commmand_ctx(true);
        if (!sim_v2_task_active() && command_queue.empty()) {
            conveyor->wait_for_idle();
            if (command_queue.empty()) { queue_settled = true; break; }
        } else {
            safe_sleep(1);
        }
    }
    if (!queue_settled) {
        std::fprintf(stderr, "version 2 queued commands did not settle\n");
        return 2;
    }
    if (nested_command_error) return 2;
    if (mechanics_enabled && !mechanics.check_health()) {
        std::fprintf(stderr, "mechanical feedback became unhealthy\n");
        return 2;
    }
    if (thermal_enabled && !thermal.healthy()) {
        std::fprintf(stderr,"thermal feedback became unhealthy\n");
        return 2;
    }
    SIM.now_us = static_cast<uint64_t>(sim_v2_timer_now_us());
    SIM.serial_out += serial.str();
    for (std::size_t i = 0; i < robot->actuators.size(); ++i) {
        auto *motor = robot->actuators[i];
        machine.steps_per_mm.push_back(motor->get_steps_per_mm());
        machine.pulse_position.push_back(i < SIM.pulse_position.size() ? SIM.pulse_position[i] : 0);
        machine.machine_position.push_back(robot->get_axis_position(i));
        machine.final_steps.push_back(motor->get_current_step_position());
    }
    if (!sim_write_result(output_path.c_str(), blocks_path.empty() ? "gcode" : "blocks", full_steps, machine)) return 2;
    if (!spi_trace.empty() && !SPI::write_trace(spi_trace)) {
        std::fprintf(stderr, "cannot write version 2 SPI trace\n");
        return 2;
    }
    if (!motor_trace.empty() && !sim_v2_motor_trace_write(motor_trace)) {
        std::fprintf(stderr, "cannot write version 2 motor trace\n");
        return 2;
    }
    ticker->stop();
    sim_v2_race_report();
    if (!alloc_trace.empty()) sim_alloc_write(alloc_trace.c_str());
    return 0;
}
