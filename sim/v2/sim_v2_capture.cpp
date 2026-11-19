#include "sim_v2_hooks.h"
#include "sim_runtime.h"
#include "Block.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "step_feedback.h"
#include "Module.h"
#include <cstdio>
#include <fstream>
#include <map>
#include <utility>
#include <typeindex>
#include <cxxabi.h>
#include <cstdlib>

extern "C" double sim_v2_timer_now_us();
extern "C" bool sim_v2_timer_force_step();
extern "C" bool sim_v2_timer_step_within(double);
extern "C" void sim_v2_timer_advance_to(double);

void sim_v2_spin_once() { sim_v2_timer_advance_to(sim_v2_timer_now_us() + 100.0); }
uint64_t *sim_v2_timer_callback_counter(const char *name) { return &SIM.v2_timer_callback_counts[name]; }
ElapsedStat *sim_v2_timer_callback_elapsed(const char *name) { return &SIM.v2_timer_callback_elapsed_us[name]; }
SimV2HandlerMethodScope::SimV2HandlerMethodScope(const char *method) : name(method), start_us(sim_v2_timer_now_us())
{
    ++SIM.v2_handler_method_counts[name];
}
SimV2HandlerMethodScope::~SimV2HandlerMethodScope()
{
    auto elapsed = static_cast<uint64_t>(sim_v2_timer_now_us() - start_us);
    auto &stat = SIM.v2_handler_method_elapsed_us[name];
    stat.total_us += elapsed;
    if (elapsed > stat.max_us) stat.max_us = elapsed;
}
void sim_v2_handler_method_register(const char *name) { SIM.v2_handler_method_counts.emplace(name, 0); }

namespace {
struct RaceState {
    uint64_t interval = 0, points = 0, fired = 0, target_started = 0, protected_writes_while_ticking = 0;
    uint64_t unprotected_rate_writes_while_ticking = 0;
    uint64_t conveyor_flag_writes = 0, conveyor_flag_interrupts = 0, conveyor_lost_stores = 0;
    double window_us = 0;
    bool injecting = false;
} race;
bool motor_trace_enabled = false;
std::map<unsigned, int64_t> motor_pulses;
std::vector<std::string> command_stack;
std::map<std::type_index, std::string> handler_classes;

const std::string &handler_class(const std::type_info &type)
{
    auto found = handler_classes.find(std::type_index(type));
    if (found != handler_classes.end()) return found->second;
    int status = 0;
    char *raw = abi::__cxa_demangle(type.name(), nullptr, nullptr, &status);
    std::string description = status == 0 && raw ? raw : "";
    std::free(raw);
    std::string name = "unattributed";
    const auto end = description.find("::*");
    const auto begin = end == std::string::npos ? std::string::npos : description.rfind('(', end);
    if (description.compare(0, 11, "std::_Bind<") == 0 && begin != std::string::npos && end > begin + 1)
        name = description.substr(begin + 1, end - begin - 1);
    return handler_classes.emplace(std::type_index(type), std::move(name)).first->second;
}
}

SimV2ModuleScope::SimV2ModuleScope(const Module *module, const char *event)
    : name(std::string(module->get_group_name()) + "::" + event), start_us(sim_v2_timer_now_us()) {}

SimV2ModuleScope::~SimV2ModuleScope()
{
    ++SIM.v2_module_callback_counts[name];
    const auto elapsed = static_cast<uint64_t>(sim_v2_timer_now_us() - start_us);
    auto &stat = SIM.v2_module_callback_elapsed_us[name];
    stat.total_us += elapsed;
    if (elapsed > stat.max_us) stat.max_us = elapsed;
}

SimV2CommandScope::SimV2CommandScope(char kind, unsigned code, const std::type_info &type)
{
    command_stack.push_back(std::string(1, kind) + std::to_string(code));
    handler_name = handler_class(type) + "::" + command_stack.back();
}

SimV2CommandScope::SimV2CommandScope(const std::string &name, const std::type_info &type)
    : handler_name(handler_class(type) + "::" + name) { command_stack.push_back(name); }

SimV2CommandScope::~SimV2CommandScope()
{
    ++SIM.v2_command_callback_counts[command_stack.back()];
    ++SIM.v2_handler_callback_counts[handler_name];
    command_stack.pop_back();
}

void sim_v2_record_player_delay(uint64_t elapsed_us)
{
    auto &stat = SIM.v2_player_delays;
    ++stat.calls;
    stat.total_us += elapsed_us;
    if (elapsed_us > stat.max_us) stat.max_us = elapsed_us;
}

void sim_v2_record_command_wait(const char *kind, uint64_t elapsed_us)
{
    if (command_stack.empty()) return;
    WaitRec wait;
    wait.kind = kind;
    // Timer callbacks can run while dispatch is active; this is context, not attribution.
    wait.module = "command context";
    wait.event = command_stack.back();
    wait.microseconds = elapsed_us;
    wait.gcode_line = SIM.current_line;
    SIM.waits.push_back(wait);
}

void sim_v2_motor_trace_enable() { motor_trace_enabled = true; }

bool sim_v2_motor_trace_write(const std::string &path)
{
    std::ofstream output(path);
    if (!output) return false;
    output << "{";
    bool first = true;
    for (const auto &motor : motor_pulses) {
        if (!first) output << ",";
        output << "\"" << motor.first << "\":" << motor.second;
        first = false;
    }
    output << "}\n";
    output.close();
    return !output.fail();
}

void sim_v2_race_configure(uint64_t interval) { race.interval = interval; }
void sim_v2_race_window(double window_us) { race.window_us = window_us; }

static bool sim_v2_race_fire(uint64_t point)
{
    if (race.interval && point % race.interval == 0) return sim_v2_timer_force_step();
    return race.window_us > 0 && sim_v2_timer_step_within(race.window_us);
}

void sim_v2_conveyor_flag_preempt()
{
    if ((!race.interval && race.window_us <= 0) || SIM.in_isr || race.injecting) return;
    ++race.conveyor_flag_writes;
    race.injecting = true;
    if (sim_v2_race_fire(race.conveyor_flag_writes)) ++race.conveyor_flag_interrupts;
    race.injecting = false;
}

void sim_v2_conveyor_flag_lost(uint8_t) { ++race.conveyor_lost_stores; }

void sim_v2_race_checkpoint(Block *block)
{
    if ((!race.interval && race.window_us <= 0) || SIM.in_isr || race.injecting) return;
    ++race.points;
    race.injecting = true;
    const bool was_ticking = block->is_ticking;
    if (sim_v2_race_fire(race.points)) {
        ++race.fired;
        if (!was_ticking && block->is_ticking) ++race.target_started;
    }
    race.injecting = false;
}

void sim_v2_race_protected_write(Block *block)
{
    if ((race.interval || race.window_us > 0) && block->is_ticking) ++race.protected_writes_while_ticking;
}

void sim_v2_race_unprotected_write(Block *block)
{
    if (!race.interval && race.window_us <= 0) return;
    sim_v2_race_checkpoint(block);
    if (block->is_ticking) ++race.unprotected_rate_writes_while_ticking;
}

void sim_v2_race_report()
{
    if (race.interval || race.window_us > 0)
        std::fprintf(stderr, "SIM_V2_RACE points=%llu fired=%llu target_started=%llu protected_writes_while_ticking=%llu unprotected_rate_writes_while_ticking=%llu conveyor_flag_writes=%llu conveyor_flag_interrupts=%llu conveyor_lost_stores=%llu\n",
                     static_cast<unsigned long long>(race.points), static_cast<unsigned long long>(race.fired),
                     static_cast<unsigned long long>(race.target_started),
                     static_cast<unsigned long long>(race.protected_writes_while_ticking),
                     static_cast<unsigned long long>(race.unprotected_rate_writes_while_ticking),
                     static_cast<unsigned long long>(race.conveyor_flag_writes),
                     static_cast<unsigned long long>(race.conveyor_flag_interrupts),
                     static_cast<unsigned long long>(race.conveyor_lost_stores));
}

void sim_v2_step_tick_begin()
{
    SIM.now_us = static_cast<uint64_t>(sim_v2_timer_now_us());
    SIM.in_isr = true;
}

void sim_v2_step_tick_end()
{
    ++SIM.tick_counter;
    SIM.in_isr = false;
}

void sim_v2_step_pulse(StepperMotor *motor, bool direction, bool physical_direction)
{
    if (motor_trace_enabled) motor_pulses[motor->get_motor_id()] += physical_direction ? -1 : 1;
    const auto &actuators = Robot::getInstance()->actuators;
    std::size_t axis = 0;
    while (axis < actuators.size() && actuators[axis] != motor) ++axis;
    if (axis == actuators.size()) return;
    auto& feedback = smoothie::v2::step_feedback_port();
    if (feedback.attached())
        feedback.publish({axis, physical_direction, motor->is_enabled(), motor->get_steps_per_mm()});
    if (SIM.pulse_position.size() < actuators.size()) SIM.pulse_position.resize(actuators.size());
    SIM.pulse_position[axis] += physical_direction ? -1 : 1;
    if (SIM.active_block < 0) return;
    auto &record = SIM.blocks[SIM.active_block];
    if (axis >= record.step_ticks.size()) return;
    const auto offset = static_cast<uint32_t>(SIM.tick_counter - record.start_tick);
    const auto step_index = static_cast<uint32_t>(record.step_ticks[axis].size());
    if (SIM.capture_steps) record.step_ticks[axis].push_back(offset);
    record.step_dirs[axis] = direction;
    const int sign = direction ? -1 : 1;
    if (record.sign_now[axis] != sign) {
        record.dir_changes[axis].emplace_back(step_index, sign);
        record.sign_now[axis] = sign;
    }
    record.ticks_run = offset + 1;
}

void sim_v2_block_queued()
{
    SIM.queued_lines.push_back(SIM.current_line);
}

void sim_v2_block_started(Block *block)
{
    BlockRec record;
    record.index = static_cast<int>(SIM.blocks.size());
    if (static_cast<std::size_t>(record.index) < SIM.queued_lines.size())
        record.gcode_line = SIM.queued_lines[record.index];
    record.start_tick = SIM.tick_counter;
    record.start_us = SIM.now_us;
    const int axes = Block::n_actuators;
    for (int i = 0; i < axes; ++i) record.steps.push_back(block->steps[i]);
    record.steps_event_count = block->steps_event_count;
    record.nominal_rate = block->nominal_rate;
    record.nominal_speed = block->nominal_speed;
    record.millimeters = block->millimeters;
    record.entry_speed = block->entry_speed;
    record.exit_speed = block->exit_speed;
    record.acceleration = block->acceleration;
    record.initial_rate = block->initial_rate;
    record.maximum_rate = block->maximum_rate;
    record.max_entry_speed = block->max_entry_speed;
    record.accelerate_until = block->accelerate_until;
    record.decelerate_after = block->decelerate_after;
    record.total_move_ticks = block->total_move_ticks;
    record.direction_bits = static_cast<uint32_t>(block->direction_bits.to_ulong());
    record.is_g123 = block->is_g123;
    record.primary_axis = block->primary_axis;
    constexpr double scale = 4611686018427387904.0;
    for (int i = 0; i < axes; ++i) {
        const auto &tick = block->tick_info[i];
        record.steps_per_tick.push_back(tick.steps_per_tick / scale);
        record.acceleration_change.push_back(tick.acceleration_change / scale);
        record.deceleration_change.push_back(tick.deceleration_change / scale);
        record.plateau_rate.push_back(tick.plateau_rate / scale);
        record.steps_to_move.push_back(tick.steps_to_move);
    }
    record.step_ticks.assign(axes, {});
    record.step_dirs.assign(axes, 0);
    record.dir_changes.assign(axes, {});
    record.sign_now.assign(axes, 0);
    SIM.blocks.push_back(std::move(record));
    SIM.active_block = static_cast<int>(SIM.blocks.size()) - 1;
}
