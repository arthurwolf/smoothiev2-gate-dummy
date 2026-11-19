// Real firmware configuration, command parsing, dispatch, planning and stepping.
#include "sim_v2_hooks.h"
#include "sim_runtime.h"
#include "sim_output.h"
#include "ConfigReader.h"
#include "Conveyor.h"
#include "Dispatcher.h"
#include "GCodeProcessor.h"
#include "MemoryPool.h"
#include "OutputStream.h"
#include "Planner.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StepTicker.h"
#include <cassert>
#include <sstream>

extern "C" uint32_t get_microseconds();
extern "C" void sim_v2_timer_advance_to(double);
Pin *fets_enable_pin = nullptr;
Pin *fets_power_enable_pin = nullptr;
extern "C" void print_to_all_consoles(const char *) {}
SimState SIM;
MemoryPool pool;
MemoryPool *_DTCMRAM = &pool;
MemoryPool *_SRAM_1 = &pool;
void safe_sleep(uint32_t ms) { sim_v2_timer_advance_to(get_microseconds() + ms * 1000.0); }

int main() {
    std::istringstream input(
        "[motion control]\narm_solution = cartesian\ndefault_acceleration = 100\n"
        "[actuator]\nalpha.step_pin = PA0\nalpha.dir_pin = PA1!\nalpha.steps_per_mm = 80\n"
        "beta.step_pin = PB0\nbeta.dir_pin = PB1\nbeta.steps_per_mm = 80\n"
        "gamma.step_pin = PC0\ngamma.dir_pin = PC1\ngamma.steps_per_mm = 80\n");
    ConfigReader config(input);
    auto *ticker = StepTicker::getInstance();
    auto *planner = Planner::getInstance();
    assert(planner->configure(config));
    auto *conveyor = Conveyor::getInstance();
    assert(conveyor->configure(config));
    auto *robot = Robot::createInstance();
    assert(robot->configure(config));
    assert(planner->initialize(robot->get_number_registered_motors()));
    conveyor->start();
    assert(ticker->start());
    std::ostringstream output;
    OutputStream stream(&output);
    GCodeProcessor processor;
    GCodeProcessor::GCodes_t commands;
    SIM.current_line = 1;
    assert(processor.parse("G1 X1 Y0.5 F600", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, stream));
    conveyor->force_queue();
    sim_v2_timer_advance_to(3000000);
    assert(conveyor->is_idle());
    assert(SIM.queued_lines.size() == 1 && SIM.blocks.size() == 1);
    assert(SIM.blocks[0].gcode_line == 1);
    assert(SIM.blocks[0].steps[0] == 80 && SIM.blocks[0].steps[1] == 40);
    assert(SIM.blocks[0].step_ticks[0].size() == 80);
    assert(SIM.blocks[0].step_ticks[1].size() == 40);
    assert(SIM.pulse_position[0] == -80 && SIM.pulse_position[1] == 40);
    assert(SIM.tick_counter > 0 && SIM.blocks[0].start_us > 0);
    assert(robot->actuators[0]->get_step_count() == 80);
    assert(robot->actuators[1]->get_step_count() == 40);
    commands.clear();
    assert(processor.parse("G91", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, stream));
    commands.clear();
    SIM.current_line = 3;
    assert(processor.parse("G1 X-0.5 Y0.5 F600", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, stream));
    conveyor->force_queue();
    sim_v2_timer_advance_to(6000000);
    assert(conveyor->is_idle());
    assert(robot->actuators[0]->get_step_count() == 40);
    assert(robot->actuators[1]->get_step_count() == 80);
    commands.clear();
    SIM.current_line = 4;
    assert(processor.parse("G90 G1 X0 Y0 F600", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, stream));
    conveyor->force_queue();
    sim_v2_timer_advance_to(9000000);
    assert(conveyor->is_idle());
    assert(robot->actuators[0]->get_step_count() == 0);
    assert(robot->actuators[1]->get_step_count() == 0);
    assert(SIM.queued_lines.size() == 3 && SIM.blocks.size() == 3);
    assert(SIM.blocks[1].gcode_line == 3 && SIM.blocks[2].gcode_line == 4);
    assert(SIM.blocks[1].steps[0] == 40 && SIM.blocks[1].steps[1] == 40);
    assert((SIM.blocks[1].direction_bits & 1) != 0);
    assert(SIM.blocks[1].step_ticks[0].size() == 40);
    assert(SIM.blocks[1].step_dirs[0] == 1);
    assert(SIM.pulse_position[0] == 0 && SIM.pulse_position[1] == 0);
    // Queue consecutive moves before advancing time so the firmware can
    // transition between blocks in one interrupt.
    commands.clear();
    SIM.current_line = 5;
    assert(processor.parse("G1 X1 F600", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, stream));
    commands.clear();
    SIM.current_line = 6;
    assert(processor.parse("G1 X2 F600", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, stream));
    conveyor->force_queue();
    sim_v2_timer_advance_to(12000000);
    assert(conveyor->is_idle());
    assert(SIM.blocks.size() == 5);
    assert(SIM.blocks[3].gcode_line == 5 && SIM.blocks[4].gcode_line == 6);
    assert(SIM.blocks[3].step_ticks[0].size() == 80);
    assert(SIM.blocks[4].step_ticks[0].size() == 80);
    assert(SIM.pulse_position[0] == -160);
    SimMachineSnapshot machine;
    for (std::size_t i = 0; i < robot->actuators.size(); ++i) {
        auto *motor = robot->actuators[i];
        machine.steps_per_mm.push_back(motor->get_steps_per_mm());
        machine.pulse_position.push_back(i < SIM.pulse_position.size() ? SIM.pulse_position[i] : 0);
        machine.machine_position.push_back(robot->get_axis_position(i));
        machine.final_steps.push_back(motor->get_current_step_position());
    }
    assert(sim_write_result("output/v2-result.json", "gcode", true, machine));
    ticker->stop();
    const auto before_manual_step = get_microseconds();
    robot->actuators[0]->manual_step(false);
    assert(get_microseconds() - before_manual_step >= 4);
}
