#include "sim_runtime.h"
#include "ConfigReader.h"
#include "Dispatcher.h"
#include "GCodeProcessor.h"
#include "MemoryPool.h"
#include "Module.h"
#include "OutputStream.h"
#include "TemperatureControl.h"
#include "TemperatureSwitch.h"
#include "Laser.h"
#include "main.h"
#include "SlowTicker.h"
#include "FastTicker.h"
#include "Pin.h"
#include "Planner.h"
#include "Conveyor.h"
#include "Robot.h"
#include "StepTicker.h"
#include "Switch.h"
#include "Pwm.h"
#include <cassert>
#include <sstream>
#include <vector>
static std::vector<StartupFunc_t> startup_callbacks;
void register_startup(StartupFunc_t callback) { startup_callbacks.push_back(callback); }
extern "C" void sim_v2_timer_advance_to(double);
extern "C" double sim_v2_timer_now_us();
extern "C" void print_to_all_consoles(const char *s) { if (s) SIM.serial_out += s; }
void safe_sleep(uint32_t ms) { sim_v2_timer_advance_to(sim_v2_timer_now_us() + ms * 1000.0); }
bool dispatch_line(OutputStream &, const char *) { return false; }
SimState SIM;
Pin *fets_enable_pin = nullptr;
Pin *fets_power_enable_pin = nullptr;
MemoryPool pool;
MemoryPool *_DTCMRAM = &pool;
MemoryPool *_SRAM_1 = &pool;
int main()
{
    std::istringstream input("[motion control]\narm_solution = cartesian\n"
        "[actuator]\nalpha.step_pin = PA1\nalpha.dir_pin = PA2\nalpha.steps_per_mm = 80\n"
        "beta.step_pin = PB0\nbeta.dir_pin = PB1\nbeta.steps_per_mm = 80\n"
        "gamma.step_pin = PC0\ngamma.dir_pin = PC1\ngamma.steps_per_mm = 80\n"
        "[switch]\nfan.enable = true\nfan.output_pin = PD0\nfan.output_type = digital\n"
        "fan.input_on_command = M106\nfan.input_off_command = M107\n"
        "spindle.enable = true\nspindle.output_pin = PWM1_1\nspindle.output_type = hwpwm\n"
        "spindle.input_on_command = M3\nspindle.input_off_command = M5\n"
        "[temperature control]\nhotend.enable = true\nhotend.tool_id = 0\n"
        "hotend.heater_pin = PA0\nhotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n"
        "[temperature switch]\nhotend.enable = true\nhotend.designator = T\n"
        "hotend.switch = fan\nhotend.threshold_temp = 0\n"
        "[laser]\nenable = true\npwm_pin = PWM2_1\nproportional_power_frequency = 1000\n");
    ConfigReader config(input);
    auto *planner = Planner::getInstance();
    auto *conveyor = Conveyor::getInstance();
    auto *robot = Robot::createInstance();
    assert(planner->configure(config));
    assert(conveyor->configure(config));
    assert(robot->configure(config));
    assert(planner->initialize(robot->get_number_registered_motors()));
    conveyor->start();
    assert(StepTicker::getInstance()->start());
    assert(Pwm::setup(0, 1000));
    assert(Pwm::setup(1, 1000));
    assert(TemperatureControl::load_controls(config));
    assert(Switch::load_switches(config));
    assert(TemperatureSwitch::load(config));
    assert(Laser::create(config));
    assert(Module::lookup("laser"));
    for (auto &callback : startup_callbacks) callback();
    assert(SlowTicker::getInstance()->start());
    assert(FastTicker::getInstance()->start());
    std::ostringstream text;
    OutputStream output(&text);
    assert(Dispatcher::getInstance()->dispatch("fire 50", output));
    assert(Pwm::get_allocation(1, 0) && Pwm::get_allocation(1, 0)->get() == 0.5f);
    assert(Dispatcher::getInstance()->dispatch("fire off", output));
    assert(Pwm::get_allocation(1, 0)->get() == 0);
    GCodeProcessor parser;
    GCodeProcessor::GCodes_t commands;
    auto *fan = Module::lookup("switch", "fan");
    assert(fan);
    bool fan_state = false;
    assert(parser.parse("M106", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, output));
    assert(fan->request("state", &fan_state) && fan_state);
    commands.clear();
    assert(parser.parse("M107", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, output));
    assert(fan->request("state", &fan_state) && !fan_state);
    commands.clear();
    assert(parser.parse("M3 S50", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, output));
    assert(Pwm::get_allocation(0, 0) && Pwm::get_allocation(0, 0)->get() == 0.5f);
    commands.clear();
    assert(parser.parse("M5", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, output));
    assert(Pwm::get_allocation(0, 0)->get() == 0);
    commands.clear();
    assert(parser.parse("M104 S200", commands));
    for (auto &command : commands) assert(Dispatcher::getInstance()->dispatch(command, output));
    SIM.adc_value = 3900;
    sim_v2_timer_advance_to(1000000);
    assert(!Module::is_halted());
    assert(fan->request("state", &fan_state) && fan_state);
    SIM.adc_value = 0;
    sim_v2_timer_advance_to(2100000);
    assert(Module::is_halted());
    assert(!SIM.serial_out.empty());
}
