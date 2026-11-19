// Real SmoothieV2 planner, conveyor and step ticker; Robot configuration is
// isolated here so this test measures motion without needing the command loop.
#include <string>
#include <vector>
#include <map>
#include <set>
#include <stack>
#include <functional>
#include <atomic>
#include <cstring>
#define private public
#include "Planner.h"
#undef private
#include "Conveyor.h"
#include "StepperMotor.h"
#include "StepTicker.h"
#include "Robot.h"
#include "ConfigReader.h"
#include "MemoryPool.h"
#include <cassert>

extern "C" void sim_v2_timer_advance_to(double);
extern "C" uint32_t get_microseconds();
MemoryPool pool;
MemoryPool *_DTCMRAM = &pool;
bool Module::halted = false;
Module::Module(const char *) {}
Module::~Module() {}
Robot *Robot::instance = nullptr;
Robot::Robot() : Module("robot") {}
bool Robot::configure(ConfigReader &) { return true; }
void Robot::on_halt(bool) {}
float Robot::get_feed_rate(int) const { return 0; }
void safe_sleep(uint32_t ms) { sim_v2_timer_advance_to(get_microseconds() + ms * 1000.0); }

int main() {
    Pin sx("PA0"), dx("PA1"), ex("PA2");
    Pin sy("PB0"), dy("PB1"), ey("PB2");
    Pin sz("PC0"), dz("PC1"), ez("PC2");
    StepperMotor x(sx, dx, ex), y(sy, dy, ey), z(sz, dz, ez);
    x.change_steps_per_mm(80); y.change_steps_per_mm(80); z.change_steps_per_mm(80);
    auto *robot = Robot::createInstance();
    robot->actuators = {&x, &y, &z};
    auto *ticker = StepTicker::getInstance();
    ticker->register_actuator(&x); ticker->register_actuator(&y); ticker->register_actuator(&z);
    assert(ticker->start());
    auto *planner = Planner::getInstance();
    assert(planner->initialize(3));
    auto *conveyor = Conveyor::getInstance();
    conveyor->start();
    ActuatorCoordinates target{};
    target[0] = 1.0f;
    float vector1[] = {1, 0, 0};
    assert(planner->append_block(target, 3, 10, 1, vector1, 100, 0, true));
    target[0] = 0.5f;
    float vector2[] = {-1, 0, 0};
    assert(planner->append_block(target, 3, 10, 0.5, vector2, 100, 0, true));
    conveyor->force_queue();
    sim_v2_timer_advance_to(2000000);
    assert(conveyor->is_idle());
    assert(x.get_step_count() == 40);
    assert(y.get_step_count() == 0 && z.get_step_count() == 0);
    assert(!sx.get());
    target[1] = 1.0f;
    float vector3[] = {0, 1, 0};
    assert(planner->append_block(target, 3, 10, 1, vector3, 100, 0, true));
    conveyor->force_queue();
    sim_v2_timer_advance_to(4000000);
    assert(conveyor->is_idle());
    assert(x.get_step_count() == 40 && y.get_step_count() == 80);
    assert(!sy.get());
    // Repeating a destination produces no queued steps.
    assert(planner->append_block(target, 3, 10, 1, vector3, 100, 0, true));
    assert(conveyor->is_idle());
    sim_v2_timer_advance_to(4100000);
    assert(x.get_step_count() == 40 && y.get_step_count() == 80);
    ticker->stop();
}
