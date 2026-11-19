// Integration test: real SmoothieV2 Block, StepTicker and StepperMotor with a
// controlled conveyor. This isolates timer and motor execution from the planner.
#include "Block.h"
#include "Conveyor.h"
#include "Module.h"
#include "Robot.h"
#include "StepTicker.h"
#include "StepperMotor.h"
#include "MemoryPool.h"
#include <cassert>
#include <cstdio>

extern "C" void sim_v2_timer_advance_to(double);
MemoryPool pool;
MemoryPool *_DTCMRAM = &pool;
bool Module::halted = false;
Module::Module(const char *) {}
Module::~Module() {}
Conveyor *Conveyor::instance = nullptr;
Conveyor::Conveyor() : Module("conveyor") {}
bool Conveyor::configure(ConfigReader &) { return true; }
void Conveyor::on_halt(bool) {}
Robot *Robot::instance = nullptr;
static Block *queued;
static int finished;
bool Conveyor::get_next_block(Block **block) {
    *block = queued;
    queued = nullptr;
    return *block != nullptr;
}
void Conveyor::block_finished() { ++finished; }
float Robot::get_feed_rate(int) const { return 0; }

int main() {
    Block::init(3);
    Block block;
    block.backlash_steps.fill(0);
    block.tick_info[0].steps_to_move = 4;
    block.tick_info[0].steps_per_tick = STEPTICKER_FPSCALE / 2;
    block.total_move_ticks = 8;
    block.accelerate_until = 100;
    block.decelerate_after = 100;
    block.direction_bits[0] = true;
    queued = &block;
    Pin step("PA0"), direction("PA1"), enable("PA2");
    StepperMotor motor(step, direction, enable);
    auto *ticker = StepTicker::getInstance();
    ticker->set_frequency(100000);
    ticker->set_unstep_time(2);
    ticker->register_actuator(&motor);
    assert(ticker->start());
    sim_v2_timer_advance_to(20);
    assert(step.get());
    assert(direction.get());
    sim_v2_timer_advance_to(22);
    assert(!step.get());
    sim_v2_timer_advance_to(100);
    assert(finished == 1);
    if (motor.get_step_count() != -4) {
        std::fprintf(stderr, "expected -4 steps, got %d\n", motor.get_step_count());
        return 1;
    }
    assert(!motor.is_moving());
    block.clear();
    block.backlash_steps.fill(0);
    block.tick_info[0].steps_to_move = 2;
    block.tick_info[0].steps_per_tick = STEPTICKER_FPSCALE / 2;
    block.total_move_ticks = 4;
    block.accelerate_until = 100;
    block.decelerate_after = 100;
    queued = &block;
    sim_v2_timer_advance_to(160);
    assert(finished == 2);
    assert(motor.get_step_count() == -2);
    assert(!direction.get());
    assert(!step.get());
    ticker->stop();
}
