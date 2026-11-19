#include "Block.h"
#include "MemoryPool.h"
#include "sim_runtime.h"
#include "sim_v2_hooks.h"
#include <cassert>

SimState SIM;
MemoryPool pool;
MemoryPool *_DTCMRAM = &pool;
int main() {
    Block::init(3);
    Block block;
    block.steps[0] = 80; block.steps[1] = 40;
    block.direction_bits[1] = true;
    block.steps_event_count = 80;
    block.nominal_speed = 10;
    block.millimeters = 1.118f;
    block.tick_info[0].steps_to_move = 80;
    block.tick_info[0].steps_per_tick = 1LL << 61;
    SIM.current_line = 7;
    sim_v2_block_queued();
    SIM.current_line = 99;
    SIM.tick_counter = 12;
    SIM.now_us = 120;
    sim_v2_block_started(&block);
    assert(SIM.blocks.size() == 1 && SIM.active_block == 0);
    const auto &b = SIM.blocks[0];
    assert(b.gcode_line == 7 && b.start_tick == 12 && b.start_us == 120);
    assert(b.steps.size() == 3 && b.steps[0] == 80 && b.steps[1] == 40);
    assert(b.direction_bits == 2 && b.steps_event_count == 80);
    assert(b.steps_per_tick[0] == 0.5 && b.steps_to_move[0] == 80);
    assert(b.step_ticks.size() == 3 && b.dir_changes.size() == 3);
}
