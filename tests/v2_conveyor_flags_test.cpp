#include "v2_conveyor_flags.h"
#include <cassert>

union Flags {
    uint8_t raw;
    SimV2ConveyorFlag<1, 1> allow_fetch;
    SimV2ConveyorFlag<3, 2> continuous_mode;
};

static Flags *flags;
static bool interrupt_enabled;
static uint8_t lost_bits;

void sim_v2_conveyor_flag_preempt()
{
    if (interrupt_enabled) flags->raw |= 1u << 3;
}
void sim_v2_conveyor_flag_lost(uint8_t bits) { lost_bits |= bits; }

int main()
{
    Flags storage{};
    flags = &storage;
    storage.continuous_mode = 2;
    assert(storage.raw == 16 && storage.continuous_mode == 2);
    storage.continuous_mode = 0;
    interrupt_enabled = true;
    storage.allow_fetch = 1;
    assert(storage.raw == 2 && lost_bits == 8);
}
