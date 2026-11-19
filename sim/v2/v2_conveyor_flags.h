#pragma once
#include <cstdint>

void sim_v2_conveyor_flag_preempt();
void sim_v2_conveyor_flag_lost(uint8_t bits);

// Model a shared byte's read-modify-write, including an interrupt between the two.
template<unsigned Offset, unsigned Width> struct SimV2ConveyorFlag {
    volatile uint8_t byte;
    static constexpr uint8_t mask = ((1u << Width) - 1u) << Offset;
    operator uint8_t() const { return (byte & mask) >> Offset; }
    SimV2ConveyorFlag &operator=(uint8_t value) {
        uint8_t before = byte;
        sim_v2_conveyor_flag_preempt();
        uint8_t lost = (byte ^ before) & ~mask;
        if (lost) sim_v2_conveyor_flag_lost(lost);
        byte = (before & ~mask) | ((value << Offset) & mask);
        return *this;
    }
};
