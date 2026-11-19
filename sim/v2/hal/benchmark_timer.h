#pragma once
#include <cstdint>
extern "C" uint32_t get_microseconds();
extern "C" double sim_v2_timer_now_us();
extern "C" void sim_v2_timer_advance_to(double);
inline uint32_t benchmark_timer_start() { return get_microseconds(); }
inline uint32_t benchmark_timer_elapsed(uint32_t start) {
    // Firmware busy waits read a running hardware counter; advance the host
    // clock on each read so its timer interrupts can also run during the wait.
    sim_v2_timer_advance_to(sim_v2_timer_now_us() + 1.0);
    return get_microseconds() - start;
}
inline uint32_t benchmark_timer_as_us(uint32_t ticks) { return ticks; }
inline uint32_t benchmark_timer_as_ns(uint32_t ticks) { return ticks * 1000U; }
