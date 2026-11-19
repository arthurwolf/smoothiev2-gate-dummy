#pragma once
#include <functional>
#include <cstdint>
#include <utility>
#include "sim_runtime.h"
extern "C" double sim_v2_timer_now_us();

uint64_t *sim_v2_timer_callback_counter(const char *name);
ElapsedStat *sim_v2_timer_callback_elapsed(const char *name);
inline std::function<void()> sim_v2_named_timer(const char *name, std::function<void()> callback)
{
    auto *count = sim_v2_timer_callback_counter(name);
    auto *elapsed = sim_v2_timer_callback_elapsed(name);
    return [count, elapsed, callback = std::move(callback)] {
        const double start = sim_v2_timer_now_us();
        callback();
        const auto duration = static_cast<uint64_t>(sim_v2_timer_now_us() - start);
        elapsed->total_us += duration;
        if (duration > elapsed->max_us) elapsed->max_us = duration;
        ++*count;
    };
}
