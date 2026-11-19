#pragma once
#include <cstdint>
#ifdef SIM_V2_COOPERATIVE_TASKS
#include "v2_task.h"
#include "v2_callback_context.h"
extern "C" double sim_v2_timer_now_us();
extern "C" void sim_v2_timer_advance_to(double);
#endif
struct SimV2Mutex { bool taken = false; uint64_t owner = 0; };
inline uint64_t sim_v2_mutex_owner() {
#ifdef SIM_V2_COOPERATIVE_TASKS
    if (sim_v2_callback_depth()) return 2 + sim_v2_callback_depth();
    return sim_v2_task_running() ? (UINT64_C(1) << 63) | sim_v2_task_generation() : 1;
#else
    return 1;
#endif
}
inline void *xSemaphoreCreateMutex() { return new SimV2Mutex; }
inline void vSemaphoreDelete(void *handle) { delete static_cast<SimV2Mutex *>(handle); }
inline int xSemaphoreTake(void *handle, uint32_t timeout) {
    auto *mutex = static_cast<SimV2Mutex *>(handle);
    if (!mutex) return 0;
#ifdef SIM_V2_COOPERATIVE_TASKS
    if (mutex->taken && timeout && !sim_v2_callback_depth()) {
        const uint32_t start = static_cast<uint32_t>(static_cast<uint64_t>(sim_v2_timer_now_us() / 1000.0));
        while (mutex->taken) {
            const uint32_t now = static_cast<uint32_t>(static_cast<uint64_t>(sim_v2_timer_now_us() / 1000.0));
            if (timeout != UINT32_MAX && now - start >= timeout) return 0;
            if (sim_v2_task_running()) {
                sim_v2_task_yield(1);
            } else {
                sim_v2_task_step();
                if (mutex->taken) sim_v2_timer_advance_to(sim_v2_timer_now_us() + 1000.0);
            }
        }
    }
#else
    (void)timeout;
#endif
    if (mutex->taken) return 0;
    mutex->taken = true;
    mutex->owner = sim_v2_mutex_owner();
    return 1;
}
inline int xSemaphoreGive(void *handle) {
    auto *mutex = static_cast<SimV2Mutex *>(handle);
    if (!mutex || !mutex->taken || mutex->owner != sim_v2_mutex_owner()) return 0;
    mutex->taken = false;
    mutex->owner = 0;
    return 1;
}
