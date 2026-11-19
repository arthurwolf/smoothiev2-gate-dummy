#pragma once
#include "FreeRTOS.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
extern "C" double sim_v2_timer_now_us();
extern "C" void sim_v2_timer_advance_to(double);
inline TickType_t xTaskGetTickCount() { return static_cast<TickType_t>(static_cast<uint64_t>(sim_v2_timer_now_us() / 1000.0)); }
#ifdef SIM_V2_COOPERATIVE_TASKS
#include "v2_task.h"
#include "v2_callback_context.h"
void sim_v2_record_player_delay(uint64_t elapsed_us) __attribute__((weak));
inline const char *pcTaskGetName(void *) { return sim_v2_task_name(); }
inline BaseType_t xTaskCreate(void (*entry)(void *), const char *name, unsigned, void *argument, unsigned, TaskHandle_t *) { return sim_v2_task_create(entry, name, argument); }
#else
inline const char *pcTaskGetName(void *) { return "CommandThread"; }
inline BaseType_t xTaskCreate(void (*)(void *), const char *, unsigned, void *, unsigned, TaskHandle_t *) { return 0; }
#endif
inline void vTaskDelete(void *) {}
inline void vTaskDelay(TickType_t ticks) {
#ifdef SIM_V2_COOPERATIVE_TASKS
    if (sim_v2_task_running()) {
        const double start = sim_v2_timer_now_us();
        const bool player_context = sim_v2_callback_depth() == 0;
        sim_v2_task_yield(ticks);
        if (player_context && sim_v2_record_player_delay)
            sim_v2_record_player_delay(static_cast<uint64_t>(sim_v2_timer_now_us() - start));
        return;
    }
#endif
    sim_v2_timer_advance_to(sim_v2_timer_now_us() + ticks * 1000.0);
}
