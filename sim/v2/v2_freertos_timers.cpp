#include "timers.h"
#include "v2_scheduler.h"
#include <algorithm>
#include <deque>
#include <string>

namespace {
struct HostTimer {
    std::string name;
    TickType_t period;
    bool repeat;
    void *id;
    TimerCallbackFunction_t callback;
    int event = -1;
    unsigned long generation = 0;
};
struct PendingCallback { HostTimer *timer; unsigned long generation; };
std::deque<PendingCallback> pending_callbacks;
bool dispatching_callbacks = false;

void dispatch_callback(HostTimer *timer)
{
    pending_callbacks.push_back({timer, timer->generation});
    if (dispatching_callbacks) return;
    dispatching_callbacks = true;
    while (!pending_callbacks.empty()) {
        const auto pending = pending_callbacks.front();
        pending_callbacks.pop_front();
        if (pending.generation == pending.timer->generation)
            pending.timer->callback(pending.timer);
    }
    dispatching_callbacks = false;
}
}
TimerHandle_t xTimerCreate(const char *name, TickType_t period, int repeat, void *id, TimerCallbackFunction_t cb)
{
    if (!period || !cb) return nullptr;
    return new HostTimer{name ? name : "", period, repeat != 0, id, cb, -1, 0};
}
int xTimerStart(TimerHandle_t handle, TickType_t)
{
    auto *timer = static_cast<HostTimer *>(handle);
    if (!timer) return 0;
    if (timer->event >= 0) sim_v2_scheduler().detach(timer->event);
    ++timer->generation;
    timer->event = sim_v2_scheduler().attach_period(timer->period * 1000.0, [timer] {
        if (!timer->repeat) { sim_v2_scheduler().detach(timer->event); timer->event = -1; }
        dispatch_callback(timer);
    });
    return timer->event >= 0 ? pdPASS : 0;
}
int xTimerStop(TimerHandle_t handle, TickType_t)
{
    auto *timer = static_cast<HostTimer *>(handle);
    if (!timer) return 0;
    ++timer->generation;
    if (timer->event >= 0) sim_v2_scheduler().detach(timer->event);
    timer->event = -1;
    return pdPASS;
}
int xTimerDelete(TimerHandle_t handle, TickType_t)
{
    if (!handle) return 0;
    xTimerStop(handle, 0);
    pending_callbacks.erase(std::remove_if(pending_callbacks.begin(), pending_callbacks.end(),
        [handle](const PendingCallback &pending) { return pending.timer == handle; }), pending_callbacks.end());
    delete static_cast<HostTimer *>(handle);
    return pdPASS;
}
void *pvTimerGetTimerID(TimerHandle_t handle) { return static_cast<HostTimer *>(handle)->id; }
const char *pcTimerGetName(TimerHandle_t handle) { return static_cast<HostTimer *>(handle)->name.c_str(); }
