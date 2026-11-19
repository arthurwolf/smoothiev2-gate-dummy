#include "SlowTicker.h"
#include "task.h"
#include "timers.h"
#include <cassert>
extern "C" void sim_v2_timer_advance_to(double);
static TimerHandle_t target_timer;
static int target_calls;
static bool delete_target;
static void target_callback(TimerHandle_t) { ++target_calls; }
static void cancel_callback(TimerHandle_t)
{
    vTaskDelay(50);
    if (delete_target) assert(xTimerDelete(target_timer, 0) == pdPASS);
    else assert(xTimerStop(target_timer, 0) == pdPASS);
}
int main()
{
    auto *ticker = SlowTicker::getInstance();
    int calls = 0;
    bool delay_in_callback = false;
    assert(ticker->attach(3, [&] { ++calls; if (delay_in_callback) vTaskDelay(50); }) >= 0);
    assert(ticker->start());
    sim_v2_timer_advance_to(332999);
    assert(calls == 0);
    sim_v2_timer_advance_to(666000);
    assert(calls == 2);
    assert(ticker->stop());
    sim_v2_timer_advance_to(1000000);
    assert(calls == 2);
    assert(ticker->start());
    const TickType_t start = xTaskGetTickCount();
    vTaskDelay(333);
    assert(xTaskGetTickCount() - start == 333);
    assert(calls == 3);
    delay_in_callback = true;
    sim_v2_timer_advance_to(1666000);
    assert(xTaskGetTickCount() == 1716);
    assert(calls == 4);
    assert(ticker->stop());
    sim_v2_timer_advance_to(4294968000.0);
    assert(xTaskGetTickCount() == 4294968);
    SlowTicker::deleteInstance();
    ticker = SlowTicker::getInstance();
    bool first_running = false;
    int first_calls = 0, second_calls = 0;
    assert(ticker->attach(10, [&] { ++first_calls; first_running = true; vTaskDelay(50); first_running = false; }) >= 0);
    assert(ticker->attach(10, [&] { assert(!first_running); ++second_calls; }) >= 0);
    assert(ticker->start());
    sim_v2_timer_advance_to(4295068000.0);
    assert(first_calls == 1 && second_calls == 1);
    assert(ticker->stop());
    for (int deletion = 0; deletion != 2; ++deletion) {
        delete_target = deletion != 0;
        auto cancel = xTimerCreate("cancel", 100, 0, nullptr, cancel_callback);
        target_timer = xTimerCreate("target", 100, 0, nullptr, target_callback);
        assert(xTimerStart(cancel, 0) == pdPASS);
        assert(xTimerStart(target_timer, 0) == pdPASS);
        vTaskDelay(100);
        assert(target_calls == 0);
        assert(xTimerDelete(cancel, 0) == pdPASS);
        if (!delete_target) assert(xTimerDelete(target_timer, 0) == pdPASS);
    }
}
