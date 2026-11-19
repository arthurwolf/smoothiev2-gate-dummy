#include "semphr.h"
#include "v2_task.h"
#include "v2_scheduler.h"
#include "Pin.h"
#include <cassert>

static double now_us = 0;
extern "C" double sim_v2_timer_now_us() { return now_us; }
extern "C" void sim_v2_timer_advance_to(double target) { now_us = target; }

struct Attempt { void *mutex; unsigned timeout; int result = -1; };
static void attempt(void *arg)
{
    auto *a = static_cast<Attempt *>(arg);
    a->result = xSemaphoreTake(a->mutex, a->timeout);
    if (a->result) xSemaphoreGive(a->mutex);
}

static void try_release_other_task_mutex(void *arg)
{
    auto *a = static_cast<Attempt *>(arg);
    a->result = xSemaphoreGive(a->mutex);
}

static void acquire_and_exit(void *arg)
{
    assert(xSemaphoreTake(arg, 0));
}

static void hold_then_release(void *arg)
{
    auto *mutex = arg;
    assert(xSemaphoreTake(mutex, 0));
    sim_v2_task_yield(2);
    assert(xSemaphoreGive(mutex));
}

struct CallbackAttempt { V2Scheduler *scheduler; void *mutex; int result = -1; };
static void run_callback(void *arg)
{
    auto *a = static_cast<CallbackAttempt *>(arg);
    a->scheduler->attach_period(1000, [a] { a->result = xSemaphoreTake(a->mutex, 5); });
    a->scheduler->advance_to(1000);
}

static void run_interrupt(void *arg)
{
    auto *a = static_cast<CallbackAttempt *>(arg);
    V2StepTimer timer;
    assert(timer.start(1000, 2, [a] { a->result = xSemaphoreTake(a->mutex, 5); }, [] {}));
    timer.advance_to(1000);
}

int main()
{
    void *mutex = xSemaphoreCreateMutex();
    assert(xSemaphoreTake(mutex, 0) == 1);
    Attempt foreign_release{mutex, 0};
    assert(sim_v2_task_create(try_release_other_task_mutex, "PlayThread", &foreign_release));
    sim_v2_task_step();
    assert(!sim_v2_task_active() && foreign_release.result == 0);
    assert(xSemaphoreTake(mutex, 0) == 0);
    Attempt released{mutex, 5};
    assert(sim_v2_task_create(attempt, "PlayThread", &released));
    sim_v2_task_step();
    assert(sim_v2_task_active() && released.result == -1);
    assert(xSemaphoreGive(mutex));
    now_us += 1000;
    sim_v2_task_step();
    assert(!sim_v2_task_active() && released.result == 1);

    assert(xSemaphoreTake(mutex, 0));
    Attempt timed_out{mutex, 2};
    assert(sim_v2_task_create(attempt, "PlayThread", &timed_out));
    sim_v2_task_step();
    now_us += 1000; sim_v2_task_step();
    assert(sim_v2_task_active() && timed_out.result == -1);
    now_us += 1000; sim_v2_task_step();
    assert(!sim_v2_task_active() && timed_out.result == 0);
    Attempt immediate{mutex, 0};
    assert(sim_v2_task_create(attempt, "PlayThread", &immediate));
    sim_v2_task_step();
    assert(!sim_v2_task_active() && immediate.result == 0);
    now_us = static_cast<double>(UINT32_MAX - 1) * 1000;
    Attempt wrap{mutex, 3};
    assert(sim_v2_task_create(attempt, "PlayThread", &wrap));
    sim_v2_task_step();
    now_us += 2000; sim_v2_task_step();
    assert(sim_v2_task_active() && wrap.result == -1);
    now_us += 1000; sim_v2_task_step();
    assert(!sim_v2_task_active() && wrap.result == 0);
    Attempt indefinite{mutex, UINT32_MAX};
    assert(sim_v2_task_create(attempt, "PlayThread", &indefinite));
    sim_v2_task_step();
    now_us += 10000; sim_v2_task_step();
    assert(sim_v2_task_active() && indefinite.result == -1);
    assert(xSemaphoreGive(mutex));
    now_us += 1000; sim_v2_task_step();
    assert(!sim_v2_task_active() && indefinite.result == 1);
    assert(xSemaphoreTake(mutex, 0));
    V2StepTimer timer;
    V2Scheduler scheduler(timer);
    CallbackAttempt callback{&scheduler, mutex};
    assert(sim_v2_task_create(run_callback, "PlayThread", &callback));
    sim_v2_task_step();
    assert(!sim_v2_task_active() && callback.result == 0);
    CallbackAttempt interrupt{&scheduler, mutex};
    assert(sim_v2_task_create(run_interrupt, "PlayThread", &interrupt));
    sim_v2_task_step();
    assert(!sim_v2_task_active() && interrupt.result == 0);
    assert(xSemaphoreGive(mutex));
    assert(sim_v2_task_create(hold_then_release, "PlayThread", mutex));
    sim_v2_task_step();
    assert(sim_v2_task_active());
    assert(xSemaphoreTake(mutex, 5) == 1);
    assert(!sim_v2_task_active());
    const double start = now_us;
    assert(xSemaphoreTake(mutex, 0) == 0 && now_us == start);
    assert(xSemaphoreTake(mutex, 2) == 0 && now_us == start + 2000);
    assert(xSemaphoreGive(mutex));
    Pin gpio("PA0", Pin::AS_INPUT);
    int gpio_result = -1;
    assert(gpio.as_interrupt([&] { gpio_result = xSemaphoreTake(mutex, 5); }));
    assert(xSemaphoreTake(mutex, 0));
    const double gpio_start = now_us;
    assert(Pin::set_external_level("PA0", true));
    assert(gpio_result == 0 && now_us == gpio_start);
    assert(xSemaphoreGive(mutex));
    assert(gpio.deinit());
    assert(sim_v2_task_create(acquire_and_exit, "PlayThread", mutex));
    sim_v2_task_step();
    assert(!sim_v2_task_active());
    Attempt prior_player_release{mutex, 0};
    assert(sim_v2_task_create(try_release_other_task_mutex, "PlayThread", &prior_player_release));
    sim_v2_task_step();
    assert(!sim_v2_task_active() && prior_player_release.result == 0);
    vSemaphoreDelete(mutex);
}
