#include "v2_task.h"
#include <ucontext.h>
#include <array>
#include <cstring>

extern "C" double sim_v2_timer_now_us();
extern "C" void sim_v2_timer_advance_to(double);

namespace {
ucontext_t command_context, player_context;
std::array<char, 128 * 1024> player_stack;
void (*player_entry)(void *) = nullptr;
void *player_argument = nullptr;
char player_name[16] = {};
bool active = false, running = false;
uint64_t generation = 0;
double wake_us = 0;

void run_player()
{
    player_entry(player_argument);
    active = false;
    running = false;
}
}

int sim_v2_task_create(void (*entry)(void *), const char *name, void *argument)
{
    if (active || !entry || !name || std::strcmp(name, "PlayThread") != 0 || generation == (UINT64_MAX >> 1)) return 0;
    if (getcontext(&player_context) != 0) return 0;
    player_context.uc_stack.ss_sp = player_stack.data();
    player_context.uc_stack.ss_size = player_stack.size();
    player_context.uc_stack.ss_flags = 0;
    player_context.uc_link = &command_context;
    player_entry = entry;
    player_argument = argument;
    std::strncpy(player_name, name, sizeof(player_name) - 1);
    player_name[sizeof(player_name) - 1] = '\0';
    wake_us = sim_v2_timer_now_us();
    ++generation;
    active = true;
    makecontext(&player_context, run_player, 0);
    return 1;
}

bool sim_v2_task_active() { return active; }
bool sim_v2_task_running() { return running; }
uint64_t sim_v2_task_generation() { return generation; }
const char *sim_v2_task_name() { return running ? player_name : "CommandThread"; }

void sim_v2_task_yield(unsigned delay_ms)
{
    if (!running) return;
    wake_us = sim_v2_timer_now_us() + delay_ms * 1000.0;
    running = false;
    swapcontext(&player_context, &command_context);
    running = true;
}

void sim_v2_task_step()
{
    if (!active || running || sim_v2_timer_now_us() < wake_us) return;
    running = true;
    swapcontext(&command_context, &player_context);
    running = false;
}

bool sim_v2_join_aborted_player()
{
    for (int count = 0; active && count < 1000; ++count) {
        sim_v2_task_step();
        if (active) sim_v2_timer_advance_to(sim_v2_timer_now_us() + 1000);
    }
    return !active;
}
