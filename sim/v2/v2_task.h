#pragma once
#include <cstdint>

// One cooperative player task. The command thread drives it at safe boundaries.
int sim_v2_task_create(void (*entry)(void *), const char *name, void *argument);
bool sim_v2_task_active();
bool sim_v2_task_running();
uint64_t sim_v2_task_generation();
const char *sim_v2_task_name();
void sim_v2_task_yield(unsigned delay_ms);
void sim_v2_task_step();
bool sim_v2_join_aborted_player();
