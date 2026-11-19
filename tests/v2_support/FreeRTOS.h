#pragma once
#include <cstdint>
using TickType_t = uint32_t;
#define configTICK_RATE_HZ 1000
#define TICKS2MS(ticks) (ticks)
#include <cassert>
#define configASSERT(condition) assert(condition)
#define configMAX_TASK_NAME_LEN 16
#define portMAX_DELAY UINT32_MAX
