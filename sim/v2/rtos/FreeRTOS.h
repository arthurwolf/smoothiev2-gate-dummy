#pragma once
#include <cstdint>
using TickType_t = uint32_t;
using BaseType_t = int;
using TaskHandle_t = void *;
#define configTICK_RATE_HZ 1000
#include <cassert>
#define configASSERT(condition) assert(condition)
#define configMAX_TASK_NAME_LEN 16
#define portMAX_DELAY UINT32_MAX
#define pdPASS 1
#define tskIDLE_PRIORITY 0
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) (ms)
#define TICKS2MS(ticks) (ticks)
