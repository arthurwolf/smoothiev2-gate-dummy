#pragma once
#include "FreeRTOS.h"
extern "C" uint32_t get_microseconds();
inline TickType_t xTaskGetTickCount() { return get_microseconds() / 1000; }
inline const char *pcTaskGetName(void *) { return "CommandThread"; }
