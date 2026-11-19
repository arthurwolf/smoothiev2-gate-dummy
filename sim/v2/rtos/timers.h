#pragma once
#include "FreeRTOS.h"
using TimerHandle_t = void *;
using TimerCallbackFunction_t = void (*)(TimerHandle_t);
TimerHandle_t xTimerCreate(const char *, TickType_t, int, void *, TimerCallbackFunction_t);
int xTimerStart(TimerHandle_t, TickType_t);
int xTimerStop(TimerHandle_t, TickType_t);
int xTimerDelete(TimerHandle_t, TickType_t);
void *pvTimerGetTimerID(TimerHandle_t);
const char *pcTimerGetName(TimerHandle_t);
