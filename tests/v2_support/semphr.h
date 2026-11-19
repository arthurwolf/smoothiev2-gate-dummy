#pragma once
#include <cstdint>
inline void *xSemaphoreCreateMutex() { static int mutex; return &mutex; }
inline void vSemaphoreDelete(void *) {}
inline int xSemaphoreTake(void *, uint32_t) { return 1; }
inline int xSemaphoreGive(void *) { return 1; }
