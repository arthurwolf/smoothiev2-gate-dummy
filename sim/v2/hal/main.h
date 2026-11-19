#pragma once
#include <cstdint>
#include <functional>
using StartupFunc_t = std::function<void()>;
void register_startup(StartupFunc_t);
void safe_sleep(uint32_t);
extern uint8_t board_id;
float get_voltage_monitor(const char *);
