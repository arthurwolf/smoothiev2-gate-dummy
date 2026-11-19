#pragma once
#include <cstdint>
#include <string>
#include <typeinfo>
class Block;
class StepperMotor;
class Module;
void sim_alloc_firmware_scope_enter() __attribute__((weak));
void sim_alloc_firmware_scope_leave() __attribute__((weak));
class SimV2FirmwareAllocationScope {
public:
    SimV2FirmwareAllocationScope() { if (sim_alloc_firmware_scope_enter) sim_alloc_firmware_scope_enter(); }
    ~SimV2FirmwareAllocationScope() { if (sim_alloc_firmware_scope_leave) sim_alloc_firmware_scope_leave(); }
    SimV2FirmwareAllocationScope(const SimV2FirmwareAllocationScope &) = delete;
    SimV2FirmwareAllocationScope &operator=(const SimV2FirmwareAllocationScope &) = delete;
};
class SimV2ModuleScope {
public:
    SimV2ModuleScope(const Module *module, const char *event);
    ~SimV2ModuleScope();
    SimV2ModuleScope(const SimV2ModuleScope &) = delete;
    SimV2ModuleScope &operator=(const SimV2ModuleScope &) = delete;
private:
    std::string name;
    double start_us;
};
class SimV2CommandScope {
public:
    SimV2CommandScope(char kind, unsigned code, const std::type_info &type);
    SimV2CommandScope(const std::string &name, const std::type_info &type);
    ~SimV2CommandScope();
    SimV2CommandScope(const SimV2CommandScope &) = delete;
    SimV2CommandScope &operator=(const SimV2CommandScope &) = delete;
private:
    std::string handler_name;
};
void sim_v2_record_command_wait(const char *kind, uint64_t elapsed_us);
class SimV2HandlerMethodScope {
public:
    explicit SimV2HandlerMethodScope(const char *name);
    ~SimV2HandlerMethodScope();
    SimV2HandlerMethodScope(const SimV2HandlerMethodScope &) = delete;
    SimV2HandlerMethodScope &operator=(const SimV2HandlerMethodScope &) = delete;
private:
    const char *name;
    double start_us;
};
void sim_v2_handler_method_register(const char *name);
void sim_v2_spin_once();
// Called when a planned block enters the queue and when it starts executing.
void sim_v2_block_queued();
void sim_v2_block_started(Block *block);
void sim_v2_step_tick_begin();
void sim_v2_step_tick_end();
void sim_v2_step_pulse(StepperMotor *motor, bool direction, bool physical_direction);
void sim_v2_motor_trace_enable();
bool sim_v2_motor_trace_write(const std::string &path);
void sim_v2_race_configure(uint64_t interval);
void sim_v2_race_window(double window_us);
void sim_v2_race_checkpoint(Block *block);
void sim_v2_race_unprotected_write(Block *block);
void sim_v2_race_protected_write(Block *block);
void sim_v2_race_report();
