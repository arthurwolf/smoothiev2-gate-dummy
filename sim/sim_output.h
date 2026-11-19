#pragma once
#include <string>
#include <vector>

struct SimMachineSnapshot {
    std::vector<double> steps_per_mm, pulse_position, machine_position, initial_steps, final_steps;
};

bool sim_write_result(const char *path, const std::string &mode, bool full_steps,
                      const SimMachineSnapshot &machine);
