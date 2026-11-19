#include "sim_output.h"
#include "sim_runtime.h"
#include <cstdio>

// ---------------- JSON output ----------------
static std::string esc(const std::string &s) {
    std::string o; for (unsigned char c : s) { if (c == '"' || c == '\\') { o += '\\'; o += c; } else if (c == '\n') o += "\\n"; else if (c == '\r') o += "\\r"; else if (c < 32 || c >= 127) o += '?'; else o += c; } return o;
}
static void put_num(FILE *f, double v) { if (v != v || v > 1e300 || v < -1e300) fputs("null", f); else fprintf(f, "%.9g", v); }
template <typename T> static void put_arr(FILE *f, const std::vector<T> &v) { fputc('[', f); for (size_t i = 0; i < v.size(); i++) { if (i) fputc(',', f); put_num(f, (double)v[i]); } fputc(']', f); }

bool sim_write_result(const char *path, const std::string &mode, bool full_steps, const SimMachineSnapshot &machine) {
    FILE *f = fopen(path, "w"); if (!f) { perror(path); return false; }
    fprintf(f, "{\"schema\":1,\"mode\":\"%s\",\"step_period_us\":%.6f,\"n_actuators\":%d,\n", mode.c_str(), SIM.step_period_us, (int)machine.steps_per_mm.size());
    fputs("\"steps_per_mm\":", f); put_arr(f, machine.steps_per_mm);
    fputs(",\n\"pulse_position\":", f); put_arr(f, machine.pulse_position);
    fputs(",\n\"machine_position\":", f); put_arr(f, machine.machine_position);
    if (!machine.initial_steps.empty()) { fputs(",\n\"initial_steps\":", f); put_arr(f, machine.initial_steps); }
    fputs(",\n\"final_steps\":", f); put_arr(f, machine.final_steps);
    fputs(",\n\"blocks\":[\n", f);
    for (size_t bi = 0; bi < SIM.blocks.size(); bi++) {
        auto &b = SIM.blocks[bi];
        if (bi) fputs(",\n", f);
        fprintf(f, "{\"index\":%d,\"gcode_line\":%d,\"start_tick\":%llu,\"steps\":", b.index, b.gcode_line, (unsigned long long)b.start_tick); put_arr(f, b.steps);
        fprintf(f, ",\"steps_event_count\":%u,\"nominal_rate\":", b.steps_event_count); put_num(f, b.nominal_rate);
        fputs(",\"nominal_speed\":", f); put_num(f, b.nominal_speed); fputs(",\"millimeters\":", f); put_num(f, b.millimeters);
        fputs(",\"entry_speed\":", f); put_num(f, b.entry_speed); fputs(",\"exit_speed\":", f); put_num(f, b.exit_speed);
        fputs(",\"acceleration\":", f); put_num(f, b.acceleration); fputs(",\"initial_rate\":", f); put_num(f, b.initial_rate);
        fputs(",\"maximum_rate\":", f); put_num(f, b.maximum_rate); fputs(",\"max_entry_speed\":", f); put_num(f, b.max_entry_speed);
        fprintf(f, ",\"accelerate_until\":%u,\"decelerate_after\":%u,\"total_move_ticks\":%u,\"direction_bits\":%u,\"is_g123\":%s,\"primary_axis\":%s",
                b.accelerate_until, b.decelerate_after, b.total_move_ticks, b.direction_bits, b.is_g123 ? "true" : "false", b.primary_axis ? "true" : "false");
        fputs(",\"steps_per_tick\":", f); put_arr(f, b.steps_per_tick); fputs(",\"acceleration_change\":", f); put_arr(f, b.acceleration_change);
        fputs(",\"deceleration_change\":", f); put_arr(f, b.deceleration_change); fputs(",\"plateau_rate\":", f); put_arr(f, b.plateau_rate);
        fputs(",\"steps_to_move\":", f); put_arr(f, b.steps_to_move);
        fprintf(f, ",\"ticks_run\":%u,\"step_dirs\":", b.ticks_run); put_arr(f, b.step_dirs);
        fputs(",\"step_counts\":[", f); for (size_t m = 0; m < b.step_ticks.size(); m++) fprintf(f, "%s%zu", m ? "," : "", b.step_ticks[m].size()); fputc(']', f);
        fputs(",\"dir_changes\":[", f);
        for (size_t m = 0; m < b.dir_changes.size(); m++) { fputs(m ? ",[" : "[", f); for (size_t k = 0; k < b.dir_changes[m].size(); k++) fprintf(f, "%s[%u,%d]", k ? "," : "", b.dir_changes[m][k].first, b.dir_changes[m][k].second); fputc(']', f); }
        fputc(']', f);
        if (b.has_filament_advance) {
            fputs(",\"advance_lead\":", f); put_arr(f, b.advance_lead); fputs(",\"flip_tick\":", f); put_arr(f, b.flip_tick);
            fputs(",\"decel_start_adjust\":", f); put_arr(f, b.decel_start_adjust); fputs(",\"advance_motors\":", f); put_arr(f, b.advance_motors);
        }
        fputs(",\"step_ticks\":[", f);
        for (size_t m = 0; m < b.step_ticks.size(); m++) { if (m) fputc(',', f); if (full_steps || b.step_ticks[m].size() <= 4000) put_arr(f, b.step_ticks[m]); else { // thin out long lists to every Nth step
            std::vector<uint32_t> t; size_t stride = b.step_ticks[m].size() / 2000 + 1; for (size_t i = 0; i < b.step_ticks[m].size(); i += stride) t.push_back(b.step_ticks[m][i]); put_arr(f, t); } }
        fprintf(f, "],\"step_tick_stride\":[");
        for (size_t m = 0; m < b.step_ticks.size(); m++) fprintf(f, "%s%zu", m ? "," : "", (full_steps || b.step_ticks[m].size() <= 4000) ? (size_t)1 : b.step_ticks[m].size() / 2000 + 1);
        fputs("]}", f);
    }
    fputs("\n],\n\"waits\":[", f);
    for (size_t i = 0; i < SIM.waits.size(); i++) { auto &w = SIM.waits[i]; fprintf(f, "%s{\"kind\":\"%s\",\"module\":\"%s\",\"event\":\"%s\",\"us\":%llu,\"gcode_line\":%d}", i ? "," : "", w.kind.c_str(), esc(w.module).c_str(), w.event.c_str(), (unsigned long long)w.microseconds, w.gcode_line); }
    fputs("],\n\"handlers\":{", f);
    { bool first = true; for (auto &kv : SIM.handlers) { fprintf(f, "%s\"%s\":{\"calls\":%llu,\"self_us\":%llu,\"max_self_us\":%llu}", first ? "" : ",", esc(kv.first).c_str(), (unsigned long long)kv.second.calls, (unsigned long long)kv.second.self_us, (unsigned long long)kv.second.max_self_us); first = false; } }
    fputs("}", f);
    if (!SIM.v2_module_callback_counts.empty()) {
        fputs(",\n\"v2_module_callback_counts\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_module_callback_counts) {
            fprintf(f, "%s\"%s\":%llu", first ? "" : ",", esc(entry.first).c_str(), (unsigned long long)entry.second);
            first = false;
        }
        fputs("}", f);
    }
    if (!SIM.v2_module_callback_elapsed_us.empty()) {
        fputs(",\n\"v2_module_callback_elapsed_us\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_module_callback_elapsed_us) {
            fprintf(f, "%s\"%s\":{\"total_us\":%llu,\"max_us\":%llu}", first ? "" : ",", esc(entry.first).c_str(),
                    (unsigned long long)entry.second.total_us, (unsigned long long)entry.second.max_us);
            first = false;
        }
        fputs("}", f);
    }
    if (!SIM.v2_command_callback_counts.empty()) {
        fputs(",\n\"v2_command_callback_counts\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_command_callback_counts) {
            fprintf(f, "%s\"%s\":%llu", first ? "" : ",", esc(entry.first).c_str(), (unsigned long long)entry.second);
            first = false;
        }
        fputs("}", f);
    }
    if (!SIM.v2_timer_callback_counts.empty()) {
        fputs(",\n\"v2_timer_callback_counts\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_timer_callback_counts) {
            fprintf(f, "%s\"%s\":%llu", first ? "" : ",", esc(entry.first).c_str(), (unsigned long long)entry.second);
            first = false;
        }
        fputs("}", f);
    }
    if (!SIM.v2_timer_callback_elapsed_us.empty()) {
        fputs(",\n\"v2_timer_callback_elapsed_us\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_timer_callback_elapsed_us) {
            fprintf(f, "%s\"%s\":{\"total_us\":%llu,\"max_us\":%llu}", first ? "" : ",", esc(entry.first).c_str(),
                    (unsigned long long)entry.second.total_us, (unsigned long long)entry.second.max_us);
            first = false;
        }
        fputs("}", f);
    }
    if (!SIM.v2_handler_callback_counts.empty()) {
        fputs(",\n\"v2_handler_callback_counts\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_handler_callback_counts) {
            fprintf(f, "%s\"%s\":%llu", first ? "" : ",", esc(entry.first).c_str(), (unsigned long long)entry.second);
            first = false;
        }
        fputs("}", f);
    }
    if (!SIM.v2_handler_method_counts.empty()) {
        fputs(",\n\"v2_handler_method_counts\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_handler_method_counts) {
            fprintf(f, "%s\"%s\":%llu", first ? "" : ",", esc(entry.first).c_str(), (unsigned long long)entry.second);
            first = false;
        }
        fputs("}", f);
    }
    if (!SIM.v2_handler_method_elapsed_us.empty()) {
        fputs(",\n\"v2_handler_method_elapsed_us\":{", f);
        bool first = true;
        for (const auto &entry : SIM.v2_handler_method_elapsed_us) {
            fprintf(f, "%s\"%s\":{\"total_us\":%llu,\"max_us\":%llu}", first ? "" : ",", esc(entry.first).c_str(),
                    (unsigned long long)entry.second.total_us, (unsigned long long)entry.second.max_us);
            first = false;
        }
        fputs("}", f);
    }
    if (SIM.v2_player_delays.calls) {
        const auto &stat = SIM.v2_player_delays;
        fprintf(f, ",\n\"v2_task_delays\":{\"PlayThread\":{\"calls\":%llu,\"total_us\":%llu,\"max_us\":%llu}}",
                (unsigned long long)stat.calls, (unsigned long long)stat.total_us, (unsigned long long)stat.max_us);
    }
    fprintf(f, ",\n\"serial_out\":\"%s\",\n\"end_us\":%llu,\"ticks\":%llu}\n", esc(SIM.serial_out).c_str(), (unsigned long long)SIM.now_us, (unsigned long long)SIM.tick_counter);
    if (fclose(f) != 0) { perror(path); return false; }
    return true;
}
