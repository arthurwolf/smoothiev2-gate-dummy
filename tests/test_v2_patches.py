"""Version 2 motion capture must fail closed when firmware hook sites drift."""
import pathlib
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "stage_v2_patch.py"


class MotionHooks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        (self.root / 'Module.cpp').write_text('#include "Module.h"\n'
            '                j.second->on_halt(flg);\n'
            '            x.module->on_halt(flg);\n'
            '                    j.second->in_command_ctx(idle);\n'
            '                x.module->in_command_ctx(idle);\n')
        libs = self.root / 'libs'
        libs.mkdir()
        (libs / 'OutputStream.h').write_text('OutputStream() : xWriteMutex(nullptr), os(nullptr), fdbuf(nullptr), deleteos(false) { usb_flag= closed= uploading= false; clear_flags(); };\n'
                                             'OutputStream(std::ostream *o) : xWriteMutex(nullptr), os(o), fdbuf(nullptr), deleteos(false) { usb_flag= closed= uploading= false; clear_flags(); };\n')
        (self.root / 'Dispatcher.cpp').write_text('#include "Dispatcher.h"\n\tfor (auto it = f.first; it != f.second; ++it) {\n\t\tif(it->second(gc, os)) {\n'
                                                  '\tfor (auto it = f.first; it != f.second; ++it) {\n\t\tif(it->second(params, os)) {\n')
        self.robot = self.root / "robot"
        self.robot.mkdir()
        (self.robot / 'Conveyor.h').write_text('#include "Module.h"\n'
            '    struct {\n        volatile bool running:1;\n        volatile bool allow_fetch:1;\n'
            '        volatile bool hold_queue:1;\n        volatile uint8_t continuous_mode:2;\n'
            '        bool flush:1;\n        bool halted:1;\n    };\n')
        (self.robot / "Planner.cpp").write_text('#include "Planner.h"\n    return true;\n}\n\nvoid Planner::recalculate()\n'
                                                '    block->maximum_rate = std::min(maximum_possible_rate, block->nominal_rate);\n'
                                                '    block->locked= true;\n\n    // is_ticking was tested\n'
                                                '    block->accelerate_until = acceleration_ticks;\n')
        (self.robot / "StepTicker.cpp").write_text('#include "StepTicker.h"\n    StepTicker::getInstance()->step_tick();\n    current_tick = 0;\n\n    if(ok) {\n')
        (self.robot / "StepperMotor.h").write_text('#include "Pin.h"\n            step_pin.set(1); step_count += (direction?-1:1);\n')
        (self.robot / "Robot.cpp").write_text('cr.get_string(mm, driver_type_key, a >= 4 ? "external" : def_driver)\n')
        filament = self.root / 'modules/tools/filamentdetector'
        filament.mkdir(parents=True)
        (filament / 'FilamentDetector.cpp').write_text('    filament_out_alarm= false;\n    bulge_detected= false;\n')
        probe = self.root / 'modules/tools/zprobe'
        probe.mkdir(parents=True)
        (probe / 'ZProbe.cpp').write_text('            printf("ERROR: config-zprobe: Unknown leveling stratagy: %s", leveling.c_str());\n'
                                         '                printf("ERROR: config-zprobe: failed to load %s leveling strategy\\n", leveling.c_str());\n'
                                         '            printf("ERROR: config-zprobe: Unknown calibration stratagy: %s", calibration.c_str());\n'
                                         '                printf("ERROR: config-zprobe: failed to load %s calibration strategy\\n", calibration.c_str());\n'
                                         '                if(leveling_strategy->handle_gcode(gcode, os)) {\n                    return true;\n\n                } else if(calibration_strategy->handle_gcode(gcode, os)) {\n'
                                         '                if(i == 0) {\n                    if(leveling_strategy->handle_gcode(gcode, os)) {\n'
                                         '                } else if(i == 1) {\n                    if(calibration_strategy->handle_gcode(gcode, os)) {\n')
        temperature = self.root / 'modules/tools/temperaturecontrol'
        temperature.mkdir(parents=True)
        (temperature / 'TemperatureControl.cpp').write_text('THEDISPATCHER->add_handler("thermistors", std::bind(Thermistor::print_predefined_thermistors, _1, _2));\n')
        (temperature / 'Thermistor.cpp').write_text('bool Thermistor::print_predefined_thermistors(std::string& params, OutputStream& os) { return true; }\n')
        dro = self.root / 'modules/utils/dro'
        dro.mkdir(parents=True)
        (dro / 'dro.h').write_text('        uint32_t poll_freq;\n')
        lathe = self.root / 'modules/tools/lathe'
        lathe.mkdir(parents=True)
        (lathe / 'Lathe.cpp').write_text('#include "Lathe.h"\n'
                                         '                    while(curindex == index_pulse.load()) {\n'
                                         '                        while(curindex+2 > index_pulse.load()) {\n')
        (lathe / 'timer.cpp').write_text('SlowTicker::getInstance()->attach(1, std::bind(&Lathe::tick, this));\n'
                                        '// SlowTicker::getInstance()->attach(1, std::bind(&Lathe::ignored, this));\n'
                                        'THEDISPATCHER->add_handler("rpm", std::bind( &Lathe::rpm_cmd, this, _1, _2));\n'
                                        '// dispatcher->add_handler("x", std::bind(&Lathe::ignored, this, _1, _2));\n'
                                        'bool Lathe::rpm_cmd(std::string& cmd, OutputStream& os) { return true; }\n')
        display = self.root / 'modules/utils/display/tm1638'
        display.mkdir(parents=True)
        (display / 'TM1638.cpp').write_text('extern "C" uint32_t get_microseconds();\nstatic void delayMicroseconds(uint32_t us)\n{\n    uint32_t t = get_microseconds();\n    uint32_t st= t + us;\n    // allow for wrap\n    if(st >= t) {\n        while(st > get_microseconds()) ;\n    } else {\n        while(st < get_microseconds()) ;\n    }\n}\n')
        graphics = self.root / 'modules/utils/display/st7920'
        graphics.mkdir(parents=True)
        (graphics / 'ST7920.cpp').write_text('        uint32_t t = get_microseconds();\n        uint32_t st= t + us;\n        // allow for wrap\n        if(st >= t) {\n            while(st > get_microseconds()) ;\n        } else {\n            while(st < get_microseconds()) ;\n        }\n')
        player = self.root / 'modules/utils/player'
        player.mkdir(parents=True)
        (player / 'Player.cpp').write_text('#include "Player.h"\n'
                                          '                    abort_command(cmd, nullos);\n\n                    if(!currentfn.empty()) {\n'
                                          '    this->current_file_handler = fopen( this->filename.c_str(), "r");\n'
                                          '    if (stat(filename.c_str(), &buf) >= 0) {\n')

    def run_patch(self):
        return subprocess.run([sys.executable, str(SCRIPT), str(self.root)], capture_output=True, text=True)

    def test_motion_hooks_are_inserted(self):
        result = self.run_patch()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('sim_v2_block_queued()', (self.robot / 'Planner.cpp').read_text())
        self.assertEqual((self.robot / 'Planner.cpp').read_text().count('sim_v2_race_checkpoint(block)'), 2)
        self.assertIn('sim_v2_race_protected_write(block)', (self.robot / 'Planner.cpp').read_text())
        self.assertIn('sim_v2_race_unprotected_write(block)', (self.robot / 'Planner.cpp').read_text())
        self.assertIn('sim_v2_block_started(current_block)', (self.robot / 'StepTicker.cpp').read_text())
        self.assertIn('sim_v2_step_tick_begin()', (self.robot / 'StepTicker.cpp').read_text())
        self.assertIn('sim_v2_step_pulse(this, direction, dir_pin.physical_level())', (self.robot / 'StepperMotor.h').read_text())
        self.assertEqual((self.root / 'Module.cpp').read_text().count('SimV2ModuleScope sim_scope'), 4)
        timer = (self.root / 'modules/tools/lathe/timer.cpp').read_text()
        self.assertIn('sim_v2_named_timer("Lathe::tick", std::bind(&Lathe::tick, this))', timer)
        self.assertNotIn('sim_v2_named_timer("Lathe::ignored"', timer)
        self.assertIn('SimV2HandlerMethodScope sim_handler_scope("Lathe::rpm_cmd")', timer)
        self.assertIn('sim_v2_handler_method_register("Lathe::rpm_cmd");', timer)
        self.assertNotIn('SimV2HandlerMethodScope sim_handler_scope("Lathe::ignored")', timer)
        thermistor = (self.root / 'modules/tools/temperaturecontrol/Thermistor.cpp').read_text()
        registration = (self.root / 'modules/tools/temperaturecontrol/TemperatureControl.cpp').read_text()
        self.assertIn('SimV2HandlerMethodScope sim_handler_scope("Thermistor::print_predefined_thermistors")', thermistor)
        self.assertIn('sim_v2_handler_method_register("Thermistor::print_predefined_thermistors")', registration)

    def test_changed_hook_site_leaves_all_files_unchanged(self):
        path = self.robot / 'StepTicker.cpp'
        path.write_text(path.read_text().replace('if(ok)', 'if (ok)'))
        before = {p: p.read_bytes() for p in self.root.rglob('*.cpp')} | {p: p.read_bytes() for p in self.root.rglob('*.h')}
        result = self.run_patch()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('StepTicker.cpp', result.stderr)
        self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob('*.cpp')} | {p: p.read_bytes() for p in self.root.rglob('*.h')})

    def test_duplicate_hook_site_is_rejected(self):
        path = self.robot / 'StepTicker.cpp'
        path.write_text(path.read_text() + '    current_tick = 0;\n\n    if(ok) {\n')
        before = {p: p.read_bytes() for p in self.robot.iterdir()}
        result = self.run_patch()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('found 2', result.stderr)
        self.assertEqual(before, {p: p.read_bytes() for p in self.robot.iterdir()})

    def test_filament_pointer_fix_rejects_an_unrecognized_firmware(self):
        filament = self.root / 'modules/tools/filamentdetector/FilamentDetector.cpp'
        filament.write_text('different firmware constructor\n')
        planner = self.robot / 'Planner.cpp'
        before = planner.read_bytes()
        result = self.run_patch()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('FilamentDetector.cpp', result.stderr)
        self.assertEqual(planner.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
