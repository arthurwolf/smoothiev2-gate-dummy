"""Exercise shared machine configuration through the ordinary V2 executable."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SIM = os.environ.get('SMOOTHIE_V2_SIM')


@unittest.skipUnless(SIM, 'set SMOOTHIE_V2_SIM to the built V2 simulator')
class MechanicalCLI(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.config = self.root / 'config.ini'
        self.model = self.root / 'simulation.ini'
        self.program = self.root / 'move.gcode'
        self.output = self.root / 'result.json'
        self.base = ('[motion control]\narm_solution = cartesian\ndefault_acceleration = 100\n'
                     '[actuator]\nalpha.step_pin = PA0\nalpha.dir_pin = PA1\nalpha.steps_per_mm = 80\n'
                     'beta.step_pin = PB0\nbeta.dir_pin = PB1\nbeta.steps_per_mm = 80\n'
                     'gamma.step_pin = PC0\ngamma.dir_pin = PC1\ngamma.steps_per_mm = 80\n')
        self.config.write_text(self.base)
        self.model.write_text(self.axes())

    @staticmethod
    def axes(minimum=-200, maximum=200):
        text = '[axis]\n'
        for axis in ('x', 'y', 'z'):
            for key, value in [('steps_per_mm', 80), ('initial_mm', 2), ('minimum_mm', minimum),
                               ('maximum_mm', maximum), ('positive_direction_level', 'false'), ('travel', 'stall')]:
                text += f'{axis}.{key} = {value}\n'
        return text

    @staticmethod
    def switch():
        return ('[switch]\nminx.pin = PD1\nminx.axis = x\nminx.threshold_mm = 0\n'
                'minx.hysteresis_mm = 0.1\nminx.active_level = false\nminx.disconnected_level = true\n')

    @staticmethod
    def probe(active=False, fault='normal'):
        return ('[probe]\nplate.pin = PD1\nplate.offset_mm = 0\nplate.x_slope = 0\nplate.y_slope = 0\n'
                'plate.hysteresis_mm = 0.1\nplate.active_level = ' + str(active).lower() +
                '\nplate.disconnected_level = true\nplate.fault = ' + fault + '\n')

    def run_program(self, program):
        self.program.write_text(program)
        self.output.unlink(missing_ok=True)
        return subprocess.run([SIM, '--modules', '--config', str(self.config), '--machine-model', str(self.model),
                               '--gcode', str(self.program), '--out', str(self.output)],
                              capture_output=True, text=True, timeout=30)

    def successful(self, program):
        result = self.run_program(program)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return json.loads(self.output.read_text())

    def snapshot(self, data):
        rows = []
        for line in data['serial_out'].splitlines():
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if row.get('synthetic') is True and 'axes' in row:
                rows.append(row)
        self.assertTrue(rows, data['serial_out'])
        self.assertTrue(rows[-1]['ok'] and rows[-1]['healthy'], rows[-1])
        return rows[-1]

    def test_physical_motion_stalls_independently_of_firmware_coordinates(self):
        self.model.write_text(self.axes(0, 3))
        data = self.successful('G1 X1 F600\nG1 X2\nG1 X1\nM400\nsim-model status\n')
        state = self.snapshot(data)
        self.assertEqual(data['final_steps'][0], 80)
        self.assertAlmostEqual(state['axes'][0]['physical_mm'], 2)
        self.assertEqual(state['axes'][0]['emitted_pulses'], 240)
        self.assertEqual(state['axes'][0]['stalled_pulses'], 80)
        self.assertEqual(state['axes'][1]['emitted_pulses'], 0)

    def test_firmware_calibration_does_not_recalibrate_physical_mechanics(self):
        data = self.successful('M92 X160\nG1 X1 F600\nM400\nsim-model status\n')
        state = self.snapshot(data)
        self.assertEqual(state['axes'][0]['emitted_pulses'], 160)
        self.assertAlmostEqual(state['axes'][0]['physical_mm'], 4)

    def test_homing_uses_position_contact_and_can_repeat_after_coordinate_reset(self):
        self.config.write_text(self.base + '[endstops]\nminx.enable = true\nminx.pin = PD1!\nminx.axis = X\n'
                               'minx.homing_direction = home_to_min\nminx.max_travel = 5\n'
                               'minx.fast_rate = 30\nminx.slow_rate = 5\nminx.retract = 1\n')
        self.model.write_text(self.axes() + self.switch())
        data = self.successful('G28 X0\nG28 X0\nG28.6\nsim-model status\n')
        state = self.snapshot(data)
        self.assertIn('X:1', data['serial_out'])
        self.assertEqual(data['final_steps'][0], 0)
        self.assertAlmostEqual(state['axes'][0]['physical_mm'], 0, delta=0.25)
        self.assertGreater(state['axes'][0]['emitted_pulses'], 322)

    def test_probes_use_geometry_polarity_and_explicit_faults(self):
        for active, fault in [(False, 'normal'), (True, 'normal'), (False, 'stuck-inactive'),
                               (False, 'stuck-active'), (False, 'disconnected')]:
            with self.subTest(active=active, fault=fault):
                self.config.write_text(self.base + '[zprobe]\nenable = true\nprobe_pin = ' +
                                       ('PD1v' if active else 'PD1!') + '\nslow_feedrate = 5\nmax_travel = 5\n')
                self.model.write_text(self.axes() + self.probe(active, fault))
                data = self.successful('G30 L1\nsim-model status\n')
                state = self.snapshot(data)
                self.assertEqual(state['inputs'][0]['fault'], fault)
                if fault == 'normal':
                    self.assertIn('Z:', data['serial_out'])
                    self.assertAlmostEqual(state['axes'][2]['physical_mm'], 0, delta=0.25)
                elif fault == 'stuck-active':
                    self.assertIn('triggered before move', data['serial_out'])
                    self.assertEqual(state['axes'][2]['emitted_pulses'], 0)
                else:
                    self.assertIn('ZProbe not triggered', data['serial_out'])
                    self.assertAlmostEqual(state['axes'][2]['physical_mm'], -3)

    def test_live_fault_command_changes_only_a_bound_input(self):
        self.config.write_text(self.base + '[zprobe]\nenable = true\nprobe_pin = PD1!\nslow_feedrate = 5\nmax_travel = 5\n')
        self.model.write_text(self.axes() + self.probe())
        data = self.successful('sim-model fault PD1 stuck-active\nG30 L1\nsim-model status\n')
        self.assertIn('triggered before move', data['serial_out'])
        state = self.snapshot(data)
        self.assertEqual(state['inputs'][0]['fault'], 'stuck-active')
        self.assertEqual(state['axes'][2]['emitted_pulses'], 0)

    def test_initial_geometry_overflow_fails_before_commands(self):
        self.config.write_text(self.base + '[zprobe]\nenable = true\nprobe_pin = PD1!\nslow_feedrate = 5\nmax_travel = 5\n')
        self.model.write_text(self.axes() + self.probe().replace('plate.x_slope = 0', 'plate.x_slope = 1e308'))
        result = self.run_program('G1 X1 F600\nM400\n')
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('mechanical', result.stderr.lower())
        self.assertFalse(self.output.exists())

    def test_geometry_overflow_after_motion_fails_without_a_success_result(self):
        self.config.write_text(self.base + '[zprobe]\nenable = true\nprobe_pin = PD1!\nslow_feedrate = 5\nmax_travel = 5\n')
        model = self.axes().replace('x.initial_mm = 2', 'x.initial_mm = 0')
        self.model.write_text(model + self.probe().replace('plate.x_slope = 0', 'plate.x_slope = 1e308'))
        result = self.run_program('G1 X2 F600\nM400\nsim-model status\n')
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('mechanical', result.stderr.lower())
        self.assertFalse(self.output.exists())

    def test_invalid_management_commands_do_not_change_bound_faults(self):
        self.config.write_text(self.base + '[zprobe]\nenable = true\nprobe_pin = PD1!\nslow_feedrate = 5\nmax_travel = 5\n')
        self.model.write_text(self.axes() + self.probe())
        data = self.successful('sim-model fault PD1 disconnected extra\n'
                               'sim-model fault PD3 stuck-active\nsim-model unsupported\nsim-model status\n')
        state = self.snapshot(data)
        self.assertEqual(state['inputs'][0]['fault'], 'normal')
        self.assertEqual(state['axes'][2]['emitted_pulses'], 0)
        errors = [json.loads(line) for line in data['serial_out'].splitlines() if line.startswith('{')]
        self.assertEqual(sum(not row['ok'] for row in errors), 3)

    def test_invalid_models_and_unsupported_bindings_fail_before_commands(self):
        endstop = ('[endstops]\nminx.enable = true\nminx.pin = PD1!\nminx.axis = X\n'
                   'minx.homing_direction = home_to_min\nminx.max_travel = 5\n')
        valid = self.axes() + self.switch()
        for firmware, model in [(self.base.replace('cartesian', 'corexy') + endstop, valid),
                                 (self.base + endstop, valid.replace('x.steps_per_mm = 80', 'x.steps_per_mm = 81')),
                                 (self.base + endstop, valid.replace('minx.pin = PD1', 'minx.pin = PA0')),
                                 (self.base + endstop, valid.replace('minx.pin = PD1', 'minx.pin = PD3')),
                                 (self.base + endstop, valid.replace('minx.axis = x', 'minx.axis = y')),
                                 (self.base + endstop, self.axes()),
                                 (self.base, self.axes() + 'invalid = value\n')]:
            with self.subTest(model=model, firmware=firmware):
                self.config.write_text(firmware)
                self.model.write_text(model)
                result = self.run_program('G1 X1\n')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('mechanical', result.stderr.lower())
                self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
