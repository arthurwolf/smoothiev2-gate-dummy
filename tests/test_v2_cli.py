"""Exercise the staged V2 firmware through its simulator command line."""
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

SIM = os.environ.get('SMOOTHIE_V2_SIM')


@unittest.skipUnless(SIM, 'set SMOOTHIE_V2_SIM to the built version 2 simulator')
class Version2CLI(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.config = self.root / 'config.ini'
        self.config.write_text('[motion control]\narm_solution = cartesian\ndefault_acceleration = 100\n'
                               '[actuator]\nalpha.step_pin = PA0\nalpha.dir_pin = PA1\nalpha.steps_per_mm = 80\n'
                               'beta.step_pin = PB0\nbeta.dir_pin = PB1\nbeta.steps_per_mm = 80\n'
                               'gamma.step_pin = PC0\ngamma.dir_pin = PC1\ngamma.steps_per_mm = 80\n')

    def test_motion_result(self):
        program = self.root / 'move.gcode'
        program.write_text('G1 X1 Y0.5 F600\nG91\nG1 X-0.5 Y0.5\nG90\nG1 X0 Y0\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual([block['gcode_line'] for block in data['blocks']], [1, 3, 5])
        self.assertEqual(data['final_steps'][:2], [0, 0])
        self.assertEqual(data['pulse_position'][:2], [0, 0])
        self.assertEqual(data['blocks'][0]['step_counts'][:2], [80, 40])
        self.assertEqual(data['v2_handler_method_counts']['Robot::handle_dwell'], 0)
        self.assertEqual(data['v2_handler_method_counts']['Robot::handle_motion_command'], 3)

    def test_coupled_axis_kinematics_execute_real_motion(self):
        program = self.root / 'coupled.gcode'
        program.write_text('G1 X2 Y1 Z0.5 F600\n')
        original = self.config.read_text()
        for solution, expected in [('corexy', [240, 80, 40]), ('corexz', [280, 40, 80])]:
            with self.subTest(solution=solution):
                self.config.write_text(original.replace('arm_solution = cartesian', 'arm_solution = ' + solution)
                                       + '\n[corexz]\nx_reduction = 1\nz_reduction = 3\n')
                output = self.root / (solution + '.json')
                result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)],
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr)
                data = json.loads(output.read_text())
                self.assertEqual(data['final_steps'][:3], expected)
                self.assertEqual(data['pulse_position'][:3], expected)

    def test_linear_delta_motion_matches_tower_geometry(self):
        config = self.config.read_text().replace('arm_solution = cartesian',
                                                  'arm_solution = linear_delta\nmust_be_homed = false')
        self.config.write_text(config + '\n[linear delta]\narm_length = 250\narm_radius = 124\n')
        program = self.root / 'delta.gcode'
        program.write_text('G1 X1 Y0 Z0 F600\n')
        output = self.root / 'delta.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(output.read_text())
        self.assertTrue(data['blocks'], data['serial_out'])
        initial_height = math.sqrt(250**2 - 124**2)
        for axis, angle in enumerate((210, 330, 90)):
            x = 124 * math.cos(math.radians(angle))
            y = 124 * math.sin(math.radians(angle))
            height = math.sqrt(250**2 - (1 - x)**2 - y**2)
            self.assertAlmostEqual(data['pulse_position'][axis], (height - initial_height) * 80, delta=1.1)
            self.assertAlmostEqual(data['final_steps'][axis], height * 80, delta=1.1)

    def test_rotary_delta_centered_move_drives_three_arms_equally(self):
        self.config.write_text(self.config.read_text().replace('arm_solution = cartesian',
                               'arm_solution = rotary_delta\nmust_be_homed = false') + '\n[rotary delta]\n')
        program = self.root / 'rotary.gcode'
        program.write_text('G1 Z1 F600\n')
        output = self.root / 'rotary.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(output.read_text())
        self.assertTrue(data['blocks'], data['serial_out'])
        self.assertNotEqual(data['pulse_position'][0], 0)
        self.assertEqual(len(set(data['pulse_position'][:3])), 1)
        self.assertEqual([data['initial_steps'][i] + data['pulse_position'][i] for i in range(3)], data['final_steps'][:3])
        self.assertAlmostEqual(data['machine_position'][2], 1)

    def test_scara_motion_matches_forward_arm_geometry(self):
        self.config.write_text(self.config.read_text().replace('arm_solution = cartesian', 'arm_solution = morgan'))
        program = self.root / 'scara.gcode'
        program.write_text('G1 X1 Y1 F600\n')
        output = self.root / 'scara.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(output.read_text())
        self.assertTrue(data['blocks'], data['serial_out'])
        alpha, beta = (math.radians(steps / 80) for steps in data['final_steps'][:2])
        self.assertAlmostEqual(150 * (math.cos(alpha) + math.cos(beta)) + 100, 1, delta=0.05)
        self.assertAlmostEqual(150 * (math.sin(alpha) + math.sin(beta)) - 60, 1, delta=0.05)
        self.assertTrue(all(data['pulse_position'][:2]))

    def test_delta_checkers_account_for_initial_actuator_position(self):
        root = Path(__file__).resolve().parents[1]
        self.config.write_text(self.config.read_text().replace('arm_solution = cartesian',
                               'arm_solution = linear_delta\nmust_be_homed = false')
                               + '\n[linear delta]\narm_length = 250\narm_radius = 124\n')
        program = self.root / 'delta.gcode'
        program.write_text('G1 X1 Y0 Z0 F600\n')
        for checker in ('gcode_to_blocks.py', 'blocks_to_steps.py'):
            with self.subTest(checker=checker):
                output = self.root / (checker + '.json')
                result = subprocess.run([sys.executable, str(root / 'checkers' / checker), '--sim', SIM,
                                         '--config', str(self.config), '--gcode', str(program),
                                         '--workdir', str(self.root / checker), '--out', str(output)],
                                        capture_output=True, text=True, timeout=30)
                detail = output.read_text() if output.exists() else ''
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr + detail)

    def test_module_checker_rejects_firmware_command_errors(self):
        root = Path(__file__).resolve().parents[1]
        delta_config = self.root / 'delta.ini'
        delta_config.write_text(self.config.read_text().replace('arm_solution = cartesian', 'arm_solution = linear_delta')
                                + '\n[linear delta]\narm_length = 250\narm_radius = 124\n')
        valid = self.root / 'valid.gcode'
        valid.write_text('G4 P1\n')
        rejected = self.root / 'rejected.gcode'
        rejected.write_text('G1 X1 F600\n')
        output = self.root / 'modules.json'
        result = subprocess.run([sys.executable, str(root / 'checkers/modules_check.py'), '--sim', SIM,
                                 '--src', str(self.root), '--config', str(self.config), '--workload', str(valid),
                                 '--extra-run', str(delta_config), str(rejected), '--workdir', str(self.root / 'work'),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        checks = json.loads(output.read_text())['checks']
        self.assertTrue(any(c['status'] == 'fail' and 'Must be homed' in c['detail'] for c in checks))
        result = subprocess.run([sys.executable, str(root / 'checkers/modules_check.py'), '--sim', SIM,
                                 '--src', str(self.root), '--config', str(delta_config), '--workload', str(valid),
                                 '--motion', str(rejected), '--workdir', str(self.root / 'motion'), '--out', str(output)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        checks = json.loads(output.read_text())['checks']
        self.assertTrue(any(c['status'] == 'fail' and 'Must be homed' in c['detail'] for c in checks))

    def test_step_checker_rejects_motion_refused_by_firmware(self):
        root = Path(__file__).resolve().parents[1]
        self.config.write_text(self.config.read_text().replace('arm_solution = cartesian', 'arm_solution = linear_delta')
                               + '\n[linear delta]\narm_length = 250\narm_radius = 124\n')
        program = self.root / 'rejected.gcode'
        program.write_text('G1 X1 F600\n')
        output = self.root / 'steps.json'
        for checker in ('gcode_to_blocks.py', 'blocks_to_steps.py'):
            with self.subTest(checker=checker):
                result = subprocess.run([sys.executable, str(root / 'checkers' / checker), '--sim', SIM,
                                         '--config', str(self.config), '--gcode', str(program),
                                         '--workdir', str(self.root / 'work'), '--out', str(output)],
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                checks = json.loads(output.read_text())['checks']
                self.assertTrue(any(c['status'] == 'fail' and 'Must be homed' in c['detail'] for c in checks))

    def test_motion_checkers_reject_error_after_completed_block(self):
        root = Path(__file__).resolve().parents[1]
        program = self.root / 'partial-error.gcode'
        program.write_text('G1 X1 F600\n;@sim wait 1000\nG2 X2 R1\n')
        output = self.root / 'result.json'
        for checker in ('gcode_to_blocks.py', 'blocks_to_steps.py'):
            with self.subTest(checker=checker):
                result = subprocess.run([sys.executable, str(root / 'checkers' / checker), '--sim', SIM,
                                         '--config', str(self.config), '--gcode', str(program),
                                         '--workdir', str(self.root / checker), '--out', str(output)],
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                report = json.loads(output.read_text())
                self.assertGreater(report['data']['blocks'], 0)
                self.assertTrue(any(c['status'] == 'fail' and 'Radius mode not supported' in c['detail']
                                    for c in report['checks']))

    def test_handler_elapsed_simulated_time(self):
        program = self.root / 'dwell.gcode'
        program.write_text('G4 P10\nG4 P20\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        stats = json.loads(output.read_text())['v2_handler_method_elapsed_us']['Robot::handle_dwell']
        self.assertGreaterEqual(stats['total_us'], 30000)
        self.assertGreaterEqual(stats['max_us'], 20000)

    @unittest.skipUnless(os.environ.get('SMOOTHIE_V2_COVERAGE_SIM'), 'set SMOOTHIE_V2_COVERAGE_SIM to a coverage build')
    def test_checker_collects_coverage_for_recognized_handler(self):
        program = self.root / 'coverage.gcode'
        program.write_text('G1 X1 F600\n;@sim wait 1100\n')
        output = self.root / 'coverage.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers/modules_check.py'
        result = subprocess.run([sys.executable, str(checker), '--firmware-version', '2',
                                 '--sim', os.environ['SMOOTHIE_V2_COVERAGE_SIM'], '--src', str(self.root),
                                 '--config', str(self.config), '--workload', str(program),
                                 '--workdir', str(self.root / 'work'), '--out', str(output)],
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        stats = json.loads(output.read_text())['data']['v2_handler_method_coverage']
        self.assertGreater(stats['Robot::handle_motion_command']['calls'], 0)
        self.assertGreater(stats['Robot::handle_motion_command']['lines_hit'], 0)
        self.assertEqual(stats['Robot::handle_dwell']['calls'], 0)
        timers = json.loads(output.read_text())['data']['v2_timer_callback_coverage']
        self.assertGreater(timers['Robot::periodic_checks']['calls'], 0)
        self.assertEqual(subprocess.run(result.args, capture_output=True, text=True, timeout=60).returncode, 0)
        repeated = json.loads(output.read_text())['data']['v2_handler_method_coverage']
        self.assertEqual(repeated, stats)
        extra_command = result.args + ['--extra-run', str(self.config), str(program)]
        extra = subprocess.run(extra_command, capture_output=True, text=True, timeout=60)
        self.assertEqual(extra.returncode, 0, extra.stdout + extra.stderr)
        merged = json.loads(output.read_text())['data']['v2_handler_method_coverage']
        self.assertEqual(merged['Robot::handle_motion_command']['calls'], 2 * stats['Robot::handle_motion_command']['calls'])

    def test_static_command_handler_in_other_source_is_counted(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                         'hotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n')
        program = self.root / 'thermistors.gcode'
        program.write_text('thermistors\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['v2_handler_method_counts']['Thermistor::print_predefined_thermistors'], 1)
        self.assertIn('Thermistor::print_predefined_thermistors', data['v2_handler_method_elapsed_us'])

    def test_manual_pulse_generator_moves_and_reverses(self):
        with self.config.open('a') as config:
            config.write('[mpg]\nx.enable = true\nx.enca_pin = PD1\nx.encb_pin = PD2\nx.mmperpulse = 0.1\n')
        program = self.root / 'manual-pulse.gcode'
        program.write_text(';@sim pin PD1 1\n;@sim pin PD2 1\n'
                           ';@sim pin PD2 0\n;@sim pin PD1 0\n;@sim pin PD2 1\n;@sim pin PD1 1\n'
                           ';@sim wait 1000\n'
                           ';@sim pin PD1 0\n;@sim pin PD2 0\n;@sim pin PD1 1\n;@sim pin PD2 1\n'
                           ';@sim wait 1000\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 0)
        self.assertEqual([block['step_counts'][0] for block in data['blocks']], [8, 8])
        self.assertGreater(data['v2_module_callback_counts']['mpg::in_command_ctx'], 0)

    def test_shared_manual_pulse_generator_selects_axis(self):
        with self.config.open('a') as config:
            config.write('[mpg]\nshared.enable = true\nshared.enca_pin = PD1\nshared.encb_pin = PD2\n')
        pulse = ';@sim pin PD2 0\n;@sim pin PD1 0\n;@sim pin PD2 1\n;@sim pin PD1 1\n;@sim wait 1000\n'
        program = self.root / 'manual-pulse-shared.gcode'
        program.write_text(';@sim pin PD1 1\n;@sim pin PD2 1\nM922 X0.1\n' + pulse +
                           'M922 Y0.1\n' + pulse + 'M922\n' + pulse)
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][:2], [8, 8])

    def test_manual_pulse_generator_rejects_invalid_configuration(self):
        original = self.config.read_text()
        program = self.root / 'manual-pulse-invalid.gcode'
        program.write_text('')
        cases = ('x.enable = true\nx.enca_pin = PD1\nx.encb_pin = PE1\n',
                 'x.enable = true\nx.enca_pin = PD1\nx.encb_pin = PD2\nshared.enable = true\n')
        for case in cases:
            with self.subTest(case=case):
                self.config.write_text(original + '[mpg]\n' + case)
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_lathe_measures_spindle_speed_from_index_pulses(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nuse_qe = false\nindex_pin = PD1v\nindex_edge = rising\n')
        program = self.root / 'lathe-rpm.gcode'
        program.write_text(';@sim wait 100\n;@sim pin PD1 1\n;@sim pin PD1 0\n'
                           ';@sim wait 1000\n;@sim pin PD1 1\n;@sim wait 200\nrpm\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('60.0', json.loads(output.read_text())['serial_out'])
        self.assertGreater(json.loads(output.read_text())['v2_command_callback_counts']['rpm'], 0)

    def test_lathe_index_synchronized_move(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nuse_qe = false\nindex_pin = PD1v\nindex_edge = rising\n')
        program = self.root / 'lathe-move.gcode'
        program.write_text(';@sim wait 100\n;@sim pin PD1 1\n;@sim pin PD1 0\n'
                           ';@sim wait 1000\n;@sim pin PD1 1\n;@sim wait 200\n'
                           ';@sim pin-after 500 PD1 0\n;@sim pin-after 800 PD1 1\n'
                           ';@sim pin-after 1500 PD1 0\n;@sim pin-after 1800 PD1 1\nG33 Z1 K1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][2], 80)

    def test_lathe_wait_reports_stopped_spindle(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nuse_qe = false\nindex_pin = PD1v\nindex_edge = rising\n')
        program = self.root / 'lathe-stop.gcode'
        program.write_text(';@sim wait 100\n;@sim pin PD1 1\n;@sim pin PD1 0\n'
                           ';@sim wait 1000\n;@sim pin PD1 1\n;@sim wait 200\nG33 Z1 K1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('Spindle stopped running', data['serial_out'])
        self.assertEqual(data['final_steps'][2], 0)

    def test_lathe_manual_encoder_motion_and_scheduled_stop(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nencoder_ppr = 100\n')
        program = self.root / 'lathe-manual.gcode'
        program.write_text(';@sim encoder-after 10 -200\n;@sim stop-after 200\nG33.1 K1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(json.loads(output.read_text())['final_steps'][2], (-80, -79))
        program.write_text('G33.1 K1\n')
        no_stop = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                  '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertNotEqual(no_stop.returncode, 0)
        self.assertIn('scheduled stop', no_stop.stderr)

    def test_lathe_encoder_synchronized_move(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nencoder_ppr = 100\n')
        program = self.root / 'lathe-encoder.gcode'
        program.write_text(';@sim encoder -2\n;@sim wait 200\n;@sim encoder -2\n;@sim wait 150\n'
                           ';@sim encoder-after 10 -200\nG33.1 Z-1 K1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][2], -80)

    def test_lathe_firmware_positive_encoder_move_stops_without_motion(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nencoder_ppr = 100\n')
        program = self.root / 'lathe-positive.gcode'
        program.write_text(';@sim encoder -2\n;@sim wait 200\n;@sim encoder -2\n;@sim wait 150\n'
                           ';@sim encoder-after 10 -200\nG33.1 Z1 K1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][2], 0)

    def test_electronic_leadscrew_displays_speed_and_button_edit(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nuse_qe = false\nindex_pin = PD1v\nindex_edge = rising\n'
                         '[tm1638]\nenable = true\nclock_pin = PD2\ndata_pin = PD3\nstrobe_pin = PD4\n'
                         '[els]\nenable = true\n')
        program = self.root / 'leadscrew.gcode'
        program.write_text(';@sim wait 100\n;@sim pin PD1 1\n;@sim pin PD1 0\n'
                           ';@sim wait 1000\n;@sim pin PD1 1\n;@sim wait 1000\n'
                           ';@sim expect-tm1638 4 125\n;@sim expect-tm1638 6 63\n'
                           ';@sim tm-buttons 128\n;@sim wait 200\n;@sim expect-tm1638 14 6\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads((self.root / 'result.json').read_text())
        self.assertGreater(data['v2_timer_callback_counts']['ELS::update_rpm'], 0)
        self.assertGreater(data['v2_timer_callback_elapsed_us']['ELS::update_rpm']['max_us'], 0)

    def test_graphical_display_driver_transfers_pixels(self):
        with self.config.open('a') as config:
            config.write('[st7920]\nenable = true\nclk = PD5\nmosi = PD6\n')
        program = self.root / 'graphics.gcode'
        program.write_text(';@sim graphics-init\n;@sim graphics-pixel 0 0 1\n'
                           ';@sim graphics-pixel 127 63 1\n;@sim graphics-refresh\n'
                           ';@sim expect-graphics 0 0 1\n;@sim expect-graphics 127 63 1\n'
                           ';@sim expect-graphics 1 0 0\n;@sim graphics-pixel 0 0 0\n'
                           ';@sim graphics-refresh\n;@sim expect-graphics 0 0 0\n'
                           ';@sim expect-graphics 127 63 1\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_graphical_display_rejects_invalid_input(self):
        program = self.root / 'invalid-graphics.gcode'
        program.write_text(';@sim graphics-init\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)
        with self.config.open('a') as config:
            config.write('[st7920]\nenable = true\nclk = PD5\nmosi = PD5\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)
        self.config.write_text(self.config.read_text().replace('mosi = PD5', 'mosi = PD6'))
        program.write_text(';@sim graphics-pixel 0 0 1\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)

    def test_electronic_leadscrew_and_digital_readout_coexist(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nuse_qe = false\nindex_pin = PD1v\nindex_edge = rising\n'
                         '[tm1638]\nenable = true\nclock_pin = PD2\ndata_pin = PD3\nstrobe_pin = PD4\n'
                         '[els]\nenable = true\n[max7219]\nenable = true\nclk = PD5\nmosi = PD6\n'
                         '[dro]\ncommon.poll_frequency_hz = 10\nx.cs_pin = PD7\n'
                         '[st7920]\nenable = true\nclk = PE5\nmosi = PE6\n')
        program = self.root / 'combined-displays.gcode'
        program.write_text(';@sim graphics-init\n;@sim graphics-pixel 1 33 1\n;@sim graphics-refresh\n'
                           'G1 X1 F600\n;@sim wait 100\n;@sim pin PD1 1\n;@sim pin PD1 0\n'
                           ';@sim wait 1000\n;@sim pin PD1 1\n;@sim wait 1000\n'
                           ';@sim expect-tm1638 4 125\n;@sim expect-tm1638 6 63\n'
                           ';@sim expect-display PD7 0 4 129\n;@sim expect-graphics 1 33 1\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_electronic_leadscrew_button_starts_and_stops_manual_move(self):
        with self.config.open('a') as config:
            config.write('[lathe]\nenable = true\nencoder_ppr = 100\n'
                         '[tm1638]\nenable = true\nclock_pin = PD2\ndata_pin = PD3\nstrobe_pin = PD4\n'
                         '[els]\nenable = true\n')
        program = self.root / 'leadscrew-run.gcode'
        program.write_text(';@sim tm-buttons 128\n;@sim wait 200\n;@sim tm-buttons 0\n;@sim wait 200\n'
                           ';@sim encoder-after 300 -200\n;@sim stop-after 600\n'
                           ';@sim tm-buttons 2\n;@sim wait 1000\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(json.loads((self.root / 'result.json').read_text())['final_steps'][2], (-8, -7))

    def test_electronic_leadscrew_rejects_missing_devices_and_bad_pins(self):
        original = self.config.read_text()
        program = self.root / 'leadscrew-invalid.gcode'
        program.write_text('')
        cases = ('[els]\nenable = true\n',
                 '[lathe]\nenable = true\n[els]\nenable = true\n',
                 '[tm1638]\nenable = true\nclock_pin = PD2\ndata_pin = PD2\nstrobe_pin = PD4\n')
        for case in cases:
            with self.subTest(case=case):
                self.config.write_text(original + case)
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_button_box_dispatches_press_and_release_commands(self):
        with self.config.open('a') as config:
            config.write('[button box]\nmove.pin = PD1\nmove.press = G1 X1 F600\n'
                         'move.release = G1 X2 F600\n')
        program = self.root / 'buttons.gcode'
        program.write_text(';@sim pin PD1 1\n;@sim wait 100\n;@sim pin PD1 0\n;@sim wait 100\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 160)
        self.assertEqual([block['step_counts'][0] for block in data['blocks']], [80, 80])

    def test_external_button_dispatches_press_and_release_commands(self):
        with self.config.open('a') as config:
            config.write('[button box]\nmove.pin = external\nmove.press = G1 X1 F600\n'
                         'move.release = G1 X2 F600\n')
        program = self.root / 'external-buttons.gcode'
        program.write_text(';@sim button move 1\n;@sim wait 100\n;@sim button move 0\n;@sim wait 100\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 160)
        self.assertEqual([block['step_counts'][0] for block in data['blocks']], [80, 80])
        for directive in (';@sim button unknown 1\n', ';@sim button move 2\n'):
            program.write_text(directive)
            result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                     '--out', str(output)], capture_output=True, text=True, timeout=20)
            self.assertNotEqual(result.returncode, 0, directive)

    def test_button_box_rejects_invalid_inputs_and_unhandled_actions(self):
        original = self.config.read_text()
        program = self.root / 'buttons-invalid.gcode'
        output = self.root / 'result.json'
        program.write_text(';@sim pin PD1 1\n;@sim wait 100\n')
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)]
        cases = (
            'move.pin = PD1\nmove.press = G1 X1\nother.pin = PD.1\nother.press = G1 X2\n',
            'common.poll_frequency_hz = 0\nmove.pin = PD1\nmove.press = G1 X1\n',
            'move.pin = PD1\nmove.press = ' + 'X' * 132 + '\n',
            'move.pin = PD1\nmove.press = unsupported\n',
            'move.pin = PD1\nmove.press = M32\n',
        )
        for case in cases:
            with self.subTest(case=case[:45]):
                self.config.write_text(original + '[button box]\n' + case)
                result = subprocess.run(command, capture_output=True, text=True, timeout=20)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_button_queued_firmware_error_is_not_silent(self):
        with self.config.open('a') as config:
            config.write('[button box]\nmove.pin = external\nmove.press = G2 X1 R1 F600\n')
        program = self.root / 'button-error.gcode'
        program.write_text(';@sim button move 1\n;@sim wait 100\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Radius mode not supported', result.stderr)

    def test_button_box_can_start_a_file_from_the_simulated_sd_card(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[button box]\nstart.pin = PD1\nstart.press = M32 job.gcode\n')
        (self.root / 'sd').mkdir()
        (self.root / 'sd' / 'job.gcode').write_text('G1 X1 F600\n')
        program = self.root / 'button-play.gcode'
        program.write_text(';@sim pin PD1 1\n;@sim wait 150\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--sd-root', str(self.root / 'sd'), '--out', str(output)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)

    def test_inverted_button_box_input_uses_physical_pin_level(self):
        with self.config.open('a') as config:
            config.write('[button box]\nmove.pin = PD1!\nmove.press = G1 X1 F600\n')
        program = self.root / 'inverted-button.gcode'
        program.write_text(';@sim pin PD1 1\n;@sim wait 100\n;@sim pin PD1 0\n;@sim wait 100\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)

    def test_button_box_retries_when_command_queue_is_full(self):
        with self.config.open('a') as config:
            config.write('[button box]\n')
            for index in range(11):
                config.write(f'b{index:02}.pin = PE{index}\nb{index:02}.press = G1 X{index + 1} F600\n')
        program = self.root / 'many-buttons.gcode'
        program.write_text(''.join(f';@sim pin PE{index} 1\n' for index in range(11)) + ';@sim wait 150\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 880)
        self.assertEqual(len(data['blocks']), 11)

    def test_button_box_kill_interrupts_motion(self):
        with self.config.open('a') as config:
            config.write('[button box]\nstop.pin = PD1v\nstop.press = KILL\n')
        program = self.root / 'stop-button.gcode'
        program.write_text(';@sim pin-after 500 PD1 1\nG1 X100 F600\n;@sim wait 1000\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        position = json.loads(output.read_text())['final_steps'][0]
        self.assertGreater(position, 0)
        self.assertLess(position, 8000)

    def test_digital_switch_outputs_can_be_checked_in_workload(self):
        with self.config.open('a') as config:
            config.write('[switch]\nfan.enable = true\nfan.output_pin = PD1\nfan.output_type = digital\n'
                         'fan.input_on_command = M106\nfan.input_off_command = M107\n')
        program = self.root / 'switch-output.gcode'
        output = self.root / 'result.json'
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)]
        program.write_text('M106\n;@sim expect-pin PD1 1\nM107\n;@sim expect-pin PD1 0\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        program.write_text('M106\n;@sim expect-pin PD1 0\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        program.write_text(';@sim expect-pin invalid 0\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.config.write_text(self.config.read_text().replace('fan.output_pin = PD1', 'fan.output_pin = PD1!'))
        program.write_text('M106\n;@sim expect-pin PD1 0\nM107\n;@sim expect-pin PD1 1\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_drilling_cycle_runs_firmware_moves_and_retracts(self):
        with self.config.open('a') as config:
            config.write('[drillingcycles]\nenable = true\n')
        program = self.root / 'drill.gcode'
        program.write_text('G1 Z3 F600\nG99\nG81 X1 Y2 Z-2 R1 F300\nG80\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][:3], [80, 160, 240])
        self.assertTrue(any(block['step_counts'][2] == 240 for block in data['blocks']))

    def test_peck_drilling_and_relative_mode_rejection(self):
        with self.config.open('a') as config:
            config.write('[drillingcycles]\nenable = true\n')
        program = self.root / 'peck.gcode'
        output = self.root / 'result.json'
        program.write_text('G1 Z3 F600\nG99\nG83 X1 Z-2 R1 Q1 F300\nG80\n')
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][:3], [80, 0, 240])
        self.assertGreaterEqual(sum(block['step_counts'][2] != 0 for block in data['blocks']), 6)
        program.write_text('G99\nG91\nG81 X1 Z-2 R1 F300\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)

    def test_race_mode_preempts_planner_and_preserves_steps(self):
        with self.config.open('a') as config:
            config.write('[conveyor]\nqueue_delay_time_ms = 0\n')
        program = self.root / 'race.gcode'
        program.write_text(''.join(f'G1 X{i / 10:.1f} F600\n' for i in range(1, 31)))
        output = self.root / 'race.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output),
                                 '--race-force-every', '1'], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        match = re.search(r'SIM_V2_RACE points=(\d+) fired=(\d+) target_started=(\d+) protected_writes_while_ticking=(\d+)', result.stderr)
        self.assertIsNotNone(match, result.stderr)
        self.assertGreater(int(match.group(1)), 0)
        self.assertGreater(int(match.group(2)), 0)
        self.assertGreater(int(match.group(3)), 0)
        self.assertEqual(int(match.group(4)), 0)
        rate_write = re.search(r'unprotected_rate_writes_while_ticking=(\d+)', result.stderr)
        self.assertIsNotNone(rate_write, result.stderr)
        self.assertGreater(int(rate_write.group(1)), 0)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 240)
        self.assertEqual(data['pulse_position'][0], 240)

    def test_race_mode_explores_conveyor_shared_flag_writes(self):
        with self.config.open('a') as config:
            config.write('[conveyor]\nqueue_delay_time_ms = 0\n')
        program = self.root / 'conveyor-race.gcode'
        program.write_text(''.join(f'G1 X{i / 10:.1f} F600\n' for i in range(1, 31)))
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'conveyor-race.json'), '--race-force-every', '1'],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        match = re.search(r'conveyor_flag_writes=(\d+) conveyor_flag_interrupts=(\d+)', result.stderr)
        self.assertIsNotNone(match, result.stderr)
        self.assertGreater(int(match.group(1)), 0)
        self.assertGreater(int(match.group(2)), 0)

    def test_race_window_runs_only_near_due_step_interrupts(self):
        with self.config.open('a') as config:
            config.write('[conveyor]\nqueue_delay_time_ms = 0\n')
        program = self.root / 'race-window.gcode'
        program.write_text(''.join(f'G1 X{i / 10:.1f} F600\n' for i in range(1, 31)))
        command = [SIM, '--config', str(self.config), '--gcode', str(program),
                   '--out', str(self.root / 'race-window.json'), '--race-window-us']
        result = subprocess.run(command + ['100'], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        match = re.search(r'SIM_V2_RACE points=(\d+) fired=(\d+)', result.stderr)
        self.assertIsNotNone(match, result.stderr)
        self.assertGreater(int(match.group(2)), 0)
        for invalid in ('-1', 'nan', 'inf', 'bad'):
            with self.subTest(invalid=invalid):
                result = subprocess.run(command + [invalid], capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)

    def test_firmware_command_wait_is_recorded_without_driver_waits(self):
        program = self.root / 'dwell.gcode'
        program.write_text(';@sim wait 10\nG4 P20\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        waits = json.loads(output.read_text())['waits']
        self.assertEqual([(w['kind'], w['module'], w['event'], w['us'], w['gcode_line']) for w in waits],
                         [('safe_sleep', 'command context', 'G4', 20000, 2)])
        self.assertGreater(json.loads(output.read_text())['v2_command_callback_counts']['G4'], 0)

    def test_nested_drilling_dwell_records_inner_command_context(self):
        with self.config.open('a') as config:
            config.write('[drillingcycles]\nenable = true\ndwell_units = P\n')
        program = self.root / 'drill-dwell.gcode'
        program.write_text('G99\nG82 X1 Z-1 R1 P20 F300\nG80\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        waits = json.loads(output.read_text())['waits']
        self.assertTrue(any(w['kind'] == 'safe_sleep' and w['event'] == 'G4' and w['us'] == 20000 for w in waits))

    def test_race_mode_rejects_invalid_interval(self):
        program = self.root / 'empty.gcode'
        program.write_text('')
        for interval in ('0', '-1', 'bad'):
            with self.subTest(interval=interval):
                result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(self.root / 'race.json'), '--race-force-every', interval],
                                        capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)

    def test_bad_input_fails(self):
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(self.root / 'missing')], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)

    def test_unwritable_output_fails(self):
        program = self.root / 'empty.gcode'
        program.write_text('')
        output = self.root / 'missing' / 'result.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)

    def test_unwritable_spi_trace_fails(self):
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json'),
                                 '--spi-trace', str(self.root / 'missing' / 'spi.json')],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('SPI trace', result.stderr)

    def test_unwritable_motor_trace_fails(self):
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json'),
                                 '--motor-trace', str(self.root / 'missing' / 'motors.json')],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('motor trace', result.stderr)

    def test_replays_planned_blocks(self):
        blocks = self.root / 'blocks.txt'
        blocks.write_text('3 80 40 0 0 10 800 100 1.118034 0 0\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--blocks-in', str(blocks), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['blocks'][0]['step_counts'][:2], [80, 40])
        self.assertEqual(data['final_steps'][:2], [80, 40])

    def test_steps_checker_runs_on_gcode(self):
        program = self.root / 'move.gcode'
        program.write_text('G1 X1 Y0.5 F600\nG1 X2 Y1\nG1 X0 Y0\n')
        output = self.root / 'check.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers' / 'blocks_to_steps.py'
        result = subprocess.run([sys.executable, str(checker), '--sim', SIM, '--config', str(self.config), '--gcode', str(program), '--workdir', str(self.root / 'work'), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        checks = json.loads(output.read_text())['checks']
        self.assertNotIn('fail', [check['status'] for check in checks])

    def test_blocks_checker_runs_on_gcode(self):
        program = self.root / 'move.gcode'
        program.write_text('G1 X1 Y0.5 F600\nG1 X2 Y1\n')
        output = self.root / 'check.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers' / 'gcode_to_blocks.py'
        result = subprocess.run([sys.executable, str(checker), '--sim', SIM, '--config', str(self.config), '--gcode', str(program), '--cartesian', '--workdir', str(self.root / 'work'), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_module_checker_does_not_claim_unmeasured_v2_timing_passes(self):
        self.config.write_text(self.config.read_text().replace('[motion control]', '[motion control] ; trailing text'))
        program = self.root / 'module-workload.gcode'
        program.write_text('G1 X1 F600\nG4 P20\n')
        output = self.root / 'modules.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers' / 'modules_check.py'
        result = subprocess.run([sys.executable, str(checker), '--sim', SIM, '--src', str(self.root),
                                 '--config', str(self.config), '--workload', str(program),
                                 '--workdir', str(self.root / 'work'), '--out', str(output)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        checks = {check['check']: check for check in json.loads(output.read_text())['checks']}
        for name in ('handlers did not block while running the workload', 'no handler ran longer than threshold', 'module coverage'):
            self.assertEqual(checks[name]['status'], 'warn')
            self.assertIn('not measured', checks[name]['detail'])
        waits = json.loads(output.read_text())['data']['runtime_waits']
        self.assertTrue(any(wait['event'] == 'G4' and wait['us'] == 20000 for wait in waits))

    def test_module_checker_keeps_observed_forbidden_wait_failure(self):
        root = Path(__file__).resolve().parents[1]
        program = self.root / 'forbidden-wait.gcode'
        program.write_text('G4 P20\n')
        rules = json.loads((root / 'rules.json').read_text())
        rules['v2_calls']['safe_sleep'] = {'status': 'forbidden', 'reason': 'test rule'}
        rules_path = self.root / 'rules.json'
        rules_path.write_text(json.dumps(rules))
        output = self.root / 'modules.json'
        result = subprocess.run([sys.executable, str(root / 'checkers/modules_check.py'), '--sim', SIM,
                                 '--src', str(self.root), '--config', str(self.config), '--workload', str(program),
                                 '--rules', str(rules_path), '--workdir', str(self.root / 'work'), '--out', str(output)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        checks = {check['check']: check for check in json.loads(output.read_text())['checks']}
        observed = checks['handlers did not block while running the workload']
        self.assertEqual(observed['status'], 'fail')
        self.assertIn('safe_sleep', observed['detail'])
        self.assertIn('not measured', observed['detail'])

    def test_v2_static_checker_identifies_registered_command_handler_paths(self):
        source = self.root / 'firmware'
        modules = source / 'src' / 'modules'
        modules.mkdir(parents=True)
        (modules / 'sample.cpp').write_text(
            'void Sample::configure()\n{\n    add_handler(1, std::bind(&Sample::handle, this));\n}\n'
            'void Sample::handle()\n{\n    helper();\n}\n'
            'void Sample::helper()\n{\n    safe_sleep(10);\n}\n')
        output = self.root / 'static.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers' / 'modules_check.py'
        result = subprocess.run([sys.executable, str(checker), '--static-only', '--firmware-version', '2',
                                 '--src', str(source), '--workdir', str(self.root / 'work'), '--out', str(output)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        hits = json.loads(output.read_text())['data']['static_hits']
        self.assertEqual([(hit['call'], hit['context'], hit['function']) for hit in hits],
                         [('safe_sleep', 'handler', 'helper')])

    def test_v2_static_checker_reports_firmware_sleep_calls(self):
        source = self.root / 'firmware'
        modules = source / 'src' / 'modules'
        modules.mkdir(parents=True)
        (modules / 'sample.cpp').write_text('void Sample::handler()\n{\n    safe_sleep(10);\n    vTaskDelay(1);\n}\n')
        output = self.root / 'static.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers' / 'modules_check.py'
        result = subprocess.run([sys.executable, str(checker), '--static-only', '--firmware-version', '2',
                                 '--src', str(source), '--workdir', str(self.root / 'work'), '--out', str(output)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(output.read_text())
        self.assertEqual({hit['call'] for hit in report['data']['static_hits']}, {'safe_sleep', 'vTaskDelay'})
        self.assertTrue(all(hit['severity'] == 'warn' for hit in report['data']['static_hits']))
        auto_command = [sys.executable, str(checker), '--static-only', '--config', str(self.config),
                        '--src', str(source), '--workdir', str(self.root / 'work'), '--out', str(output)]
        result = subprocess.run(auto_command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(json.loads(output.read_text())['data']['static_hits']), 2)
        self.config.write_text('alpha_steps_per_mm 80\n')
        result = subprocess.run(auto_command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['data']['static_hits'], [])

    @unittest.skipUnless(os.environ.get('SMOOTHIE_V2_COVERAGE_SIM'), 'set SMOOTHIE_V2_COVERAGE_SIM to a coverage build')
    def test_report_uses_separate_uninstrumented_memory_simulator(self):
        root = Path(__file__).resolve().parents[1]
        layout = self.root / 'layout.json'
        layout.write_text(json.dumps({'architecture': 'stm32h745', 'type_sizes': {},
                                      'main_sram': {'pool_start': 0x24001000, 'pool_end': 0x24080000},
                                      'dtcm': {'pool_start': 0x20001000, 'pool_end': 0x2001f000},
                                      'sram1': {'pool_start': 0x30001000, 'pool_end': 0x30020000}}))
        out = self.root / 'separate-memory-report'
        env = dict(os.environ, V2_MEMORY_SIM=SIM, STM32_LAYOUT=str(layout))
        result = subprocess.run(['bash', str(root / 'run_checks_v2.sh'), os.environ['SMOOTHIE_V2_COVERAGE_SIM'],
                                 str(self.root), '-', str(out)], env=env, capture_output=True, text=True, timeout=90)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        memory = json.loads((out / 'memory.result.json').read_text())
        self.assertTrue(memory['data']['measured'])
        self.assertFalse(any(c['check'] == 'coverage instrumentation' for c in memory['checks']))
        modules = json.loads((out / 'modules.result.json').read_text())
        self.assertTrue(modules['data']['v2_handler_method_coverage'])

    def test_version2_report_workflow(self):
        root = Path(__file__).resolve().parents[1]
        out = self.root / 'report'
        env = dict(os.environ, V2_CONFIG=str(root / 'cases/v2/config.ini'),
                   V2_MOTION=str(root / 'cases/v2/motion.gcode'), V2_WORKLOAD=str(root / 'cases/v2/modules.gcode'))
        env.pop('STM32_LAYOUT', None)
        result = subprocess.run(['bash', str(root / 'run_checks_v2.sh'), SIM, str(self.root), '-', str(out)],
                                env=env, capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((out / 'report.html').is_file())
        memory = json.loads((out / 'memory.result.json').read_text())
        self.assertFalse(memory['data']['measured'])
        modules = json.loads((out / 'modules.result.json').read_text())
        self.assertEqual(modules['data']['runtime_instrumentation'], 'not measured')
        self.assertTrue(modules['data']['v2_command_callback_counts'])
        self.assertTrue(modules['data']['v2_timer_callback_counts'])
        self.assertTrue(modules['data']['v2_handler_callback_counts'])
        self.assertTrue(modules['data']['v2_handler_method_counts'])
        checker = root / 'checkers/modules_check.py'
        combined = self.root / 'combined-modules.json'
        config = root / 'cases/v2/config.ini'
        workload = root / 'cases/v2/modules.gcode'
        result = subprocess.run([sys.executable, str(checker), '--firmware-version', '2', '--sim', SIM,
                                 '--src', str(self.root), '--config', str(config), '--workload', str(workload),
                                 '--extra-run', str(config), str(workload), '--workdir', str(self.root / 'combined-work'),
                                 '--out', str(combined)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        merged = json.loads(combined.read_text())['data']['v2_command_callback_counts']
        for name, count in modules['data']['v2_command_callback_counts'].items():
            self.assertEqual(merged[name], 2 * count)
        merged_timers = json.loads(combined.read_text())['data']['v2_timer_callback_counts']
        for name, count in modules['data']['v2_timer_callback_counts'].items():
            self.assertEqual(merged_timers[name], 2 * count)
        merged_handlers = json.loads(combined.read_text())['data']['v2_handler_callback_counts']
        for name, count in modules['data']['v2_handler_callback_counts'].items():
            self.assertEqual(merged_handlers[name], 2 * count)
        merged_methods = json.loads(combined.read_text())['data']['v2_handler_method_counts']
        for name, count in modules['data']['v2_handler_method_counts'].items():
            self.assertEqual(merged_methods[name], 2 * count)
        merged_elapsed = json.loads(combined.read_text())['data']['v2_handler_method_elapsed_us']
        for name, stat in modules['data']['v2_handler_method_elapsed_us'].items():
            self.assertEqual(merged_elapsed[name]['total_us'], 2 * stat['total_us'])
            self.assertEqual(merged_elapsed[name]['max_us'], stat['max_us'])

    def test_prime_board_supplies_default_motor_pins(self):
        config = self.config.read_text()
        config = '\n'.join(line for line in config.splitlines() if '.step_pin' not in line and '.dir_pin' not in line) + '\n'
        self.config.write_text(config)
        program = self.root / 'move.gcode'
        program.write_text('G1 X1 F600\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)

    def test_configured_trinamic_driver_is_not_silently_treated_as_external(self):
        original = self.config.read_text()
        program = self.root / 'move.gcode'
        program.write_text('G1 X1 F600\n')
        for setting in ('alpha.driver = tmc2590', 'alpha.driver = tmc2660', 'alpha.slaved_to = X'):
            with self.subTest(setting=setting):
                self.config.write_text(original.replace('alpha.step_pin = PA0', setting + '\nalpha.step_pin = PA0'))
                result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('actuator', result.stderr)

    def test_trinamic_slave_receives_steps_and_current(self):
        self.config.write_text(self.config.read_text().replace('alpha.step_pin = PA0',
                               'alpha.driver = tmc2590\nalpha.step_pin = PA0') +
                               'delta.driver = tmc2590\ndelta.slaved_to = alpha\n'
                               'delta.step_pin = PD0\ndelta.dir_pin = PD1!\n'
                               '[tmc2590]\nalpha.spi_cs_pin = PD5\ndelta.spi_cs_pin = PD6\n'
                               '[current control]\nalpha.current = 1.2\n')
        program = self.root / 'slave.gcode'
        program.write_text('M906 X800\nG1 X1 F600\n')
        output, spi, motors = self.root / 'result.json', self.root / 'spi.json', self.root / 'motors.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output), '--spi-trace', str(spi), '--motor-trace', str(motors)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)
        self.assertEqual(json.loads(motors.read_text()), {'0': 80, '3': -80})
        writes = json.loads(spi.read_text())
        for pin in ('PD5', 'PD6'):
            currents = [entry['word'] for entry in writes if entry['pin'] == pin and entry['word'] >> 17 == 6]
            self.assertGreaterEqual(len(set(currents)), 2)

    def test_trinamic_slave_can_home_separately_after_primary(self):
        self.config.write_text(self.config.read_text().replace('alpha.step_pin = PA0',
                               'alpha.driver = tmc2590\nalpha.step_pin = PA0') +
                               'delta.driver = tmc2590\ndelta.slaved_to = alpha\n'
                               'delta.step_pin = PD0\ndelta.dir_pin = PD1\n'
                               '[tmc2590]\nalpha.spi_cs_pin = PD5\ndelta.spi_cs_pin = PD6\n'
                               '[endstops]\nminx.enable = true\nminx.pin = PD2v\nminx.axis = X\n'
                               'minx.homing_direction = home_to_min\nminx.max_travel = 100\n'
                               'minx.fast_rate = 30\nminx.slow_rate = 5\nminx.retract = 1\n'
                               'slave.enable = true\nslave.pin = PD3v\nslave.axis = A\n')
        program = self.root / 'slave-home.gcode'
        program.write_text(';@sim pin-after 500 PD2 1\n;@sim pin-after 700 PD2 0\n'
                           ';@sim pin-after 1300 PD2 1\nG28 X0\n'
                           ';@sim pin-after 200 PD3 1\nG28.7 X0\nG28.6\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        serial = json.loads(output.read_text())['serial_out']
        self.assertIn('X:1', serial)
        self.assertRegex(serial, r'Actuator 3, moved [1-9][0-9]* steps')
        self.assertNotIn('error:', serial)

    def test_invalid_trinamic_slave_pairings_fail(self):
        original = self.config.read_text()
        program = self.root / 'slave-invalid.gcode'
        program.write_text('G1 X1 F600\n')
        cases = (
            ('external master', '', 'delta.driver = tmc2590\ndelta.slaved_to = alpha\n'),
            ('unknown master', 'alpha.driver = tmc2590\n', 'delta.driver = tmc2590\ndelta.slaved_to = omega\n'),
            ('different driver', 'alpha.driver = tmc2660\n', 'delta.driver = tmc2590\ndelta.slaved_to = alpha\n'),
            ('different microsteps', 'alpha.driver = tmc2590\nalpha.microsteps = 16\n', 'delta.driver = tmc2590\ndelta.slaved_to = alpha\n'),
        )
        for name, master, slave in cases:
            with self.subTest(name=name):
                self.config.write_text(original.replace('alpha.step_pin = PA0', master + 'alpha.step_pin = PA0') +
                                       slave + 'delta.step_pin = PD0\ndelta.dir_pin = PD1\n'
                                       '[tmc2590]\nalpha.spi_cs_pin = PD5\ndelta.spi_cs_pin = PD6\n'
                                       '[tmc2660]\nalpha.spi_cs_pin = PD5\n')
                result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('actuator', result.stderr)

    def test_unconnected_slave_motor_is_not_silently_ignored(self):
        self.config.write_text(self.config.read_text().replace('alpha.step_pin = PA0',
                               'alpha.driver = tmc2590\nalpha.step_pin = PA0') +
                               'delta.driver = tmc2590\ndelta.slaved_to = alpha\n'
                               'delta.step_pin = invalid\ndelta.dir_pin = PD1\n'
                               '[tmc2590]\nalpha.spi_cs_pin = PD5\ndelta.spi_cs_pin = PD6\n')
        program = self.root / 'invalid-slave.gcode'
        program.write_text('G1 X1 F600\n')
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_trinamic_motor_current_commands_use_firmware_driver(self):
        original = self.config.read_text()
        program = self.root / 'current.gcode'
        program.write_text('M907\nM906 X800\nM907\n')
        output = self.root / 'result.json'
        trace = self.root / 'spi.json'
        for driver in ('tmc2590', 'tmc2660'):
            with self.subTest(driver=driver):
                self.config.write_text(original.replace('alpha.step_pin = PA0',
                                       f'alpha.driver = {driver}\nalpha.step_pin = PA0') +
                                       f'[{driver}]\nalpha.spi_cs_pin = PD5\n'
                                       '[current control]\nalpha.current = 1.2\n')
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(output), '--spi-trace', str(trace)], capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertNotIn('send20bits failed', result.stdout)
                self.assertNotIn('send262 failed', result.stdout)
                serial = json.loads(output.read_text())['serial_out']
                self.assertIn('alpha: 1.20000 Amps', serial)
                self.assertIn('alpha: 0.80000 Amps', serial)
                writes = json.loads(trace.read_text())
                current_registers = [write['word'] for write in writes if write['pin'] == 'PD5' and write['word'] >> 17 == 6]
                self.assertGreaterEqual(len(set(current_registers)), 2)

    def test_trinamic_and_thermocouple_require_compatible_spi_channels(self):
        self.config.write_text(self.config.read_text().replace('alpha.step_pin = PA0',
                               'alpha.driver = tmc2590\nalpha.step_pin = PA0') +
                               '[tmc2590]\nalpha.spi_cs_pin = PD5\n'
                               '[current control]\nalpha.current = 1.2\n'
                               '[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                               'hotend.sensor = max31855\nhotend.spi_channel = 0\nhotend.spi_select_pin = PD2\n')
        program = self.root / 'shared-devices.gcode'
        program.write_text(';@sim thermocouple PD2 250\n;@sim wait 500\nM907\nM105\n')
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                   '--out', str(self.root / 'result.json')]
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('alpha: 1.20000 Amps', json.loads((self.root / 'result.json').read_text())['serial_out'])
        self.config.write_text(self.config.read_text().replace('hotend.spi_channel = 0', 'hotend.spi_channel = 1'))
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)

    @unittest.skipUnless(os.environ.get('SMOOTHIE_V2_DEVEBOX_SIM'), 'set SMOOTHIE_V2_DEVEBOX_SIM to the DeveBox simulator')
    def test_devebox_trinamic_uses_board_default_spi_channel(self):
        self.config.write_text(self.config.read_text().replace('alpha.step_pin = PA0',
                               'alpha.driver = tmc2590\nalpha.step_pin = PA0') +
                               '[tmc2590]\nalpha.spi_cs_pin = PD5\n'
                               '[current control]\nalpha.current = 1.2\n'
                               '[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                               'hotend.sensor = max31855\nhotend.spi_channel = 1\nhotend.spi_select_pin = PD2\n')
        program = self.root / 'devebox-spi.gcode'
        program.write_text(';@sim thermocouple PD2 250\n;@sim wait 500\nM907\nM105\n')
        command = [os.environ['SMOOTHIE_V2_DEVEBOX_SIM'], '--modules', '--config', str(self.config),
                   '--gcode', str(program), '--out', str(self.root / 'result.json')]
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('alpha: 1.20000 Amps', json.loads((self.root / 'result.json').read_text())['serial_out'])
        self.config.write_text(self.config.read_text().replace('hotend.spi_channel = 1', 'hotend.spi_channel = 0'))
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)

    def test_trinamic_shared_bus_selects_each_motor(self):
        self.config.write_text(self.config.read_text().replace('alpha.step_pin = PA0',
                               'alpha.driver = tmc2590\nalpha.step_pin = PA0').replace('beta.step_pin = PB0',
                               'beta.driver = tmc2590\nbeta.step_pin = PB0') +
                               '[tmc2590]\nalpha.spi_cs_pin = PD5\nbeta.spi_cs_pin = PD6\n'
                               '[current control]\nalpha.current = 1.2\nbeta.current = 1.0\n')
        program = self.root / 'two-motors.gcode'
        program.write_text('M906 X800\nM906 Y600\nM907\n')
        output, trace = self.root / 'result.json', self.root / 'spi.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output), '--spi-trace', str(trace)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        serial = json.loads(output.read_text())['serial_out']
        self.assertIn('alpha: 0.80000 Amps', serial)
        self.assertIn('beta: 0.60000 Amps', serial)
        self.assertEqual({event['pin'] for event in json.loads(trace.read_text())}, {'PD5', 'PD6'})
        self.assertNotIn('send20bits failed', result.stdout)

    def test_temperature_fault_halts_motion(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.tool_id = 0\n'
                         'hotend.heater_pin = PD0\nhotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n')
        program = self.root / 'fault.gcode'
        program.write_text('M104 S200\nG1 X1 F600\n;@sim adc 0\n;@sim wait 1100\nG1 X2\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('MINTEMP or MAXTEMP triggered', data['serial_out'])
        self.assertEqual(len(data['blocks']), 1)
        program.write_text('M104 S200\nG1 X1 F600\n;@sim adc 3900\n;@sim wait 1100\nG1 X2\n')
        control = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(control.returncode, 0, control.stdout + control.stderr)
        data = json.loads(output.read_text())
        self.assertNotIn('MINTEMP or MAXTEMP triggered', data['serial_out'])
        self.assertEqual(len(data['blocks']), 2)

    def test_initial_adc_option_controls_temperature_fault(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.tool_id = 0\n'
                         'hotend.heater_pin = PD0\nhotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n')
        program = self.root / 'initial-adc.gcode'
        program.write_text('M104 S200\n;@sim wait 1100\nG1 X1 F600\n')
        output = self.root / 'initial-adc.json'
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output), '--adc-value']
        fault = subprocess.run(command + ['0'], capture_output=True, text=True, timeout=20)
        self.assertEqual(fault.returncode, 0, fault.stdout + fault.stderr)
        self.assertIn('MINTEMP or MAXTEMP triggered', json.loads(output.read_text())['serial_out'])
        normal = subprocess.run(command + ['3900'], capture_output=True, text=True, timeout=20)
        self.assertEqual(normal.returncode, 0, normal.stdout + normal.stderr)
        self.assertNotIn('MINTEMP or MAXTEMP triggered', json.loads(output.read_text())['serial_out'])
        for invalid in ('-1', '4096', 'bad'):
            with self.subTest(invalid=invalid):
                self.assertNotEqual(subprocess.run(command + [invalid], capture_output=True, text=True, timeout=10).returncode, 0)

    def test_rejects_partial_temperature_configuration(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.tool_id = 0\n'
                         'hotend.heater_pin = PD0\nhotend.thermistor_pin = ADC1_0\n'
                         'broken.enable = true\nbroken.thermistor_pin = nc\n')
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(self.root / 'result.json')],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)

        self.assertIn('temperature configuration failed', result.stderr)

    def test_rejects_invalid_temperature_adc(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                         'hotend.thermistor_pin = invalid\n')
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(self.root / 'result.json')],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)

    def test_rejects_duplicate_temperature_adc(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                         'hotend.thermistor_pin = ADC1_0\nbed.enable = true\n'
                         'bed.heater_pin = PD1\nbed.thermistor_pin = ADC1_0\n')
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(self.root / 'result.json')],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.config.write_text(self.config.read_text().replace('bed.thermistor_pin = ADC1_0', 'bed.thermistor_pin = ADC1_1'))
        control = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(self.root / 'result.json')],
                                 capture_output=True, text=True, timeout=10)
        self.assertEqual(control.returncode, 0, control.stderr)

    def test_temperature_adc_values_are_independent(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                         'hotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n'
                         'bed.enable = true\nbed.heater_pin = PD1\nbed.thermistor_pin = ADC1_1\n'
                         'bed.max_temp = 150\n')
        program = self.root / 'two-temperatures.gcode'
        program.write_text(';@sim adc 3900\n;@sim adc ADC1_1 0\nM140 S70\n;@sim wait 1100\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('MINTEMP or MAXTEMP triggered on B', json.loads(output.read_text())['serial_out'])
        program.write_text(';@sim adc 3900\n;@sim adc ADC1_1 3900\nM140 S70\n;@sim wait 1100\n')
        control = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                  '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(control.returncode, 0, control.stdout + control.stderr)
        self.assertNotIn('MINTEMP or MAXTEMP triggered', json.loads(output.read_text())['serial_out'])

    def test_max31855_thermocouple_fault_and_control(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                         'hotend.sensor = max31855\nhotend.spi_channel = 0\nhotend.spi_select_pin = PD2\n'
                         'hotend.max_temp = 300\n')
        program = self.root / 'thermocouple.gcode'
        output = self.root / 'result.json'
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)]
        program.write_text(';@sim thermocouple PD2 400\nM104 S200\n;@sim wait 1500\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('MINTEMP or MAXTEMP triggered', json.loads(output.read_text())['serial_out'])
        program.write_text(';@sim thermocouple PD2 25\nM104 S200\n;@sim wait 1500\n')
        control = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(control.returncode, 0, control.stdout + control.stderr)
        self.assertNotIn('MINTEMP or MAXTEMP triggered', json.loads(output.read_text())['serial_out'])

    def test_max31855_shared_bus_selects_the_configured_sensor(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                         'hotend.sensor = max31855\nhotend.spi_channel = 0\nhotend.spi_select_pin = PD2\n'
                         'hotend.max_temp = 300\nbed.enable = true\nbed.heater_pin = PD1\n'
                         'bed.sensor = max31855\nbed.spi_channel = 0\nbed.spi_select_pin = PD3\n'
                         'bed.max_temp = 150\n')
        program = self.root / 'two-thermocouples.gcode'
        program.write_text(';@sim thermocouple PD2 25\n;@sim thermocouple PD3 250\nM140 S70\n;@sim wait 1500\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('MINTEMP or MAXTEMP triggered on B', json.loads(output.read_text())['serial_out'])

    def test_max31855_electrical_faults_are_reported_and_halt_wait(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD0\n'
                         'hotend.sensor = max31855\nhotend.spi_channel = 0\nhotend.spi_select_pin = PD2\n'
                         'hotend.max_temp = 300\n')
        program = self.root / 'open-thermocouple.gcode'
        output = self.root / 'result.json'
        for fault, text in (('open', 'open circuit'), ('short-gnd', 'short to ground'), ('short-vcc', 'short to Vcc')):
            with self.subTest(fault=fault):
                program.write_text(f';@sim thermocouple PD2 {fault}\n;@sim wait 500\nM305\nM109 S200\n')
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(output)], capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                serial = json.loads(output.read_text())['serial_out']
                self.assertIn(text, serial)
                self.assertIn('Temperature reading is unreliable', serial)

    def test_temperature_switch_loads_with_its_output(self):
        with self.config.open('a') as config:
            config.write('[switch]\nfan.enable = true\nfan.output_pin = PD1\nfan.output_type = digital\n'
                         'fan.input_on_command = M106\nfan.input_off_command = M107\n'
                         '[temperature control]\nhotend.enable = true\nhotend.tool_id = 0\n'
                         'hotend.heater_pin = PD0\nhotend.thermistor_pin = ADC1_0\n'
                         '[temperature switch]\nhotend.enable = true\nhotend.designator = T\n'
                         'hotend.switch = fan\nhotend.threshold_temp = 50\n')
        program = self.root / 'temperature-switch.gcode'
        program.write_text(';@sim adc 3900\n;@sim wait 1100\nG1 X1 F600\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)

    def test_laser_module_loads_and_rejects_invalid_pwm(self):
        with self.config.open('a') as config:
            config.write('[laser]\nenable = true\npwm_pin = PWM1_1\n'
                         'proportional_power_frequency = 1000\n')
        program = self.root / 'laser.gcode'
        program.write_text('G1 X1 F600 S0.5\n')
        output = self.root / 'result.json'
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)
        self.config.write_text(self.config.read_text().replace('proportional_power_frequency = 1000', 'proportional_power_frequency = 1'))
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.config.write_text(self.config.read_text().replace('PWM1_1', 'invalid'))
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)

    def test_extruder_moves_and_retracts(self):
        with self.config.open('a') as config:
            config.write('delta.step_pin = PD2\ndelta.dir_pin = PD3\ndelta.steps_per_mm = 100\n'
                         '[extruder]\nhotend.enable = true\nhotend.tool_id = 0\n')
        program = self.root / 'extrude.gcode'
        program.write_text('G1 X1 E2 F600\nG1 E1 F600\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][:4], [80, 0, 0, 100])
        self.config.write_text('\n'.join(line for line in self.config.read_text().splitlines() if not line.startswith('delta.')) + '\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)

    def test_player_executes_a_file_through_the_firmware_command_queue(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        played = self.root / 'played.gcode'
        played.write_text('; start of job\n\n' + ''.join(f'G1 X{i} F600\n' for i in range(1, 21)))
        program = self.root / 'player.gcode'
        program.write_text(f'play {played}\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 1600)
        self.assertEqual(len(data['blocks']), 20)

    def test_player_dispatches_console_command_from_file(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD1\n'
                         'hotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n')
        played = self.root / 'console.gcode'
        played.write_text('thermistors\nG1 X1 F600\n')
        program = self.root / 'player.gcode'
        program.write_text(f'play {played}\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['v2_handler_method_counts']['Thermistor::print_predefined_thermistors'], 1)
        self.assertEqual(data['final_steps'][0], 80)

    def test_player_queued_firmware_error_is_not_silent(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        played = self.root / 'rejected.gcode'
        played.write_text('G2 X1 R1 F600\n')
        program = self.root / 'player.gcode'
        program.write_text(f'play {played}\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Radius mode not supported', result.stderr)

    def test_player_selects_and_runs_a_file_from_a_simulated_sd_card(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        (self.root / 'sd').mkdir()
        (self.root / 'sd' / 'job.gcode').write_text('G1 X1 F600\n')
        program = self.root / 'player.gcode'
        output = self.root / 'result.json'
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                   '--sd-root', str(self.root / 'sd'), '--out', str(output)]
        for body in ('M23 job.gcode\nM24\n', 'M32 job.gcode\n'):
            program.write_text(body)
            result = subprocess.run(command, capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)
        outside = self.root / 'outside.gcode'
        outside.write_text('G1 X10 F600\n')
        (self.root / 'sd' / 'escape.gcode').symlink_to(outside)
        for name in ('../outside.gcode', 'escape.gcode'):
            program.write_text(f'M32 {name}\n')
            result = subprocess.run(command, capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            data = json.loads(output.read_text())
            self.assertIn('file.open failed', data['serial_out'])
            self.assertEqual(data['final_steps'][0], 0)

    def test_player_reports_sd_initialization_and_file_progress(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        (self.root / 'sd').mkdir()
        (self.root / 'sd' / 'job.gcode').write_text('G1 X1 F600\n')
        program = self.root / 'player.gcode'
        program.write_text('M21\nM23 job.gcode\nM27\nM24\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--sd-root', str(self.root / 'sd'), '--out', str(output)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('SD card ok', data['serial_out'])
        self.assertIn('SD printing byte', data['serial_out'])
        self.assertEqual(data['final_steps'][0], 80)

    def test_player_runs_configured_startup_file_before_workload(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = true\non_boot_gcode = /sd/boot.gcode\n'
                         '[switch]\nfan.enable = true\nfan.output_pin = PD1\nfan.output_type = digital\n'
                         'fan.input_on_command = M106\nfan.input_off_command = M107\n')
        (self.root / 'sd').mkdir()
        (self.root / 'sd' / 'boot.gcode').write_text('M106\n')
        program = self.root / 'startup.gcode'
        program.write_text(';@sim expect-pin PD1 1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--sd-root', str(self.root / 'sd'), '--out', str(output)],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_player_can_start_paused_and_resume(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[switch]\nfan.enable = true\nfan.output_pin = PD1\nfan.output_type = digital\n'
                         'fan.input_on_command = M106\nfan.input_off_command = M107\n')
        played = self.root / 'paused.gcode'
        played.write_text('M106\nG1 X1 F600\n')
        program = self.root / 'player.gcode'
        program.write_text(f'play {played} -p\n;@sim wait 250\n;@sim expect-pin PD1 0\n'
                           'M24\n;@sim wait 250\n;@sim expect-pin PD1 1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 80)
        delay = data['v2_task_delays']['PlayThread']
        self.assertGreaterEqual(delay['calls'], 1)
        self.assertGreaterEqual(delay['max_us'], 200000)

    def test_player_yields_during_a_long_file_without_losing_commands(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        played = self.root / 'long.gcode'
        played.write_text(''.join(f'G1 X{1 if i % 2 == 0 else 2} F600\n' for i in range(120)))
        program = self.root / 'player.gcode'
        program.write_text(f'play {played}\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(len(data['blocks']), 120)
        self.assertEqual(data['final_steps'][0], 160)

    def test_player_suspends_and_resumes_without_filament_detector(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        program = self.root / 'player.gcode'
        program.write_text('M600\n;@sim wait 100\nM601\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        serial = json.loads(output.read_text())['serial_out']
        self.assertIn('// action:pause', serial)
        self.assertIn('// action:resume', serial)
        program.write_text('M22\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('file playback is unsupported', result.stderr)
        self.config.write_text(self.config.read_text().replace('on_boot_gcode_enable = false', 'on_boot_gcode_enable = true'))
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('on_boot_gcode_enable', result.stderr)

    def test_player_handles_empty_and_missing_files_then_restarts(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        empty = self.root / 'empty.gcode'
        empty.write_text('')
        played = self.root / 'played-again.gcode'
        played.write_text('G1 X1 F600\n')
        program = self.root / 'player.gcode'
        program.write_text(f'play {empty}\nplay {self.root / "missing.gcode"}\nplay {played}\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('File not found:', data['serial_out'])
        self.assertEqual(data['final_steps'][0], 80)

    def test_player_reset_restarts_a_file(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        played = self.root / 'reset.gcode'
        played.write_text('G1 X1 F600\nG1 X1 F600\n')
        program = self.root / 'player.gcode'
        output = self.root / 'result.json'
        for pause in ('', 'M25\n'):
            with self.subTest(paused=bool(pause)):
                program.write_text(f'G91\nplay {played}\n{pause}M26\n')
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(output)], capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(json.loads(output.read_text())['final_steps'][0], 160)

    def test_player_reset_without_a_file(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        program = self.root / 'player.gcode'
        program.write_text('M26\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('No file loaded', data['serial_out'])
        self.assertEqual(data['final_steps'][0], 0)

    def test_player_reports_progress_and_aborts_a_paused_file(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        played = self.root / 'abort.gcode'
        played.write_text('G1 X10 F600\n')
        program = self.root / 'player.gcode'
        program.write_text(f'play {played} -p\nprogress\nabort\n;@sim wait 250\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['final_steps'][0], 0)
        self.assertIn('SD print is paused', data['serial_out'])

    def test_player_can_pause_a_running_file(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[switch]\nfan.enable = true\nfan.output_pin = PD1\nfan.output_type = digital\n'
                         'fan.input_on_command = M106\nfan.input_off_command = M107\n')
        played = self.root / 'pause-running.gcode'
        played.write_text('M106\nM107\nG1 X1 F600\n')
        program = self.root / 'player.gcode'
        program.write_text(f'play {played}\nM25\n;@sim wait 250\n;@sim expect-pin PD1 1\n'
                           'M24\n;@sim wait 250\n;@sim expect-pin PD1 0\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(output.read_text())['final_steps'][0], 80)

    def test_filament_switch_suspends_and_resumes_firmware(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[filament detector]\nenable = true\ndetector_pin = PD2\n')
        program = self.root / 'filament.gcode'
        program.write_text(';@sim pin PD2 1\n;@sim wait 100\nM407\n'
                           ';@sim pin PD2 0\nM601\nM407\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        serial = json.loads(output.read_text())['serial_out']
        self.assertIn('Filament detector has been triggered', serial)
        self.assertIn('system is suspended', serial)
        self.assertIn('// action:pause', serial)
        self.assertIn('// action:resume', serial)

    def test_filament_player_rejects_unsupported_file_playback(self):
        with self.config.open('a') as config:
            config.write('[filament detector]\nenable = true\ndetector_pin = PD2\n')
        program = self.root / 'filament.gcode'
        program.write_text('M22\n')
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                   '--out', str(self.root / 'result.json')]
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('on_boot_gcode_enable', result.stderr)
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('file playback is unsupported', result.stderr)
        program.write_text('M32\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('file playback is unsupported', result.stderr)

    def test_filament_switch_configuration_loads(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[filament detector]\nenable = true\ndetector_pin = PD2\n')
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_filament_bulge_input_triggers_firmware_alarm(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[filament detector]\nenable = true\nbulge_pin = PD2\n')
        program = self.root / 'bulge.gcode'
        output = self.root / 'result.json'
        program.write_text(';@sim pin PD2 1\n;@sim wait 100\nM407\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('bulge in the filament', json.loads(output.read_text())['serial_out'])
        program.write_text(';@sim wait 100\nM407\n')
        control = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                  '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(control.returncode, 0, control.stdout + control.stderr)
        self.assertNotIn('bulge in the filament', json.loads(output.read_text())['serial_out'])

    def test_filament_encoder_detects_missing_pulses(self):
        with self.config.open('a') as config:
            config.write('delta.step_pin = PD3\ndelta.dir_pin = PD4\ndelta.steps_per_mm = 100\n'
                         '[extruder]\nhotend.enable = true\nhotend.tool_id = 0\n'
                         '[player]\non_boot_gcode_enable = false\n'
                         '[filament detector]\nenable = true\nencoder_pin = PD2v\nseconds_per_check = 1\n')
        program = self.root / 'filament-encoder.gcode'
        output = self.root / 'result.json'
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)]
        program.write_text(';@sim wait 1100\nG1 E2 F600\n;@sim wait 1100\nM407\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Filament detector has been triggered', json.loads(output.read_text())['serial_out'])
        program.write_text(';@sim wait 1100\n;@sim pin PD2 1\nG1 E2 F600\n;@sim wait 1100\nM407\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn('Filament detector has been triggered', json.loads(output.read_text())['serial_out'])

    def test_filament_encoder_accepts_pulse_workload_directive(self):
        with self.config.open('a') as config:
            config.write('[player]\non_boot_gcode_enable = false\n'
                         '[filament detector]\nenable = true\nencoder_pin = PD2v\nseconds_per_check = 1\n')
        program = self.root / 'encoder-pulses.gcode'
        output = self.root / 'encoder-pulses.json'
        command = [SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)]
        program.write_text(';@sim pulses PD2 3\nM407\n')
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Encoder pulses: 3', json.loads(output.read_text())['serial_out'])
        for directive in ('pulses PD2 -1', 'pulses PA15 1', 'pulses PD2 invalid'):
            with self.subTest(directive=directive):
                program.write_text(';@sim ' + directive + '\n')
                self.assertNotEqual(subprocess.run(command, capture_output=True, text=True, timeout=20).returncode, 0)

    def test_endstop_hard_limit_halts_motion(self):
        with self.config.open('a') as config:
            config.write('[endstops]\ncommon.debounce_ms = 0\nminx.enable = true\n'
                         'minx.pin = PD1v\nminx.axis = X\nminx.limit_enable = true\n')
        program = self.root / 'limit.gcode'
        program.write_text(';@sim pin PD1 1\nG1 X100 F600\n;@sim wait 100\nG1 X120\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('ALARM: Hard limit', data['serial_out'])
        self.assertLess(data['final_steps'][0], 8000)

    def test_homing_with_untriggered_endstop_finishes(self):
        with self.config.open('a') as config:
            config.write('[endstops]\nminx.enable = true\nminx.pin = PD1v\nminx.axis = X\n'
                         'minx.homing_direction = home_to_min\nminx.max_travel = 1\n'
                         'minx.fast_rate = 30\nminx.slow_rate = 5\n')
        program = self.root / 'home.gcode'
        program.write_text('G28 X0\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertGreater(len(json.loads(output.read_text())['blocks']), 0)

    def test_scheduled_input_changes_during_motion(self):
        with self.config.open('a') as config:
            config.write('[endstops]\nminx.enable = true\nminx.pin = PD1v\nminx.axis = X\nminx.limit_enable = true\n')
        program = self.root / 'scheduled-limit.gcode'
        program.write_text(';@sim pin-after 500 PD1 1\nG1 X100 F600\n;@sim wait 1000\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('ALARM: Hard limit', data['serial_out'])
        self.assertGreater(data['final_steps'][0], 0)
        self.assertLess(data['final_steps'][0], 8000)

    def test_homing_uses_triggered_endstop(self):
        with self.config.open('a') as config:
            config.write('[endstops]\nminx.enable = true\nminx.pin = PD1v\nminx.axis = X\n'
                         'minx.homing_direction = home_to_min\nminx.max_travel = 100\n'
                         'minx.fast_rate = 30\nminx.slow_rate = 5\nminx.retract = 1\n')
        program = self.root / 'triggered-home.gcode'
        program.write_text(';@sim pin-after 500 PD1 1\n;@sim pin-after 700 PD1 0\n'
                           ';@sim pin-after 1300 PD1 1\nG28 X0\nG28.6\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('X:1', data['serial_out'])
        self.assertGreater(len(data['blocks']), 1)

    def test_z_probe_without_strategy_rejects_leveling_without_crashing(self):
        with self.config.open('a') as config:
            config.write('[zprobe]\nenable = true\nprobe_pin = PD1v\n')
        program = self.root / 'missing-strategy.gcode'
        output = self.root / 'result.json'
        for command in ('G29', 'G29 P0', 'G29 P1'):
            with self.subTest(command=command):
                program.write_text(command + '\n')
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(output)], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn('unhandled G-code', result.stderr)

    def test_z_probe_rejects_invalid_strategy_configuration(self):
        original = self.config.read_text()
        program = self.root / 'invalid-strategy.gcode'
        program.write_text('')
        for setting in ('leveling = three point', 'leveling = unknown', 'calibration = delta', 'calibration = unknown'):
            with self.subTest(setting=setting):
                self.config.write_text(original + '[zprobe]\nenable = true\nprobe_pin = PD1v\n' + setting + '\n')
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_z_probe_dispatches_configured_leveling_strategy(self):
        with self.config.open('a') as config:
            config.write('[zprobe]\nenable = true\nprobe_pin = PD1v\nleveling = three point\n'
                         '[three point leveling strategy]\nhome_first = false\n')
        program = self.root / 'configured-strategy.gcode'
        program.write_text('G31 P0\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Bed leveling plane is not set', json.loads(output.read_text())['serial_out'])

    def test_z_probe_stops_on_scheduled_input(self):
        with self.config.open('a') as config:
            config.write('[zprobe]\nenable = true\nprobe_pin = PD1v\nslow_feedrate = 5\nmax_travel = 10\n')
        program = self.root / 'probe.gcode'
        program.write_text(';@sim pin-after 500 PD1 1\nG30 L1\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('Z:', data['serial_out'])
        self.assertLess(data['final_steps'][2], 0)
        program.write_text('G30 L1\n')
        control = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(control.returncode, 0, control.stdout + control.stderr)
        self.assertIn('ZProbe not triggered', json.loads(output.read_text())['serial_out'])
        program.write_text(';@sim pin PD1 1\nG30 L1\n')
        control = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(control.returncode, 0, control.stdout + control.stderr)
        self.assertIn('triggered before move', json.loads(output.read_text())['serial_out'])

    def test_allocation_trace_records_firmware_allocations(self):
        program = self.root / 'trace.gcode'
        program.write_text('G1 X1 F600\n')
        trace = self.root / 'alloc.trace'
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--config', str(self.config), '--gcode', str(program), '--out', str(output), '--alloc-trace', str(trace)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        lines = trace.read_text().splitlines()
        self.assertTrue(lines[0].startswith('B '))
        self.assertTrue(any(line.startswith('a ') for line in lines))
        self.assertTrue(any(line.startswith('a ') and line.split()[3] == '1' for line in lines), 'DTCM pool allocation missing')

    def test_stm32_memory_checker_replays_distinct_pools(self):
        program = self.root / 'memory.gcode'
        program.write_text('G1 X1 F600\n')
        layout = self.root / 'layout.json'
        layout.write_text(json.dumps({'architecture': 'stm32h745', 'type_sizes': {},
                                      'main_sram': {'pool_start': 0x24001000, 'pool_end': 0x24080000},
                                      'dtcm': {'pool_start': 0x20001000, 'pool_end': 0x2001f000},
                                      'sram1': {'pool_start': 0x30001000, 'pool_end': 0x30020000}}))
        output = self.root / 'memory.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers' / 'memory_check.py'
        result = subprocess.run([sys.executable, str(checker), '--sim', SIM, '--config', str(self.config), '--gcode', str(program), '--layout', str(layout), '--workdir', str(self.root / 'memory-work'), '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())['data']
        self.assertEqual(data['architecture'], 'stm32h745')
        self.assertGreater(data['dtcm']['peak_live_bytes'], 0)
        self.assertEqual(data['sram1']['peak_live_bytes'], 0)
        with self.config.open('a') as config:
            config.write('[current control]\nalpha.current = 1.2\n')
        unsupported = subprocess.run([sys.executable, str(checker), '--sim', SIM, '--config', str(self.config),
                                      '--gcode', str(program), '--layout', str(layout),
                                      '--workdir', str(self.root / 'unsupported-work'), '--out', str(output)],
                                     capture_output=True, text=True, timeout=30)
        self.assertNotEqual(unsupported.returncode, 0)
        self.assertIn('current control', unsupported.stderr)

    def test_fault_checker_uses_version2_temperature_configuration(self):
        with self.config.open('a') as config:
            config.write('[temperature control] ; trailing text\nhotend.enable = true\nhotend.tool_id = 0\n'
                         'hotend.heater_pin = PD0\nhotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n')
        output = self.root / 'faults.json'
        checker = Path(__file__).resolve().parents[1] / 'checkers' / 'fault_check.py'
        result = subprocess.run([sys.executable, str(checker), '--sim', SIM, '--config', str(self.config), '--workdir', str(self.root / 'work'), '--out', str(output)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual({check['status'] for check in json.loads(output.read_text())['checks']}, {'pass'})

    def test_enabled_unsupported_module_is_reported(self):
        with self.config.open('a') as config:
            config.write('[network]\nenable = true\n')
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('network', result.stderr)

    def test_disabled_digital_readout_does_not_reject_axis_settings(self):
        with self.config.open('a') as config:
            config.write('[dro]\ncommon.enable = false\nx.cs_pin = PD1\n')
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.config.write_text(self.config.read_text().replace('common.enable = false', 'common.enable = true'))
        enabled = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                  '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(enabled.returncode, 0)
        self.assertIn('dro.x', enabled.stderr)
        self.config.write_text(self.config.read_text().replace('x.cs_pin = PD1\n', ''))
        no_axes = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                  '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
        self.assertEqual(no_axes.returncode, 0, no_axes.stdout + no_axes.stderr)

    def test_digital_readout_displays_firmware_machine_position(self):
        with self.config.open('a') as config:
            config.write('[max7219]\nenable = true\nclk = PD1\nmosi = PD2\n'
                         '[dro]\ncommon.enable = true\ncommon.poll_frequency_hz = 10\nx.cs_pin = PD3\n')
        program = self.root / 'display.gcode'
        program.write_text('G1 X1 F600\n;@sim wait 2000\n;@sim expect-display PD3 0 4 129\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_digital_readout_displays_cascaded_axes(self):
        with self.config.open('a') as config:
            config.write('[max7219]\nenable = true\nclk = PD1\nmosi = PD2\ncs = PD3\ncascaded = 2\n'
                         '[dro]\ncommon.poll_frequency_hz = 10\nx.id = 1\ny.id = 0\n')
        program = self.root / 'display-cascade.gcode'
        program.write_text('G1 X1 F600\n;@sim wait 2000\n'
                           ';@sim expect-display PD3 1 4 129\n;@sim expect-display PD3 0 4 128\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        program.write_text(';@sim wait 200\n;@sim expect-display PD3 2 4 0\n')
        invalid = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                  '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertNotEqual(invalid.returncode, 0)
        program.write_text(';@sim wait 200\n;@sim expect-display PD3 1 4 1\n')
        wrong = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertNotEqual(wrong.returncode, 0)

    def test_digital_readout_uses_default_poll_frequency(self):
        with self.config.open('a') as config:
            config.write('[max7219]\nenable = true\nclk = PD1\nmosi = PD2\n'
                         '[dro]\nx.cs_pin = PD3\n')
        program = self.root / 'display-default.gcode'
        program.write_text(';@sim wait 200\n;@sim expect-display PD3 0 4 128\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_digital_readout_rejects_invalid_display_configuration(self):
        original = self.config.read_text()
        program = self.root / 'display-invalid.gcode'
        program.write_text(';@sim wait 100\n')
        cases = (
            '[max7219]\nenable = true\nclk = PD1\nmosi = PD1\n[dro]\nx.cs_pin = PD3\n',
            '[max7219]\nenable = true\nclk = PD1\nmosi = PD2\ncs = PD3\ncascaded = 2\n[dro]\nx.id = 2\n',
            '[max7219]\nenable = true\nclk = PD1\nmosi = PD2\ncs = PD3\ncascaded = 2\n[dro]\nx.id = 0\ny.id = 0\n',
            '[max7219]\nenable = true\nclk = PD1\nmosi = PD2\n[dro]\ncommon.poll_frequency_hz = 0\nx.cs_pin = PD3\n',
        )
        for case in cases:
            with self.subTest(case=case):
                self.config.write_text(original + case)
                result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                         '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=30)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_current_control_is_not_silently_ignored(self):
        with self.config.open('a') as config:
            config.write('[current control]\nalpha.current = 1.2\n')
        program = self.root / 'empty.gcode'
        program.write_text('')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(self.root / 'result.json')],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('current control', result.stderr)
        self.config.write_text(self.config.read_text().replace('alpha.current = 1.2', 'alpha.current = 0'))
        control = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(self.root / 'result.json')],
                                 capture_output=True, text=True, timeout=10)
        self.assertEqual(control.returncode, 0, control.stderr)

    def test_kill_button_halts_motion(self):
        with self.config.open('a') as config:
            config.write('[kill button]\nenable = true\npin = PD1\n')
        program = self.root / 'kill.gcode'
        program.write_text(';@sim pin PD1 1\n;@sim pin-after 500 PD1 0\nG1 X100 F600\n;@sim wait 1000\nG1 X120\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertIn('Kill button pressed', data['serial_out'])
        self.assertLess(data['final_steps'][0], 8000)
        self.assertGreater(data['v2_module_callback_counts']['killbutton::on_halt'], 0)

    def test_digital_switch_accepts_its_commands(self):
        with self.config.open('a') as config:
            config.write('[switch]\nfan.enable = true\nfan.output_pin = PD0\nfan.output_type = digital\n'
                         'fan.input_on_command = M106\nfan.input_off_command = M107\n')
        program = self.root / 'switch.gcode'
        program.write_text('M106\nM107\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertGreater(json.loads(output.read_text())['v2_handler_callback_counts']['Switch::M106'], 0)
        self.assertEqual(json.loads(output.read_text())['v2_handler_method_counts']['Switch::handle_gcode'], 2)

    def test_input_switches_respect_startup_pulls_and_external_drive(self):
        with self.config.open('a') as config:
            config.write('[switch]\nup.enable = true\nup.input_pin = PD0\n'
                         'down.enable = true\ndown.input_pin = PD1v\n')
        program = self.root / 'input-pulls.gcode'
        program.write_text(';@sim expect-pin PD0 1\n;@sim expect-pin PD1 0\n'
                           ';@sim pin PD0 0\n;@sim expect-pin PD0 0\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_input_switch_dispatches_commands_on_pin_changes(self):
        with self.config.open('a') as config:
            config.write('[switch]\nbutton.enable = true\nbutton.input_pin = PD0v\n'
                         'button.output_on_command = G1_X1_F600\nbutton.output_off_command = G1_X0\n')
        program = self.root / 'button.gcode'
        program.write_text(';@sim pin PD0 1\n;@sim wait 50\n;@sim pin PD0 0\n;@sim wait 50\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(len(data['blocks']), 2)
        self.assertEqual(data['final_steps'][0], 0)

    def test_switch_callback_records_nested_command_elapsed_time(self):
        with self.config.open('a') as config:
            config.write('[switch]\nbutton.enable = true\nbutton.input_pin = PD1v\n'
                         'button.input_pin_behavior = momentary\nbutton.output_on_command = G4_P20\n')
        program = self.root / 'switch-dwell.gcode'
        program.write_text(';@sim pin PD1 1\n;@sim wait 100\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertTrue(any(w['event'] == 'G4' and w['us'] == 20000 for w in data['waits']))
        self.assertGreaterEqual(data['v2_module_callback_elapsed_us']['switch::in_command_ctx']['max_us'], 20000)

    def test_nested_switch_console_command_runs_firmware_handler(self):
        with self.config.open('a') as config:
            config.write('[temperature control]\nhotend.enable = true\nhotend.heater_pin = PD1\n'
                         'hotend.thermistor_pin = ADC1_0\nhotend.max_temp = 300\n'
                         '[switch]\nbutton.enable = true\nbutton.input_pin = PD0v\n'
                         'button.output_on_command = thermistors\n')
        program = self.root / 'switch-console.gcode'
        program.write_text(';@sim pin PD0 1\n;@sim wait 100\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(output)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        data = json.loads(output.read_text())
        self.assertEqual(data['v2_handler_method_counts']['Thermistor::print_predefined_thermistors'], 1)

    def test_unhandled_nested_switch_console_command_fails(self):
        with self.config.open('a') as config:
            config.write('[switch]\nbutton.enable = true\nbutton.input_pin = PD0v\n'
                         'button.output_on_command = invalidconsolecommand\n')
        program = self.root / 'switch-unknown-console.gcode'
        program.write_text(';@sim pin PD0 1\n;@sim wait 100\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('invalidconsolecommand', result.stderr)

    def test_unhandled_nested_switch_gcode_fails(self):
        with self.config.open('a') as config:
            config.write('[switch]\nbutton.enable = true\nbutton.input_pin = PD0v\n'
                         'button.output_on_command = G9999\n')
        program = self.root / 'switch-unknown-gcode.gcode'
        program.write_text(';@sim pin PD0 1\n;@sim wait 100\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('G9999', result.stderr)

    def test_nested_switch_command_error_is_not_silent(self):
        with self.config.open('a') as config:
            config.write('[switch]\nbutton.enable = true\nbutton.input_pin = PD0v\n'
                         'button.output_on_command = G2_X1_R1_F600\n')
        program = self.root / 'switch-error.gcode'
        program.write_text(';@sim pin PD0 1\n;@sim wait 100\n')
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program),
                                 '--out', str(self.root / 'result.json')], capture_output=True, text=True, timeout=20)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Radius mode not supported', result.stderr)

    def test_hardware_pwm_switch_reports_its_output_duty(self):
        with self.config.open('a') as config:
            config.write('[switch]\nfan.enable = true\nfan.output_pin = PWM1_1\nfan.output_type = hwpwm\n'
                         'fan.input_on_command = M106\nfan.input_off_command = M107\n')
        program = self.root / 'pwm.gcode'
        program.write_text('M106 S50\n;@sim expect-pwm PWM1_1 0.5\nM107\n;@sim expect-pwm PWM1_1 0\n')
        output = self.root / 'result.json'
        result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for directive in (';@sim expect-pwm PWM1_1 1', ';@sim expect-pwm PWM1_2 0',
                          ';@sim expect-pwm invalid 0', ';@sim expect-pwm PWM1_1 nan',
                          ';@sim expect-pwm PWM1_1 1.1'):
            program.write_text('M106 S50\n' + directive + '\n')
            result = subprocess.run([SIM, '--modules', '--config', str(self.config), '--gcode', str(program), '--out', str(output)], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0, directive)


if __name__ == '__main__':
    unittest.main()
