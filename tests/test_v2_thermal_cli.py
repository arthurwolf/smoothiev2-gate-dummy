"""Real cooperative V2 heater control using separate shared model configuration."""
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
SIM=os.environ.get('SMOOTHIE_V2_SIM')
@unittest.skipUnless(SIM,'set SMOOTHIE_V2_SIM to the built V2 simulator')
class ThermalCLI(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name)
        self.config=self.root/'config.ini';self.model=self.root/'simulation.ini'
        self.program=self.root/'program.gcode';self.output=self.root/'result.json'
        self.base='[motion control]\narm_solution=cartesian\ndefault_acceleration=100\n[actuator]\n'
        for name,pin in [('alpha','PA'),('beta','PB'),('gamma','PC')]:
            self.base+=f'{name}.step_pin={pin}0\n{name}.dir_pin={pin}1\n{name}.steps_per_mm=80\n'
        self.config.write_text(self.base+'[temperature control]\nhotend.enable=true\nhotend.heater_pin=PD0\n'
                               'hotend.thermistor_pin=ADC1_0\nhotend.designator=T\n')
        self.thermal=(Path(__file__).resolve().parents[2]/'lib-smoothie/tests/fixtures/thermal.ini').read_text()
        self.model.write_text(self.thermal)
    def run_program(self,program):
        self.program.write_text(program);self.output.unlink(missing_ok=True)
        return subprocess.run([SIM,'--modules','--config',str(self.config),'--machine-model',str(self.model),
                               '--gcode',str(self.program),'--out',str(self.output)],capture_output=True,text=True,timeout=30)
    def test_real_heater_heats_and_cools_through_thermistor_adc(self):
        result=self.run_program('G4 S1\nM105\nM104 S80\nG4 S5\nM105\nM104 S0\nG4 S5\nM105\n')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        data=json.loads(self.output.read_text())
        readings=[float(value) for value in re.findall(r'T:\s*([-0-9.]+) /',data['serial_out'])]
        self.assertEqual(len(readings),3,data['serial_out'])
        baseline,heated,cooled=readings
        self.assertTrue(23<baseline<27,readings)
        self.assertGreater(heated,baseline+2)
        self.assertTrue(baseline<cooled<heated-1,readings)
    def test_rejects_unallocated_heater_and_sensor_without_result(self):
        for wrong in [self.thermal.replace('PD0','PD9'),self.thermal.replace('PD0','PA1'),self.thermal.replace('ADC1_0','ADC1_6')]:
            with self.subTest(model=wrong):
                self.model.write_text(wrong)
                result=self.run_program('M104 S80\nG4 S1\n')
                self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertFalse(self.output.exists())
    def test_mixed_model_keeps_firmware_motion_and_heater_feedback(self):
        axes='[axis]\n'
        for axis in 'xyz':
            axes+=f'{axis}.steps_per_mm=80\n{axis}.initial_mm=0\n{axis}.minimum_mm=-10\n{axis}.maximum_mm=10\n{axis}.positive_direction_level=false\n{axis}.travel=stall\n'
        self.model.write_text(axes+self.thermal)
        result=self.run_program('G1 X1 F600\nM400\nG4 S1\nM104 S80\nG4 S5\nM105\nsim-model status\n')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        data=json.loads(self.output.read_text())
        self.assertRegex(data['serial_out'],r'T:\s*[3-9][0-9]\.')
        states=[json.loads(line) for line in data['serial_out'].splitlines() if line.startswith('{')]
        self.assertEqual(states[-1]['axes'][0]['emitted_pulses'],80)
        self.assertTrue(states[-1]['healthy'])

    def test_fault_commands_and_sensor_lag_use_real_adapter_feedback(self):
        self.model.write_text(self.thermal+'hotend.sensor_lag_seconds=10\n')
        result=self.run_program('sim-thermal fault ADC1_0 stuck_on normal\nG4 S10\nsim-thermal status\n'
                                'sim-thermal fault ADC1_0 disconnected frozen\nG4 S10\nsim-thermal status\n')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        data=json.loads(self.output.read_text())
        states=[json.loads(line) for line in data['serial_out'].splitlines() if line.startswith('{')]
        self.assertEqual(len(states),4,data['serial_out'])
        heated=states[1]['channels'][0];cooled=states[3]['channels'][0]
        self.assertTrue(states[1]['synthetic'] and states[1]['healthy'])
        self.assertAlmostEqual(heated['temperature_c'],88.2120558829,places=2)
        self.assertAlmostEqual(heated['sensor_temperature_c'],51.4241117657,places=2)
        self.assertFalse(heated['output_level'])
        self.assertLess(cooled['temperature_c'],49)
        self.assertEqual(cooled['adc'],states[2]['channels'][0]['adc'])
        self.assertEqual(cooled['sensor_fault'],'frozen')

    def test_open_and_shorted_sensor_trigger_firmware_alarm(self):
        for fault in ['disconnected','shorted']:
            with self.subTest(fault=fault):
                self.model.write_text(self.thermal)
                result=self.run_program(f'sim-thermal fault ADC1_0 normal {fault}\nM104 S80\nG4 S3\nM105\n')
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                data=json.loads(self.output.read_text())
                self.assertIn('MINTEMP or MAXTEMP triggered',data['serial_out'])
                self.assertIn('HALT asserted',data['serial_out'])
