"""Version 2 staging copies real firmware sources without mutating the input."""
import shutil
import pathlib
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "stage_v2.py"
REQUIRED = ("main.cpp", "Dispatcher.cpp", "robot/Robot.cpp", "robot/Planner.cpp",
            "robot/Conveyor.cpp", "robot/StepTicker.cpp")


class Version2Staging(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        self.source = self.root / "firmware"
        self.dest = self.root / "stage"

    def populate(self):
        for name in REQUIRED:
            path = self.source / "Firmware" / "src" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("// " + name + "\n")
        for name in ("Hal/src/tmr-setup.h", "RTOS/include/FreeRTOS.h"):
            path = self.source / "Firmware" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("// " + name + "\n")

    def run_stage(self):
        return subprocess.run([sys.executable, str(SCRIPT), str(self.source), str(self.dest)],
                              capture_output=True, text=True)

    def test_valid_source_is_copied_and_input_is_unchanged(self):
        self.populate()
        result = self.run_stage()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.dest / "src/robot/Robot.cpp").read_text(), "// robot/Robot.cpp\n")
        self.assertEqual((self.dest / "Hal/src/tmr-setup.h").read_text(), "// Hal/src/tmr-setup.h\n")
        self.assertEqual((self.dest / "RTOS/include/FreeRTOS.h").read_text(), "// RTOS/include/FreeRTOS.h\n")
        (self.dest / "src/robot/Robot.cpp").write_text("changed\n")
        self.assertEqual((self.source / "Firmware/src/robot/Robot.cpp").read_text(), "// robot/Robot.cpp\n")

    def test_incomplete_source_fails_without_creating_stage(self):
        self.populate()
        (self.source / "Firmware/src/robot/StepTicker.cpp").unlink()
        result = self.run_stage()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("StepTicker.cpp", result.stderr)
        self.assertFalse(self.dest.exists())

    def test_existing_stage_is_never_overwritten(self):
        self.populate()
        self.dest.mkdir()
        (self.dest / "keep").write_text("keep")
        result = self.run_stage()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("stage already exists", result.stderr)
        self.assertEqual((self.dest / "keep").read_text(), "keep")

    def test_shared_source_and_header_changes_invalidate_build_and_missing_dependency_fails(self):
        self.populate()
        tools = self.root / 'smoothie-sim/tools'
        tools.mkdir(parents=True)
        for name in ('v2_source_key.py', 'stage_v2.py', 'stage_v2_patch.py'):
            shutil.copyfile(ROOT / 'tools' / name, tools / name)
        shared = self.root / 'lib-smoothie/include'
        for name in ('smoothie/v2/hal/Pin.h', 'smoothie/v2/hal/Adc.h',
                     'smoothie/v2/hal/Pwm.h', 'smoothie/v2/hal/Spi.h', 'smoothie/v2/v2_callback_context.h',
                     'smoothie/v2/v2_step_timer.h', 'smoothie/v2/v2_scheduler.h',
                     'smoothie/v2/step_feedback.h', 'smoothie/v2/mechanical_runtime.h',
                     'smoothie/v2/mechanical_config.h'):
            path = shared / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('// shared fixture\n')
        implementation = shared.parent / 'src/v2/v2_step_timer.cpp'
        implementation.parent.mkdir(parents=True)
        implementation.write_text('// shared timer fixture\n')
        for name in ('v2_scheduler.cpp', 'tmr_setup.cpp', 'mechanical_runtime.cpp', 'mechanical_config.cpp'):
            implementation.with_name(name).write_text('// shared fixture\n')
        shared_tools = shared.parent / 'tools'
        shared_tools.mkdir()
        for name in ('stage_v2.py', 'stage_v2_patch.py'):
            (shared_tools / name).write_text('# shared tool fixture\n')
        patcher = shared_tools / 'stage_v2_patch.py'
        command = [sys.executable, str(tools / 'v2_source_key.py'), str(self.source)]
        first = subprocess.run(command, capture_output=True, text=True, check=True).stdout
        header = shared / 'smoothie/v2/hal/Pin.h'
        header.write_text('// changed shared fixture\n')
        second = subprocess.run(command, capture_output=True, text=True, check=True).stdout
        self.assertNotEqual(first, second)
        implementation.write_text('// changed timer fixture\n')
        third = subprocess.run(command, capture_output=True, text=True, check=True).stdout
        self.assertNotEqual(second, third)
        patcher.write_text('# changed instrumentation fixture\n')
        fourth = subprocess.run(command, capture_output=True, text=True, check=True).stdout
        self.assertNotEqual(third, fourth)
        for dependency in (header, implementation, patcher, shared / 'smoothie/v2/step_feedback.h',
                           shared / 'smoothie/v2/mechanical_runtime.h',
                           shared / 'smoothie/v2/mechanical_config.h',
                           implementation.with_name('mechanical_runtime.cpp'),
                           shared / 'smoothie/v2/hal/Adc.h', shared / 'smoothie/v2/hal/Pwm.h',
                           shared / 'smoothie/v2/hal/Spi.h'):
            original = dependency.read_bytes()
            dependency.unlink()
            missing = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(missing.returncode, 0)
            self.assertIn('Missing canonical lib-smoothie', missing.stderr)
            dependency.write_bytes(original)

    def test_firmware_fingerprint_changes_with_source(self):
        self.populate()
        command = [sys.executable, str(ROOT / 'tools/v2_source_key.py'), str(self.source)]
        first = subprocess.run(command, capture_output=True, text=True, check=True).stdout
        (self.source / 'Firmware/src/robot/Robot.cpp').write_text('// changed\n')
        second = subprocess.run(command, capture_output=True, text=True, check=True).stdout
        self.assertNotEqual(first, second)


if __name__ == "__main__":
    unittest.main()
