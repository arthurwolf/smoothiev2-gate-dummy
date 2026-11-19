import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("step_trace", Path(__file__).resolve().parents[1] / "checkers" / "step_trace.py")
step_trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(step_trace)


class V2TraceTests(unittest.TestCase):
    def test_equivalent_period_notation_matches(self):
        result = {"n_actuators": 1, "steps_per_mm": [80], "initial_steps": [12],
                  "final_steps": [12], "step_period_us": 10, "blocks": []}
        reference = step_trace.record(result)
        actual = step_trace.record(dict(result, step_period_us=10.0))
        self.assertEqual(step_trace.compare(reference, actual)["status"], "pass")

    def test_v2_output_cannot_match_reference_without_initial_state(self):
        result = {"n_actuators": 1, "steps_per_mm": [80], "initial_steps": [12],
                  "final_steps": [12], "step_period_us": 10, "blocks": []}
        actual = step_trace.record(result)
        reference = step_trace.record({key: value for key, value in result.items()
                                       if key not in ("initial_steps", "step_period_us")})
        self.assertEqual(step_trace.compare(reference, actual)["status"], "fail")

    def test_initial_position_and_period_are_part_of_v2_reference(self):
        result = {"n_actuators": 1, "steps_per_mm": [80], "initial_steps": [12],
                  "final_steps": [12], "step_period_us": 20, "blocks": []}
        reference = step_trace.record(result)
        self.assertEqual(step_trace.compare(reference, step_trace.record(result))["status"], "pass")
        for key, value in (("initial_steps", [13]), ("step_period_us", 10)):
            changed = dict(result, **{key: value})
            self.assertEqual(step_trace.compare(reference, step_trace.record(changed))["status"], "fail")


if __name__ == "__main__":
    unittest.main()
