import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'checkers'))
from common import Result
from gcode_to_blocks import check_invariants


class TrapezoidCheck(unittest.TestCase):
    def test_rejects_deceleration_after_block_end(self):
        block = {'gcode_line': 1, 'steps': [10], 'direction_bits': 0, 'steps_event_count': 10,
                 'nominal_speed': 10, 'nominal_rate': 100, 'millimeters': 1,
                 'entry_speed': 0, 'exit_speed': 0, 'acceleration': 100,
                 'accelerate_until': 2, 'decelerate_after': 11, 'total_move_ticks': 10,
                 'max_entry_speed': 0, 'is_g123': False}
        result = Result('test', 'test')
        check_invariants(result, {'blocks': [block], 'steps_per_mm': [10], 'final_steps': [10]}, None, False)
        checks = {entry['check']: entry['status'] for entry in result.to_dict()['checks']}
        self.assertEqual(checks['per-block planner invariants (rates, limits, trapezoid)'], 'fail')


if __name__ == '__main__':
    unittest.main()
