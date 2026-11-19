import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'checkers'))
import size_check


class V1StaticSizeTests(unittest.TestCase):
    def test_heap_and_dummy_stack_are_not_counted_as_static_data(self):
        sections = [('.text', 100, 0x4000), ('.data', 400, 0x10000000),
                    ('.bss', 2024, 0x10000190), ('.heap', 2048, 0x10000978),
                    ('.stack_dummy', 3072, 0x10000978), ('.AHBSRAM0', 1200, 0x2007c000),
                    ('.AHBSRAM1', 5944, 0x20080000)]
        usage, _ = size_check.classify(sections)
        self.assertEqual(usage, {'flash': 500, 'ram': 2424, 'ahb_ram': 7144})


if __name__ == '__main__':
    unittest.main()
