"""STM32 section accounting includes RAM execution and flash load images."""
import os
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'checkers'))
import size_check


class Stm32Size(unittest.TestCase):
    def test_remaining_space_accounts_for_linker_alignment_gaps(self):
        sections = [('.text', 16, 0x08000000, 0x08000000, {'ALLOC', 'LOAD'}),
                    ('.data', 8, 0x24000020, 0x08000020, {'ALLOC', 'LOAD'}),
                    ('.bss', 8, 0x24000040, 0x08000028, {'ALLOC'})]
        use, _ = size_check.classify_stm32(sections)
        footprint = size_check.stm32_footprints(sections)
        self.assertEqual((use['flash'], footprint['flash']), (24, 40))
        self.assertEqual((use['axi_sram'], footprint['axi_sram']), (16, 72))
        self.assertEqual(footprint['sram1'], 0)

    def test_sections_and_load_images(self):
        text = '''Idx Name          Size      VMA       LMA       File off  Algn
  0 .text         00000080  08000000  08000000  00001000  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  1 .data         00000010  24000000  08000080  00002000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  2 .itcm_text    00000020  00000000  08000090  00003000  2**2
                  CONTENTS, ALLOC, LOAD, CODE
  3 .dtcm_bss     00000040  20000000  080000b0  00004000  2**2
                  ALLOC
  4 .debug_info   00000100  00000000  00000000  00005000  2**0
                  CONTENTS, READONLY, DEBUGGING
'''
        use, _ = size_check.classify_stm32(size_check.parse_objdump_sections(text))
        self.assertEqual(use, {'flash': 0xb0, 'itcm': 0x20, 'dtcm': 0x40,
                               'axi_sram': 0x10, 'sram1': 0, 'sram2': 0, 'sram3': 0, 'sram4': 0})

    def test_all_banks_and_unknown_allocations(self):
        use, _ = size_check.classify_stm32([
            ('.sram3', 16, 0x30040000, 0x30040000, {'ALLOC'}),
            ('.sram4', 32, 0x38000000, 0x38000000, {'ALLOC'}),
        ])
        self.assertEqual((use['sram3'], use['sram4']), (16, 32))
        with self.assertRaises(ValueError):
            size_check.classify_stm32([('.unknown', 16, 0x60000000, 0x60000000, {'ALLOC'})])

    def test_growth_in_any_ram_pool_trips_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            ref, candidate, result = root / 'ref', root / 'candidate', root / 'result.json'
            section = '  0 .data  {size:08x}  24000000  08000000  00001000  2**2\n                  CONTENTS, ALLOC, LOAD, DATA\n'
            ref.write_text(section.format(size=16))
            candidate.write_text(section.format(size=32))
            command = [sys.executable, size_check.__file__, '--architecture', 'stm32h745',
                       '--ref-size-file', str(ref), '--cand-size-file', str(candidate),
                       '--max-ram-increase', '8', '--out', str(result)]
            run = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(run.returncode, 1, run.stderr)
            data = json.loads(result.read_text())
            self.assertEqual(data['status'], 'fail')
            self.assertIn('RAM grew by 16 bytes', data['problems'][0])


if __name__ == '__main__':
    unittest.main()
