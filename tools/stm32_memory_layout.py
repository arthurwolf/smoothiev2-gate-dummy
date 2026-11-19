#!/usr/bin/env python3
"""Extract SmoothieV2 STM32H745 allocator bounds from a linked firmware map and ELF."""
import argparse
import json
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'checkers'))
from lpc_dwarf import elf_type_sizes
from lpc_memory_layout import parse_symbol

REGION = re.compile(r'^\s*(DTCMRAM|AXI_SRAM|SRAM_1)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\b', re.M)


def build_layout(map_text, type_sizes, commit=''):
    regions = {name: (int(start, 16), int(start, 16) + int(size, 16))
               for name, start, size in REGION.findall(map_text)}
    if set(regions) != {'DTCMRAM', 'AXI_SRAM', 'SRAM_1'}:
        raise ValueError('linker map must contain DTCMRAM, AXI_SRAM and SRAM_1')
    bounds = {}
    for key, region, start_symbol, end_symbol in (
        ('main_sram', 'AXI_SRAM', '__HeapBase', '__HeapTop'),
        ('dtcm', 'DTCMRAM', '__dtcm_bss_end', '__top_DTCMRAM'),
        ('sram1', 'SRAM_1', '__sram_1_end', '__top_SRAM_1'),
    ):
        start, end = parse_symbol(map_text, start_symbol), parse_symbol(map_text, end_symbol)
        if not regions[region][0] <= start < end <= regions[region][1]:
            raise ValueError(f'invalid {region} allocator bounds')
        bounds[key] = {'pool_start': start, 'pool_end': end}
    return {'schema_version': 1, 'architecture': 'stm32h745', 'firmware_commit': commit,
            'type_sizes': type_sizes, **bounds}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--elf', required=True)
    parser.add_argument('--host-elf')
    parser.add_argument('--readelf', default=shutil.which('arm-none-eabi-readelf') or 'arm-none-eabi-readelf')
    parser.add_argument('--commit', default='')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    wanted = elf_type_sizes(shutil.which('readelf') or 'readelf', args.host_elf) if args.host_elf else None
    sizes = elf_type_sizes(args.readelf, args.elf, set(wanted) if wanted is not None else None)
    with open(args.map) as source:
        layout = build_layout(source.read(), sizes, args.commit)
    with open(args.out, 'w') as output:
        json.dump(layout, output, indent=1, sort_keys=True)
    print(json.dumps({'type': 'result', 'layout': args.out, 'types': len(sizes)}))


if __name__ == '__main__':
    main()
