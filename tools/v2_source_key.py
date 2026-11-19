#!/usr/bin/env python3
"""Identify all firmware and staging inputs used by a version 2 build."""
import hashlib
from pathlib import Path
import sys

source = Path(sys.argv[1]).resolve()
firmware = source / 'Firmware' if (source / 'Firmware').is_dir() else source
roots = [firmware / name for name in ('src', 'Hal', 'RTOS')]
if any(not root.is_dir() for root in roots):
    print('SmoothieV2 source must contain src, Hal and RTOS', file=sys.stderr)
    sys.exit(1)
digest = hashlib.sha256()
for root in roots:
    for path in sorted(p for p in root.rglob('*') if p.is_file()):
        digest.update(path.relative_to(firmware).as_posix().encode() + b'\0')
        digest.update(hashlib.sha256(path.read_bytes()).digest())
for path in (Path(__file__), Path(__file__).with_name('stage_v2.py'), Path(__file__).with_name('stage_v2_patch.py')):
    digest.update(hashlib.sha256(path.read_bytes()).digest())
shared = Path(__file__).resolve().parents[2] / 'lib-smoothie'
required_shared = ('include/smoothie/v2/hal/Adc.h', 'include/smoothie/v2/hal/Pwm.h',
                   'include/smoothie/v2/hal/Spi.h', 'include/smoothie/v2/hal/Pin.h', 'include/smoothie/v2/v2_callback_context.h',
                   'include/smoothie/v2/v2_step_timer.h', 'include/smoothie/v2/v2_scheduler.h',
                   'include/smoothie/v2/step_feedback.h', 'include/smoothie/v2/mechanical_runtime.h',
                   'include/smoothie/v2/mechanical_config.h', 'src/v2/mechanical_runtime.cpp',
                   'src/v2/mechanical_config.cpp',
                   'src/v2/v2_step_timer.cpp', 'src/v2/v2_scheduler.cpp', 'src/v2/tmr_setup.cpp',
                   'tools/stage_v2.py', 'tools/stage_v2_patch.py')
if any(not (shared / name).is_file() for name in required_shared):
    raise SystemExit('Missing canonical lib-smoothie dependency')
for directory, pattern in (('include', '*.h'), ('src', '*.cpp'), ('tools', '*.py')):
    for path in sorted((shared / directory).rglob(pattern)):
        digest.update(path.relative_to(shared).as_posix().encode() + b'\0')
        digest.update(hashlib.sha256(path.read_bytes()).digest())
print(digest.hexdigest())
