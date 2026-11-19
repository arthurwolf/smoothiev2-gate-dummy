#!/usr/bin/env python3
"""Forward to canonical shared V2 firmware instrumentation."""
from pathlib import Path
import runpy

shared = Path(__file__).resolve().parents[2] / 'lib-smoothie/tools/stage_v2_patch.py'
if not shared.is_file():
    raise SystemExit('Missing canonical lib-smoothie/tools/stage_v2_patch.py dependency')
globals().update(runpy.run_path(str(shared), run_name=__name__))
