#!/usr/bin/env python3
"""Forward to the canonical shared V2 firmware-copy stager."""
from pathlib import Path
import runpy

shared = Path(__file__).resolve().parents[2] / 'lib-smoothie/tools/stage_v2.py'
if not shared.is_file():
    raise SystemExit('Missing canonical lib-smoothie/tools/stage_v2.py dependency')
globals().update(runpy.run_path(str(shared), run_name=__name__))
