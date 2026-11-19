#!/usr/bin/env bash
# Experimental SmoothieV2 checker workflow. BASE may be - to omit a reference tree.
# run_checks_v2.sh [SIM [FIRMWARE_SRC [BASE [OUT [REF_ELF CAND_ELF]]]]]
# V2_CONFIG, V2_MOTION, V2_WORKLOAD override the included Cartesian examples.
# STM32_LAYOUT supplies a layout; BASE_SIM supplies a reference simulator.
# V2_MEMORY_SIM optionally supplies an uninstrumented simulator for memory estimates.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SIM=${1:-$HERE/build-v2/smoothie-sim-v2}
MEMORY_SIM=${V2_MEMORY_SIM:-$SIM}
SRC=${2:-$HERE/../smoothieware-microros/src/SmoothieV2/Firmware}
BASE=${3:--}; OUT=${4:-$HERE/out-v2}; REF_ELF=${5:-}; CAND_ELF=${6:-}
if { [ -n "$REF_ELF" ] && [ -z "$CAND_ELF" ]; } || { [ -z "$REF_ELF" ] && [ -n "$CAND_ELF" ]; }; then
  printf '%s\n' 'Both reference and candidate ELF files are required for the size check.' >&2
  exit 2
fi
CONFIG=${V2_CONFIG:-$HERE/cases/v2/config.ini}
MOTION=${V2_MOTION:-$HERE/cases/v2/motion.gcode}
WORKLOAD=${V2_WORKLOAD:-$HERE/cases/v2/modules.gcode}
mkdir -p "$OUT"
rc=0
python3 "$HERE/checkers/gcode_to_blocks.py" --sim "$SIM" --config "$CONFIG" --gcode "$MOTION" --workdir "$OUT/w1" --cartesian --out "$OUT/gcode_to_blocks.result.json" || rc=1
python3 "$HERE/checkers/blocks_to_steps.py" --sim "$SIM" --config "$CONFIG" --gcode "$MOTION" --workdir "$OUT/w2" --out "$OUT/gcode_to_steps.result.json" || rc=1
BASEARG=(); [ "$BASE" != "-" ] && BASEARG=(--base-src "$BASE")
BASESIMARG=(); [ -n "${BASE_SIM:-}" ] && BASESIMARG=(--base-sim "$BASE_SIM")
python3 "$HERE/checkers/modules_check.py" --firmware-version 2 --sim "$SIM" --src "$SRC" "${BASEARG[@]}" "${BASESIMARG[@]}" --config "$CONFIG" --workload "$WORKLOAD" --motion "$MOTION" --workdir "$OUT/w3" --out "$OUT/modules.result.json" || rc=1
LAYOUTARG=(); [ -n "${STM32_LAYOUT:-}" ] && LAYOUTARG=(--layout "$STM32_LAYOUT")
python3 "$HERE/checkers/memory_check.py" --sim "$MEMORY_SIM" --config "$CONFIG" --gcode "$WORKLOAD" "${LAYOUTARG[@]}" --workdir "$OUT/w4" --out "$OUT/memory.result.json" || rc=1
python3 "$HERE/checkers/fault_check.py" --sim "$SIM" --config "$CONFIG" --workdir "$OUT/w5" --out "$OUT/faults.result.json" || rc=1
SIZEARG=()
if [ -n "$REF_ELF" ] && [ -n "$CAND_ELF" ]; then
  python3 "$HERE/checkers/size_check.py" --architecture stm32h745 --ref-elf "$REF_ELF" --cand-elf "$CAND_ELF" --tool-prefix "${SIZE_TOOL_PREFIX-arm-none-eabi-}" --out "$OUT/size.result.json" || rc=1
  SIZEARG=(--size "$OUT/size.result.json")
fi
python3 "$HERE/checkers/report.py" --results "$OUT/gcode_to_blocks.result.json" "$OUT/gcode_to_steps.result.json" "$OUT/modules.result.json" "$OUT/memory.result.json" "$OUT/faults.result.json" "${SIZEARG[@]}" --sim-json "$OUT/w1/gcode_to_blocks.json" --subject "${SUBJECT:-SmoothieV2 simulator checks}" --run-url "${RUN_URL:-}" --html "$OUT/report.html" --summary-md "$OUT/summary.md" --comment-md "$OUT/comment.md" || rc=1
exit "$rc"
