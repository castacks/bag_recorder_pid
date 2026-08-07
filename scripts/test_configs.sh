#!/usr/bin/env bash
set -uo pipefail

# Usage: test_configs.sh [duration_seconds] [output_dir]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGGER="${SCRIPT_DIR}/bagger.py"
CONFIG_DIR="${SCRIPT_DIR}/../config"

DURATION="${1:-5}"
OUTPUT_DIR="${2:-${HOME}/rosbags/$(date +%Y%m%d)/test_diagnostics$(date +%%H%M%S)}"

mkdir -p "${OUTPUT_DIR}"
cd "${OUTPUT_DIR}"

configs=("${CONFIG_DIR}"/test_routines/test_*.yaml)
echo "Testing ${#configs[@]} configs, ${DURATION}s each, output in ${OUTPUT_DIR}"
echo

for cfg in "${configs[@]}"; do
    name="$(basename "${cfg}" .yaml)"
    echo "=================================================================="
    echo "=== ${name} (${DURATION}s) ==="
    echo "=================================================================="

    # -s INT mimics Ctrl+C so bagger.py does KeyboardInterrupt and runs
    # post_logger routines
    echo y | timeout -s INT "${DURATION}" python3 "${BAGGER}" -c "${cfg}" -o "${name}" 2>&1 | tee "${name}.log"

    # wait a bit
    sleep 2
    echo
done

echo "Done. Results in ${OUTPUT_DIR}"
