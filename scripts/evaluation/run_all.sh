#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 "${SCRIPT_DIR}/run_evaluation.py" "$@"

echo "Evaluation completed."
echo "See summary.md in the output directory (default: evaluation/results_<timestamp>/)."
