#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

bash scripts/run_case.sh \
  --config configs/eistute/article.conf \
  --results results/eistute \
  --verify "python3 scripts/verify_eistute.py results/eistute" \
  --clean-results
