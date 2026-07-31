#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

echo "============================================================"
echo "Adaptive non-manifold meshing — representative reproduction"
echo "Computers & Graphics Figure 7 — Eistute model"
echo "============================================================"

./scripts/build_linux.sh
./scripts/run_eistute_linux.sh

echo
echo "Representative Eistute result reproduced successfully."
echo "Generated data: results/eistute"
