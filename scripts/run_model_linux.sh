#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_model_linux.sh <model> [profile] [--no-verify]

Maintained models:
  book
  eistute
  decor_shelf

Profiles:
  article        Default maintained profile.
  <name>         Any configuration available under configs/<model>/.

Examples:
  bash scripts/run_model_linux.sh book
  bash scripts/run_model_linux.sh eistute article
  bash scripts/run_model_linux.sh decor_shelf article --no-verify

This is a convenience interface. Arbitrary configuration files can be run with:
  bash scripts/run_case.sh --config <file.conf>
EOF
}

if [[ $# -lt 1 || $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

MODEL="$1"
PROFILE="article"
VERIFY=1

if [[ $# -ge 2 ]]; then
  if [[ "$2" == "--no-verify" ]]; then
    VERIFY=0
  else
    PROFILE="$2"
  fi
fi

if [[ $# -eq 3 ]]; then
  if [[ "$3" != "--no-verify" ]]; then
    echo "Unsupported option: $3" >&2
    usage >&2
    exit 2
  fi
  VERIFY=0
fi

case "${MODEL}" in
  book)
    VERIFY_SCRIPT="scripts/verify_book.py"
    ;;
  eistute)
    VERIFY_SCRIPT="scripts/verify_eistute.py"
    ;;
  decor_shelf)
    VERIFY_SCRIPT="scripts/verify_decor_shelf.py"
    ;;
  *)
    echo "Unsupported maintained model: ${MODEL}" >&2
    usage >&2
    exit 2
    ;;
esac

CONFIG_PATH="configs/${MODEL}/${PROFILE}.conf"
RESULTS_DIR="results/${MODEL}"

command=(
  bash scripts/run_case.sh
  --config "${CONFIG_PATH}"
  --results "${RESULTS_DIR}"
  --clean-results
)

if [[ ${VERIFY} -eq 1 ]]; then
  command+=(--verify "python3 ${VERIFY_SCRIPT} ${RESULTS_DIR}")
fi

"${command[@]}"
