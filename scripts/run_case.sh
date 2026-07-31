#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_case.sh --config <file.conf> [options]

Options:
  --config <path>       Configuration file passed to ap_mesh (required).
  --results <path>      Directory used for the captured run log.
  --verify <command>    Optional verification command executed after the run.
  --executable <path>   Executable to run (default: build/bin/ap_mesh).
  --build               Build the software before execution.
  --clean-results       Remove the results directory before execution.
  --skip-input-check    Skip the independent configuration/input preflight.
  --help                Show this help message.

Examples:
  bash scripts/run_case.sh \
    --config configs/book/article.conf \
    --results results/book \
    --verify "python3 scripts/verify_book.py results/book" \
    --build --clean-results

  bash scripts/run_case.sh \
    --config configs/eistute/article.conf \
    --results results/eistute
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG=""
RESULTS_DIR=""
VERIFY_COMMAND=""
EXECUTABLE=""
BUILD_FIRST=0
CLEAN_RESULTS=0
CHECK_INPUT=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      [[ $# -ge 2 ]] || { echo "Missing value for --config" >&2; exit 2; }
      CONFIG="$2"
      shift 2
      ;;
    --results)
      [[ $# -ge 2 ]] || { echo "Missing value for --results" >&2; exit 2; }
      RESULTS_DIR="$2"
      shift 2
      ;;
    --verify)
      [[ $# -ge 2 ]] || { echo "Missing value for --verify" >&2; exit 2; }
      VERIFY_COMMAND="$2"
      shift 2
      ;;
    --executable)
      [[ $# -ge 2 ]] || { echo "Missing value for --executable" >&2; exit 2; }
      EXECUTABLE="$2"
      shift 2
      ;;
    --build)
      BUILD_FIRST=1
      shift
      ;;
    --clean-results)
      CLEAN_RESULTS=1
      shift
      ;;
    --skip-input-check)
      CHECK_INPUT=0
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "${CONFIG}" ]] || { echo "--config is required" >&2; usage >&2; exit 2; }

cd "${ROOT_DIR}"

if [[ ! -f "${CONFIG}" ]]; then
  echo "Configuration file not found: ${CONFIG}" >&2
  exit 3
fi

if [[ ${CHECK_INPUT} -eq 1 ]]; then
  python3 scripts/validate_case_input.py "${CONFIG}"
fi

if [[ ${BUILD_FIRST} -eq 1 ]]; then
  bash scripts/build_linux.sh
fi

if [[ -z "${EXECUTABLE}" ]]; then
  EXECUTABLE="${ROOT_DIR}/build/bin/ap_mesh"
fi
if [[ ! -x "${EXECUTABLE}" ]]; then
  echo "Executable not found: ${EXECUTABLE}" >&2
  echo "Build first with: bash scripts/build_linux.sh" >&2
  exit 4
fi

if [[ -z "${RESULTS_DIR}" ]]; then
  case_name="$(basename "${CONFIG}")"
  case_name="${case_name%.*}"
  RESULTS_DIR="results/${case_name}"
fi

if [[ ${CLEAN_RESULTS} -eq 1 ]]; then
  rm -rf "${RESULTS_DIR}"
fi
mkdir -p "${RESULTS_DIR}"

RUN_LOG="${RESULTS_DIR}/run.log"
{
  echo "Adaptive Non-Manifold Meshing"
  echo "Configuration: ${CONFIG}"
  echo "Results directory: ${RESULTS_DIR}"
  echo "Started: $(date -u +'%Y-%m-%dT%H:%M:%SZ')"
  echo
} | tee "${RUN_LOG}"

set +e
"${EXECUTABLE}" --config "${CONFIG}" 2>&1 | tee -a "${RUN_LOG}"
run_status=${PIPESTATUS[0]}
set -e

if [[ ${run_status} -ne 0 ]]; then
  echo "Execution failed with status ${run_status}." | tee -a "${RUN_LOG}" >&2
  exit "${run_status}"
fi

if [[ -n "${VERIFY_COMMAND}" ]]; then
  echo | tee -a "${RUN_LOG}"
  echo "Verification: ${VERIFY_COMMAND}" | tee -a "${RUN_LOG}"
  bash -lc "${VERIFY_COMMAND}" 2>&1 | tee -a "${RUN_LOG}"
fi

echo | tee -a "${RUN_LOG}"
echo "Completed: $(date -u +'%Y-%m-%dT%H:%M:%SZ')" | tee -a "${RUN_LOG}"

python3 scripts/write_result_manifest.py "${RESULTS_DIR}" "${CONFIG}"
python3 scripts/verify_result_manifest.py "${RESULTS_DIR}"
