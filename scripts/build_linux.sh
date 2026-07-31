#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

for command_name in cmake c++; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: required command not found: ${command_name}" >&2
    exit 1
  fi
done

BUILD_DIR="${AP_MESH_BUILD_DIR:-build}"
BUILD_JOBS="${AP_MESH_BUILD_JOBS:-2}"
CLEAN_BUILD="${AP_MESH_CLEAN_BUILD:-1}"

if ! [[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: AP_MESH_BUILD_JOBS must be a positive integer." >&2
  exit 1
fi

if [[ "${CLEAN_BUILD}" == "1" ]]; then
  rm -rf "${BUILD_DIR}"
fi

cmake \
  -S . \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DAP_MESH_USE_MPI=OFF \
  -DAP_MESH_USE_OPENMP=OFF \
  -DAP_MESH_ENABLE_LTO=OFF

cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS}"

EXECUTABLE="${BUILD_DIR}/bin/ap_mesh"
if [[ ! -x "${EXECUTABLE}" ]]; then
  echo "ERROR: expected executable was not generated: ${EXECUTABLE}" >&2
  exit 1
fi

echo "Linux build completed: ${EXECUTABLE}"
