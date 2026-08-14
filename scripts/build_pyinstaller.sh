#!/usr/bin/env bash
# Build both Motion executables from the frozen lock in an isolated Python 3.12 environment.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_ENV="${ROOT}/.venv_build"
DIST_DIR="${ROOT}/dist"
WORK_DIR="${ROOT}/build/pyinstaller"
SPEC_FILE="${SCRIPT_DIR}/motion.spec"
MOTION_SERVER="${DIST_DIR}/motion_server"
MOTION_ACTION_POLICY="${DIST_DIR}/motion_action_policy"

cleanup() {
  rm -rf "${BUILD_ENV}"
}
trap cleanup EXIT

cd "${ROOT}"
rm -rf \
  "${BUILD_ENV}" \
  "${WORK_DIR}" \
  "${MOTION_SERVER}" \
  "${MOTION_ACTION_POLICY}"
mkdir -p "${DIST_DIR}" "${WORK_DIR}"

echo "==> [motion] syncing frozen Python 3.12 build environment..."
UV_PROJECT_ENVIRONMENT="${BUILD_ENV}" uv sync \
  --project "${ROOT}" \
  --python 3.12 \
  --frozen \
  --no-dev \
  --group build

PYTHON_VERSION="$("${BUILD_ENV}/bin/python" -c \
  'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "${PYTHON_VERSION}" != "3.12" ]]; then
  echo "ERROR: expected Python 3.12, got ${PYTHON_VERSION}" >&2
  exit 1
fi

CMEEL_LIB="$("${BUILD_ENV}/bin/python" -c \
  'from pathlib import Path; import pinocchio; print(Path(pinocchio.__file__).resolve().parents[3])')"
ORIGINAL_LD_LIBRARY_PATH="${LD_LIBRARY_PATH-}"
export LD_LIBRARY_PATH="${CMEEL_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

echo "==> [motion] building one-file executables..."
"${BUILD_ENV}/bin/pyinstaller" \
  --noconfirm \
  --clean \
  --distpath "${DIST_DIR}" \
  --workpath "${WORK_DIR}" \
  "${SPEC_FILE}"

if [[ -n "${ORIGINAL_LD_LIBRARY_PATH}" ]]; then
  export LD_LIBRARY_PATH="${ORIGINAL_LD_LIBRARY_PATH}"
else
  unset LD_LIBRARY_PATH
fi

for binary in "${MOTION_SERVER}" "${MOTION_ACTION_POLICY}"; do
  if [[ ! -x "${binary}" ]]; then
    echo "ERROR: missing executable ${binary}" >&2
    exit 1
  fi
done

echo "==> [motion] checking CLI imports without hardware access..."
"${MOTION_SERVER}" --help >/dev/null
"${MOTION_ACTION_POLICY}" --help >/dev/null

expect_usage_error() {
  local binary="$1"
  shift
  local status=0
  "${binary}" "$@" >/dev/null 2>&1 || status=$?
  if [[ "${status}" -ne 2 ]]; then
    echo "ERROR: ${binary} expected argparse exit 2, got ${status}" >&2
    exit 1
  fi
}

# Argument parsing exits before config loading or Dora node creation.
expect_usage_error "${MOTION_SERVER}"
expect_usage_error "${MOTION_ACTION_POLICY}" --invalid-build-smoke-option

echo "==> [motion] build succeeded:"
du -h "${MOTION_ACTION_POLICY}" "${MOTION_SERVER}"
