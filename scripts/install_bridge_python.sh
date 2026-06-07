#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PYTHON_BIN="${PYTHON_BIN:-python}"
INSTALL_IRSIM=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --python)
      PYTHON_BIN="$2"
      shift 2
      ;;
    --skip-irsim)
      INSTALL_IRSIM=0
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage: bash scripts/install_bridge_python.sh [--python /path/to/python] [--skip-irsim]

Installs the bridge Python dependencies into the selected interpreter.
By default it also installs `ir-sim[all]` from PyPI so the quick demo can run
without a local IR-SIM checkout.
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

echo "[install] python: ${PYTHON_BIN}"
"${PYTHON_BIN}" -m pip install --upgrade pip
if [[ "${INSTALL_IRSIM}" == "1" ]]; then
  "${PYTHON_BIN}" -m pip install "ir-sim[all]"
fi
"${PYTHON_BIN}" -m pip install -r "${BRIDGE_ROOT}/requirements-bridge.txt"

cat <<EOF
[install] done
- interpreter: ${PYTHON_BIN}
- ir-sim from pip: ${INSTALL_IRSIM}
- bridge requirements: ${BRIDGE_ROOT}/requirements-bridge.txt
EOF
