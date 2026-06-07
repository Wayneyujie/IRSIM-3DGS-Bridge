#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ENV_FILE="${BRIDGE_ROOT}/.bridge.env"
HABITAT_GS_ROOT="${HABITAT_GS_ROOT:-}"
IRSIM_ROOT="${IRSIM_ROOT:-}"
DATA_ROOT="${DATA_ROOT:-${BRIDGE_ROOT}/data}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${BRIDGE_ROOT}/outputs}"
HABITAT_PYTHON="${HABITAT_PYTHON:-}"
BRIDGE_PYTHON="${BRIDGE_PYTHON:-python}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env-file)
      ENV_FILE="$2"
      shift 2
      ;;
    --habitat-gs-root)
      HABITAT_GS_ROOT="$2"
      shift 2
      ;;
    --irsim-root)
      IRSIM_ROOT="$2"
      shift 2
      ;;
    --data-root)
      DATA_ROOT="$2"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --habitat-python)
      HABITAT_PYTHON="$2"
      shift 2
      ;;
    --bridge-python)
      BRIDGE_PYTHON="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage: bash scripts/write_bridge_env.sh [options]

Writes a reusable shell env file for the bridge.

Common usage:
  bash scripts/write_bridge_env.sh \
    --habitat-gs-root /path/to/habitat-gs \
    --habitat-python /path/to/habitat-env/bin/python

Then load it with:
  source .bridge.env
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "$(dirname "${ENV_FILE}")"
cat > "${ENV_FILE}" <<EOF
export BRIDGE_ROOT="${BRIDGE_ROOT}"
export HABITAT_GS_ROOT="${HABITAT_GS_ROOT}"
export IRSIM_ROOT="${IRSIM_ROOT}"
export DATA_ROOT="${DATA_ROOT}"
export OUTPUT_ROOT="${OUTPUT_ROOT}"
export HABITAT_PYTHON="${HABITAT_PYTHON}"
export BRIDGE_PYTHON="${BRIDGE_PYTHON}"
EOF

echo "[env] wrote ${ENV_FILE}"
echo "[env] load with: source ${ENV_FILE}"
