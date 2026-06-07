#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${BRIDGE_ROOT}/.bridge.env"

if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
fi

BRIDGE_PYTHON="${BRIDGE_PYTHON:-python}"
HABITAT_PYTHON="${HABITAT_PYTHON:-}"
HABITAT_GS_ROOT="${HABITAT_GS_ROOT:-}"
DATA_ROOT="${DATA_ROOT:-${BRIDGE_ROOT}/data}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${BRIDGE_ROOT}/outputs}"
VIEWER_MODE="first_person"
FORCE=0
DISPLAY_IRSIM=0
PREPARE_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bridge-python)
      BRIDGE_PYTHON="$2"
      shift 2
      ;;
    --habitat-python)
      HABITAT_PYTHON="$2"
      shift 2
      ;;
    --habitat-gs-root)
      HABITAT_GS_ROOT="$2"
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
    --viewer)
      VIEWER_MODE="$2"
      shift 2
      ;;
    --prepare-only)
      PREPARE_ONLY=1
      VIEWER_MODE="none"
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --display-irsim)
      DISPLAY_IRSIM=1
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage: bash scripts/run_scene01_demo.sh [options]

Run the public scene01 demo with the repository defaults:
  3DGS -> occupancy -> IR-SIM follow -> GS trajectory -> optional Habitat-GS replay

Examples:
  bash scripts/run_scene01_demo.sh
  bash scripts/run_scene01_demo.sh --viewer first_person \
    --habitat-gs-root /path/to/habitat-gs \
    --habitat-python /path/to/habitat-env/bin/python
  bash scripts/run_scene01_demo.sh --viewer overview --force
  bash scripts/run_scene01_demo.sh --prepare-only

Viewer modes:
  none
  first_person
  overview
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

SCENE_ROOT="${DATA_ROOT}/gs_scenes"
QUICK_ROOT="${OUTPUT_ROOT}/quickstart/scene01"
OCC_DIR="${QUICK_ROOT}/occupancy"
WORLD_DIR="${QUICK_ROOT}/world"
FOLLOW_DIR="${QUICK_ROOT}/follow"
SYNC_DIR="${QUICK_ROOT}/gs_sync"

SCENE_PLY="${SCENE_ROOT}/train/scene01/scene01.gs.ply"
DATASET_JSON="${SCENE_ROOT}/train.scene_dataset_config.json"
WORLD_YAML="${WORLD_DIR}/scene01_gs_irsim_demo.yaml"
TRACE_JSONL="${FOLLOW_DIR}/irsim_follow_trace.jsonl"
CLICKED_WORLD_YAML="${FOLLOW_DIR}/clicked_start_goal_world.yaml"
GS_TRAJ_JSONL="${SYNC_DIR}/gs_agent_trajectory.jsonl"
VIEWPOINT_JSON="${BRIDGE_ROOT}/configs/live_sync/my_overview_viewpoint.json"

START="72.8741379310345,95.7265337423313"
GOAL="79.7,53.3"

mkdir -p "${QUICK_ROOT}"

if [[ "${FORCE}" == "1" ]]; then
  rm -rf "${OCC_DIR}" "${WORLD_DIR}" "${FOLLOW_DIR}" "${SYNC_DIR}"
fi

if [[ ! -f "${SCENE_PLY}" ]]; then
  echo "[demo] downloading scene01"
  "${BRIDGE_PYTHON}" "${BRIDGE_ROOT}/scripts/download_scene01.py" --output_dir "${SCENE_ROOT}"
fi

if [[ ! -f "${OCC_DIR}/map.yaml" ]]; then
  echo "[demo] building occupancy map"
  "${BRIDGE_PYTHON}" "${BRIDGE_ROOT}/scripts/gs_to_occupancy.py" \
    --input "${SCENE_PLY}" \
    --output_dir "${OCC_DIR}" \
    --resolution 0.05 \
    --robot_radius 0.25 \
    --image_transform transpose_rotate_180
fi

if [[ ! -f "${WORLD_YAML}" ]]; then
  echo "[demo] exporting IR-SIM world"
  "${BRIDGE_PYTHON}" "${BRIDGE_ROOT}/scripts/export_irsim_world_from_occupancy.py" \
    --occupancy_dir "${OCC_DIR}" \
    --output_dir "${WORLD_DIR}" \
    --world_name scene01_gs_irsim_demo \
    --unknown_as free
fi

if [[ ! -f "${TRACE_JSONL}" ]]; then
  echo "[demo] running IR-SIM follow"
  FOLLOW_ARGS=(
    "${BRIDGE_PYTHON}" "${BRIDGE_ROOT}/scripts/interactive_astar_irsim.py"
    --world "${WORLD_YAML}"
    --output_dir "${FOLLOW_DIR}"
    --start "${START}"
    --goal "${GOAL}"
    --planning-inflation-radius 0.6
    --follow
  )
  if [[ -n "${IRSIM_ROOT:-}" ]]; then
    FOLLOW_ARGS+=(--irsim_root "${IRSIM_ROOT}")
  fi
  if [[ "${DISPLAY_IRSIM}" == "1" ]]; then
    FOLLOW_ARGS+=(--display)
  fi
  MPLCONFIGDIR=/tmp/matplotlib "${FOLLOW_ARGS[@]}"
fi

if [[ ! -f "${GS_TRAJ_JSONL}" ]]; then
  echo "[demo] converting IR-SIM trace back to Habitat-GS poses"
  "${BRIDGE_PYTHON}" "${BRIDGE_ROOT}/scripts/convert_irsim_trace_to_gs_trajectory.py" \
    --trace "${TRACE_JSONL}" \
    --map_yaml "${OCC_DIR}/map.yaml" \
    --world "${CLICKED_WORLD_YAML}" \
    --output_dir "${SYNC_DIR}"
fi

echo "[demo] outputs ready under ${QUICK_ROOT}"
echo "[demo] trajectory: ${GS_TRAJ_JSONL}"

if [[ "${PREPARE_ONLY}" == "1" || "${VIEWER_MODE}" == "none" ]]; then
  exit 0
fi

if [[ -z "${HABITAT_GS_ROOT}" || -z "${HABITAT_PYTHON}" ]]; then
  echo "[demo] Habitat-GS replay is the default final step." >&2
  echo "[demo] Set both --habitat-gs-root and --habitat-python, or write them once with:" >&2
  echo "       bash scripts/write_bridge_env.sh --habitat-gs-root /path/to/habitat-gs --habitat-python /path/to/habitat-env/bin/python" >&2
  echo "[demo] If you only want intermediate bridge outputs, re-run with --prepare-only" >&2
  exit 1
fi

cd "${HABITAT_GS_ROOT}"

VIEWER_ARGS=(
  examples/gaussian_viewer.py
  --dataset "${DATASET_JSON}"
  --scene scene01
  --agent-trajectory "${GS_TRAJ_JSONL}"
  --trajectory-snap-to-navmesh
  --trajectory-camera-height 1.5
)

if [[ "${VIEWER_MODE}" == "overview" ]]; then
  VIEWER_ARGS+=(
    --viewpoint "${VIEWPOINT_JSON}"
    --trajectory-overview
    --hide-text
  )
elif [[ "${VIEWER_MODE}" != "first_person" ]]; then
  echo "[demo] unsupported viewer mode: ${VIEWER_MODE}" >&2
  exit 1
fi

exec "${HABITAT_PYTHON}" "${VIEWER_ARGS[@]}"
