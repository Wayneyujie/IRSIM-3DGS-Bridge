# IRSIM-3DGS-Bridge

Bridge scripts and documentation for a practical closed loop between a 3D Gaussian Splatting scene, IR-SIM path following, and Habitat-GS first-person playback.

This repository does not train 3DGS scenes, build Habitat-GS, or ship IR-SIM itself. It focuses on the glue:

1. Download or prepare a Habitat-GS-compatible 3DGS scene.
2. Convert the scene into a 2D occupancy map.
3. Export an IR-SIM world from that occupancy map.
4. Run A* and IR-SIM waypoint following.
5. Convert the resulting IR-SIM trace back into a Habitat-GS camera trajectory.
6. Replay that trajectory inside `gaussian_viewer.py` in either overview or first-person mode.

## Repository Scope

Included here:

- Bridge scripts in [scripts](scripts)
- A minimal public demo path based on `scene01`
- Lightweight example outputs in [examples/expected_outputs](examples/expected_outputs)
- Live-sync viewer configuration in [configs/live_sync](configs/live_sync)

Not included here:

- Full Habitat-GS source tree
- Full IR-SIM source tree
- Large scene assets
- 3DGS training pipeline
- Collision mesh generation from arbitrary custom GS scenes

## Prerequisites

You need working local installations of:

- `habitat-gs`
- `IR-SIM`
- Python with `numpy`, `scipy`, `matplotlib`, `opencv-python`, `pyyaml`

Optional but recommended:

- `huggingface_hub` for downloading the public `scene01` demo
- `plyfile` if you need to strip a GS PLY for Habitat-GS compatibility

The bridge scripts were validated with:

- Habitat-GS scene loading through `examples/gaussian_viewer.py`
- IR-SIM follow traces written as `irsim_follow_trace.jsonl`
- Habitat-GS trajectory replay via `--agent-trajectory`

For a full environment setup, including the recommended two-conda-environment workflow, see [docs/setup.md](docs/setup.md).

## Environment Setup

The recommended layout uses two environments:

- `habitat-gs`: Habitat-GS build, `gaussian_viewer.py`, occupancy extraction, offline trajectory conversion
- `irsim_latest`: IR-SIM runtime, `interactive_astar_irsim.py`, live watcher

High-level steps:

1. Build or prepare `habitat-gs`
2. Build or prepare `IR-SIM`
3. Install the bridge script dependencies
4. Export shared path variables

Minimal bridge-side Python packages:

```bash
pip install -r $BRIDGE_ROOT/requirements-bridge.txt
```

Shared path variables:

```bash
export BRIDGE_ROOT=/path/to/IRSIM-3DGS-Bridge
export HABITAT_GS_ROOT=/path/to/habitat-gs
export IRSIM_ROOT=/path/to/ir-sim
export DATA_ROOT=$BRIDGE_ROOT/data
export OUTPUT_ROOT=$BRIDGE_ROOT/outputs
```

The full step-by-step setup is in [docs/setup.md](docs/setup.md).

## Quickstart

Set these paths once:

```bash
export BRIDGE_ROOT=/path/to/IRSIM-3DGS-Bridge
export HABITAT_GS_ROOT=/path/to/habitat-gs
export IRSIM_ROOT=/path/to/ir-sim
export DATA_ROOT=$BRIDGE_ROOT/data
export OUTPUT_ROOT=$BRIDGE_ROOT/outputs
```

### 1. Download the public `scene01` sample

```bash
python $BRIDGE_ROOT/scripts/download_scene01.py \
  --output_dir $DATA_ROOT/gs_scenes
```

This gives you:

- `$DATA_ROOT/gs_scenes/train.scene_dataset_config.json`
- `$DATA_ROOT/gs_scenes/train/scene01/scene01.gs.ply`
- `$DATA_ROOT/gs_scenes/train/scene01/scene01.navmesh`
- `$DATA_ROOT/gs_scenes/train/scene01/scene01_avatar.scene_instance.json`

### 2. Convert 3DGS to an occupancy map

Start with orientation search:

```bash
python $BRIDGE_ROOT/scripts/gs_to_occupancy.py \
  --input $DATA_ROOT/gs_scenes/train/scene01/scene01.gs.ply \
  --output_dir $OUTPUT_ROOT/scene01_orientation_check \
  --resolution 0.05 \
  --robot_radius 0.25 \
  --save_orientation_variants
```

Inspect `orientation_variants.png`, then generate the aligned map:

```bash
python $BRIDGE_ROOT/scripts/gs_to_occupancy.py \
  --input $DATA_ROOT/gs_scenes/train/scene01/scene01.gs.ply \
  --output_dir $OUTPUT_ROOT/scene01_occupancy_aligned \
  --resolution 0.05 \
  --robot_radius 0.25 \
  --image_transform transpose_rotate_180
```

### 3. Export an IR-SIM world

```bash
python $BRIDGE_ROOT/scripts/export_irsim_world_from_occupancy.py \
  --occupancy_dir $OUTPUT_ROOT/scene01_occupancy_aligned \
  --output_dir $OUTPUT_ROOT/scene01_irsim_free_unknown \
  --world_name scene01_gs_irsim_free_unknown \
  --unknown_as free
```

### 4. Run A* and IR-SIM follow

Non-interactive example with fixed start and goal:

```bash
MPLCONFIGDIR=/tmp/matplotlib python $BRIDGE_ROOT/scripts/interactive_astar_irsim.py \
  --world $OUTPUT_ROOT/scene01_irsim_free_unknown/scene01_gs_irsim_free_unknown.yaml \
  --output_dir $OUTPUT_ROOT/manual_click_follow \
  --start 72.8741379310345,95.7265337423313 \
  --goal 79.7,53.3 \
  --planning-inflation-radius 0.6 \
  --follow \
  --irsim_root $IRSIM_ROOT
```

If you want to click start and goal manually, omit `--start` and `--goal`. Add `--display` if you want the IR-SIM GUI during following.

### 5. Convert the IR-SIM trace back to Habitat-GS

Offline conversion:

```bash
python $BRIDGE_ROOT/scripts/convert_irsim_trace_to_gs_trajectory.py \
  --trace $OUTPUT_ROOT/manual_click_follow/irsim_follow_trace.jsonl \
  --map_yaml $OUTPUT_ROOT/scene01_occupancy_aligned/map.yaml \
  --world $OUTPUT_ROOT/manual_click_follow/clicked_start_goal_world.yaml \
  --output_dir $OUTPUT_ROOT/manual_click_follow_gs_sync
```

### 6. Replay inside Habitat-GS

First-person replay:

```bash
cd $HABITAT_GS_ROOT
python examples/gaussian_viewer.py \
  --dataset $DATA_ROOT/gs_scenes/train.scene_dataset_config.json \
  --scene scene01 \
  --agent-trajectory $OUTPUT_ROOT/manual_click_follow_gs_sync/gs_agent_trajectory.jsonl \
  --trajectory-snap-to-navmesh \
  --trajectory-camera-height 1.5
```

Overview replay:

```bash
cd $HABITAT_GS_ROOT
python examples/gaussian_viewer.py \
  --dataset $DATA_ROOT/gs_scenes/train.scene_dataset_config.json \
  --scene scene01 \
  --viewpoint $BRIDGE_ROOT/configs/live_sync/my_overview_viewpoint.json \
  --agent-trajectory $OUTPUT_ROOT/manual_click_follow_gs_sync/gs_agent_trajectory.jsonl \
  --trajectory-snap-to-navmesh \
  --trajectory-camera-height 1.5 \
  --trajectory-overview \
  --hide-text
```

## Live Sync

The live-sync setup uses four cooperating processes:

1. IR-SIM writes `irsim_follow_trace.jsonl`
2. The watcher converts each new row into `gs_live_trajectory.jsonl`
3. Habitat-GS overview viewer reads the growing GS trajectory
4. Habitat-GS first-person viewer reads the same growing GS trajectory

The overview and first-person viewers do not need to be opened at the same time. They are two consumer views over the same trajectory file.

See [docs/pipeline.md](docs/pipeline.md) for the full window-by-window recipe.

## Custom Scenes

For your own GS scene, the minimum path is:

1. Make sure the PLY is Habitat-GS-compatible.
2. Generate or attach a usable navmesh for viewer replay.
3. Run `gs_to_occupancy.py`
4. Validate map orientation
5. Export the IR-SIM world
6. Follow a path in IR-SIM
7. Convert the trace back to GS poses

See [docs/custom_scene.md](docs/custom_scene.md).

## Example Outputs

Lightweight expected outputs are checked in under [examples/expected_outputs](examples/expected_outputs):

- `scene01_occupancy_aligned`
- `scene01_irsim_free_unknown`
- `manual_click_follow_gs_sync`
- `live_sync_repro`

These are not meant to replace the full run. They exist so users can inspect the expected file structure and debug plots before running the pipeline.

## Troubleshooting

Common issues are listed in [docs/troubleshooting.md](docs/troubleshooting.md).

## Script Inventory

- `scripts/download_scene01.py`: fetch the public demo scene
- `scripts/strip_gs_ply_for_habitat.py`: remove unsupported GS fields before Habitat-GS loading
- `scripts/gs_to_occupancy.py`: build a 2D occupancy map from a 3DGS PLY
- `scripts/export_irsim_world_from_occupancy.py`: create an IR-SIM world from the occupancy map
- `scripts/interactive_astar_irsim.py`: A* planning and optional IR-SIM following
- `scripts/convert_irsim_trace_to_gs_trajectory.py`: offline trace conversion
- `scripts/watch_irsim_trace_to_gs_trajectory.py`: live trace conversion
