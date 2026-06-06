# Pipeline

This document describes the intended closed loop and the practical viewer setup.

## Data Flow

```text
scene01.gs.ply
  -> gs_to_occupancy.py
  -> scene01_occupancy_aligned/map.yaml + map.png
  -> export_irsim_world_from_occupancy.py
  -> scene01_gs_irsim_free_unknown.yaml
  -> interactive_astar_irsim.py --follow
  -> irsim_follow_trace.jsonl
  -> convert_irsim_trace_to_gs_trajectory.py
  -> gs_agent_trajectory.jsonl
  -> gaussian_viewer.py --agent-trajectory
```

## Live Sync Loop

The real-time bridge uses a growing IR-SIM trace file and a growing Habitat-GS trajectory file.

```text
interactive_astar_irsim.py --follow --display
  writes irsim_follow_trace.jsonl

watch_irsim_trace_to_gs_trajectory.py
  reads irsim_follow_trace.jsonl
  writes gs_live_trajectory.jsonl

gaussian_viewer.py --agent-trajectory-live
  reads gs_live_trajectory.jsonl
```

## Four-Window Setup

### Window 1: watcher

```bash
MPLCONFIGDIR=/tmp/matplotlib python $BRIDGE_ROOT/scripts/watch_irsim_trace_to_gs_trajectory.py \
  --trace $OUTPUT_ROOT/live_sync/irsim_follow_trace.jsonl \
  --map_yaml $OUTPUT_ROOT/scene01_occupancy_aligned/map.yaml \
  --world $OUTPUT_ROOT/live_sync/clicked_start_goal_world.yaml \
  --output $OUTPUT_ROOT/live_sync/gs_live_trajectory.jsonl \
  --overwrite
```

### Window 2: fixed overview view

```bash
cd $HABITAT_GS_ROOT
python examples/gaussian_viewer.py \
  --dataset $DATA_ROOT/gs_scenes/train.scene_dataset_config.json \
  --scene scene01 \
  --viewpoint $BRIDGE_ROOT/configs/live_sync/my_overview_viewpoint.json \
  --agent-trajectory $OUTPUT_ROOT/live_sync/gs_live_trajectory.jsonl \
  --agent-trajectory-live \
  --trajectory-snap-to-navmesh \
  --trajectory-camera-height 1.5 \
  --trajectory-overview \
  --hide-text
```

### Window 3: first-person view

```bash
cd $HABITAT_GS_ROOT
python examples/gaussian_viewer.py \
  --dataset $DATA_ROOT/gs_scenes/train.scene_dataset_config.json \
  --scene scene01 \
  --agent-trajectory $OUTPUT_ROOT/live_sync/gs_live_trajectory.jsonl \
  --agent-trajectory-live \
  --trajectory-snap-to-navmesh \
  --trajectory-camera-height 1.5
```

### Window 4: IR-SIM planning and following

```bash
MPLCONFIGDIR=/tmp/matplotlib python $BRIDGE_ROOT/scripts/interactive_astar_irsim.py \
  --world $OUTPUT_ROOT/edited_maps/scene01_manual_world.yaml \
  --output_dir $OUTPUT_ROOT/live_sync \
  --start 72.8741379310345,95.7265337423313 \
  --goal 79.7,53.3 \
  --planning-inflation-radius 0.6 \
  --follow \
  --display \
  --irsim_root $IRSIM_ROOT
```

## Notes

- Window 2 and Window 3 are two different camera modes over the same live trajectory file.
- They do not need to be open simultaneously.
- If you restart the IR-SIM run, clear the previous `trace`, `clicked_start_goal_world.yaml`, and output trajectory before relaunching, or use `--overwrite` on the watcher.
