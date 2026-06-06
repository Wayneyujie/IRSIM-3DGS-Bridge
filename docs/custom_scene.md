# Custom Scenes

This bridge is scene-agnostic as long as you can provide three things:

1. A GS PLY that Habitat-GS can load
2. A navmesh that Habitat-GS can use for replay and snap-to-navmesh
3. A plausible top-down occupancy extraction

## Minimal Path

### 1. Prepare the GS PLY

If your PLY contains unsupported SH-rest fields, strip it:

```bash
python scripts/strip_gs_ply_for_habitat.py \
  --input /path/to/point_cloud.ply \
  --output /path/to/scene_stripped.gs.ply
```

### 2. Register the scene in Habitat-GS

Your local Habitat-GS dataset config needs:

- a scene `.gs.ply`
- a scene `.navmesh`
- optionally a linked collision mesh

This repository does not generate navmeshes by itself. Use your Habitat-GS workflow for that step.

### 3. Build occupancy

```bash
python scripts/gs_to_occupancy.py \
  --input /path/to/scene_stripped.gs.ply \
  --output_dir outputs/custom_orientation_check \
  --resolution 0.05 \
  --robot_radius 0.25 \
  --save_orientation_variants
```

Then re-run with the chosen `--image_transform`.

### 4. Export IR-SIM world

```bash
python scripts/export_irsim_world_from_occupancy.py \
  --occupancy_dir outputs/custom_occupancy \
  --output_dir outputs/custom_irsim \
  --world_name custom_gs_irsim \
  --unknown_as free
```

### 5. Follow a path

```bash
python scripts/interactive_astar_irsim.py \
  --world outputs/custom_irsim/custom_gs_irsim.yaml \
  --output_dir outputs/custom_follow \
  --follow \
  --irsim_root $IRSIM_ROOT
```

### 6. Map the trace back to GS

```bash
python scripts/convert_irsim_trace_to_gs_trajectory.py \
  --trace outputs/custom_follow/irsim_follow_trace.jsonl \
  --map_yaml outputs/custom_occupancy/map.yaml \
  --world outputs/custom_follow/clicked_start_goal_world.yaml \
  --output_dir outputs/custom_gs_sync
```

## About Mesh-Based Pipelines

For some custom scenes you may prefer:

```text
3DGS -> point cloud / mesh -> collision mesh -> navmesh
```

That is a separate concern from the IR-SIM bridge itself. This repository intentionally keeps that part out of scope.
