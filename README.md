# IRSIM-3DGS-Bridge

Bridge scripts for a practical closed loop:

`3D Gaussian Splatting -> 2D occupancy -> IR-SIM planning/following -> Habitat-GS replay`

<p align="center">
  <img src="assets/demo.gif" alt="IRSIM-3DGS-Bridge demo" width="900" />
</p>

<p align="center">
  <a href="assets/demo.mp4">Download demo video</a>
</p>

## Why This Repository Exists

This repository focuses on the glue, not the full upstream stacks.

It gives you:

- A public `scene01` demo that starts from downloading a GS scene
- A minimal closed loop from `3DGS -> IR-SIM -> Habitat-GS`
- A one-command demo runner with repository defaults
- A live-sync workflow where IR-SIM motion is reflected back into Habitat-GS

It does not:

- train 3DGS scenes
- vendor Habitat-GS
- vendor IR-SIM
- generate collision meshes for arbitrary custom scenes

## What You See After Running It

- A top-down occupancy map extracted from `scene01.gs.ply`
- An IR-SIM world generated from that map
- An A* path plus an `irsim_follow_trace.jsonl`
- A converted `gs_agent_trajectory.jsonl`
- Habitat-GS replay in either first-person or overview mode

Example checked-in artifacts live under [examples/expected_outputs](examples/expected_outputs).

## Tested Versions

This bridge was validated against:

- `habitat-gs` commit `eb322e97772dedd00e36c4267dbc5619d0bffa52`
- `ir-sim` commit `201244932d60942e8f214757bb01ca10373c1e5c`

`IR-SIM` can be installed directly from PyPI for the quick demo. `Habitat-GS` is still the heavy dependency and should be installed separately.

## Fastest Path

### 1. Install the bridge-side Python stack

This installs:

- `ir-sim[all]`
- the bridge Python dependencies from [requirements-bridge.txt](requirements-bridge.txt)

```bash
bash scripts/install_bridge_python.sh
```

If you want a non-default interpreter:

```bash
bash scripts/install_bridge_python.sh --python /path/to/python
```

### 2. Save your local paths once

This writes a reusable `.bridge.env` file in the repository root.

```bash
bash scripts/write_bridge_env.sh \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python /path/to/habitat-gs-env/bin/python
```

Then load it:

```bash
source .bridge.env
```

### 3. Run the full public `scene01` demo

This single command:

- downloads `scene01` if missing
- builds the aligned occupancy map
- exports the IR-SIM world
- runs A* plus IR-SIM following
- converts the follow trace back into Habitat-GS poses

```bash
bash scripts/run_scene01_demo.sh
```

Outputs go to:

```text
outputs/quickstart/scene01/
```

### 4. Open the final effect in Habitat-GS

First-person replay:

```bash
bash scripts/run_scene01_demo.sh --viewer first_person
```

Overview replay:

```bash
bash scripts/run_scene01_demo.sh --viewer overview
```

If you prefer not to save `.bridge.env`, you can pass the paths inline:

```bash
bash scripts/run_scene01_demo.sh \
  --viewer first_person \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python /path/to/habitat-gs-env/bin/python
```

## Quick Demo Defaults

The one-command demo intentionally hardcodes the known-good public `scene01` settings:

- `resolution=0.05`
- `robot_radius=0.25`
- `image_transform=transpose_rotate_180`
- fixed start: `72.8741379310345,95.7265337423313`
- fixed goal: `79.7,53.3`
- planning inflation radius: `0.6`
- replay camera height: `1.5`

Those defaults keep the homepage demo short and reproducible. The detailed knobs remain available in the Python scripts and docs.

## Repository Layout

```text
scripts/
  install_bridge_python.sh
  write_bridge_env.sh
  run_scene01_demo.sh
  gs_to_occupancy.py
  export_irsim_world_from_occupancy.py
  interactive_astar_irsim.py
  convert_irsim_trace_to_gs_trajectory.py
  watch_irsim_trace_to_gs_trajectory.py

configs/live_sync/
  my_overview_viewpoint.json

examples/expected_outputs/
  scene01_occupancy_aligned/
  scene01_irsim_free_unknown/
  manual_click_follow_gs_sync/
  live_sync_repro/
```

## When You Want More Control

The homepage keeps the path short on purpose. Use the docs when you want to customize things:

- [docs/setup.md](docs/setup.md): detailed environment setup and version notes
- [docs/pipeline.md](docs/pipeline.md): full live-sync multi-window workflow
- [docs/custom_scene.md](docs/custom_scene.md): adapting the bridge to your own GS scene
- [docs/troubleshooting.md](docs/troubleshooting.md): common failure modes

## Notes

- `IRSIM_ROOT` is optional for the quick demo when `ir-sim` is installed from PyPI.
- `Habitat-GS` remains the main heavyweight dependency. The quick scripts assume you already have a working `gaussian_viewer.py`.
- If you downloaded only `scene01`, Habitat-GS may still warn about missing `scene02` to `scene55` navmeshes in the shared dataset config. That is expected as long as `scene01` itself loads.
