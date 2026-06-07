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

This repository gives you:

- A public `scene01` demo that starts from downloading a GS scene
- A minimal closed loop from `3DGS -> IR-SIM -> Habitat-GS`
- A one-command demo runner with repository defaults
- A live-sync workflow where IR-SIM motion is reflected back into Habitat-GS

## What You See After Running It

- A top-down occupancy map extracted from `scene01.gs.ply`
- An IR-SIM world generated from that map
- An A* path plus an `irsim_follow_trace.jsonl`
- A converted `gs_agent_trajectory.jsonl`
- Habitat-GS first-person replay inside the GS scene

Example checked-in artifacts live under [examples/expected_outputs](examples/expected_outputs).

## Fastest Path

### 1. Clone the repository

```bash
git clone https://github.com/Wayneyujie/IRSIM-3DGS-Bridge.git
cd IRSIM-3DGS-Bridge
```

### 2. Install the bridge-side Python stack

This installs:

- `ir-sim[all]`
- the bridge Python dependencies from [requirements-bridge.txt](requirements-bridge.txt)

```bash
bash scripts/install_bridge_python.sh
```

If you are not using the shell's default `python`, pass the exact interpreter you want to install into.

Example:

```bash
bash scripts/install_bridge_python.sh --python /path/to/conda-env/bin/python
```

### 3. Save your local paths once

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

### 4. Run the full public `scene01` demo

This single command:

- downloads `scene01` if missing
- builds the aligned occupancy map
- exports the IR-SIM world
- runs A* plus IR-SIM following
- converts the follow trace back into Habitat-GS poses
- opens Habitat-GS in first-person replay mode

```bash
bash scripts/run_scene01_demo.sh
```

The final viewer step uses the Habitat-GS paths saved in `.bridge.env`.

Outputs are also saved under:

```text
outputs/quickstart/scene01/
```

### 5. Open the overview camera instead

```bash
bash scripts/run_scene01_demo.sh --viewer overview
```

### 6. If you only want the intermediate bridge outputs

```bash
bash scripts/run_scene01_demo.sh --prepare-only
```

### 7. If you prefer not to save `.bridge.env`

```bash
bash scripts/run_scene01_demo.sh \
  --viewer first_person \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python /path/to/habitat-gs-env/bin/python
```

## What The One-Command Demo Uses

The one-command demo intentionally hardcodes the known-good public `scene01` settings:

- `resolution=0.05`
- `robot_radius=0.25`
- `image_transform=transpose_rotate_180`
- fixed start: `72.8741379310345,95.7265337423313`
- fixed goal: `79.7,53.3`
- planning inflation radius: `0.6`
- replay camera height: `1.5`

Those defaults keep the homepage demo short and reproducible. The detailed knobs remain available in the Python scripts and docs.

## What You Need Before Running The Viewer

- A working local `habitat-gs`
- A working `gaussian_viewer.py`
- The path to your Habitat-GS Python interpreter

The quick demo installs the IR-SIM side directly from `pip install ir-sim[all]`, but the final one-command effect still depends on your local Habitat-GS viewer.

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
- `HABITAT_PYTHON` means the Python executable inside the environment where Habitat-GS already works.
- Example: `/path/to/miniconda3/envs/habitat-gs/bin/python`
