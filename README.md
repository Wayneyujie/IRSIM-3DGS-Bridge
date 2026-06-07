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

This repository gives you a public `scene01` demo, a minimal `3DGS -> IR-SIM -> Habitat-GS` closed loop, and a live-sync workflow for replaying IR-SIM motion back inside Habitat-GS.

## What You See After Running It

After the demo finishes, you get:

- an occupancy map from `scene01.gs.ply`
- an IR-SIM world built from that map
- an A* path and `irsim_follow_trace.jsonl`
- a converted `gs_agent_trajectory.jsonl`
- Habitat-GS first-person replay inside the GS scene

Example checked-in artifacts live under [examples/expected_outputs](examples/expected_outputs).

## Fastest Path

### 0. Before you start: Habitat-GS must already work

Install Habitat-GS first, then come back here.

The one-command demo ends by opening `gaussian_viewer.py` in Habitat-GS first-person mode, so you need:

- a working local `habitat-gs`
- a working `gaussian_viewer.py`
- the Python executable inside that working Habitat-GS environment

Check that first:

```bash
cd /path/to/habitat-gs
/path/to/habitat-gs-env/bin/python examples/gaussian_viewer.py --help
```

If that command does not work yet, stop here and set up Habitat-GS first. A tested setup flow is in [docs/setup.md](docs/setup.md).

### 1. Clone the repository

```bash
git clone https://github.com/Wayneyujie/IRSIM-3DGS-Bridge.git
cd IRSIM-3DGS-Bridge
```

### 2. Activate your existing `habitat-gs` environment

```bash
conda activate habitat-gs
```

Then make sure `python` is the same one that already runs `gaussian_viewer.py`:

```bash
which python
```

### 3. Install IR-SIM and the bridge Python packages into that same environment

This puts `ir-sim[all]` and the bridge Python packages into your current `habitat-gs` environment.

```bash
bash scripts/install_bridge_python.sh
```

If your current shell is using the wrong `python`, point the script at the Habitat-GS one directly:

```bash
bash scripts/install_bridge_python.sh --python /path/to/conda-env/bin/python
```

### 4. Save your local paths once

This writes a small `.bridge.env` file in the repository root so the demo script knows:

- where `habitat-gs` is
- which Python should launch `gaussian_viewer.py`

```bash
bash scripts/write_bridge_env.sh \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python /path/to/habitat-gs-env/bin/python
```

Example:

```bash
bash scripts/write_bridge_env.sh \
  --habitat-gs-root /home/you/habitat-gs \
  --habitat-python /home/you/miniconda3/envs/habitat-gs/bin/python
```

Then load it:

```bash
source .bridge.env
```

That just means: load those saved paths into the current terminal so the next command can use them.

### 5. Run the full public `scene01` demo

This one command does the whole public `scene01` flow and ends by opening Habitat-GS in first-person replay mode:

```bash
bash scripts/run_scene01_demo.sh
```

Outputs are also saved under:

```text
outputs/quickstart/scene01/
```

### 6. Open the overview camera instead

```bash
bash scripts/run_scene01_demo.sh --viewer overview
```

### 7. If you only want the intermediate bridge outputs

```bash
bash scripts/run_scene01_demo.sh --prepare-only
```

### 8. If you prefer not to save `.bridge.env`

```bash
bash scripts/run_scene01_demo.sh \
  --viewer first_person \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python /path/to/habitat-gs-env/bin/python
```

## What The One-Command Demo Uses

The one-command demo uses fixed `scene01` settings on purpose:

- `resolution=0.05`
- `robot_radius=0.25`
- `image_transform=transpose_rotate_180`
- fixed start: `72.8741379310345,95.7265337423313`
- fixed goal: `79.7,53.3`
- planning inflation radius: `0.6`
- replay camera height: `1.5`

That keeps the homepage demo short and repeatable. If you want to change the knobs later, they are still available in the scripts and docs.

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

The homepage keeps the path short on purpose. For everything more advanced, use:

- [docs/setup.md](docs/setup.md): detailed setup, including the advanced two-environment workflow (`habitat-gs` + `irsim_latest`)
- [docs/pipeline.md](docs/pipeline.md): full live-sync multi-window workflow
- [docs/custom_scene.md](docs/custom_scene.md): adapting the bridge to your own GS scene
- [docs/troubleshooting.md](docs/troubleshooting.md): common failure modes

## Notes

- `IRSIM_ROOT` is optional for the quick demo when `ir-sim` is installed from PyPI.
- `HABITAT_PYTHON` is just the Python inside the environment where Habitat-GS already works.
- Example: `/path/to/miniconda3/envs/habitat-gs/bin/python`
