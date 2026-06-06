# Setup

This repository is easiest to use with two separate conda environments:

- `habitat-gs`
- `irsim_latest`

That mirrors the workflow used during development:

- `habitat-gs` runs Habitat-GS itself and the 3DGS-side bridge scripts
- `irsim_latest` runs IR-SIM following and the live watcher

## Recommended Layout

```text
/path/to/habitat-gs
/path/to/ir-sim
/path/to/IRSIM-3DGS-Bridge
```

Export these paths in every shell session or put them in your shell rc:

```bash
export BRIDGE_ROOT=/path/to/IRSIM-3DGS-Bridge
export HABITAT_GS_ROOT=/path/to/habitat-gs
export IRSIM_ROOT=/path/to/ir-sim
export DATA_ROOT=$BRIDGE_ROOT/data
export OUTPUT_ROOT=$BRIDGE_ROOT/outputs
```

## Environment 1: `habitat-gs`

Use this environment for:

- `examples/gaussian_viewer.py`
- `scripts/gs_to_occupancy.py`
- `scripts/export_irsim_world_from_occupancy.py`
- `scripts/convert_irsim_trace_to_gs_trajectory.py`
- `scripts/strip_gs_ply_for_habitat.py`

### Create the environment

```bash
conda create -n habitat-gs python=3.12 cmake=3.27 -y
conda activate habitat-gs
python -m pip install --upgrade pip setuptools wheel ninja
```

### Install PyTorch

Choose the wheel index appropriate for your CUDA setup. One working example was:

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

### Build Habitat-GS

Example flow:

```bash
git clone https://github.com/zju3dv/habitat-gs.git
cd habitat-gs
git submodule sync --recursive
git submodule update --init --recursive --jobs 1
```

If you need CUDA builds, export your CUDA toolchain first. One known working setup used:

```bash
export CUDA_HOME=/usr/local/cuda-12.4
export CUDA_PATH=/usr/local/cuda-12.4
export CUDACXX=/usr/local/cuda-12.4/bin/nvcc
export PATH=/usr/local/cuda-12.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH
export CMAKE_ARGS="-DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.4/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=89"
```

Then install:

```bash
HABITAT_WITH_CUDA=ON HABITAT_WITH_BULLET=OFF pip install -e . --no-build-isolation
```

If your setup differs, follow the official Habitat-GS instructions first and use this repository only after `gaussian_viewer.py` works.

### Install bridge-side dependencies into `habitat-gs`

```bash
pip install -r $BRIDGE_ROOT/requirements-bridge.txt
```

### Optional: download the public demo scene

```bash
python $BRIDGE_ROOT/scripts/download_scene01.py \
  --output_dir $DATA_ROOT/gs_scenes
```

## Environment 2: `irsim_latest`

Use this environment for:

- `scripts/interactive_astar_irsim.py --follow`
- `scripts/watch_irsim_trace_to_gs_trajectory.py`

You can use a separate environment because IR-SIM runtime constraints often differ from Habitat-GS.

### Create the environment

```bash
conda create -n irsim_latest python=3.10 -y
conda activate irsim_latest
python -m pip install --upgrade pip setuptools wheel
```

### Install IR-SIM

This repository does not vendor IR-SIM. Point `IRSIM_ROOT` at your local IR-SIM checkout:

```bash
export IRSIM_ROOT=/path/to/ir-sim
```

Install whatever IR-SIM itself requires according to its own repository instructions. After that, install the bridge-side dependencies needed by the watcher and plotting:

```bash
pip install -r $BRIDGE_ROOT/requirements-bridge.txt
```

If IR-SIM is not installed as a normal package, that is fine. The bridge uses:

```bash
python $BRIDGE_ROOT/scripts/interactive_astar_irsim.py --irsim_root $IRSIM_ROOT ...
```

and imports IR-SIM from that path.

## Which Script Runs in Which Environment

### Run in `habitat-gs`

```text
scripts/download_scene01.py
scripts/strip_gs_ply_for_habitat.py
scripts/gs_to_occupancy.py
scripts/export_irsim_world_from_occupancy.py
scripts/convert_irsim_trace_to_gs_trajectory.py
examples/gaussian_viewer.py
```

### Run in `irsim_latest`

```text
scripts/interactive_astar_irsim.py
scripts/watch_irsim_trace_to_gs_trajectory.py
```

## Sanity Checks

### Habitat-GS side

```bash
conda activate habitat-gs
cd $HABITAT_GS_ROOT
python examples/gaussian_viewer.py --help
```

If this fails at:

```text
from magnum import shaders, text
```

then your Python environment likely has an incompatible `magnum` package on `PYTHONPATH` or in the active site-packages. Fix the Habitat-GS viewer environment first before debugging the bridge.

### IR-SIM side

```bash
conda activate irsim_latest
python $BRIDGE_ROOT/scripts/interactive_astar_irsim.py --help
python $BRIDGE_ROOT/scripts/watch_irsim_trace_to_gs_trajectory.py --help
```

### Bridge-side Python imports

```bash
python - <<'PY'
import cv2, yaml, numpy, scipy, matplotlib
print("bridge deps ok")
PY
```

## Notes

- The offline bridge steps can often run entirely inside `habitat-gs`.
- The live-sync path is usually simpler if the watcher runs in `irsim_latest`, alongside the IR-SIM follow process.
- If you want a single environment instead of two, that can work, but this repository documents the two-environment approach because it was more stable in practice.
