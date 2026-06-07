# Setup

This repository supports two setup modes.

## Mode A: Single-Environment Demo

Use this if your goal is simple:

- get the public `scene01` demo running
- end in Habitat-GS first-person replay
- keep the README path short

In this mode, you use your existing `habitat-gs` environment for everything:

- Habitat-GS viewer
- occupancy export
- IR-SIM follow
- trajectory conversion

That is the recommended path for first-time users of this repository.

From the repository root:

```bash
conda activate habitat-gs
bash scripts/install_bridge_python.sh
bash scripts/write_bridge_env.sh \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python "$(which python)"
source .bridge.env
bash scripts/run_scene01_demo.sh
```

## Mode B: Two-Environment Workflow

Use this if your goal is more advanced:

- keep Habitat-GS isolated
- keep IR-SIM isolated
- run the multi-window live-sync workflow
- avoid dependency pollution while developing

This is the workflow used during development:

- `habitat-gs`
- `irsim_latest`

- `habitat-gs` runs Habitat-GS itself and the 3DGS-side bridge scripts
- `irsim_latest` runs IR-SIM following and the live watcher

Validated references:

- `habitat-gs`: `eb322e97772dedd00e36c4267dbc5619d0bffa52`
- `ir-sim`: `201244932d60942e8f214757bb01ca10373c1e5c`

## Fast Path For Advanced Users

If you already know you want the two-environment workflow, the shortest path is:

```bash
conda activate habitat-gs
bash scripts/install_bridge_python.sh --skip-irsim --python "$(which python)"
conda activate irsim_latest
bash scripts/install_bridge_python.sh --python "$(which python)"
bash scripts/write_bridge_env.sh \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python /path/to/habitat-gs-env/bin/python \
  --irsim-root /path/to/ir-sim
source .bridge.env
```

The rest of this page explains the detailed manual setup for the two-environment workflow.

## Recommended Layout

```text
/path/to/habitat-gs
/path/to/ir-sim
/path/to/IRSIM-3DGS-Bridge
```

Export these paths in every shell session, put them in your shell rc, or generate them with:

```bash
bash scripts/write_bridge_env.sh \
  --habitat-gs-root /path/to/habitat-gs \
  --habitat-python /path/to/habitat-gs-env/bin/python
source .bridge.env
```

Manual form:

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

### Install PyTorch and basic build dependencies

Choose the wheel index appropriate for your CUDA setup. One working example was:

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

Additional packages that were useful during setup:

```bash
python -m pip install numpy==2.3.5 scipy pillow==10.4.0
python -m pip install scikit-build-core pybind11 ninja cmake
```

### Build Habitat-GS

Example flow:

```bash
git clone https://github.com/zju3dv/habitat-gs.git
cd habitat-gs
git checkout eb322e97772dedd00e36c4267dbc5619d0bffa52
git submodule sync --recursive
git submodule update --init --recursive --jobs 1
```

If submodules still look incomplete, verify with:

```bash
git submodule status --recursive | grep '^-'
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

Then clean any stale build products and install:

```bash
rm -rf build _skbuild
HABITAT_WITH_CUDA=ON HABITAT_WITH_BULLET=OFF pip install -e . --no-build-isolation
```

The development flow that led to a working build was effectively:

```bash
HABITAT_WITH_CUDA=ON HABITAT_WITH_BULLET=OFF pip install -e .
```

followed by a rebuild with explicit CUDA variables and:

```bash
rm -rf build _skbuild
HABITAT_WITH_CUDA=ON HABITAT_WITH_BULLET=OFF pip install -e . --no-build-isolation
```

If your setup differs, follow the official Habitat-GS instructions first and use this repository only after `gaussian_viewer.py` works.

### Install bridge-side dependencies into `habitat-gs`

```bash
pip install -r $BRIDGE_ROOT/requirements-bridge.txt
```

Or from the repository root:

```bash
bash scripts/install_bridge_python.sh --skip-irsim --python /path/to/habitat-gs-env/bin/python
```

### Optional: download the public demo scene

```bash
python $BRIDGE_ROOT/scripts/download_scene01.py \
  --output_dir $DATA_ROOT/gs_scenes
```

### Habitat-GS verification checklist

Before using the bridge, verify the Habitat-GS side in this order:

1. CUDA is visible to PyTorch

```bash
python - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("cuda device:", torch.cuda.get_device_name(0))
PY
```

2. Habitat-Sim imports cleanly

```bash
python - <<'PY'
import habitat_sim
print("habitat_sim import ok")
PY
```

3. The viewer CLI at least parses

```bash
cd $HABITAT_GS_ROOT
python examples/gaussian_viewer.py --help
```

4. The public `scene01` can be loaded once the data is present

```bash
cd $HABITAT_GS_ROOT
python examples/gaussian_viewer.py \
  --dataset $DATA_ROOT/gs_scenes/train.scene_dataset_config.json \
  --scene scene01
```

If step 4 works, the bridge has a valid 3DGS-side consumer.

If you downloaded only `scene01`, Habitat-GS may still print validation warnings for `scene02` to `scene55` navmesh entries referenced in `train.scene_dataset_config.json`. Those warnings are expected as long as the specific scene you actually launch, such as `scene01`, initializes successfully.

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

The upstream IR-SIM repository documents three installation styles:

- `pip install ir-sim`
- `pip install ir-sim[all]`
- source install with `pip install -e .`

For the bridge quick demo, pip installation is the lowest-friction option:

```bash
pip install "ir-sim[all]"
```

For bridge development, a source checkout is still practical:

```bash
git clone https://github.com/hanruihua/ir-sim.git
cd ir-sim
pip install -e .
```

If you use a source checkout and want the bridge to import IR-SIM from there directly, point `IRSIM_ROOT` at it:

```bash
export IRSIM_ROOT=/path/to/ir-sim
```

Do not rely on bridge-side dependencies alone for the IR-SIM environment. In testing, installing only `requirements-bridge.txt` was not enough because upstream IR-SIM runtime dependencies such as `shapely` were still missing.

After that, install the bridge-side dependencies needed by the watcher and plotting:

```bash
pip install -r $BRIDGE_ROOT/requirements-bridge.txt
```

If IR-SIM is not installed as a normal package, that is fine. The bridge can also use:

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
cd $IRSIM_ROOT
python - <<'PY'
import irsim
print("irsim import ok")
PY
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
