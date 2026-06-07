# Troubleshooting

## Occupancy map is mirrored or rotated

Run:

```bash
python scripts/gs_to_occupancy.py \
  --input /path/to/scene.gs.ply \
  --output_dir outputs/orientation_check \
  --save_orientation_variants
```

Then inspect `orientation_variants.png` and re-run with the matching `--image_transform`.

## Almost everything is unknown

Check:

- `floor_z`
- `ground_min_height`
- `ground_max_height`
- `opacity_threshold`

Also inspect:

- `debug_topdown.png`
- `height_histogram.png`

## IR-SIM follower clips obstacles

Increase:

- `--planning-inflation-radius` in `interactive_astar_irsim.py`

Typical useful values are `0.4` to `0.8`.

## The watcher is waiting forever for the world file

The watcher expects `clicked_start_goal_world.yaml` to appear after the follow script starts. Make sure:

- `interactive_astar_irsim.py --follow` is actually running
- both commands point to the same `output_dir`

## The watcher is waiting forever for the trace file

The trace file is only created during the follow loop. Check:

- whether `ir-sim` is installed in the active Python interpreter
- IR-SIM import path via `--irsim_root` or `$IRSIM_ROOT` when using a source checkout
- whether the follow process exited early

## `import irsim` fails even though bridge requirements were installed

Installing only:

```bash
pip install -r requirements-bridge.txt
```

is not enough for the IR-SIM runtime environment.

Install IR-SIM itself first. The fastest option is:

```bash
pip install "ir-sim[all]"
```

Or install from source:

```bash
git clone https://github.com/hanruihua/ir-sim.git
cd ir-sim
pip install -e .
```

Then install the bridge requirements.

## Habitat-GS does not load the PLY

Try stripping the PLY first:

```bash
python scripts/strip_gs_ply_for_habitat.py \
  --input /path/to/point_cloud.ply \
  --output /path/to/scene.gs.ply
```

## Habitat-GS prints many missing navmesh warnings for scenes you did not download

If you downloaded only `scene01` from the public GS scenes dataset, Habitat-GS may still validate all scene entries in `train.scene_dataset_config.json`.

Warnings for missing `scene02` to `scene55` navmeshes are expected in that case.

What matters is whether the specific scene you launched, for example:

```bash
python examples/gaussian_viewer.py \
  --dataset /path/to/train.scene_dataset_config.json \
  --scene scene01
```

actually initializes successfully.

## `gaussian_viewer.py` fails on `from magnum import shaders, text`

This is a Habitat-GS environment issue, not a bridge logic issue.

Typical cause:

- an incompatible `magnum` package is installed in the active Python environment

Check:

```bash
conda activate habitat-gs
python $HABITAT_GS_ROOT/examples/gaussian_viewer.py --help
```

If that fails before any bridge script is involved, repair the Habitat-GS viewer environment first.

## First-person camera is too high or too low

Adjust:

- `--camera_height` during conversion
- `--trajectory-camera-height` during viewer playback

The demo workflow uses `1.5`.
