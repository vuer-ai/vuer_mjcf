# Vuer Examples Quick Start Guide

## What is This?

This folder contains simplified examples for loading MuJoCo manipulation tasks into Vuer for VR visualization and teleoperation. Each example is ready to run with minimal setup.

## Quick Start (3 Steps)

### 1. List all examples

```bash
python -m vuer_mjcf.vuer_examples.list_examples
```

### 2. Run an example

```bash
# Start with simple pick and place
python -m vuer_mjcf.vuer_examples.scenes.simple_pick_place
```

### 3. Open in browser

When the scene starts, you'll see:
```
Visit: https://vuer.ai/editor?ws=ws://localhost:8012
```

Open this URL in your browser (VR headset browser for full VR experience).

## Examples by Difficulty

### ⭐ Beginner
- `simple_pick_place` - Move a cube between areas
- `push_t` - Push a T-shape to target

### ⭐⭐ Intermediate
- `mug_tree` - Hang a mug on tree
- `microwave_muffin` - Open appliance and place object
- `mug_tree_ur` - Same task with full robot arm

### ⭐⭐⭐ Advanced
- `tie_knot` - Tie rope with bimanual control
- `pour_liquid` - Fluid dynamics with particles
- `kitchen_room` - Full kitchen environment

### ⭐⭐⭐⭐ Expert
- `shadow_hands_kitchen` - Dexterous bimanual manipulation

## Files in This Folder

- `viewer.py` - Master scene loader (use this to load any task)
- `list_examples.py` - Lists all available examples
- `scenes/` - Individual runnable example scripts
- `README.md` - Full documentation

## Master Viewer Usage

Run with defaults (loads pick_place scene):

```bash
python -m vuer_mjcf.vuer_examples.viewer
```

Load any task using command-line arguments:

```bash
python -m vuer_mjcf.vuer_examples.viewer \
    --factory_fn "vuer_mjcf.tasks.mug_tree:make_schema" \
    --name "mug_tree" \
    --actuators "mono" \
    --verbose
```

**Tip**: Edit the bottom of `viewer.py` to change the default scene!

## Common Options

- `--actuators mono` - Single hand (right)
- `--actuators duo` - Both hands (bimanual)
- `--actuators none` - No hands (for robot arms)
- `--show_lights` - Enable lighting (slower)
- `--verbose` - Print debug info
- `--vuer_port 8012` - Change server port

## Programmatic Usage

```python
from vuer_mjcf.vuer_examples.viewer import load_scene

load_scene(
    factory_fn="vuer_mjcf.tasks.mug_tree:make_schema",
    name="mug_tree",
    actuators="mono",
)
```

## Compared to `collect_demo.py`

The original `collect_demo.py` script includes:
- ✅ Scene loading (we have this)
- ✅ VR visualization (we have this)
- ✅ Hand actuators (we have this)
- ❌ Demo recording (removed for simplicity)
- ❌ Trajectory logging (removed)
- ❌ Reset/record buttons (removed)
- ❌ ml-logger integration (removed)

These examples focus on **visualization only**. For data collection, use the original `collect_demo.py`.

## Need Help?

1. **See all examples**: `python -m vuer_mjcf.vuer_examples.list_examples`
2. **Read full docs**: `cat vuer_mjcf/vuer_examples/README.md`
3. **Port in use**: Add `--vuer_port 8013` to change port
4. **Assets missing**: Ensure you have the required asset folders

## Example Comparison

| Example | Gripper Type | # of Hands | Special Features |
|---------|-------------|-----------|------------------|
| simple_pick_place | Robotiq | 1 | Basic grasping |
| push_t | None | 0 | Non-prehensile |
| mug_tree | Robotiq | 1 | Precise placement |
| microwave_muffin | Robotiq | 1 | Articulated objects |
| tie_knot | Robotiq | 2 | Soft body physics |
| pour_liquid | Shadow Hand | 2 | Particle simulation |
| kitchen_room | Robotiq | 1 | Complex scene |
| mug_tree_ur | Robotiq (UR5) | 1 | Full robot arm |
| shadow_hands_kitchen | Shadow Hand | 2 | Dexterous + complex |
