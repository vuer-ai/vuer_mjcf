# Vuer Examples

Simple examples for loading and visualizing MuJoCo scenes in Vuer.

## Overview

This module provides easy-to-use examples demonstrating how to load various manipulation tasks into Vuer for VR visualization and teleoperation. Each example is self-contained and runnable with minimal setup.

## Quick Start

### Run Individual Examples

Each scene can be run directly:

```bash
# Simple examples
python -m vuer_mjcf.vuer_examples.scenes.simple_pick_place
python -m vuer_mjcf.vuer_examples.scenes.push_t

# Intermediate examples
python -m vuer_mjcf.vuer_examples.scenes.mug_tree
python -m vuer_mjcf.vuer_examples.scenes.microwave_muffin

# Advanced examples
python -m vuer_mjcf.vuer_examples.scenes.tie_knot
python -m vuer_mjcf.vuer_examples.scenes.pour_liquid
python -m vuer_mjcf.vuer_examples.scenes.kitchen_room

# Robot variants
python -m vuer_mjcf.vuer_examples.scenes.mug_tree_ur
python -m vuer_mjcf.vuer_examples.scenes.shadow_hands_kitchen
```

### Use the Master Viewer

The master viewer can be run directly with defaults (loads pick_place scene):

```bash
python -m vuer_mjcf.vuer_examples.viewer
```

Or with command-line arguments for any task:

```bash
python -m vuer_mjcf.vuer_examples.viewer \
    --factory_fn "vuer_mjcf.tasks.mug_tree:make_schema" \
    --name "mug_tree" \
    --actuators "mono"
```

To change the default scene, edit the bottom of `viewer.py` and uncomment your preferred example.

### Programmatic Usage

```python
from vuer_mjcf.vuer_examples.viewer import load_scene

# Load any scene
load_scene(
    factory_fn="vuer_mjcf.tasks.mug_tree:make_schema",
    name="mug_tree",
    actuators="mono",
    show_lights=False,
    verbose=True,
)
```

## Examples Overview

### Simple Examples

**simple_pick_place.py** - Basic object manipulation
- Task: Move a green cube from red area to blue goal area
- Gripper: Single Robotiq 2F-85
- Complexity: ⭐ Beginner

**push_t.py** - Non-prehensile manipulation
- Task: Push and rotate T-shape to target position
- Actuator: Moving cylinder (no grasping)
- Complexity: ⭐ Beginner

### Intermediate Examples

**mug_tree.py** - Precise placement
- Task: Hang mug on mug tree hook
- Gripper: Single Robotiq 2F-85
- Complexity: ⭐⭐ Intermediate

**microwave_muffin.py** - Articulated objects
- Task: Open microwave and place muffin inside
- Gripper: Single Robotiq 2F-85
- Environment: Kitchen scene
- Complexity: ⭐⭐ Intermediate

### Advanced Examples

**tie_knot.py** - Deformable objects
- Task: Tie a knot in a rope
- Grippers: Dual Robotiq 2F-85 (bimanual)
- Physics: Soft-body simulation
- Complexity: ⭐⭐⭐ Advanced

**pour_liquid.py** - Fluid dynamics
- Task: Pour liquid between containers
- Hands: Dual Shadow Hands (bimanual)
- Physics: Particle simulation
- Complexity: ⭐⭐⭐ Advanced

**kitchen_room.py** - Complex environment
- Scene: Full kitchen with appliances and furniture
- Gripper: Single Robotiq 2F-85
- Features: Multiple objects, articulated furniture
- Complexity: ⭐⭐⭐ Advanced

### Robot Variants

**mug_tree_ur.py** - Full robot arm
- Same task as mug_tree but with UR5e arm
- Demonstrates joint control vs floating gripper
- Better for sim-to-real transfer
- Complexity: ⭐⭐ Intermediate

**shadow_hands_kitchen.py** - Dexterous manipulation
- Full kitchen with dual Shadow Hands
- 24+ DOF per hand for fine manipulation
- Bimanual coordination required
- Complexity: ⭐⭐⭐⭐ Expert

## Viewer Parameters

The master viewer (`viewer.py`) supports the following parameters:

- `--factory_fn`: Factory function path (required) - format: "module.path:function_name"
- `--name`: Scene name (optional, derived from factory_fn if not provided)
- `--vuer_port`: Server port (default: 8012)
- `--actuators`: Hand mode - "mono", "duo", or "none" (default: "mono")
- `--show_lights`: Enable scene lighting (default: False for performance)
- `--verbose`: Print verbose output (default: False)

## Accessing Vuer

After starting a scene, you'll see:

```
Vuer Viewer Running!
Visit: https://vuer.ai/editor?ws=ws://localhost:8012
```

Open the URL in a VR-compatible browser or use VR headset to interact with the scene.

## Scene Features

### Actuator Modes

- **mono**: Single hand/gripper control (right hand)
- **duo**: Dual hand/gripper control (both hands)
- **none**: No hand actuators (for robot arms or custom control)

### Common Controls

- **VR Controllers**: Map to gripper/hand positions
- **Trigger Squeeze**: Close gripper or grasp with fingers
- **Movement**: Controller position controls end-effector
- **Pause**: Press space or use VR menu

## Creating Custom Examples

To create your own example:

```python
"""My Custom Scene"""

from vuer_mjcf.vuer_examples.viewer import load_scene

def main():
    load_scene(
        factory_fn="vuer_mjcf.tasks.my_task:make_schema",
        name="my_scene",
        actuators="mono",
        verbose=True,
    )

if __name__ == "__main__":
    print("My Custom Scene")
    main()
```

## Troubleshooting

**Port already in use**: Change the port with `--vuer_port`
```bash
python -m vuer_mjcf.vuer_examples.scenes.mug_tree --vuer_port 8013
```

**Assets not loading**: Ensure you're running from the correct directory with access to asset folders

**Slow rendering**: Disable lights with `--show_lights=False` (default)

**VR not connecting**: Check your VR headset browser supports WebXR

## Directory Structure

```
vuer_mjcf/vuer_examples/
├── __init__.py
├── README.md                    # This file
├── viewer.py                    # Master viewer
└── scenes/
    ├── __init__.py
    ├── simple_pick_place.py     # Simple examples
    ├── push_t.py
    ├── mug_tree.py              # Intermediate
    ├── microwave_muffin.py
    ├── tie_knot.py              # Advanced
    ├── pour_liquid.py
    ├── kitchen_room.py
    ├── mug_tree_ur.py           # Robot variants
    └── shadow_hands_kitchen.py
```

## Further Reading

- Main documentation: See project README
- Task definitions: `vuer_mjcf/tasks/`
- Stage sets: `vuer_mjcf/stage_sets/`
- Reference implementation: See the original `collect_demo.py` for data collection features
