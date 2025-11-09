# Vuer Examples - Scene Viewer

Simple VR viewer for visualizing MuJoCo scenes in Vuer. **This is a visualization-only subset** - it does not include data collection, trajectory recording, or other advanced features.

## What This Is

A lightweight viewer for loading and interacting with MuJoCo scenes in VR:
- ✅ Load any task scene into Vuer for VR visualization
- ✅ VR hand/gripper control via controllers
- ✅ Reset button to reload scenes
- ✅ 9 pre-configured example scenes
- ✅ Programmatic API for custom scenes

## What This Is NOT

This viewer does **not** include:
- ❌ Data collection or trajectory recording
- ❌ Demo playback
- ❌ Reward computation
- ❌ Task success detection
- ❌ Automated demonstrations
- ❌ Interactive example browser (planned but not implemented)

For full data collection features, use the complete `collect_demo.py` script from the main repository.

## Quick Start

### Run Individual Scene Examples

Each scene can be run directly as a Python module:

```bash
# Simple examples
python -m vuer_examples.scenes.simple_pick_place
python -m vuer_examples.scenes.push_t

# Intermediate examples
python -m vuer_examples.scenes.mug_tree
python -m vuer_examples.scenes.microwave_muffin

# Advanced examples
python -m vuer_examples.scenes.tie_knot
python -m vuer_examples.scenes.pour_liquid
python -m vuer_examples.scenes.kitchen_room

# Robot variants
python -m vuer_examples.scenes.mug_tree_ur
python -m vuer_examples.scenes.shadow_hands_kitchen
```

### Use the Master Viewer

Load any task using the master viewer with command-line arguments:

```bash
python -m vuer_examples.viewer \
    --factory_fn "vuer_mjcf.tasks.mug_tree:make_schema" \
    --name "mug_tree" \
    --actuators "mono"
```

Or run directly (uses default scene from bottom of viewer.py):

```bash
python -m vuer_examples.viewer
```

### Programmatic Usage

```python
from vuer_examples.viewer import load_scene

# Load any scene
load_scene(
    factory_fn="vuer_mjcf.tasks.mug_tree:make_schema",
    name="mug_tree",
    actuators="mono",  # "mono", "duo", or "none"
    show_lights=False,
    verbose=True,
)
```

## Available Scenes

### Simple (Beginner)

**simple_pick_place** - Basic object manipulation
Move a cube from red start area to blue goal area. Single Robotiq gripper.

**push_t** - Non-prehensile manipulation
Push and rotate T-shape to target position using a moving cylinder (no grasping).

### Intermediate

**mug_tree** - Precise placement
Hang a mug on a mug tree hook. Requires careful positioning. Single Robotiq gripper.

**microwave_muffin** - Articulated objects
Open microwave door and place muffin inside. Kitchen scene. Single Robotiq gripper.

**mug_tree_ur** - Full robot arm
Same mug tree task but with UR5e arm instead of floating gripper. Better for sim-to-real.

### Advanced

**tie_knot** - Deformable objects
Tie a knot in a rope using dual Robotiq grippers (bimanual). Soft-body physics.

**pour_liquid** - Fluid dynamics
Pour liquid between containers using dual Shadow Hands (bimanual). Particle simulation.

**kitchen_room** - Complex environment
Full kitchen with appliances, furniture, and multiple objects. Single Robotiq gripper.

**shadow_hands_kitchen** - Dexterous manipulation
Full kitchen with dual Shadow Hands (24+ DOF per hand). Expert-level bimanual coordination.

## Viewer Features

### Command-Line Options

```bash
--factory_fn    Factory function (required): "module.path:function_name"
--name          Scene name (optional, auto-derived from factory_fn)
--vuer_port     Server port (default: 8012)
--actuators     Hand mode: "mono" | "duo" | "none" (default: "mono")
--show_lights   Enable lighting (default: False for better performance)
--verbose       Print detailed output (default: False)
```

### Actuator Modes

- **mono**: Single hand/gripper (right hand only)
- **duo**: Dual hands/grippers (bimanual tasks)
- **none**: No actuators (for full robot arms with joint control)

### Interactive Buttons

Two buttons appear in VR for scene control:

**Reset Button** (blue box at `[-0.4, 1.4, -1]`):
- Click to reload the scene
- Resets physics simulation
- Does NOT save trajectories (visualization only)

**Record Initial Pose** (red sphere at `[0.4, 1.4, -1]`):
- Click to save the current scene state
- Saves to `{scene_name}.frame.yaml`
- Useful for recording starting configurations

## Accessing the Viewer

After starting, you'll see:

```
Vuer Viewer Running!
Visit: https://vuer.ai?ws=ws://localhost:8012
Scene: mug_tree
Press Ctrl+C to exit
```

Open the URL in:
- VR headset browser (Quest, Vision Pro, etc.)
- Desktop browser for non-VR viewing
- Any WebXR-compatible device

## VR Controls

- **Controllers**: Control gripper/hand position
- **Trigger Squeeze**: Close gripper or pinch fingers
- **Reset Button** (blue box): Click to reset scene
- **Record Button** (red sphere): Click to save initial pose to YAML
- **Pause**: Space bar or VR menu

## Creating Custom Scenes

Add a new scene file in `vuer_examples/scenes/`:

```python
"""My Custom Scene"""

from vuer_examples.viewer import load_scene

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

**Port already in use**:
```bash
python -m vuer_examples.viewer --vuer_port 8013
```

**Assets not loading**: Ensure you're running from the project root directory

**Slow rendering**: Lights are disabled by default. If you enabled them, try `--show_lights=False`

**VR not connecting**: Check that your browser supports WebXR

**Import errors**: Make sure vuer-mjcf is installed: `pip install -e .`

## Directory Structure

```
vuer_examples/
├── __init__.py           # Module initialization
├── README.md             # This file
├── viewer.py             # Master viewer with reset button
└── scenes/               # Pre-configured scenes
    ├── __init__.py
    ├── simple_pick_place.py
    ├── push_t.py
    ├── mug_tree.py
    ├── microwave_muffin.py
    ├── tie_knot.py
    ├── pour_liquid.py
    ├── kitchen_room.py
    ├── mug_tree_ur.py
    └── shadow_hands_kitchen.py
```

## Related

- **Main Project**: See `vuer-mjcf` README for full package documentation
- **Task Definitions**: See `vuer_mjcf/tasks/` for all available tasks
- **Stage Sets**: See `vuer_mjcf/stage_sets/` for scene configurations
- **Data Collection**: For trajectory recording and demos, use `collect_demo.py` from the main repository
