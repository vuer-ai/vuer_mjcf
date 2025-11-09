"""
Vuer Examples - Simple scene loading and visualization examples.

This module provides easy-to-use examples for loading and visualizing
MuJoCo scenes in Vuer. See the `scenes/` directory for example scenes.

Quick Start:
    # List all available examples
    python -m vuer_mjcf.vuer_examples.list_examples

    # Run an example
    python -m vuer_mjcf.vuer_examples.scenes.simple_pick_place

    # Interactive menu
    python -m vuer_mjcf.vuer_examples.run_example

Programmatic Usage:
    from vuer_mjcf.vuer_examples.viewer import load_scene

    load_scene(
        factory_fn="vuer_mjcf.tasks.mug_tree:make_schema",
        name="mug_tree",
        actuators="mono",
    )

Available Examples:
    - simple_pick_place: Basic pick and place task
    - push_t: Push T-shape to target
    - mug_tree: Hang mug on tree
    - microwave_muffin: Open microwave and place muffin
    - tie_knot: Tie rope knot (bimanual)
    - pour_liquid: Pour liquid between containers
    - kitchen_room: Full kitchen environment
    - mug_tree_ur: Mug tree with UR5 robot arm
    - shadow_hands_kitchen: Kitchen with Shadow Hands

For more details, see:
    - vuer_mjcf/vuer_examples/README.md
    - vuer_mjcf/vuer_examples/EXAMPLES_GUIDE.md
"""

from vuer_mjcf.vuer_examples.viewer import load_scene
from vuer_mjcf.vuer_examples.list_examples import EXAMPLES, get_example_list, get_example_by_name

__all__ = ['load_scene', 'EXAMPLES', 'get_example_list', 'get_example_by_name']
