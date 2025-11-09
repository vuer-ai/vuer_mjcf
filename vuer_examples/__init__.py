"""
Vuer Examples - Simplified VR Scene Viewer

A streamlined viewer for visualizing MuJoCo scenes in VR using Vuer.
This is a visualization-only module - it does NOT include data collection,
trajectory recording, or other advanced features.

Quick Start:
    # Run an example scene
    python -m vuer_examples.scenes.simple_pick_place
    python -m vuer_examples.scenes.mug_tree
    python -m vuer_examples.scenes.kitchen_room

    # Or use the master viewer
    python -m vuer_examples.viewer --factory_fn "vuer_mjcf.tasks.mug_tree:make_schema"

Programmatic Usage:
    from vuer_examples.viewer import load_scene

    load_scene(
        factory_fn="vuer_mjcf.tasks.mug_tree:make_schema",
        name="mug_tree",
        actuators="mono",
        verbose=True,
    )

Available Scenes:
    - simple_pick_place: Basic object manipulation
    - push_t: Non-prehensile manipulation
    - mug_tree: Precise placement task
    - microwave_muffin: Articulated objects
    - tie_knot: Deformable objects (bimanual)
    - pour_liquid: Fluid dynamics (bimanual)
    - kitchen_room: Complex environment
    - mug_tree_ur: Full robot arm variant
    - shadow_hands_kitchen: Dexterous manipulation

Features:
    ✅ VR visualization with WebXR
    ✅ VR controller-based gripper/hand control
    ✅ Reset button to reload scenes
    ✅ Record initial pose button
    ❌ No data collection or trajectory recording
    ❌ No reward computation or task success detection

For more details, see vuer_examples/README.md
"""

from vuer_examples.viewer import load_scene

__all__ = ['load_scene']
