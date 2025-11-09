"""
Mug Tree Scene

A placement task where a mug needs to be hung on a mug tree. This task requires
precise positioning and orientation to successfully place the mug on the tree hooks.

Features:
- Single floating Robotiq 2F-85 gripper
- Coffee mug with handle
- Mug tree with multiple hooks
- Goal sites marked on tree hooks for visualization
- Realistic table and camera setup

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.mug_tree
"""

from vuer_examples.viewer import load_scene


def main():
    """Load the mug tree scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.mug_tree:make_schema",
        name="mug_tree",
        actuators="mono",  # Single gripper
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Mug Tree Scene")
    print("=" * 60)
    print("\nTask: Pick up the mug and hang it on the mug tree")
    print("\nFeatures:")
    print("  - Precise placement required")
    print("  - Multiple possible hook positions")
    print("  - Goal sites show target locations")
    print("\nControls:")
    print("  - Use VR controller to position gripper")
    print("  - Squeeze trigger to grasp mug handle")
    print("  - Align mug handle with tree hook")
    print("  - Release to hang mug")
    print("\n" + "=" * 60 + "\n")

    main()
