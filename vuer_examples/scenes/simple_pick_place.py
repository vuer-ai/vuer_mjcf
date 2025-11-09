"""
Simple Pick and Place Scene

A basic manipulation task where a green box needs to be moved from a red start area
to a blue goal area. This is a fundamental task for learning object manipulation.

Features:
- Single floating Robotiq 2F-85 gripper
- One green cube object
- Start and goal areas marked with colored plates
- Simple table surface

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.simple_pick_place
"""

from vuer_mjcf.vuer_examples.viewer import load_scene


def main():
    """Load the pick and place scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.pick_place:make_schema",
        name="pick_place",
        actuators="mono",  # Single gripper
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Simple Pick and Place Scene")
    print("=" * 60)
    print("\nTask: Move the green cube from the red area to the blue area")
    print("\nControls:")
    print("  - Use VR controllers to manipulate the gripper")
    print("  - Squeeze trigger to close gripper")
    print("  - Move controller to position gripper")
    print("\n" + "=" * 60 + "\n")

    main()
