"""
Mug Tree with UR5 Robot Arm

The same mug tree task but using a full UR5 robot arm with Robotiq gripper instead
of a floating gripper. This demonstrates how the same task can be performed with
different robot configurations - useful for sim-to-real transfer research.

Features:
- Full UR5e robot arm with joint control
- Robotiq 2F-85 gripper mounted on end-effector
- Same mug and mug tree objects as simple version
- Wooden table for realistic workspace
- Demonstrates full arm kinematics vs floating gripper

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.mug_tree_ur
"""

from vuer_examples.viewer import load_scene


def main():
    """Load the mug tree UR5 scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.mug_tree_ur:make_schema",
        name="mug_tree_ur",
        actuators="none",  # Robot arm uses joint control, not hand actuators
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Mug Tree with UR5 Robot Arm")
    print("=" * 60)
    print("\nTask: Pick up mug and hang it on tree using UR5 robot")
    print("\nFeatures:")
    print("  - Full UR5e robot arm with 6 DOF")
    print("  - Robotiq 2F-85 gripper")
    print("  - Joint-level control")
    print("  - Same task as floating gripper version")
    print("\nControls:")
    print("  - VR controller maps to end-effector position")
    print("  - Robot arm follows using inverse kinematics")
    print("  - Squeeze trigger to close gripper")
    print("  - More realistic workspace constraints")
    print("\nKey Differences from Floating Gripper:")
    print("  - Joint limits and collision constraints")
    print("  - Realistic arm reach limitations")
    print("  - Better for sim-to-real transfer")
    print("  - More representative of real robot behavior")
    print("\n" + "=" * 60 + "\n")

    main()
