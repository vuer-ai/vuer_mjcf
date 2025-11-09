"""
Push T Scene

A pushing task where a T-shaped object needs to be moved and rotated to align with
a target position. This task requires precise control and is commonly used in
imitation learning research.

Features:
- Moving cylinder actuator (touch-based manipulation)
- T-shaped object with sliding physics
- Table with reduced friction
- Orthographic camera view for top-down perspective

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.push_t
"""

from vuer_examples.viewer import load_scene


def main():
    """Load the push T scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.push_t:make_schema",
        name="push_t",
        actuators="none",  # No gripper, uses moving cylinder
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Push T Scene")
    print("=" * 60)
    print("\nTask: Push and rotate the T-shape to align with target")
    print("\nFeatures:")
    print("  - Touch-based manipulation (no grasping)")
    print("  - Requires precise positioning and orientation")
    print("  - Top-down orthographic view")
    print("\nControls:")
    print("  - Use VR controller position to control the pusher")
    print("  - Contact the T-shape to move it")
    print("\n" + "=" * 60 + "\n")

    main()
