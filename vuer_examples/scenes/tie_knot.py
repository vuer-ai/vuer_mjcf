"""
Tie Knot Scene

An advanced deformable object manipulation task. This scene features a flexible rope
that must be tied into a knot using bimanual manipulation. Demonstrates soft-body
physics and complex sequential actions.

Features:
- Dual floating Robotiq 2F-85 grippers (bimanual)
- Deformable rope with composite simulation
- Optical table for precise manipulation
- Stereo camera rig for depth perception

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.tie_knot
"""

from vuer_examples.viewer import load_scene


def main():
    """Load the tie knot scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.tie_knot:make_schema",
        name="tie_knot",
        actuators="duo",  # Dual grippers for bimanual manipulation
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Tie Knot Scene")
    print("=" * 60)
    print("\nTask: Tie a knot in the rope using two hands")
    print("\nFeatures:")
    print("  - Deformable rope physics")
    print("  - Bimanual coordination required")
    print("  - Complex sequential manipulation")
    print("  - Soft-body simulation")
    print("\nControls:")
    print("  - Use BOTH VR controllers (left and right hands)")
    print("  - Squeeze triggers to grasp rope")
    print("  - Coordinate both hands to loop and tie rope")
    print("  - Requires precise timing and positioning")
    print("\nTips:")
    print("  - Grasp rope at two points")
    print("  - Create a loop with one hand")
    print("  - Thread rope end through loop with other hand")
    print("\n" + "=" * 60 + "\n")

    main()
