"""
Kitchen Room with Shadow Hands

A comprehensive kitchen scene using dual Shadow Hands for bimanual dexterous
manipulation. This is the most complex example, combining a large environment
with highly articulated hands for fine manipulation of kitchen objects.

Features:
- Dual floating Shadow Hands (bimanual dexterous manipulation)
- Full kitchen environment with all appliances
- Multiple kitchen objects (mugs, bowls, plates, cups, spoons)
- Articulated cabinets and drawer stacks
- Large workspace for complex multi-step tasks

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.shadow_hands_kitchen
"""

from vuer_mjcf.vuer_examples.viewer import load_scene


def main():
    """Load the kitchen room with Shadow Hands scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.tri_demo_kitchen_room_shadow_hands:make_schema",
        name="shadow_hands_kitchen",
        actuators="none",  # Shadow Hands use their own actuators
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Kitchen Room with Shadow Hands")
    print("=" * 60)
    print("\nScene: Full kitchen with bimanual dexterous hands")
    print("\nFeatures:")
    print("  - Dual Shadow Hands (24+ DOF each)")
    print("  - Complete kitchen environment")
    print("  - Fine manipulation capabilities")
    print("  - Bimanual coordination")
    print("  - Articulated furniture and appliances")
    print("\nAvailable Tasks:")
    print("  - Dexterous object manipulation")
    print("  - Opening containers and drawers")
    print("  - Bimanual coordination tasks")
    print("  - Complex in-hand manipulation")
    print("\nControls:")
    print("  - Use BOTH VR controllers (left and right)")
    print("  - Each controller maps to one Shadow Hand")
    print("  - Squeeze triggers for finger control")
    print("  - Fine motor control available")
    print("\nKey Differences from Grippers:")
    print("  - Much higher DOF (24+ per hand vs 1 for gripper)")
    print("  - Capable of in-hand manipulation")
    print("  - Can grasp objects in multiple ways")
    print("  - More human-like manipulation capabilities")
    print("\nTips:")
    print("  - Shadow Hands allow complex grasps")
    print("  - Try different grasp types (pinch, power, etc.)")
    print("  - Practice bimanual coordination")
    print("  - Great for learning dexterous manipulation")
    print("\n" + "=" * 60 + "\n")

    main()
