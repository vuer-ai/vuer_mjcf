"""
Kitchen Room Scene

A comprehensive kitchen environment with multiple objects and appliances. This is
one of the most complex scenes, featuring a full-scale kitchen with walls, cabinets,
drawers, appliances, and numerous manipulatable objects for diverse task training.

Features:
- Single floating Robotiq 2F-85 gripper
- Full kitchen environment with walls and furniture
- Multiple appliances: microwave, oven, dishwasher, refrigerator, sink
- Kitchen cabinets and drawer stacks (some articulated)
- Various kitchen objects: mugs, bowls, cups, plates, spoons
- Granite countertops with realistic materials

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.kitchen_room
"""

from vuer_examples.viewer import load_scene


def main():
    """Load the kitchen room scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.kitchen_room:make_schema",
        name="kitchen_room",
        actuators="mono",  # Single gripper
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Kitchen Room Scene")
    print("=" * 60)
    print("\nScene: Full-scale realistic kitchen environment")
    print("\nFeatures:")
    print("  - Complete kitchen with appliances")
    print("  - Articulated cabinets and drawers")
    print("  - Multiple kitchen objects to manipulate")
    print("  - Realistic materials and textures")
    print("  - Large interaction space")
    print("\nAvailable Tasks:")
    print("  - Open/close drawers and cabinets")
    print("  - Manipulate kitchen utensils")
    print("  - Place objects on countertops")
    print("  - Interact with appliances")
    print("\nControls:")
    print("  - Use VR controller to position gripper")
    print("  - Squeeze trigger to grasp objects")
    print("  - Navigate the kitchen space")
    print("  - Interact with articulated furniture")
    print("\nTips:")
    print("  - Explore different areas of the kitchen")
    print("  - Try opening drawers and cabinets")
    print("  - This scene is ideal for multi-task learning")
    print("\n" + "=" * 60 + "\n")

    main()
