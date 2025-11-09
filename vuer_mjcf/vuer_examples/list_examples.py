"""
List all available Vuer examples with descriptions.

Run this to see all available scene examples:
    python -m vuer_mjcf.vuer_examples.list_examples
"""

EXAMPLES = {
    "Simple Examples (Beginner)": [
        {
            "name": "simple_pick_place",
            "module": "vuer_mjcf.vuer_examples.scenes.simple_pick_place",
            "description": "Move a green cube from red area to blue goal area",
            "actuator": "mono",
            "complexity": "⭐",
        },
        {
            "name": "push_t",
            "module": "vuer_mjcf.vuer_examples.scenes.push_t",
            "description": "Push and rotate T-shape to align with target",
            "actuator": "none",
            "complexity": "⭐",
        },
    ],
    "Intermediate Examples": [
        {
            "name": "mug_tree",
            "module": "vuer_mjcf.vuer_examples.scenes.mug_tree",
            "description": "Hang a mug on a mug tree hook",
            "actuator": "mono",
            "complexity": "⭐⭐",
        },
        {
            "name": "microwave_muffin",
            "module": "vuer_mjcf.vuer_examples.scenes.microwave_muffin",
            "description": "Open microwave and place muffin inside",
            "actuator": "mono",
            "complexity": "⭐⭐",
        },
    ],
    "Advanced Examples": [
        {
            "name": "tie_knot",
            "module": "vuer_mjcf.vuer_examples.scenes.tie_knot",
            "description": "Tie a knot in a deformable rope (bimanual)",
            "actuator": "duo",
            "complexity": "⭐⭐⭐",
        },
        {
            "name": "pour_liquid",
            "module": "vuer_mjcf.vuer_examples.scenes.pour_liquid",
            "description": "Pour liquid between containers (particle sim)",
            "actuator": "duo",
            "complexity": "⭐⭐⭐",
        },
        {
            "name": "kitchen_room",
            "module": "vuer_mjcf.vuer_examples.scenes.kitchen_room",
            "description": "Full kitchen environment with appliances",
            "actuator": "mono",
            "complexity": "⭐⭐⭐",
        },
    ],
    "Robot Variants": [
        {
            "name": "mug_tree_ur",
            "module": "vuer_mjcf.vuer_examples.scenes.mug_tree_ur",
            "description": "Mug tree task with UR5e robot arm",
            "actuator": "none",
            "complexity": "⭐⭐",
        },
        {
            "name": "shadow_hands_kitchen",
            "module": "vuer_mjcf.vuer_examples.scenes.shadow_hands_kitchen",
            "description": "Kitchen with dual Shadow Hands (dexterous)",
            "actuator": "none",
            "complexity": "⭐⭐⭐⭐",
        },
    ],
}


def print_examples():
    """Print all available examples in a formatted table."""
    print("\n" + "=" * 80)
    print("VUER EXAMPLES - Available Scenes")
    print("=" * 80)

    for category, examples in EXAMPLES.items():
        print(f"\n{category}")
        print("-" * 80)

        for ex in examples:
            print(f"\n  {ex['complexity']} {ex['name']}")
            print(f"     {ex['description']}")
            print(f"     Actuator: {ex['actuator']}")
            print(f"     Run: python -m {ex['module']}")

    print("\n" + "=" * 80)
    print("Usage:")
    print("  python -m vuer_mjcf.vuer_examples.scenes.<scene_name>")
    print("\nFor more details:")
    print("  cat vuer_mjcf/vuer_examples/README.md")
    print("=" * 80 + "\n")


def get_example_list():
    """Get a flat list of all examples."""
    all_examples = []
    for category, examples in EXAMPLES.items():
        all_examples.extend(examples)
    return all_examples


def get_example_by_name(name):
    """Get example info by name."""
    for category, examples in EXAMPLES.items():
        for ex in examples:
            if ex["name"] == name:
                return ex
    return None


if __name__ == "__main__":
    print_examples()
