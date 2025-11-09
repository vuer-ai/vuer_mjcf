"""
Pour Liquid Scene

An advanced particle simulation task demonstrating fluid dynamics. This scene uses
particle-based liquid simulation where liquid must be poured from one container to
another, requiring careful manipulation of containers with dynamic contents.

Features:
- Dual floating Shadow Hand grippers (bimanual dexterous manipulation)
- Particle-based liquid simulation
- Kitchen environment with walls and countertops
- Multiple containers (bowls, mugs, cups)
- Complex scene with drawer stacks and appliances

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.pour_liquid
"""

from vuer_examples.viewer import load_scene


def main():
    """Load the pour liquid scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.pour_liquid:make_schema",
        name="pour_liquid",
        actuators="duo",  # Dual Shadow Hands for bimanual manipulation
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Pour Liquid Scene")
    print("=" * 60)
    print("\nTask: Pour liquid from one container to another")
    print("\nFeatures:")
    print("  - Particle-based fluid simulation")
    print("  - Bimanual Shadow Hand manipulation")
    print("  - Realistic kitchen environment")
    print("  - Dynamic liquid physics")
    print("\nControls:")
    print("  - Use BOTH VR controllers (dual hands)")
    print("  - Squeeze triggers to grasp containers")
    print("  - Carefully tilt container to pour liquid")
    print("  - Position receiving container underneath")
    print("\nTips:")
    print("  - Grasp container firmly before tilting")
    print("  - Pour slowly to avoid spilling")
    print("  - Watch liquid particle flow")
    print("  - May require coordination between both hands")
    print("\n" + "=" * 60 + "\n")

    main()
