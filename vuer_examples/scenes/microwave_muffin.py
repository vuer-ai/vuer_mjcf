"""
Microwave Muffin Scene

A kitchen manipulation task requiring interaction with articulated objects. The goal
is to open the microwave door and place a muffin inside. This demonstrates handling
of articulated mechanisms and sequential manipulation.

Features:
- Single floating Robotiq 2F-85 gripper
- Kitchen scene with microwave and countertops
- Articulated microwave door with hinge joint
- Cupcake/muffin object to manipulate
- Realistic kitchen walls and furniture

Run this example:
    python -m vuer_mjcf.vuer_examples.scenes.microwave_muffin
"""

from vuer_mjcf.vuer_examples.viewer import load_scene


def main():
    """Load the microwave muffin scene in Vuer."""
    load_scene(
        factory_fn="vuer_mjcf.tasks.microwave_muffin:make_schema",
        name="microwave_muffin",
        actuators="mono",  # Single gripper
        show_lights=False,
        verbose=True,
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Microwave Muffin Scene")
    print("=" * 60)
    print("\nTask: Open the microwave and place the muffin inside")
    print("\nFeatures:")
    print("  - Articulated microwave door")
    print("  - Realistic kitchen environment")
    print("  - Sequential manipulation required")
    print("  - Multiple object interactions")
    print("\nControls:")
    print("  - Use VR controller to position gripper")
    print("  - Squeeze trigger to grasp objects")
    print("  - First: grasp and pull microwave handle to open door")
    print("  - Second: pick up muffin and place inside microwave")
    print("\n" + "=" * 60 + "\n")

    main()
