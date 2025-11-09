from pathlib import Path

from vuer_mjcf.objects.bigym_dishdrainer import BigymDishdrainer
from vuer_mjcf.objects.bigym_plate import BigymPlate
from vuer_mjcf.objects.bigym_table import BigymTable
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane

# Generate random values for r, g, and b
# r, g, b = random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)

def make_schema():
    from vuer_mjcf.utils.file import Prettify

    ground = GroundPlane()
    table = BigymTable(pos=[0.7, 0, 0], quat=[0.7071, 0, 0, -0.7071])

    drainer1 = BigymDishdrainer(
        pos=[0.7, 0.3, 0.95],
        quat=[1, 0, 0, 0],
        prefix="drainer1",
        _attributes={
            "name": "drainer1",
        },
    )
    drainer2 = BigymDishdrainer(
        pos=[0.7, -0.3, 0.95],
        quat=[1, 0, 0, 0],
        prefix="drainer2",
        _attributes={
            "name": "drainer2",
        },
    )

    plate = BigymPlate(
        pos=[0.7, -0.18, 1.1],
        quat=[0.7071, 0.7071, 0, 0],
        prefix="plate1",
        _attributes={
            "name": "plate1",
        },
    )

    scene = FloatingRobotiq2f85(
        ground,
        table,
        drainer1,
        drainer2,
        plate,
        pos=[0, 0, 0.8],
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Move Plate task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Move Plate task loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")

            data = mujoco.MjData(model)
            print("Launching interactive viewer...")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)
    except ImportError:
        print("MuJoCo not available")
    except Exception as e:
        print(f"✗ Error: {e}")
        raise
