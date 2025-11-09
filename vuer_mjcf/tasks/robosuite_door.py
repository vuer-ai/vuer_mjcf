import random
from pathlib import Path

from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.third_party.robosuite.robosuite_door import RobosuiteDoor
from vuer_mjcf.third_party.robosuite.robosuite_tablearena import RobosuiteTableArena

r, g, b = random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)
x, y = random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify

    arena = RobosuiteTableArena(table_pos=f"{x} {y} 0.775")
    door = RobosuiteDoor(
        _attributes=dict(name="door-1", pos=f"{x + 0.1} {y - 0.002} 1.1", quat="0.618783 0 0 -0.785562"),
    )
    scene = FloatingRobotiq2f85(
        arena,
        door,
        pos=[0, 0, 0.8],
        **options,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Robosuite Door task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Robosuite Door task loaded successfully!")
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
