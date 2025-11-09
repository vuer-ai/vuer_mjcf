import random
from pathlib import Path

from vuer_mjcf.schema import Body
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.third_party.robohive.robohive_object import RobohiveObj

# Generate random values for r, g, and b
r, g, b = random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)
x, y = random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)

def make_schema():
    from vuer_mjcf.utils.file import Prettify

    table = RobohiveObj(
        otype="furniture",
        asset_key="simpleTable/simpleWoodTable",
        pos=[0, 0, 0],
    )
    box = Body(
        attributes=dict(name="box-1", pos=f"{x} {y} 0.8"),
        rgba=f"{r} {g} {b} 1.0",
        _children_raw="""
        <joint type="free" name="{name}"/>
        <geom name="box-1" type="box" size="0.015 0.015 0.015" mass="0.1" rgba="{rgba}" density="1000"/>
        """,
    )
    scene = FloatingRobotiq2f85(
        table,
        box,
        pos=[0, 0, 0.8],
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Pick Block task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Pick Block task loaded successfully!")
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
