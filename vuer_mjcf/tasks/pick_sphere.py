import random
from pathlib import Path

from vuer_mjcf.schema import Body
from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.basic_components.table_slab import Table
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85

x1, y1 = random.uniform(-0.2, 0.2), random.uniform(-0.1, -0.5)
x2, y2 = random.uniform(-0.2, 0.2), random.uniform(-0.1, -0.5)

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify

    # table = BrownTable(pos=[0, 0, -0.1], assets="brown_table")

    basket = MjSDF(pos=[0, 0.2, 0.67], assets="black_basket", _attributes={"name": "basket"})

    table = Table(pos=[0, 0, 0.6], rgba="0.7 0.65 0.57 1")
    ball = Body(
        attributes=dict(name="ball-1", pos=f"{x1} {y1} 0.7"),
        _children_raw="""
        <joint type="free" name="{name}"/>
        <geom name="sphere-1" type="sphere" size="0.03" rgba="0.5 0.5 0.5 1" mass="0.1" solref="0.003 1" solimp="0.95 0.99 0.001" friction="2 0.01 0.002"/>
        """,
    )

    can = Body(
        attributes=dict(name="can-1", pos=f"{x2} {y2} 0.7"),
        _children_raw="""
        <joint type="free" name="{name}"/>
        <geom name="can-1" type="cylinder" size="0.03 0.05" rgba="0.5 0.5 0.5 1" mass="0.1" solref="0.003 1" solimp="0.95 0.99 0.001" friction="2 0.01 0.002"/>
        """,
    )

    ground = Body(
        attributes=dict(name="ground"),
        _children_raw="""
        <geom name="ground" type="plane" pos="0 0 0" size="10 10 0.1" rgba="0.2 0.3 0.4 1"/>
        """,
    )

    scene = FloatingRobotiq2f85(
        table,
        ball,
        can,
        basket,
        ground,
        pos=[0, 0, 0.8],
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Pick Sphere task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Pick Sphere task loaded successfully!")
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
