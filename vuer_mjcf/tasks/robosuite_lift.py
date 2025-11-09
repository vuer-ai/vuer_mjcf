import random
from pathlib import Path

from vuer_mjcf.schema import Body
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.third_party.robosuite.robosuite_tablearena import RobosuiteTableArena

r, g, b = random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)
x, y = random.uniform(-0.5, 0.5), random.uniform(-0.2, 0.2)

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify

    arena = RobosuiteTableArena()
    box = Body(
        rgba=f"{r} {g} {b} 1.0",
        _attributes={
            "name": "box-1",
            "pos": f"{x} {y} 0.8",
        },
        _preamble="""
          <asset>
            <texture type="cube" name="cube_redwood" file="textures/red-wood.png"/>
            <material name="cube_redwood_mat" texture="cube_redwood" specular="0.4" shininess="0.1"/>
          </asset>
        """,
        _children_raw="""
          <joint name="{name}-cube_joint0" type="free" limited="false" actuatorfrclimited="false"/>
          <geom name="{name}-cube_g0" size="0.021556 0.0218909 0.02073" type="box" rgba="0.5 0 0 1"/>
          <geom name="{name}-cube_g0_vis" size="0.021556 0.0218909 0.02073" type="box" contype="0" conaffinity="0" group="1" mass="0" material="cube_redwood_mat"/>
          <site name="{name}-cube_default_site" pos="0 0 0" size="0.002" rgba="1 0 0 -1"/>
        """,
    )
    scene = FloatingRobotiq2f85(
        arena,
        box,
        pos=[0, 0, 0.8],
        **options,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Robosuite Lift task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Robosuite Lift task loaded successfully!")
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
