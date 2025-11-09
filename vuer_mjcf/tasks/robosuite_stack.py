import random
from pathlib import Path
import numpy as np

from vuer_mjcf.utils.file import Prettify
from vuer_mjcf.schema import Body
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.third_party.robosuite.robosuite_tablearena import RobosuiteTableArena
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig

r, g, b = random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)
x1, y1 = random.uniform(-0.4, 0.4), random.uniform(-0.2, 0.2)
x2, y2 = random.uniform(-0.4, 0.4), random.uniform(-0.2, 0.2)
x3, y3 = random.uniform(-0.4, 0.4), random.uniform(-0.2, 0.2)

def random_quat():
    theta = np.random.uniform(0, 2 * np.pi)
    w = np.cos(theta / 2)
    z = np.sin(theta / 2)
    return w, 0, 0, z

quat1= random_quat()
q1 = f"{quat1[0]} 0 0 {quat1[3]}"

quat2 = random_quat()
q2 = f"{quat2[0]} 0 0 {quat2[3]}"

quat3 = random_quat()
q3 = f"{quat3[0]} 0 0 {quat3[3]}"

def make_schema(**options):
    arena = RobosuiteTableArena()

    camera_rig = make_camera_rig(arena.table_pos_list)

    box_1 = Body(
        attributes=dict(name="box-1", pos=f"{x1} {y1} 0.8", quat=q1),
        rgba="0 1 0 1.0",
        _preamble="""
          <asset>
            <material name="bright_green_mat" rgba="0 1 0 1" specular="1.0" shininess="1.0" reflectance="0.3"/>
            <material name="top_blue_mat" rgba="0 0 1 1" specular="1.0" shininess="1.0"/>
          </asset>
        """,
        _children_raw="""
          <joint name="{name}-cube_joint0" type="free" limited="false" actuatorfrclimited="false"/>
          <geom name="{name}-cube_g0" size="0.021556 0.0218909 0.02073" type="box" rgba="{rgba}"/>
          <geom name="{name}-cube_g0_vis" size="0.021556 0.0218909 0.02073" type="box" contype="0" conaffinity="0" group="1" mass="0" material="bright_green_mat"/>
          <geom name="{name}-top_face" type="box" size="0.021556 0.0218909 0.001" pos="0 0 0.021" contype="0" conaffinity="0" group="1" mass="0" material="top_blue_mat"/>
          <site name="{name}-cube_default_site" pos="0 0 0" size="0.002" rgba="1 0 0 -1"/>
        """,
    )

    box_2 = Body(
        attributes=dict(name="box-2", pos=f"{x2} {y2} 0.8"),
        rgba="1 0 0 1.0",
        _preamble="""
          <asset>
            <material name="bright_red_mat" rgba="1 0 0 1" specular="1.0" shininess="1.0" reflectance="0.3"/>
            <material name="top_cyan_mat" rgba="0 1 1 1" specular="1.0" shininess="1.0"/>
          </asset>
        """,
        _children_raw="""
          <joint name="{name}-cube_joint0" type="free" limited="false" actuatorfrclimited="false"/>
          <geom name="{name}-cube_g0" size="0.021556 0.0218909 0.02073" type="box" rgba="{rgba}"/>
          <geom name="{name}-cube_g0_vis" size="0.021556 0.0218909 0.02073" type="box" contype="0" conaffinity="0" group="1" mass="0" material="bright_red_mat"/>
          <geom name="{name}-top_face" type="box" size="0.021556 0.0218909 0.001" pos="0 0 0.021" contype="0" conaffinity="0" group="1" mass="0" material="top_cyan_mat"/>
          <site name="{name}-cube_default_site" pos="0 0 0" size="0.002" rgba="1 0 0 -1"/>
        """,
    )

    box_3 = Body(
        attributes=dict(name="box-3", pos=f"{x3} {y3} 0.8"),
        rgba="1 1 0 1.0",
        _preamble="""
          <asset>
            <material name="bright_yellow_mat" rgba="1 1 0 1" specular="1.0" shininess="1.0" reflectance="0.3"/>
            <material name="top_magenta_mat" rgba="1 0 1 1" specular="1.0" shininess="1.0"/>
          </asset>
        """,
        _children_raw="""
          <joint name="{name}-cube_joint0" type="free" limited="false" actuatorfrclimited="false"/>
          <geom name="{name}-cube_g0" size="0.021556 0.0218909 0.02073" type="box" rgba="{rgba}"/>
          <geom name="{name}-cube_g0_vis" size="0.021556 0.0218909 0.02073" type="box" contype="0" conaffinity="0" group="1" mass="0" material="bright_yellow_mat"/>
          <geom name="{name}-top_face" type="box" size="0.021556 0.0218909 0.001" pos="0 0 0.021" contype="0" conaffinity="0" group="1" mass="0" material="top_magenta_mat"/>
          <site name="{name}-cube_default_site" pos="0 0 0" size="0.002" rgba="1 0 0 -1"/>
        """,
    )

    scene = FloatingRobotiq2f85(
        *camera_rig.get_cameras(),
        arena,
        box_1,
        box_2,
        box_3,
        pos=[0, 0, 0.8],
        **options,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Robosuite Stack task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Robosuite Stack task loaded successfully!")
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
