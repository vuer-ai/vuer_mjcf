import random
from pathlib import Path

import numpy as np

from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.basic_components.rigs.camera_rig_calibrated import make_camera_rig
from vuer_mjcf.stage_sets.astribot_robotiq import AstribotRobotiq2f85
from vuer_mjcf.stage_sets.ur5e_robotiq_scene import UR5Robotiq2f85
from vuer_mjcf.objects.mj_obj import MjObj
from vuer_mjcf.third_party.robohive.robohive_object import RobohiveObj
from vuer_mjcf.schema import Body
from vuer_mjcf.basic_components.force_plate import ForcePlate
import mink

# scene center is 0.41 + 0.2 / 2 = 0.50
x1, y1 = -0.08 + 0.4, 0.08
x2, y2 = 0.28 + 0.4, 0.38
x_bounds = [-0.08 + 0.4, 0.28 + 0.4]
y_bounds = [0.08, 0.38]
x_bounds_smaller = [-0.05 + 0.4, 0.15 + 0.4]
y_bounds_smaller = [0.08, 0.28]

goal_x, goal_y = 0.1 + 0.4, -0.30

def _compute_look_at_rotation(
    head_pos: np.ndarray,
    target_pos: np.ndarray,
    world_up: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> mink.SE3:
    """Returns SE3 whose rotation points +X from head_pos toward target_pos."""
    look_direction = target_pos - head_pos
    x_axis = look_direction / (np.linalg.norm(look_direction) + 1e-12)

    y_axis = np.cross(world_up, x_axis)
    ny = np.linalg.norm(y_axis)
    if ny < 1e-6:
        y_axis = np.cross(x_axis, np.array([1.0, 0.0, 0.0]))
        ny = np.linalg.norm(y_axis)
        if ny < 1e-6:
            y_axis = np.cross(x_axis, np.array([0.0, 1.0, 0.0]))
            ny = np.linalg.norm(y_axis)
    y_axis /= max(ny, 1e-12)

    z_axis = np.cross(x_axis, y_axis)
    R = np.column_stack((x_axis, y_axis, z_axis))
    return mink.SE3.from_rotation(mink.SO3.from_matrix(R))

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify
    from vuer_mjcf.objects.cylinder import Cylinder

    robohive_table = RobohiveObj(
        otype="furniture",
        asset_key="simpleTable/simpleWoodTable",
        pos=[0.4, 0, 0],
        quat=[0.7071, 0, 0, 0.7071],
        local_robohive_root="robohive",
    )
    robot_room = MjObj(
        geom_quat="0 0 0.7071 0.7071",
        geom_pos="-1.2 0.8 1.5",
        assets="45_robot_room",
        free=False,
        collision=False,
    )

    camera_rig = make_camera_rig(pos=[0, 0, 0.77])

    box = Body(
        pos=(x1, y1, 0.8),
        attributes=dict(name="box-1"),
        _children_raw="""
        <joint type="free" name="{name}"/>
        <geom name="box-1" type="box" pos="0 0.0 0.0"  rgba="0 1 0 1" size="0.02 0.02 0.02" density="1000"/>
        <site name="box-1" pos="0 0.0 0.0" size="0.005"/>
        """,
    )

    goal_area = ForcePlate(
        name="goal-area",
        pos=(goal_x, goal_y, 0.77),
        # quat=(0, 1, 0, 0),
        type="box",
        size="0.215 0.14 0.005",
        rgba="1. 1. 1. 1.",
    )

    children = [
        *camera_rig.get_cameras(),
        robohive_table,
        # robot_room,
        box,
        goal_area,
    ]

    scene = UR5Robotiq2f85(
        *children,
        pos=[0, 0, 0.77],
        **options,
        camera_rig=camera_rig,
    )

    base_pos = np.array([-0.2, -0.1, 0])
    head_target = _compute_look_at_rotation(head_pos=base_pos + np.array([0.0, 0.0, 1.5]), target_pos=np.array([x1, y1, 0.8]))

    head_quat_wxyz = head_target.wxyz_xyz[:4]  # wxyz
    
    scene = AstribotRobotiq2f85(
        *children,
        head_quat=head_quat_wxyz.tolist(),
        mocap_pos="0.45211 0.20153 1.29",
        mocap_quat=[0, 0, -1, 0],
        pos=base_pos.tolist(),
        **options,
        camera_rig=camera_rig,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Pick Place Robot Room Astribot task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Pick Place Robot Room Astribot task loaded successfully!")
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
