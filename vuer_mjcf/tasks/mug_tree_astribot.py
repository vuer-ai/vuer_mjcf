import os
from pathlib import Path

import numpy as np

from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.objects.vuer_mug import VuerMug
from vuer_mjcf.basic_components.rigs.camera_rig_calibrated import make_camera_rig
# from vuer_mjcf.basic_components.rigs.camera_rig_stereo import make_origin_stereo_rig

from vuer_mjcf.stage_sets.astribot_robotiq import AstribotRobotiq2f85
import mink
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.objects.orbit_table import OpticalTable

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

center = 0
x1, y1 = center - 0.05, -0.025
x2, y2 = center + 0.17, 0.0

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify

    # table = RobohiveObj(
    #     otype="furniture",
    #     asset_key="simpleTable/simpleWoodTable",
    #     pos=[0, 0, 0],
    #     quat=[0.7071, 0, 0, 0.7071],
    #     local_robohive_root="robohive",
    # )
    optical_table = OpticalTable(
        pos=[0.05, -0.5, 0.77],
        quat=[0.7071, 0, 0, 0.7071],
        # pos=[-0.4, 0, 0.77],
        # quat=[0, 0, 0, 1],
        assets="model",
        _attributes={"name": "table_optical"},
    )
    table = ConcreteSlab(
        assets="model",
        pos=[0, 0, 0.76],
        group=4,
        rgba="0.8 0 0 0.0",
        _attributes={
            "name": "table",
        },
    )

    camera_rig = make_camera_rig(pos=[-0.4, 0, 0.77])

    mug = VuerMug(
        pos=[x1, y1, 0.85],
        quat=[0, 0, 0, 1],
        _attributes={
            "name": "mug",
        },
    )

    mug_tree = MjSDF(
        pos=[x2, y2, 1],
        assets="mug_tree",
        mass=0.3,
        _attributes={
            "name": "tree",
            "quat": "0.5 0.5 0.5 0.5",
        },
        additional_children_raw="""\t<site name="tree_goal_1" pos="-.03 0.076 -0.036" size="0.007" rgba="1 1 1 1"/>
                                              \t<site name="tree_goal_2" pos="-.03 0.066 -0.016" size="0.007" rgba="1 1 1 1"/>
                                              \t<site name="tree_goal_3" pos="-.03 0.071 -0.026" size="0.007" rgba="1 1 1 1"/>
                                              \t<site name="tree_goal_4" pos="-.03 0.081 -0.046" size="0.007" rgba="1 1 1 1"/>
                                              \t<site name="tree_goal_5" pos="-.03 0.086 -0.056" size="0.007" rgba="1 1 1 1"/>
                                              \t<site name="tree_goal_6" pos="-.03 0.091 -0.066" size="0.007" rgba="1 1 1 1"/>
                                                """,
        scale="0.17 0.2118 0.17",
    )

    children = [
        *camera_rig.get_cameras(),
        optical_table,
        mug_tree,
        mug,
        table,
    ]

    # base_pos = np.array([-0.55, -0.2, -0.2])  # base position of the robot
    base_pos = np.array([-0.4, -0.2, 0.1])  # base position of the robot
    head_target = _compute_look_at_rotation(
        head_pos=base_pos + np.array([0.0, 0.0, 1.5]),
        target_pos=np.array([x2, y2, 1.0])
    )

    head_quat_wxyz = head_target.wxyz_xyz[:4]  # wxyz
    
    scene = AstribotRobotiq2f85(
        *children,
        pos=base_pos.tolist(),
        head_quat=head_quat_wxyz.tolist(),
        mocap_pos="-0.096904911 0.00114953518 1.14964232",
        mocap_quat=[0, 0, -1, 0],
        # pos=[0, 0, 0],
        **options,
        camera_rig=camera_rig,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Mug Tree Astribot task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Mug Tree Astribot task loaded successfully!")
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
