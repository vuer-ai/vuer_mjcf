import os
from pathlib import Path

import numpy as np

from vuer_mjcf.basic_components.rigs.camera_rig_calibrated import make_camera_rig
from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.objects.vuer_mug import VuerMug
# from vuer_mjcf.basic_components.rigs.camera_rig_calibrated import make_camera_rig
from vuer_mjcf.basic_components.rigs.camera_rig_stereo import make_origin_stereo_rig
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.stage_sets.ur5e_robotiq_scene import UR5Robotiq2f85
from vuer_mjcf.objects.orbit_table import OpticalTable
from vuer_mjcf.third_party.robohive.robohive_object import RobohiveObj

center = 0
x1, y1 = center - 0.05, -0.025
x2, y2 = center + 0.17, 0.0

def make_schema(mode="cameraready", robot="panda", show_robot=False, **options):
    from vuer_mjcf.utils.file import Prettify

    optical_table = OpticalTable(
        pos=[-0.4, 0, 0.77],
        assets="optical_table",
        _attributes={"name": "table_optical"},
    )

    table = RobohiveObj(otype="furniture", asset_key="simpleTable/simpleWoodTable", pos=[0, 0, 0], quat=[0.7071, 0, 0, 0.7071])
    table = ConcreteSlab(
        assets="model",
        pos=[0, 0, 0.76],
        group=4,
        rgba="0.8 0 0 0.0",
        _attributes={
            "name": "table",
        },
    )
    stereo_cameras = make_origin_stereo_rig(pos=[-0.4, 0, 0.77])
    camera_rig = make_camera_rig(pos=[-0.4, 0, 0.77])

    if robot == "panda":
        from vuer_mjcf.robots.franka_panda import Panda
        from vuer_mjcf.robots.tomika_gripper import TomikaGripper

        gripper = TomikaGripper(
            name="test_gripper",
            mocap_pos="0.25 0.0 1.10",
            mocap_quat="0 0 1 0",
            wrist_mount=camera_rig.wrist_camera(),
        )
        panda = Panda(end_effector=gripper, name="test_panda", pos=(0, 0, 0.79), quat=(1, 0, 0, 0))
    else:
        raise ValueError(f"Unknown robot: {robot}")

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
        *stereo_cameras.get_cameras(),
        mug_tree,
        mug,
        table,
    ]

    if mode == "demo":
        pass
    elif mode == "cameraready":
        children += [optical_table]

    if show_robot:
        scene = UR5Robotiq2f85(
            *children,
            pos=[-0.4, 0, 0.77],
            **options,
            camera_rig=camera_rig,
        )
    else:
        scene = FloatingRobotiq2f85(
            *children,
            pos=[-0.1, 0, 0.9],
            **options,
            camera_rig=camera_rig,
        )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Mug Tree task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Mug Tree task loaded successfully!")
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
