import warnings
from pathlib import Path

import numpy as np

from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.objects.orbit_table import OpticalTable

# x1, y1 = random.uniform(0.15, 0.2), random.uniform(-0.1, 0.1)
# x2, y2 = random.uniform(0.2, 0.4), random.uniform(-0.2, 0.2)

# scene center is 0.41 + 0.2 / 2 = 0.50
center = 0.5
x1, y1 = center - 0.07, 0.09
x2, y2 = center - 0.10, -0.05
x3, y3 = center - 0.00, -0.15
x4, y4 = center - 0.15, -0.14

def make_schema(mode="cameraready", robot="panda", show_robot=False, **options):
    from vuer_mjcf.utils.file import Prettify
    from vuer_mjcf.objects.ball import Ball

    if robot == "panda":
        from vuer_mjcf.robots.franka_panda import Panda
        from vuer_mjcf.robots.tomika_gripper import TomikaGripper

        gripper = TomikaGripper(name="test_gripper", mocap_pos="0.25 0.0 1.10", mocap_quat="0 0 1 0")
        panda = Panda(end_effector=gripper, name="test_panda", pos=(0, 0, 0.79), quat=(1, 0, 0, 0))
    else:
        raise ValueError(f"Unknown robot: {robot}")

    optical_table = OpticalTable(
        pos=[0, 0, 0.79],
        assets="optical_table",
        _attributes={"name": "table_optical"},
    )
    # camera_rig = make_camera_rig(optical_table._pos)

    table_slab = ConcreteSlab(
        assets="model",
        pos=[0, 0, 0.777],
        group=4,
        rgba="0.8 0 0 0.9",
        _attributes={
            "name": "table",
        },
    )

    camera_rig = make_camera_rig(table_slab.surface_origin)

    box = MjSDF(
        mass=0.3,
        pos=[x1, y1, 0.85],
        quat=[0.607299, 0.606557, -0.360798, -0.364831],
        assets="ball_sorting_toy",
        _attributes={"name": "ball-sorting-toy", "quat": "0.707 0.707 0 0"},
        additional_children_raw="""
        <!-- Approximated square corners for circle in XZ plane -->
        <site name="ball-sorting-toy_corner1" pos="0.03 0.035 -0.08" size="0.01" rgba="1 1 0 0"/>
        <site name="ball-sorting-toy_corner2" pos="-0.03 0.035 -0.08" size="0.01" rgba="1 1 0 0"/>
        <site name="ball-sorting-toy_corner3" pos="0.0 0.035 -0.05" size="0.01" rgba="1 1 0 0"/>
        <site name="ball-sorting-toy_corner4" pos="0.0 0.035 -0.11" size="0.01" rgba="1 1 0 0"/>
        """,
        scale="0.12 0.12 0.12",
    )
    ball_1 = Ball(
        size="0.022",
        pos=[x2, y2, 0.85],
        quat=[0, 0, 1, 0],
        rgba="1 0 0 1",
        _attributes={
            "name": "ball-1",
        },
    )
    ball_2 = Ball(
        size="0.022",
        pos=[x3, y3, 0.85],
        quat=[0, 0, 1, 0],
        rgba="0 0 1 1",
        _attributes={
            "name": "ball-2",
        },
    )
    ball_3 = Ball(
        size="0.022",
        pos=[x4, y4, 0.85],
        quat=[0, 0, 1, 0],
        rgba="0 1 0 1",
        _attributes={
            "name": "ball-3",
        },
    )

    children = [
        *camera_rig.get_cameras(),
        table_slab,
        box,
        ball_1,
        ball_2,
        ball_3,
    ]

    if mode == "demo":
        pass
    elif mode == "cameraready":
        children += [optical_table]

    if show_robot:
        children.append(panda)
        children.append(gripper._mocaps)
        pass

    scene = FloatingRobotiq2f85(*children, pos=[0, 0, 1.0], **options)

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Obj Permanence task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Obj Permanence task loaded successfully!")
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
