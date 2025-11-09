import random
from copy import copy
from pathlib import Path

import numpy as np

from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.utils.se3.se3_mujoco import Vector3
from vuer_mjcf.schema import Body
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.stage_sets._floating_shadowhand import FloatingShadowHand
from vuer_mjcf.basic_components.force_plate import ForcePlate

def make_schema(**_):
    from vuer_mjcf.utils.file import Prettify

    table = ConcreteSlab(pos=[0, 0, 0.6], rgba="0.8 0.8 0.8 1")
    box = Body(
        pos=(0, 0.24, 0.7),
        attributes=dict(name="box-1"),
        _children_raw="""
        <joint type="free" name="{name}"/>
        <geom name="box-1" type="box" pos="0 0.0 0.0"  rgba="0 1 0 1" size="0.025 0.025 0.025" density="1000"/>
        <site name="box-1" pos="0 0.0 0.0" size="0.025"/>
        """,
    )

    start_area = ForcePlate(
        name="start-area",
        pos=(0, 0.24, 0.6052),
        quat=(0, 1, 0, 0),
        type="box",
        size="0.2 0.2 0.005",
        rgba="1 0 0 1",
    )

    rig_origin = table.surface_origin + Vector3(x=-0.55, y=0, z=0)
    camera_rig = make_camera_rig(rig_origin)

    print(table.surface_origin, type(table.surface_origin))

    goal_area = ForcePlate(
        name="goal-area",
        pos=(0, -0.24, 0.6052),
        quat=(0, 1, 0, 0),
        type="box",
        size="0.2 0.2 0.005",
        rgba="0.137 0.667 1. 1.",
    )

    # scene = FloatingRobotiq2f85(
    #     table,
    #     start_area,
    #     goal_area,
    #     box,
    #     *camera_rig.get_cameras(),
    #     pos=[0, 0, 0.8],
    # )

    scene = FloatingShadowHand(
        table,
        start_area,
        goal_area,
        box,
        *camera_rig.get_cameras(),
        bimanual=False,
        pos=[-0.3, -0.15, 0.4],
    )

    xml = scene._xml | Prettify()

    # model = mujoco.MjModel.from_xml_string(xml)
    #
    # new_data = mujoco.MjData(model).copy()
    #
    # new_data.qpos[7:14] = [-0.5, .15, 1.1, 0, 0.707, 00, 0.707] # set hand to init pos
    #
    # mujoco.mj_forward(modle, new_data)

    return xml

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Pick Place Hand task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Pick Place Hand task loaded successfully!")
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
