import os
from pathlib import Path
import random
from copy import copy
import numpy as np

from vuer_mjcf.objects.objaverse_mujoco_bowl import ObjaverseMujocoBowl
from vuer_mjcf.objects.objaverse_mujoco_cup import ObjaverseMujocoCup
from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.objects.mug import ObjaverseMujocoMug
from vuer_mjcf.objects.plate import ObjaverseMujocoPlate
from vuer_mjcf.objects.spoon_7 import ObjaverseMujocoSpoon
from vuer_mjcf.robots.robotiq_2f85 import Robotiq2F85
from vuer_mjcf.objects.cabinet import KitchenCabinet
from vuer_mjcf.objects.cupcake import Cupcake
from vuer_mjcf.objects.dishwasher import KitchenDishwasher
from vuer_mjcf.objects.drawer_stack import DrawerStack
from vuer_mjcf.objects.granite_countertop import GraniteCountertop
from vuer_mjcf.objects.refrigerator import KitchenFridge
from vuer_mjcf.objects.room_wall import RoomWall
from vuer_mjcf.objects.sink_wide import KitchenSinkWide
from vuer_mjcf.schema import Body, Replicate
from vuer_mjcf.basic_components.rigs.camera_rig_lower_fov import make_camera_rig
from vuer_mjcf.stage_sets._floating_shadowhand import FloatingShadowHand
import vuer_mjcf.utils.se3.se3_mujoco as m
from vuer_mjcf.objects.microwave_scaled import KitchenMicrowave
from vuer_mjcf.objects.vuer_mug import VuerMug
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85

x1, y1 = 0.2, -1
origin = m.Vector3(-0.7, -0.2, 0.2)

def make_schema(mode="production", **kwargs):
    from vuer_mjcf.utils.file import Prettify

    quat = m.WXYZ(0.7071068, 0, 0, -0.7071068)

    front_wall = RoomWall(
        panel_sz=(1.1, 1.6, 0.02),
        pos=(0.035, 0.02, 0.89),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room",
        assets="kitchen/wall",
        backing=False,
    )

    side_wall = RoomWall(
        panel_sz=(0.02, 1.6, 1.385),
        pos=(-1.005, -1.385, 0.89),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room_side",
        assets="kitchen/wall",
        backing=False,
    )

    start_indicator = Body(
        pos=(-0.2, 0.1, 0.8),
        attributes=dict(name="start_indicator-1"),
        _children_raw="""
        <site name="start_indicator-1" pos="0 0.0 0.0" size="0.025"/>
        """,
    )

    table_A = GraniteCountertop(
        assets="kitchen/counter", size="1.06 0.325 0.015", attributes={"pos": "0.075 -0.325 0.905", "name": "table_A"}
    )
    table_B = GraniteCountertop(
        assets="kitchen/counter", size="0.325 1.06 0.015", attributes={"pos": "-0.66 -1.71 0.905", "name": "table_B"}
    )

    cameras = make_camera_rig(pos=[-1, 0.1, 1])

    microwave = KitchenMicrowave(pos=[0.035, -0.3, 1.09], quat=[1, 0, 0, 0])

    cabinet_B = KitchenCabinet(assets="kitchen/cabinet", attributes={"pos": "0.5 -0.20 2.0", "name": "cabinet_B"}, remove_joints=True)

    mug = VuerMug(
        pos=[0.45, -0.3, 0.8],
        quat=[0, 0, 0, 1],
        scale="0.0186 0.0284 0.0186",
        _attributes={
            "name": "mug",
            # 0.00933 0.0142 0.00933
        },
        remove_joints=True,
    )
    cupcake = Cupcake(
        pos=[0.4, 0.225, 0.82],
        scale="0.05 0.05 0.05",
    )
    dw = KitchenDishwasher(assets="kitchen/dishwasher", attributes={"pos": "-0.27 -0.075 0.00", "name": "dishwasher", "childclass": "dw"}, remove_joints=True)

    stack_D = DrawerStack(base_pos="0.3 -0.3 0.00", name="stack_D", assets="kitchen", remove_joints=True)
    stack_E = DrawerStack(base_pos="0.81 -0.3 0.00", name="stack_E", assets="kitchen", remove_joints=True)

    if mode == "production":
        extra_children = [
            Body(
                dw,
                stack_D,
                stack_E,
                # cabinet_A,
                cabinet_B,
                attributes=dict(name="furniture", pos=-1 * origin, quat=quat),
                remove_joints=True
            ),
            mug,
        ]
        extra_extra_children = [
            front_wall,
            side_wall,
        ]
    else:
        extra_extra_children = []
        extra_children = []

    # scene = FloatingShadowHand(
    scene = FloatingRobotiq2f85(
        Body(
            *extra_extra_children,
            table_A,
            table_B,
            microwave,
            attributes=dict(name="room", pos=-1 * origin, quat=quat),
        ),
        # muffin,
        start_indicator,
        cupcake,
        *extra_children,
        *cameras.get_cameras(),
        camera_rig=cameras,
        pos=[-0.2, 0.1, 0.8],
        # bimanual=False,
        # quat=[0, 0, 0, 1],
        # dual_gripper=True,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Microwave Muffin task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Microwave Muffin task loaded successfully!")
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
