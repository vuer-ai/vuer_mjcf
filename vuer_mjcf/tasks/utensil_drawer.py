import os
from pathlib import Path
import random
from copy import copy
import numpy as np

from vuer_mjcf.objects.bagel import Bagel
from vuer_mjcf.objects.cabinet import KitchenCabinet
from vuer_mjcf.objects.drawer_stack import DrawerStack
from vuer_mjcf.objects.objaverse_mujoco_bowl import ObjaverseMujocoBowl
from vuer_mjcf.objects.objaverse_mujoco_cup import ObjaverseMujocoCup
from vuer_mjcf.objects.spoon_7 import ObjaverseMujocoSpoon
from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.objects.mug import ObjaverseMujocoMug
from vuer_mjcf.objects.plate import ObjaverseMujocoPlate
from vuer_mjcf.objects.spoon_7 import ObjaverseMujocoSpoon
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
from vuer_mjcf.objects.objaverse_mujoco_cup import ObjaverseMujocoCup
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85

x1, y1 = 0.2, -1
origin = m.Vector3(-0.7, -0.2, 0)

def make_schema(mode="production",**kwargs):
    from vuer_mjcf.utils.file import Prettify

    quat = m.WXYZ(0.7071068, 0, 0, -0.7071068)

    front_wall = RoomWall(
        panel_sz=(2.1, 1.6, 0.02),
        pos=(0.035, 0.02, 0.89),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room",
        assets="kitchen/wall",
        backing=False,
    )
    cup = ObjaverseMujocoCup(assets="kitchen/cup_4", pos=[0.3, 0, 1.0], name="cup", collision_count=32, randomize_colors=False)
    spoon = ObjaverseMujocoSpoon(assets="kitchen/spoon", pos=[0.3, 0, 1.1], quat=[0, 1, 0, 0], name="spoon", collision_count=32, randomize_colors=False)

    side_wall = RoomWall(
        panel_sz=(0.02, 1.6, 1.385),
        pos=(-1.005, -1.385, 0.89),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room_side",
        assets="kitchen/wall",
        backing=False,
    )
    mug = VuerMug(
        pos=[0.5, -0.15, 0.95],
        quat=[1, 0, 0, 0],
        _attributes={
            "name": "mug",
        },
        remove_joints=True,
    )

    cupcake = Cupcake(
        pos=[0.4, 0.225, 0.82],
        scale="0.05 0.05 0.05",
        remove_joints=True,
    )
    
    start_indicator = Body(
        pos=(-0.2, 0.1, 0.8),
        attributes=dict(name="start_indicator-1"),
        _children_raw="""
        <site name="start_indicator-1" pos="0 0.0 0.0" size="0.025"/>
        """,
    )

    table_A = GraniteCountertop(
        assets="kitchen/counter", size="2 0.325 0.015", attributes={"pos": "0.075 -0.325 0.905", "name": "table_A"}
    )

    stack_D = DrawerStack(base_pos="-0.2 -0.3 0.00", name="stack_D", assets="kitchen")
    stack_E = DrawerStack(base_pos="0.31 -0.3 0.00", name="stack_E", assets="kitchen")

    cabinet_A = KitchenCabinet(assets="kitchen/cabinet", attributes={"pos": "-0.77 -0.20 2.0", "name": "cabinet_A"}, remove_joints=True)
    cabinet_B = KitchenCabinet(assets="kitchen/cabinet", attributes={"pos": "0.88 -0.20 2.0", "name": "cabinet_B"}, remove_joints=True)

    cameras = make_camera_rig(pos=[-1.25, -0.1, 1.1])

    microwave = KitchenMicrowave(pos=[0.8, -0.3, 1.09],
                                 quat=[1, 0, 0, 0], remove_joints=True)

    bagel = Bagel( attributes={"pos": "0.5 0.3 0.95", "name": "bagel"}, remove_joints=True)
    bowl = ObjaverseMujocoBowl(assets="kitchen/bowl", collision_count=32, scale=0.2, pos=[0.3, 0.4, 0.95], name="bowl", remove_joints=True)

    dw = KitchenDishwasher(assets="kitchen/dishwasher", attributes={"pos": "0.88 -0.075 0.00", "name": "dishwasher", "childclass": "dw"}, remove_joints=True)
    dw2 = KitchenDishwasher(
        assets="kitchen/dishwasher", prefix="dw2", attributes={"pos": "0.88 -0.075 0.00", "name": "dishwasher2", "childclass": "dw"}, remove_joints=True
    )

    extra_children = []
    if mode == "production":
        extra_children = [
            dw,
            cabinet_A,
            cabinet_B,
        ]

    # scene = FloatingShadowHand(
    scene = FloatingRobotiq2f85(
        Body(
            front_wall,
            table_A,
            stack_D,
            stack_E,
            *extra_children,
            attributes=dict(name="room", pos=-1 * origin, quat=quat),
        ),
        cup,
        spoon,
        bagel,
        mug,
        cupcake,
        bowl,
        start_indicator,
        *cameras.get_cameras(),
        camera_rig=cameras,
        pos=[-0.3, 0, 0.95],
        # quat=[0, 0, 0, 1],
        # dual_gripper=True,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Utensil Drawer task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Utensil Drawer task loaded successfully!")
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
