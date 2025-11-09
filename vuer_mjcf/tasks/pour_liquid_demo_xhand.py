from pathlib import Path

import numpy as np

from vuer_mjcf.basic_components.rigs.camera_rig_stereo import make_origin_stereo_rig
from vuer_mjcf.objects.objaverse_mujoco_bowl import ObjaverseMujocoBowl
from vuer_mjcf.objects.objaverse_mujoco_cup import ObjaverseMujocoCup
from vuer_mjcf.objects.mug import ObjaverseMujocoMug
from vuer_mjcf.objects.plate import ObjaverseMujocoPlate
from vuer_mjcf.objects.tshape import TShape
from vuer_mjcf.objects.spoon_7 import ObjaverseMujocoSpoon
from vuer_mjcf.objects.cabinet import KitchenCabinet
from vuer_mjcf.objects.dishwasher import KitchenDishwasher
from vuer_mjcf.objects.drawer_stack import DrawerStack
from vuer_mjcf.objects.granite_countertop import GraniteCountertop
from vuer_mjcf.objects.microwave_scaled import KitchenMicrowave
from vuer_mjcf.objects.refrigerator import KitchenFridge
from vuer_mjcf.objects.room_wall import RoomWall
from vuer_mjcf.objects.sink_wide import KitchenSinkWide
from vuer_mjcf.schema import Body, Replicate
from vuer_mjcf.stage_sets._floating_xhand import FloatingXHand
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.stage_sets._floating_shadowhand import FloatingShadowHand
import vuer_mjcf.utils.se3.se3_mujoco as m

x1, y1 = 0.2, -1
origin = m.Vector3(0, -1.0, 0)
nx, ny, nz = 2, 2, 6

def make_schema(**kwargs):
    from vuer_mjcf.utils.file import Prettify

    quat = m.WXYZ(0.7071068, 0, 0, -0.7071068)

    front_wall = RoomWall(
        panel_sz=(1.8, 1.0, 0.02),
        pos=(0.65, 0.02, 1.0),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room",
        assets="kitchen/wall",
        backing=False,
    )

    cabinet_A = KitchenCabinet(assets="kitchen/cabinet", attributes={"pos": "0.3 -0.20 2.0", "name": "cabinet_A"}, show_texture=False)
    cabinet_B = KitchenCabinet(assets="kitchen/cabinet", attributes={"pos": "1.8 -0.20 2.0", "name": "cabinet_B"}, show_texture=False)

    stack_B = DrawerStack(base_pos="0.51 -0.3 0.00", name="stack_B", assets="kitchen", visual=True)
    stack_C = DrawerStack(base_pos="1.02 -0.3 0.00", name="stack_C", assets="kitchen", visual=True)

    table_A = GraniteCountertop(
        assets="kitchen/counter", size="0.35 0.325 0.015", attributes={"pos": "0.075 -0.325 0.905", "name": "table_A"}
    )
    table_B = GraniteCountertop(
        assets="kitchen/counter", size="0.36 0.0345 0.015", attributes={"pos": "0.77 -0.616 0.905", "name": "table_B"}
    )
    table_C = GraniteCountertop(
        assets="kitchen/counter", size="0.36 0.0345 0.015", attributes={"pos": "0.77 -0.01 0.905", "name": "table_C"}
    )
    table_D = GraniteCountertop(
        assets="kitchen/counter", size="0.35 0.325 0.015", attributes={"pos": "1.41 -0.325 0.905", "name": "table_D"}
    )

    countertop = GraniteCountertop(
        assets="kitchen/counter", size="1.06 0.325 0.015", attributes={"pos": "0.75 0.325 1.1", "name": "countertop"}
    )
    sink = KitchenSinkWide(assets="kitchen/sink_wide", attributes={"pos": "0.75 -0.31 0.94"})

    dw = KitchenDishwasher(assets="kitchen/dishwasher", attributes={"pos": "-0.1 -0.075 0.00", "name": "dishwasher", "childclass": "dw"})
    dw2 = KitchenDishwasher(assets="kitchen/dishwasher", prefix="dw2", attributes={"pos": "1.6 -0.075 0.00", "name": "dishwasher2", "childclass": "dw"})
    fridge = KitchenFridge(assets="kitchen/fridge", attributes={"pos": "-0.85 -0.45 0.95"})

    microwave = KitchenMicrowave(pos=[0.1, -0.3, 1.09],
                                 quat=[1, 0, 0, 0])

    t_pos = list(np.array([-0.4, -0.5, 0.95]) - origin)
    # t_shape = TShape(assets="kitchen/tshape", pos=t_pos, name="tshape", scale=0.3, collision_count=32, randomize_colors=False)

    plate = ObjaverseMujocoPlate(assets="kitchen/plate", pos=[0, -0.5, 0.95], name="plate", collision_count=32, randomize_colors=False)
    mug_pos = list(np.array([-0.4, -1.25, 1]) - origin)
    mug = ObjaverseMujocoMug(assets="kitchen/mug", pos=mug_pos, collision_count=32, visual_count=2)
    bowl_pos = list(np.array([-0.2, -1.25, 1]) - origin)
    bowl = ObjaverseMujocoBowl(assets="kitchen/bowl", collision_count=32, scale=0.2, pos=bowl_pos, name="bowl")
    spoon_pos = list(np.array([-0.4, -1.45, 1.2]) - origin)
    spoon = ObjaverseMujocoSpoon(assets="kitchen/spoon", pos=spoon_pos, name="spoon", scale=0.3, collision_count=32, randomize_colors=False)

    cup_pos = list(np.array([x1, y1, 1.2]) - origin)
    cup = ObjaverseMujocoCup(assets="kitchen/cup", pos=cup_pos, name="cup", collision_count=32, randomize_colors=False)

    particles_pos = list(np.array([x1 - 0.01, y1 - 0.01, 1.18]) - origin)
    particles = Replicate(
        Replicate(
            Replicate(
                Body(
                    pos=particles_pos,
                    _attributes={
                        "name": "particle",
                    },
                    _children_raw="""
                            <freejoint/>
                            <geom size=".007" rgba=".8 .2 .1 1" condim="1" solref="0.001 1" solimp="0.99 0.99 0.001"/>
                        """,

                ),
                _attributes=dict(
                    count=nz,
                    offset="0.0 0.0 0.014",
                )
            ),
            _attributes=dict(
                count=ny,
                offset="0.0 0.014 0.0",
            ),
        ),
        _attributes=dict(
            count=nx,
            offset="0.014 0.0 0.0",
        )
    )

    cameras = make_camera_rig(pos=[x1 - 0.6, y1 , 1.2])
    stereo_cameras = make_origin_stereo_rig(pos=[x1 - 0.8, y1 , 1.0])

    # scene = FloatingShadowHand(
    scene = FloatingXHand(
        Body(
            front_wall,
            cabinet_A,
            # cabinet_B,
            # stack_B,
            # stack_C,
            table_A,
            table_B,
            table_C,
            table_D,
            # countertop,
            sink,
            # oven,
            # microwave,
            # dw,
            # dw2,
            # fridge,
            attributes=dict(name="room", pos=-1 * origin, quat=quat),
        ),
        # mug,
        bowl,
        # spoon,
        # cup,
        # t_shape,
        # dish_drainer,
        # plate,
        # particles,
        *cameras.get_cameras(),
        *stereo_cameras.get_cameras(),
        pos=(np.array([-0.8, -1.15, 1.1]) - origin).tolist(),
        # bimanual=False,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Pour Liquid Demo Xhand task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Pour Liquid Demo Xhand task loaded successfully!")
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
