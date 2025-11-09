from pathlib import Path

import numpy as np

from vuer_mjcf.objects.vuer_mug import VuerMug
from vuer_mjcf.basic_components.rigs.camera_rig_hand import make_camera_rig
from vuer_mjcf.schema import Replicate, Body
from vuer_mjcf.third_party.robohive.robohive_object import RobohiveObj
from vuer_mjcf.stage_sets._floating_shadowhand import FloatingShadowHand

center = 0
x1, y1 = center - 0.05, -0.025
x2, y2 = center + 0.17, 0.0

def get_initial_positions():
    base_pos = np.array([0.295, 0.195, 0.85])

    x_offset = np.array([0.01, 0.0, 0.0])
    y_offset = np.array([0.0, 0.01, 0.0])
    z_offset = np.array([0.0, 0.0, 0.01])

    positions = []
    for i in range(3):  # outermost loop (x-axis)
        for j in range(2):  # middle loop (y-axis)
            for k in range(2):  # innermost loop (z-axis)
                pos = base_pos + i * x_offset + j * y_offset + k * z_offset
                positions.append(pos)

    return np.array(positions)

def make_schema(**_):
    from vuer_mjcf.utils.file import Prettify

    robohive_table = RobohiveObj(
        otype="furniture",
        asset_key="simpleTable/simpleWoodTable",
        pos=[0, 0, 0],
        quat=[0.7071, 0, 0, 0.7071],
        local_robohive_root="robohive",
    )
    robohive_table_2 = RobohiveObj(
        otype="furniture",
        asset_key="simpleTable/simpleWoodTable",
        name="robohive_table_2",
        pos=[0.75, 0, 0],
        quat=[0.7071, 0, 0, 0.7071],
        local_robohive_root="robohive",
    )

    particles = Replicate(
        Replicate(
            Replicate(
                Body(
                    name="particle",
                    pos=[0.295, 0.195, 0.85],
                    _children_raw="""
                                    <freejoint/>
                                    <geom size=".008" mass="0.001" rgba=".8 .2 .1 1" condim="1"/>
                                """,
                ),
                _attributes=dict(
                    count=2,
                    offset="0.0 0.0 0.01",
                ),
            ),
            _attributes=dict(
                count=2,
                offset="0.0 0.01 0.0",
            ),
        ),
        _attributes=dict(
            count=3,
            offset="0.01 0.0 0.0",
        ),
    )

    camera_rig = make_camera_rig(pos=[-0.4, 0, 0.77])

    vuer_mug = VuerMug(pos=[0.3, -0.2, 0.8])
    # mujoco_mug = MuJoCoMug(pos=[0.3, 0.2, 0.8])
    from vuer_mjcf.objects.objaverse_mujoco_cup import ObjaverseMujocoCup

    cup = ObjaverseMujocoCup(
        assets="kitchen/cup",
        pos=[0.3, 0.2, 0.8],
        name="cup",
        collision_count=32,
        randomize_colors=False,
    )

    # scene = UR5ShadowHand(
    #     vuer_mug,
    #     mujoco_mug,
    #     particles,
    #     # table,
    #     robohive_table,
    #     robohive_table_2,
    #     *camera_rig.get_cameras(),
    #     bimanual=False,
    #     camera_rig=camera_rig,
    #     pos=[-0.4, 0, 0.77],
    # )

    scene = FloatingShadowHand(
        vuer_mug,
        cup,
        particles,
        # table,
        robohive_table,
        robohive_table_2,
        *camera_rig.get_cameras(),
        bimanual=False,
        camera_rig=camera_rig,
        pos=[-0.4, 0, 0.77],
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Particle Pour task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Particle Pour task loaded successfully!")
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
