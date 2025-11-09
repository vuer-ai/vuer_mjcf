from pathlib import Path

import numpy as np

from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.stage_sets.astribot_robotiq import AstribotRobotiq2f85
from vuer_mjcf.basic_components.camera import make_camera
from vuer_mjcf.schema import FreeBody, Body
from vuer_mjcf.basic_components.rigs.camera_rig_calibrated import make_camera_rig, FOV
from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.objects.orbit_table import OpticalTable
import mink

center = 0
x1, y1 = (
    0.05,
    0,
)

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

def make_schema(mode="cameraready", robot="panda", show_robot=False, **options):
    from vuer_mjcf.objects.rope import MuJoCoRope
    from vuer_mjcf.utils.file import Prettify

    optical_table = OpticalTable(
        pos=[-0.4, 0, 0.77],
        assets="optical_table",
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
    camera_rig = make_camera_rig(pos=[-0.3, 0, 1.07])
    camera_rig.left_camera._xml = camera_rig.left_camera._xml.replace("0.14 0.355 1.395", "0.14 0.355 1.2")
    camera_rig.right_camera._xml = camera_rig.right_camera._xml.replace("pos=\"0.07500000000000001 -0.355 1.395\"", "pos=\"0.07500000000000001 -0.355 1.2\"")
    def rotated_wrist_camera(name="wrist", pos="0.1 0.0 0.01", quat="-0.11 0.7 -0.7 0.11"):
        return make_camera(
            name,
            pos=pos,
            quat=quat,
            fovy=FOV,
        )
    camera_rig.wrist_camera = rotated_wrist_camera

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

    rope = MuJoCoRope(
        pos=[x1, y1, 1.1],
        damping=0.0005,
        twist=1e1,
        bend=1e1,
        geom_size=0.008,
        attributes={
            "prefix": "rope_",
            "count": "50 1 1",
            "curve": "s",
            "size": "0.43",
            "initial": "none",
            "offset": "0 0 0.8",
        },
    )
    rope_anchor = Body(
        """
          <!-- translate along x -->
        <joint name="jx" type="slide" axis="1 0 0"
               damping="500" frictionloss="200" armature="5"/>
        <!-- translate along y -->
        <joint name="jy" type="slide" axis="0 1 0"
               damping="500" frictionloss="200" armature="5"/>
        <!-- optional: rotate about z in the plane -->
        <!-- <joint name="jz" type="hinge" axis="0 0 1"
               damping="200" frictionloss="50" armature="1"/> -->
        <!-- your geoms here -->
        <geom type="sphere" size="0.01" contype="0" conaffinity="0" rgba="1 1 1 0"/>
        """,
        pos=[x1, y1, 1.1],
        quat=[1, 0, 0, 0],
        attributes={"name": "rope_anchor"},
        postamble="""
        <equality>
            <weld body1="rope_B_first" body2="rope_anchor" relpose="0 0 0  1 0 0 0" solref="0.001 3" solimp="0.9 0.95 0.01"/>
        </equality>
        """,
    )
    rope = FreeBody(rope)

    children = [
        *camera_rig.get_cameras(),
        table,
        rope,
        rope_anchor,
    ]

    if mode == "demo":
        pass
    elif mode == "cameraready":
        children += [optical_table]

    if show_robot:
        children.append(panda)
        children.append(gripper._mocaps)
        
    base_pos = np.array([-0.6, -0.1, 0])
    head_target = _compute_look_at_rotation(head_pos=base_pos + np.array([0.0, 0.0, 1.5]), target_pos=np.array([x1, y1, 1.1]))

    head_quat_wxyz = head_target.wxyz_xyz[:4]  # wxyz

    scene = AstribotRobotiq2f85(
        *children,
        pos=base_pos.tolist(),
        mocap_pos="0 0 1.35",
        mocap_quat=[0, 0, -1, 0],
        head_quat=head_quat_wxyz.tolist(),
        camera_rig=camera_rig,
        **options,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Tie Knot Astribot task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Tie Knot Astribot task loaded successfully!")
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
