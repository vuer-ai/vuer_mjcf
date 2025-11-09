import numpy as np

from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
from vuer_mjcf.robots.astribot import Astribot
from vuer_mjcf.robots.robotiq_2f85 import Robotiq2F85
from vuer_mjcf.schema import Mjcf
from vuer_mjcf.utils.se3.rot_gs6 import _compute_look_at_rotation


class AstribotRobotiq2f85(Mjcf):
    """A class that combines UR5 robot with Robotiq gripper."""

    name = "ur5-robotiq-2f85"

    _preamble = """
    <compiler angle="radian" autolimits="true" assetdir="{assets}" meshdir="{assets}" texturedir="{assets}"/>
    <!--<statistic center="0.2 0 0.4" extent=".65"/>-->

    <visual>
      <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
      <rgba haze="0.15 0.25 0.35 1"/>
      <global azimuth="150" elevation="-20" offwidth="1280" offheight="1024"/>
    </visual>

    <asset>
      <texture type="skybox" builtin="gradient" rgb1="0.9 0.7 0.9" rgb2="0.94 0.97 0.97"  width="512" height="3072"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
        markrgb="0.8 0.8 0.8" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    </asset>
    """

    def __init__(
        self,
        *_children,
        
        dual_robot=False,
        camera_rig=None,
        head_quat=None,
        right_mocap_pos=None,
        right_mocap_quat=None,
        left_mocap_pos=None,
        left_mocap_quat=None,
        **kwargs,
    ):
        super().__init__(*_children, **kwargs)

        # if right_mocap_pos is None:
        #     right_mocap_pos="0.3205938935279846 0.01334109716117382 1.215453326702118"
        #     right_mocap_quat=[0.259, 0, 0.966, 0]
        # if left_mocap_pos is None:
        #     left_mocap_pos="-0.3205938935279846 0.01334109716117382 1.215453326702118"
        #     left_mocap_quat=[0.259, 0, 0.966, 0]

        head_target = _compute_look_at_rotation(
            head_pos=self._pos + np.array([0.0, 0.0, 1.5]), target_pos=np.array([0.465, 0.07, 0.82])
        )
        head_quat_wxyz = head_target.wxyz_xyz[:4]  # wxyz
        if head_quat is None:
            head_quat = head_quat_wxyz.tolist()

        if camera_rig is None:
            camera_rig = make_camera_rig(self._pos)
        else:
            print("Using passed in camera rig!")
        light_rig = make_lighting_rig(self._pos)

        # Create UR5 robot with Robotiq gripper
        robot = Astribot(
            name="astribot",
            assets="robots/astribot",
            pos=self._pos,
            head_mocap_quat=head_quat,
            quat=[1, 0, 0, 0],  # Aligned with table scene
            left_end_effector=Robotiq2F85(
                assets="robots/robotiq_2f85",
                name="left_gripper",
                gripper_mass=0.05,
                pos=[0, 0, 0],  # Position at the end of UR5
                quat=[1, 0, 0, 0],  # Aligned with table scene
                wrist_mount=camera_rig.wrist_camera(name="left-wrist"),
                # mocap_pos=f"{self._pos[0] + 0.4} {self._pos[1] + 0.4} {self._pos[2] + 1}",
                mocap_pos=right_mocap_pos,
                mocap_quat=right_mocap_quat,
            ),
            right_end_effector=Robotiq2F85(
                assets="robots/robotiq_2f85",
                name="right_gripper",
                gripper_mass=0.05,
                pos=[0, 0, 0],  # Position at the end of UR5
                quat=[1, 0, 0, 0],  # Aligned with table scene
                wrist_mount=camera_rig.wrist_camera(name="right-wrist"),
                # mocap_pos=f"{self._pos[0] + 0.4} {self._pos[1] + 0.4} {self._pos[2] + 1}",
                mocap_pos=left_mocap_pos,
                mocap_quat=left_mocap_quat,
            ),
        )
        _children = (
            robot,
            robot.left_end_effector._mocaps,
            robot.right_end_effector._mocaps,
            robot._mocaps,
        )
        # _children = (robot,)

        if all(not isinstance(child, str) or "light" not in child for child in _children):
            self._children = (
                light_rig.key,
                light_rig.fill,
                light_rig.back,
            )

        self._children = (
            *self._children,
            *_children,
        )

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify
    from pathlib import Path

    assets = str(Path(__file__).parent.parent.parent / "assets")
    ground = GroundPlane()
    scene = AstribotRobotiq2f85(ground, **options)

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Astribot Robotiq scene")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Astribot Robotiq scene loaded successfully!")
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
