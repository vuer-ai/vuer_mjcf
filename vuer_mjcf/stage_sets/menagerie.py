from pathlib import Path

from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
from vuer_mjcf.schema import Mjcf

from vuer_mjcf.robots.trossen_widow_x_arm import WidowXArm
from vuer_mjcf.robots.astribot import Astribot
from vuer_mjcf.robots.lift import Lift
from vuer_mjcf.robots.r5a import R5a
from vuer_mjcf.robots.ur5e import UR5e
from vuer_mjcf.robots.x5a import X5a
from vuer_mjcf.robots.x7 import X7
from vuer_mjcf.robots.xhand import XHandRight, XHandLeft

class Menagerie(Mjcf):
    name = "All-Robots"
    _attributes = {
        "model": "all",
    }

    _preamble = """
    <compiler angle="radian" meshdir="{assets}" texturedir="{assets}" autolimits="true"/>

    <option integrator="implicitfast" cone="elliptic" impratio="10"/>

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

    def __init__(self, *_children,  **kwargs):
        super().__init__(*_children, **kwargs)

        camera_rig = make_camera_rig(self._pos)
        light_rig = make_lighting_rig(self._pos)

        kwargs["assets"] = "robots/trossen_widow_x_arm"
        trossen_arm = WidowXArm(
            name="widowxarm",
            pos=[1, 1, 0],
            quat=[1, 0, 0, 0],
            **kwargs
        )

        kwargs["assets"] = "robots/astribot"
        astribot = Astribot(
            name="astribot",
            pos= [0, 0, 0],
            **kwargs
        )

        kwargs["assets"] = "robots/lift"
        lift = Lift(
            name="lift",
            pos=[0, 1, 0],
            quat=[0, 0, 0, 1],
            **kwargs
        )

        kwargs["assets"] = "robots/r5a"
        r5a = R5a(
            name="r5a",
            pos= [1, -0.33, 0],
            **kwargs
        )

        kwargs["assets"] = "robots/x5a"
        x5a = X5a(
            name="x5a",
            pos=[1, 0.33, 0],
            **kwargs
        )

        kwargs["assets"] = "robots/x7"
        x7 = X7(
            name="x7",
            assets="robots/x7",
            pos=[0, -1, 0],
        )

        kwargs["assets"] = "robots/xhand_right"
        xhand_right = XHandRight(
            name="xhand_right",
            pos=[1, -1, 0],
            **kwargs
        )

        kwargs["assets"] = "robots/xhand_left"
        xhand_left = XHandLeft(
            name="xhand_left",
            pos=[1, -1, 0.1],
            **kwargs
        )

        _children = (trossen_arm, astribot, lift, r5a, x5a, x7, xhand_right, xhand_left)
        # _children = (robot,)
        for child in _children:
            child._add_mocaps()

        self._children = (
            *self._children,
            *_children,
        )

def make_schema(mode="cameraready", robot="trossen_widow_x_arm", show_robot=True, **options):
    from vuer_mjcf.utils.file import Prettify
    from pathlib import Path
    assets = str(Path(__file__).parent.parent.parent / "assets")

    scene = Menagerie(
        assets=assets,
        **options
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile

    # Set correct assets path

    xml_str = make_schema()
    print("Generated XML for Menagerie")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Menagerie loaded successfully!")
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