from pathlib import Path
from copy import copy
import random
from scipy.spatial.transform import Rotation as R

from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.basic_components.rigs.camera_rig_hand import make_camera_rig
from vuer_mjcf.utils.se3.se3_mujoco import Vector3
from vuer_mjcf.basic_components.force_plate import ForcePlate
from vuer_mjcf.stage_sets._floating_shadowhand import FloatingShadowHand
import numpy as np
# Generate random values for r, g, and b
# r, g, b = random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)

def make_schema(**_):
    from vuer_mjcf.utils.file import Prettify
    from vuer_mjcf.objects.vuer_mug import VuerMug

    table = ConcreteSlab(pos=[0, 0, 0.6], rgba="0.8 0.8 0.8 1", roughness="0.2")

    rig_origin = table.surface_origin + Vector3(x=-0.55, y=0, z=0)
    camera_rig = make_camera_rig(rig_origin)

    mug = VuerMug(pos=[0, -0.1, 0.8], quat=[0, 0, 1, 0])

    work_area = ForcePlate(
        name="start-area",
        pos=(0, 0, 0.6052),
        quat=(0, 1, 0, 0),
        type="box",
        size="0.35 0.35 0.005",
        # rgba=f"{r} {g} {b} 1.0",
        rgba="0.5 0 0 1.0",
    )

    scene = FloatingShadowHand(
        mug,
        table,
        work_area,
        *camera_rig.get_cameras(),
        bimanual=False,
        camera_rig=camera_rig,
        pos=[0, 0, 0.8],
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Flip Mug task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Flip Mug task loaded successfully!")
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
