import warnings
from pathlib import Path

import numpy as np

from vuer_mjcf.objects.decomposed_obj import ObjMujocoObject
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.rigs.camera_rig_zoomed_out import make_camera_rig
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.stage_sets._floating_shadowhand import FloatingShadowHand
from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
from vuer_mjcf.objects.orbit_table import OpticalTable

def make_schema(**_):
    from vuer_mjcf.utils.file import Prettify

    optical_table = OpticalTable(
        pos=[-0.2, 0, 0.79],
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
    lighting = make_lighting_rig(optical_table._pos)
    camera_rig = make_camera_rig(table_slab.surface_origin)

    basketball_hoop = ObjMujocoObject(
        name="basketball_hoop",
        assets="basketball_hoop",
        prefix="hoop",
        visual_count=1,
        pos=[-0.7, 0, 0.85],
        quat=[0.7071068, 0.7071068, 0, 0],
        scale=0.5,
        collision_count=12,
        textures=["DefaultMaterial_baseColor"],
        free=False,
        randomize_colors=True,
        _additional_children_raw = """
        <site name="{prefix}_corner1" pos="0.425 0.45 -0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner2" pos="0.605 0.45 -0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner3" pos="0.605 0.45  0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner4" pos="0.425 0.45  0.09" size="0.01" rgba="1 1 0 0"/>
        """
    )
    basketball = ObjMujocoObject(
        name="basketball",
        assets="basketball",
        visual_count=1,
        pos=[0, 0, 0.85],
        scale=0.0215,
        collision_count=1,
        textures=["Basketball_size6_baseColor"],
        randomize_colors=True,
        _additional_children_raw = """
        <site name="{name}" pos="0 0.07 0" size="0.0215" rgba="1 1 0 0"/>
        """
    )

    scene = FloatingShadowHand(
        optical_table,
        table_slab,
        *camera_rig.get_cameras(),
        basketball_hoop,
        basketball,
        pos=[0, 0, 0.8],
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Basketball Shot task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Basketball Shot task loaded successfully!")
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
