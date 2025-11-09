import random
import warnings
from pathlib import Path
import numpy as np

from vuer_mjcf.objects.sort_shapes import SquareBlock, HexBlock, TriangleBlock, CircleBlock, LeBox
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.objects.orbit_table import OpticalTable

def make_schema(mode="cameraready", robot="panda", show_robot=False, **options):
    from vuer_mjcf.utils.file import Prettify

    if robot == "panda":
        from vuer_mjcf.robots.franka_panda import Panda
        from vuer_mjcf.robots.tomika_gripper import TomikaGripper

        gripper = TomikaGripper(name="test_gripper", mocap_pos="0.25 0.0 1.10", mocap_quat="0 0 1 0")
        panda = Panda(end_effector=gripper, name="test_panda", pos=(0, 0, 0.79), quat=(1, 0, 0, 0))
    else:
        raise ValueError(f"Unknown robot: {robot}")

    optical_table = OpticalTable(
        pos=[0, 0, 0.79],
        assets="optical_table",
        _attributes={"name": "table_optical"},
    )

    table_slab = ConcreteSlab(
        assets="model",
        pos=[0, 0, 0.777],
        group=4,
        rgba="0.8 0 0 0.0",
        _attributes={
            "name": "table",
        },
    )
    camera_rig = make_camera_rig(table_slab.surface_origin)

    scale = 0.075

    lebox = LeBox(pos=[0.5, 0, 0.85], assets="sort_shape", _attributes={"name": "lebox", "quat": "1 0 0 0"}, scale=1.1 * scale)
    hexblock = HexBlock(pos=[0.25, -0.05, 0.85], assets="sort_shape", _attributes={"name": "block_hex", "quat": "1 0 0 0"}, scale=scale)
    squareblock = SquareBlock(
        pos=[0.25, 0.05, 0.85], assets="sort_shape", _attributes={"name": "block_square", "quat": "1 0 0 0"}, scale=scale
    )
    triangleblock = TriangleBlock(
        pos=[0.35, -0.05, 0.85], assets="sort_shape", _attributes={"name": "block_triangle", "quat": "1 0 0 0"}, scale=scale
    )
    circleblock = CircleBlock(
        pos=[0.35, 0.05, 0.85], assets="sort_shape", _attributes={"name": "block_circle", "quat": "1 0 0 0"}, scale=scale
    )

    # work_area = ForcePlate(
    #     name="start-area",
    #     pos=(0.3, 0, 0.605),
    #     quat=(0, 1, 0, 0),
    #     type="box",
    #     size="0.15 0.15 0.01",
    #     rgba="1 0 0 0.1",
    # )
    children = [
        *camera_rig.get_cameras(),
        # work_area,
        table_slab,
        # objects
        lebox,
        hexblock,
        squareblock,
        triangleblock,
        circleblock,
    ]

    if mode == "demo":
        pass
    elif mode == "cameraready":
        children += [optical_table]

    if show_robot:
        children.append(panda)
        children.append(gripper._mocaps)

    scene = FloatingRobotiq2f85(
        *children,
        pos=[0, 0, 1.0],
        **options,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Insert Shapes task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Insert Shapes task loaded successfully!")
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
