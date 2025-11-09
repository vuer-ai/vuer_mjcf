from pathlib import Path

from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85
from vuer_mjcf.basic_components.force_plate import ForcePlate

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify
    from vuer_mjcf.objects.sort_shapes import LeBox, TriangleBlock, CircleBlock, SquareBlock, HexBlock

    table = ConcreteSlab(pos=[0, 0, 0.6], rgba="0.8 0.8 0.8 1")
    scale = 0.075

    ibox = LeBox(attributes={"name": "insertion-box"}, assets="sort_shape", pos=[0, 0, 0.7], scale=1.1*scale)
    square_block = SquareBlock(attributes={"name": "square"}, assets="sort_shape", pos=[0.2, -0.1, 0.7], scale=scale)
    triangle_block = TriangleBlock(attributes={"name": "triangle"}, assets="sort_shape", pos=[0.4, -0.1, 0.7], scale=scale)
    hex_block = HexBlock(attributes={"name": "hex"}, assets="sort_shape", pos=[0.2, 0.1, 0.7], scale=scale)
    circle_block = CircleBlock(attributes={"name": "circle"}, assets="sort_shape", pos=[0.4, 0.1, 0.7], scale=scale)

    work_area = ForcePlate(
        name="start-area",
        pos=(0.3, 0, 0.605),
        quat=(0, 1, 0, 0),
        type="box",
        size="0.15 0.15 0.01",
        rgba="1 0 0 0.1",
    )

    scene = FloatingRobotiq2f85(
        table,
        ibox,
        square_block,
        triangle_block,
        hex_block,
        circle_block,
        work_area,
        pos=[0, 0, 0.8],
        **options,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Sort Blocks task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Sort Blocks task loaded successfully!")
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
