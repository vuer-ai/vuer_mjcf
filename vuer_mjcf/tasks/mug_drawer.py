from pathlib import Path

from vuer_mjcf.objects.vuer_mug import VuerMug
from vuer_mjcf.objects.mimicgen_drawer import MimicGenDrawer
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab
from vuer_mjcf.stage_sets._floating_robotiq import FloatingRobotiq2f85

# Generate random values for r, g, and b
# r, g, b = random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)

def make_schema(**_,):
    from vuer_mjcf.utils.file import Prettify

    table = ConcreteSlab(pos=[0, 0, 0.6], rgba="0.8 0.8 0.8 1", roughness="0.2")

    drawer = MimicGenDrawer(pos=[0.1, 0, 0.63], quat=[1, 0, 0, 0])

    mug = VuerMug(pos=[-0.2, -0.3, 0.65], quat=[0, 0, 1, 0])

    scene = FloatingRobotiq2f85(
        drawer,
        mug,
        table,
        pos=[0, 0, 0.8],
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Mug Drawer task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Mug Drawer task loaded successfully!")
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
