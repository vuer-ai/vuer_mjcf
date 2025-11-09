#!/usr/bin/env python3
"""
Generate screenshots for all stage scenes using MuJoCo's offscreen rendering.
"""

import os
import sys
import tempfile
from pathlib import Path
import mujoco
import numpy as np
try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    try:
        import matplotlib.pyplot as plt
        import matplotlib
        matplotlib.use('Agg')  # Non-interactive backend
        HAS_MPL = True
    except ImportError:
        HAS_MPL = False

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

# Output directory
DOCS_DIR = Path(__file__).parent / "docs" / "examples"
STAGES_FIG_DIR = DOCS_DIR / "stages" / "figures"
ROBOTS_FIG_DIR = DOCS_DIR / "robots" / "figures"

# Ensure directories exist
STAGES_FIG_DIR.mkdir(parents=True, exist_ok=True)
ROBOTS_FIG_DIR.mkdir(parents=True, exist_ok=True)

# Stage scenes to screenshot
STAGE_SCENES = {
    "astribot_robotiq": "vuer_mjcf.stage_sets.astribot_robotiq.AstribotRobotiq2f85",
    "astribot_shadow_hand": "vuer_mjcf.stage_sets.astribot_shadow_hand.AstribotHand",
    "astribot_scene": "vuer_mjcf.stage_sets.astribot_scene.AstriBotScene",
    "default_scene": "vuer_mjcf.stage_sets.default_scene.DefaultStage",
    "dual_panda": "vuer_mjcf.stage_sets.dual_panda.DualPanda",
    "g1_scene": "vuer_mjcf.stage_sets.g1_scene.G1Scene",
    "lift_scene": "vuer_mjcf.stage_sets.lift_scene.LiftScene",
    "menagerie": "vuer_mjcf.stage_sets.menagerie.Menagerie",
    "panda_tomika": "vuer_mjcf.stage_sets.panda_tomika_scene.PandaTomika",
    "r5a_scene": "vuer_mjcf.stage_sets.r5a_scene.R5aScene",
    "ufactory_xarm7_scene": "vuer_mjcf.stage_sets.ufactory_xarm7_scene.UfactoryXArm7Scene",
    "ur5e_robotiq": "vuer_mjcf.stage_sets.ur5e_robotiq_scene.UR5Robotiq2f85",
    "ur5e_shadow_hand": "vuer_mjcf.stage_sets.ur5e_shadow_hand_scene.UR5ShadowHand",
    "widow_x_scene": "vuer_mjcf.stage_sets.single_widow_x_arm.FloatingWidowXArm",
    "x5a_scene": "vuer_mjcf.stage_sets.x5a_scene.X5aScene",
    "x7_scene": "vuer_mjcf.stage_sets.x7_scene.X7Scene",
    "x7s_scene": "vuer_mjcf.stage_sets.x7s_scene.X7sScene",
    "xhand_scene": "vuer_mjcf.stage_sets.xhand_scene.XHandScene",
}
# Note: panda_shadow_hand and trossen_table are skipped due to known issues

# Assets path
ASSETS_PATH = Path(__file__).parent / "assets" / "robots"


def import_class(class_path: str):
    """Dynamically import a class from a module path."""
    module_path, class_name = class_path.rsplit(".", 1)
    module = __import__(module_path, fromlist=[class_name])
    return getattr(module, class_name)


def render_scene(xml_string: str, width: int = 800, height: int = 600) -> np.ndarray:
    """Render a scene from XML string using offscreen rendering."""
    # Create model and data
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    # Forward kinematics
    mujoco.mj_forward(model, data)

    # Create renderer
    renderer = mujoco.Renderer(model, height=height, width=width)

    # Update scene with default camera
    renderer.update_scene(data)

    # Render and get pixels
    pixels = renderer.render()

    return pixels


def generate_screenshot(scene_name: str, class_path: str):
    """Generate a screenshot for a scene."""
    print(f"Generating screenshot for {scene_name}...")

    try:
        # Import the scene class
        SceneClass = import_class(class_path)

        # Instantiate the scene with assets path
        try:
            scene = SceneClass(assets=str(ASSETS_PATH))
        except TypeError:
            # Some scenes might not take assets parameter
            scene = SceneClass()

        # Get XML string
        from vuer_mjcf.utils.file import Prettify
        xml_string = scene._xml | Prettify()

        # Render the scene
        pixels = render_scene(xml_string)

        # Save image
        output_path = STAGES_FIG_DIR / f"{scene_name}.png"

        if HAS_PIL:
            img = Image.fromarray(pixels)
            img.save(output_path)
        elif HAS_MPL:
            plt.imsave(output_path, pixels)
        else:
            # Fallback: save as raw numpy array
            np.save(output_path.with_suffix('.npy'), pixels)
            print(f"  ! Warning: Saved as .npy (install Pillow or matplotlib for PNG)")

        print(f"  ✓ Saved to {output_path}")
        return True

    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False


def main():
    """Generate all screenshots."""
    print(f"Assets path: {ASSETS_PATH}")
    print(f"Output directory: {STAGES_FIG_DIR}")
    print(f"\nGenerating screenshots for {len(STAGE_SCENES)} scenes...\n")

    success_count = 0
    failed_scenes = []

    for scene_name, class_path in STAGE_SCENES.items():
        if generate_screenshot(scene_name, class_path):
            success_count += 1
        else:
            failed_scenes.append(scene_name)

    print(f"\n{'='*60}")
    print(f"Results: {success_count}/{len(STAGE_SCENES)} screenshots generated successfully")

    if failed_scenes:
        print(f"\nFailed scenes:")
        for scene in failed_scenes:
            print(f"  - {scene}")

    print(f"\nScreenshots saved to: {STAGES_FIG_DIR}")


if __name__ == "__main__":
    main()
