import os

from pathlib import Path

import numpy as np

from vuer_mjcf.basic_components.rigs.camera_rig_stereo import make_origin_stereo_rig
from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.objects.mug import ObjaverseMujocoMug
from vuer_mjcf.objects.rope import MuJoCoRope
from vuer_mjcf.objects.sort_shapes import LeBox, SquareBlock, TriangleBlock, HexBlock, CircleBlock
from vuer_mjcf.objects.vuer_mug import VuerMug
from vuer_mjcf.schema import FreeBody, Body
from vuer_mjcf.stage_sets.ur5e_robotiq_scene import UR5Robotiq2f85
from vuer_mjcf.basic_components.rigs.camera_rig_calibrated import make_camera_rig
from itertools import combinations
from math import hypot
import random

from vuer_mjcf.third_party.robohive.robohive_object import RobohiveObj

x_center, y_center = 0.3, 0.15
x1, y1 = 0.3, -0.25
x2, y2 = 0.45, -0.15

def sample_two_points(
    *,
    xmin,
    xmax,
    ymin,
    ymax,
    d,
    eps,
    exclude_center=None,  # (xc, yc) or None
    exclude_radius=0.0,  # distance to keep away from the center
    max_attempts=10_000,
):
    xs = np.arange(xmin, xmax + 1e-12, d)
    ys = np.arange(ymin, ymax + 1e-12, d)
    grid = np.array(np.meshgrid(xs, ys)).T.reshape(-1, 2)

    if grid.shape[0] < 3:
        raise ValueError("Grid contains fewer than three points.")

    # Pre-compute mask for the exclusion zone, if requested
    if exclude_center is not None and exclude_radius > 0.0:
        cx, cy = exclude_center
        dists_to_center = np.hypot(grid[:, 0] - cx, grid[:, 1] - cy)
        valid_mask = dists_to_center > exclude_radius
        candidate_indices = np.nonzero(valid_mask)[0]
        if len(candidate_indices) < 3:
            raise ValueError("Exclusion zone leaves fewer than three valid points.")
    else:
        candidate_indices = np.arange(len(grid))

    for _ in range(max_attempts):
        idx = random.sample(list(candidate_indices), k=2)
        pts = grid[idx]

        # pair-wise separation constraint
        if all(hypot(*(a - b)) > eps for a, b in combinations(pts, 2)):
            return [tuple(p) for p in pts]

    raise ValueError("Unable to find three points satisfying all constraints; " "relax `eps` / `exclude_radius` or enlarge the grid.")

def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify
    from vuer_mjcf.objects.cylinder import Cylinder

    robohive_table = RobohiveObj(
        otype="furniture",
        asset_key="simpleTable/simpleWoodTable",
        pos=[0.4, 0, 0],
        quat=[0.7071, 0, 0, 0.7071],
        local_robohive_root="robohive",
    )
    camera_rig = make_camera_rig(pos=[0, 0, 0.77])
    stereo_rig = make_origin_stereo_rig(pos=[0, 0, 0.77])

    ibox = LeBox(attributes={"name": "insertion-box"}, assets="sort_shape", pos=[0.5, 0, 0.82], scale=0.08)
    square_block = SquareBlock(attributes={"name": "square"}, assets="sort_shape", pos=[0.3, -0.0, 0.82], scale=0.07)
    triangle_block = TriangleBlock(attributes={"name": "triangle"}, assets="sort_shape", pos=[0.4, -0.0, 0.82], scale=0.07)
    hex_block = HexBlock(attributes={"name": "hex"}, assets="sort_shape", pos=[0.3, 0.1, 0.82], scale=0.07)
    circle_block = CircleBlock(attributes={"name": "circle"}, assets="sort_shape", pos=[0.4, 0.1, 0.82], scale=0.07)

    rope = MuJoCoRope(
        pos=[x1, -y1, 1.1],
        damping=0.0005,
        twist=1e1,
        bend=1e1,
        geom_size=0.008,
        attributes={
            "prefix": "rope_",
            "count": "30 1 1",
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
        pos=[x1, -y1, 1.1],
        quat=[1, 0, 0, 0],
        attributes={"name": "rope_anchor"},
        postamble="""
            <equality>
                <weld body1="rope_B_first" body2="rope_anchor" relpose="0 0 0  1 0 0 0" solref="0.001 3" solimp="0.9 0.95 0.01"/>
            </equality>
            """,
    )
    rope = FreeBody(rope)

    box = MjSDF(
        pos=[0.465, 0.27, 0.82],
        quat=[1, 0, 0, 0],
        assets="ball_sorting_toy",
        geom_quat = "-0.5 -0.5 0.5 0.5",
        _attributes={"name": "ball-sorting-toy"},
        scale="0.12 0.12 0.12",
        additional_children_raw="""
        <site name="hole-1" pos="0.08 0 0.05" size="0.03" rgba="1 0 0 0" type="sphere"/>
        """
    )
    #
    ball_1 = Cylinder(
        pos=[x_center + 0.05, y_center, 0.79],
        quat=[0, 0, 1, 0],
        rgba="1 0.5 0 1",
        _attributes={
            "name": "cylinder-orange",
        },
    )
    ball_2 = Cylinder(
        pos=[x_center - 0.05, y_center, 0.79],
        quat=[0, 0, 1, 0],
        rgba="0 0 1 1",
        _attributes={
            "name": "cylinder-blue",
        },
    )
    mug = ObjaverseMujocoMug(assets="kitchen/mug", pos=[x1, y1, 0.85], collision_count=32, visual_count=2)
    mug_tree = MjSDF(
        pos=[x2, y2, 1],
        assets="mug_tree",
        mass=0.3,
        _attributes={
            "name": "tree",
            "quat": "0.5 0.5 0.5 0.5",
        },
        additional_children_raw="""\t<site name="tree_goal_1" pos="-.03 0.076 -0.036" size="0.007" rgba="1 1 1 1"/>
                                          \t<site name="tree_goal_2" pos="-.03 0.066 -0.016" size="0.007" rgba="1 1 1 1"/>
                                          \t<site name="tree_goal_3" pos="-.03 0.071 -0.026" size="0.007" rgba="1 1 1 1"/>
                                          \t<site name="tree_goal_4" pos="-.03 0.081 -0.046" size="0.007" rgba="1 1 1 1"/>
                                          \t<site name="tree_goal_5" pos="-.03 0.086 -0.056" size="0.007" rgba="1 1 1 1"/>
                                          \t<site name="tree_goal_6" pos="-.03 0.091 -0.066" size="0.007" rgba="1 1 1 1"/>
                                            """,
        scale="0.17 0.2118 0.17",
    )

    children = [
        *camera_rig.get_cameras(),
        *stereo_rig.get_cameras(),
        robohive_table,
        # robot_room,
        # box,
        ball_1,
        # ball_2,
        ibox,
        square_block,
        triangle_block,
        hex_block,
        circle_block,
        mug,
        # mug_tree,
        rope,
        rope_anchor,
    ]

    scene = UR5Robotiq2f85(
        *children,
        pos=[0, 0, 0.77],
        **options,
        camera_rig=camera_rig,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Tri Demo Ur5 Tabletop task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Tri Demo Ur5 Tabletop task loaded successfully!")
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
