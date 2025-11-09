import os

from pathlib import Path

import numpy as np

from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.stage_sets.astribot_robotiq import AstribotRobotiq2f85
from vuer_mjcf.stage_sets.ur5e_robotiq_scene import UR5Robotiq2f85
from vuer_mjcf.basic_components.rigs.camera_rig_calibrated import make_camera_rig
from itertools import combinations
from math import hypot
import random

from vuer_mjcf.third_party.robohive.robohive_object import RobohiveObj
import mink

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

x_center, y_center = 0.5, -0.15

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

    box = MjSDF(
        pos=[0.465, 0.07, 0.82],
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

    children = [
        *camera_rig.get_cameras(),
        robohive_table,
        # robot_room,
        box,
        ball_1,
        ball_2,
    ]

    # scene = UR5Robotiq2f85(
    #     *children,
    #     pos=[0, 0, 0.77],
    #     **options,
    #     camera_rig=camera_rig,
    # )
    
    base_pos = np.array([0.05, -0.15, 0.15])
    head_target = _compute_look_at_rotation(
        head_pos=base_pos + np.array([0.0, 0.0, 1.5]),
        target_pos=np.array([0.465, 0.07, 0.82])
    )

    head_quat_wxyz = head_target.wxyz_xyz[:4]  # wxyz
    
    scene = AstribotRobotiq2f85(
        *children,
        pos=base_pos.tolist(),
        head_quat=head_quat_wxyz.tolist(),
        mocap_pos="0.3205938935279846 0.01334109716117382 1.215453326702118",
        mocap_quat=[0.259, 0, 0.966, 0],
        **options,
        camera_rig=camera_rig,
    )

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Ball Sorting Toy New Astribot task")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Ball Sorting Toy New Astribot task loaded successfully!")
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
