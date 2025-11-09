import numpy as np
from scipy.spatial.transform import Rotation as R
import vuer_mjcf.utils.se3.se3_mujoco as m


def compose(pos1, mujoco_quat1, pos2):
    """
    Compose pos1 + rotation(mujoco_quat1) applied to pos2.
    mujoco_quat1: [w, x, y, z]
    """
    # Convert [w, x, y, z] -> [x, y, z, w]
    quat1_scipy = np.array([mujoco_quat1[1], mujoco_quat1[2], mujoco_quat1[3], mujoco_quat1[0]])

    r1 = R.from_quat(quat1_scipy)
    pos2_world = r1.apply(pos2)
    global_pos = np.array(pos1) + pos2_world

    return m.Vector3(*global_pos.tolist())
