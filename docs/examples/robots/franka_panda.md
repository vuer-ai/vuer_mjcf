# Franka Panda

The Franka Emika Panda is a popular 7-DOF collaborative robot arm widely used in research and industry.

## Usage Example

```python
from vuer_mjcf.robots.franka_panda import Panda
from vuer_mjcf.robots.robotiq_2f85 import Robotiq2F85

# Create Panda arm
panda = Panda(
    name="panda",
    pos=[0, 0, 0],
    assets="franka_panda"
)

# Add gripper as end effector
gripper = Robotiq2F85(
    name="gripper",
    assets="robotiq_2f85"
)

# Or use pre-configured scene
from vuer_mjcf.stage_sets.panda_robotiq_scene import PandaRobotiq2f85
scene = PandaRobotiq2f85()
```

## Assets Required

Mesh and texture files for Panda should be in `assets/robots/franka_panda/`
