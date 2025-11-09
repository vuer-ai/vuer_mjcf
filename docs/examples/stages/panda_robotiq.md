# Panda + Robotiq 2F-85

Complete manipulation setup with Franka Panda arm and Robotiq parallel gripper.

## Components

- **Robot**: Franka Panda 7-DOF arm
- **End Effector**: Robotiq 2F-85 parallel gripper
- **Environment**: Ground plane, lighting
- **Control**: Mocap bodies for motion control

## Usage

```python
from vuer_mjcf.stage_sets.panda_robotiq_scene import PandaRobotiq2f85

# Create scene
scene = PandaRobotiq2f85(assets="path/to/assets")

# With dual arms
scene_dual = PandaRobotiq2f85(dual_robot=True)

# Add objects
from vuer_mjcf.objects.ball import Ball
ball = Ball(name="ball", pos=[0.3, 0, 0.5])
scene = PandaRobotiq2f85(ball)
```

## Features

- Ready-to-use manipulation setup
- Dual robot option for bimanual tasks
- Integrated camera rig
- Mocap control

## Use Cases

- Pick and place
- Assembly
- Grasping research
- Bimanual manipulation (dual mode)

## See Also

- [Franka Panda](../robots/franka_panda.md)
- [Robotiq 2F-85](../robots/robotiq_2f85.md)
