# UR5e + Robotiq 2F-85

Industrial manipulation setup with UR5e arm and Robotiq gripper.

![UR5e with Robotiq gripper](figures/ur5e_robotiq.png)

## Components

- **Robot**: Universal Robots UR5e
- **End Effector**: Robotiq 2F-85 gripper
- **Environment**: Ground, lighting

## Usage

```python
from vuer_mjcf.stage_sets.ur5e_robotiq_scene import UR5Robotiq2f85

scene = UR5Robotiq2f85(assets="path/to/assets")
```

## Use Cases

- Industrial automation
- Pick and place
- Assembly tasks

## See Also

- [UR5e](../robots/ur5e.md)
- [Robotiq 2F-85](../robots/robotiq_2f85.md)
