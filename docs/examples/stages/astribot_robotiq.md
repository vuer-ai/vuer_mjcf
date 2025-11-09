# Astribot + Robotiq 2F-85

Mobile humanoid manipulation setup with Astribot and Robotiq gripper.

![Astribot with Robotiq gripper](figures/astribot_robotiq.png)

## Components

- **Robot**: Astribot humanoid/mobile manipulator
- **End Effector**: Robotiq 2F-85 gripper
- **Environment**: Ground, lighting
- **Control**: Mocap control

## Usage

```python
from vuer_mjcf.stage_sets.astribot_robotiq import AstribotRobotiq2f85

scene = AstribotRobotiq2f85(assets="path/to/assets")
```

## Use Cases

- Mobile manipulation
- Humanoid research
- Complex environments
- Dual-arm tasks

## See Also

- [Astribot](../robots/astribot.md)
- [Robotiq 2F-85](../robots/robotiq_2f85.md)
- [Astribot + Shadow Hand](astribot_shadow_hand.md)
