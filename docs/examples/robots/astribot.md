# Astribot

Astribot is a humanoid/mobile manipulator robot system.

![Astribot robot](figures/astribot.png)

## Usage

```python
from vuer_mjcf.robots.astribot import Astribot
from vuer_mjcf.stage_sets.astribot_robotiq import AstribotRobotiq2f85
from vuer_mjcf.stage_sets.astribot_shadow_hand import AstribotHand

# With Robotiq gripper
scene_gripper = AstribotRobotiq2f85()

# With Shadow Hand
scene_hand = AstribotHand()
```
