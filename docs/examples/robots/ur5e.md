# UR5e

Universal Robots UR5e is a lightweight collaborative robot arm with 6 degrees of freedom.

![UR5e robot arm](figures/ur5e.png)

## Usage Example

```python
from vuer_mjcf.robots.ur5e import UR5e
from vuer_mjcf.stage_sets.ur5e_robotiq_scene import UR5Robotiq2f85

# Use pre-configured scene
scene = UR5Robotiq2f85()
```
