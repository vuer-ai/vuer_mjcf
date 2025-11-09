# Robotiq 2F-85

Popular parallel-jaw gripper with 85mm stroke, widely used in research and industry.

![Robotiq 2F-85 gripper](figures/robotiq_2f85.png)

## Usage

```python
from vuer_mjcf.robots.robotiq_2f85 import Robotiq2F85

gripper = Robotiq2F85(
    name="gripper",
    assets="robotiq_2f85",
    pos=[0, 0, 0]
)
```
