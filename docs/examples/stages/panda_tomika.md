# Panda + Tomika Scene

Franka Panda robot arm equipped with Tomika parallel-jaw gripper.

## Components

- Franka Panda arm
- Tomika gripper
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.panda_tomika_scene import PandaTomika

# Single arm
scene = PandaTomika(assets="path/to/assets")

# Dual arms
scene_dual = PandaTomika(dual_robot=True, assets="assets/robots")

# Add objects
from vuer_mjcf.objects.mug import Mug
scene = PandaTomika(Mug(pos=[0.5, 0, 0.5]))
```

## Use Cases

- Pick and place operations
- Grasping research
- Bimanual manipulation (dual mode)
- Manipulation experiments

## See Also

- [Franka Panda](../robots/franka_panda.md)
- [Tomika Gripper](../robots/tomika_gripper.md)
- [Stages Index](index.md)
