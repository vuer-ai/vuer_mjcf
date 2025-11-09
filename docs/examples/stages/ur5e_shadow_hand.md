# UR5e + Shadow Hand Scene

UR5e robot arm equipped with Shadow Hand for dexterous manipulation.

![UR5e with Shadow Hand](figures/ur5e_shadow_hand.png)

## Components

- UR5e robot arm
- Shadow Hand (left variant)
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.ur5e_shadow_hand_scene import UR5eShadowHand

# Single arm
scene = UR5eShadowHand(assets="path/to/assets")

# Dual arms
scene_dual = UR5eShadowHand(dual_robot=True, assets="assets/robots")

# Add objects for manipulation
from vuer_mjcf.objects.ball import Ball
scene = UR5eShadowHand(Ball(pos=[0.5, 0, 0.5]))
```

## Use Cases

- Dexterous manipulation
- In-hand manipulation research
- Complex grasping tasks
- Bimanual dexterous manipulation (dual mode)

## See Also

- [UR5e Robot](../robots/ur5e.md)
- [Shadow Hand](../robots/shadow_hand.md)
- [Stages Index](index.md)
