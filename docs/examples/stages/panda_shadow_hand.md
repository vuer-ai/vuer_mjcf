# Panda + Shadow Hand

Dexterous manipulation setup with Franka Panda arm and Shadow Hand.

## Components

- **Robot**: Franka Panda arm
- **End Effector**: Shadow Hand (5-finger dexterous)
- **Environment**: Ground, lighting
- **Control**: Mocap for arm and finger control

## Usage

```python
from vuer_mjcf.stage_sets.panda_shadow_hand_scene import PandaShadowHand

scene = PandaShadowHand(assets="path/to/assets")
```

## Use Cases

- Dexterous manipulation
- In-hand object manipulation
- Complex grasping
- Finger gaiting research

## See Also

- [Franka Panda](../robots/franka_panda.md)
- [Shadow Hand](../robots/shadow_hand.md)
