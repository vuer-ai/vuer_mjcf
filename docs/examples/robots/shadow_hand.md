# Shadow Hand

High-fidelity 5-finger dexterous robotic hand available in left and right versions.

![Shadow Hand](figures/shadow_hand.png)

## Usage

```python
from vuer_mjcf.robots.shadow_hand import ShadowHandRight, ShadowHandLeft

# Right hand
right_hand = ShadowHandRight(
    name="shadow_right",
    assets="shadow_hand",
    pos=[0, 0, 0]
)

# Left hand
left_hand = ShadowHandLeft(
    name="shadow_left",
    assets="shadow_hand",
    pos=[0, 0, 0]
)

# With arm
from vuer_mjcf.stage_sets.panda_shadow_hand_scene import PandaShadowHand
scene = PandaShadowHand()
```

## Features

- High-fidelity tendon-driven actuation
- Realistic finger kinematics
- Suitable for dexterous manipulation research
