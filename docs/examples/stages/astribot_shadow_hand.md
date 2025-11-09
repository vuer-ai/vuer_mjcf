# Astribot + Shadow Hand Scene

Astribot robot equipped with Shadow Hand for dexterous manipulation tasks.

![Astribot with Shadow Hand](figures/astribot_shadow_hand.png)

## Components

- Astribot robot
- Shadow Hand (left variant)
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.astribot_shadow_hand import AstribotShadowHand

# Basic scene
scene = AstribotShadowHand(assets="path/to/assets")

# Add objects
from vuer_mjcf.objects.mug import Mug
scene = AstribotShadowHand(Mug(pos=[0.5, 0, 0.5]), assets="assets/robots")
```

## Use Cases

- Dexterous manipulation
- In-hand object manipulation
- Humanoid manipulation research
- Complex grasping tasks

## See Also

- [Astribot Robot](../robots/astribot.md)
- [Shadow Hand](../robots/shadow_hand.md)
- [Astribot + Robotiq Scene](astribot_robotiq.md)
- [Stages Index](index.md)
