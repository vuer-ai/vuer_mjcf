# Astribot Scene

Complete scene with Astribot humanoid robot for full-body manipulation.

## Components

- Astribot humanoid robot
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.astribot_scene import AstribotScene

# Basic scene
scene = AstribotScene(assets="path/to/assets")

# Add environment objects
from vuer_mjcf.objects.cabinet import Cabinet
scene = AstribotScene(Cabinet(pos=[0.7, 0, 0]), assets="assets/robots")
```

## Use Cases

- Humanoid robot research
- Full-body manipulation
- Mobile manipulation
- Bimanual coordination

## See Also

- [Astribot Robot](../robots/astribot.md)
- [Astribot + Robotiq Scene](astribot_robotiq.md)
- [Astribot + Shadow Hand Scene](astribot_shadow_hand.md)
- [Stages Index](index.md)
