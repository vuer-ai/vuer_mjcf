# XHand Scene

Complete scene with XHand dexterous hands for advanced manipulation.

## Components

- XHand (left and/or right)
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.xhand_scene import XHandScene

# Basic scene
scene = XHandScene(assets="path/to/assets")

# Add objects for manipulation
from vuer_mjcf.objects.ball import Ball
scene = XHandScene(Ball(pos=[0, 0, 0.5]), assets="assets/robots")
```

## Use Cases

- Dexterous manipulation research
- Bimanual coordination
- In-hand manipulation
- Advanced grasping studies

## See Also

- [XHand Robot](../robots/xhand.md)
- [Stages Index](index.md)
