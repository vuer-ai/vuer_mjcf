# R5a Scene

Complete scene with R5a robot arm ready for manipulation tasks.

## Components

- R5a robot arm
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.r5a_scene import R5aScene

# Basic scene
scene = R5aScene(assets="path/to/assets")

# Add custom objects
from vuer_mjcf.objects.ball import Ball
scene = R5aScene(Ball(pos=[0.5, 0, 0.5]), assets="assets/robots")
```

## Use Cases

- Manipulation research
- Robot arm control
- Pick and place tasks
- Motion planning

## See Also

- [R5a Robot](../robots/r5a.md)
- [Stages Index](index.md)
