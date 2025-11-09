# X5a Scene

Complete scene with X5a robot arm ready for manipulation tasks.

## Components

- X5a robot arm
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.x5a_scene import X5aScene

# Basic scene
scene = X5aScene(assets="path/to/assets")

# Add custom objects
from vuer_mjcf.objects.mug import Mug
scene = X5aScene(Mug(pos=[0.5, 0, 0.5]), assets="assets/robots")
```

## Use Cases

- Manipulation research
- Robot arm control
- Pick and place tasks
- Motion planning

## See Also

- [X5a Robot](../robots/x5a.md)
- [Stages Index](index.md)
