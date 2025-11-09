# Lift Scene

Complete scene with Lift robot arm for manipulation tasks.

## Components

- Lift robot arm
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.lift_scene import LiftScene

# Basic scene
scene = LiftScene(assets="path/to/assets")

# Add custom objects
from vuer_mjcf.objects.ball import Ball
scene = LiftScene(Ball(pos=[0.5, 0, 0.5]), assets="assets/robots")
```

## Use Cases

- Manipulation research
- Pick and place tasks
- Robot control
- Motion planning

## See Also

- [Lift Robot](../robots/lift.md)
- [Stages Index](index.md)
