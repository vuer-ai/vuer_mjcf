# X7 Scene

Complete scene with X7 7-DOF robot arm for dexterous manipulation.

## Components

- X7 robot arm (7 DOF)
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.x7_scene import X7Scene

# Basic scene
scene = X7Scene(assets="path/to/assets")

# Add custom objects
from vuer_mjcf.objects.cabinet import Cabinet
scene = X7Scene(Cabinet(pos=[0.7, 0, 0]), assets="assets/robots")
```

## Use Cases

- Dexterous manipulation
- Complex motion planning
- Research applications
- 7-DOF control studies

## See Also

- [X7 Robot](../robots/x7.md)
- [Stages Index](index.md)
