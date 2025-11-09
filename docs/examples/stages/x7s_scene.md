# X7s Scene

Complete scene with X7s 7-DOF robot arm variant for manipulation tasks.

![X7s robot arm](figures/x7s_scene.png)

## Components

- X7s robot arm (7 DOF)
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.x7s_scene import X7sScene

# Basic scene
scene = X7sScene(assets="path/to/assets")

# Add custom objects
from vuer_mjcf.objects.basket import Basket
scene = X7sScene(Basket(pos=[0.5, 0, 0]), assets="assets/robots")
```

## Use Cases

- Dexterous manipulation
- Motion planning research
- 7-DOF control
- Advanced manipulation tasks

## See Also

- [X7s Robot](../robots/x7s.md)
- [Stages Index](index.md)
