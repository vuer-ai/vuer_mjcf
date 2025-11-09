# Widow X Scene

Complete scene with single Widow X robot arm for manipulation tasks.

## Components

- Widow X robot arm
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.widow_x_scene import WidowXScene

# Basic scene
scene = WidowXScene(assets="path/to/assets")

# Add custom objects
from vuer_mjcf.objects.basket import Basket
scene = WidowXScene(Basket(pos=[0.5, 0, 0]), assets="assets/robots")
```

## Use Cases

- Compact manipulation
- Pick and place tasks
- Research applications
- Budget-friendly robotics

## See Also

- [Widow X Robot](../robots/widow_x.md)
- [Stages Index](index.md)
