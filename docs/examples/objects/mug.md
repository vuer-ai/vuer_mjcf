# Mug

A cup object from the Objaverse dataset, ideal for manipulation and pouring tasks.

## Description

The `ObjaverseMujocoMug` class provides a realistic mug model with visual and collision geometry suitable for grasping and manipulation research.

## Parameters

- `name` (str): Identifier for the mug
- `pos` (list): Position [x, y, z]
- `quat` (list): Orientation [w, x, y, z]

## Usage Example

```python
from vuer_mjcf.schema import Mjcf
from vuer_mjcf.objects.mug import ObjaverseMujocoMug
from vuer_mjcf.utils.file import Prettify

# Create a mug
mug = ObjaverseMujocoMug(
    name="coffee_mug",
    pos=[0, 0, 0.5]
)

# Add to scene
scene = Mjcf(mug, model="MugDemo")
scene.save("mug_scene.xml")
```

## Features

- High-quality 3D mesh from Objaverse
- Realistic collision geometry
- Free 6-DOF motion
- Suitable for grasping tasks

## Use Cases

- Pick and place demonstrations
- Pouring simulations
- Grasping research
- Table setting tasks

## See Also

- [Plate](plate.md) - Companion tableware
- [Bowl](index.md#tableware--containers) - Similar container
- [Objects Index](index.md)
