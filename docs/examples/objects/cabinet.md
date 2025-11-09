# Cabinet

An articulated two-door kitchen cabinet with shelves and working hinges.

## Description

The `KitchenCabinet` class creates a realistic base cabinet with:
- Two hinged doors
- Interior shelves
- Decorative trim and handles
- Realistic door physics

## Parameters

- `name` (str): Identifier for the cabinet
- `pos` (list): Position [x, y, z]
- `quat` (list): Orientation [w, x, y, z]

## Usage Example

```python
from vuer_mjcf.schema import Mjcf
from vuer_mjcf.objects.cabinet import KitchenCabinet
from vuer_mjcf.utils.file import Prettify

# Create a cabinet
cabinet = KitchenCabinet(
    name="base_cabinet",
    pos=[0, 0, 0]
)

# Add to scene
scene = Mjcf(cabinet, model="KitchenDemo")
scene.save("cabinet_scene.xml")
```

## Features

- Articulated doors with hinge joints
- Interior storage space
- Realistic cabinet geometry
- Suitable for manipulation research

## Use Cases

- Kitchen task simulations
- Door opening research
- Object placement in cabinets
- Long-horizon manipulation

## See Also

- [Drawer](index.md#kitchen-appliances) - Drawer variant
- [Microwave](index.md#kitchen-appliances) - Another appliance
- [Objects Index](index.md)
