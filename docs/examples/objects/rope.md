# Rope

A deformable cable using MuJoCo's composite/cable system.

## Description

The `MuJoCoRope` class creates a flexible rope using MuJoCo's composite elements. The rope is made of connected segments that can bend and interact with other objects.

## Parameters

- `name` (str): Identifier for the rope
- `pos` (list): Starting position [x, y, z]
- Composite parameters (count, spacing, etc.)

## Usage Example

```python
from vuer_mjcf.schema import Mjcf
from vuer_mjcf.objects.rope import MuJoCoRope

# Create a flexible rope
rope = MuJoCoRope(
    name="cable",
    pos=[0, 0, 1]
)

scene = Mjcf(rope, model="RopeDemo")
scene.save("rope_scene.xml")
```

## Features

- Deformable composite structure
- Self-collision
- Cable-like dynamics
- Can be grasped and manipulated

## Use Cases

- Cable manipulation
- Rope physics research
- Deformable object handling
- Knot tying simulations

## Source Code

See the full implementation in `vuer_mjcf/objects/rope.py`:

```python
class MuJoCoRope(Composite):
    rgba = "0.7 0 0 1"
    mass = "0.001"
    geom_size = ".004"
    damping = ".015"
    condim = "1"
    twist = "1e7"
    bend = "3e5"
    vmax = "0.005"

    _attributes = {
        "prefix": "rope_",
        "type": "cable",
        "curve": "s",
        "count": "41 1 1",
        "size": 1,
        "initial": "free",
        "offset": "0 0 0.7",
    }

    _preamble = """
        <extension>
            <plugin plugin="mujoco.elasticity.cable"/>
        </extension>
    """

    _children_raw = """
    <plugin plugin="mujoco.elasticity.cable">
        <!--Units are in Pa (SI)-->
        <config key="twist" value="{twist}"/>
        <config key="bend" value="{bend}"/>
        <config key="vmax" value="{vmax}"/>
    </plugin>
    <joint kind="main" damping="{damping}"/>
    <geom type="capsule" size="{geom_size}" rgba="{rgba}" condim="{condim}" mass="{mass}"/>
    """
```

## See Also

- [Poncho](poncho.md) - Another deformable object
- [Objects Index](index.md)
