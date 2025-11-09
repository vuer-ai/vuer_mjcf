# Concrete Slab

A concrete platform/work surface for environments.

## Usage Example

```python
from vuer_mjcf.objects.concrete_slab import ConcreteSlab

slab = ConcreteSlab(name="platform", pos=[0, 0, 0])
```

## Use Cases

- Work surfaces
- Environment platforms
- Structural elements

## Source Code

See the full implementation in `vuer_mjcf/objects/concrete_slab.py`:

```python
class ConcreteSlab(Body):
    _attributes = {
        "name": "concrete-slab",
        # "childclass": "concrete-slab",
    }
    _preamble = """
    <asset>
      <material name="{name}-concrete" rgba="0.2 0.2 0.2 1" shininess="0.5"/>
    </asset>
    """
    # use {childclass} when you want to use defaults. Just {name}- if no
    # defaults are involved.
    # todo: how do I make sure it has collision? what about friction?
    _children_raw = """
    <geom name="{name}-concrete" type="box" material="{name}-concrete" size="0.7 0.7 0.05" pos="0 0 -0.05"/>
    """
```
