# Trossen Table Scene

Dual-arm manipulation setup with Trossen robot arms on a shared table.

## Components

- Two Trossen robot arms
- Shared workspace table
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.trossen_table import TrossenTable

# Dual arm scene
scene = TrossenTable(assets="path/to/assets")

# Add objects for bimanual tasks
from vuer_mjcf.objects.mug import Mug
from vuer_mjcf.objects.plate import Plate
scene = TrossenTable(
    Mug(pos=[0.3, 0, 0.7]),
    Plate(pos=[-0.3, 0, 0.7]),
    assets="assets/robots"
)
```

## Use Cases

- Bimanual manipulation
- Dual-arm coordination
- Collaborative robot tasks
- Shared workspace research

## See Also

- [Dual Panda Scene](dual_panda.md)
- [Stages Index](index.md)
