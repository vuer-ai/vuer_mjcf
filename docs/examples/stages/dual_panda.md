# Dual Panda

Bimanual manipulation setup with two Franka Panda arms.

## Components

- **Robots**: Two Franka Panda arms
- **End Effectors**: Grippers or hands
- **Environment**: Ground, lighting
- **Control**: Dual mocap control

## Usage

```python
from vuer_mjcf.stage_sets.dual_panda import DualPanda

scene = DualPanda(assets="path/to/assets")
```

## Use Cases

- Bimanual manipulation
- Coordinated dual-arm tasks
- Handover research
- Complex assembly

## See Also

- [Franka Panda](../robots/franka_panda.md)
