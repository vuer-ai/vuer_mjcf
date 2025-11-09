# Ufactory xArm7 Scene

Complete scene with Ufactory xArm7 robot arm for manipulation tasks.

![Ufactory xArm7 robot](figures/ufactory_xarm7_scene.png)

## Components

- Ufactory xArm7 robot arm
- Ground plane
- Camera rig
- Lighting rig

## Usage

```python
from vuer_mjcf.stage_sets.ufactory_xarm7_scene import UfactoryXArm7Scene

# Basic scene
scene = UfactoryXArm7Scene(assets="path/to/assets")

# Add custom objects
from vuer_mjcf.objects.mug import Mug
scene = UfactoryXArm7Scene(Mug(pos=[0.5, 0, 0.5]), assets="assets/robots")
```

## Use Cases

- Industrial manipulation
- Pick and place operations
- Assembly tasks
- Robot control research

## See Also

- [Ufactory xArm7 Robot](../robots/ufactory_xarm7.md)
- [Ufactory Gripper](../robots/ufactory_gripper.md)
- [Stages Index](index.md)
