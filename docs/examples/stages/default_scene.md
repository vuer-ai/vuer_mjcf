# Default Scene

The basic stage setup with floor, lighting, and skybox - perfect for quick prototyping.

![Default scene with lighting and ground plane](figures/default_scene.png)

## Description

`DefaultStage` provides a minimal but complete MuJoCo environment with:
- Ground plane with grid material
- Three-point lighting (key, fill, back)
- Skybox with gradient
- Ready to add any objects or robots

## Usage Example

```python
from vuer_mjcf.stage_sets.default_scene import DefaultStage
from vuer_mjcf.objects.ball import Ball

# Add objects to default scene
ball = Ball(name="ball", pos=[0, 0, 1])

scene = DefaultStage(ball, model="MyScene")
scene.save("my_scene.xml")
```

## What's Included

- **Ground**: Textured plane with checker pattern
- **Lighting**: Professional three-point setup
- **Skybox**: Gradient background
- **Camera**: Default viewpoint

## Customization

```python
scene = DefaultStage(
    my_objects,
    pos=[0, 0, 0],  # Base position
    assets="path/to/assets"
)
```

## Use Cases

- Quick prototyping
- Testing objects
- Simple simulations
- Base for custom scenes

## See Also

- [Objects](../objects/index.md)
- [Stages Index](index.md)
