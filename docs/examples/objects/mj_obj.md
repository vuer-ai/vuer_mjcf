# MjObj - Generic OBJ Loader

Generic utility for loading OBJ files with optional textures.

## Description

The `MjObj` and `MjObjTextureless` classes provide a flexible way to load any OBJ file into your MuJoCo scene.

## Usage Example

```python
from vuer_mjcf.objects.mj_obj import MjObj

# Load an OBJ file with texture
my_object = MjObj(
    name="custom_object",
    file="path/to/model.obj",
    texture="path/to/texture.png",
    pos=[0, 0, 0.5]
)

# Or without texture
from vuer_mjcf.objects.mj_obj import MjObjTextureless

simple_object = MjObjTextureless(
    name="simple",
    file="path/to/model.obj"
)
```

## Features

- Load any OBJ file
- Optional texture support
- Automatic mesh scaling
- Free 6-DOF motion

## Use Cases

- Custom object import
- Rapid prototyping
- Using existing 3D models
- Asset pipeline integration
