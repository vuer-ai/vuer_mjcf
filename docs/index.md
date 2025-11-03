# Welcome to Vuer-MJCF

**Vuer-MJCF** is a Python schema system for building MuJoCo MJCF (Model XML) files programmatically. It provides a clean, composable API for creating robot environments, scenes, and simulations for use with MuJoCo and Lucid-XR.

## Installation

```shell
pip install vuer-mjcf
```

## Quick Start

Here's a simple example that creates a MuJoCo scene with a ball:

```python
from vuer_mjcf.schema import Mjcf, Body, Geom, Raw
from vuer_mjcf.utils.file import Prettify

# Create a simple scene
ground = Raw('<geom name="ground" type="plane" size="5 5 .05" material="grid"/>')

ball = Body(
    name="ball",
    pos="0 0 1",
    _children_raw='<freejoint/><geom type="sphere" size="0.1" rgba="1 0 0 1"/>'
)

scene = Mjcf(ground, ball, model="SimpleBall")

# Generate XML
xml_string = scene._xml | Prettify()
print(xml_string)
```

## Key Features

- **Composable Schema**: Build complex scenes from reusable components
- **Inheritance Support**: Create component hierarchies with attribute inheritance
- **Type-Safe**: Python classes for MJCF elements
- **Example Library**: Pre-built components for robots, objects, and scenes
- **MuJoCo Integration**: Direct compatibility with MuJoCo physics engine

## What's Included

### Schema System
The core schema system provides Python classes for all MJCF elements:
- `Mjcf` - Root schema class
- `Body` - Body elements with automatic formatting
- `Geom`, `Joint`, `Site` - Standard MJCF components
- `Raw` - Raw XML insertion for flexibility

### Example Scenes
Over 20 example MuJoCo scenes demonstrating various features:
- **Flex Bodies**: Soft bodies, cloth, plates, baskets
- **Robots**: Shadow Hand, Cassie, humanoids
- **Objects**: Blocks, balls, cards, Rubik's cube
- **Scenes**: Complete environments with lighting, cameras, and physics

### Components Library
Pre-built components for common use cases:
- Basic shapes and primitives
- Robot models (arms, hands, mobile robots)
- Manipulation objects
- Stage sets and environments

## Documentation Structure

```{eval-rst}
.. toctree::
   :hidden:
   :maxdepth: 1
   :titlesonly:

   Quick Start <quick_start>
   Report Issues <https://github.com/vuer-ai/vuer-mjcf/issues>

.. toctree::
   :maxdepth: 2
   :caption: API Reference
   :hidden:

   vuer_mjcf.schema.base — Base XML Classes <api/base.md>
   vuer_mjcf.schema.schema — MuJoCo Schema <api/schema.md>

.. toctree::
   :maxdepth: 2
   :caption: Examples
   :hidden:

   Objects <examples/objects/index.md>
   Robots <examples/robots/index.md>
   Stages <examples/stages/index.md>

```

## Getting Help

- **GitHub Issues**: [Report bugs or request features](https://github.com/vuer-ai/vuer-mjcf/issues)
- **Documentation**: This site contains comprehensive API documentation and examples
- **Examples**: Check the `vuer_mjcf/mujoco_examples/` directory for more examples

## License

Vuer-MJCF is open source software licensed under the MIT License.