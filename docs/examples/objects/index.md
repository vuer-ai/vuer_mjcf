# Objects

Vuer-MJCF provides a comprehensive library of objects for robotics simulation. Objects range from simple geometric primitives to complex articulated structures and deformable bodies.

## Featured Objects

Explore these key objects in detail:

- [Ball](ball.md) - Simple sphere with free joint
- [Mug](mug.md) - Cup for manipulation tasks
- [Cabinet](cabinet.md) - Articulated kitchen cabinet with doors
- [Rope](rope.md) - Deformable cable using composite
- [Basket](basket.md) - Container for object collection
- [Bin](bin.md) - Rectangular container with walls
- [Plate](plate.md) - Tableware for manipulation
- [Concrete Slab](concrete_slab.md) - Platform/work surface
- [MjObj](mj_obj.md) - Generic OBJ file loader
- [Poncho](poncho.md) - Deformable cloth object

## All Available Objects

### Basic Geometric Objects
- **Ball** (`ball.py`) - Sphere with customizable size and color
- **Cylinder** (`cylinder.py`) - Basic cylindrical object
- **Bin** (`bin.py`) - Rectangular container with configurable dimensions

### Kitchen Appliances
- **Cabinet** (`cabinet.py`) - Two-door base cabinet with shelves and hinges
- **Single Cabinet** (`single_cabinet.py`) - Single-door cabinet variant
- **Drawer** (`drawer.py`) - Functional kitchen drawer
- **Drawer Stack** (`drawer_stack.py`) - Multiple stacked drawers
- **Drawer Visual** (`drawer_visual.py`) - Visual-only drawer
- **MimicGen Drawer** (`mimicgen_drawer.py`) - Drawer from MimicGen benchmark
- **Dishwasher** (`dishwasher.py`) - Kitchen dishwasher
- **Microwave** (`microwave.py`) - Kitchen microwave
- **Microwave Scaled** (`microwave_scaled.py`) - Scaled microwave variant
- **Oven** (`oven.py`) - Kitchen oven
- **Refrigerator** (`refrigerator.py`) - Kitchen refrigerator
- **Sink** (`sink.py`) - Kitchen sink
- **Sink Wide** (`sink_wide.py`) - Wider kitchen sink

### Tableware & Containers
- **Basket** (`basket.py`) - Container basket
- **Mug** (`mug.py`) - Objaverse mug model
- **MuJoCo Mug** (`mujoco_mug.py`) - Simple mug
- **Vuer Mug** (`vuer_mug.py`) - Custom Vuer mug
- **Plate** (`plate.py`) - Objaverse plate
- **Bigym Plate** (`bigym_plate.py`) - BiGym benchmark plate
- **Bowl** (`objaverse_mujoco_bowl.py`) - Objaverse bowl
- **Cup** (`objaverse_mujoco_cup.py`) - Objaverse cup
- **Dishdrainer** (`bigym_dishdrainer.py`) - BiGym dish drainer
- **Spoon** (`spoon_7.py`) - Objaverse spoon model

### Food Items
- **Bagel** (`bagel.py`) - 3D bagel model
- **Cupcake** (`cupcake.py`) - 3D cupcake model

### Furniture & Structural
- **Table** (`bigym_table.py`) - BiGym benchmark table
- **Orbit Table** (`orbit_table.py`) - Optical/laboratory table
- **Concrete Slab** (`concrete_slab.py`) - Concrete platform
- **Room Wall** (`room_wall.py`) - Room wall structure

### Shape Sorting Objects
- **HexBlock** (`sort_shapes.py`) - Hexagonal block
- **TriangleBlock** (`sort_shapes.py`) - Triangle block
- **SquareBlock** (`sort_shapes.py`) - Square block
- **CircleBlock** (`sort_shapes.py`) - Circle block
- **LeBox** (`sort_shapes.py`) - Box for shape sorting
- **TShape** (`tshape.py`) - T-shaped object

### Deformable Objects
- **Rope** (`rope.py`) - Flexible cable using MuJoCo composite
- **Poncho** (`poncho.py`) - Soft deformable cloth

### Generic Object Loaders
- **MjObj** (`mj_obj.py`) - Load OBJ files with optional textures
- **MjObjTextureless** (`mj_obj.py`) - Load OBJ files without textures
- **Decomposed Obj** (`decomposed_obj.py`) - Load Objaverse assets with multiple meshes
- **MjSDF** (`mj_sdf.py`) - Signed distance field objects

## Usage Example

Here's a basic example of using objects:

```python
from vuer_mjcf.schema import Mjcf
from vuer_mjcf.objects.ball import Ball
from vuer_mjcf.objects.bin import Bin
from vuer_mjcf.utils.file import Prettify

# Create objects
ball = Ball(name="red_ball", pos=[0, 0, 1], rgba="1 0 0 1")
container = Bin(name="container", pos=[0.5, 0, 0.5])

# Create scene
scene = Mjcf(ball, container, model="ObjectsDemo")
xml = scene._xml | Prettify()
```

## See Also

- [Robots](../robots/index.md) - Available robot models
- [Stages](../stages/index.md) - Pre-configured scenes
