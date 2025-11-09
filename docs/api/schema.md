# `vuer_mjcf.schema.schema`

This module provides MuJoCo-specific schema classes for building MJCF (MuJoCo XML Format) files. All classes inherit from `XmlTemplate` and provide specialized functionality for robotics and physics simulation.

## Overview

The module contains classes for:

- **Mjcf** - Root MuJoCo element
- **Body** - Robot bodies/links
- **FreeBody** - Bodies with free joints
- **Composite** - Composite structures
- **Replicate** - Element replication
- **MocapBody** - Motion capture bodies

## Core Classes

### `Mjcf`

The root element of a MuJoCo XML file. This is the main entry point for creating scenes.

**Constructor:**
```python
Mjcf(*_children, pos=None, quat=None, **kwargs)
```

**Parameters:**
- `*_children`: Child elements to add to worldbody
- `pos` (list/tuple, optional): Position [x, y, z] (stored but not in attributes)
- `quat` (list/tuple, optional): Quaternion [w, x, y, z] (stored but not in attributes)
- `**kwargs`: Additional attributes (e.g., `model="MyModel"`)

**Template Structure:**
```xml
<mujoco {attributes}>
    {preamble}
    <worldbody>
        {children}
    </worldbody>
    {postamble}
</mujoco>
```

**Methods:**
- `save(fname: str)` - Save the XML to a file with pretty formatting

**Example:**
```python
from vuer_mjcf.schema import Mjcf, Body, Raw
from vuer_mjcf.utils.file import Prettify

# Create ground plane
ground = Raw('<geom name="ground" type="plane" size="5 5 .05"/>')

# Create a simple ball
ball = Body(
    name="ball",
    pos=[0, 0, 1],
    _children_raw='<freejoint/><geom type="sphere" size="0.1" rgba="1 0 0 1"/>'
)

# Create scene
scene = Mjcf(ground, ball, model="SimpleBall")

# Generate XML
xml_str = scene._xml | Prettify()

# Or save to file
scene.save("my_scene.xml")
```

---

### `MjNode`

Base class for MuJoCo nodes. Provides common functionality for all MuJoCo elements.

**Constructor:**
```python
MjNode(*_children, attributes=None, preamble=None, postamble=None, children=None, **kwargs)
```

**Parameters:**
- `*_children`: Child elements
- `attributes` (dict, optional): Dictionary of XML attributes
- `preamble` (str, optional): Preamble XML
- `postamble` (str, optional): Postamble XML
- `children` (Xml | List[Xml], optional): Alternative way to specify children
- `**kwargs`: Additional attributes and parameters

**Features:**
- Automatically sets kwargs as both instance attributes AND in `_attributes` dict
- Provides `_pos` (Vector3) and `_quat` (WXYZ) for pose tracking
- Merges preambles/postambles using `mujoco_hash` for deduplication

**Template:**
```xml
<{tag} {attributes}>{children}</{tag}>
```

---

### `Body`

Represents a body (link) in the MuJoCo scene. Bodies can have geoms, joints, and other elements.

**Constructor:**
```python
Body(*_children, pos=None, quat=None, remove_joints=False, **kwargs)
```

**Parameters:**
- `*_children`: Child elements (geoms, joints, etc.)
- `pos` (list/tuple, optional): Position [x, y, z]
- `quat` (list/tuple, optional): Quaternion [w, x, y, z]
- `remove_joints` (bool): If True, comments out all joint elements
- `**kwargs`: Body attributes (name, etc.)

**Example:**
```python
from vuer_mjcf.schema import Body, Raw

# Simple body with geom
link = Body(
    name="link1",
    pos=[0, 0, 0.5],
    quat=[1, 0, 0, 0],
    _children_raw='''
        <joint name="joint1" type="hinge" axis="0 0 1"/>
        <geom type="box" size="0.1 0.1 0.2" rgba="0 1 0 1"/>
    '''
)

# Using composition
geom = Raw('<geom type="sphere" size="0.05"/>')
joint = Raw('<joint type="hinge" axis="1 0 0"/>')
link2 = Body(geom, joint, name="link2", pos=[0, 0, 1])
```

**Special Features:**
- `remove_joints=True` - Comments out all `<joint>` elements in template and children
- Position and quaternion automatically become attributes
- Supports `_children_raw` for template-based children

---

### `FreeBody`

A body with a free joint, allowing 6-DOF motion.

**Constructor:**
```python
FreeBody(*_children, pos=None, quat=None, **kwargs)
```

**Parameters:**
- Same as Body, but automatically adds `<freejoint/>` to children

**Example:**
```python
from vuer_mjcf.schema import FreeBody

# Falling ball with free motion
ball = FreeBody(
    name="ball",
    pos=[0, 0, 2],
    _children_raw='<geom type="sphere" size="0.1" rgba="1 0 0 1"/>'
)
```

---

### `Composite`

Represents a composite structure in MuJoCo. Useful for creating complex deformable objects.

**Constructor:**
```python
Composite(*_children, pos=None, quat=None, **kwargs)
```

**Parameters:**
- `*_children`: Child elements
- `pos` (list/tuple, optional): Sets `offset` attribute (not `pos`)
- `**kwargs`: Composite attributes (type, count, spacing, etc.)

**Note:** Position becomes `offset` attribute for composites.

**Example:**
```python
from vuer_mjcf.schema import Composite

# Soft body grid
soft = Composite(
    type="grid",
    count="10 10 10",
    spacing="0.1 0.1 0.1",
    pos=[0, 0, 1],  # Becomes offset="0 0 1"
    _children_raw='''
        <geom type="sphere" size="0.01" rgba="0 0 1 1"/>
    '''
)
```

---

### `Replicate`

Replicates a body or structure multiple times with offsets.

**Constructor:**
```python
Replicate(*_children, **kwargs)
```

**Parameters:**
- `*_children`: Elements to replicate
- `**kwargs`: Replicate attributes (count, offset, etc.)

**Example:**
```python
from vuer_mjcf.schema import Replicate, Body

# Create a grid of boxes
box = Body(
    name="box",
    pos=[0, 0, 0],
    _children_raw='<geom type="box" size="0.1 0.1 0.1"/>'
)

grid = Replicate(
    Replicate(
        box,
        count="5",
        offset="0.25 0 0"
    ),
    count="5",
    offset="0 0.25 0"
)
```

---

### `MocapBody`

Body with motion capture support. Adds mocap bodies and equality constraints.

**Class Attributes:**
- `_mocaps_body` (str): Template for mocap body elements
- `_mocaps_equality` (str): Template for equality constraints

**Methods:**
- `_add_mocaps()` - Adds mocap elements to template and postamble

**Example:**
```python
from vuer_mjcf.schema import MocapBody

class GripperWithMocap(MocapBody):
    _mocaps_body = '''
        <body mocap="true" name="{name}_mocap" pos="{mocap_pos}">
            <geom type="box" size="0.01 0.01 0.01" rgba="1 0 0 0.5" contype="0" conaffinity="0"/>
        </body>
    '''

    _mocaps_equality = '''
        <equality>
            <weld body1="{name}" body2="{name}_mocap" solref="0.02 1"/>
        </equality>
    '''

gripper = GripperWithMocap(name="gripper", mocap_pos="0 0 1")
gripper._add_mocaps()
```

## Composition Pattern

vuer-mjcf uses a compositional approach where complex scenes are built from simple components:

```python
from vuer_mjcf.schema import Mjcf, Body, Raw
from vuer_mjcf.utils.file import Prettify

# 1. Create components
ground = Raw('<geom name="ground" type="plane" size="10 10 .05"/>')

table = Body(
    name="table",
    pos=[0, 0, 0.5],
    _children_raw='<geom type="box" size="0.5 0.3 0.05" rgba="0.7 0.5 0.3 1"/>'
)

cube = Body(
    name="cube",
    pos=[0, 0, 1],
    _children_raw='''
        <freejoint/>
        <geom type="box" size="0.05 0.05 0.05" rgba="1 0 0 1"/>
    '''
)

# 2. Compose into scene
scene = Mjcf(ground, table, cube, model="TableScene")

# 3. Generate XML
xml = scene._xml | Prettify()
```

## Preamble and Postamble

Components can define assets, materials, and other elements that bubble up:

```python
class ColoredBox(Body):
    _preamble = '''
    <asset>
        <material name="mat_{name}" rgba="{color}"/>
    </asset>
    '''

    template = '''
    <body name="{name}" pos="{pos}">
        <geom type="box" size="{size}" material="mat_{name}"/>
    </body>
    '''

    color = "1 0 0 1"
    size = "0.1 0.1 0.1"

# The preamble automatically goes to the right place
box1 = ColoredBox(name="box1", color="1 0 0 1")
box2 = ColoredBox(name="box2", color="0 1 0 1")

scene = Mjcf(box1, box2, model="Boxes")
# Assets from both boxes appear in <asset> section
```

## Utility Functions

### `mujoco_hash(e: etree.Element) -> str`

Creates a hash identifier for MuJoCo elements based on key attributes (name, class, joint1, body1, etc.). Used for deduplication when merging preambles/postambles.

### `comment_out_joints(xml_str: str) -> str`

Comments out all `<joint>` elements in an XML string. Useful when you want to freeze a body.

**Example:**
```python
from vuer_mjcf.schema.schema import comment_out_joints

xml_with_joints = '<body><joint type="hinge"/><geom type="box"/></body>'
xml_frozen = comment_out_joints(xml_with_joints)
# '<body><!-- <joint type="hinge"/> --><geom type="box"/></body>'
```

## Advanced Usage

### Creating Reusable Components

```python
from vuer_mjcf.schema import Body

class Wheel(Body):
    """Reusable wheel component"""
    template = '''
    <body name="{name}" pos="{pos}">
        <joint name="{name}_joint" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size="{radius} {width}" rgba="0.2 0.2 0.2 1"/>
    </body>
    '''

    radius = "0.1"
    width = "0.05"

# Use it
front_left = Wheel(name="wheel_fl", pos=[0.5, 0.3, 0])
front_right = Wheel(name="wheel_fr", pos=[0.5, -0.3, 0])
```

### Inheriting and Specializing

```python
class Robot(Body):
    _preamble = '''
    <asset>
        <mesh file="{mesh_file}"/>
    </asset>
    '''

    mesh_file = "robot.obj"

class MyRobot(Robot):
    mesh_file = "my_robot.obj"  # Override

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Add custom initialization
```

## See Also

- `vuer_mjcf.schema.base` - Base XML classes (Raw, Xml, XmlTemplate)
- `vuer_mjcf.utils.file` - Utilities like `Prettify()` for formatting
- MuJoCo XML Reference - http://mujoco.org/book/XMLreference.html
