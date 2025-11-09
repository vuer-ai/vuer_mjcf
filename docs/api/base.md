# `vuer_mjcf.schema.base`

The base module provides the foundational classes for building XML structures in vuer_mjcf. These classes use a compositional approach to construct complex MuJoCo MJCF files from simple, reusable components.

## Overview

The module contains three main classes:

- **Raw** - Wraps raw XML strings for direct insertion
- **Xml** - Base class for structured XML elements
- **XmlTemplate** - Advanced template-based XML with preamble/postamble support

## Classes

### `Raw`

A simple wrapper for raw XML strings that need to be inserted directly into the document.

**Constructor:**
```python
Raw(string, **kwargs)
```

**Parameters:**
- `string` (str): The raw XML string to wrap
- `**kwargs`: Optional format arguments for string interpolation

**Example:**
```python
from vuer_mjcf.schema import Raw

# Simple raw XML
ground = Raw('<geom name="ground" type="plane" size="5 5 .05"/>')

# With format arguments
light = Raw('<light pos="{x} {y} {z}"/>', x=0, y=0, z=5)
```

**Properties:**
- `_xml` - Returns the XML string

---

### `Xml`

Base class for all XML elements. Provides structure for tag-based XML with attributes and children.

**Constructor:**
```python
Xml(*_children, tag=None, children=None, **attributes)
```

**Parameters:**
- `*_children`: Variable positional arguments for child elements
- `tag` (str, optional): XML tag name (defaults to class attribute `tag`)
- `children` (tuple, optional): Alternative way to specify children
- `**attributes`: XML attributes as keyword arguments

**Class Attributes:**
- `tag` (str): The XML tag name (e.g., "body", "geom")
- `_attributes` (dict): Default attributes for the element
- `_children` (tuple): Child elements
- `_children_raw` (str): Raw XML string for children

**Properties:**
- `attributes` - Returns formatted attribute string
- `children` - Returns formatted children string
- `_xml` - Returns complete XML string
- `_minimized` - Returns minimized XML (whitespace removed)

**Example:**
```python
from vuer_mjcf.schema.base import Xml

# Create a simple XML element
geom = Xml(tag="geom", type="sphere", size="0.1", rgba="1 0 0 1")
print(geom._xml)
# <geom type="sphere" size="0.1" rgba="1 0 0 1"></geom>

# With children
body = Xml(
    Xml(tag="geom", type="box", size="0.1 0.1 0.1"),
    Xml(tag="joint", type="hinge", axis="0 0 1"),
    tag="body",
    name="link1"
)
```

---

### `XmlTemplate`

Advanced XML class that uses string templates for structure. Supports preamble (assets, defaults) and postamble (actuators, sensors) sections that bubble up through the component hierarchy.

**Constructor:**
```python
XmlTemplate(*args, preamble=None, postamble=None, **kwargs)
```

**Parameters:**
- `*args`: Child elements
- `preamble` (str, optional): Preamble XML (overrides class `_preamble`)
- `postamble` (str, optional): Postamble XML (overrides class `_postamble`)
- `**kwargs`: Attributes and other parameters

**Class Attributes:**
- `template` (str): Format string for the XML structure
- `_preamble` (str): XML to insert before main content (e.g., assets)
- `_postamble` (str): XML to insert after main content (e.g., actuators)
- `_children_raw` (str): Raw XML template for children

**Properties:**
- `preamble` - Returns preamble with children's preambles merged
- `children` - Returns formatted children from template
- `postamble` - Returns postamble with children's postambles merged
- `_xml` - Returns complete XML from template

**Methods:**
- `_format_dict(omit: set = {})` - Collects all attributes including inherited ones
- `join(*s: str)` - Joins non-empty strings with newlines

**Example:**
```python
from vuer_mjcf.schema.base import XmlTemplate

class Sphere(XmlTemplate):
    tag = "body"
    template = '''
    <body {attributes}>
        <geom type="sphere" size="{radius}" rgba="{color}"/>
        {children}
    </body>
    '''

    _preamble = '''
    <asset>
        <material name="mat_{name}" rgba="{color}"/>
    </asset>
    '''

    # Default values
    radius = "0.1"
    color = "1 0 0 1"

# Create instance
ball = Sphere(name="ball1", radius="0.2", color="0 1 0 1")

# Access different parts
print(ball.preamble)  # Assets section
print(ball._xml)      # Complete XML
```

## Template System

The template system allows you to:

1. **Define structure once**: Use format strings with placeholders
2. **Inherit attributes**: Child classes automatically inherit parent attributes
3. **Compose hierarchically**: Preambles and postambles automatically merge up the tree
4. **Format dynamically**: All class attributes and properties available for formatting

**Format Variables Available:**
- All class attributes (e.g., `name`, `size`, `pos`)
- All properties (accessed via `@property`)
- All instance `__dict__` values
- All `_attributes` dictionary values

**Example of Attribute Inheritance:**
```python
class ColoredBody(XmlTemplate):
    template = '<body name="{name}" {attributes}><geom rgba="{rgba}"/></body>'
    rgba = "1 1 1 1"  # Default white

class RedBody(ColoredBody):
    rgba = "1 0 0 1"  # Override to red

# Instances inherit the hierarchy
white = ColoredBody(name="white_body")  # Uses "1 1 1 1"
red = RedBody(name="red_body")          # Uses "1 0 0 1"
custom = RedBody(name="custom", rgba="0 0 1 1")  # Override to blue
```

## Best Practices

1. **Use Raw for simple insertions**: When you just need to inject XML without structure
2. **Use Xml for programmatic construction**: When building XML element by element
3. **Use XmlTemplate for reusable components**: When creating components with templates
4. **Leverage preambles**: Put assets, materials, textures in `_preamble`
5. **Leverage postambles**: Put actuators, sensors, equalities in `_postamble`
6. **Use inheritance**: Create base classes with common structure and specialize

## See Also

- `vuer_mjcf.schema.schema` - MuJoCo-specific schema classes built on these bases
- MuJoCo XML Reference - http://mujoco.org/book/XMLreference.html
