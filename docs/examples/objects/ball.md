# Ball

A simple sphere object with a free joint, perfect for testing physics simulations and manipulation tasks.

## Description

The `Ball` class creates a spherical object with:
- Free 6-DOF motion
- Customizable size and color
- Realistic physics properties (mass, friction, damping)
- Site marker for tracking

## Parameters

- `name` (str): Identifier for the ball (default: "ball")
- `pos` (list): Position [x, y, z] (default: [0, 0, 0])
- `size` (float): Radius of the sphere (default: 0.03)
- `rgba` (str): Color and transparency "r g b a" (default: "1 0 0 0.1" - semi-transparent red)

## Usage Example

```python
from vuer_mjcf.schema import Mjcf
from vuer_mjcf.objects.ball import Ball
from vuer_mjcf.utils.file import Prettify

# Create a red ball
red_ball = Ball(
    name="red_ball",
    pos=[0, 0, 1],
    size=0.05,
    rgba="1 0 0 1"
)

# Create a blue ball
blue_ball = Ball(
    name="blue_ball",
    pos=[0.2, 0, 1],
    size=0.03,
    rgba="0 0 1 1"
)

# Add to scene
scene = Mjcf(red_ball, blue_ball, model="BallsDemo")
xml = scene._xml | Prettify()

# Save or use
scene.save("balls_scene.xml")
```

## Complete Example with Viewer

```python
import tempfile
from pathlib import Path
from vuer_mjcf.stage_sets.default_scene import DefaultStage
from vuer_mjcf.objects.ball import Ball
from vuer_mjcf.utils.file import Prettify

# Create ball
ball = Ball(name="test_ball", pos=[0, 0, 1], rgba="0 1 0 1")

# Create scene with default stage (includes floor, lighting)
scene = DefaultStage(ball, model="ball_demo")

# Generate XML
xml_str = scene._xml | Prettify()

# Launch in MuJoCo viewer
try:
    import mujoco
    import mujoco.viewer

    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(f.name)
        data = mujoco.MjData(model)
        mujoco.viewer.launch(model, data)
        Path(f.name).unlink()
except ImportError:
    print("MuJoCo not available, XML generated successfully")
    print(xml_str)
```

## Physics Properties

The ball includes realistic physics:
- **Mass**: 0.01 kg
- **Friction**: High friction for realistic rolling (10, 0.3, 0.1)
- **Density**: 50 kg/m³
- **Damping**: Soft contact (solimp="0.998 0.998 0.001", solref="0.001 1")
- **Condim**: 4 (friction cone with torsional friction)

## Source Code

See the full implementation in `vuer_mjcf/objects/ball.py`:

```python
class Ball(Body):
    size = 0.03
    rgba = "1 0 0 0.1"

    _attributes = {
        "name": "ball",
    }
    _children_raw = """
    <joint type="free" name="{name}"/>
    <geom name="{name}-sphere" type="sphere" size="{size}" rgba="{rgba}"
          mass="0.01" condim="4" solimp="0.998 0.998 0.001" solref="0.001 1"
          friction="10 0.3 0.1" density="50"/>
    <site name="{name}" pos="0 0 0" size="0.01" rgba="1 0 0 0" type="sphere"/>
    """
```

## Use Cases

- Testing physics simulations
- Object manipulation tasks
- Throwing/catching demonstrations
- Multi-ball juggling
- Collision detection testing

## See Also

- [Bin](bin.md) - Container for collecting balls
- [Basket](basket.md) - Another container option
- [Objects Index](index.md) - All available objects
