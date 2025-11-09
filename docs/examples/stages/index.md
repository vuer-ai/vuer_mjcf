# Stages

Stages are pre-configured complete scenes that combine robots, environments, and lighting. They provide ready-to-use setups for various robotics tasks.

## Featured Stages

- [Default Scene](default_scene.md) - Basic scene with floor and lighting
- [Panda + Robotiq](panda_robotiq.md) - Panda arm with parallel gripper
- [Panda + Shadow Hand](panda_shadow_hand.md) - Panda with dexterous hand
- [UR5e + Robotiq](ur5e_robotiq.md) - UR5e arm with gripper
- [Dual Panda](dual_panda.md) - Two Panda arms for bimanual tasks
- [G1 Scene](g1_scene.md) - G1 humanoid robot
- [Astribot + Robotiq](astribot_robotiq.md) - Astribot with gripper

## All Available Stages

### Default/Base Scenes
- **Default Scene** (`default_scene.py`) - Basic scene with floor, lighting, and skybox

### Single Arm Scenes
- **R5a Scene** (`r5a_scene.py`) - Scene with R5a robot
- **X5a Scene** (`x5a_scene.py`) - Scene with X5a robot
- **X7 Scene** (`x7_scene.py`) - Scene with X7 robot
- **X7s Scene** (`x7s_scene.py`) - Scene with X7s robot
- **Lift Scene** (`lift_scene.py`) - Scene with Lift robot
- **Ufactory xArm7 Scene** (`ufactory_xarm7_scene.py`) - Scene with xArm7
- **Single Widow X Arm** (`single_widow_x_arm.py`) - Floating Widow-X setup

### Arm + Gripper Combinations
- **Panda + Robotiq** (`panda_robotiq_scene.py`) - Panda arm with Robotiq gripper
- **Panda + Tomika** (`panda_tomika_scene.py`) - Panda arm with Tomika gripper
- **UR5e + Robotiq** (`ur5e_robotiq_scene.py`) - UR5e arm with Robotiq gripper
- **Astribot + Robotiq** (`astribot_robotiq.py`) - Astribot with Robotiq gripper

### Arm + Dexterous Hand Combinations
- **Panda + Shadow Hand** (`panda_shadow_hand_scene.py`) - Panda arm with Shadow Hand
- **UR5e + Shadow Hand** (`ur5e_shadow_hand_scene.py`) - UR5e arm with Shadow Hand
- **Astribot + Shadow Hand** (`astribot_shadow_hand.py`) - Astribot with Shadow Hand
- **XHand Scene** (`xhand_scene.py`) - Scene with XHand

### Dual Arm Scenes
- **Dual Panda** (`dual_panda.py`) - Two Panda arms for bimanual manipulation
- **Trossen Table** (`trossen_table.py`) - Dual-arm Trossen setup

### Humanoid Scenes
- **G1 Scene** (`g1_scene.py`) - G1 humanoid robot
- **Astribot Scene** (`astribot_scene.py`) - Astribot humanoid

### Utility/Example Scenes
- **Menagerie** (`menagerie.py`) - Collection of models from MuJoCo Menagerie

## Usage Example

Using a pre-configured stage:

```python
from vuer_mjcf.stage_sets.panda_robotiq_scene import PandaRobotiq2f85
from vuer_mjcf.utils.file import Prettify

# Create complete scene with Panda + Robotiq
scene = PandaRobotiq2f85(
    assets="path/to/assets",
    camera_rig=None,  # Use default camera
)

# Generate XML
xml = scene._xml | Prettify()

# Save to file
scene.save("panda_robotiq.xml")
```

## Adding Objects to Stages

You can add objects to any stage:

```python
from vuer_mjcf.stage_sets.panda_robotiq_scene import PandaRobotiq2f85
from vuer_mjcf.objects.ball import Ball
from vuer_mjcf.objects.bin import Bin

# Create objects
ball = Ball(name="ball", pos=[0.3, 0, 0.5])
container = Bin(name="bin", pos=[0.5, 0, 0.3])

# Create scene with objects
scene = PandaRobotiq2f85(ball, container)
```

## Customizing Stages

Stages support customization:

```python
from vuer_mjcf.stage_sets.default_scene import DefaultStage

# Custom position for elements
scene = DefaultStage(
    my_objects,
    pos=[0, 0, 1],  # Base position
    assets="custom/assets/path"
)
```

## Stage Components

Most stages include:
- **Robot(s)**: Arm(s), gripper(s), or hand(s)
- **Environment**: Floor/ground plane
- **Lighting**: Key, fill, and back lights
- **Camera Rig**: Optional camera positions
- **Mocap Bodies**: For motion capture control

## Dual Robot Configuration

Some stages support dual robot setups:

```python
from vuer_mjcf.stage_sets.panda_robotiq_scene import PandaRobotiq2f85

# Enable dual robot mode
scene = PandaRobotiq2f85(dual_robot=True)
```

## Asset Organization

Stages require asset files organized as:
```
assets/
└── robots/
    ├── franka_panda/
    ├── ur5e/
    ├── robotiq_2f85/
    ├── shadow_hand/
    └── ...
```

## See Also

- [Robots](../robots/index.md) - Available robot models
- [Objects](../objects/index.md) - Objects to add to stages
- API Reference - Stage base classes
