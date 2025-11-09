# Robots

Vuer-MJCF provides a comprehensive collection of robot models including robotic arms, grippers, dexterous hands, and humanoids.

## Robot Arms

- [**Franka Panda**](franka_panda.md) (`franka_panda.py`) - Popular 7-DOF collaborative robot arm
- [**UR5e**](ur5e.md) (`ur5e.py`) - Universal Robots UR5e collaborative arm
- [**Ufactory xArm7**](ufactory_xarm7.md) (`ufactory_xarm7.py`) - Ufactory's 7-DOF robotic arm
- [**Trossen Widow X**](widow_x.md) (`trossen_widow_x_arm.py`) - Trossen Robotics Widow-X arm
- [**X5a**](x5a.md) (`x5a.py`) - X5a robot arm
- [**X7**](x7.md) (`x7.py`) - X7 robot arm
- [**X7s**](x7s.md) (`x7s.py`) - X7s robot arm variant
- [**R5a**](r5a.md) (`r5a.py`) - R5a robot arm

## Grippers (Parallel-Jaw)

- [**Robotiq 2F-85**](robotiq_2f85.md) (`robotiq_2f85.py`) - Popular parallel-jaw gripper
- [**Tomika Gripper**](tomika_gripper.md) (`tomika_gripper.py`) - Tomika parallel gripper
- [**Ufactory Gripper**](ufactory_gripper.md) (`ufactory_gripper.py`) - Ufactory's gripper

## Dexterous Hands

- [**Shadow Hand**](shadow_hand.md) (`shadow_hand.py`) - High-fidelity 5-finger dexterous hand (left/right)
- [**Ability Hand**](ability_hand.md) (`ability_hand.py`) - Anthropomorphic hand (left/right)
- [**XHand**](xhand.md) (`xhand.py`) - XHand dexterous manipulator (left/right)
- [**DexHand**](dexhand.md) (`dexhand.py`) - DexHand dexterous manipulator (left/right)
- [**MPL Hand**](mpl_hand.md) (`mpl_hand.py`) - Modular Prosthetic Limb hand (right)

## Humanoid Robots

- [**G1**](g1.md) (`g1.py`) - G1 humanoid robot
- [**Astribot**](astribot.md) (`astribot.py`) - Astribot humanoid/mobile manipulator
- [**Lift**](lift.md) (`lift.py`) - Lift mobile humanoid robot

## Usage Example

Basic robot usage:

```python
from vuer_mjcf.schema import Mjcf
from vuer_mjcf.robots.franka_panda import Panda
from vuer_mjcf.robots.robotiq_2f85 import Robotiq2F85
from vuer_mjcf.utils.file import Prettify

# Create robot arm
panda = Panda(
    name="panda",
    pos=[0, 0, 0],
    assets="path/to/panda/assets"
)

# Create gripper as end effector
gripper = Robotiq2F85(
    name="gripper",
    assets="path/to/gripper/assets",
    pos=[0, 0, 0]
)

# Combine (or use pre-configured stage sets)
scene = Mjcf(panda, model="RobotDemo")
xml = scene._xml | Prettify()
```

## Using with Stages

Most robots are available in pre-configured stage sets:

```python
from vuer_mjcf.stage_sets.panda_robotiq_scene import PandaRobotiq2f85

# Complete scene with Panda + Robotiq gripper
scene = PandaRobotiq2f85(assets="path/to/assets")
```

See [Stages](../stages/index.md) for all pre-configured robot setups.

## Robot Compatibility

### Arm + Gripper Combinations

Compatible arm and gripper combinations:
- Panda + Robotiq 2F-85
- Panda + Tomika
- UR5e + Robotiq 2F-85
- Astribot + Robotiq 2F-85
- xArm7 + Ufactory Gripper

### Arm + Dexterous Hand Combinations

Compatible arm and hand combinations:
- Panda + Shadow Hand
- UR5e + Shadow Hand
- Astribot + Shadow Hand
- Standalone XHand, DexHand, Ability Hand, MPL Hand

## Assets

Robot models require asset files (meshes, textures). These should be placed in:
```
assets/robots/
├── franka_panda/
├── ur5e/
├── robotiq_2f85/
├── shadow_hand/
└── ...
```

## See Also

- [Stages](../stages/index.md) - Pre-configured robot scenes
- [Objects](../objects/index.md) - Objects for manipulation
- API Reference - Robot base classes
