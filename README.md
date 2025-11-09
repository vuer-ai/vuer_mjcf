# vuer_mjcf: MuJoCo Environments for VR Teleoperation

A Python library for creating and visualizing MuJoCo manipulation tasks in VR using Vuer.

## Installation

```bash
# Clone the repository
git clone https://github.com/vuer-ai/vuer_mjcf.git
cd vuer_mjcf

# Install with uv (recommended)
uv pip install -e .

# Or with pip
pip install -e .
```

## Quick Start

### Run a VR Scene Viewer

```bash
# Run a pre-configured scene
python -m vuer_examples.scenes.mug_tree

# Or use the master viewer
python -m vuer_examples.viewer --factory_fn "vuer_mjcf.tasks.pick_place:make_schema"
```

### Use in Your Code

```python
from vuer_mjcf.tasks.pick_place import make_schema

# Generate MuJoCo XML for a task
xml_string = make_schema()

# Load in MuJoCo
import mujoco
model = mujoco.MjModel.from_xml_string(xml_string)
```

## Project Structure

```
vuer_mjcf/                    # Root directory
├── vuer_mjcf/                # Main package
│   ├── basic_components/     # Scene building blocks
│   ├── objects/              # Object definitions
│   ├── robots/               # Robot models
│   ├── schema/               # XML schema definitions
│   ├── stage_sets/           # Pre-built scenes
│   ├── tasks/                # Task definitions
│   ├── third_party/          # External integrations
│   └── utils/                # Utility functions
├── vuer_examples/            # VR viewer examples
│   ├── scenes/               # Pre-configured scenes
│   └── viewer.py             # Master scene viewer
├── assets/                   # MuJoCo assets (meshes, textures)
├── docs/                     # Documentation
└── pyproject.toml            # Package configuration
```

## Available Tasks

State observation contains the `[position, gram-schmidt 6D pose]` of the gripper pose.
Action space is the same `3 + 6` vector, corresponding to the mocap pose.

| **Name**     | **Description**                                    | **Complexity** |
|--------------|----------------------------------------------------|----------------|
| `pick_place` | Pick up a block and place it in a target location | ⭐ Simple      |
| `mug_tree`   | Hang a mug on a tree hook                          | ⭐⭐ Medium     |
| `tie_knot`   | Tie a knot in a rope (bimanual)                    | ⭐⭐⭐ Advanced  |

See `vuer_examples/README.md` for full list of available scenes and examples.

## Documentation

- **VR Viewer Guide**: `vuer_examples/README.md`
- **API Documentation**: https://vuer_mjcf.readthedocs.io
- **Task Definitions**: `vuer_mjcf/tasks/`
- **Stage Sets**: `vuer_mjcf/stage_sets/`

## Development

```bash
# Run tests
pytest

# Format code
ruff check --fix .

# Build documentation
cd docs && make html
```

## License

See LICENSE file for details.