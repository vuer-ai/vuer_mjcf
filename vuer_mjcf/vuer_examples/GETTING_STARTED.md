# Getting Started with Vuer Examples

## Installation

The required dependencies have been installed. If you need to reinstall:

```bash
uv add 'vuer[all]'
```

## Quickest Start (3 Commands)

### 1. Run the default viewer
```bash
python -m vuer_mjcf.vuer_examples.viewer
```

This loads the **pick_place** scene by default.

### 2. Open in browser
When the viewer starts, you'll see:
```
Vuer Viewer Running!
Visit: https://vuer.ai/editor?ws=ws://localhost:8012
```

Copy and paste that URL into your browser.

### 3. Interact with VR
- Use VR controllers or mouse to interact
- Squeeze trigger to grasp objects
- Move controllers to manipulate

## Try Different Scenes

### Method 1: Run Individual Examples
```bash
# Beginner
python -m vuer_mjcf.vuer_examples.scenes.simple_pick_place
python -m vuer_mjcf.vuer_examples.scenes.push_t

# Intermediate
python -m vuer_mjcf.vuer_examples.scenes.mug_tree
python -m vuer_mjcf.vuer_examples.scenes.microwave_muffin

# Advanced
python -m vuer_mjcf.vuer_examples.scenes.tie_knot
python -m vuer_mjcf.vuer_examples.scenes.pour_liquid
python -m vuer_mjcf.vuer_examples.scenes.kitchen_room
```

### Method 2: Edit viewer.py Defaults
Open `vuer_mjcf/vuer_examples/viewer.py` and scroll to the bottom.

Uncomment your preferred scene:
```python
# Change this:
ViewerParams.factory_fn = "vuer_mjcf.tasks.pick_place:make_schema"

# To this (for example):
ViewerParams.factory_fn = "vuer_mjcf.tasks.mug_tree:make_schema"
ViewerParams.name = "mug_tree"
```

Then run:
```bash
python -m vuer_mjcf.vuer_examples.viewer
```

### Method 3: Use Command-Line Arguments
```bash
python -m vuer_mjcf.vuer_examples.viewer \
    --factory_fn "vuer_mjcf.tasks.kitchen_room:make_schema" \
    --name "kitchen_room" \
    --actuators "mono" \
    --verbose
```

### Method 4: Interactive Menu
```bash
python -m vuer_mjcf.vuer_examples.run_example
```

This shows a menu where you can select scenes by number.

## See All Available Examples

```bash
python -m vuer_mjcf.vuer_examples.list_examples
```

## Common Options

- `--actuators mono` - Single hand (right hand only)
- `--actuators duo` - Both hands (bimanual control)
- `--actuators none` - No hand actuators (for robot arms)
- `--show_lights` - Enable scene lighting (slower but prettier)
- `--verbose` - Print debug information
- `--vuer_port 8013` - Use different port if 8012 is busy

## Troubleshooting

**Port already in use:**
```bash
python -m vuer_mjcf.vuer_examples.viewer --vuer_port 8013
```

**Can't connect in browser:**
- Make sure the URL matches the port in the console output
- Try `ws://localhost:8012` instead of the ngrok URL if local

**Import errors:**
- Make sure you're in the project root directory
- Reinstall dependencies: `uv add 'vuer[all]'`

**Assets not loading:**
- Check that you have the asset folders in the correct location
- The viewer creates an `*_export` folder with assets automatically

## What Each Example Does

| Example | What You'll See | Difficulty |
|---------|-----------------|------------|
| `simple_pick_place` | Green cube, red/blue areas | ⭐ Easy |
| `push_t` | T-shaped object to push | ⭐ Easy |
| `mug_tree` | Mug + tree with hooks | ⭐⭐ Medium |
| `microwave_muffin` | Kitchen with microwave | ⭐⭐ Medium |
| `tie_knot` | Flexible rope to tie | ⭐⭐⭐ Hard |
| `pour_liquid` | Containers with particles | ⭐⭐⭐ Hard |
| `kitchen_room` | Full kitchen scene | ⭐⭐⭐ Hard |
| `mug_tree_ur` | Same as mug_tree but with robot arm | ⭐⭐ Medium |
| `shadow_hands_kitchen` | Kitchen with dexterous hands | ⭐⭐⭐⭐ Expert |

## Next Steps

1. **Start simple**: Try `simple_pick_place` first
2. **Read the code**: Each example file has detailed comments
3. **Customize**: Edit `viewer.py` defaults for your favorite scene
4. **Explore**: All 9 examples demonstrate different capabilities

For full documentation, see `README.md` in this folder.
