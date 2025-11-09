#!/usr/bin/env python3
"""
Script to refactor all task files:
1. Remove add_env imports
2. Remove make_env import (unless used in make_schema)
3. Remove register() function
4. Remove add_env() calls
5. Replace __main__ block with new template
"""

import re
from pathlib import Path

TASKS_DIR = Path("/Users/yajvanravan/fortyfive/vuer_mjcf/vuer_mjcf/tasks")

# Template for new __main__ block
NEW_MAIN_TEMPLATE = '''if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for {task_name}")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ {task_name} loaded successfully!")
            print(f"  - Number of bodies: {{model.nbody}}")

            data = mujoco.MjData(model)
            print("Launching interactive viewer...")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)
    except ImportError:
        print("MuJoCo not available")
    except Exception as e:
        print(f"✗ Error: {{e}}")
        raise
'''

def get_task_name(filename):
    """Convert filename to human-readable task name."""
    name = filename.replace('.py', '').replace('_', ' ').title()
    return f"{name} task"

def check_make_env_in_make_schema(content):
    """Check if make_env is used in make_schema function."""
    # Find make_schema function
    make_schema_match = re.search(r'def make_schema\([^)]*\):(.*?)(?=\ndef |\nclass |\nif __name__|$)', content, re.DOTALL)
    if make_schema_match:
        make_schema_body = make_schema_match.group(1)
        if 'make_env' in make_schema_body:
            return True
    return False

def refactor_file(filepath):
    """Refactor a single task file."""
    print(f"Processing {filepath.name}...")

    with open(filepath, 'r') as f:
        content = f.read()

    original_content = content

    # Check if the file has a make_schema function
    has_make_schema = 'def make_schema' in content

    # Check if make_env is used in make_schema
    keep_make_env = check_make_env_in_make_schema(content)

    # 1. Remove 'from vuer_mjcf.tasks import add_env' import
    content = re.sub(r'^from vuer_mjcf\.tasks import add_env\n', '', content, flags=re.MULTILINE)

    # 2. Remove 'from vuer_mjcf.tasks.entrypoint import make_env' (unless used in make_schema)
    if not keep_make_env:
        content = re.sub(r'^from vuer_mjcf\.tasks\.entrypoint import make_env.*?\n', '', content, flags=re.MULTILINE)
        content = re.sub(r'^from vuer_mjcf\.tasks\.entrypoint import make_real_env.*?\n', '', content, flags=re.MULTILINE)
        content = re.sub(r'^from vuer_mjcf\.tasks\.entrypoint import make_offline_env.*?\n', '', content, flags=re.MULTILINE)

    # 3. Remove 'from vuer_mjcf.utils.file import Save' at module level
    content = re.sub(r'^from vuer_mjcf\.utils\.file import Save\n', '', content, flags=re.MULTILINE)

    # 4. Remove register() function and all its contents
    # This regex removes from 'def register' until we hit another def, class, or if __name__
    content = re.sub(r'def register\([^)]*\):.*?(?=\n(?:def |class |if __name__|$))', '', content, flags=re.DOTALL)

    # 5. Remove standalone add_env() calls (anywhere in the file)
    # This removes any add_env calls, even those that weren't inside a function
    # We need to handle multi-line add_env calls
    while True:
        old_content = content
        # Match add_env calls with proper parentheses balancing
        content = re.sub(r'\s*add_env\s*\((?:[^)(]|\((?:[^)(]|\([^)(]*\))*\))*\)\s*\n?', '', content, flags=re.DOTALL)
        if old_content == content:
            break

    # 6. Replace __main__ block
    # First remove ALL old __main__ blocks entirely (there might be duplicates from previous runs)
    # Also handle case where if __name__ might be stuck to previous line
    content = re.sub(r'if __name__ == "__main__":.*$', '', content, flags=re.DOTALL)

    # Clean up any trailing whitespace and ensure we end with exactly one newline
    content = content.rstrip() + '\n'

    # Add new __main__ block only if the file has make_schema
    if has_make_schema:
        task_name = get_task_name(filepath.name)
        new_main = NEW_MAIN_TEMPLATE.format(task_name=task_name)
        content = content.rstrip() + '\n\n' + new_main.rstrip() + '\n'

    # Write back
    with open(filepath, 'w') as f:
        f.write(content)

    return original_content != content

# List of all files to process
FILES_TO_PROCESS = [
    "ball_sorting_toy_new.py",
    "ball_sorting_toy_new_astribot.py",
    "ball_sorting_toy_new_floating.py",
    "basketball_shot.py",
    "dual_ur5_pick_block.py",
    "fishing_toy.py",
    "flip_mug.py",
    "insert_shapes.py",
    "juggle_cube_dex_hands.py",
    "juggle_cubes_shadow_hands.py",
    "kitchen_room.py",
    "kitchen_room_xhand.py",
    "microwave_muffin.py",
    "microwave_muffin_astribot.py",
    "move_plate.py",
    "mug_cabinet.py",
    "mug_drawer.py",
    "mug_tree.py",
    "mug_tree_astribot.py",
    "mug_tree_ur.py",
    "obj_permanence.py",
    "orbit_table.py",
    "particle_pour.py",
    "particle_pour_kitchen.py",
    "particle_sweep.py",
    "pick_block.py",
    "pick_place.py",
    "pick_place_eval.py",
    "pick_place_hand.py",
    "pick_place_robot_room.py",
    "pick_place_robot_room_astribot.py",
    "pick_sphere.py",
    "poncho_table.py",
    "pour_liquid.py",
    "pour_liquid_demo.py",
    "pour_liquid_demo_xhand.py",
    "push_t.py",
    "robosuite_door.py",
    "robosuite_lift.py",
    "robosuite_nutassembly.py",
    "robosuite_pickplace.py",
    "robosuite_stack.py",
    "single_ur5_ball_sorting.py",
    "single_ur_mug_tree.py",
    "sort_blocks.py",
    "sort_shape.py",
    "stack_blocks.py",
    "table_scene.py",
    "teddy_bear_table.py",
    "tie_knot.py",
    "tie_knot_astribot.py",
    "tri_demo_kitchen_room_gripper.py",
    "tri_demo_kitchen_room_shadow_hands.py",
    "tri_demo_ur5_tabletop.py",
    "ur5_basic.py",
    "utensil_drawer.py",
    "weighted_cubes_ability_hands.py",
    "weighted_cubes_mpl_hand.py",
    "weighted_cubes_robotic_2f85.py",
    "weighted_cubes_shadow_hands.py",
    "xy_pick_place.py",
    "xy_pick_place_hand.py",
    "xy_pick_place_multi_hand.py",
]

def main():
    modified_count = 0
    skipped_count = 0
    error_count = 0

    for filename in FILES_TO_PROCESS:
        filepath = TASKS_DIR / filename

        if not filepath.exists():
            print(f"WARNING: {filename} not found, skipping")
            skipped_count += 1
            continue

        try:
            if refactor_file(filepath):
                modified_count += 1
            else:
                print(f"  No changes needed for {filename}")
        except Exception as e:
            print(f"ERROR processing {filename}: {e}")
            error_count += 1

    print("\n" + "="*60)
    print("SUMMARY:")
    print(f"  Modified: {modified_count}")
    print(f"  Skipped: {skipped_count}")
    print(f"  Errors: {error_count}")
    print(f"  Total: {len(FILES_TO_PROCESS)}")
    print("="*60)

if __name__ == "__main__":
    main()
