"""
Simplified Vuer Scene Viewer

A streamlined version of the demo collection script focused on scene visualization.
This viewer loads MuJoCo scenes into Vuer without data collection features.

Usage:
    python -m vuer_mjcf.vuer_examples.viewer --factory_fn "vuer_mjcf.tasks.mug_tree:make_schema"

Or import and use directly from scene examples.
"""

import os
import sys
import shutil
import tempfile
from asyncio import sleep
from importlib import import_module
from os.path import join
from pathlib import Path
from typing import List, Literal

import numpy as np
from params_proto import ARGS, Flag, ParamsProto, Proto


class ViewerParams(ParamsProto, cli_parse=False):
    """Parameters for the Vuer scene viewer."""

    # Server configuration
    vuer_port: int = Proto(8012, help="Vuer server port")

    # Scene configuration
    name: str = Proto("scene", help="Scene name")
    factory_fn: str = Proto(None, help="Factory function path (module:function)")

    # Asset paths
    assets: str = Proto("{name}", help="Asset directory path")
    asset_prefix: str = Proto("http://localhost:{vuer_port}/static", help="Asset URL prefix")

    # MuJoCo configuration
    frame_keys: str = Proto(
        "mocap_pos mocap_quat qpos qvel site_xpos site_xmat ctrl sensordata",
        help="Frame data keys to track"
    )
    visible_groups: List[int] = Proto([0, 1, 2], help="Visible MuJoCo groups")
    show_lights: bool = Flag("Show scene lights (may reduce performance)")
    fps: int = Proto(50, help="Simulation FPS")

    # Hand actuator mode
    actuators: Literal["mono", "duo", "none"] = Proto(
        "mono",
        help="Hand actuator mode: 'mono' (single hand), 'duo' (both hands), 'none' (no actuators)"
    )

    # Debug options
    verbose: bool = Flag("Print verbose output")

    def __post_init__(self):
        """Format string parameters with self.__dict__ values."""
        for k, v in self.__dict__.items():
            if isinstance(v, str) and "{" in v:
                value = v.format(**self.__dict__)
                setattr(self, k, value)


def create_mujoco_component(params: ViewerParams):
    """Create the MuJoCo viewer component with proper configuration.

    Args:
        params: ViewerParams instance with scene configuration

    Returns:
        MuJoCo component configured for the scene
    """
    from vuer.schemas import HandActuator, MuJoCo
    from vuer_mjcf.utils.collect_asset_paths import collect_asset_paths

    # Collect asset paths
    asset_folder = params.assets
    path = build_xml(params,return_path=True)
    assets = collect_asset_paths(path)
    asset_paths = [join(params.asset_prefix, params.assets, asset) for asset in assets]

    if params.verbose:
        print(f"Found {len(assets)} assets in {asset_folder}")
        print("Assets:", *assets, sep="\n")

    # Create hand actuators based on mode
    actuators = []

    if params.actuators.lower() in ["mono", "duo"]:
        # Right hand actuator
        actuators.append(
            HandActuator(
                key="pinch-on-squeeze",
                cond="right-squeeze",
                value="right:thumb-tip,right:index-finger-tip",
                offset=0.10,
                scale=-12,
                low=0,
                high=1,
                ctrlId=-1,
            )
        )

    if params.actuators.lower() == "duo":
        # Left hand actuator
        actuators.append(
            HandActuator(
                key="left-pinch-on-squeeze",
                cond="left-squeeze",
                value="left:thumb-tip,left:index-finger-tip",
                offset=0.10,
                scale=-12,
                low=0,
                high=1,
                ctrlId=-2,
            )
        )

    # Create MuJoCo component
    # Include init_keyframe if it was loaded from YAML
    init_keyframe = getattr(params, '_init_keyframe', {})

    return MuJoCo(
        *actuators,
        key="viewer-sim",
        src=params.asset_prefix + "/" + path.name,
        assets=asset_paths,
        frameKeys=params.frame_keys,
        pause=True,
        useLights=params.show_lights,
        visible=params.visible_groups,
        mocapHandleSize=0.05,
        mocapHandleWireframe=True,
        fps=params.fps,
        **init_keyframe,
    )

def build_xml(params, return_path=False):
    """Build XML from factory function."""
    module_name, fn_name = params.factory_fn.rsplit(":", 1)

    # Auto-reload module for hot reloading during development
    if module_name in sys.modules:
        del sys.modules[module_name]

    m = import_module(module_name)
    xml = getattr(m, fn_name)(dual_gripper=(params.actuators == "duo"))

    # Export XML and assets
    export_dir = join(os.getcwd(), f"{params.name}_export")
    assets_dir = join(export_dir, "assets")
    os.makedirs(assets_dir, exist_ok=True)

    # Save XML file
    xml_path_local = join(export_dir, f"{params.name}.mjcf.xml")
    with open(xml_path_local, "w") as f:
        f.write(xml)
    if return_path:
        return Path(xml_path_local)

    # Copy assets
    from vuer_mjcf.utils.collect_asset_paths import collect_asset_paths
    assets = collect_asset_paths(xml_path_local)
    for asset in assets:
        src_path = join(params.assets, asset)
        dst_path = join(assets_dir, asset)
        os.makedirs(os.path.dirname(dst_path), exist_ok=True)
        if os.path.exists(src_path):
            shutil.copy2(src_path, dst_path)


    return xml

def load_scene(
    factory_fn: str,
    name: str = None,
    port: int = 8012,
    actuators: Literal["mono", "duo", "none"] = "mono",
    show_lights: bool = False,
    verbose: bool = False,
):
    """Load and display a MuJoCo scene in Vuer.

    Args:
        factory_fn: Factory function path in format "module.path:function_name"
        name: Scene name (default: derived from factory_fn)
        port: Vuer server port (default: 8012)
        actuators: Hand actuator mode - "mono", "duo", or "none" (default: "mono")
        show_lights: Whether to show scene lights (default: False)
        verbose: Print verbose output (default: False)

    Example:
        load_scene("vuer_mjcf.tasks.mug_tree:make_schema", name="mug_tree")
    """
    from vuer import Vuer, VuerSession

    # Configure parameters
    params = ViewerParams()
    params.factory_fn = factory_fn
    params.vuer_port = port
    params.actuators = actuators
    params.show_lights = show_lights
    params.verbose = verbose

    # Derive name from factory_fn if not provided
    if name:
        params.name = name
    else:
        module_name = factory_fn.split(":")[0]
        params.name = module_name.split(".")[-1]

    # Re-run post_init to format strings
    params.__post_init__()

    if verbose:
        print(f"Loading scene: {params.name}")
        print(f"Factory function: {params.factory_fn}")
        print(f"Port: {params.vuer_port}")

    # Create Vuer server with static file serving
    vuer = Vuer(static_root="..", port=params.vuer_port)

    # Add dynamic route for XML generation
    xml_path = "/static/" + params.name + ".mjcf.xml"

    vuer.add_route(xml_path, lambda: build_xml(params))

    # Print connection URL
    print(f"\nVuer Viewer Running!")
    print(f"Visit: https://vuer.ai?ws=ws://localhost:{params.vuer_port}")
    print(f"Scene: {params.name}")
    print(f"Press Ctrl+C to exit\n")

    # Flag to track plugin loading
    is_loaded = False

    @vuer.add_handler("ON_CONTRIB_LOAD")
    async def on_contrib_load(event, proxy):
        nonlocal is_loaded
        is_loaded = True
        if verbose:
            print("MuJoCo plugin loaded")

    last_frame = None

    @vuer.add_handler("ON_MUJOCO_FRAME")
    async def on_frame(event, proxy: VuerSession):
        """Capture the latest frame for recording initial state."""
        nonlocal last_frame
        last_frame = event.value.get("keyFrame")

    async def handle_reset(proxy: VuerSession):
        """Reset the scene by reloading the MuJoCo component with saved keyframe if available."""
        if verbose:
            print("Reset button pressed - reloading scene...")

        import yaml
        from pathlib import Path

        # Try to load saved keyframe from {name}.frame.yaml
        frame_file = Path(f"{params.name}.frame.yaml")

        if frame_file.exists():
            try:
                with open(frame_file, "r") as f:
                    keyframes = yaml.load(f, Loader=yaml.FullLoader)

                if keyframes and len(keyframes) > 0:
                    # Use the last keyframe
                    last_keyframe = keyframes[-1]

                    # Extract relevant keys for initialization
                    init_keyframe = {
                        k: np.array(v)
                        for k, v in last_keyframe.items()
                        if k in ["qpos", "qvel", "mocap_pos", "mocap_quat", "ctrl"]
                    }

                    if verbose:
                        print(f"Loaded keyframe from {frame_file}")
                        print(f"Keys: {list(init_keyframe.keys())}")

                    # Update params with the saved keyframe
                    # We need to pass this to the MuJoCo component
                    # Store it temporarily in a way create_mujoco_component can access it
                    params._init_keyframe = init_keyframe
                else:
                    params._init_keyframe = {}

            except Exception as e:
                if verbose:
                    print(f"Error loading keyframe: {e}")
                params._init_keyframe = {}
        else:
            params._init_keyframe = {}

        proxy.upsert @ create_mujoco_component(params)

        if verbose:
            print("Scene reset complete!")

    async def handle_record(proxy: VuerSession):
        """Record the current pose as initial state."""
        nonlocal last_frame

        if last_frame is None:
            print("No frame data available to record")
            return

        import yaml
        from pathlib import Path

        # Save to {name}.frame.yaml
        frame_file = Path(f"{params.name}.frame.yaml")

        if verbose:
            print(f"Recording initial pose to {frame_file}")

        # Append to yaml file
        yaml_content = yaml.dump([last_frame], default_flow_style=False)

        if frame_file.exists():
            with open(frame_file, "a") as f:
                f.write(yaml_content)
        else:
            with open(frame_file, "w") as f:
                f.write(yaml_content)

        print(f"Initial pose saved to {frame_file}")

    @vuer.add_handler("ON_CLICK")
    async def on_click(event, proxy: VuerSession):
        """Handle button click events."""
        key = event.value.get("key")

        if key == "reset-button":
            await handle_reset(proxy)
            print(f"Scene reset")
        elif key == "record-button":
            await handle_record(proxy)

    @vuer.spawn(start=True)
    async def main_loop(proxy: VuerSession):
        nonlocal is_loaded
        from vuer.schemas import Box, Html, Sphere, group, span

        # Wait for plugin to load (with timeout)
        timeout = 10.0
        elapsed = 0.0
        while not is_loaded and elapsed < timeout:
            if verbose:
                print(f"Waiting for MuJoCo plugin to load... ({elapsed:.1f}s)")
            await sleep(1.0)
            elapsed += 1.0

        if not is_loaded:
            print("Warning: Plugin load timeout. Attempting to load scene anyway.")

        # Load the MuJoCo scene
        proxy.upsert @ create_mujoco_component(params)

        if verbose:
            print("Scene loaded successfully!")

        # Add reset button
        proxy.upsert @ group(
            Html(
                span("reset scene"),
                key="reset-label",
                style={"top": 30, "width": 700, "fontSize": 20},
            ),
            Box(
                args=[0.25, 0.25, 0.25],
                key="reset-button",
                material={"color": "#23aaff"},
            ),
            key="reset-button",
            position=[-0.4, 1.4, -1],
        )

        # Add record initial pose button
        proxy.upsert @ group(
            Html(
                span("record initial pose"),
                key="record-label",
                style={"top": 30, "width": 150, "fontSize": 20},
            ),
            Sphere(
                args=[0.1, 32, 16],
                key="record-button",
                material={"color": "red"},
            ),
            key="record-button",
            position=[0.4, 1.4, -1],
        )

        if verbose:
            print("Reset button added at position [-0.4, 1.4, -1]")
            print("Record button added at position [0.4, 1.4, -1]")

        # Keep the server running
        while True:
            await sleep(1.0)


def main():
    """Main entry point for CLI usage."""
    # Parse CLI arguments
    ARGS.parse_args()
    params = ViewerParams()

    if not params.factory_fn:
        print("Error: --factory_fn is required")
        print("\nUsage:")
        print('  python -m vuer_mjcf.vuer_examples.viewer --factory_fn "vuer_mjcf.tasks.mug_tree:make_schema"')
        print("\nOptions:")
        print("  --name: Scene name (default: derived from factory_fn)")
        print("  --vuer_port: Server port (default: 8012)")
        print("  --actuators: Hand mode - mono/duo/none (default: mono)")
        print("  --show_lights: Enable scene lighting")
        print("  --verbose: Print verbose output")
        sys.exit(1)

    # Load the scene
    load_scene(
        factory_fn=params.factory_fn,
        name=params.name,
        port=params.vuer_port,
        actuators=params.actuators,
        show_lights=params.show_lights,
        verbose=params.verbose,
    )


if __name__ == "__main__":
    # Set default parameters for easy testing
    # Comment out or modify these to try different scenes

    ViewerParams.assets = "assets"
    ViewerParams.verbose = True
    # Simple example (default)
    ViewerParams.factory_fn = "vuer_mjcf.tasks.pick_place:make_schema"
    ViewerParams.name = "pick_place"
    ViewerParams.actuators = "mono"

    # Other examples to try (uncomment to use):

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.push_t:make_schema"
    # ViewerParams.name = "push_t"
    # ViewerParams.actuators = "none"

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.mug_tree:make_schema"
    # ViewerParams.name = "mug_tree"
    # ViewerParams.actuators = "mono"

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.microwave_muffin:make_schema"
    # ViewerParams.name = "microwave_muffin"
    # ViewerParams.actuators = "mono"

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.tie_knot:make_schema"
    # ViewerParams.name = "tie_knot"
    # ViewerParams.actuators = "duo"

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.pour_liquid:make_schema"
    # ViewerParams.name = "pour_liquid"
    # ViewerParams.actuators = "duo"

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.kitchen_room:make_schema"
    # ViewerParams.name = "kitchen_room"
    # ViewerParams.actuators = "mono"

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.mug_tree_ur:make_schema"
    # ViewerParams.name = "mug_tree_ur"
    # ViewerParams.actuators = "none"

    # ViewerParams.factory_fn = "vuer_mjcf.tasks.tri_demo_kitchen_room_shadow_hands:make_schema"
    # ViewerParams.name = "tri_demo_kitchen_room_shadow_handsnow pl"
    # ViewerParams.actuators = "none"

    # Optional: Change port
    # ViewerParams.vuer_port = 8013

    main()
