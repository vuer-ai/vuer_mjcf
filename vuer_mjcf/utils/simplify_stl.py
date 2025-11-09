#!/usr/bin/env python3
"""
simplify_stls_in_dir.py

Usage:
  python simplify_stls_in_dir.py /path/to/folder [--reduction 0.5]
  # or target a triangle count:
  python simplify_stls_in_dir.py /path/to/folder --target-triangles 25000
  # dry-run to preview work:
  python simplify_stls_in_dir.py /path/to/folder --dry-run

Notes:
- Overwrites meshes in place (writes to a temporary file, then atomically replaces).
- By default removes ~50% of triangles. Use --target-triangles to specify an explicit target.
- Requires: pip install open3d
"""
import glob

import argparse
import os
import sys
from pathlib import Path
from typing import Optional
import tempfile
import shutil

try:
    import open3d as o3d
except Exception:
    print("ERROR: This script requires Open3D. Install with: pip install open3d", file=sys.stderr)
    raise

def parse_args():
    p = argparse.ArgumentParser(description="Simplify all STL meshes under a folder (recursive), overwriting in place.")
    p.add_argument("--folder", type=Path, help="Folder containing STL files (searched recursively).",
                   default="vuer_mujoco/tasks/assets/xhand_left")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--reduction", type=float, default=0.5,
                   help="Fraction of triangles to remove (0..1). 0.5 means ~50%% fewer triangles. Default: 0.5")
    g.add_argument("--target-triangles", type=int, default=None,
                   help="Target triangle count (overrides --reduction if provided).")
    p.add_argument("--pattern", type=str, default="*",
                   help="Glob pattern (case-insensitive) to match files, default *.stl")
    p.add_argument("--dry-run", action="store_true",
                   help="List what would be done without modifying files.")
    p.add_argument("--min-tris", type=int, default=1,
                   help="Skip meshes with fewer than this many triangles. Default: 1")
    return p.parse_args()

def iter_stl_files(root: Path, pattern: str):
    # Case-insensitive glob by checking suffix after rglob pattern
    for p in root.rglob(pattern):
        print(p.suffix.lower())
        if p.is_file() and p.suffix.lower() == ".stl":
            yield p
def simplify_stl_inplace(path: Path, reduction: Optional[float], target_tris: Optional[int], min_tris: int):
    if not path.exists():
        raise FileNotFoundError(path)

    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.is_empty():
        raise ValueError(f"Failed to load or empty mesh: {path}")

    # Cleanup before decimation
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()

    cur_tris = len(mesh.triangles)
    if cur_tris < min_tris:
        return f"Skipped (triangles {cur_tris} < {min_tris})"

    # Determine target triangle count
    if target_tris is None:
        keep_ratio = max(0.01, min(1.0, 1.0 - float(reduction if reduction is not None else 0.5)))
        target_tris = max(1, int(cur_tris * keep_ratio))
    target_tris = max(1, min(cur_tris, target_tris))

    if target_tris == cur_tris:
        return f"Skipped (already at target: {cur_tris} tris)"

    simplified = mesh.simplify_quadric_decimation(target_number_of_triangles=target_tris)

    # Cleanup after decimation
    simplified.remove_duplicated_vertices()
    simplified.remove_degenerate_triangles()
    simplified.remove_duplicated_triangles()
    simplified.remove_non_manifold_edges()
    simplified.compute_vertex_normals()

    # Write to a temp file then atomically replace the original
    with tempfile.TemporaryDirectory(dir=str(path.parent)) as td:
        tmp = Path(td) / (path.stem + "_tmp" + path.suffix )
        ok = o3d.io.write_triangle_mesh(str(tmp), simplified, print_progress=False)
        if not ok:
            raise IOError(f"Failed to write temporary mesh: {tmp}")
        # On some filesystems, replace may not be atomic; shutil.move handles cross-device
        shutil.move(str(tmp), str(path))

    return f"Reduced {cur_tris} -> {len(simplified.triangles)} tris"

def main():
    args = parse_args()
    folder: Path = args.folder

    print(os.getcwd())
    print(glob.glob(str(folder / "*")))
    if not folder.exists() or not folder.is_dir():
        print(f"ERROR: Not a directory: {folder}", file=sys.stderr)
        sys.exit(1)

    stls = list(iter_stl_files(folder, args.pattern))
    if not stls:
        print("No STL files found.")
        return

    print(f"Found {len(stls)} STL file(s) under: {folder}")
    errors = 0
    for i, stl in enumerate(stls, 1):
        rel = stl.relative_to(folder)
        if args.dry_run:
            print(f"[{i}/{len(stls)}] (dry-run) Would simplify: {rel}")
            continue
        try:
            result = simplify_stl_inplace(
                stl,
                None if args.target_triangles is not None else args.reduction,
                args.target_triangles,
                args.min_tris
            )
            print(f"[{i}/{len(stls)}] {rel}: {result}")
        except Exception as e:
            errors += 1
            print(f"[{i}/{len(stls)}] {rel}: ERROR: {e}", file=sys.stderr)

    if errors:
        print(f"Completed with {errors} error(s).", file=sys.stderr)
    else:
        print("Completed.")

if __name__ == "__main__":
    main()
