#!/usr/bin/env python3
"""gen_robot_model_assets.py — vendor go2_description's URDF+meshes for the web viewer.

Converts the 7 Collada (.dae) meshes referenced by go2_description/urdf/go2.urdf
into binary glTF (.glb), and copies the URDF alongside them with its mesh refs
rewritten to point at the converted files. The output is a self-contained copy
served by the FastAPI app's static mount (app/static/robot_model/) — the web
app has no runtime dependency on the outer repo's go2_description package or
checkout layout.

Re-run this whenever go2_description's URDF or meshes change; the output
directory is generated/vendored, never hand-edited.

Up-axis note: the source .dae files declare <up_axis>Z_UP</up_axis>, matching
go2.urdf's joint origins (which apply no corrective rotation, since they were
authored assuming mesh vertices are already in the link's native Z-up frame).
trimesh's COLLADA importer normalizes Z-up meshes to Y-up on load (a +90 deg
rotation about X), which would silently misalign every mesh from the URDF's
joint frames if exported as-is. So after loading, we apply the inverse
rotation (-90 deg about X) to undo trimesh's normalization before exporting,
restoring the mesh vertices to the exact coordinates the URDF expects. This
was verified empirically: extracting a raw <float_array> from base.dae and
comparing its bounds directly against trimesh's loaded-then-corrected bounds
gives identical min/max per axis.

Usage:
    pip install trimesh pygltflib pycollada
    python3 tools/gen_robot_model_assets.py \\
        --src ../../../src/go2_description \\
        --out app/static/robot_model
"""

import argparse
import re
import shutil
from pathlib import Path

import numpy as np
import trimesh

# Undo trimesh's Z-up -> Y-up COLLADA normalization (a +90 deg rotation about
# X), restoring the exact vertex coordinates go2_description's meshes were
# authored in and that go2.urdf's joint origins assume.
_UNDO_ZUP_TO_YUP = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])


def convert_mesh(dae_path: Path, glb_path: Path) -> None:
    scene_or_mesh = trimesh.load(str(dae_path))
    scene_or_mesh.apply_transform(_UNDO_ZUP_TO_YUP)
    glb_path.parent.mkdir(parents=True, exist_ok=True)
    scene_or_mesh.export(str(glb_path), file_type="glb")


def rewrite_urdf(src_urdf: Path, out_urdf: Path) -> list[str]:
    text = src_urdf.read_text(encoding="utf-8")

    mesh_names = sorted(set(
        re.findall(r'package://go2_description/meshes/([\w.]+)\.dae', text)
    ))

    def _replace(m: re.Match) -> str:
        name = m.group(1)
        return f'filename="./meshes/{name}.glb"'

    new_text = re.sub(
        r'filename="package://go2_description/meshes/([\w.]+)\.dae"',
        _replace,
        text,
    )

    out_urdf.parent.mkdir(parents=True, exist_ok=True)
    out_urdf.write_text(new_text, encoding="utf-8")
    return mesh_names


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--src", required=True,
        help="Path to go2_description package (contains urdf/go2.urdf and meshes/)",
    )
    parser.add_argument(
        "--out", required=True,
        help="Output directory (e.g. app/static/robot_model)",
    )
    args = parser.parse_args()

    src = Path(args.src).resolve()
    out = Path(args.out).resolve()

    src_urdf = src / "urdf" / "go2.urdf"
    src_meshes = src / "meshes"
    out_urdf = out / "go2.urdf"
    out_meshes = out / "meshes"

    if not src_urdf.is_file():
        raise SystemExit(f"URDF not found: {src_urdf}")

    print(f"Rewriting URDF: {src_urdf} -> {out_urdf}")
    mesh_names = rewrite_urdf(src_urdf, out_urdf)
    print(f"  {len(mesh_names)} mesh references found: {mesh_names}")

    for name in mesh_names:
        dae_path = src_meshes / f"{name}.dae"
        glb_path = out_meshes / f"{name}.glb"
        if not dae_path.is_file():
            raise SystemExit(f"Mesh not found: {dae_path}")
        print(f"  Converting {dae_path.name} -> {glb_path.relative_to(out)}")
        convert_mesh(dae_path, glb_path)

    print(f"\nDone. Generated {out_urdf} + {len(mesh_names)} .glb meshes under {out_meshes}/")
    print("Commit this output as vendored/generated assets; re-run this script "
          "if go2_description's URDF or meshes change.")


if __name__ == "__main__":
    main()
