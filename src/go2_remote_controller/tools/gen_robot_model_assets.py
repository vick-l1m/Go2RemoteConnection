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

Up-axis note: each .dae is a COLLADA <scene> whose node graph already carries
the transform that reconciles the file's raw <float_array> data (long axis on
Y, e.g. calf.dae spans ~0.27m in Y) with go2.urdf's joint origins (which
expect the calf's long axis on -Z: FL_calf_joint's child FL_foot_joint sits at
xyz="0 0 -0.213"). trimesh applies that node-graph transform automatically
when a .dae is flattened with force="mesh" -- loading calf.dae that way puts
its long axis on Z, range [-0.236, 0.031], matching the -0.213 foot offset
almost exactly, with no extra rotation needed.

Two earlier versions of this script got this wrong by calling
`trimesh.load(dae_path)` with no `force="mesh"`, which for a multi-node
COLLADA scene returns a Scene whose `.geometry` dict holds each mesh's raw,
untransformed vertices (the node-graph transform lives separately, in
`scene.graph`, and is invisible if you inspect `.geometry` directly). Both
versions then bolted on their own guessed corrective rotation (+90 or -90 deg
about X) to compensate for what looked like a missing transform -- but the
correct transform was already sitting in the scene graph, so either guess
just rotates an already-correctly-oriented mesh into a wrong one. This is
what produced a robot that loads and animates (joint math was never wrong)
but renders as a collapsed/scattered mesh instead of a standing quadruped.
The fix is to let trimesh do the flattening (`force="mesh"`) and export that
result as-is, applying no additional rotation at all.

Usage:
    pip install trimesh pygltflib pycollada
    python3 tools/gen_robot_model_assets.py \\
        --src ../../../src/go2_description \\
        --out app/static/robot_model
"""

import argparse
import math
import re
import shutil
from pathlib import Path

import trimesh


def convert_mesh(dae_path: Path, glb_path: Path) -> None:
    # force="mesh" flattens the .dae's COLLADA scene graph into the mesh's
    # already-correct world-space vertices (see the module docstring's
    # "Up-axis note") -- no additional rotation belongs here.
    mesh = trimesh.load(str(dae_path), force="mesh")
    glb_path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(glb_path), file_type="glb")


def camera_urdf_block(camera_yaml: Path, go2_sim: Path) -> str:
    """Links + fixed joints for the add-on RealSense D435i and its bracket.

    go2.urdf deliberately does not carry the D435i: it is an optional add-on, and
    in ROS the sim publishes its pose as a static TF with a separate
    ``camera_description`` (see the outer repo's camera_support.py). The web
    viewer has no TF at all -- it renders one URDF -- so here the camera has to
    be joined into the tree directly, with the pose read from the same
    camera.yaml the sim and the static TF use.

    Geometry comes from the outer repo's ``go2_sim.camera_geometry`` (stdlib
    only, imported by path), so the web viewer, RViz and the Isaac scene cannot
    drift apart. Note ``front_camera`` in go2.urdf is the Go2's OWN built-in
    camera on the head -- unrelated to this.
    """
    import sys

    import yaml

    sys.path.insert(0, str(go2_sim))
    from go2_sim import camera_geometry as cg   # noqa: E402

    cam = yaml.safe_load(camera_yaml.read_text(encoding="utf-8"))["camera"]
    t = cam.get("translation", [0.28, 0.0, 0.10])
    pitch = math.radians(float(cam.get("pitch_deg", 30.0)))
    yaw = math.radians(float(cam.get("yaw_deg", 0.0)))
    roll = math.radians(float(cam.get("roll_deg", 0.0)))
    link = cam.get("link_frame", "camera_link")
    housing = cam.get("housing", {}) or {}
    mount = cam.get("mount", {}) or {}

    # The URDF is rooted at base_link; camera.yaml's parent_frame ("base") is the
    # ROS/Isaac spelling of the same body frame, bridged by an identity TF there.
    parent = "base_link"

    cam_materials, cam_visuals = cg.camera_visuals_urdf(housing.get("color"))
    mnt_materials, mnt_visuals = cg.mount_visuals_urdf(t, mount.get("color"))

    block = ("\n  <!-- Add-on RealSense D435i + mount bracket (generated from "
             "camera.yaml by tools/gen_robot_model_assets.py) -->\n")
    block += cam_materials + mnt_materials
    block += f'  <link name="{link}">\n{cam_visuals}  </link>\n'
    block += (f'  <joint name="{link}_joint" type="fixed">\n'
              f'    <parent link="{parent}"/>\n'
              f'    <child link="{link}"/>\n'
              f'    <origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" '
              f'rpy="{roll:.6f} {pitch:.6f} {yaw:.6f}"/>\n'
              f'  </joint>\n')
    if mnt_visuals:
        block += f'  <link name="camera_mount">\n{mnt_visuals}  </link>\n'
        block += ('  <joint name="camera_mount_joint" type="fixed">\n'
                  f'    <parent link="{parent}"/>\n'
                  '    <child link="camera_mount"/>\n'
                  '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                  '  </joint>\n')
    return block


def rewrite_urdf(src_urdf: Path, out_urdf: Path,
                 camera_block: str = "") -> list[str]:
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

    if camera_block:
        # Insert before the closing tag rather than appending, so the document
        # stays well-formed whatever trailing whitespace the source URDF has.
        idx = new_text.rindex("</robot>")
        new_text = new_text[:idx] + camera_block + new_text[idx:]

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
    parser.add_argument(
        "--camera-yaml", default=None,
        help="camera.yaml to read the D435i pose from "
             "(default: <src>/../go2_bringup/config/camera.yaml)",
    )
    parser.add_argument(
        "--go2-sim", default=None,
        help="go2_sim package, for camera_geometry "
             "(default: <src>/../go2_sim)",
    )
    parser.add_argument(
        "--no-camera", action="store_true",
        help="omit the add-on D435i and its bracket from the generated URDF",
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

    camera_block = ""
    if not args.no_camera:
        camera_yaml = Path(args.camera_yaml).resolve() if args.camera_yaml else (
            src.parent / "go2_bringup" / "config" / "camera.yaml")
        go2_sim = Path(args.go2_sim).resolve() if args.go2_sim else (
            src.parent / "go2_sim")
        if camera_yaml.is_file() and (go2_sim / "go2_sim" / "camera_geometry.py").is_file():
            print(f"Adding D435i + bracket from {camera_yaml}")
            camera_block = camera_urdf_block(camera_yaml, go2_sim)
        else:
            print(f"[WARN] no camera.yaml/go2_sim beside {src}; "
                  "generating without the D435i (pass --no-camera to silence)")

    print(f"Rewriting URDF: {src_urdf} -> {out_urdf}")
    mesh_names = rewrite_urdf(src_urdf, out_urdf, camera_block)
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
