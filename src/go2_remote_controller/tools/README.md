# tools/

Dev-only scripts. Not run by the deployed app.

## gen_robot_model_assets.py

Regenerates `app/static/robot_model/` (the URDF + meshes served to the browser
for the live 3D robot viewer) from the outer workspace's `go2_description`
package. That directory is **generated/vendored output** — never hand-edit it,
re-run this script instead.

Run whenever `go2_description`'s URDF or meshes change. Needs a dev checkout
that has both this repo and the outer `Go2_RL_workflow` workspace present
(the generated output itself has no such dependency at runtime).

```bash
pip install trimesh==5.1.0 pygltflib==1.16.5 pycollada
cd src/go2_remote_controller
python3 tools/gen_robot_model_assets.py \
    --src ../../../src/go2_description \
    --out app/static/robot_model
```

What it does:
- Converts the 7 `.dae` (Collada) meshes to binary `.glb` (~25MB -> ~7.6MB).
- Copies `go2.urdf`, rewriting `package://go2_description/meshes/X.dae` ->
  `./meshes/X.glb`.

**Up-axis correction**: the source `.dae` files declare `Z_UP`, matching
`go2.urdf`'s un-rotated joint origins. `trimesh`'s COLLADA loader normalizes
Z-up meshes to Y-up on import (a +90 deg rotation about X), which would
silently misalign every mesh from the URDF's joint frames. The script undoes
this (-90 deg about X) before export, so the `.glb` vertex data lands back at
the exact coordinates the URDF expects — verified by comparing a raw
`<float_array>` pulled directly from `base.dae`'s XML against the converted
mesh's bounds (identical to the printed decimal place). No corrective
rotation is needed anywhere else (viewer, URDF) as a result.
