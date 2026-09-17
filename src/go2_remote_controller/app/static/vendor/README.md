# app/static/vendor/

Vendored third-party JS, loaded via a native `<script type="importmap">` (no
bundler — this app has none). Not fetched from a CDN: this viewer is meant to
be a reliable always-on operational display, and the robot's network segment
isn't guaranteed to reach the public internet.

Pinned versions (extracted 2026-09-17):
- `three@0.186.0` — `build/three.module.js`, plus from `examples/jsm/`:
  `controls/OrbitControls.js`, `loaders/GLTFLoader.js`,
  `utils/BufferGeometryUtils.js`, `utils/SkeletonUtils.js` (GLTFLoader's two
  transitive deps). Directory structure under `three/addons/` mirrors
  `examples/jsm/` exactly so their relative imports (`../utils/...js`) resolve
  unchanged.
- `urdf-loader@0.13.1` — `src/URDFLoader.js` (patched, see below) and
  `src/URDFClasses.js` (unmodified).

**`urdf-loader/URDFLoader.js` is locally patched**: the stock file statically
imports `STLLoader`/`ColladaLoader` and uses them in `defaultMeshLoader()`.
This app's `robot_model/` assets are exclusively `.glb`
(`tools/gen_robot_model_assets.py`) and always supplies its own `loadMeshCb`
(see `../robot_viewer.js`) that loads `.glb` via `GLTFLoader`, so the stock
STL/Collada path is dead weight — vendoring it would also pull in
`ColladaLoader.js`'s own dependency tree for zero benefit. The patch drops
those two imports and empties `defaultMeshLoader()` down to a warning (it's
only reached if `robot_viewer.js`'s `loadMeshCb` wiring is missing). See the
comment block at the top of that file for the full rationale.

## Re-vendoring / upgrading

```bash
mkdir /tmp/vendor_pack && cd /tmp/vendor_pack
npm pack three@<version>
npm pack urdf-loader@<version>
tar xzf three-<version>.tgz && mv package three-package
tar xzf urdf-loader-<version>.tgz && mv package urdf-loader-package
```

Then copy the same file set listed above into this directory, and re-apply
the `URDFLoader.js` patch (diff against the previous vendored copy in git to
see exactly what changed, since upstream may have moved lines around).
