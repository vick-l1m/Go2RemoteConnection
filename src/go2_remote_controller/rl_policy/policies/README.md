# Exported RL policies

Drop exported `.onnx` policies here and reference them from
[`../policies.json`](../policies.json) (the registry that populates the RL web
page's policy dropdown).

The baseline `flat` policy lives one level up as `../policy.onnx` for backward
compatibility; new policies go here and are referenced by a `policies/<name>.onnx`
path in the registry.

## Adding a policy

1. Export the trained checkpoint to ONNX (see `../README.md`) into this folder,
   e.g. `flat_custom.onnx`.
2. Add or edit its entry in `../policies.json`:
   - `id` — stable key (no spaces)
   - `name` / `model` — shown in the dropdown
   - `type` — `blind` (proprioceptive) or `camera` (needs a height scan)
   - `obs_dim`, `uses_heightmap`, `runnable`, `path`
3. Reload the web page. The dropdown reads the registry live — no server restart.
   Selecting a policy hot-swaps the onnx in the running controller (idle only).

Camera / height-scan policies (`uses_heightmap: true` or `runnable: false`) are
listed but cannot be engaged through the RL launcher yet: it does not run the
perception pipeline that produces the 187-ray height scan. See
`../../../../realsense_d435i_plan.md`.
