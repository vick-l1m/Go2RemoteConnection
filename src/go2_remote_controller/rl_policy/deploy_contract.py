"""Reader for ``deploy.yaml`` — the sim-to-real contract emitted at training time.

``training/go2_training/utils/export_deploy_cfg.py`` writes this file next to every
trained policy, reading the values off the live Isaac Lab env. This module is the
consumer side: it turns that file into the handful of numbers a policy node needs,
and refuses to guess when they are missing.

Why it refuses. ``docs/plans/sim_to_real_deployment_plan.md`` lists the values that
must match training exactly — joint order, action scale, the default pose, the
observation layout. Every one of them fails *silently* when wrong: the robot walks,
badly, and you go looking at the policy. Defaulting them is what makes that failure
mode possible, so a policy without a contract is treated as unloadable rather than
loaded on assumptions.

Dependency-free by design (``yaml`` + stdlib): imported by ``rl_policy_node`` here
and by ``Go2RemoteConnection``'s real-robot node, neither of which has Isaac Lab.

.. note::
   **This file is mirrored, byte for byte, at**
   ``Go2RemoteConnection/src/go2_remote_controller/rl_policy/deploy_contract.py``.
   The robot launcher runs its RL node as raw Python from the source tree and never
   sources this workspace's overlay -- the submodule is deployable standalone -- so the
   robot cannot import this package. ``src/go2_rl/test/test_deploy_contract.py`` fails
   if the two copies drift. Edit this one, then copy it across.
"""

from __future__ import annotations

import os

import yaml


class DeployContractError(RuntimeError):
    """Raised when the contract is missing, unreadable, or internally inconsistent."""


# Observation terms the builder knows how to produce, in the order Isaac Lab
# concatenates them. A contract naming anything else is a contract this node cannot
# honour, and saying so beats silently producing a differently-shaped vector.
KNOWN_OBS_TERMS = (
    "base_lin_vel",
    "base_ang_vel",
    "projected_gravity",
    "velocity_commands",
    "joint_pos_rel",
    "joint_vel_rel",
    "last_action",
    "height_scan",
)


class DeployContract:
    """The values a policy must be run with, as recorded when it was trained."""

    def __init__(self, data: dict, source: str):
        self.source = source
        self._d = data

        missing = [k for k in ("step_dt", "default_joint_pos", "observations", "actions") if k not in data]
        if missing:
            raise DeployContractError(f"{source}: missing required key(s): {', '.join(missing)}")

        self.step_dt: float = float(data["step_dt"])
        if not self.step_dt > 0:
            raise DeployContractError(f"{source}: step_dt must be positive, got {self.step_dt}")

        self.default_joint_pos: list[float] = [float(x) for x in data["default_joint_pos"]]
        self.joint_ids_map: list[int] | None = (
            [int(i) for i in data["joint_ids_map"]] if data.get("joint_ids_map") is not None else None
        )
        self.stiffness: list[float] | None = data.get("stiffness")
        self.damping: list[float] | None = data.get("damping")

        self._parse_action()
        self._parse_observations()

    # -- derived views the nodes actually consume -------------------------------

    @property
    def control_rate(self) -> float:
        """Hz. The inverse of the training control period — not a tunable."""
        return 1.0 / self.step_dt

    @property
    def include_base_lin_vel(self) -> bool:
        """True for the 48-dim layout, False for the hardware-observable 45-dim one."""
        return "base_lin_vel" in self.obs_terms

    @property
    def uses_height_scan(self) -> bool:
        return "height_scan" in self.obs_terms

    def _uniform_gain(self, values, what: str) -> float:
        """Collapse a per-joint gain list to the single value the nodes command.

        The Go2's lowcmd carries a kp/kd per motor, but every policy trained here uses
        one gain for all twelve. A contract with mixed gains would be quietly flattened,
        so refuse it instead -- the robot would hold a different stance than it trained
        with, which looks like a bad policy rather than a config error.
        """
        if values is None:
            raise DeployContractError(f"{self.source}: no {what} recorded")
        vals = [round(float(v), 9) for v in values]
        if not vals:
            raise DeployContractError(f"{self.source}: {what} is empty")
        if len(set(vals)) != 1:
            raise DeployContractError(
                f"{self.source}: per-joint {what} {sorted(set(vals))} is not supported by this node"
            )
        return vals[0]

    @property
    def kp(self) -> float:
        """Joint stiffness the policy was trained against."""
        return self._uniform_gain(self.stiffness, "stiffness")

    @property
    def kd(self) -> float:
        """Joint damping the policy was trained against."""
        return self._uniform_gain(self.damping, "damping")

    # -- parsing ----------------------------------------------------------------

    def _parse_action(self) -> None:
        actions = self._d["actions"]
        if len(actions) != 1:
            raise DeployContractError(
                f"{self.source}: expected exactly one action term, found {sorted(actions)}. "
                "The policy nodes drive joint positions only."
            )
        self.action_term, term = next(iter(actions.items()))
        scale = term.get("scale")
        if scale is None:
            raise DeployContractError(f"{self.source}: action term {self.action_term!r} has no scale")
        scales = [float(s) for s in scale] if isinstance(scale, (list, tuple)) else [float(scale)]
        # The nodes apply one scalar scale. Per-joint scales are legal in Isaac Lab but
        # would silently be collapsed here, so reject rather than mis-apply them.
        if len(set(scales)) != 1:
            raise DeployContractError(
                f"{self.source}: per-joint action scales {scales} are not supported by this node"
            )
        self.action_scale: float = scales[0]
        self.action_offset: list[float] | None = term.get("offset")
        self.action_clip = term.get("clip")

    def _parse_observations(self) -> None:
        obs = self._d["observations"]
        if not isinstance(obs, dict) or not obs:
            raise DeployContractError(f"{self.source}: 'observations' must be a non-empty mapping")

        # Prefer the explicit order: YAML mapping order survives the exporter (which
        # dumps with sort_keys=False) but not a careless round-trip, and a reordered
        # observation is the exact silent failure this contract exists to prevent.
        declared = self._d.get("observation_order")
        if declared is not None:
            if sorted(declared) != sorted(obs):
                raise DeployContractError(
                    f"{self.source}: observation_order {list(declared)} does not match "
                    f"the observations mapping {sorted(obs)}"
                )
            self.obs_terms: list[str] = list(declared)
            self.order_is_explicit = True
        else:
            self.obs_terms = list(obs)
            self.order_is_explicit = False

        unknown = [t for t in self.obs_terms if t not in KNOWN_OBS_TERMS]
        if unknown:
            raise DeployContractError(
                f"{self.source}: observation term(s) {unknown} are not produced by this node. "
                f"Known terms: {', '.join(KNOWN_OBS_TERMS)}."
            )

        self.obs_widths: dict[str, int] = {}
        for name in self.obs_terms:
            term = obs[name]
            scale = term.get("scale")
            if scale is None:
                raise DeployContractError(f"{self.source}: observation {name!r} has no scale (cannot infer width)")
            width = len(scale) if isinstance(scale, (list, tuple)) else 1
            self.obs_widths[name] = width * int(term.get("history_length", 1) or 1)

        self.obs_dim: int = sum(self.obs_widths.values())

    # -- loading ----------------------------------------------------------------

    @staticmethod
    def candidate_paths(policy_dir: str) -> list[str]:
        """Where a contract may sit: beside the policy, or in the run's ``params/``."""
        return [
            os.path.join(policy_dir, "deploy.yaml"),
            os.path.join(policy_dir, "params", "deploy.yaml"),
            os.path.join(os.path.dirname(policy_dir.rstrip("/")), "params", "deploy.yaml"),
        ]

    @classmethod
    def load(cls, policy_dir: str) -> DeployContract:
        """Load the contract for a policy directory, or raise :class:`DeployContractError`."""
        tried = cls.candidate_paths(policy_dir)
        for path in tried:
            if os.path.isfile(path):
                try:
                    with open(path) as fh:
                        data = yaml.safe_load(fh)
                except (OSError, yaml.YAMLError) as exc:
                    raise DeployContractError(f"{path}: could not be read ({exc})") from exc
                if not isinstance(data, dict):
                    raise DeployContractError(f"{path}: expected a YAML mapping, got {type(data).__name__}")
                return cls(data, source=path)

        raise DeployContractError(
            "No deploy.yaml found for this policy — looked in:\n  "
            + "\n  ".join(tried)
            + "\n\nThe contract records the joint order, action scale, default pose and "
            "observation layout the policy was trained with. Running without it means "
            "guessing them, and a wrong guess is silent. Re-export the policy with a "
            "current training/scripts/train.py, which writes params/deploy.yaml."
        )

    # -- cross-checking ---------------------------------------------------------

    def disagreements(self, *, control_rate=None, action_scale=None, include_base_lin_vel=None) -> list[str]:
        """Describe where supplied overrides differ from the contract. Empty means agreement."""
        out = []
        if control_rate is not None and abs(float(control_rate) - self.control_rate) > 1e-6:
            out.append(f"control_rate: param {control_rate} vs contract {self.control_rate:g}")
        if action_scale is not None and abs(float(action_scale) - self.action_scale) > 1e-9:
            out.append(f"action_scale: param {action_scale} vs contract {self.action_scale:g}")
        if include_base_lin_vel is not None and bool(include_base_lin_vel) != self.include_base_lin_vel:
            out.append(
                f"include_base_lin_vel: param {bool(include_base_lin_vel)} vs contract {self.include_base_lin_vel}"
            )
        return out

    def __repr__(self) -> str:
        return (
            f"<DeployContract {self.obs_dim}-dim obs {self.obs_terms}, "
            f"{self.control_rate:g} Hz, action_scale={self.action_scale:g}, src={self.source}>"
        )
