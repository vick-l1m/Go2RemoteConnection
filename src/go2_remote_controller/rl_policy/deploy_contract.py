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

**Per-term observation scales are part of that list.** Stock Isaac Lab's Go2 velocity
task leaves every observation term unscaled, so the first 48-dim policies deployed
correctly from raw sensor values and both nodes were written assuming "no
normalization". The ``unitree_rl_lab``-derived tasks (``go2-velocity``, ``go2-tap``,
``flat-dr``) do not: they carry ``scale=0.2`` on ``base_ang_vel`` and ``scale=0.05``
on ``joint_vel_rel``. Feeding those policies raw gyro and raw joint velocity puts the
observation 5x and 20x outside anything training ever showed them, and the actions
saturate — measured at ``|a|=28`` (a 7 rad joint-target offset) on ``flat-dr`` at
trotting joint speeds. So the scales are exposed here (:attr:`DeployContract.obs_scales`)
and applied by the builders, exactly as unitree_rl_lab's own C++ deploy stack does in
``ObservationTermCfg::add()``.

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
    "gait_phase",
    "posture_command",
)

# Isaac Lab and unitree_rl_lab spell three of those terms differently, and this repo
# trains tasks descending from both. The names above are the unitree_rl_lab spelling,
# which the obs builders key on; stock Isaac Lab's locomotion-velocity task names the
# same three terms ``joint_pos`` / ``joint_vel`` / ``actions`` (velocity_env_cfg.py's
# ObservationsCfg.PolicyCfg), while binding them to the identical mdp functions --
# ``mdp.joint_pos_rel``, ``mdp.joint_vel_rel``, ``mdp.last_action``.
#
# So every policy descending from the stock rough task -- which is the whole PERCEPTION
# lineage: camera-rough, stepfield-lane, stepfield-spec -- emitted a deploy.yaml naming
# terms this loader rejected, and was therefore unloadable by BOTH policy nodes. Not a
# subtle failure (it raises), but a total one, and it was invisible until a perception
# policy was first taken to the robot.
#
# Aliasing rather than renaming the terms upstream, because the contracts of policies
# ALREADY trained and promoted are on disk and cannot be regenerated without retraining.
# The alias is applied on load, so everything downstream -- obs_terms, obs_scales,
# obs_clips, the builders -- sees only the canonical name.
OBS_TERM_ALIASES = {
    "joint_pos": "joint_pos_rel",
    "joint_vel": "joint_vel_rel",
    "actions": "last_action",
}


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

    @property
    def uses_gait_phase(self) -> bool:
        return "gait_phase" in self.obs_terms

    @property
    def uses_posture_command(self) -> bool:
        """True for a posture policy (sit/stand), which is commanded by a discrete
        target rather than a velocity. Such a policy has no ``velocity_commands``
        term, so the joystick does not drive it."""
        return "posture_command" in self.obs_terms

    @property
    def gait_phase_period(self) -> float | None:
        """Seconds per full gait cycle, or None when the policy has no gait_phase.

        The period is the policy's clock: the (sin, cos) pair it was trained against
        advances at ``step_dt / period`` per control step, so a wrong period phase-shifts
        every gait-conditioned action without changing the observation width. Recorded
        under the term's own ``params`` by the exporter -- refuse a contract that declares
        the term without one rather than substituting a default.
        """
        if not self.uses_gait_phase:
            return None
        params = self._d["observations"]["gait_phase"].get("params") or {}
        period = params.get("period")
        if period is None:
            raise DeployContractError(
                f"{self.source}: observation 'gait_phase' has no params.period. "
                "The period is what the (sin, cos) clock is scaled by; running without it "
                "would phase-shift every action the policy takes."
            )
        period = float(period)
        if period <= 0.0:
            raise DeployContractError(
                f"{self.source}: observation 'gait_phase' has params.period={period}, "
                "which must be positive"
            )
        return period

    @property
    def command_ranges(self) -> tuple[list[float], list[float]] | None:
        """Per-axis ``(lows, highs)`` for (lin_vel_x, lin_vel_y, ang_vel_z), or None.

        The envelope the velocity command was *sampled from* in training, so it is also
        the envelope the joystick has to be clamped to. The Go2 tasks use +-1.0 on x and
        yaw but only +-0.4 laterally, so a single symmetric clip hands the policy a
        full-stick strafe 2.5x outside anything it ever saw. unitree_rl_lab's C++ deploy
        clamps each axis against exactly this node (``velocity_commands`` in
        ``observations.h``); this is the Python side of that.

        None when the contract records no ``base_velocity`` command, so the caller keeps
        whatever fallback it had rather than inventing an envelope.
        """
        ranges = ((self._d.get("commands") or {}).get("base_velocity") or {}).get("ranges")
        if not ranges:
            return None
        lows: list[float] = []
        highs: list[float] = []
        for axis in ("lin_vel_x", "lin_vel_y", "ang_vel_z"):
            pair = ranges.get(axis)
            if pair is None or len(pair) != 2:
                raise DeployContractError(
                    f"{self.source}: commands.base_velocity.ranges.{axis} must be a "
                    f"[min, max] pair, got {pair!r}"
                )
            lows.append(float(pair[0]))
            highs.append(float(pair[1]))
        return lows, highs

    @property
    def command_style(self) -> str | None:
        """How the ``(vx, vy, wz)`` command is meant to be *produced*, or None for the default.

        ``"forward_biased"`` means the policy was trained with no lateral channel at all:
        a world-frame heading target, and forward speed gated by the heading error still
        outstanding (``go2_training.mdp.ForwardBiasedVelocityCommand``). A node driving
        such a policy must convert the operator's lateral stick into a turn rather than
        pass it through -- see :mod:`forward_bias`.

        Announced by the trainer rather than inferred from a zero lateral range, because
        the two failure modes are not equivalent: guessing wrong here means the robot
        silently ignores half the joystick, with nothing in any log to say why.
        """
        style = ((self._d.get("commands") or {}).get("base_velocity") or {}).get("style")
        if style is None:
            return None
        style = str(style)
        if style not in ("forward_biased",):
            raise DeployContractError(
                f"{self.source}: commands.base_velocity.style is {style!r}, which this node "
                "does not know how to drive. Update the node, or run a policy it supports."
            )
        return style

    @property
    def is_forward_biased(self) -> bool:
        """True when the lateral stick must be converted to a turn, not forwarded."""
        return self.command_style == "forward_biased"

    @property
    def heading_stiffness(self) -> float:
        """Heading error -> yaw rate gain, as trained. Defaults to Isaac Lab's 1.0.

        Only meaningful for a forward-biased policy. Defaulted rather than required so a
        contract written before this key existed still loads.
        """
        raw = ((self._d.get("commands") or {}).get("base_velocity") or {}).get("heading_stiffness")
        if raw is None:
            return 1.0
        value = float(raw)
        if not value > 0.0:
            raise DeployContractError(
                f"{self.source}: commands.base_velocity.heading_stiffness must be positive, "
                f"got {value}"
            )
        return value

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

        # Map stock-Isaac-Lab term names onto the canonical ones before anything else
        # reads them (see OBS_TERM_ALIASES). Done here rather than at each use site so
        # obs_terms, obs_widths, obs_scales and obs_clips are all keyed consistently and
        # the builders never see an alias.
        obs = {OBS_TERM_ALIASES.get(k, k): v for k, v in obs.items()}
        if len(obs) != len(self._d["observations"]):
            raise DeployContractError(
                f"{self.source}: observation names collide once aliases are applied "
                f"({sorted(self._d['observations'])}). A contract may spell a term either "
                "way, but not both."
            )
        self.aliased_terms = {
            k: OBS_TERM_ALIASES[k] for k in self._d["observations"] if k in OBS_TERM_ALIASES
        }

        # Prefer the explicit order: YAML mapping order survives the exporter (which
        # dumps with sort_keys=False) but not a careless round-trip, and a reordered
        # observation is the exact silent failure this contract exists to prevent.
        declared = self._d.get("observation_order")
        if declared is not None:
            declared = [OBS_TERM_ALIASES.get(t, t) for t in declared]
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
                f"Known terms: {', '.join(KNOWN_OBS_TERMS)} "
                f"(also accepted, as aliases: {', '.join(sorted(OBS_TERM_ALIASES))})."
            )

        # Token the policy was trained to read as "camera could not see this cell".
        #
        # None only when the policy carries no height scan at all. A contract that
        # HAS a scan but no recorded value was exported before the field existed,
        # and that code hardcoded 0.0 -- so absent means 0.0, not "unknown". That
        # distinction matters: heightmap_node's fill has since moved to -1.0, so
        # treating absent as unknown would silently feed every pre-existing
        # perception policy a hole wherever it learnt "not seen", across roughly
        # half the scan. Inferring the legacy value makes that a loud mismatch.
        LEGACY_UNOBSERVED = 0.0
        hs = obs.get("height_scan")
        if hs is None:
            self.height_scan_unobserved = None
        else:
            raw_unobs = (hs.get("params") or {}).get("unobserved_value")
            self.height_scan_unobserved = (
                LEGACY_UNOBSERVED if raw_unobs is None else float(raw_unobs))

        self.obs_widths: dict[str, int] = {}
        # Frames of history per term (1 = no history). Isaac Lab stacks oldest ->
        # newest and flattens; the builder has to reproduce that exactly, so the
        # length is surfaced here rather than only folded into obs_widths below.
        self.obs_history: dict[str, int] = {}
        self.obs_scales: dict[str, list[float] | None] = {}
        self.obs_clips: dict[str, tuple[float, float] | None] = {}
        for name in self.obs_terms:
            term = obs[name]
            scale = term.get("scale")
            if scale is None:
                raise DeployContractError(f"{self.source}: observation {name!r} has no scale (cannot infer width)")
            scale = [float(s) for s in scale] if isinstance(scale, (list, tuple)) else [float(scale)]

            history = int(term.get("history_length", 1) or 1)
            self.obs_history[name] = history
            self.obs_widths[name] = len(scale) * history

            # None means identity, and the builders then skip the multiply entirely --
            # which keeps the stock-Isaac-Lab policies (every scale 1.0, because that
            # task sets no scales at all) byte-identical to how they ran before scales
            # were honoured here.
            self.obs_scales[name] = None if all(s == 1.0 for s in scale) else scale

            clip = term.get("clip")
            if clip is None:
                self.obs_clips[name] = None
            elif len(clip) == 2:
                self.obs_clips[name] = (float(clip[0]), float(clip[1]))
            else:
                raise DeployContractError(
                    f"{self.source}: observation {name!r} has clip={clip!r}; Isaac Lab "
                    "clips a whole term with one (min, max) pair"
                )

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
        scaled = [t for t, s in self.obs_scales.items() if s is not None]
        return (
            f"<DeployContract {self.obs_dim}-dim obs {self.obs_terms}, "
            f"{self.control_rate:g} Hz, action_scale={self.action_scale:g}, "
            f"obs_scaled={scaled or 'none'}, src={self.source}>"
        )
