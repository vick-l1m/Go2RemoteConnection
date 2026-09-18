#!/usr/bin/env python3
"""Does the Go2's odometry survive releasing the sport service?

WHY THIS EXISTS
    An accumulated elevation map needs the robot's pose *while the RL policy is
    driving rt/lowcmd*. But /sportmodestate -- the only pose source on this robot
    (see go2_remote_viz/recording/sportmodestate_to_tf.py) -- is published BY the
    high-level sport service, and go2_rl_policy_node RELEASES that service on
    engage. So the map may have no pose source at all for the entire time it
    matters. That is a design-blocking question, and it is cheap to measure.

    Note --dry_run on the policy node does NOT answer this: the release is gated
    behind `if self.handover and not self.dry_run`, so a dry run leaves sport in
    charge and the topic keeps publishing for trivial reasons.

WHAT IT MEASURES
    Before and after MotionSwitcherClient.ReleaseMode(), for /sportmodestate:
      * message rate (Hz)
      * whether the publisher's own stamp is still ADVANCING -- a frozen stamp at
        a healthy rate means a stale cached message, not a live estimator
      * reported position, so an optional manual-move phase can show whether the
        estimator is still INTEGRATING or merely alive

SAFETY
    The robot goes limp for a moment when the sport service is released, so this
    crouches it first (StandDown), exactly as the policy node does. Put the robot
    on the floor with clear space. Nothing is ever commanded to move. The sport
    service is restored on every exit path, including Ctrl-C and exceptions.

RESULT (measured 2026-09-18, unitree-jetson-payload, sport mode "mcf")
    Every onboard localisation source stops when the sport service is released:

      rt/sportmodestate    296.9 Hz  ->  no messages at all
      /utlidar/robot_odom  149.5 Hz  ->  9.580 s gap
      /utlidar/cloud_base   14.7 Hz  ->  9.597 s gap

    Only rt/lowstate (joints, IMU, foot force) and the payload D435i survive --
    payload USB sensors are independent of the motion switcher, Go2-service ones
    are not. So an odom-frame elevation map is unavailable on this platform
    without first writing a state estimator from rt/lowstate alone, which is why
    the design moved to a recurrent policy instead.

    Full write-up: docs/measurements/go2_localisation_while_engaged.md (outer repo).

USAGE (on the robot)
    python3 check_odom_while_released.py --net eth0
    python3 check_odom_while_released.py --net eth0 --move-test
"""

import argparse
import statistics
import sys
import threading
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

TOPIC = "rt/sportmodestate"


class Watcher:
    """Collects arrival times, publisher stamps and positions off the topic."""

    def __init__(self):
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self.arrivals = []
            self.stamps = []
            self.positions = []

    def on_msg(self, msg):
        # Publisher stamp, not arrival time: the two diverge exactly in the failure
        # mode this is looking for (DDS still handing out a cached sample).
        stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        with self._lock:
            self.arrivals.append(time.monotonic())
            self.stamps.append(stamp)
            self.positions.append(tuple(float(v) for v in msg.position))

    def snapshot(self):
        with self._lock:
            return list(self.arrivals), list(self.stamps), list(self.positions)


def summarise(label, watcher, window_s):
    arrivals, stamps, positions = watcher.snapshot()
    n = len(arrivals)
    print(f"\n--- {label} ---")
    if n == 0:
        print("  NO MESSAGES AT ALL -- no pose source while in this state.")
        return {"n": 0, "hz": 0.0, "stamp_advancing": False, "pos": None}

    hz = n / window_s
    stamp_span = stamps[-1] - stamps[0]
    # "Advancing" needs to be a real fraction of the window, not just non-zero:
    # a couple of late samples can give a tiny positive span while frozen.
    advancing = stamp_span > 0.5 * window_s
    jitter = (statistics.stdev([b - a for a, b in zip(arrivals, arrivals[1:])])
              if n > 2 else 0.0)

    print(f"  messages           : {n} in {window_s:.1f} s  ->  {hz:.1f} Hz "
          f"(inter-arrival sd {jitter * 1000:.1f} ms)")
    print(f"  publisher stamp    : spans {stamp_span:.2f} s  ->  "
          f"{'ADVANCING (live estimator)' if advancing else 'FROZEN (stale sample!)'}")
    print(f"  position first/last: {positions[0]}  /  {positions[-1]}")
    return {"n": n, "hz": hz, "stamp_advancing": advancing, "pos": positions[-1]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--net", default="", help="DDS interface to the robot (e.g. eth0)")
    ap.add_argument("--window", type=float, default=8.0,
                    help="seconds to sample in each phase (default 8)")
    ap.add_argument("--hold", type=float, default=0.0, metavar="SECONDS",
                    help="release the sport service, hold for SECONDS, then restore -- "
                         "without sampling rt/sportmodestate. Use this to probe OTHER "
                         "pose sources from another terminal while the robot is in the "
                         "state the RL policy actually runs in, e.g. "
                         "`ros2 topic hz /utlidar/robot_odom`. The L1 lidar's "
                         "localisation is a separate stack from the sport service, so "
                         "it may survive a release that kills /sportmodestate.")
    ap.add_argument("--move-test", action="store_true",
                    help="after release, pause so you can slide the robot ~1 m by hand; "
                         "reports whether the pose followed (alive != integrating)")
    args = ap.parse_args()

    if args.net:
        ChannelFactoryInitialize(0, args.net)
    else:
        ChannelFactoryInitialize(0)

    watcher = Watcher()
    sub = ChannelSubscriber(TOPIC, SportModeState_)
    sub.Init(watcher.on_msg, 10)

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    sport = SportClient()
    sport.SetTimeout(5.0)
    sport.Init()

    _status, mode = msc.CheckMode()
    prev = (mode or {}).get("name") or "normal"
    print(f"sport service currently: {mode}  (will restore '{prev}')")

    released = False
    try:
        if args.hold > 0.0:
            # Probe mode: get the robot into the released state and stay there, so
            # ROS 2 tooling can measure any topic without this script needing to know
            # its DDS type.
            print("\ncrouching (StandDown) before release...")
            sport.StopMove()
            sport.StandDown()
            time.sleep(2.0)
            print("releasing sport service...")
            for _ in range(10):
                _status, mode = msc.CheckMode()
                if not mode or not mode.get("name"):
                    break
                msc.ReleaseMode()
                time.sleep(0.5)
            released = True
            _status, mode = msc.CheckMode()
            print(f"sport service RELEASED (mode now: {mode})")
            print(f"\n>>> Probe other pose sources NOW, for {args.hold:.0f} s. In another "
                  f"terminal:")
            for topic in ("/utlidar/robot_odom", "/utlidar/robot_pose",
                          "/lf/sportmodestate", "/odommodestate"):
                print(f"      ros2 topic hz {topic}")
            print("    A live rate is not enough -- also echo it and check the values "
                  "CHANGE\n    when you slide the robot by hand, or it is a frozen "
                  "cached sample.\n")
            time.sleep(args.hold)
            print("hold elapsed.")
            return 0

        # --- Phase 1: sport service owning the robot ---
        print(f"\nsampling {TOPIC} for {args.window:.0f} s with sport ACTIVE...")
        watcher.reset()
        time.sleep(args.window)
        before = summarise("SPORT ACTIVE (baseline)", watcher, args.window)

        # --- Crouch, then release. Same order the policy node uses. ---
        print("\ncrouching (StandDown) before release, so the robot does not fall limp "
              "from standing...")
        sport.StopMove()
        sport.StandDown()
        time.sleep(2.0)

        print("releasing sport service...")
        for _ in range(10):
            _status, mode = msc.CheckMode()
            if not mode or not mode.get("name"):
                break
            msc.ReleaseMode()
            time.sleep(0.5)
        released = True
        _status, mode = msc.CheckMode()
        print(f"sport service released (mode now: {mode})")

        # --- Phase 2: this is the state the RL policy runs in ---
        print(f"\nsampling {TOPIC} for {args.window:.0f} s with sport RELEASED...")
        watcher.reset()
        time.sleep(args.window)
        after = summarise("SPORT RELEASED (what the RL policy sees)", watcher, args.window)

        moved = None
        if args.move_test and after["n"] > 0:
            print("\nMOVE TEST: slide or carry the robot about 1 m forward, then press Enter.")
            watcher.reset()
            input()
            moved = summarise("SPORT RELEASED, after a manual ~1 m move",
                              watcher, max(args.window, 1.0))
            if moved["n"] > 0 and after["pos"] is not None:
                d = [b - a for a, b in zip(after["pos"], moved["pos"])]
                dist = sum(v * v for v in d) ** 0.5
                print(f"  position delta     : {[round(v, 3) for v in d]}  "
                      f"|d| = {dist:.3f} m")
                print(f"  -> estimator is {'INTEGRATING' if dist > 0.2 else 'NOT integrating'} "
                      f"while released")

        # --- Verdict ---
        print("\n================ VERDICT ================")
        print(f"rate: {before['hz']:.1f} Hz active  ->  {after['hz']:.1f} Hz released")
        if after["n"] == 0 or not after["stamp_advancing"]:
            print("NO usable odometry while the sport service is released.")
            print("An accumulated elevation map therefore needs its own state")
            print("estimator (leg odometry + IMU from LowState) BEFORE the map is")
            print("worth building -- and on a stepfield with slipping feet that is")
            print("a substantial sub-project which caps the map's accuracy.")
        elif moved is not None and moved["n"] > 0:
            print("Odometry survives the release AND keeps integrating.")
            print("Next: measure DRIFT, which is what actually bounds map quality.")
            print("Walk a tape-measured straight line (say 5 m) under the RL policy")
            print("and compare the reported displacement against the tape.")
        else:
            print("Odometry survives the release (live, advancing stamps).")
            print("Re-run with --move-test to confirm it still INTEGRATES: a live")
            print("estimator that does not integrate is useless for a map.")
        print("=========================================")

    finally:
        if released:
            print(f"\nrestoring sport service ('{prev}')...")
            for _ in range(10):
                msc.SelectMode(prev)
                time.sleep(0.5)
                _status, mode = msc.CheckMode()
                if mode and mode.get("name"):
                    break
            print(f"sport service restored (mode now: {mode})")
            try:
                sport.BalanceStand()
            except Exception as exc:                       # noqa: BLE001
                print(f"BalanceStand failed ({exc}); the robot is crouched but held.")
        else:
            print("\nsport service was never released; nothing to restore.")


if __name__ == "__main__":
    sys.exit(main())
