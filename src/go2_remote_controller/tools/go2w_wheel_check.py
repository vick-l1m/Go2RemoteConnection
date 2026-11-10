#!/usr/bin/env python3
"""Go2-W: which wheel sits on which lowcmd slot? Spin them one at a time and look.

WHY THIS EXISTS
    The RL node and training agree that the Go2-W's wheel axles are lowcmd/lowstate
    slots 12..15 in the order FR, FL, RR, RL (unitree_sdk2 example/go2w
    `unitree_legged_const.LegID`). Unitree's own examples spin all four wheels with the
    same velocity, so they never *show* that order, and a swapped pair would make a
    drive policy steer the wrong way with no error anywhere. This checks it on the real
    robot before any policy touches the wheels, and reports whether the wheels respond
    to velocity-mode commands at all.

WHAT IT DOES
    1. Releases the sport service (MotionSwitcherClient.ReleaseMode), as the RL node
       does on engage, keeping the 12 leg motors DAMPED (kp 0, kd 3) throughout.
    2. For each slot 12..15: commands dq = --speed rad/s (kp 0, kd --kd) for --seconds,
       then 0, and prints the slot's measured dq from lowstate. You say which wheel
       turned (FR/FL/RR/RL) and it tallies the mapping against the expected one.
    3. Restores the sport service on every exit path (Ctrl-C included).

SAFETY
    The legs are limp the whole time: put the robot ON A GANTRY / its back / blocks
    so the wheels are free and it cannot roll. Nothing commands the legs.

    ~/venvs/unitree_sdk2_python/bin/python tools/go2w_wheel_check.py --net eth0
    ... --dry-run      prints what it would send, never publishes lowcmd
    ... --no-prompt    skip the per-wheel question, just log the measured dq
"""

import argparse
import signal
import sys
import threading
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

PosStopF, VelStopF = 2.146e9, 16000.0
WHEEL_SLOTS = {12: "FR", 13: "FL", 14: "RR", 15: "RL"}   # the order the node + training assume
LEG_DAMP_KD = 3.0


class WheelCheck:
    def __init__(self, args):
        self.args = args
        self.low_state = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.target_dq = {s: 0.0 for s in WHEEL_SLOTS}
        self.crc = CRC()
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_cmd.head[0], self.low_cmd.head[1] = 0xFE, 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            m = self.low_cmd.motor_cmd[i]
            m.mode, m.q, m.kp, m.dq, m.kd, m.tau = 0x01, PosStopF, 0.0, VelStopF, 0.0, 0.0
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_lowstate, 10)
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(10.0)
        self.msc.Init()
        self.prev_mode = None

    def _on_lowstate(self, msg):
        with self.lock:
            self.low_state = msg

    def _write_loop(self):
        while not self.stop.is_set():
            for i in range(12):                      # legs: damped, never positioned
                m = self.low_cmd.motor_cmd[i]
                m.mode, m.q, m.kp, m.dq, m.kd, m.tau = 0x01, 0.0, 0.0, 0.0, LEG_DAMP_KD, 0.0
            with self.lock:
                targets = dict(self.target_dq)
            for s, dq in targets.items():            # wheels: velocity mode
                m = self.low_cmd.motor_cmd[s]
                m.mode, m.q, m.kp, m.dq, m.kd, m.tau = 0x01, 0.0, 0.0, float(dq), self.args.kd, 0.0
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            if not self.args.dry_run:
                self.pub.Write(self.low_cmd)
            time.sleep(0.002)

    def release_sport(self):
        _st, mode = self.msc.CheckMode()
        self.prev_mode = (mode or {}).get("name") or "normal"
        print(f"sport service mode: {mode}; releasing it (will restore '{self.prev_mode}')")
        if self.args.dry_run:
            return
        for _ in range(10):
            if not mode or not mode.get("name"):
                break
            self.msc.ReleaseMode()
            time.sleep(0.5)
            _st, mode = self.msc.CheckMode()
        print(f"released (mode now: {mode})")

    def restore_sport(self):
        if self.args.dry_run:
            return
        print(f"restoring sport service '{self.prev_mode}' ...")
        for _ in range(6):
            self.msc.SelectMode(self.prev_mode)
            time.sleep(1.0)
            _st, mode = self.msc.CheckMode()
            if mode and mode.get("name"):
                print(f"sport service back: {mode}")
                return
        print("WARNING: sport service did not come back -- use the remote (L2+A) or power-cycle")

    def measured_dq(self, slot):
        with self.lock:
            ls = self.low_state
        return None if ls is None else float(ls.motor_state[slot].dq)

    def run(self):
        for _ in range(50):
            if self.low_state is not None:
                break
            time.sleep(0.1)
        if self.low_state is None:
            print("no rt/lowstate -- is --net right?")
            return 1
        self.release_sport()
        t = threading.Thread(target=self._write_loop, daemon=True)
        t.start()
        time.sleep(0.5)
        seen = {}
        try:
            for slot, expect in WHEEL_SLOTS.items():
                print(f"\nslot {slot}: dq = {self.args.speed:+.1f} rad/s for {self.args.seconds:.1f} s "
                      f"(expected wheel: {expect})")
                with self.lock:
                    self.target_dq[slot] = self.args.speed
                t0 = time.monotonic()
                peak = 0.0
                while time.monotonic() - t0 < self.args.seconds:
                    dq = self.measured_dq(slot)
                    if dq is not None:
                        peak = max(peak, abs(dq))
                    time.sleep(0.05)
                with self.lock:
                    self.target_dq[slot] = 0.0
                time.sleep(0.5)
                print(f"  measured |dq| peak on slot {slot}: {peak:.2f} rad/s"
                      + ("  <-- did not move; velocity mode not accepted?" if peak < 0.2 else ""))
                if self.args.no_prompt:
                    continue
                ans = input("  which wheel turned? [FR/FL/RR/RL, or - if none]: ").strip().upper()
                seen[slot] = ans
        finally:
            with self.lock:
                for s in self.target_dq:
                    self.target_dq[s] = 0.0
            time.sleep(0.3)
            self.stop.set()
            t.join(timeout=1.0)
            self.restore_sport()
        if seen:
            ok = all(seen.get(s) == w for s, w in WHEEL_SLOTS.items())
            print("\nslot -> wheel:", {s: seen.get(s, "?") for s in WHEEL_SLOTS},
                  "expected", WHEEL_SLOTS)
            print("[OK] wheel order matches SDK_WHEEL_JOINTS / GO2W_WHEEL_JOINT_SDK_NAMES" if ok
                  else "[FAIL] wheel order differs -- fix SDK_WHEEL_JOINTS in go2_rl_policy_node.py "
                       "AND GO2W_WHEEL_JOINT_SDK_NAMES in training/go2_training/assets/joint_order.py, "
                       "then retrain/re-export (joint_ids_map is derived from those names)")
            return 0 if ok else 2
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--net", default="", help="DDS network interface to the robot (e.g. eth0)")
    ap.add_argument("--speed", type=float, default=2.0, help="wheel velocity target, rad/s")
    ap.add_argument("--kd", type=float, default=1.0, help="wheel velocity-mode damping gain")
    ap.add_argument("--seconds", type=float, default=2.0, help="spin time per wheel")
    ap.add_argument("--dry-run", action="store_true", help="never publish lowcmd or release sport")
    ap.add_argument("--no-prompt", action="store_true", help="do not ask which wheel turned")
    args = ap.parse_args()
    print("WARNING: legs go LIMP while this runs. Robot on a gantry / wheels free. E-stop in hand.")
    if not args.dry_run:
        input("Press Enter to continue...")
    ChannelFactoryInitialize(0, args.net) if args.net else ChannelFactoryInitialize(0)
    chk = WheelCheck(args)
    signal.signal(signal.SIGINT, lambda *_: chk.stop.set())
    return chk.run()


if __name__ == "__main__":
    sys.exit(main())
