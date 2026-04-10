#!/usr/bin/env python3
import argparse
import logging
import math
import select
import signal
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PIPER_SDK_ROOT = Path("/home/sivan/whole_body/piper_sdk")

for path in (REPO_ROOT, PIPER_SDK_ROOT):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

import lcm
import numpy as np
from piper_sdk import C_PiperInterface_V2

from go2_deployment.go2_gym.go2_arx_deploy.lcm_types.gripper_lcmt import gripper_lcmt
from go2_deployment.go2_gym.go2_arx_deploy.lcm_types.leg_control_data_lcmt import leg_control_data_lcmt
from go2_deployment.go2_gym.go2_arx_deploy.lcm_types.pd_tau_targets_lcmt import pd_tau_targets_lcmt


LOGGER = logging.getLogger("piper_bridge")
LCM_URL = "udpm://239.255.76.67:7314?ttl=255"
PIPER_ARM_FEEDBACK_CHANNEL = "piper_arm_feedback"
RAD_TO_PIPER_UNITS = 180000.0 / math.pi
PIPER_UNITS_TO_RAD = math.pi / 180000.0
M_TO_PIPER_GRIPPER_UNITS = 1_000_000.0
PIPER_GRIPPER_UNITS_TO_M = 1.0 / M_TO_PIPER_GRIPPER_UNITS


class PiperBridge:
    def __init__(self, args):
        self.args = args
        self.shutdown = False
        self.lc = lcm.LCM(args.lcm_url)

        self.command_msg = pd_tau_targets_lcmt()
        self.have_arm_command = False

        self.command_joint_targets = np.zeros(6, dtype=np.float64)
        self.command_gripper_m = 0.0
        self.have_gripper_command = False

        self.latest_joint_feedback = np.zeros(6, dtype=np.float64)
        self.latest_gripper_feedback_m = 0.0
        self.received_joint_feedback = False

        self.feedback_msg = leg_control_data_lcmt()
        self.feedback_channel = args.feedback_channel
        self.last_debug_log_time = 0.0

        self.lc.subscribe("pd_plustau_targets", self._handle_arm_command)
        self.lc.subscribe("gripper_command", self._handle_gripper_command)

        self.piper = C_PiperInterface_V2(args.can_name)

    def _handle_arm_command(self, channel, data):
        del channel
        msg = pd_tau_targets_lcmt.decode(data)
        self.command_msg = msg
        self.command_joint_targets = np.asarray(msg.q_arm_des[:6], dtype=np.float64)
        self.have_arm_command = True

        # Keep a fallback path for senders that still stuff the gripper target into q_arm_des[6].
        if not self.have_gripper_command and len(msg.q_arm_des) >= 7:
            self.command_gripper_m = self._input_gripper_to_meters(msg.q_arm_des[6])

    def _handle_gripper_command(self, channel, data):
        del channel
        msg = gripper_lcmt.decode(data)
        self.command_gripper_m = self._input_gripper_to_meters(msg.gripper_cmd)
        self.have_gripper_command = True

    def _input_gripper_to_meters(self, value):
        value = float(value)
        normalized = np.clip(value / self.args.gripper_input_max, 0.0, 1.0)
        return normalized * self.args.gripper_open_m

    def _meters_to_input_gripper(self, meters):
        meters = float(np.clip(meters, 0.0, self.args.gripper_open_m))
        return meters / self.args.gripper_open_m * self.args.gripper_input_max

    def _rad_to_joint_units(self, joints_rad):
        return [int(round(float(joint) * RAD_TO_PIPER_UNITS)) for joint in joints_rad]

    def _joint_units_to_rad(self, joint_msg):
        joint_state = joint_msg.joint_state
        return np.array([
            joint_state.joint_1,
            joint_state.joint_2,
            joint_state.joint_3,
            joint_state.joint_4,
            joint_state.joint_5,
            joint_state.joint_6,
        ], dtype=np.float64) * PIPER_UNITS_TO_RAD

    def connect(self):
        LOGGER.info("Connecting to Piper on %s", self.args.can_name)
        self.piper.ConnectPort()

        deadline = time.monotonic() + self.args.enable_timeout
        while not self.shutdown and time.monotonic() < deadline:
            if self.piper.EnablePiper():
                LOGGER.info("Piper enabled")
                break
            time.sleep(0.01)
        else:
            raise RuntimeError(f"Failed to enable Piper within {self.args.enable_timeout:.1f}s")

        # Clear gripper faults once, then switch to enabled state.
        self.piper.GripperCtrl(0, self.args.gripper_effort, 0x02, 0)
        time.sleep(0.05)
        self.piper.GripperCtrl(0, self.args.gripper_effort, 0x01, 0)
        self.piper.ModeCtrl(0x01, 0x01, self.args.move_speed_rate, 0x00)

    def disconnect(self):
        if hasattr(self.piper, "DisconnectPort"):
            try:
                self.piper.DisconnectPort()
            except Exception as exc:  # pragma: no cover - best-effort shutdown
                LOGGER.warning("DisconnectPort failed: %s", exc)

    def _refresh_feedback(self):
        joint_msg = self.piper.GetArmJointMsgs()
        gripper_msg = self.piper.GetArmGripperMsgs()

        self.latest_joint_feedback = self._joint_units_to_rad(joint_msg)
        self.latest_gripper_feedback_m = (
            float(gripper_msg.gripper_state.grippers_angle) * PIPER_GRIPPER_UNITS_TO_M
        )

        if not self.received_joint_feedback:
            self.received_joint_feedback = True
            LOGGER.info("Received first Piper joint feedback: %s", np.round(self.latest_joint_feedback, 3))

        if not self.have_arm_command:
            # Hold the current pose until the first LCM command arrives.
            self.command_joint_targets = self.latest_joint_feedback.copy()
        if not self.have_gripper_command:
            self.command_gripper_m = self.latest_gripper_feedback_m

    def _send_commands(self):
        self.piper.MotionCtrl_2(0x01, 0x01, self.args.move_speed_rate, 0x00)

        joint_units = self._rad_to_joint_units(self.command_joint_targets)
        self.piper.JointCtrl(*joint_units)

        gripper_units = int(round(max(0.0, self.command_gripper_m) * M_TO_PIPER_GRIPPER_UNITS))
        self.piper.GripperCtrl(gripper_units, self.args.gripper_effort, 0x01, 0)

    def _publish_feedback(self):
        feedback = self.feedback_msg
        feedback.timestamp_us = int(time.time() * 1e6)
        feedback.q_arm[:6] = self.latest_joint_feedback.astype(np.float32)
        feedback.q_arm[6] = float(self._meters_to_input_gripper(self.latest_gripper_feedback_m))
        self.lc.publish(self.feedback_channel, feedback.encode())

    def _maybe_log_debug(self):
        if self.args.log_interval <= 0:
            return

        now = time.monotonic()
        if now - self.last_debug_log_time < self.args.log_interval:
            return

        self.last_debug_log_time = now
        LOGGER.info(
            "cmd_j(rad)=%s fb_j(rad)=%s cmd_g=%.3f/%.3fm fb_g=%.3f/%.3fm",
            np.array2string(np.round(self.command_joint_targets, 3), precision=3, separator=", "),
            np.array2string(np.round(self.latest_joint_feedback, 3), precision=3, separator=", "),
            self._meters_to_input_gripper(self.command_gripper_m),
            self.command_gripper_m,
            self._meters_to_input_gripper(self.latest_gripper_feedback_m),
            self.latest_gripper_feedback_m,
        )

    def step(self):
        self._refresh_feedback()
        self._send_commands()
        self._publish_feedback()
        self._maybe_log_debug()

    def run(self):
        self.connect()
        period = 1.0 / self.args.control_rate_hz
        next_tick = time.monotonic()

        while not self.shutdown:
            timeout = max(0.0, next_tick - time.monotonic())
            rfds, _, _ = select.select([self.lc.fileno()], [], [], timeout)
            if rfds:
                self.lc.handle()

            now = time.monotonic()
            if now < next_tick:
                continue

            try:
                self.step()
            except Exception as exc:
                LOGGER.exception("Piper bridge step failed: %s", exc)
                time.sleep(0.1)

            next_tick += period
            if now - next_tick > period:
                next_tick = now + period


def build_argparser():
    parser = argparse.ArgumentParser(description="Bridge LCM arm commands to a Piper arm")
    parser.add_argument("--can-name", default="can0", help="Piper CAN interface name")
    parser.add_argument("--lcm-url", default=LCM_URL, help="LCM multicast URL")
    parser.add_argument(
        "--feedback-channel",
        default=PIPER_ARM_FEEDBACK_CHANNEL,
        help="LCM channel used for Piper arm feedback",
    )
    parser.add_argument(
        "--control-rate-hz",
        type=float,
        default=200.0,
        help="Command/feedback loop rate",
    )
    parser.add_argument(
        "--move-speed-rate",
        type=int,
        default=30,
        help="Piper MOVE J speed percentage passed to MotionCtrl_2",
    )
    parser.add_argument(
        "--gripper-effort",
        type=int,
        default=1000,
        help="Piper gripper effort in 0.001 N*m",
    )
    parser.add_argument(
        "--gripper-input-max",
        type=float,
        default=4.5,
        help="Current upstream gripper command is assumed to span [0, this value]",
    )
    parser.add_argument(
        "--gripper-open-m",
        type=float,
        default=0.08,
        help="Physical full-open stroke of the Piper gripper in meters",
    )
    parser.add_argument(
        "--enable-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for EnablePiper()",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=0.5,
        help="Seconds between debug logs; set <= 0 to disable",
    )
    return parser


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )
    args = build_argparser().parse_args()
    bridge = PiperBridge(args)

    def _signal_handler(sig, frame):
        del sig, frame
        bridge.shutdown = True

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        bridge.run()
    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main()
