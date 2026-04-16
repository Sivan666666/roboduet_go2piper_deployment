#!/usr/bin/env python3
import argparse
import os
import select
import signal
import sys
import termios
import time
import tty

import lcm
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from go2_arx_deploy.lcm_types.gripper_lcmt import gripper_lcmt
from go2_arx_deploy.lcm_types.rc_command_lcmt import rc_command_lcmt
from go2_arx_deploy.lcm_types.vr_command_lcmt import vr_command_lcmt


LCM_URL = "udpm://239.255.76.67:7314?ttl=255"
HOME_LPY = np.array([0.5, 0.2, 0.0], dtype=np.float64)
HOME_RPY = np.array([0.0, 0.0, 0.0], dtype=np.float64)
HOME_GRIPPER = 4.5

BODY_STEP = 0.1
ARM_L_STEP = 0.05
ARM_ANGLE_STEP = 0.1
GRIPPER_STEP = 0.3

BODY_LIMITS = {
    "vx": (-1.0, 1.0),
    "vy": (-0.5, 0.5),
    "yaw": (-1.0, 1.0),
}
ARM_LIMITS = {
    "l": (0.3, 0.8),
    "p": (-np.pi / 3.0, np.pi / 3.0),
    "y": (-np.pi / 2.0, np.pi / 2.0),
    "roll": (-np.pi * 0.45, np.pi * 0.45),
    "pitch": (-1.5, 1.5),
    "yaw": (-1.4, 1.4),
}
GRIPPER_LIMITS = (0.0, 4.5)

HELP_TEXT = """
Keyboard Teleop For Deployment
Use terminal focus and keep NumLock enabled if you use the numpad.

Base commands:
  8 / 5 : move forward / backward
  4 / 6 : move left / right
  7 / 9 : turn left / right
  0     : zero base velocity

Arm Cartesian commands:
  I / K : arm forward / backward
  U / O : arm up / down
  J / L : arm left / right

Arm orientation commands:
  W / S : pitch down / up
  A / D : roll left / right
  Q / E : yaw left / right

Gripper:
  N / M : open / close

Utility:
  R     : reset base + arm + gripper to home
  H     : print help
  X     : exit
  Ctrl-C: exit
""".strip()


def lpy_to_local_xyz(lpy):
    l, pitch, yaw = np.asarray(lpy, dtype=np.float64)
    x = l * np.cos(pitch) * np.cos(yaw)
    y = l * np.cos(pitch) * np.sin(yaw)
    z = l * np.sin(pitch)
    return np.array([x, y, z], dtype=np.float64)


def clamp(value, bounds):
    lower, upper = bounds
    return float(np.clip(value, lower, upper))


class RawKeyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s):
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None
        char = os.read(self.fd, 1).decode("utf-8", errors="ignore")
        if char == "\x1b":
            return "ESC"
        return char


class KeyboardTeleopState:
    def __init__(self):
        self.body = np.zeros(3, dtype=np.float64)
        self.arm_lpy = HOME_LPY.copy()
        self.arm_rpy = HOME_RPY.copy()
        self.gripper = float(HOME_GRIPPER)
        self.home_xyz = lpy_to_local_xyz(HOME_LPY)

    def reset(self):
        self.body[:] = 0.0
        self.arm_lpy[:] = HOME_LPY
        self.arm_rpy[:] = HOME_RPY
        self.gripper = float(HOME_GRIPPER)

    def zero_base(self):
        self.body[:] = 0.0

    def apply_key(self, key):
        changed = False
        key = key.lower()

        if key == "8":
            self.body[0] = clamp(self.body[0] + BODY_STEP, BODY_LIMITS["vx"])
            changed = True
        elif key == "5":
            self.body[0] = clamp(self.body[0] - BODY_STEP, BODY_LIMITS["vx"])
            changed = True
        elif key == "4":
            self.body[1] = clamp(self.body[1] + BODY_STEP, BODY_LIMITS["vy"])
            changed = True
        elif key == "6":
            self.body[1] = clamp(self.body[1] - BODY_STEP, BODY_LIMITS["vy"])
            changed = True
        elif key == "7":
            self.body[2] = clamp(self.body[2] + BODY_STEP, BODY_LIMITS["yaw"])
            changed = True
        elif key == "9":
            self.body[2] = clamp(self.body[2] - BODY_STEP, BODY_LIMITS["yaw"])
            changed = True
        elif key == "0":
            self.zero_base()
            changed = True

        elif key == "i":
            self.arm_lpy[0] = clamp(self.arm_lpy[0] + ARM_L_STEP, ARM_LIMITS["l"])
            changed = True
        elif key == "k":
            self.arm_lpy[0] = clamp(self.arm_lpy[0] - ARM_L_STEP, ARM_LIMITS["l"])
            changed = True
        elif key == "u":
            self.arm_lpy[1] = clamp(self.arm_lpy[1] + ARM_ANGLE_STEP, ARM_LIMITS["p"])
            changed = True
        elif key == "o":
            self.arm_lpy[1] = clamp(self.arm_lpy[1] - ARM_ANGLE_STEP, ARM_LIMITS["p"])
            changed = True
        elif key == "j":
            self.arm_lpy[2] = clamp(self.arm_lpy[2] + ARM_ANGLE_STEP, ARM_LIMITS["y"])
            changed = True
        elif key == "l":
            self.arm_lpy[2] = clamp(self.arm_lpy[2] - ARM_ANGLE_STEP, ARM_LIMITS["y"])
            changed = True

        elif key == "w":
            self.arm_rpy[1] = clamp(self.arm_rpy[1] + ARM_ANGLE_STEP, ARM_LIMITS["pitch"])
            changed = True
        elif key == "s":
            self.arm_rpy[1] = clamp(self.arm_rpy[1] - ARM_ANGLE_STEP, ARM_LIMITS["pitch"])
            changed = True
        elif key == "a":
            self.arm_rpy[0] = clamp(self.arm_rpy[0] + ARM_ANGLE_STEP, ARM_LIMITS["roll"])
            changed = True
        elif key == "d":
            self.arm_rpy[0] = clamp(self.arm_rpy[0] - ARM_ANGLE_STEP, ARM_LIMITS["roll"])
            changed = True
        elif key == "q":
            self.arm_rpy[2] = clamp(self.arm_rpy[2] + ARM_ANGLE_STEP, ARM_LIMITS["yaw"])
            changed = True
        elif key == "e":
            self.arm_rpy[2] = clamp(self.arm_rpy[2] - ARM_ANGLE_STEP, ARM_LIMITS["yaw"])
            changed = True

        elif key == "n":
            self.gripper = clamp(self.gripper + GRIPPER_STEP, GRIPPER_LIMITS)
            changed = True
        elif key == "m":
            self.gripper = clamp(self.gripper - GRIPPER_STEP, GRIPPER_LIMITS)
            changed = True
        elif key == "r":
            self.reset()
            changed = True

        return changed

    def build_vr_pose(self):
        target_xyz = lpy_to_local_xyz(self.arm_lpy)
        delta_xyz = target_xyz - self.home_xyz
        return np.concatenate((delta_xyz, self.arm_rpy), axis=0)

    def build_rc_msg(self):
        msg = rc_command_lcmt()
        msg.mode = 0
        msg.left_stick = [-float(self.body[1]), float(self.body[0])]
        msg.right_stick = [-float(self.body[2]), 0.0]
        msg.knobs = [0.0, 0.0]
        return msg

    def build_vr_msg(self):
        msg = vr_command_lcmt()
        msg.ee_pose = self.build_vr_pose().tolist()
        return msg

    def build_gripper_msg(self):
        msg = gripper_lcmt()
        msg.gripper_cmd = float(self.gripper)
        return msg

    def format_status(self):
        pose = self.build_vr_pose()
        return (
            f"base | vx: {self.body[0]: .2f}, vy: {self.body[1]: .2f}, yaw: {self.body[2]: .2f}\n"
            f"arm  | l: {self.arm_lpy[0]: .2f}, p: {self.arm_lpy[1]: .2f}, y: {self.arm_lpy[2]: .2f}, "
            f"roll: {self.arm_rpy[0]: .2f}, pitch: {self.arm_rpy[1]: .2f}, yaw: {self.arm_rpy[2]: .2f}\n"
            f"delta| x: {pose[0]: .3f}, y: {pose[1]: .3f}, z: {pose[2]: .3f}, "
            f"roll: {pose[3]: .3f}, pitch: {pose[4]: .3f}, yaw: {pose[5]: .3f}\n"
            f"grip | cmd: {self.gripper: .2f}"
        )


def publish_state(lcm_node, state, publish_rc):
    if publish_rc:
        lcm_node.publish("rc_command", state.build_rc_msg().encode())
    lcm_node.publish("vr_command", state.build_vr_msg().encode())
    lcm_node.publish("gripper_command", state.build_gripper_msg().encode())


def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop publisher for deployment")
    parser.add_argument("--lcm-url", default=LCM_URL, help="LCM URL")
    parser.add_argument("--publish-hz", type=float, default=50.0, help="publish frequency")
    parser.add_argument(
        "--arm-only",
        action="store_true",
        help="only publish vr/gripper commands, do not publish rc_command for the base",
    )
    args = parser.parse_args()

    lcm_node = lcm.LCM(args.lcm_url)
    state = KeyboardTeleopState()
    shutdown = False
    publish_period = 1.0 / max(args.publish_hz, 1.0)
    publish_rc = not args.arm_only

    def signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(HELP_TEXT)
    print("")
    print("Current command:")
    print(state.format_status())
    print("")
    if publish_rc:
        print("Publishing: rc_command + vr_command + gripper_command")
        print("Note: do not run another keyboard/RC publisher on the same channels at the same time.")
    else:
        print("Publishing: vr_command + gripper_command")
    print("")

    publish_state(lcm_node, state, publish_rc)

    with RawKeyboard() as keyboard:
        while not shutdown:
            key = keyboard.read_key(publish_period)
            if key is None:
                publish_state(lcm_node, state, publish_rc)
                continue

            key_lower = key.lower()
            if key_lower == "h":
                print("")
                print(HELP_TEXT)
                print("")
                continue
            if key_lower == "x" or key == "ESC":
                shutdown = True
                continue

            if state.apply_key(key):
                publish_state(lcm_node, state, publish_rc)
                print("")
                print(state.format_status())

    state.reset()
    publish_state(lcm_node, state, publish_rc)
    print("")
    print("Keyboard teleop stopped. Published home pose and zero base velocity.")


if __name__ == "__main__":
    main()
