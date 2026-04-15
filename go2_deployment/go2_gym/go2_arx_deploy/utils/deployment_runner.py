import copy
import time
import os

import numpy as np
import torch

from go2_arx_deploy.utils.logger import MultiLogger
from go2_arx_deploy.envs.lcm_agent import LCMAgent
from typing import Dict


class DeploymentRunner:
    def __init__(self, experiment_name="unnamed", se=None, log_root="."):
        self.agents: Dict[str, LCMAgent] = {}
        self.arm_policy = None
        self.dog_policy = None
        self.command_profile = None
        self.logger = MultiLogger()
        self.se = se
        self.vision_server = None

        self.log_root = log_root
        self.init_log_filename()
        self.control_agent_name = None
        self.command_agent_name = None

        self.triggered_commands = {i: None for i in range(4)} # command profiles for each action button on the controller
        self.button_states = np.zeros(4)

        self.is_currently_probing = False
        self.is_currently_logging = [False, False, False, False]
        self.profile_print_interval = 100

    def init_log_filename(self):
        datetime = time.strftime("%Y/%m_%d/%H_%M_%S")

        for i in range(100):
            try:
                os.makedirs(f"{self.log_root}/{datetime}_{i}")
                self.log_filename = f"{self.log_root}/{datetime}_{i}/log.pkl"
                return
            except FileExistsError:
                continue


    def add_open_loop_agent(self, agent, name):
        self.agents[name] = agent
        self.logger.add_robot(name, agent.env.cfg)

    def add_control_agent(self, agent, name):
        self.control_agent_name = name
        self.agents[name] = agent
        self.logger.add_robot(name, agent.env.cfg)

    def add_vision_server(self, vision_server):
        self.vision_server = vision_server

    def set_command_agents(self, name):
        self.command_agent = name

    def add_policy(self, arm_policy, dog_policy):
        self.arm_policy = arm_policy
        self.dog_policy = dog_policy

    def add_command_profile(self, command_profile):
        self.command_profile = command_profile


    def calibrate(self, wait=True, low=False):
        # first, if the robot is not in nominal pose, move slowly to the nominal pose
        for agent_name in self.agents.keys():
            if hasattr(self.agents[agent_name], "get_arm_observations"):
                agent = self.agents[agent_name]
                agent.get_arm_observations()
                joint_pos = agent.env.dof_pos
                if low:
                    final_goal = np.array([0., 0.3, -0.7,
                                           0., 0.3, -0.7,
                                           0., 0.3, -0.7,
                                           0., 0.3, -0.7,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                else:
                    final_goal = np.zeros(18)
                nominal_joint_pos = agent.env.default_dof_pos

                print(f"About to calibrate; the robot will stand [Press R2 to calibrate]")
                while wait:  # fist R2
                    self.button_states = self.command_profile.get_buttons()
                    if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                        break

                # move dog to the target sequence
                cal_action = np.zeros(agent.env.num_actions)    
                target_sequence = []
                target = joint_pos - nominal_joint_pos
                while np.max(np.abs(target - final_goal)) > 0.01:
                    target -= np.clip((target - final_goal), -0.05, 0.05)
                    target_sequence += [copy.deepcopy(target)]
                for target in target_sequence:
                    next_target = target
                    if isinstance(agent.env.cfg, dict):
                        hip_reduction = agent.env.cfg["control"]["hip_scale_reduction"]
                        action_scale = agent.env.cfg["control"]["action_scale"]
                    else:
                        hip_reduction = agent.env.cfg["control"]["hip_scale_reduction"]
                        action_scale = agent.env.cfg["control"]["action_scale"]

                    next_target[[0, 3, 6, 9]] /= hip_reduction
                    next_target = next_target / action_scale
                    cal_action[0:12] = next_target[0:12]
                    cal_action[-6:] = 0
                    agent.step(torch.from_numpy(cal_action[0:12]), torch.from_numpy(cal_action[-6:]))
                    agent.get_arm_observations()
                    time.sleep(0.05)
                print("Starting pose calibrated [Press R2 to start controller]")
                while True:  # second R2
                    self.button_states = self.command_profile.get_buttons()
                    if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                        break

                for agent_name in self.agents.keys():
                    obs = self.agents[agent_name].reset()
                    obs = self.agents[agent_name].get_arm_observations()
                    if agent_name == self.control_agent_name:
                        control_obs = obs

        return control_obs


    def run(self, num_log_steps=1000000000, max_steps=100000000, logging=True):
        assert self.control_agent_name is not None, "cannot deploy, runner has no control agent!"
        assert self.dog_policy is not None and self.arm_policy is not None, "cannot deploy, runner has no policy!"
        assert self.command_profile is not None, "cannot deploy, runner has no command profile!"

        profile_stats = {
            "loop_s": 0.0,
            "arm_policy_s": 0.0,
            "dog_obs_s": 0.0,
            "dog_policy_s": 0.0,
            "step_s": 0.0,
            "arm_obs_s": 0.0,
        }
        profile_count = 0

        for agent_name in self.agents.keys():
            obs = self.agents[agent_name].reset()
            if agent_name == self.control_agent_name:
                control_obs = obs

        control_obs = self.calibrate(wait=True)

        # now, run control loop

        try:
            for i in range(max_steps):
                loop_start = time.perf_counter()
                policy_info = {}
                arm_policy_start = time.perf_counter()
                actions_arm = self.arm_policy(control_obs)
                profile_stats["arm_policy_s"] += time.perf_counter() - arm_policy_start
                for agent_name in self.agents.keys():
                    agent = self.agents[agent_name]
                    agent.plan(actions_arm[..., -2:])
                    dog_obs_start = time.perf_counter()
                    dog_obs = agent.get_dog_observations()
                    profile_stats["dog_obs_s"] += time.perf_counter() - dog_obs_start
                    dog_policy_start = time.perf_counter()
                    actions_dog = self.dog_policy(dog_obs)
                    profile_stats["dog_policy_s"] += time.perf_counter() - dog_policy_start
                    step_start = time.perf_counter()
                    rew_dog, rew_arm, done, info = agent.step(actions_dog, actions_arm[:-2])
                    profile_stats["step_s"] += time.perf_counter() - step_start
                    info.update(policy_info)
                    info.update({"observation": obs, "reward": (rew_dog, rew_arm), "done": done, "timestep": i,
                                 "time": i * self.agents[self.control_agent_name].env.dt, "dog_action": actions_dog, "arm_action": actions_arm, "rpy": self.agents[self.control_agent_name].env.se.get_rpy()})

                    if logging: self.logger.log(agent_name, info)

                    if agent_name == self.control_agent_name:
                        arm_obs_start = time.perf_counter()
                        control_obs = agent.get_arm_observations()
                        profile_stats["arm_obs_s"] += time.perf_counter() - arm_obs_start

                # bad orientation emergency stop
                rpy = self.agents[self.control_agent_name].env.se.get_rpy()
                if abs(rpy[0]) > 1.6 or abs(rpy[1]) > 1.6:
                    self.calibrate(wait=False, low=True)

                # check for logging command
                prev_button_states = self.button_states[:]
                self.button_states = self.command_profile.get_buttons()


                if self.command_profile.state_estimator.left_lower_left_switch_pressed:
                    if not self.is_currently_probing:
                        print("START LOGGING")
                        self.is_currently_probing = True
                        self.agents[self.control_agent_name].set_probing(True)
                        self.init_log_filename()
                        self.logger.reset()
                    else:
                        print("SAVE LOG")
                        self.is_currently_probing = False
                        self.agents[self.control_agent_name].set_probing(False)
                        # calibrate, log, and then resume control
                        control_obs = self.calibrate(wait=False)
                        self.logger.save(self.log_filename)
                        self.init_log_filename()
                        self.logger.reset()
                        time.sleep(1)
                        control_obs = self.agents[self.control_agent_name].reset()
                        control_obs = self.agents[self.control_agent_name].get_arm_observations()
                    self.command_profile.state_estimator.left_lower_left_switch_pressed = False

                if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                    control_obs = self.calibrate(wait=False)
                    time.sleep(1)
                    self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                    while not self.command_profile.state_estimator.right_lower_right_switch_pressed:
                        time.sleep(0.01)
                    self.command_profile.state_estimator.right_lower_right_switch_pressed = False

                profile_stats["loop_s"] += time.perf_counter() - loop_start
                profile_count += 1
                if profile_count >= self.profile_print_interval:
                    avg_loop_s = profile_stats["loop_s"] / profile_count
                    avg_loop_hz = 1.0 / avg_loop_s if avg_loop_s > 0 else float("inf")
                    se = self.agents[self.control_agent_name].env.se
                    lcm_stats = se.get_lcm_debug_snapshot() if hasattr(se, "get_lcm_debug_snapshot") else {}

                    def _fmt_seconds_ms(value):
                        return "n/a" if value is None else f"{value * 1000.0:.1f}ms"

                    def _fmt_lcm_line(name):
                        stats = lcm_stats.get(name, {})
                        return f"{name}:dt={_fmt_seconds_ms(stats.get('interval_s'))},age={_fmt_seconds_ms(stats.get('age_s'))}"

                    print(
                        "profile | "
                        f"loop={avg_loop_s * 1000.0:.2f}ms ({avg_loop_hz:.2f}Hz) | "
                        f"arm_policy={profile_stats['arm_policy_s'] / profile_count * 1000.0:.2f}ms | "
                        f"dog_obs={profile_stats['dog_obs_s'] / profile_count * 1000.0:.2f}ms | "
                        f"dog_policy={profile_stats['dog_policy_s'] / profile_count * 1000.0:.2f}ms | "
                        f"agent.step={profile_stats['step_s'] / profile_count * 1000.0:.2f}ms | "
                        f"arm_obs={profile_stats['arm_obs_s'] / profile_count * 1000.0:.2f}ms | "
                        f"{_fmt_lcm_line('imu')} | "
                        f"{_fmt_lcm_line('leg')} | "
                        f"{_fmt_lcm_line('arm')} | "
                        f"{_fmt_lcm_line('rc')} | "
                        f"{_fmt_lcm_line('vr')}"
                    )
                    for key in profile_stats:
                        profile_stats[key] = 0.0
                    profile_count = 0

            # finally, return to the nominal pose
            control_obs = self.calibrate(wait=False)
            self.logger.save(self.log_filename)

        except KeyboardInterrupt:
            self.logger.save(self.log_filename)
