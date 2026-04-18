# Go2 + Piper Deployment Notes

This note records the command sequence for the current `Go2 + Piper` deployment flow.

The control chain is now split into two parts:

- `lcm_position_vr_go2`: handles only Go2 low-level control, Go2 state, and RC bridge
- `piper_bridge.py`: handles Piper arm commands, Piper feedback, and gripper control


## 1. Build `lcm_position_vr_go2`

Rebuild after any modification to:

- `go2_deployment/go2_gym/go2_arx_deploy/unitree_legged_sdk_bin/lcm_position_vr_go2.cpp`

Commands:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment/go2_deployment/go2_gym/go2_arx_deploy/unitree_legged_sdk_bin
bash make.sh
```


## 2. Terminal Order

Use separate terminals. Recommended startup order:

1. Go2 low-level bridge
2. Piper bridge
3. RL policy
4. VR receiver
5. VR streaming sender


## 3. Terminal 1: Go2 Low-Level Control

Normal:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment/go2_deployment/go2_gym/go2_arx_deploy/unitree_legged_sdk_bin
sudo ./lcm_position_vr_go2 eth0
```

With front leg motor position print:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment/go2_deployment/go2_gym/go2_arx_deploy/unitree_legged_sdk_bin
sudo ./lcm_position_vr_go2 eth0 --printmotor
```

Notes:

- Replace `eth0` with your actual Go2 network interface if needed.
- This program now does not control the arm anymore.


## 4. Terminal 2: Piper Bridge

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment
python3 go2_deployment/go2_gym/go2_arx_deploy/piper_bridge.py --can-name can0
```

If `can0` requires elevated permission:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment
sudo -E $(which python3) go2_deployment/go2_gym/go2_arx_deploy/piper_bridge.py --can-name can0
```

Useful options:

```bash
python3 go2_deployment/go2_gym/go2_arx_deploy/piper_bridge.py --can-name can0 --log-interval 0.2
python3 go2_deployment/go2_gym/go2_arx_deploy/piper_bridge.py --can-name can0 --log-interval 0
```

What it does:

- enables Piper
- moves Piper to startup pose `[0.0, 0.6, -0.5, 0.0, 0.0, 0.0]`
- then starts listening to LCM arm commands


## 5. Terminal 3: RL Policy

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment
python3 go2_deployment/go2_gym/go2_arx_deploy/scripts/deploy_policy_vr.py
```


## 6. Terminal 4: VR Receiver

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment
python3 go2_deployment/go2_gym/go2_arx_deploy/remote_pub.py
```


## 7. Terminal 5: Keyboard Receiver

Run this instead of `remote_pub.py` when you want keyboard input instead of VR:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment
python3 go2_deployment/go2_gym/go2_arx_deploy/keyboard_pub.py
```

If you only want keyboard arm control and do not want to publish base `rc_command`:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment
python3 go2_deployment/go2_gym/go2_arx_deploy/keyboard_pub.py --arm-only
```

Keyboard mapping:

- base:
  - `8 / 5`: forward / backward
  - `4 / 6`: move left / right
  - `7 / 9`: turn left / right
  - `0`: zero base velocity
- arm Cartesian delta:
  - `I / K`: `delta x` forward / backward
  - `J / L`: `delta y` left / right
  - `U / O`: `delta z` up / down
- arm orientation:
  - `W / S`: pitch down / up
  - `A / D`: roll left / right
  - `Q / E`: yaw left / right
- gripper:
  - `N / M`: open / close
- utility:
  - `R`: reset base, arm, and gripper to home
  - `H`: print help
  - `X`: exit

Notes:

- Do not run `remote_pub.py` and `keyboard_pub.py` at the same time.
- `keyboard_pub.py` publishes the same `vr_command` / `gripper_command` interface used by the VR path.


## 8. Terminal 6: VR Streaming Sender

Run this on the machine connected to the VR headset / SteamVR:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment
python3 vr_streaming.py
```


## 9. Quick Runtime Summary

- `lcm_position_vr_go2`:
  - reads Go2 lowstate
  - publishes Go2 state to LCM
  - receives leg commands from LCM
  - sends low-level commands to Go2

- `piper_bridge.py`:
  - receives `q_arm_des` and `gripper_command` from LCM
  - sends commands to Piper
  - reads Piper joint / gripper feedback
  - publishes arm feedback to LCM

- `cheetah_state_estimator.py`:
  - reads Go2 leg state from `leg_control_data`
  - reads Piper arm state from `piper_arm_feedback`
  - reads arm teleop commands from `vr_command`
  - reads gripper commands from `gripper_command`
  - reads base teleop commands from `rc_command`


## 10. Logging

- Hand controller mode:
  - press `L2` once: `START LOGGING`
  - press `L2` again: `SAVE LOG`
- Keyboard mode:
  - there is currently no dedicated keyboard logging key
  - logging still depends on the `left_lower_left_switch` / `L2` path in `rc_command`
- Current behavior after the recent change:
  - second `L2` only saves the log
  - it no longer calls `calibrate(wait=False)`
  - it no longer sends the robot back to the calibration pose


## 11. Common Problems

### `No module named lcm`

Install Python LCM into the Python environment used to run `piper_bridge.py`.

Example:

```bash
python3 -m pip install lcm
```


### Piper does not move

Check:

- `can0` is up
- Piper power is on
- Piper can be enabled by SDK
- the Python environment can import both `lcm` and `piper_sdk`


### Go2 binary changed but behavior did not change

You probably forgot to rebuild:

```bash
cd /home/unitree/Downloads/roboduet_go2piper_deployment/go2_deployment/go2_gym/go2_arx_deploy/unitree_legged_sdk_bin
bash make.sh
```
