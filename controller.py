import sys
import time
import numpy as np

try:
    import openvr
except ImportError:
    print("Please install openvr: pip install openvr")
    sys.exit(1)

class Controller:
    def __init__(self):
        print("Initializing OpenVR System...")
        try:
            # 初始化 SteamVR 
            self.vr_system = openvr.init(openvr.VRApplication_Scene)
        except openvr.OpenVRError as e:
            print(f"Failed to initialize OpenVR: {e}")
            sys.exit(1)
            
        print("OpenVR Tracking Initialized Successfully.")

    def _get_button_state(self, controller_state, button_id):
        return (controller_state.ulButtonPressed & (1 << button_id)) != 0

    def get_action(self):
        """
        生成器，不断产出大小为 20 的 float 数组并 yield 出去，对齐 remote_pub.py 的 parser_action 解析逻辑：
        [pitch, yaw, roll, y, z, x, trigger] (左手 0-6)
        [pitch, yaw, roll, y, z, x, trigger] (右手 7-13)
        14:A(Right), 15:B(Right), 16:X(Left), 17:Y(Left), 18:joystick_x, 19:joystick_y
        """
        while True:
            # 申请长度20的数组，对应原有的action解包结构
            action = np.zeros(20, dtype=np.float32)
            
            # 获取所有姿态设备 
            poses, _ = openvr.VRCompositor().waitGetPoses(openvr.TrackingUniverseStanding, None)
            
            left_handled, right_handled = False, False

            for i in range(openvr.k_unMaxTrackedDeviceCount):
                device_class = self.vr_system.getTrackedDeviceClass(i)
                if device_class != openvr.TrackedDeviceClass_Controller:
                    continue
                
                # 获取控制器类型（左手还是右手）
                role = self.vr_system.getControllerRoleForTrackedDeviceIndex(i)
                
                # 获取按键状态
                result, controller_state = self.vr_system.getControllerState(i)
                if not result:
                    continue

                # 姿态位置信息
                pose = poses[i]
                if not pose.bPoseIsValid:
                    continue

                # 提取矩阵姿态
                m = pose.mDeviceToAbsoluteTracking
                # xyz 位置（原始顺序）
                x_ori, y_ori, z_ori = m[0][3], m[1][3], m[2][3]
                
                # 计算 RPY (Roll Pitch Yaw) 从旋转矩阵
                pitch = np.arcsin(max(-1.0, min(1.0, m[2][0])))
                if np.abs(m[2][0]) < 0.999999: # 没遇到万向锁
                    roll = np.arctan2(-m[2][1], m[2][2])
                    yaw = np.arctan2(-m[1][0], m[0][0])
                else: 
                    roll = 0.0
                    yaw = np.arctan2(m[0][1], m[1][1])
                
                # 读取扳机/抓手的值 (Axis 1 is typically trigger)
                trigger = controller_state.rAxis[1].x 

                # 严格对齐 remote_pub.py 的解析顺序：
                # left: [pitch, yaw, roll, y, z, x, trigger] (0-6)
                # right: [pitch, yaw, roll, y, z, x, trigger] (7-13)
                if role == openvr.TrackedControllerRole_LeftHand:
                    # 0:pitch, 1:yaw, 2:roll, 3:y, 4:z, 5:x, 6:trigger
                    action[0] = pitch
                    action[1] = yaw
                    action[2] = roll
                    action[3] = y_ori
                    action[4] = z_ori
                    action[5] = x_ori
                    action[6] = trigger
                    
                    # 按键映射：16:X(Left), 17:Y(Left)
                    x_btn = self._get_button_state(controller_state, openvr.k_EButton_A)  # X键
                    y_btn = self._get_button_state(controller_state, openvr.k_EButton_ApplicationMenu)  # Y键
                    action[16] = 1.0 if x_btn else 0.0
                    action[17] = 1.0 if y_btn else 0.0
                    
                    # 摇杆数据：18:joystick_x, 19:joystick_y (Axis 0)
                    action[18] = controller_state.rAxis[0].x
                    action[19] = controller_state.rAxis[0].y
                    
                    left_handled = True

                elif role == openvr.TrackedControllerRole_RightHand:
                    # 7:pitch, 8:yaw, 9:roll, 10:y, 11:z, 12:x, 13:trigger
                    action[7] = pitch
                    action[8] = yaw
                    action[9] = roll
                    action[10] = y_ori
                    action[11] = z_ori
                    action[12] = x_ori
                    action[13] = trigger
                    
                    # 按键映射：14:A(Right), 15:B(Right)
                    a_btn = self._get_button_state(controller_state, openvr.k_EButton_A)  # A键
                    b_btn = self._get_button_state(controller_state, openvr.k_EButton_ApplicationMenu)  # B键
                    action[14] = 1.0 if a_btn else 0.0
                    action[15] = 1.0 if b_btn else 0.0
                    
                    right_handled = True
            
            # 返回 20 位 float 数组
            yield action
            
            # 控制频率大约 50~100Hz
            time.sleep(0.01)

if __name__ == "__main__":
    c = Controller()
    for d in c.get_action():
        print(np.around(d, 3))