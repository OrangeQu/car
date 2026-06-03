"""
机械臂控制模块
使用从 YouBot C 库移植的 arm_ik() 逆运动学算法
参考: arm.c 中的 arm_ik() 函数
"""

import math
from controller import Robot

ARM_JOINTS = ["arm1", "arm2", "arm3", "arm4", "arm5"]

# 机械臂各段长度（来自 Webots 模型定义）
# 参考: box.wbt 中 HingeJoint anchor 和 translation 值
ARM_LENGTHS = {
    "arm1": 0.077,  # 基座高度（从 ARM Solid 到 arm2 关节）
    "arm2": 0.155,  # 上臂长度（arm2 anchor 到 arm3 anchor）
    "arm3": 0.135,  # 前臂长度（arm3 anchor 到 arm4 anchor）
    "arm4": 0.081,  # 腕部长度（arm4 anchor 到 arm5 anchor）
    "arm5": 0.090   # 夹爪长度（arm5 anchor 到夹爪末端）
}

# 机械臂基座在机器人坐标系中的偏移
# ARM Solid 的 translation 为 0.156 0 0
ARM_BASE_OFFSET_X = 0.156

# 预设姿态（关节角度，弧度）
# 这些值来自 C 库 arm_set_height() 和 arm_set_orientation()
POSES = {
    "reset": {
        "arm1": 0.0,
        "arm2": 1.57,
        "arm3": -2.635,
        "arm4": 1.78,
        "arm5": 0.0,
    },
    "carry": {
        "arm1": 0.0,
        "arm2": 1.2,
        "arm3": -1.5,
        "arm4": 0.3,
        "arm5": 0.0,
    },
    "pre_grasp": {
        "arm1": 0.0,
        "arm2": 0.8,
        "arm3": -1.2,
        "arm4": 0.0,
        "arm5": 0.0,
    },
    "grasp": {
        "arm1": 0.0,
        "arm2": 1.0,
        "arm3": -1.8,
        "arm4": 0.0,
        "arm5": 0.0,
    },
    "grasp_low": {
        "arm1": 0.0,
        "arm2": 0.0,
        "arm3": -2.4,
        "arm4": -0.5,
        "arm5": 0.0,
    },
    "place_table": {
        "arm1": 0.0,
        "arm2": 0.8,
        "arm3": -0.6,
        "arm4": 0.0,
        "arm5": 0.0,
    },
}


class ArmController:
    """YouBot 5自由度机械臂控制器（使用 C 库 IK 算法）"""

    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep

        # 获取5个关节电机
        self.motors = {}
        for name in ARM_JOINTS:
            motor = robot.getDevice(name)
            if motor:
                motor.setVelocity(0.5)  # 中等速度
                self.motors[name] = motor
            else:
                print(f"  ⚠️ 未找到 {name}")

        # 尝试获取夹爪末端节点（用于获取实际位置）
        self.gripper_node = None
        try:
            # 尝试通过 getFromDef 获取夹爪节点
            self.gripper_node = robot.getFromDef("gripper")
            if self.gripper_node:
                print("  ✓ 已获取夹爪节点引用")
        except:
            pass

        # 复位到初始姿态
        self.set_pose("reset")
        self._wait_for_motors(50)

        print("  ✓ 机械臂初始化完成（C库IK算法）")

    def get_actual_angles(self):
        """获取当前实际关节角度"""
        angles = {}
        for name in ARM_JOINTS:
            if name in self.motors:
                try:
                    angles[name] = self.motors[name].getPositionSensor().getValue()
                except:
                    angles[name] = 0.0
        return angles

    def forward_kinematics(self, angles_dict):
        """
        正运动学：根据关节角度计算夹爪末端位置
        
        参数:
            angles_dict: 关节角度字典 {arm1, arm2, arm3, arm4, arm5}
        
        返回:
            (x, y, z) 夹爪末端在机器人基座坐标系下的位置
            x: 前后方向（正=前）
            y: 左右方向（正=左）
            z: 上下方向（正=上）
        """
        a1 = angles_dict.get("arm1", 0.0)  # 绕Z轴旋转
        a2 = angles_dict.get("arm2", 0.0)  # 绕Y轴旋转（上臂前后摆动）
        a3 = angles_dict.get("arm3", 0.0)  # 绕Y轴旋转（前臂前后摆动）
        a4 = angles_dict.get("arm4", 0.0)  # 绕Y轴旋转（腕部摆动）
        a5 = angles_dict.get("arm5", 0.0)  # 绕Z轴旋转（腕部旋转）
        
        L1 = ARM_LENGTHS["arm1"]  # 基座高度
        L2 = ARM_LENGTHS["arm2"]  # 上臂
        L3 = ARM_LENGTHS["arm3"]  # 前臂
        L4 = ARM_LENGTHS["arm4"]  # 腕部
        L5 = ARM_LENGTHS["arm5"]  # 夹爪
        
        # 简化正运动学（在 XZ 平面内，忽略 arm1 和 arm5 的旋转）
        # arm2 和 arm3 绕 Y 轴旋转，所以运动在 XZ 平面
        
        # 机械臂基座在机器人坐标系中的位置
        # ARM Solid 在机器人前方 0.156m
        base_x = ARM_BASE_OFFSET_X
        
        # arm2 关节位置（肩膀）
        # 在基座顶部，高度 = L1
        
        # arm2 末端（肘部）位置
        # arm2 从垂直向上（a2=0）向前摆动（a2>0）
        # 当 a2=0 时，上臂垂直向上，末端在 (base_x, 0, L1 + L2)
        # 当 a2>0 时，上臂向前倾斜
        elbow_x = base_x + L2 * math.sin(a2)
        elbow_z = L1 + L2 * math.cos(a2)
        
        # arm3 末端（腕部）位置
        # arm3 相对于 arm2 的夹角
        # arm3=0 时前臂与上臂在一条直线上
        # arm3<0 时前臂向下弯曲
        total_angle_arm3 = a2 + a3  # 前臂相对于垂直方向的角度
        wrist_x = elbow_x + L3 * math.sin(total_angle_arm3)
        wrist_z = elbow_z + L3 * math.cos(total_angle_arm3)
        
        # arm4 末端（夹爪根部）位置
        total_angle_arm4 = a2 + a3 + a4
        gripper_base_x = wrist_x + L4 * math.sin(total_angle_arm4)
        gripper_base_z = wrist_z + L4 * math.cos(total_angle_arm4)
        
        # arm5 末端（夹爪尖端）位置
        total_angle_arm5 = a2 + a3 + a4  # arm5 是绕 Z 轴旋转，不影响位置
        tip_x = gripper_base_x + L5 * math.sin(total_angle_arm5)
        tip_z = gripper_base_z + L5 * math.cos(total_angle_arm5)
        
        # 考虑 arm1 的旋转（绕 Z 轴）
        cos_a1 = math.cos(a1)
        sin_a1 = math.sin(a1)
        world_x = tip_x * cos_a1  # 前后方向
        world_y = tip_x * sin_a1  # 左右方向
        world_z = tip_z           # 上下方向
        
        return (world_x, world_y, world_z)

    def print_pose_info(self, pose_name, angles_dict):
        """打印姿态信息，包括关节角度和末端位置"""
        # 计算末端位置
        tip_pos = self.forward_kinematics(angles_dict)
        
        print(f"  📐 [{pose_name}] 关节角度: "
              f"arm1={angles_dict.get('arm1', 0):.3f}, "
              f"arm2={angles_dict.get('arm2', 0):.3f}, "
              f"arm3={angles_dict.get('arm3', 0):.3f}, "
              f"arm4={angles_dict.get('arm4', 0):.3f}, "
              f"arm5={angles_dict.get('arm5', 0):.3f}")
        print(f"     📍 夹爪末端估计位置: "
              f"前={tip_pos[0]:.3f}m, 左={tip_pos[1]:.3f}m, 高={tip_pos[2]:.3f}m")

    def set_joint_angles(self, angles_dict):
        """直接设置关节角度"""
        for name, angle in angles_dict.items():
            if name in self.motors:
                self.motors[name].setPosition(angle)

    def set_pose(self, pose_name):
        """设置预设姿态"""
        if pose_name in POSES:
            angles = POSES[pose_name]
            self.set_joint_angles(angles)
            self.print_pose_info(pose_name, angles)
            return True
        else:
            print(f"  ⚠️ 未知姿态: {pose_name}")
            return False

    def _wait_for_motors(self, steps=50):
        """等待电机到位"""
        for _ in range(steps):
            self.robot.step(self.timestep)

    def ik(self, forward, left, up):
        """
        逆运动学求解（从 C 库 arm_ik() 移植）
        
        参数:
            forward: 目标点在机器人基座坐标系下的前后方向 [m]（正=前）
            left:    目标点在机器人基座坐标系下的左右方向 [m]（正=左）
            up:      目标点在机器人基座坐标系下的上下方向 [m]（正=上）
        
        C 库中的 arm_ik(y, z, x) 参数顺序为:
            y: 前后方向 (forward)
            z: 上下方向 (up)  
            x: 左右方向 (left)
        """
        # C 库 arm_ik(y, z, x) 中:
        # y = forward (前后), z = up (上下), x = left (左右)
        y_c = forward   # 前后方向
        x_c = left      # 左右方向
        z_c = up        # 上下方向
        
        # 计算水平距离和垂直高度
        y1 = math.sqrt(x_c * x_c + y_c * y_c)
        z1 = z_c + ARM_LENGTHS["arm4"] + ARM_LENGTHS["arm5"] - ARM_LENGTHS["arm1"]

        a = ARM_LENGTHS["arm2"]
        b = ARM_LENGTHS["arm3"]
        c = math.sqrt(y1 * y1 + z1 * z1)

        # 防止数值问题
        if y1 < 0.001:
            y1 = 0.001
        if c > a + b or c < abs(a - b):
            print(f"  ⚠️ IK 无解: 目标点 ({forward:.3f}, {left:.3f}, {up:.3f}) 超出工作空间")
            return None

        # 计算各关节角度（与 C 库完全一致）
        # C 库: alpha = -asin(x / y1)，其中 x 是左右方向
        alpha = -math.asin(x_c / y1) if y1 > 0.001 else 0.0
        beta = -(math.pi / 2.0 - math.acos((a * a + c * c - b * b) / (2.0 * a * c)) - math.atan2(z1, y1))
        gamma = -(math.pi - math.acos((a * a + b * b - c * c) / (2.0 * a * b)))
        delta = -(math.pi + (beta + gamma))
        epsilon = math.pi / 2.0 + alpha

        # 关节限位裁剪（防止超出物理限制）
        # arm1: [-6.28, 6.28] (无限制)
        # arm2: [0.0, 3.14] (只能向前)
        # arm3: [-3.14, 0.0] (只能向后)
        # arm4: [-1.78, 1.78] (有限制)
        # arm5: [-3.14, 3.14] (无限制)
        beta = max(0.01, min(3.14, beta))
        gamma = max(-3.14, min(-0.01, gamma))
        delta = max(-1.75, min(1.75, delta))

        return {
            "arm1": alpha,
            "arm2": beta,
            "arm3": gamma,
            "arm4": delta,
            "arm5": epsilon
        }

    def move_to_ik(self, x, y, z):
        """
        使用 IK 移动到目标位置
        
        参数:
            x: 前后方向 [m]（正=前）
            y: 左右方向 [m]（正=左）
            z: 垂直方向 [m]（正=上）
        """
        angles = self.ik(x, y, z)
        if angles is None:
            return False

        self.set_joint_angles(angles)
        print(f"  🦾 IK 移动到 ({x:.3f}, {y:.3f}, {z:.3f})")
        self.print_pose_info("IK_target", angles)
        return True

    def _scan_with_arm1(self, gripper):
        """
        用 arm1 左右摆动扫描，用夹爪传感器检测木块的实际位置
        
        策略：
        1. 先设置 grasp_low 姿态（夹爪在地面高度）
        2. arm1 从 -0.15 摆动到 0.15 rad（左右各约 8.6 度）
        3. 在每个位置尝试闭合夹爪，检测是否碰到木块
        4. 记录碰到木块时的 arm1 角度
        
        返回: 检测到的 arm1 偏移角度 [rad]，如果没有检测到返回 0.0
        """
        print(f"  🦾 阶段0: arm1 扫描检测木块偏移...")
        
        # 先张开夹爪
        gripper.open()
        self._wait_for_motors(10)
        
        # 设置 grasp_low 姿态（夹爪在地面高度）
        self.set_pose("grasp_low")
        self._wait_for_motors(40)
        
        # arm1 扫描范围：从 -0.25 到 0.25 rad（左右各约 14.3 度）
        # 木块被底盘碰偏后可能偏移较大，需要更大的扫描范围
        scan_range = 0.25
        scan_step = 0.03  # 步长
        
        # 从右到左扫描（arm1 负方向到正方向）
        for angle in [i * scan_step for i in range(int(-scan_range/scan_step), int(scan_range/scan_step) + 1)]:
            self.motors["arm1"].setPosition(angle)
            self._wait_for_motors(15)
            
            # 尝试闭合夹爪
            gripper.close()
            self._wait_for_motors(8)
            
            # 检测是否夹到物体
            if gripper.wait_for_grasp(timeout_steps=30):
                print(f"    ✅ arm1 扫描: 在 {angle:.3f}rad 处检测到木块!")
                # 注意：不要张开夹爪，保持闭合状态
                # 因为扫描时已经夹到了木块，直接返回偏移角度
                return angle
            
            # 没夹到，张开继续
            gripper.open()
            self._wait_for_motors(8)
        
        print(f"    ⚠️ arm1 扫描未检测到木块偏移")
        return 0.0

    def grasp_block(self, block_x, block_y, block_z=0.05, gripper=None, offset_y=0.0):
        """
        抓取木块（使用预设姿态序列 + 逐步下探 + 偏移补偿）
        
        策略：
        1. 用 arm1 左右摆动扫描，检测木块实际偏移
        2. 用检测到的偏移补偿 arm1 角度
        3. pre_grasp: 抬起机械臂到准备位置
        4. grasp_low: 低姿态接近木块
        5. 逐步下探：逐步减小 arm2（向后摆），让夹爪逐步降低
           每次步进后尝试闭合夹爪，检测是否碰到木块
        
        参数:
            block_x: 木块在机器人坐标系下的 x [m]（前后，正=前）
            block_y: 木块在机器人坐标系下的 y [m]（左右，正=左）
            block_z: 木块高度 [m]（地面木块约 0.05m）
            gripper: GripperController 实例，用于检测抓取
            offset_y: 木块左右偏移补偿 [m]（正=左偏，负=右偏）
        """
        print(f"\n  🦾 ===== 抓取动作开始 =====")
        print(f"  🎯 目标木块位置: 前={block_x:.3f}m, 左={block_y:.3f}m, 高={block_z:.3f}m")
        
        # 阶段0: 用 arm1 扫描检测木块偏移
        detected_arm1_offset = 0.0
        if gripper is not None:
            detected_arm1_offset = self._scan_with_arm1(gripper)
            if abs(detected_arm1_offset) > 0.01:
                print(f"  🦾 检测到木块偏移，arm1 补偿 {detected_arm1_offset:.3f}rad")
            # 扫描后张开夹爪，避免后续阶段夹爪处于闭合状态
            gripper.open()
            self._wait_for_motors(15)
        
        # 阶段1: 准备姿态（机械臂抬起）
        print(f"  🦾 阶段1: 准备抓取姿态")
        self.set_pose("pre_grasp")
        self._wait_for_motors(60)

        # 阶段2: 低姿态接近（带 arm1 偏移补偿）
        print(f"  🦾 阶段2: 低姿态接近")
        self.set_pose("grasp_low")
        # 如果有检测到的偏移，叠加到 arm1
        if abs(detected_arm1_offset) > 0.01:
            current_arm1 = POSES["grasp_low"]["arm1"] + detected_arm1_offset
            self.motors["arm1"].setPosition(current_arm1)
            print(f"     叠加 arm1 偏移: {current_arm1:.3f}rad")
        self._wait_for_motors(80)
        
        # 阶段3: 逐步下探（如果提供了 gripper）
        if gripper is not None:
            print(f"  🦾 阶段3: 逐步下探寻找木块")
            
            # 先张开夹爪到最大，确保不会在第一次闭合时碰偏木块
            gripper.open()
            self._wait_for_motors(30)  # 多等一会儿确保完全张开
            
            # 逐步减小 arm2（向后摆），让夹爪下探
            # arm2 限位: [-1.13446, 1.5708]
            # 木块高度 100mm，夹爪张开 60mm
            # 需要下探足够深让夹爪包住木块整个高度
            # 从 arm2=0.0 下探到 arm2=-1.13446（物理限位）
            arm2_start = 0.0
            arm2_end = -1.13446  # 下探到物理限位
            arm2_step = -0.02  # 更小的步长，更精细
            
            current_arm2 = arm2_start
            found = False
            contact_arm2 = None  # 记录首次接触时的 arm2 角度
            contact_count = 0  # 接触次数计数
            max_contacts = 6  # 需要6次接触才夹紧，确保完全包住木块
            
            while current_arm2 >= arm2_end and not found:
                # 设置 arm2 角度
                self.motors["arm2"].setPosition(current_arm2)
                self._wait_for_motors(20)  # 等待电机到位
                
                # 打印当前状态
                actual = self.get_actual_angles()
                pos = self.forward_kinematics(actual)
                abs_z = pos[2] + 0.103
                print(f"    arm2={current_arm2:.3f}, 夹爪高={pos[2]:.3f}m, 绝对高={abs_z:.3f}m")
                
                # 尝试闭合夹爪检测
                gripper.close()
                self._wait_for_motors(8)
                
                # 检测是否夹到物体
                if gripper.wait_for_grasp(timeout_steps=50):
                    contact_count += 1
                    if contact_count < max_contacts:
                        # 还没到最大接触次数，继续下探
                        print(f"  📍 第{contact_count}次接触木块 (arm2={current_arm2:.3f})，继续下探包住木块...")
                        gripper.open()
                        self._wait_for_motors(10)
                        current_arm2 += arm2_step
                        continue
                    else:
                        # 达到最大接触次数，真正夹紧
                        print(f"  ✅ 下探足够深，夹紧木块! (arm2={current_arm2:.3f}, 接触次数={contact_count})")
                        found = True
                        break
                
                # 没夹到，张开夹爪继续下探
                gripper.open()
                self._wait_for_motors(8)
                
                current_arm2 += arm2_step
            
            if not found and contact_arm2 is not None:
                # 如果首次接触后下探了但没再检测到，尝试在接触位置再下探一点
                print(f"  🦾 在首次接触位置继续下探...")
                # 从接触位置再下探 3 步
                extra_steps = 3
                for i in range(extra_steps):
                    current_arm2 += arm2_step
                    if current_arm2 < arm2_end:
                        break
                    self.motors["arm2"].setPosition(current_arm2)
                    self._wait_for_motors(30)
                    
                    actual = self.get_actual_angles()
                    pos = self.forward_kinematics(actual)
                    abs_z = pos[2] + 0.103
                    print(f"    arm2={current_arm2:.3f}, 夹爪高={pos[2]:.3f}m, 绝对高={abs_z:.3f}m")
                    
                    gripper.close()
                    self._wait_for_motors(10)
                    
                    if gripper.wait_for_grasp(timeout_steps=80):
                        print(f"  ✅ 额外下探后夹到木块! (arm2={current_arm2:.3f})")
                        found = True
                        break
                    
                    gripper.open()
                    self._wait_for_motors(10)
            
            if not found:
                # 如果 arm2 下探没找到，尝试调整 arm4
                print(f"  🦾 arm2 下探未找到，尝试调整 arm4")
                # 恢复 arm2 到 0.0
                self.motors["arm2"].setPosition(0.0)
                self._wait_for_motors(30)
                
                # 逐步减小 arm4
                arm4_start = -0.5
                arm4_end = -1.2
                arm4_step = -0.1
                
                current_arm4 = arm4_start
                contact_arm4 = None
                while current_arm4 >= arm4_end and not found:
                    self.motors["arm4"].setPosition(current_arm4)
                    self._wait_for_motors(30)
                    
                    actual = self.get_actual_angles()
                    pos = self.forward_kinematics(actual)
                    abs_z = pos[2] + 0.103
                    print(f"    arm4={current_arm4:.3f}, 夹爪高={pos[2]:.3f}m, 绝对高={abs_z:.3f}m")
                    
                    gripper.close()
                    self._wait_for_motors(10)
                    
                    if gripper.wait_for_grasp(timeout_steps=80):
                        if contact_arm4 is None:
                            contact_arm4 = current_arm4
                            print(f"  📍 首次接触木块 (arm4={current_arm4:.3f})，继续下探包住木块...")
                            gripper.open()
                            self._wait_for_motors(10)
                            current_arm4 += arm4_step
                            continue
                        else:
                            print(f"  ✅ 在 arm4 下探过程中夹到木块! (arm4={current_arm4:.3f})")
                            found = True
                            break
                    
                    gripper.open()
                    self._wait_for_motors(10)
                    
                    current_arm4 += arm4_step
            
            if not found:
                print(f"  ⚠️ 下探未找到木块，使用默认姿态")
                # 恢复默认姿态
                self.set_pose("grasp_low")
                self._wait_for_motors(30)
                gripper.close()
                self._wait_for_motors(20)
        else:
            # 没有 gripper，使用默认方式
            print(f"  🦾 无夹爪反馈，使用默认抓取")
            if block_z < 0.1:
                self.set_pose("grasp_low")
            else:
                self.set_pose("grasp")
            self._wait_for_motors(80)
        
        print(f"  🦾 ===== 抓取动作结束 =====\n")
        return True

    def lift_block(self):
        """抬起木块到运输高度"""
        print(f"\n  🦾 ===== 抬起木块 =====")
        self.set_pose("carry")
        self._wait_for_motors(60)
        print(f"  🦾 ===== 抬起完成 =====\n")

    def place_on_table(self):
        """放置到桌面"""
        print(f"\n  🦾 ===== 放置到桌面 =====")
        self.set_pose("place_table")
        self._wait_for_motors(60)
        print(f"  🦾 ===== 放置完成 =====\n")

    def reset(self):
        """复位机械臂"""
        print(f"\n  🦾 ===== 复位机械臂 =====")
        self.set_pose("reset")
        self._wait_for_motors(50)
        print(f"  🦾 ===== 复位完成 =====\n")

    def wait_for_completion(self, timeout=3.0):
        """等待机械臂运动完成"""
        start_time = self.robot.getTime()
        while self.robot.step(self.timestep) != -1:
            elapsed = self.robot.getTime() - start_time
            if elapsed > timeout:
                break
