"""
机械臂控制模块
使用基于精确正运动学的数值逆运动学求解器
"""

import math
import numpy as np
from controller import Robot
from config import TABLE_HEIGHT, TABLE_SURFACE_Z, PLACEMENT

ARM_JOINTS = ["arm1", "arm2", "arm3", "arm4", "arm5"]

# ===== 从 box.wbt 提取的精确机械臂参数 =====
ARM_BASE_X = 0.156  # ARM Solid translation x

# 关节 anchor（相对于父节点）
ANCHORS = {
    "arm1": (0, 0, 0.077),
    "arm2": (0.033, 0, 0.07),
    "arm3": (0, 0, 0.155),
    "arm4": (0, 0, 0.135),
    "arm5": (0, 0, 0.081),
}

# 关节 axis
AXES = {
    "arm1": (0, 0, 1),    # 绕 Z 轴
    "arm2": (0, -1, 0),   # 绕负 Y 轴
    "arm3": (0, -1, 0),   # 绕负 Y 轴
    "arm4": (0, -1, 0),   # 绕负 Y 轴
    "arm5": (0, 0, 1),    # 绕 Z 轴
}

# 夹爪末端相对于 arm5 末端的偏移
GRIPPER_OFFSET = (0, 0.06, 0.09)

# 关节限位
LIMITS = {
    "arm1": (-2.9496, 2.9496),
    "arm2": (-1.13446, 1.5708),
    "arm3": (-2.63545, 2.54818),
    "arm4": (-1.78024, 1.78024),
    "arm5": (-2.92343, 2.92343),
}

# 预设姿态（关节角度，弧度）
POSES = {
    "reset": {
        "arm1": 0.0, "arm2": 1.57, "arm3": -2.635, "arm4": 1.78, "arm5": 0.0,
    },
    "carry": {
        "arm1": 0.0, "arm2": 1.2, "arm3": -1.5, "arm4": 0.3, "arm5": 0.0,
    },
    "pre_grasp": {
        "arm1": 0.0, "arm2": 0.8, "arm3": -1.2, "arm4": 0.0, "arm5": 0.0,
    },
    "grasp": {
        "arm1": 0.0, "arm2": 1.0, "arm3": -1.8, "arm4": 0.0, "arm5": 0.0,
    },
    "grasp_low": {
        "arm1": 0.0, "arm2": 0.0, "arm3": -2.4, "arm4": -0.5, "arm5": 0.0,
    },
    "place_mid": {
        "arm1": 0.0, "arm2": 0.2, "arm3": -0.5, "arm4": 0.0, "arm5": 0.0,
    },
    "place_up": {
        "arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": 0.0, "arm5": 0.0,
    },
    "place_approach": {
        "arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": -1.50, "arm5": 0.0,
    },
    "place_release": {
        "arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": -1.670, "arm5": 0.0,
    },
    "place_retract": {
        "arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": 0.0, "arm5": 0.0,
    },
}


def rotation_matrix(axis, angle):
    """计算绕任意轴旋转 angle 弧度的 3x3 旋转矩阵（Rodrigues 公式）"""
    x, y, z = axis
    length = math.sqrt(x*x + y*y + z*z)
    if length < 1e-10:
        return np.eye(3)
    x /= length; y /= length; z /= length
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    return np.array([
        [t*x*x + c, t*x*y - z*s, t*x*z + y*s],
        [t*x*y + z*s, t*y*y + c, t*y*z - x*s],
        [t*x*z - y*s, t*y*z + x*s, t*z*z + c]
    ])


def forward_kinematics(angles_dict):
    """
    精确正运动学：根据关节角度计算夹爪末端位置
    
    参数:
        angles_dict: 关节角度字典 {arm1, arm2, arm3, arm4, arm5}
    
    返回:
        (x, y, z) 夹爪末端在机器人基座坐标系下的位置
        x: 前后方向（正=前）
        y: 左右方向（正=左）
        z: 上下方向（正=上）
    """
    a1 = angles_dict.get("arm1", 0.0)
    a2 = angles_dict.get("arm2", 0.0)
    a3 = angles_dict.get("arm3", 0.0)
    a4 = angles_dict.get("arm4", 0.0)
    a5 = angles_dict.get("arm5", 0.0)
    
    pos = np.array([ARM_BASE_X, 0.0, 0.0])
    
    # arm1: anchor + translation
    pos += ANCHORS["arm1"]
    R1 = rotation_matrix(AXES["arm1"], a1)
    pos += R1 @ np.array([0, 0, 0.077])
    
    # arm2
    pos += R1 @ np.array(ANCHORS["arm2"])
    R2 = rotation_matrix(AXES["arm2"], a2)
    pos += R1 @ R2 @ np.array([0.033, 0, 0.07])
    
    # arm3
    pos += R1 @ R2 @ np.array(ANCHORS["arm3"])
    R3 = rotation_matrix(AXES["arm3"], a3)
    pos += R1 @ R2 @ R3 @ np.array([0, 0, 0.155])
    
    # arm4
    pos += R1 @ R2 @ R3 @ np.array(ANCHORS["arm4"])
    R4 = rotation_matrix(AXES["arm4"], a4)
    pos += R1 @ R2 @ R3 @ R4 @ np.array([0, 0, 0.135])
    
    # arm5
    pos += R1 @ R2 @ R3 @ R4 @ np.array(ANCHORS["arm5"])
    R5 = rotation_matrix(AXES["arm5"], a5)
    pos += R1 @ R2 @ R3 @ R4 @ R5 @ np.array([0, 0, 0.081])
    
    # 夹爪
    pos += R1 @ R2 @ R3 @ R4 @ R5 @ np.array(GRIPPER_OFFSET)
    
    return (pos[0], pos[1], pos[2])


def ik_numerical(target_pos, initial_guess=None, max_iter=200, tol=1e-3, lr=0.5):
    """
    数值逆运动学求解器（梯度下降法）
    
    参数:
        target_pos: (x, y, z) 目标位置
        initial_guess: 初始角度猜测 [a1, a2, a3, a4, a5]
        max_iter: 最大迭代次数
        tol: 容差 [m]
        lr: 学习率
    
    返回:
        {arm1, arm2, arm3, arm4, arm5} 关节角度字典，或 None（无解）
    """
    if initial_guess is None:
        initial_guess = [0.0, 0.0, -2.0, 0.0, 0.0]
    
    angles = np.array(initial_guess, dtype=float)
    target = np.array(target_pos, dtype=float)
    
    for iteration in range(max_iter):
        # 当前 FK
        angles_dict = {"arm1": angles[0], "arm2": angles[1], 
                       "arm3": angles[2], "arm4": angles[3], "arm5": angles[4]}
        pos = np.array(forward_kinematics(angles_dict))
        error = pos - target
        err_norm = np.linalg.norm(error)
        
        if err_norm < tol:
            break
        
        # 数值雅可比矩阵
        J = np.zeros((3, 5))
        eps = 1e-6
        
        for i in range(5):
            angles_plus = angles.copy()
            angles_plus[i] += eps
            ad = {"arm1": angles_plus[0], "arm2": angles_plus[1],
                  "arm3": angles_plus[2], "arm4": angles_plus[3], "arm5": angles_plus[4]}
            pos_plus = np.array(forward_kinematics(ad))
            J[:, i] = (pos_plus - pos) / eps
        
        # 使用伪逆
        try:
            J_pinv = np.linalg.pinv(J)
            delta = -lr * J_pinv @ error
        except:
            delta = -lr * J.T @ error
        
        angles += delta
        
        # 限位
        for i, name in enumerate(["arm1", "arm2", "arm3", "arm4", "arm5"]):
            lo, hi = LIMITS[name]
            angles[i] = max(lo, min(hi, angles[i]))
    
    if err_norm > 0.05:  # 误差超过 5cm 认为无解
        return None
    
    return {
        "arm1": angles[0],
        "arm2": angles[1],
        "arm3": angles[2],
        "arm4": angles[3],
        "arm5": angles[4],
    }


class ArmController:
    """YouBot 5自由度机械臂控制器（使用数值 IK）"""

    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep

        # 获取5个关节电机
        self.motors = {}
        for name in ARM_JOINTS:
            motor = robot.getDevice(name)
            if motor:
                motor.setVelocity(1.0)
                pos_sensor = motor.getPositionSensor()
                if pos_sensor:
                    pos_sensor.enable(timestep)
                self.motors[name] = motor
            else:
                print(f"  ⚠️ 未找到 {name}")

        # 尝试获取夹爪末端节点
        self.gripper_node = None
        try:
            self.gripper_node = robot.getFromDef("gripper")
            if self.gripper_node:
                print("  ✓ 已获取夹爪节点引用")
        except:
            pass

        # 复位到初始姿态
        self.set_pose("reset")
        self._wait_for_motors(50)

        print("  ✓ 机械臂初始化完成（数值IK算法）")

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

    def print_pose_info(self, pose_name, angles_dict):
        """打印姿态信息"""
        tip_pos = forward_kinematics(angles_dict)
        print(f"  📐 [{pose_name}] 关节角度: "
              f"arm1={angles_dict.get('arm1', 0):.3f}, "
              f"arm2={angles_dict.get('arm2', 0):.3f}, "
              f"arm3={angles_dict.get('arm3', 0):.3f}, "
              f"arm4={angles_dict.get('arm4', 0):.3f}, "
              f"arm5={angles_dict.get('arm5', 0):.3f}")
        print(f"     📍 夹爪末端位置: "
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

    def _wait_for_arm_ready(self, target_angles, tolerance=0.05, max_steps=500, debug=False):
        """等待机械臂所有关节到达目标角度"""
        for step in range(max_steps):
            actual = self.get_actual_angles()
            all_reached = True
            max_diff = 0.0
            worst_joint = ""
            for name, target in target_angles.items():
                if name in actual:
                    diff = abs(actual[name] - target)
                    if diff > max_diff:
                        max_diff = diff
                        worst_joint = name
                    if diff > tolerance:
                        all_reached = False
            if all_reached:
                if debug:
                    print(f"     电机到位 (步数={step})")
                return True
            if debug and step % 100 == 0:
                print(f"     等待电机: {worst_joint} 差={max_diff:.3f}rad (步数={step})")
            self.robot.step(self.timestep)
        
        actual = self.get_actual_angles()
        print(f"  ⚠️ 电机等待超时! 最终状态:")
        for name, target in target_angles.items():
            if name in actual:
                print(f"     {name}: 目标={target:.3f}, 实际={actual[name]:.3f}, 差={abs(actual[name]-target):.3f}")
        return False

    def ik(self, forward, left, up):
        """
        逆运动学求解（封装数值 IK）
        
        参数:
            forward: 前后方向 [m]（正=前）
            left:    左右方向 [m]（正=左）
            up:      上下方向 [m]（正=上）
        """
        # 尝试多个初始猜测
        initial_guesses = [
            [0.0, 0.0, -2.0, 0.0, 0.0],      # 默认
            [0.0, 0.0, -2.4, -0.5, 0.0],     # grasp_low
            [0.0, 0.0, 0.0, 0.0, 0.0],       # place_up
            [0.0, 0.8, -1.2, 0.0, 0.0],      # pre_grasp
            [0.0, 1.0, -1.8, 0.0, 0.0],      # grasp
        ]
        
        best_result = None
        best_error = float('inf')
        
        for init in initial_guesses:
            result = ik_numerical((forward, left, up), initial_guess=init)
            if result is not None:
                # 验证
                pos = forward_kinematics(result)
                error = math.sqrt((pos[0]-forward)**2 + (pos[1]-left)**2 + (pos[2]-up)**2)
                if error < best_error:
                    best_error = error
                    best_result = result
        
        if best_result is None or best_error > 0.05:
            print(f"  ⚠️ IK 无解: 目标 ({forward:.3f}, {left:.3f}, {up:.3f}) 超出工作空间")
            return None
        
        return best_result

    def move_to_ik(self, x, y, z):
        """使用 IK 移动到目标位置"""
        angles = self.ik(x, y, z)
        if angles is None:
            return False
        self.set_joint_angles(angles)
        print(f"  🦾 IK 移动到 ({x:.3f}, {y:.3f}, {z:.3f})")
        self.print_pose_info("IK_target", angles)
        return True

    def _scan_with_arm1(self, gripper):
        """用 arm1 左右摆动扫描，用夹爪传感器检测木块"""
        gripper.open()
        self._wait_for_motors(10)
        self.set_pose("grasp_low")
        self._wait_for_motors(40)
        
        scan_range = 0.25
        scan_step = 0.03
        
        for angle in [i * scan_step for i in range(int(-scan_range/scan_step), int(scan_range/scan_step) + 1)]:
            self.motors["arm1"].setPosition(angle)
            self._wait_for_motors(15)
            gripper.close()
            self._wait_for_motors(8)
            if gripper.wait_for_grasp(timeout_steps=30):
                return angle
            gripper.open()
            self._wait_for_motors(8)
        
        return 0.0

    def side_grasp_block(self, block_forward, block_left, block_z=0.05, gripper=None):
        """从侧面抓取木块"""
        print(f"\n  🦾 ===== 侧面抓取开始 =====")
        print(f"  🎯 木块相对位置: 前={block_forward:.3f}m, 左={block_left:.3f}m")
        
        base_forward = block_forward - ARM_BASE_X
        base_left = block_left
        arm1_target = math.atan2(base_left, base_forward)
        dist_to_block = math.sqrt(base_forward**2 + base_left**2)
        
        print(f"  📐 arm1 目标角度: {arm1_target:.3f} rad")
        print(f"  📏 基座到木块距离: {dist_to_block:.3f} m")
        
        self.motors["arm1"].setPosition(arm1_target)
        self._wait_for_motors(40)
        
        angles = self.ik(dist_to_block, 0, block_z)
        if angles is None:
            angles = self.ik(dist_to_block + 0.02, 0, block_z)
        
        if angles is None:
            print(f"  ❌ IK 无解，使用预设姿态")
            self.set_pose("grasp_low")
            self._wait_for_motors(60)
        else:
            print(f"  🦾 移动到预抓取位置")
            self.set_joint_angles(angles)
            self._wait_for_motors(60)
        
        if gripper is not None:
            self._probe_and_grasp(gripper)
        
        print(f"  🦾 ===== 侧面抓取结束 =====\n")

    def _probe_and_grasp(self, gripper):
        """逐步下探 + 夹爪检测抓取"""
        gripper.open()
        self._wait_for_motors(20)
        
        current_arm2 = self.get_actual_angles().get("arm2", 0.0)
        arm2_min = -1.13446
        arm2_step = -0.02
        
        contact_count = 0
        max_contacts = 6
        
        while current_arm2 >= arm2_min and contact_count < max_contacts:
            self.motors["arm2"].setPosition(current_arm2)
            self._wait_for_motors(15)
            
            gripper.close()
            self._wait_for_motors(8)
            
            if gripper.wait_for_grasp(timeout_steps=40):
                contact_count += 1
                if contact_count >= max_contacts:
                    print(f"  ✅ 成功抓取! (arm2={current_arm2:.3f})")
                    return True
                gripper.open()
                self._wait_for_motors(8)
            
            gripper.open()
            self._wait_for_motors(5)
            current_arm2 += arm2_step
        
        return contact_count > 0

    def grasp_block(self, block_x, block_y, block_z=0.05, gripper=None, offset_y=0.0):
        """
        抓取木块（使用 IK 精确移动到木块中心正上方 + 逐步下探）
        
        策略：
        1. 用 arm1 扫描检测木块偏移
        2. pre_grasp: 抬起机械臂
        3. 用 IK 精确移动到木块中心正上方 0.15m
        4. 逐步下探：用 arm2 逐步降低夹爪，每次尝试闭合检测
        """
        print(f"\n  🦾 ===== 抓取动作开始 =====")
        print(f"  🎯 目标木块位置: 前={block_x:.3f}m, 左={block_y:.3f}m, 高={block_z:.3f}m")
        
        # 阶段0: 用 arm1 扫描检测木块偏移
        detected_arm1_offset = 0.0
        if gripper is not None:
            detected_arm1_offset = self._scan_with_arm1(gripper)
            if abs(detected_arm1_offset) > 0.01:
                print(f"  🦾 检测到木块偏移，arm1 补偿 {detected_arm1_offset:.3f}rad")
            gripper.open()
            self._wait_for_motors(15)
        
        # 阶段1: 准备姿态
        print(f"  🦾 阶段1: 准备抓取姿态")
        self.set_pose("pre_grasp")
        self._wait_for_motors(60)

        # 阶段2: 低姿态接近
        print(f"  🦾 阶段2: 低姿态接近")
        self.set_pose("grasp_low")
        if abs(detected_arm1_offset) > 0.01:
            current_arm1 = POSES["grasp_low"]["arm1"] + detected_arm1_offset
            self.motors["arm1"].setPosition(current_arm1)
            print(f"     叠加 arm1 偏移: {current_arm1:.3f}rad")
        self._wait_for_motors(80)
        
        # 阶段2.5: 用 IK 精确移动到木块中心正上方
        print(f"  🦾 阶段2.5: IK 精确移动到木块中心正上方")
        ik_target_z = block_z + 0.15  # 木块上方 0.15m
        angles = self.ik(block_x, block_y, ik_target_z)
        if angles:
            self.set_joint_angles(angles)
            self._wait_for_motors(60)
            actual_angles = self.get_actual_angles()
            tip_pos = forward_kinematics(actual_angles)
            print(f"     IK 到达位置: 前={tip_pos[0]:.3f}m, 左={tip_pos[1]:.3f}m, 高={tip_pos[2]:.3f}m")
            print(f"     目标位置: 前={block_x:.3f}m, 左={block_y:.3f}m, 高={ik_target_z:.3f}m")
        else:
            print(f"     ⚠️ IK 无解，保持 grasp_low 姿态")
        
        # 阶段3: 逐步下探
        if gripper is not None:
            print(f"  🦾 阶段3: 逐步下探寻找木块")
            gripper.open()
            self._wait_for_motors(30)
            
            current_angles = self.get_actual_angles()
            current_arm2 = current_angles.get("arm2", 0.0)
            arm2_end = -1.13446
            arm2_step = -0.02
            
            found = False
            contact_count = 0
            max_contacts = 6
            
            while current_arm2 >= arm2_end and not found:
                self.motors["arm2"].setPosition(current_arm2)
                self._wait_for_motors(20)
                
                gripper.close()
                self._wait_for_motors(8)
                
                if gripper.wait_for_grasp(timeout_steps=50):
                    contact_count += 1
                    if contact_count < max_contacts:
                        gripper.open()
                        self._wait_for_motors(10)
                        current_arm2 += arm2_step
                        continue
                    else:
                        print(f"  ✅ 夹紧木块! (arm2={current_arm2:.3f})")
                        found = True
                        break
                
                gripper.open()
                self._wait_for_motors(8)
                current_arm2 += arm2_step
            
            if not found:
                print(f"  🦾 arm2 下探未找到，尝试调整 arm4")
                self.motors["arm2"].setPosition(0.0)
                self._wait_for_motors(30)
                
                arm4_start = -0.5
                arm4_end = -1.2
                arm4_step = -0.1
                current_arm4 = arm4_start
                
                while current_arm4 >= arm4_end and not found:
                    self.motors["arm4"].setPosition(current_arm4)
                    self._wait_for_motors(30)
                    
                    gripper.close()
                    self._wait_for_motors(10)
                    
                    if gripper.wait_for_grasp(timeout_steps=80):
                        print(f"  ✅ 在 arm4 下探过程中夹到木块!")
                        found = True
                        break
                    
                    gripper.open()
                    self._wait_for_motors(10)
                    current_arm4 += arm4_step
            
            if not found:
                print(f"  ⚠️ 下探未找到木块，使用默认姿态")
                self.set_pose("grasp_low")
                self._wait_for_motors(30)
                gripper.close()
                self._wait_for_motors(20)
        else:
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

    def place_on_table(self, gripper=None):
        """
        平稳放置木块到桌面（增强版）
        
        策略：
        1. 从 carry 过渡到 place_up（竖直向上姿态）
        2. 保持 arm2=0, arm3=0，逐步弯曲 arm4 降低夹爪
        3. 分步释放木块
        4. 抬升夹爪
        """
        print(f"\n  🦾 ===== 平稳放置到桌面（增强版） =====")
        print(f"  📐 桌面高度: {TABLE_SURFACE_Z:.3f}m")
        
        current_angles = self.get_actual_angles()
        current_pos = forward_kinematics(current_angles)
        current_height = current_pos[2]
        print(f"  📏 当前夹爪高度: {current_height:.3f}m")
        
        # 步骤1: 过渡到 place_mid
        print(f"  🦾 步骤1: 过渡到中间姿态 place_mid")
        self.set_pose("place_mid")
        self._wait_for_arm_ready(POSES["place_mid"], tolerance=0.15, max_steps=800, debug=True)
        
        current_angles = self.get_actual_angles()
        current_pos = forward_kinematics(current_angles)
        current_height = current_pos[2]
        print(f"     到达高度: {current_height:.3f}m")
        
        # 步骤1.5: 过渡到 place_up
        print(f"  🦾 步骤1.5: 过渡到竖直向上姿态 place_up")
        self.set_pose("place_up")
        self._wait_for_arm_ready(POSES["place_up"], tolerance=0.10, max_steps=600, debug=True)
        
        current_angles = self.get_actual_angles()
        current_pos = forward_kinematics(current_angles)
        current_height = current_pos[2]
        print(f"     竖直高度: {current_height:.3f}m")
        
        # 步骤2: 逐步弯曲 arm4 降低夹爪
        print(f"  🦾 步骤2: 保持 arm2=0, arm3=0，逐步弯曲 arm4 降低夹爪")
        
        target_height = TABLE_SURFACE_Z + 0.06
        
        current_angles = self.get_actual_angles()
        current_arm4 = current_angles.get("arm4", 0.0)
        arm4_min = -1.75
        arm4_step = -0.02
        
        reached_target = False
        overshoot_count = 0
        
        while current_arm4 >= arm4_min and not reached_target and overshoot_count < 3:
            self.motors["arm4"].setPosition(current_arm4)
            self._wait_for_motors(15)
            
            current_angles = self.get_actual_angles()
            current_pos = forward_kinematics(current_angles)
            current_height = current_pos[2]
            
            if current_height <= target_height:
                if overshoot_count == 0:
                    print(f"     首次到达目标高度: {current_height:.3f}m (arm4={current_arm4:.3f})")
                overshoot_count += 1
                if overshoot_count >= 3:
                    reached_target = True
                    print(f"     确认到达目标高度: {current_height:.3f}m (arm4={current_arm4:.3f})")
                    break
            else:
                overshoot_count = 0
            
            current_arm4 += arm4_step
        
        if not reached_target:
            print(f"  ⚠️ arm4 到限位或未到达目标高度，当前高度={current_height:.3f}m")
            self._wait_for_motors(100)
        
        current_angles = self.get_actual_angles()
        current_pos = forward_kinematics(current_angles)
        final_height = current_pos[2]
        print(f"     最终高度: {final_height:.3f}m (目标: {target_height:.3f}m, 桌面: {TABLE_SURFACE_Z:.3f}m)")
        
        # 步骤3: 分步释放
        print(f"  🦾 步骤3: 缓慢分步释放木块")
        if gripper is not None:
            print(f"     第1步: 微张夹爪到 10mm")
            gripper.set_gap(0.01)
            self._wait_for_motors(40)
            
            print(f"     第2步: 半张夹爪到 30mm")
            gripper.set_gap(0.03)
            self._wait_for_motors(40)
            
            print(f"     第3步: 完全张开夹爪到 60mm")
            gripper.set_gap(0.06)
            self._wait_for_motors(50)
            
            print(f"     🖐 夹爪已完全张开，木块已释放到桌面")
        else:
            print(f"     ⚠️ 无夹爪控制，跳过释放")
        
        # 步骤4: 抬升夹爪
        print(f"  🦾 步骤4: 抬升夹爪")
        self.set_pose("place_up")
        self._wait_for_arm_ready(POSES["place_up"], tolerance=0.08, max_steps=200)
        
        current_angles = self.get_actual_angles()
        current_pos = forward_kinematics(current_angles)
        retract_height = current_pos[2]
        print(f"     抬升后高度: {retract_height:.3f}m")
        
        print(f"  🦾 ===== 平稳放置完成 =====\n")

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
