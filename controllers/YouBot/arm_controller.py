"""
机械臂控制模块
包含逆运动学求解和预设姿态控制
基于 YouBot C 库中的 arm_ik 算法（从 arm.c 移植）
"""

from controller import Robot
import math
from config import ARM_LENGTHS


class ArmController:
    """YouBot 5自由度机械臂控制器"""
    
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # 获取5个关节电机（名称来自 C 库 arm.c）
        self.arm_motors = []
        for i in range(1, 6):
            motor = robot.getDevice(f"arm{i}")
            if motor:
                motor.setVelocity(0.5)  # 与 C 库一致
                self.arm_motors.append(motor)
            else:
                print(f"  ⚠️ 未找到 arm{i}")
        
        # 复位到初始姿态
        self.set_pose("reset")
        
        print("  ✓ 机械臂初始化完成")
    
    def set_joint_positions(self, positions):
        """设置5个关节的目标位置"""
        for i, motor in enumerate(self.arm_motors):
            if motor and i < len(positions):
                motor.setPosition(positions[i])
    
    def set_pose(self, pose_name):
        """设置预设姿态（从 C 库 arm.c 移植）"""
        if pose_name == "reset":
            # ARM_RESET
            self.set_joint_positions([0.0, 1.57, -2.635, 1.78, 0.0])
        elif pose_name == "front_floor":
            # ARM_FRONT_FLOOR
            self.set_joint_positions([0.0, -0.97, -1.55, -0.61, 0.0])
        elif pose_name == "front_plate":
            # ARM_FRONT_PLATE
            self.set_joint_positions([0.0, -0.62, -0.98, -1.53, 0.0])
        elif pose_name == "front_box":
            # ARM_FRONT_CARDBOARD_BOX
            self.set_joint_positions([0.0, 0.0, -0.77, -1.21, 0.0])
        elif pose_name == "back_plate_low":
            # ARM_BACK_PLATE_LOW
            self.set_joint_positions([0.0, 0.92, 0.42, 1.78, 0.0])
        elif pose_name == "back_plate_high":
            # ARM_BACK_PLATE_HIGH
            self.set_joint_positions([0.0, 0.678, 0.682, 1.74, 0.0])
        elif pose_name == "carry":
            # 运输姿态 - 抬起木块
            self.set_joint_positions([0.0, 0.5, -1.2, 1.2, 0.0])
        elif pose_name == "place_table":
            # 放置到桌面
            self.set_joint_positions([0.0, -0.4, -1.5, -0.8, 0.0])
        else:
            print(f"  ⚠️ 未知姿态: {pose_name}")
            return False
        
        print(f"  🦾 机械臂 -> {pose_name}")
        return True
    
    def arm_ik(self, x, y, z):
        """
        逆运动学求解（从 C 库 arm.c 的 arm_ik 函数移植）
        参数: 末端执行器在机器人基座坐标系下的目标位置 (x, y, z)
        """
        # 计算在水平面上的投影距离
        y1 = math.sqrt(x * x + y * y)
        if y1 < 0.001:
            y1 = 0.001  # 防止除零
        
        # 计算垂直方向的距离（考虑基座和末端高度）
        z1 = z + ARM_LENGTHS["arm4"] + ARM_LENGTHS["arm5"] - ARM_LENGTHS["arm1"]
        
        a = ARM_LENGTHS["arm2"]  # 上臂长度 0.155
        b = ARM_LENGTHS["arm3"]  # 前臂长度 0.135
        c = math.sqrt(y1 * y1 + z1 * z1)  # 肩到腕的距离
        
        # 检查可达性
        if c > a + b or c < abs(a - b):
            print(f"  ⚠️ 目标位置不可达: 距离={c:.3f}, 最大={a+b:.3f}")
            return False
        
        # 计算各关节角度（与 C 库完全一致）
        alpha = -math.asin(x / y1)  # arm1: 基座旋转
        
        cos_beta = (a * a + c * c - b * b) / (2.0 * a * c)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = -(math.pi / 2 - math.acos(cos_beta) - math.atan2(z1, y1))  # arm2: 肩关节
        
        cos_gamma = (a * a + b * b - c * c) / (2.0 * a * b)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = -(math.pi - math.acos(cos_gamma))  # arm3: 肘关节
        
        delta = -(math.pi + (beta + gamma))  # arm4: 腕关节俯仰
        
        epsilon = math.pi / 2 + alpha  # arm5: 腕关节旋转
        
        # 应用关节角度
        positions = [alpha, beta, gamma, delta, epsilon]
        self.set_joint_positions(positions)
        
        return True
    
    def grasp_position(self, block_x, block_y, block_z=0.05):
        """
        计算抓取木块所需的机械臂姿态
        block_x, block_y: 木块相对于机器人基座的位置
        block_z: 木块高度（默认0.05m，即木块中心高度）
        """
        target_x = block_x
        target_y = block_y
        target_z = block_z + 0.02  # 稍微高一点
        return self.arm_ik(target_x, target_y, target_z)
    
    def lift_block(self):
        """抬起木块到运输高度"""
        self.set_pose("carry")
    
    def place_on_table(self):
        """放置到桌面"""
        self.set_pose("place_table")
    
    def reset(self):
        """复位机械臂"""
        self.set_pose("reset")
    
    def wait_for_completion(self, timeout=3.0):
        """等待机械臂运动完成"""
        start_time = self.robot.getTime()
        while self.robot.step(self.timestep) != -1:
            elapsed = self.robot.getTime() - start_time
            if elapsed > timeout:
                break
