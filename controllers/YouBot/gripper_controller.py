"""
夹爪控制模块
基于 YouBot C 库中的 gripper.c 实现
"""

from controller import Robot
from config import GRIPPER_MIN, GRIPPER_MAX, GRIPPER_SPEED


class GripperController:
    """YouBot 夹爪控制器"""
    
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # 获取夹爪电机（名称来自 C 库 gripper.c）
        self.finger = robot.getDevice("finger::left")
        if self.finger:
            self.finger.setVelocity(GRIPPER_SPEED)  # 与 C 库一致: 0.03
            self.finger.setPosition(GRIPPER_MAX)     # 初始张开
            print("  ✓ 夹爪初始化完成")
        else:
            print("  ⚠️ 未找到 finger::left")
    
    def close(self):
        """闭合夹爪（与 C 库 gripper_grip 一致）"""
        if self.finger:
            self.finger.setPosition(GRIPPER_MIN)  # 0.0 = 夹紧
            print("  ✊ 夹爪闭合")
    
    def open(self):
        """张开夹爪（与 C 库 gripper_release 一致）"""
        if self.finger:
            self.finger.setPosition(GRIPPER_MAX)  # 0.025 = 张开
            print("  🖐 夹爪张开")
    
    def set_gap(self, gap):
        """设置夹爪开度（与 C 库 gripper_set_gap 一致）"""
        if self.finger:
            # 从 C 库: v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS)
            OFFSET_WHEN_LOCKED = 0.021
            v = 0.5 * (gap - OFFSET_WHEN_LOCKED)
            v = max(GRIPPER_MIN, min(GRIPPER_MAX, v))
            self.finger.setPosition(v)
