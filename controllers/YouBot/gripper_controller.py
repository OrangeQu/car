"""
夹爪控制模块
使用 YouBot 默认夹爪（finger::left / finger::right）
通过 Supervisor API 修改夹爪 maxPosition 以夹住 10cm 木块
"""

from controller import Robot
from config import GRIPPER_MIN, GRIPPER_MAX, GRIPPER_SPEED


class GripperController:
    """YouBot 夹爪控制器"""
    
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # 夹爪状态
        self.last_position = GRIPPER_MAX
        self.grasp_detected = False
        
        # 获取左右两个夹爪电机
        # 左手指沿 Y 正方向移动，右手指沿 Y 负方向移动
        self.finger_left = robot.getDevice("finger::left")
        self.finger_right = robot.getDevice("finger::right")
        
        if self.finger_left and self.finger_right:
            self.finger_left.setVelocity(GRIPPER_SPEED)
            self.finger_right.setVelocity(GRIPPER_SPEED)
            self.finger_left.setPosition(GRIPPER_MAX)  # 初始张开
            self.finger_right.setPosition(GRIPPER_MAX)  # 初始张开
            print(f"  ✓ 夹爪初始化完成（开度 {GRIPPER_MAX*1000:.0f}mm）")
        else:
            print("  ⚠️ 未找到夹爪设备")
            # 兼容旧版本：只使用左手指
            self.finger_left = robot.getDevice("finger::left")
            if self.finger_left:
                self.finger_left.setVelocity(GRIPPER_SPEED)
                self.finger_left.setPosition(GRIPPER_MAX)
                print(f"  ✓ 夹爪初始化完成（单手指，开度 {GRIPPER_MAX*1000:.0f}mm）")
        
        # 获取左右两个夹爪传感器
        self.left_sensor = robot.getDevice("finger::leftsensor")
        if not self.left_sensor:
            self.left_sensor = robot.getDevice("finger::left_sensor")
        if self.left_sensor:
            self.left_sensor.enable(timestep)
            print("  ✓ 左夹爪传感器已启用")
        
        self.right_sensor = robot.getDevice("finger::rightsensor")
        if not self.right_sensor:
            self.right_sensor = robot.getDevice("finger::right_sensor")
        if self.right_sensor:
            self.right_sensor.enable(timestep)
            print("  ✓ 右夹爪传感器已启用")
    
    def close(self):
        """闭合夹爪（左右手指同时闭合）"""
        if self.finger_left:
            self.finger_left.setPosition(GRIPPER_MIN)
        if self.finger_right:
            self.finger_right.setPosition(GRIPPER_MIN)
        print("  ✊ 夹爪闭合")
    
    def open(self):
        """张开夹爪（左右手指同时张开）"""
        if self.finger_left:
            self.finger_left.setPosition(GRIPPER_MAX)
        if self.finger_right:
            self.finger_right.setPosition(GRIPPER_MAX)
        print("  🖐 夹爪张开")
    
    def set_gap(self, gap):
        """设置夹爪开度"""
        v = max(GRIPPER_MIN, min(GRIPPER_MAX, gap))
        if self.finger_left:
            self.finger_left.setPosition(v)
        if self.finger_right:
            self.finger_right.setPosition(v)
    
    def _get_sensor_positions(self):
        """获取左右两个传感器的位置值"""
        left_pos = None
        right_pos = None
        try:
            if self.left_sensor:
                left_pos = self.left_sensor.getValue()
        except:
            pass
        try:
            if self.right_sensor:
                right_pos = self.right_sensor.getValue()
        except:
            pass
        return left_pos, right_pos
    
    def wait_for_grasp(self, timeout_steps=250):
        """
        等待夹爪闭合，检测是否夹到物体
        
        检测原理：
        夹爪空载闭合时，两个手指会一直移动到接近 0 的位置。
        夹到物体时，手指被物体挡住，会停在中间值（0.01~0.04）。
        
        判断条件：
        1. 两个手指都停在 0.01~0.04 之间 → 夹到物体了
        2. 两个手指都接近 0 → 空载，没夹到
        3. 只有一个手指有阻力 → 木块被推偏了
        
        返回: True=夹到了, False=没夹到
        """
        # 记录初始位置
        left_init, right_init = self._get_sensor_positions()
        if left_init is None:
            left_init = GRIPPER_MAX
        if right_init is None:
            right_init = GRIPPER_MAX
        print(f"  📐 夹爪初始位置: left={left_init:.4f}, right={right_init:.4f}")
        
        # 先等待几帧让夹爪开始移动
        for _ in range(20):
            self.robot.step(self.timestep)
        
        # 检测位置变化
        for i in range(timeout_steps):
            self.robot.step(self.timestep)
            
            left_pos, right_pos = self._get_sensor_positions()
            
            # 如果两个传感器都有效
            if left_pos is not None and right_pos is not None:
                # 两个手指都接近 0 → 没夹到物体（空载闭合）
                if left_pos < 0.008 and right_pos < 0.008:
                    print(f"  ⚠️ 抓取检测: 未夹到物体 (left={left_pos:.4f}, right={right_pos:.4f})")
                    self.grasp_detected = False
                    return False
                
                # 两个手指都在 0.01~0.04 之间（被物体挡住）
                left_blocked = left_pos > 0.01 and left_pos < 0.04
                right_blocked = right_pos > 0.01 and right_pos < 0.04
                
                if left_blocked and right_blocked:
                    # 两个手指都被挡住，说明夹到物体了
                    # 注意：夹爪张开 60mm，木块 100mm 宽
                    # 夹爪从上方压下时，两个手指会碰到木块两侧
                    # 如果两个手指都停在中间值，说明确实夹住了木块
                    print(f"  ✅ 抓取检测: 夹到物体! (left={left_pos:.4f}, right={right_pos:.4f})")
                    self.grasp_detected = True
                    return True
                
                # 只有一个手指有阻力 → 木块被推偏了
                if (left_pos > 0.01 and right_pos < 0.008) or (right_pos > 0.01 and left_pos < 0.008):
                    if i > 40:  # 等待一段时间再判断
                        print(f"  ⚠️ 抓取检测: 木块被推偏 (left={left_pos:.4f}, right={right_pos:.4f})")
                        self.grasp_detected = False
                        return False
            
            # 如果只有一个传感器，用旧逻辑
            elif left_pos is not None:
                if left_pos > 0.01 and left_pos < 0.04:
                    print(f"  ✅ 抓取检测(单传感器): 夹到物体! (left={left_pos:.4f})")
                    self.grasp_detected = True
                    return True
                if left_pos < 0.008:
                    print(f"  ⚠️ 抓取检测(单传感器): 未夹到物体 (left={left_pos:.4f})")
                    self.grasp_detected = False
                    return False
        
        # 超时
        left_pos, right_pos = self._get_sensor_positions()
        print(f"  ⏰ 抓取检测超时 (left={left_pos}, right={right_pos})")
        self.grasp_detected = False
        return False
