"""
有限状态机（FSM）任务调度模块
管理 YouBot 从初始化到完成任务的完整状态流转
"""

import math
import json
import requests
from config import (
    FSM_STATES, NAVIGATION, COLOR_NAMES,
    DEEPSEEK_API_KEY, DEEPSEEK_API_URL
)


class FSM:
    """有限状态机 - 任务调度核心"""
    
    def __init__(self, robot, drive, arm, gripper, perception):
        self.robot = robot
        self.drive = drive
        self.arm = arm
        self.gripper = gripper
        self.perception = perception
        
        # 状态
        self.state = "INIT"
        self.previous_state = None
        self.state_start_time = robot.getTime()
        
        # 任务数据
        self.grasp_order = []
        self.block_positions = {}
        self.table_position = (0.0, 0.0)
        self.current_target_index = 0
        self.current_color = None
        
        # 状态计时器
        self.state_timer = 0.0
        self.state_timeout = 10.0
        
        # 子状态
        self.approach_phase = "rotate"  # rotate / forward / final
        
        print("  ✓ 有限状态机初始化完成")
    
    def set_mission_data(self, grasp_order, block_positions, table_position):
        """设置任务数据"""
        self.grasp_order = grasp_order
        self.block_positions = block_positions
        self.table_position = table_position
        
        print(f"\n📋 任务数据已加载:")
        print(f"  抓取顺序: {', '.join([COLOR_NAMES.get(c, c) for c in grasp_order])}")
        print(f"  木块位置: {block_positions}")
        print(f"  桌面位置: {table_position}")
    
    def _transition_to(self, new_state):
        """状态转换"""
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self.state_start_time = self.robot.getTime()
            color_name = COLOR_NAMES.get(self.current_color, self.current_color) if self.current_color else ""
            print(f"\n{'='*50}")
            print(f"🔄 状态: {self.previous_state} → {new_state} [{color_name}]")
            print(f"{'='*50}")
    
    def _get_state_time(self):
        """获取当前状态已运行时间"""
        return self.robot.getTime() - self.state_start_time
    
    def _is_timeout(self, timeout=None):
        """检查是否超时"""
        if timeout is None:
            timeout = self.state_timeout
        return self._get_state_time() > timeout
    
    def _get_block_position(self, color):
        """获取指定颜色木块的位置"""
        return self.block_positions.get(color, (0.0, 0.0))
    
    def _get_relative_position(self, target_x, target_y):
        """计算目标相对于机器人的位置"""
        robot_pos = self.drive.get_position()
        robot_angle = self.drive.get_orientation()
        
        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        
        # 转换到机器人坐标系
        cos_a = math.cos(-robot_angle)
        sin_a = math.sin(-robot_angle)
        rel_x = dx * cos_a - dy * sin_a
        rel_y = dx * sin_a + dy * cos_a
        
        return rel_x, rel_y
    
    def run(self):
        """运行状态机（每步调用）"""
        if self.state == "COMPLETED":
            return True, False
        
        if self.state == "ERROR":
            return False, True
        
        # 执行当前状态
        state_method = getattr(self, f"_state_{self.state}", None)
        if state_method:
            state_method()
        
        return False, False
    
    def _state_INIT(self):
        """初始化状态"""
        self.gripper.open()
        self.arm.reset()
        self._transition_to("WAIT_ORDER")
    
    def _state_WAIT_ORDER(self):
        """等待任务指令"""
        if self.grasp_order and self.current_target_index < len(self.grasp_order):
            self.current_color = self.grasp_order[self.current_target_index]
            self._transition_to("NAVIGATE_TO_BLOCK")
    
    def _state_NAVIGATE_TO_BLOCK(self):
        """导航到木块位置"""
        if self.current_color is None:
            self._transition_to("ERROR")
            return
        
        block_pos = self._get_block_position(self.current_color)
        
        # 计算接近点（在木块前方一定距离）
        approach_dist = NAVIGATION["approach_distance"]
        # 从机器人到木块的方向
        robot_pos = self.drive.get_position()
        dx = block_pos[0] - robot_pos[0]
        dy = block_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        # 调试输出
        if int(self.robot.getTime() * 2) % 10 == 0:
            print(f"  📍 位置: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}), 目标: {block_pos}, 距离: {dist:.2f}m")
        
        if dist < approach_dist:
            # 已经足够接近，直接进入接近阶段
            print(f"  ✅ 已到达木块附近 (距离={dist:.2f}m)")
            self._transition_to("APPROACH_BLOCK")
            return
        
        # 计算目标点（木块前方 approach_dist 处）
        if dist > 0.01:
            target_x = block_pos[0] - (dx / dist) * approach_dist
            target_y = block_pos[1] - (dy / dist) * approach_dist
        else:
            target_x = block_pos[0]
            target_y = block_pos[1]
        
        # 计算朝向木块的角度
        target_angle = math.atan2(dy, dx)
        
        # 开始导航
        if not self.drive.is_navigation_active():
            print(f"  🧭 开始导航到 ({target_x:.2f}, {target_y:.2f})")
            self.drive.start_navigation(target_x, target_y, target_angle)
        
        reached = self.drive.run_navigation()
        
        if reached:
            print(f"  ✅ 导航完成")
            self._transition_to("APPROACH_BLOCK")
        
        # 超时处理
        if self._is_timeout(60.0):
            print(f"  ⚠️ 导航到木块超时")
            self._transition_to("ERROR")
    
    def _state_APPROACH_BLOCK(self):
        """接近木块到抓取距离"""
        if self.current_color is None:
            self._transition_to("ERROR")
            return
        
        block_pos = self._get_block_position(self.current_color)
        robot_pos = self.drive.get_position()
        
        dx = block_pos[0] - robot_pos[0]
        dy = block_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        # 调试输出
        if int(self.robot.getTime() * 2) % 10 == 0:
            print(f"  🎯 接近中: 距离木块 {dist:.3f}m")
        
        grasp_dist = NAVIGATION["grasp_distance"]
        
        if dist <= grasp_dist:
            # 到达抓取距离，停止并准备抓取
            self.drive.stop()
            print(f"  ✅ 到达抓取距离 ({dist:.3f}m)")
            self._transition_to("GRASP")
            return
        
        # 缓慢前进
        angle = math.atan2(dy, dx)
        robot_angle = self.drive.get_orientation()
        angle_diff = angle - robot_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 提高接近速度
        speed = min(0.2, (dist - grasp_dist) * 1.0 + 0.05)
        vx = speed * math.cos(angle_diff)
        vy = speed * math.sin(angle_diff)
        omega = 1.5 * angle_diff
        
        self.drive.move(vx, vy, omega)
        
        # 超时
        if self._is_timeout(15.0):
            print(f"  ⚠️ 接近木块超时")
            self._transition_to("ERROR")
    
    def _state_GRASP(self):
        """抓取木块"""
        # 计算木块相对于机器人的位置
        block_pos = self._get_block_position(self.current_color)
        rel_x, rel_y = self._get_relative_position(block_pos[0], block_pos[1])
        
        # 使用逆运动学定位机械臂到木块位置
        self.arm.grasp_position(rel_x, rel_y, 0.05)
        self.robot.step(32 * 30)  # 等待机械臂到位
        
        # 闭合夹爪
        self.gripper.close()
        self.robot.step(32 * 20)  # 等待夹爪闭合
        
        print(f"  ✊ 已抓取 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块")
        self._transition_to("LIFT")
    
    def _state_LIFT(self):
        """抬起木块"""
        self.arm.lift_block()
        self.robot.step(32 * 30)  # 等待机械臂抬起
        
        self._transition_to("NAVIGATE_TO_TABLE")
    
    def _state_NAVIGATE_TO_TABLE(self):
        """导航到桌子"""
        table_pos = self.table_position
        
        # 计算接近点（桌子前方）
        approach_dist = NAVIGATION["table_approach_distance"]
        robot_pos = self.drive.get_position()
        dx = table_pos[0] - robot_pos[0]
        dy = table_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < approach_dist:
            self._transition_to("PLACE_ON_TABLE")
            return
        
        # 计算目标点
        if dist > 0.01:
            target_x = table_pos[0] - (dx / dist) * approach_dist
            target_y = table_pos[1] - (dy / dist) * approach_dist
        else:
            target_x = table_pos[0]
            target_y = table_pos[1]
        
        target_angle = math.atan2(dy, dx)
        
        if not self.drive.is_navigation_active():
            self.drive.start_navigation(target_x, target_y, target_angle)
        
        reached = self.drive.run_navigation()
        
        if reached:
            self._transition_to("PLACE_ON_TABLE")
        
        if self._is_timeout(60.0):
            print(f"  ⚠️ 导航到桌子超时")
            self._transition_to("ERROR")
    
    def _state_PLACE_ON_TABLE(self):
        """放置木块到桌面"""
        # 将机械臂伸展到桌面位置
        self.arm.place_on_table()
        self.robot.step(32 * 30)  # 等待机械臂到位
        
        self._transition_to("RELEASE")
    
    def _state_RELEASE(self):
        """释放木块"""
        self.gripper.open()
        self.robot.step(32 * 20)  # 等待夹爪张开
        
        print(f"  🖐 已放置 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块到桌面")
        self._transition_to("BACK_OFF")
    
    def _state_BACK_OFF(self):
        """后退离开桌子"""
        self.arm.reset()
        self.robot.step(32 * 30)  # 等待机械臂复位
        
        # 后退一段距离
        self.drive.move(-0.1, 0.0, 0.0)
        self.robot.step(32 * 20)
        self.drive.stop()
        
        # 移动到下一个目标
        self.current_target_index += 1
        self.current_color = None
        
        if self.current_target_index >= len(self.grasp_order):
            self._transition_to("COMPLETED")
        else:
            self.current_color = self.grasp_order[self.current_target_index]
            self._transition_to("NAVIGATE_TO_BLOCK")
    
    def _state_COMPLETED(self):
        """任务完成"""
        pass
    
    def _state_ERROR(self):
        """错误状态"""
        print(f"  ❌ 状态机进入错误状态")
        self.drive.stop()
