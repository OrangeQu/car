"""
麦克纳姆轮底盘控制模块
基于 YouBot C 库中的运动学公式（从 base.c 移植）
"""

from controller import Robot
import math
from config import WHEEL_RADIUS, LX, LY, MAX_WHEEL_SPEED, NAVIGATION


class MecanumDrive:
    """麦克纳姆轮底盘控制器"""
    
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # Supervisor API 引用（由主控制器设置）
        self.position_field = None
        self.rotation_field = None
        
        # 获取四个轮子电机（名称来自 C 库 base.c）
        self.wheels = []
        for i in range(1, 5):
            wheel = robot.getDevice(f"wheel{i}")
            if wheel:
                wheel.setPosition(float('inf'))  # 速度控制模式
                wheel.setVelocity(0.0)
                self.wheels.append(wheel)
            else:
                print(f"  ⚠️ 未找到 wheel{i}")
        
        # 尝试获取 GPS 和 Compass（C 库中 base_goto_init 会获取）
        self.gps = robot.getDevice("gps")
        self.compass = robot.getDevice("compass")
        if self.gps:
            self.gps.enable(timestep)
            print("  ✓ GPS 已启用")
        else:
            print("  ⚠️ 未找到 GPS，将使用 Supervisor API 获取位置")
        
        if self.compass:
            self.compass.enable(timestep)
            print("  ✓ Compass 已启用")
        else:
            print("  ⚠️ 未找到 Compass，将使用 Supervisor API 获取朝向")
        
        # 里程计推算位置（当没有 GPS/Supervisor 时使用）
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_angle = 0.0
        self.last_time = robot.getTime()
        
        # 状态
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # 导航目标
        self.target_x = None
        self.target_y = None
        self.target_angle = None
        self.navigation_active = False
        self.navigation_start_time = None
        
        print("  ✓ 底盘初始化完成")
    
    def _get_supervisor_position(self):
        """通过 Supervisor API 获取真实位置"""
        if self.position_field:
            pos = self.position_field.getSFVec3f()
            return (pos[0], pos[1])
        return None
    
    def _get_supervisor_orientation(self):
        """通过 Supervisor API 获取真实朝向"""
        if self.rotation_field:
            rot = self.rotation_field.getSFRotation()
            # 绕 Z 轴旋转的角度
            if abs(rot[2]) > 0.99:  # 绕 Z 轴
                angle = rot[3] * (1 if rot[2] > 0 else -1)
                # 归一化到 [-pi, pi]
                while angle > math.pi:
                    angle -= 2 * math.pi
                while angle < -math.pi:
                    angle += 2 * math.pi
                return angle
            # 如果绕其他轴，用 atan2 计算
            return 0.0
        return None
    
    def move(self, vx, vy, omega):
        """
        麦克纳姆轮运动学控制（从 C 库 base.c 的 base_move 移植）
        vx: 前后速度 [m/s] (正=前进)
        vy: 左右速度 [m/s] (正=左移)
        omega: 旋转角速度 [rad/s] (正=左转)
        """
        # 麦克纳姆轮逆运动学（与 C 库完全一致）
        speeds = [
            1.0 / WHEEL_RADIUS * (vx + vy + (LX + LY) * omega),
            1.0 / WHEEL_RADIUS * (vx - vy - (LX + LY) * omega),
            1.0 / WHEEL_RADIUS * (vx - vy + (LX + LY) * omega),
            1.0 / WHEEL_RADIUS * (vx + vy - (LX + LY) * omega)
        ]
        
        # 限幅
        for i in range(4):
            speeds[i] = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, speeds[i]))
        
        # 设置速度
        for i, wheel in enumerate(self.wheels):
            if wheel:
                wheel.setVelocity(speeds[i])
        
        self.current_vx = vx
        self.current_vy = vy
        self.current_omega = omega
        
        # 更新里程计
        self._update_odometry(vx, vy, omega)
    
    def _update_odometry(self, vx, vy, omega):
        """更新里程计位置推算"""
        current_time = self.robot.getTime()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 0.5:  # 防止跳变（放宽限制）
            return
        
        # 在机器人坐标系下的速度
        # 转换到世界坐标系
        cos_a = math.cos(self.odom_angle)
        sin_a = math.sin(self.odom_angle)
        
        self.odom_x += (vx * cos_a - vy * sin_a) * dt
        self.odom_y += (vx * sin_a + vy * cos_a) * dt
        self.odom_angle += omega * dt
    
    def stop(self):
        """停止底盘"""
        self.move(0.0, 0.0, 0.0)
    
    def get_position(self):
        """获取当前位置 (x, y)"""
        # 优先使用 Supervisor API（最准确）
        supervisor_pos = self._get_supervisor_position()
        if supervisor_pos is not None:
            return supervisor_pos
        # 其次使用 GPS
        if self.gps:
            values = self.gps.getValues()
            return (values[0], values[1])
        # 最后使用里程计推算
        return (self.odom_x, self.odom_y)
    
    def get_orientation(self):
        """获取朝向角度 [rad]"""
        # 优先使用 Supervisor API
        supervisor_angle = self._get_supervisor_orientation()
        if supervisor_angle is not None:
            return supervisor_angle
        # 其次使用 Compass
        if self.compass:
            values = self.compass.getValues()
            angle = math.atan2(values[1], values[0])
            return angle
        # 最后使用里程计推算
        return self.odom_angle
    
    def reset_odometry(self, x=0.0, y=0.0, angle=0.0):
        """重置里程计"""
        self.odom_x = x
        self.odom_y = y
        self.odom_angle = angle
    
    def get_distance_to(self, target_x, target_y):
        """计算到目标点的距离"""
        pos = self.get_position()
        dx = target_x - pos[0]
        dy = target_y - pos[1]
        return math.sqrt(dx**2 + dy**2)
    
    def goto_target(self, target_x, target_y, target_angle=None):
        """
        导航到目标位置
        策略：先旋转对准目标方向，再直线前进
        """
        pos = self.get_position()
        current_angle = self.get_orientation()
        
        # 计算到目标的方向
        dx = target_x - pos[0]
        dy = target_y - pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        target_angle_to_target = math.atan2(dy, dx)
        
        # 角度差
        angle_diff = target_angle_to_target - current_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 控制参数
        max_speed = 0.5
        angle_tolerance = 0.3  # 角度容差 [rad]
        
        if abs(angle_diff) > angle_tolerance and distance > 0.3:
            # 1. 先旋转对准目标方向
            omega = 1.5 * angle_diff
            omega = max(-1.5, min(1.5, omega))
            self.move(0.0, 0.0, omega)  # 原地旋转
            return False
        elif distance > 0.3:
            # 2. 直线前进
            # 计算世界坐标系下的速度
            speed = min(max_speed, distance * 0.5 + 0.1)
            vx_world = speed * math.cos(target_angle_to_target)
            vy_world = speed * math.sin(target_angle_to_target)
            
            # 关键：将世界坐标系速度转换到机器人坐标系
            # move() 函数需要的是机器人坐标系下的 vx(前) 和 vy(左)
            cos_a = math.cos(current_angle)
            sin_a = math.sin(current_angle)
            vx_robot = vx_world * cos_a + vy_world * sin_a
            vy_robot = -vx_world * sin_a + vy_world * cos_a
            
            # 强角度修正
            omega = 1.0 * angle_diff
            omega = max(-0.8, min(0.8, omega))
            
            self.move(vx_robot, vy_robot, omega)
        else:
            # 3. 近距离：减速并精确调整
            speed = min(0.15, distance * 0.5)
            vx_world = speed * math.cos(target_angle_to_target)
            vy_world = speed * math.sin(target_angle_to_target)
            
            cos_a = math.cos(current_angle)
            sin_a = math.sin(current_angle)
            vx_robot = vx_world * cos_a + vy_world * sin_a
            vy_robot = -vx_world * sin_a + vy_world * cos_a
            
            if target_angle is not None:
                final_angle_diff = target_angle - current_angle
                while final_angle_diff > math.pi:
                    final_angle_diff -= 2 * math.pi
                while final_angle_diff < -math.pi:
                    final_angle_diff += 2 * math.pi
                omega = 1.0 * final_angle_diff
            else:
                omega = 1.0 * angle_diff
            
            omega = max(-0.5, min(0.5, omega))
            self.move(vx_robot, vy_robot, omega)
        
        # 检查是否到达
        pos_tol = NAVIGATION["position_tolerance"]
        reached = distance < pos_tol
        
        if target_angle is not None:
            final_angle_diff = target_angle - current_angle
            while final_angle_diff > math.pi:
                final_angle_diff -= 2 * math.pi
            while final_angle_diff < -math.pi:
                final_angle_diff += 2 * math.pi
            reached = reached and abs(final_angle_diff) < NAVIGATION["angle_tolerance"]
        
        return reached
    
    def start_navigation(self, target_x, target_y, target_angle=None):
        """开始导航到目标"""
        self.target_x = target_x
        self.target_y = target_y
        self.target_angle = target_angle
        self.navigation_active = True
        self.navigation_start_time = self.robot.getTime()
    
    def run_navigation(self):
        """执行导航（每步调用）"""
        if not self.navigation_active:
            return True
        
        if self.target_x is None:
            self.navigation_active = False
            return True
        
        reached = self.goto_target(self.target_x, self.target_y, self.target_angle)
        
        if reached:
            self.stop()
            self.navigation_active = False
            return True
        
        # 超时检查
        if self.navigation_start_time:
            elapsed = self.robot.getTime() - self.navigation_start_time
            if elapsed > NAVIGATION["max_navigation_time"]:
                print(f"  ⚠️ 导航超时 ({elapsed:.1f}s)")
                self.stop()
                self.navigation_active = False
                return True
        
        return False
    
    def is_navigation_active(self):
        """检查导航是否在进行中"""
        return self.navigation_active
