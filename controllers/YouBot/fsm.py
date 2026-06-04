"""
有限状态机（FSM）任务调度模块
管理 YouBot 从初始化到完成任务的完整状态流转

状态流：
INIT → WAIT_ORDER → NAVIGATE_TO_BLOCK → APPROACH_BLOCK → GRASP_BLOCK
→ LIFT → NAVIGATE_TO_TABLE → PLACE_ON_TABLE → BACK_OFF → (循环或 COMPLETED)

导航策略：使用 goto_target PID 控制 + Lidar 防碰撞
抓取策略：正面接近木块，用 grasp_block 正面抓取
"""

import math
from config import NAVIGATION, COLOR_NAMES, SIDE_GRASP


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
        self.state_timeout = 10.0

        # 导航子状态
        self.nav_phase = "far"  # far / near / precision

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

    def _get_distance_to(self, target_x, target_y):
        """计算到目标点的距离"""
        pos = self.drive.get_position()
        dx = target_x - pos[0]
        dy = target_y - pos[1]
        return math.sqrt(dx**2 + dy**2)

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
            self.nav_phase = "far"
            self._transition_to("NAVIGATE_TO_BLOCK")

    def _state_NAVIGATE_TO_BLOCK(self):
        """
        导航到木块位置
        
        三阶段策略：
        1. far（>1.0m）：用 goto_target 快速导航
        2. near（0.3~1.0m）：减速接近，启用 Lidar 防碰撞
        3. precision（<0.3m）：极慢速精确靠近，直到 Lidar 检测到木块在抓取距离
        """
        if self.current_color is None:
            self._transition_to("ERROR")
            return

        block_pos = self._get_block_position(self.current_color)
        dist = self._get_distance_to(block_pos[0], block_pos[1])

        # 获取 Lidar 前方距离
        lidar_front_dist = 5.0
        if self.perception.lidar and self.perception.lidar.enabled:
            lidar_front_dist = self.perception.lidar.get_min_distance_in_angle_range(angle_range_deg=20)

        # 判断阶段
        if dist > 1.0:
            self.nav_phase = "far"
        elif dist > 0.30:
            self.nav_phase = "near"
        else:
            self.nav_phase = "precision"

        # 执行导航
        if self.nav_phase == "far":
            # 快速导航
            reached = self.drive.goto_target(block_pos[0], block_pos[1])
            if reached:
                print(f"  ✅ 远距离导航到达 (距离={dist:.2f}m)")

        elif self.nav_phase == "near":
            # 减速接近 + Lidar 防碰撞
            robot_pos = self.drive.get_position()
            robot_angle = self.drive.get_orientation()

            dx = block_pos[0] - robot_pos[0]
            dy = block_pos[1] - robot_pos[1]
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - robot_angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # 如果 Lidar 检测到前方有障碍物且很近，减速
            speed_scale = 1.0
            if lidar_front_dist < 0.35:
                speed_scale = 0.3
                print(f"  ⚠️ Lidar 检测到障碍物 ({lidar_front_dist:.2f}m)，减速")

            if abs(angle_diff) > 0.3:
                # 先对准
                omega = 1.2 * angle_diff
                omega = max(-1.2, min(1.2, omega))
                self.drive.move(0.0, 0.0, omega)
            else:
                speed = min(0.3, dist * 0.3 + 0.05) * speed_scale
                vx_world = speed * math.cos(target_angle)
                vy_world = speed * math.sin(target_angle)
                cos_a = math.cos(robot_angle)
                sin_a = math.sin(robot_angle)
                vx_robot = vx_world * cos_a + vy_world * sin_a
                vy_robot = -vx_world * sin_a + vy_world * cos_a
                omega = 0.5 * angle_diff
                self.drive.move(vx_robot, vy_robot, omega)

        elif self.nav_phase == "precision":
            # 极慢速精确靠近
            robot_pos = self.drive.get_position()
            robot_angle = self.drive.get_orientation()

            dx = block_pos[0] - robot_pos[0]
            dy = block_pos[1] - robot_pos[1]
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - robot_angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # 使用配置参数中的 Lidar 停车距离
            lidar_stop_dist = NAVIGATION["lidar_precision_stop"]
            
            # 如果 Lidar 检测到木块很近，停止并进入抓取
            # 注意：Lidar 装在机器人前方 0.25m 处
            # 当 Lidar 检测到 0.12m 时，车头离木块还有约 0.25-0.12=0.13m
            # 这样车头不会碰撞木块
            if lidar_front_dist < lidar_stop_dist or dist < 0.10:
                print(f"  ✅ 已到达木块前方 (距离={dist:.2f}m, Lidar={lidar_front_dist:.2f}m)")
                self.drive.stop()
                self._transition_to("APPROACH_BLOCK")
                return

            # 极慢速前进
            if abs(angle_diff) > 0.2:
                omega = 0.8 * angle_diff
                omega = max(-0.8, min(0.8, omega))
                self.drive.move(0.0, 0.0, omega)
            else:
                speed = min(0.08, dist * 0.2)
                vx_world = speed * math.cos(target_angle)
                vy_world = speed * math.sin(target_angle)
                cos_a = math.cos(robot_angle)
                sin_a = math.sin(robot_angle)
                vx_robot = vx_world * cos_a + vy_world * sin_a
                vy_robot = -vx_world * sin_a + vy_world * cos_a
                omega = 0.3 * angle_diff
                self.drive.move(vx_robot, vy_robot, omega)


        # 超时检查
        if self._is_timeout(60.0):
            print(f"  ⚠️ 导航到木块超时")
            self._transition_to("ERROR")

    def _state_APPROACH_BLOCK(self):
        """
        最后一步靠近木块 + 抓取（增强版）
        
        改进：
        1. 停车后先用 Lidar 精确扫描木块位置
        2. 如果木块偏移，用底盘微调
        3. 用 Camera + Lidar 融合定位确认
        4. 用精确坐标调用机械臂抓取
        """
        if self.current_color is None:
            self._transition_to("ERROR")
            return

        print(f"\n  🎯 ===== 开始抓取 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块 =====")

        # ===== 步骤1: 用 Lidar 精确扫描木块位置 =====
        lidar_pos = None
        if self.perception.lidar and self.perception.lidar.enabled:
            print(f"  📡 用 Lidar 扫描木块精确位置...")
            lidar_pos = self.perception.lidar.locate_block_exact()
            if lidar_pos:
                forward, left = lidar_pos
                print(f"  📡 Lidar 检测到木块: 前={forward:.3f}m, 左={left:.3f}m")
            else:
                print(f"  ⚠️ Lidar 未检测到木块，使用 Supervisor 坐标")

        # ===== 步骤2: 用 Camera 识别确认 =====
        camera_pos = None
        camera_objects = self.perception.detect_blocks_by_color()
        for color, x, y, z in camera_objects:
            if color == self.current_color:
                rel = self.perception.get_block_relative_position(x, y)
                camera_pos = (rel[0], rel[1])
                print(f"  📷 Camera 检测到 {COLOR_NAMES.get(color, color)}: 前={rel[0]:.3f}m, 左={rel[1]:.3f}m")
                break

        # ===== 步骤3: 融合定位，确定最终抓取坐标 =====
        grasp_forward = None
        grasp_left = None
        use_lidar_for_adjustment = False

        if lidar_pos and camera_pos:
            # 两者都有，验证一致性
            l_dist = math.sqrt(lidar_pos[0]**2 + lidar_pos[1]**2)
            c_dist = math.sqrt(camera_pos[0]**2 + camera_pos[1]**2)
            if abs(l_dist - c_dist) < 0.15:
                # 一致，用 Lidar 精确位置
                grasp_forward, grasp_left = lidar_pos
                print(f"  ✅ 融合定位一致，使用 Lidar 精确位置")
            else:
                # 不一致，优先用 Lidar
                grasp_forward, grasp_left = lidar_pos
                print(f"  ⚠️ 融合定位不一致 (Lidar={l_dist:.3f}m, Camera={c_dist:.3f}m)，优先用 Lidar")
        elif lidar_pos:
            grasp_forward, grasp_left = lidar_pos
            print(f"  📡 仅 Lidar 检测到，使用 Lidar 位置")
        elif camera_pos:
            grasp_forward, grasp_left = camera_pos
            print(f"  📷 仅 Camera 检测到，使用 Camera 位置")
        else:
            # 都没检测到，用 Supervisor 给的坐标
            block_pos = self._get_block_position(self.current_color)
            robot_pos = self.drive.get_position()
            robot_angle = self.drive.get_orientation()
            dx = block_pos[0] - robot_pos[0]
            dy = block_pos[1] - robot_pos[1]
            cos_a = math.cos(robot_angle)
            sin_a = math.sin(robot_angle)
            grasp_forward = dx * cos_a + dy * sin_a
            grasp_left = -dx * sin_a + dy * cos_a
            print(f"  ⚠️ 传感器未检测到，使用 Supervisor 坐标: 前={grasp_forward:.3f}m, 左={grasp_left:.3f}m")

        # ===== 步骤4: 如果木块左右偏移超过 3cm，用底盘微调 =====
        if abs(grasp_left) > 0.03:
            print(f"  🔄 木块左右偏移 {grasp_left:.3f}m，底盘微调...")
            # 横向移动（麦克纳姆轮优势）
            move_direction = -grasp_left * 0.8  # 往木块方向移动
            move_direction = max(-0.05, min(0.05, move_direction))  # 限幅
            self.drive.move(0.0, move_direction, 0.0)
            self.robot.step(32 * 15)
            self.drive.stop()
            print(f"  ✅ 底盘微调完成")

            # 微调后重新用 Lidar 扫描确认
            if self.perception.lidar and self.perception.lidar.enabled:
                new_lidar_pos = self.perception.lidar.locate_block_exact()
                if new_lidar_pos:
                    grasp_forward, grasp_left = new_lidar_pos
                    print(f"  📡 微调后 Lidar 重新定位: 前={grasp_forward:.3f}m, 左={grasp_left:.3f}m")

        # ===== 步骤5: 如果木块前后距离太远，再前进一点 =====
        # 机械臂基座在机器人前方 0.156m，臂展 0.461m
        # 夹爪能到达的最远水平距离 ≈ 0.156 + 0.461 = 0.617m
        # 但地面抓取时，机械臂需要垂直下探，水平伸展受限
        # 最佳抓取距离：木块在机器人前方 0.10~0.18m
        if grasp_forward > 0.18:
            print(f"  🔄 木块距离 {grasp_forward:.3f}m 稍远，再前进一点...")
            move_dist = grasp_forward - 0.12  # 前进到距离 0.12m
            move_dist = max(0.02, min(0.08, move_dist))  # 限幅 2~8cm
            self.drive.move(move_dist, 0.0, 0.0)
            self.robot.step(32 * 15)
            self.drive.stop()
            print(f"  ✅ 前进 {move_dist:.3f}m 完成")

            # 重新扫描
            if self.perception.lidar and self.perception.lidar.enabled:
                new_lidar_pos = self.perception.lidar.locate_block_exact()
                if new_lidar_pos:
                    grasp_forward, grasp_left = new_lidar_pos
                    print(f"  📡 前进后 Lidar 重新定位: 前={grasp_forward:.3f}m, 左={grasp_left:.3f}m")

        # ===== 步骤6: 执行抓取 =====
        print(f"  🦾 最终抓取坐标: 前={grasp_forward:.3f}m, 左={grasp_left:.3f}m")
        self.arm.grasp_block(grasp_forward, grasp_left, block_z=0.05, gripper=self.gripper)

        # ===== 步骤7: 检查抓取结果 =====
        if self.gripper.grasp_detected:
            print(f"  ✊ 成功抓取 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块!")
            self._transition_to("LIFT")
        else:
            # 尝试等待一下
            grasp_success = self.gripper.wait_for_grasp(timeout_steps=80)
            if grasp_success:
                print(f"  ✊ 抓取成功!")
                self._transition_to("LIFT")
            else:
                print(f"  ⚠️ 第一次抓取失败，尝试重试...")
                
                # 重试策略：再前进 2cm
                self.drive.move(0.02, 0.0, 0.0)
                self.robot.step(32 * 15)
                self.drive.stop()
                
                # 重新用 Lidar 扫描
                if self.perception.lidar and self.perception.lidar.enabled:
                    retry_pos = self.perception.lidar.locate_block_exact()
                    if retry_pos:
                        grasp_forward, grasp_left = retry_pos
                        print(f"  📡 重试 Lidar 定位: 前={grasp_forward:.3f}m, 左={grasp_left:.3f}m")
                
                self.arm.grasp_block(grasp_forward, grasp_left, block_z=0.05, gripper=self.gripper)
                
                if self.gripper.grasp_detected:
                    print(f"  ✊ 重试成功!")
                    self._transition_to("LIFT")
                else:
                    grasp_success = self.gripper.wait_for_grasp(timeout_steps=80)
                    if grasp_success:
                        print(f"  ✊ 重试成功!")
                        self._transition_to("LIFT")
                    else:
                        print(f"  ❌ 抓取失败，跳过此木块")
                        self._transition_to("ERROR")


    def _state_LIFT(self):
        """抬起木块"""
        self.arm.lift_block()
        self.robot.step(32 * 30)
        self._transition_to("NAVIGATE_TO_TABLE")

    def _state_NAVIGATE_TO_TABLE(self):
        """
        导航到桌子正前方（北侧 0.6m 处），面向桌子中心
        
        固定目标点：(0, 0.6) 朝向 -π/2（朝南）
        这样 place_on_table 的机械臂姿态才能正确工作
        """
        table_pos = self.table_position
        approach_dist = NAVIGATION["table_approach_distance"]
        dist = self._get_distance_to(table_pos[0], table_pos[1])

        if dist < approach_dist:
            print(f"  ✅ 已到达桌子附近 (距离={dist:.2f}m)")
            self.drive.stop()
            self._transition_to("PLACE_ON_TABLE")
            return

        # 固定目标点：桌子正前方（北侧 0.6m），面向南
        target_x, target_y = 0.0, 0.6
        target_angle = -math.pi / 2  # 朝南（面向桌子中心）

        if not self.drive.is_navigation_active():
            print(f"  🚗 导航到桌子: 目标 ({target_x:.2f}, {target_y:.2f}), 朝向 {target_angle:.2f}rad")
            self.drive.start_navigation(target_x, target_y, target_angle)

        reached = self.drive.run_navigation()

        if reached:
            self.drive.stop()
            self._transition_to("PLACE_ON_TABLE")

        if self._is_timeout(60.0):
            print(f"  ⚠️ 导航到桌子超时")
            self._transition_to("ERROR")

    def _state_PLACE_ON_TABLE(self):
        """平稳放置木块到桌面"""
        self.arm.place_on_table(gripper=self.gripper)
        print(f"  ✅ 已平稳放置 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块到桌面")
        self._transition_to("BACK_OFF")

    def _state_BACK_OFF(self):
        """后退离开桌子"""
        self.arm.reset()
        self.robot.step(32 * 30)

        self.drive.move(-0.1, 0.0, 0.0)
        self.robot.step(32 * 20)
        self.drive.stop()

        self.current_target_index += 1
        self.current_color = None

        if self.current_target_index >= len(self.grasp_order):
            self._transition_to("COMPLETED")
        else:
            self.current_color = self.grasp_order[self.current_target_index]
            self.nav_phase = "far"
            self._transition_to("NAVIGATE_TO_BLOCK")

    def _state_COMPLETED(self):
        """任务完成"""
        pass

    def _state_ERROR(self):
        """错误状态"""
        print(f"  ❌ 状态机进入错误状态")
        self.drive.stop()
