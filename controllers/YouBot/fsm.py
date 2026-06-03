"""
有限状态机（FSM）任务调度模块
管理 YouBot 从初始化到完成任务的完整状态流转
集成 LLM 决策（DeepSeek API）
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

        # LLM 决策缓存
        self.llm_decision_cache = None
        self.llm_decision_time = 0.0
        self.llm_decision_interval = 2.0  # 每2秒重新决策一次
        self.llm_action_index = 0  # 当前执行到动作序列的第几个
        self.llm_action_start_time = 0.0  # 当前动作开始时间
        self.llm_action_duration = 0.5  # 每个动作持续秒数

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
        """
        计算目标相对于机器人的位置（在机器人坐标系下）
        机器人坐标系：X向前，Y向左
        """
        robot_pos = self.drive.get_position()
        robot_angle = self.drive.get_orientation()

        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]

        # 转换到机器人坐标系（旋转 -robot_angle）
        cos_a = math.cos(robot_angle)
        sin_a = math.sin(robot_angle)
        rel_x = dx * cos_a + dy * sin_a
        rel_y = -dx * sin_a + dy * cos_a

        return rel_x, rel_y

    def _call_llm_for_navigation(self, target_color, target_x, target_y):
        """
        调用 LLM 获取导航决策
        
        返回: {"thought": "...", "actions": [...]}
        """
        robot_pos = self.drive.get_position()
        robot_angle = self.drive.get_orientation()

        # 获取视觉信息
        visual_objects = self.perception.get_visual_objects()

        prompt = f"""
你是一个 YouBot 机器人的大脑。根据摄像头看到的物体和任务目标，决定下一步动作。

当前任务: 找到并接近 {COLOR_NAMES.get(target_color, target_color)} 木块

机器人当前位置: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})
机器人朝向角度: {robot_angle:.2f} rad
目标木块位置: ({target_x:.2f}, {target_y:.2f})

摄像头识别到的物体:
{json.dumps(visual_objects, indent=2, ensure_ascii=False) if visual_objects else "无"}

可用动作:
["Forwards", "Backwards", "TurnLeft", "TurnRight", "StrafeLeft", "StrafeRight", "stop"]

返回 JSON 格式:
{{
    "thought": "你的思考过程",
    "actions": ["动作1", "动作2", ...]
}}

决策规则:
1. 如果目标在正前方且距离远 → Forwards
2. 如果目标在左侧 → TurnLeft
3. 如果目标在右侧 → TurnRight
4. 如果目标很近（<0.3m）→ ["stop"]
5. 如果未看到目标 → 旋转搜索
"""

        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {DEEPSEEK_API_KEY}"
        }

        data = {
            "model": "deepseek-chat",
            "messages": [
                {"role": "system", "content": "你是一个智能机器人控制器，擅长分析视觉信息并做出决策。"},
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.3,
            "max_tokens": 300
        }

        try:
            response = requests.post(DEEPSEEK_API_URL, headers=headers, json=data, timeout=15)
            response.raise_for_status()
            result = response.json()

            if "choices" in result and len(result["choices"]) > 0:
                content = result["choices"][0]["message"]["content"]
                print(f"  🧠 LLM 决策: {content}")

                # 解析 JSON
                if "```json" in content:
                    content = content.split("```json")[1].split("```")[0].strip()
                elif "```" in content:
                    content = content.split("```")[1].split("```")[0].strip()

                return json.loads(content)
        except Exception as e:
            print(f"  ⚠️ LLM 调用失败: {e}")

        return None

    def _execute_llm_action(self, action):
        """执行 LLM 返回的单个动作"""
        if action == "Forwards":
            self.drive.move(0.3, 0.0, 0.0)
        elif action == "Backwards":
            self.drive.move(-0.3, 0.0, 0.0)
        elif action == "TurnLeft":
            self.drive.move(0.0, 0.0, 0.8)
        elif action == "TurnRight":
            self.drive.move(0.0, 0.0, -0.8)
        elif action == "StrafeLeft":
            self.drive.move(0.0, 0.3, 0.0)
        elif action == "StrafeRight":
            self.drive.move(0.0, -0.3, 0.0)
        elif action == "stop":
            self.drive.stop()
        else:
            print(f"  ⚠️ 未知动作: {action}")

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
        """导航到木块位置（LLM决策 + 动作序列完整执行 + 回退导航）"""
        if self.current_color is None:
            self._transition_to("ERROR")
            return

        block_pos = self._get_block_position(self.current_color)
        robot_pos = self.drive.get_position()
        dx = block_pos[0] - robot_pos[0]
        dy = block_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        # 调试输出（每0.5秒一次）
        if int(self.robot.getTime() * 2) % 10 == 0:
            print(f"  📍 位置: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}), 目标: {block_pos}, 距离: {dist:.2f}m")

        # 如果足够接近，进入接近阶段
        approach_dist = NAVIGATION["approach_distance"]
        if dist < approach_dist:
            print(f"  ✅ 已到达木块附近 (距离={dist:.2f}m)")
            self.drive.stop()
            self.llm_decision_cache = None
            self._transition_to("APPROACH_BLOCK")
            return

        current_time = self.robot.getTime()

        # 检查当前 LLM 动作是否已完成（持续足够时间）
        if self.llm_decision_cache is not None:
            action_duration = current_time - self.llm_action_start_time
            if action_duration >= self.llm_action_duration:
                # 当前动作完成，移动到下一个
                self.llm_action_index += 1
                self.llm_action_start_time = current_time
                # 如果所有动作都执行完了，清空缓存
                if self.llm_action_index >= len(self.llm_decision_cache):
                    self.llm_decision_cache = None
                    self.llm_action_index = 0

        # 如果没有缓存的决策，使用回退导航 + 尝试 LLM
        if self.llm_decision_cache is None:
            # 回退到传统导航（更可靠）
            self._fallback_navigation(block_pos[0], block_pos[1])

            # 每2秒尝试一次 LLM 决策
            if current_time - self.llm_decision_time > self.llm_decision_interval:
                decision = self._call_llm_for_navigation(self.current_color, block_pos[0], block_pos[1])
                if decision and "actions" in decision:
                    self.llm_decision_cache = decision["actions"]
                    self.llm_decision_time = current_time
                    self.llm_action_index = 0
                    self.llm_action_start_time = current_time
        else:
            # 执行当前动作
            action = self.llm_decision_cache[self.llm_action_index]
            self._execute_llm_action(action)

            # 如果动作是 stop，清空缓存
            if action == "stop":
                self.llm_decision_cache = None
                self.llm_action_index = 0
                if dist < approach_dist:
                    self._transition_to("APPROACH_BLOCK")

        # 超时处理
        if self._is_timeout(60.0):
            print(f"  ⚠️ 导航到木块超时")
            self._transition_to("ERROR")

    def _fallback_navigation(self, target_x, target_y):
        """回退导航（当 LLM 不可用时）"""
        robot_pos = self.drive.get_position()
        robot_angle = self.drive.get_orientation()

        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.01:
            self.drive.stop()
            return

        # 计算目标方向
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - robot_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # 先旋转对准
        if abs(angle_diff) > 0.3:
            omega = 1.5 * angle_diff
            omega = max(-1.5, min(1.5, omega))
            self.drive.move(0.0, 0.0, omega)
            return

        # 直线前进
        speed = min(0.5, dist * 0.3 + 0.1)
        vx_world = speed * math.cos(target_angle)
        vy_world = speed * math.sin(target_angle)

        cos_a = math.cos(robot_angle)
        sin_a = math.sin(robot_angle)
        vx_robot = vx_world * cos_a + vy_world * sin_a
        vy_robot = -vx_world * sin_a + vy_world * cos_a

        omega = 0.5 * angle_diff
        self.drive.move(vx_robot, vy_robot, omega)

    def _state_APPROACH_BLOCK(self):
        """接近木块到抓取距离"""
        if self.current_color is None:
            self._transition_to("ERROR")
            return

        block_pos = self._get_block_position(self.current_color)
        robot_pos = self.drive.get_position()
        robot_angle = self.drive.get_orientation()

        dx = block_pos[0] - robot_pos[0]
        dy = block_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        if int(self.robot.getTime() * 2) % 10 == 0:
            print(f"  🎯 接近中: 距离木块 {dist:.3f}m")

        grasp_dist = NAVIGATION["grasp_distance"]

        if dist <= grasp_dist:
            self.drive.stop()
            print(f"  ✅ 到达抓取距离 ({dist:.3f}m)")
            self._transition_to("GRASP")
            return

        # 计算目标方向
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - robot_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # 先旋转对准
        if abs(angle_diff) > 0.2:
            omega = 1.5 * angle_diff
            omega = max(-1.5, min(1.5, omega))
            self.drive.move(0.0, 0.0, omega)
            return

        # 直线前进
        speed = min(0.15, (dist - grasp_dist) * 0.8 + 0.03)
        vx_world = speed * math.cos(target_angle)
        vy_world = speed * math.sin(target_angle)

        cos_a = math.cos(robot_angle)
        sin_a = math.sin(robot_angle)
        vx_robot = vx_world * cos_a + vy_world * sin_a
        vy_robot = -vx_world * sin_a + vy_world * cos_a

        omega = 0.5 * angle_diff
        omega = max(-0.3, min(0.3, omega))

        self.drive.move(vx_robot, vy_robot, omega)

        if self._is_timeout(15.0):
            print(f"  ⚠️ 接近木块超时")
            self._transition_to("ERROR")

    def _state_GRASP(self):
        """抓取木块"""
        block_pos = self._get_block_position(self.current_color)
        robot_pos = self.drive.get_position()
        robot_angle = self.drive.get_orientation()

        dx = block_pos[0] - robot_pos[0]
        dy = block_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        # 计算木块相对于机器人前方的角度
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - robot_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # 阶段1: 先旋转使机械臂正对木块（更精确）
        grasp_dist = NAVIGATION["grasp_distance"]
        if abs(angle_diff) > 0.05:
            omega = 0.8 * angle_diff
            omega = max(-0.3, min(0.3, omega))
            self.drive.move(0.0, 0.0, omega)
            return

        # 阶段2: 如果距离太远，缓慢靠近到抓取距离
        # 机械臂基座在机器人前方 0.156m，地面抓取时水平伸展仅约 0.02m
        # 所以底盘需要靠近到木块前方 0.05m 以内
        if dist > grasp_dist:
            speed = min(0.03, max(0.005, (dist - grasp_dist * 0.8) * 0.2))
            vx_world = speed * math.cos(target_angle)
            vy_world = speed * math.sin(target_angle)
            cos_a = math.cos(robot_angle)
            sin_a = math.sin(robot_angle)
            vx_robot = vx_world * cos_a + vy_world * sin_a
            vy_robot = -vx_world * sin_a + vy_world * cos_a
            omega = 0.2 * angle_diff
            self.drive.move(vx_robot, vy_robot, omega)
            return

        # 阶段3: 距离足够近，停止底盘
        self.drive.stop()

        # ===== 调试输出 =====
        print(f"\n  🔍 抓取调试信息:")
        print(f"     机器人位置: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f})")
        print(f"     机器人朝向: {robot_angle:.3f} rad")
        print(f"     木块位置: {block_pos}")
        print(f"     世界坐标差: dx={dx:.3f}, dy={dy:.3f}, 距离={dist:.3f}")

        # 计算木块在机器人坐标系下的位置
        cos_a = math.cos(robot_angle)
        sin_a = math.sin(robot_angle)
        rel_x = dx * cos_a + dy * sin_a      # 前方为正
        rel_y = -dx * sin_a + dy * cos_a     # 左侧为正
        print(f"     相对位置: rel_x={rel_x:.3f}, rel_y={rel_y:.3f}")

        # 使用摄像头获取木块精确高度和偏移
        block_z = 0.05  # 默认地面高度
        offset_y = 0.0  # 木块左右偏移（正=左偏）
        
        visual_objects = self.perception.get_visual_objects()
        for obj in visual_objects:
            obj_color = obj.get('colors', [])
            # 检查颜色是否匹配当前目标
            if obj_color:
                obj_pos = obj.get('position', [0, 0, 0])
                if len(obj_pos) >= 3:
                    # 摄像头返回的 position[2] 是木块在基座坐标系下的 z 坐标
                    camera_z = obj_pos[2]
                    print(f"     摄像头检测到木块高度: z={camera_z:.3f}m")
                    # 如果摄像头检测到的高度合理（0~0.2m 地面，或 0.6~0.9m 桌面）
                    if 0.0 < camera_z < 1.0:
                        block_z = camera_z
                    
                    # 摄像头返回的 position[0] 是木块在基座坐标系下的 x 坐标（前后）
                    # position[1] 是木块在基座坐标系下的 y 坐标（左右）
                    camera_x = obj_pos[0]
                    camera_y = obj_pos[1]
                    print(f"     摄像头检测到木块位置: x={camera_x:.3f}m, y={camera_y:.3f}m")
                    
                    # 如果摄像头检测到木块在侧面（y != 0），说明木块被碰偏了
                    if abs(camera_y) > 0.01:
                        offset_y = camera_y
                        print(f"     检测到木块偏移: offset_y={offset_y:.3f}m")
                    break

        print(f"     使用木块高度: z={block_z:.3f}m")

        # 阶段4: 先后退一点，避免底盘碰到木块
        print(f"  🦾 后退一点避免碰撞木块...")
        self.drive.move(-0.03, 0.0, 0.0)  # 后退 3cm
        self.robot.step(32 * 20)
        self.drive.stop()
        
        # 重新计算木块在机器人坐标系下的位置（后退后位置变了）
        robot_pos = self.drive.get_position()
        dx = block_pos[0] - robot_pos[0]
        dy = block_pos[1] - robot_pos[1]
        cos_a = math.cos(robot_angle)
        sin_a = math.sin(robot_angle)
        rel_x = dx * cos_a + dy * sin_a
        rel_y = -dx * sin_a + dy * cos_a
        print(f"     后退后相对位置: rel_x={rel_x:.3f}m, rel_y={rel_y:.3f}m")

        # 使用预设姿态抓取（带逐步下探 + 偏移补偿）
        print(f"  🦾 木块在机器人前方 {rel_x:.2f}m 处，使用预设姿态抓取（带逐步下探）")
        self.arm.grasp_block(rel_x, rel_y, block_z, gripper=self.gripper, offset_y=offset_y)

        # grasp_block 内部已经完成了夹爪闭合和检测
        # 检查夹爪是否夹到了物体
        if self.gripper.grasp_detected:
            print(f"  ✊ 成功抓取 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块!")
            self._transition_to("LIFT")
        else:
            # 尝试最后一次检测
            grasp_success = self.gripper.wait_for_grasp(timeout_steps=50)
            if grasp_success:
                print(f"  ✊ 成功抓取 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块!")
                self._transition_to("LIFT")
            else:
                print(f"  ⚠️ 抓取可能失败，尝试重试...")
                
                # 张开夹爪
                self.gripper.open()
                self.robot.step(32 * 30)
                
                # 再往前靠近一点
                self.drive.move(0.03, 0.0, 0.0)
                self.robot.step(32 * 30)
                self.drive.stop()
                
                # 重新尝试抓取
                print(f"  🦾 重试: 使用低姿态抓取")
                self.arm.set_pose("grasp_low")
                self.robot.step(32 * 60)
                self.gripper.close()
                grasp_success = self.gripper.wait_for_grasp(timeout_steps=150)
                
                if grasp_success:
                    print(f"  ✊ 重试成功! 已抓取 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块")
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
        """导航到桌子"""
        table_pos = self.table_position

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
        self.arm.place_on_table()
        self.robot.step(32 * 30)

        self._transition_to("RELEASE")

    def _state_RELEASE(self):
        """释放木块"""
        self.gripper.open()
        self.robot.step(32 * 20)

        print(f"  🖐 已放置 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块到桌面")
        self._transition_to("BACK_OFF")

    def _state_BACK_OFF(self):
        """后退离开桌子"""
        self.arm.reset()
        self.robot.step(32 * 30)

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
