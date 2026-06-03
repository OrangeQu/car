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

        # 计算木块在机器人坐标系下的位置
        cos_a = math.cos(robot_angle)
        sin_a = math.sin(robot_angle)
        rel_x = dx * cos_a + dy * sin_a      # 前方为正
        rel_y = -dx * sin_a + dy * cos_a     # 左侧为正

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
                    camera_z = obj_pos[2]
                    if 0.0 < camera_z < 1.0:
                        block_z = camera_z
                    camera_x = obj_pos[0]
                    camera_y = obj_pos[1]
                    if abs(camera_y) > 0.01:
                        offset_y = camera_y
                    break

        # 阶段4: 先后退一点，避免底盘碰到木块
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

        # 使用预设姿态抓取（带逐步下探 + 偏移补偿）
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
                
                # 张开夹爪，再往前靠近一点重试
                self.gripper.open()
                self.robot.step(32 * 30)
                self.drive.move(0.03, 0.0, 0.0)
                self.robot.step(32 * 30)
                self.drive.stop()
                
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
        """
        导航到桌子（使用固定安全目标点，避开桌腿）
        
        桌子在 (0,0)，桌面尺寸 1.2m x 0.8m
        桌腿位置：4条圆柱腿在 (±0.55, ±0.35)，半径 0.04m
        
        策略：不使用动态目标点（避免目标点落在桌腿附近），
        而是根据机器人当前位置，选择桌子四个方向上的固定安全点之一。
        """
        table_pos = self.table_position
        approach_dist = NAVIGATION["table_approach_distance"]
        robot_pos = self.drive.get_position()
        
        # 计算到桌子的距离
        dx = table_pos[0] - robot_pos[0]
        dy = table_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        # 检查是否已经到达
        if dist < approach_dist:
            print(f"  ✅ 已到达桌子附近 (距离={dist:.2f}m)")
            self.drive.stop()
            self._transition_to("PLACE_ON_TABLE")
            return
        
        # 定义4个固定安全目标点（桌子四条边的外侧，避开桌腿区域）
        # 桌子尺寸 1.2m x 0.8m，桌腿在 (±0.55, ±0.35)
        # 机械臂竖直向上时，arm4 向后弯曲（负方向），夹爪朝南（y负方向）伸约 0.17m
        # 小车在 (0, 0.6) 面向南（朝桌子中心），arm4 负方向弯曲使夹爪朝南伸向桌子中心
        # 安全点选在桌子边缘外侧 0.2m 处（北/南：y=0.6，东/西：x=0.8）
        # 这样 arm4=-1.670 时夹爪在机器人前方 0.17m（朝南），正好在桌面范围内
        # 北/南方向：桌子 y 范围 [-0.4, 0.4]，安全点 y=0.6（距边缘 0.2m 外侧）
        # 东/西方向：桌子 x 范围 [-0.6, 0.6]，安全点 x=0.8（距边缘 0.2m 外侧）
        # 优先选择前方（北），因为机械臂向前伸展
        safe_points = [
            (0.0, 0.6),    # 北（y正方向）- 桌子前方（优先）
            (0.0, -0.6),   # 南（y负方向）- 桌子后方
            (-0.8, 0.0),   # 西（x负方向）- 桌子左方
            (0.8, 0.0),    # 东（x正方向）- 桌子右方
        ]
        
        # 优先选择前方（北），如果前方太远则选择最近的安全点
        front_point = safe_points[0]  # (0.0, 0.6)
        front_dist = math.sqrt((front_point[0] - robot_pos[0])**2 + (front_point[1] - robot_pos[1])**2)
        
        # 如果前方距离 < 8m（场地半对角线），优先选择前方
        # 否则选择最近的安全点
        if front_dist < 8.0:
            best_point = front_point
        else:
            best_point = None
            best_dist = float('inf')
            for px, py in safe_points:
                d = math.sqrt((px - robot_pos[0])**2 + (py - robot_pos[1])**2)
                if d < best_dist:
                    best_dist = d
                    best_point = (px, py)
        
        target_x, target_y = best_point
        
        # 目标朝向：面向桌子中心
        target_angle = math.atan2(table_pos[1] - target_y, table_pos[0] - target_x)
        
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
        """平稳放置木块到桌面（包含释放）"""
        # 使用新的多步骤平稳放置方法
        # place_on_table 内部已经包含了：接近 → 缓慢下降 → 释放 → 抬升
        self.arm.place_on_table(gripper=self.gripper)

        print(f"  ✅ 已平稳放置 {COLOR_NAMES.get(self.current_color, self.current_color)} 木块到桌面")
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
