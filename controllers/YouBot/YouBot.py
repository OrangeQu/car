"""
YouBot 主控制器 - 基于YouBot的多目标物自主搜集与定点放置

功能：
1. 从 Supervisor 接收任务数据（木块位置、抓取顺序）
2. 使用有限状态机调度任务流程
3. 麦克纳姆轮底盘导航
4. 机械臂逆运动学抓取（C库 IK 算法）
5. 多传感器融合感知（CameraRecognition）
6. LLM 决策（DeepSeek API）
7. 键盘控制（调试用）
"""

from controller import Supervisor, Receiver, Keyboard
import json
import sys
import os

# 添加当前目录到路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config import TIME_STEP, COLOR_NAMES
from mecanum_drive import MecanumDrive
from arm_controller import ArmController
from gripper_controller import GripperController
from perception import Perception
from fsm import FSM


class YouBotController:
    """YouBot 主控制器"""

    def __init__(self):
        # 使用 Supervisor 类
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        print("=" * 60)
        print("🤖 YouBot 多目标物自主搜集与定点放置")
        print("=" * 60)

        # 初始化键盘
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        # 初始化各模块
        print("\n--- 初始化硬件模块 ---")
        self.drive = MecanumDrive(self.robot, self.timestep)
        self.arm = ArmController(self.robot, self.timestep)
        self.gripper = GripperController(self.robot, self.timestep)

        # 设置 Supervisor API 获取真实位置
        self._setup_supervisor_api()

        print("\n--- 初始化感知模块 ---")
        self.perception = Perception(self.robot, self.timestep)

        # 设置位姿回调
        self.perception.set_robot_pose_callback(
            self.drive.get_position,
            self.drive.get_orientation
        )

        print("\n--- 初始化状态机 ---")
        self.fsm = FSM(self.robot, self.drive, self.arm, self.gripper, self.perception)

        # 初始化通信
        self.receiver = None
        self._init_communication()

        # 任务状态
        self.mission_data_received = False
        self.task_completed = False
        self.task_error = False

        # 自动模式标志
        self.auto_mode = True

        print("\n" + "=" * 60)
        print("✅ YouBot 控制器初始化完成")
        print("=" * 60)
        print("键盘控制:")
        print("  V - 打印视觉识别信息")
        print("  P - 打印物体识别信息")
        print("  W/S - 前进/后退")
        print("  A/D - 左转/右转")
        print("  Q/E - 左移/右移")
        print("  G - 夹爪闭合")
        print("  Y - 夹爪张开")
        print("  空格 - 停止")
        print("  M - 切换自动/手动模式")
        print("=" * 60)

    def _setup_supervisor_api(self):
        """设置 Supervisor API 获取真实位置"""
        try:
            self_node = self.robot.getSelf()
            if self_node:
                self.drive.position_field = self_node.getField("translation")
                self.drive.rotation_field = self_node.getField("rotation")
                print("  ✓ 使用 Supervisor API 获取真实位置")
            else:
                print("  ⚠️ 无法获取自身节点")
        except Exception as e:
            print(f"  ⚠️ Supervisor API 初始化失败: {e}")

    def _init_communication(self):
        """初始化通信设备"""
        self.receiver = self.robot.getDevice("receiver")
        if self.receiver:
            self.receiver.enable(self.timestep)
            print("  ✓ Receiver 初始化成功")
        else:
            print("  ⚠️ 未找到 Receiver 设备，将使用默认任务数据")

    def receive_mission_data(self):
        """从 Supervisor 接收任务数据"""
        if self.receiver and self.receiver.getQueueLength() > 0:
            try:
                message = self.receiver.getString()
                self.receiver.nextPacket()

                data = json.loads(message)
                if data.get("type") == "mission_data":
                    grasp_order = data.get("grasp_order", [])
                    block_positions = data.get("block_positions", {})
                    table_position = data.get("table_position", [0.0, 0.0])

                    # 转换为元组格式
                    positions = {}
                    for color, pos in block_positions.items():
                        positions[color] = (pos[0], pos[1])

                    self.fsm.set_mission_data(grasp_order, positions, tuple(table_position))
                    self.mission_data_received = True
                    return True
            except Exception as e:
                print(f"  ⚠️ 接收数据失败: {e}")

        return False

    def use_default_mission(self):
        """使用默认任务数据（当没有 Supervisor 时）"""
        print("\n📋 使用默认任务数据...")

        grasp_order = ["red", "blue", "green", "yellow"]
        block_positions = {
            "red": (-4.0, 4.0),
            "blue": (4.0, 4.0),
            "green": (4.0, -4.0),
            "yellow": (-4.0, -4.0)
        }

        self.fsm.set_mission_data(grasp_order, block_positions, (0.0, 0.0))
        self.mission_data_received = True

    def handle_keyboard(self):
        """处理键盘输入"""
        key = self.keyboard.getKey()

        if key == ord('V'):  # 打印视觉识别信息
            objects = self.perception.get_visual_objects()
            print("\n" + "=" * 60)
            print("📷 视觉识别物体详细信息")
            print("=" * 60)
            if not objects:
                print("  未识别到任何物体")
            for idx, obj in enumerate(objects, 1):
                pos = obj['position']
                colors = obj.get('colors', [])
                print(f"\n  物体 {idx}: {obj['name']}")
                print(f"  ├─ 位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")
                print(f"  ├─ 距离: {obj['distance']:.3f} m")
                print(f"  ├─ 颜色: {colors}")
                print(f"  └─ 图像坐标: {obj['position_on_image']} px")
            print("=" * 60 + "\n")

        elif key == ord('P'):  # 打印物体识别信息
            blocks = self.perception.detect_blocks_by_color()
            print("\n📦 检测到的彩色木块:")
            for color, x, y, z in blocks:
                print(f"  {COLOR_NAMES.get(color, color)}: ({x:.3f}, {y:.3f}, {z:.3f})")

        elif key == ord('W'):  # 前进
            self.drive.move(0.3, 0.0, 0.0)
        elif key == ord('S'):  # 后退
            self.drive.move(-0.3, 0.0, 0.0)
        elif key == ord('A'):  # 左转
            self.drive.move(0.0, 0.0, 0.8)
        elif key == ord('D'):  # 右转
            self.drive.move(0.0, 0.0, -0.8)
        elif key == ord('Q'):  # 左移
            self.drive.move(0.0, 0.3, 0.0)
        elif key == ord('E'):  # 右移
            self.drive.move(0.0, -0.3, 0.0)
        elif key == ord('G'):  # 夹爪闭合
            self.gripper.close()
        elif key == ord('Y'):  # 夹爪张开
            self.gripper.open()
        elif key == ord(' '):  # 停止
            self.drive.stop()
        elif key == ord('M'):  # 切换模式
            self.auto_mode = not self.auto_mode
            print(f"\n{'='*50}")
            print(f"{'🤖 自动模式' if self.auto_mode else '🎮 手动模式'}")
            print(f"{'='*50}")

    def run(self):
        """主循环"""
        print("\n🚀 开始执行任务...\n")

        while self.robot.step(self.timestep) != -1:
            # 处理键盘输入
            self.handle_keyboard()

            # 自动模式：执行任务
            if self.auto_mode:
                # 1. 接收任务数据
                if not self.mission_data_received:
                    if not self.receive_mission_data():
                        if self.robot.getTime() > 3.0:
                            self.use_default_mission()
                    continue

                # 2. 执行状态机
                completed, error = self.fsm.run()

                if completed:
                    if not self.task_completed:
                        self.task_completed = True
                        print("\n" + "=" * 60)
                        print("🎉 任务完成！所有木块已成功放置到桌面！")
                        print("=" * 60)
                    self.drive.stop()

                if error:
                    if not self.task_error:
                        self.task_error = True
                        print("\n" + "=" * 60)
                        print("❌ 任务出错，请检查控制台日志")
                        print("=" * 60)
                    self.drive.stop()


if __name__ == "__main__":
    controller = YouBotController()
    controller.run()
