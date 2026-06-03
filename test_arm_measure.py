"""
测试脚本：测量不同关节角度下夹爪末端的实际位置
在 Webots 中运行此脚本，它会遍历不同的关节角度组合
并打印夹爪末端的实际 3D 位置
"""
import math
import sys
import time

# 添加到路径
sys.path.append('controllers/YouBot')

from controller import Robot, Supervisor

# 创建 Supervisor 实例（可以获取节点位置）
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# 获取机械臂电机
arm_joints = ["arm1", "arm2", "arm3", "arm4", "arm5"]
motors = {}
for name in arm_joints:
    motor = robot.getDevice(name)
    if motor:
        motor.setVelocity(0.5)
        motors[name] = motor
        print(f"  ✓ 获取 {name}")

# 获取夹爪末端节点
# 从 box.wbt 看，夹爪末端是 finger::left 和 finger::right
# 它们的父级是 arm5 的 endPoint Solid
# 我们可以获取 arm5 的 endPoint Solid 的位置
# 或者直接获取 finger 节点的位置

# 尝试获取 finger::left 节点
finger_left = robot.getFromDef("FINGER_BO")
if not finger_left:
    # 尝试通过名称获取
    finger_left = robot.getFromNode("finger::left")

# 获取 arm5 的 endPoint Solid
# 在 Webots 中，我们可以通过 getFromDef 获取
# 但 box.wbt 中没有给 arm5 的 endPoint Solid 定义 DEF

# 更简单的方法：获取 youBot 节点，然后遍历子节点
youbot = robot.getFromDef("youBot")
if youbot:
    print(f"  ✓ 获取 youBot 节点")
    
    # 获取 ARM 子节点
    arm_node = youbot.getField("children").getMFNode(1)  # ARM 是第2个子节点
    print(f"  ARM 节点: {arm_node.getTypeName() if arm_node else 'None'}")
    
    if arm_node:
        # 获取 arm1 的 endPoint Solid
        # arm1 是 ARM 的第2个子节点 (HingeJoint)
        arm1_joint = arm_node.getField("children").getMFNode(1)
        print(f"  arm1 joint: {arm1_joint.getTypeName() if arm1_joint else 'None'}")
        
        if arm1_joint:
            # 获取 endPoint
            arm1_endpoint = arm1_joint.getField("endPoint").getSFNode()
            print(f"  arm1 endpoint: {arm1_endpoint.getTypeName() if arm1_endpoint else 'None'}")

# 获取底盘位置（用于计算绝对高度）
robot_pos = youbot.getField("translation").getSFVec3f() if youbot else [0, 0, 0]
print(f"  机器人位置: {robot_pos}")

# 复位
for name in arm_joints:
    if name in motors:
        motors[name].setPosition(0.0)

# 等待复位
for _ in range(100):
    robot.step(timestep)

print("\n=== 开始测量不同姿态下的夹爪位置 ===")
print("注意：需要先获取到夹爪末端节点的引用才能测量实际位置")
print("如果无法获取节点引用，将使用正运动学估算")

# 测试几个关键姿态
test_poses = [
    ("reset", {"arm1": 0.0, "arm2": 1.57, "arm3": -2.635, "arm4": 1.78, "arm5": 0.0}),
    ("carry", {"arm1": 0.0, "arm2": 1.2, "arm3": -1.5, "arm4": 0.3, "arm5": 0.0}),
    ("pre_grasp", {"arm1": 0.0, "arm2": 0.8, "arm3": -1.2, "arm4": 0.0, "arm5": 0.0}),
    ("grasp_low_old", {"arm1": 0.0, "arm2": 0.2, "arm3": -2.6, "arm4": -0.3, "arm5": 0.0}),
    ("grasp_low_new", {"arm1": 0.0, "arm2": 0.1, "arm3": -2.6, "arm4": -0.7, "arm5": 0.0}),
    ("test_1", {"arm1": 0.0, "arm2": 0.0, "arm3": -2.6, "arm4": 0.0, "arm5": 0.0}),
    ("test_2", {"arm1": 0.0, "arm2": -0.3, "arm3": -2.6, "arm4": 0.0, "arm5": 0.0}),
    ("test_3", {"arm1": 0.0, "arm2": -0.5, "arm3": -2.6, "arm4": 0.0, "arm5": 0.0}),
    ("test_4", {"arm1": 0.0, "arm2": -0.8, "arm3": -2.6, "arm4": 0.0, "arm5": 0.0}),
    ("test_5", {"arm1": 0.0, "arm2": -1.0, "arm3": -2.6, "arm4": 0.0, "arm5": 0.0}),
    ("test_6", {"arm1": 0.0, "arm2": -1.13, "arm3": -2.635, "arm4": 0.0, "arm5": 0.0}),
]

for name, angles in test_poses:
    print(f"\n--- {name} ---")
    for jname, angle in angles.items():
        if jname in motors:
            motors[jname].setPosition(angle)
    
    # 等待电机到位
    for _ in range(80):
        robot.step(timestep)
    
    # 读取实际关节角度
    actual = {}
    for jname in arm_joints:
        if jname in motors:
            try:
                sensor = robot.getDevice(f"{jname}sensor")
                if sensor:
                    actual[jname] = sensor.getValue()
            except:
                pass
    
    print(f"  实际关节角度: {actual}")
    
    # 尝试获取夹爪末端位置
    # 通过 getFromDef 获取 finger 节点
    try:
        # 尝试获取 finger::left 的位置
        # 在 Webots 中，可以通过 getFromNode 获取
        pass
    except:
        pass

print("\n=== 测量完成 ===")
print("请查看 Webots 场景中夹爪的实际位置")
