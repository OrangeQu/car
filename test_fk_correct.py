import math

# 从 box.wbt 中提取的真实参数
# ARM Solid translation: 0.156 0 0
# arm1 anchor: 0 0 0.077 (基座高度)
# arm2 anchor: 0.033 0 0.07 (相对于 arm1 末端)
# arm3 anchor: 0 0 0.155 (相对于 arm2 末端)
# arm4 anchor: 0 0 0.135 (相对于 arm3 末端)
# arm5 anchor: 0 0 0.081 (相对于 arm4 末端)
# 夹爪末端: 0 0 0.09 (相对于 arm5 末端)

ARM_BASE_X = 0.156  # ARM Solid 在机器人坐标系中的 x 偏移

# 各段长度
L1 = 0.077  # arm1 anchor 高度
L2_OFFSET_X = 0.033  # arm2 anchor 相对于 arm1 末端的 x 偏移
L2_OFFSET_Z = 0.070  # arm2 anchor 相对于 arm1 末端的 z 偏移
L2 = 0.155  # arm2 长度 (arm2 anchor 到 arm3 anchor)
L3 = 0.135  # arm3 长度
L4 = 0.081  # arm4 长度
L5 = 0.090  # arm5 长度 (到夹爪末端)

def fk_correct(angles):
    """
    正确的正运动学，基于 box.wbt 中的实际关节链
    """
    a1, a2, a3, a4, a5 = angles
    
    # 基座位置
    base_x = ARM_BASE_X
    base_z = 0.0
    
    # arm1 末端 (旋转绕 Z 轴)
    # arm1 anchor 在 (0, 0, 0.077) 相对于 ARM Solid
    arm1_end_x = base_x
    arm1_end_y = 0.0
    arm1_end_z = base_z + L1
    
    # arm2 关节位置 (anchor: 0.033 0 0.07 相对于 arm1 末端)
    # 注意: arm2 的 anchor 是相对于 arm1 末端 Solid 的 translation
    # arm1 末端 Solid 的 translation 是 0 0 0.077
    # arm2 anchor 是 0.033 0 0.07
    # 所以 arm2 关节在 arm1 末端坐标系中的位置是 (0.033, 0, 0.07)
    # 在基座坐标系中: (base_x + 0.033, 0, L1 + 0.07)
    shoulder_x = arm1_end_x + L2_OFFSET_X
    shoulder_z = arm1_end_z + L2_OFFSET_Z
    
    # arm2 绕 Y 轴旋转 (a2)
    # arm2 从垂直向上 (a2=0) 向前摆动 (a2>0)
    # arm2 anchor 到 arm3 anchor 的距离是 L2=0.155
    # arm3 anchor 在 arm2 末端坐标系中的位置是 (0, 0, 0.155)
    # 当 a2=0 时，arm2 垂直向上，arm3 anchor 在 (shoulder_x, 0, shoulder_z + L2)
    # 当 a2>0 时，arm2 向前倾斜
    elbow_x = shoulder_x + L2 * math.sin(a2)
    elbow_z = shoulder_z + L2 * math.cos(a2)
    
    # arm3 绕 Y 轴旋转 (a3)
    # arm3 anchor 在 arm2 末端坐标系中的位置是 (0, 0, 0.135)
    # a3 是相对于 arm2 的夹角
    # a3=0 时前臂与上臂在一条直线上
    # a3<0 时前臂向下弯曲
    total_a3 = a2 + a3
    wrist_x = elbow_x + L3 * math.sin(total_a3)
    wrist_z = elbow_z + L3 * math.cos(total_a3)
    
    # arm4 绕 Y 轴旋转 (a4)
    # arm4 anchor 在 arm3 末端坐标系中的位置是 (0, 0, 0.081)
    total_a4 = a2 + a3 + a4
    gripper_base_x = wrist_x + L4 * math.sin(total_a4)
    gripper_base_z = wrist_z + L4 * math.cos(total_a4)
    
    # arm5 绕 Z 轴旋转 (a5) - 不影响位置
    # 夹爪末端在 arm5 末端坐标系中的位置是 (0, 0, 0.09)
    total_a5 = a2 + a3 + a4  # arm5 绕 Z 轴，不影响位置
    tip_x = gripper_base_x + L5 * math.sin(total_a5)
    tip_z = gripper_base_z + L5 * math.cos(total_a5)
    
    # 考虑 arm1 的旋转 (绕 Z 轴)
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return (world_x, world_y, world_z)

# 测试当前 grasp_low 姿态
print("=== 当前 grasp_low 姿态 (arm2=0.1, arm3=-2.6, arm4=-0.7) ===")
angles = (0.0, 0.1, -2.6, -0.7, 0.0)
pos = fk_correct(angles)
print(f"夹爪末端: 前={pos[0]:.3f}m, 左={pos[1]:.3f}m, 高={pos[2]:.3f}m")
print(f"绝对高度 (底盘 0.103m): {pos[2] + 0.103:.3f}m")

print("\n=== 尝试更低的姿态 ===")
test_poses = [
    (0.10, -2.60, -0.70),  # 当前
    (0.10, -2.63, -0.70),
    (0.10, -2.635, -0.70),
    (0.05, -2.60, -0.70),
    (0.05, -2.63, -0.70),
    (0.05, -2.635, -0.70),
    (0.00, -2.60, -0.70),
    (0.00, -2.63, -0.70),
    (0.00, -2.635, -0.70),
    (0.10, -2.60, -0.80),
    (0.10, -2.60, -0.90),
    (0.10, -2.60, -1.00),
    (0.05, -2.60, -0.80),
    (0.05, -2.60, -0.90),
    (0.00, -2.60, -0.80),
    (0.00, -2.60, -0.90),
]

print(f"{'arm2':>6} {'arm3':>8} {'arm4':>8} -> {'前':>8} {'高':>8} {'绝对高':>8}")
print("=" * 55)
for a2, a3, a4 in test_poses:
    angles = (0.0, a2, a3, a4, 0.0)
    pos = fk_correct(angles)
    abs_z = pos[2] + 0.103
    print(f"{a2:6.3f} {a3:8.3f} {a4:8.3f} -> {pos[0]:8.3f} {pos[2]:8.3f} {abs_z:8.3f}")
