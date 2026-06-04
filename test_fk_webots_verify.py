"""
使用 Webots 的 getPosition() 来验证 FK 公式
这个脚本模拟 Webots 的关节链计算
"""

import math

# ===== 从 box.wbt 提取的精确参数 =====
ARM_BASE_X = 0.156  # ARM Solid translation x

# 关节链（从 ARM Solid 开始）
# ARM Solid: translation 0.156 0 0
#   arm1: anchor 0 0 0.077, axis 0 0 1
#     arm1_end: translation 0 0 0.077
#       arm2: anchor 0.033 0 0.07, axis 0 -1 0
#         arm2_end: translation 0.033 0 0.07
#           arm3: anchor 0 0 0.155, axis 0 -1 0
#             arm3_end: translation 0 0 0.155
#               arm4: anchor 0 0 0.135, axis 0 -1 0
#                 arm4_end: translation 0 0 0.135
#                   arm5: anchor 0 0 0.081, axis 0 0 1
#                     arm5_end: translation 0 0 0.081
#                       夹爪: translation 0 0.06 0.09

# 每个关节的 anchor 相对于父节点
ANCHORS = {
    "arm1": (0, 0, 0.077),     # anchor 0 0 0.077
    "arm2": (0.033, 0, 0.07),  # anchor 0.033 0 0.07
    "arm3": (0, 0, 0.155),     # anchor 0 0 0.155
    "arm4": (0, 0, 0.135),     # anchor 0 0 0.135
    "arm5": (0, 0, 0.081),     # anchor 0 0 0.081
}

# 每个关节的 axis
AXES = {
    "arm1": (0, 0, 1),    # 绕 Z 轴
    "arm2": (0, -1, 0),   # 绕负 Y 轴
    "arm3": (0, -1, 0),   # 绕负 Y 轴
    "arm4": (0, -1, 0),   # 绕负 Y 轴
    "arm5": (0, 0, 1),    # 绕 Z 轴
}

# 夹爪末端相对于 arm5 末端的偏移
GRIPPER_OFFSET = (0, 0.06, 0.09)


def rotation_matrix(axis, angle):
    """
    计算绕任意轴旋转 angle 弧度的旋转矩阵
    使用 Rodrigues 旋转公式
    """
    x, y, z = axis
    # 归一化
    length = math.sqrt(x*x + y*y + z*z)
    if length < 1e-10:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x /= length
    y /= length
    z /= length
    
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    
    return [
        [t*x*x + c, t*x*y - z*s, t*x*z + y*s],
        [t*x*y + z*s, t*y*y + c, t*y*z - x*s],
        [t*x*z - y*s, t*y*z + x*s, t*z*z + c]
    ]


def transform_point(matrix, point):
    """应用 3x3 旋转矩阵到点"""
    x, y, z = point
    return (
        matrix[0][0]*x + matrix[0][1]*y + matrix[0][2]*z,
        matrix[1][0]*x + matrix[1][1]*y + matrix[1][2]*z,
        matrix[2][0]*x + matrix[2][1]*y + matrix[2][2]*z
    )


def fk_webots_exact(angles):
    """
    使用 Webots 的关节链计算 FK
    每个关节的旋转是相对于父关节的
    """
    a1, a2, a3, a4, a5 = angles
    
    # 从 ARM Solid 开始
    # ARM Solid 在机器人坐标系中的位置
    pos = [ARM_BASE_X, 0.0, 0.0]
    
    # arm1: anchor 0 0 0.077, axis 0 0 1
    # 先移动到 anchor 位置
    anchor1 = ANCHORS["arm1"]
    pos[0] += anchor1[0]
    pos[1] += anchor1[1]
    pos[2] += anchor1[2]
    
    # arm1 绕 Z 轴旋转
    R1 = rotation_matrix(AXES["arm1"], a1)
    # 注意：旋转后，后续的 anchor 和 translation 都要旋转
    
    # arm1 末端: translation 0 0 0.077
    # 这个 translation 是在 arm1 旋转后的坐标系中
    t1 = (0, 0, 0.077)
    t1_rot = transform_point(R1, t1)
    pos[0] += t1_rot[0]
    pos[1] += t1_rot[1]
    pos[2] += t1_rot[2]
    
    # arm2: anchor 0.033 0 0.07, axis 0 -1 0
    anchor2 = ANCHORS["arm2"]
    a2_rot = transform_point(R1, anchor2)
    pos[0] += a2_rot[0]
    pos[1] += a2_rot[1]
    pos[2] += a2_rot[2]
    
    # arm2 绕 axis 0 -1 0 旋转
    # 注意：arm2 的 axis 是在 arm1 旋转后的坐标系中
    # 但 axis 0 -1 0 在 arm1 旋转后不变（因为 arm1 绕 Z 轴旋转，不影响 Y 轴）
    R2 = rotation_matrix(AXES["arm2"], a2)
    
    # arm2 末端: translation 0.033 0 0.07
    t2 = (0.033, 0, 0.07)
    # 这个 translation 是在 arm2 旋转前的坐标系中
    # 先应用 arm1 的旋转，再应用 arm2 的旋转
    t2_rot = transform_point(R1, transform_point(R2, t2))
    pos[0] += t2_rot[0]
    pos[1] += t2_rot[1]
    pos[2] += t2_rot[2]
    
    # arm3: anchor 0 0 0.155, axis 0 -1 0
    anchor3 = ANCHORS["arm3"]
    a3_rot = transform_point(R1, transform_point(R2, anchor3))
    pos[0] += a3_rot[0]
    pos[1] += a3_rot[1]
    pos[2] += a3_rot[2]
    
    R3 = rotation_matrix(AXES["arm3"], a3)
    
    # arm3 末端: translation 0 0 0.155
    t3 = (0, 0, 0.155)
    t3_rot = transform_point(R1, transform_point(R2, transform_point(R3, t3)))
    pos[0] += t3_rot[0]
    pos[1] += t3_rot[1]
    pos[2] += t3_rot[2]
    
    # arm4: anchor 0 0 0.135, axis 0 -1 0
    anchor4 = ANCHORS["arm4"]
    a4_rot = transform_point(R1, transform_point(R2, transform_point(R3, anchor4)))
    pos[0] += a4_rot[0]
    pos[1] += a4_rot[1]
    pos[2] += a4_rot[2]
    
    R4 = rotation_matrix(AXES["arm4"], a4)
    
    # arm4 末端: translation 0 0 0.135
    t4 = (0, 0, 0.135)
    t4_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, t4))))
    pos[0] += t4_rot[0]
    pos[1] += t4_rot[1]
    pos[2] += t4_rot[2]
    
    # arm5: anchor 0 0 0.081, axis 0 0 1
    anchor5 = ANCHORS["arm5"]
    a5_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, anchor5))))
    pos[0] += a5_rot[0]
    pos[1] += a5_rot[1]
    pos[2] += a5_rot[2]
    
    R5 = rotation_matrix(AXES["arm5"], a5)
    
    # arm5 末端: translation 0 0 0.081
    t5 = (0, 0, 0.081)
    t5_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, transform_point(R5, t5)))))
    pos[0] += t5_rot[0]
    pos[1] += t5_rot[1]
    pos[2] += t5_rot[2]
    
    # 夹爪末端: translation 0 0.06 0.09
    gripper = GRIPPER_OFFSET
    g_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, transform_point(R5, gripper)))))
    pos[0] += g_rot[0]
    pos[1] += g_rot[1]
    pos[2] += g_rot[2]
    
    return tuple(pos)


def fk_simple(angles):
    """简化版 FK（当前 arm_controller.py 中的版本）"""
    a1, a2, a3, a4, a5 = angles
    
    L1 = 0.077
    L2 = 0.155
    L3 = 0.135
    L4 = 0.081
    L5 = 0.090
    
    base_x = ARM_BASE_X
    
    elbow_x = base_x + L2 * math.sin(a2)
    elbow_z = L1 + L2 * math.cos(a2)
    
    total_a3 = a2 + a3
    wrist_x = elbow_x + L3 * math.sin(total_a3)
    wrist_z = elbow_z + L3 * math.cos(total_a3)
    
    total_a4 = a2 + a3 + a4
    gripper_base_x = wrist_x + L4 * math.sin(total_a4)
    gripper_base_z = wrist_z + L4 * math.cos(total_a4)
    
    total_a5 = a2 + a3 + a4
    tip_x = gripper_base_x + L5 * math.sin(total_a5)
    tip_z = gripper_base_z + L5 * math.cos(total_a5)
    
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return (world_x, world_y, world_z)


# ===== 测试 =====
print("=" * 80)
print("Webots 精确 FK vs 简化 FK")
print("=" * 80)

test_poses = [
    ("reset",       (0.0, 1.57, -2.635, 1.78, 0.0)),
    ("carry",       (0.0, 1.2, -1.5, 0.3, 0.0)),
    ("pre_grasp",   (0.0, 0.8, -1.2, 0.0, 0.0)),
    ("grasp",       (0.0, 1.0, -1.8, 0.0, 0.0)),
    ("grasp_low",   (0.0, 0.0, -2.4, -0.5, 0.0)),
    ("place_mid",   (0.0, 0.2, -0.5, 0.0, 0.0)),
    ("place_up",    (0.0, 0.0, 0.0, 0.0, 0.0)),
    ("place_approach", (0.0, 0.0, 0.0, -1.5, 0.0)),
    ("place_release",  (0.0, 0.0, 0.0, -1.67, 0.0)),
]

print(f"\n{'姿态':>16} -> {'精确前':>8} {'精确左':>8} {'精确高':>8} | {'简化前':>8} {'简化高':>8}")
print("-" * 80)

for name, angles in test_poses:
    exact = fk_webots_exact(angles)
    simple = fk_simple(angles)
    print(f"{name:>16} -> {exact[0]:8.3f} {exact[1]:8.3f} {exact[2]:8.3f} | {simple[0]:8.3f} {simple[2]:8.3f}")

# 测试 arm2 方向
print(f"\n{'='*80}")
print("arm2 方向测试（精确 FK）")
print(f"{'='*80}")
print(f"{'a2':>8} -> {'前':>8} {'左':>8} {'高':>8}")
print("-" * 45)
for a2 in [1.57, 1.2, 0.8, 0.4, 0.0, -0.4, -0.8, -1.134]:
    pos = fk_webots_exact((0.0, a2, 0.0, 0.0, 0.0))
    print(f"{a2:8.3f} -> {pos[0]:8.3f} {pos[1]:8.3f} {pos[2]:8.3f}")

# 测试 arm3 方向
print(f"\n{'='*80}")
print("arm3 方向测试（精确 FK，a2=0）")
print(f"{'='*80}")
print(f"{'a3':>8} -> {'前':>8} {'左':>8} {'高':>8}")
print("-" * 45)
for a3 in [0.0, -0.5, -1.0, -1.5, -2.0, -2.5, -2.635]:
    pos = fk_webots_exact((0.0, 0.0, a3, 0.0, 0.0))
    print(f"{a3:8.3f} -> {pos[0]:8.3f} {pos[1]:8.3f} {pos[2]:8.3f}")
