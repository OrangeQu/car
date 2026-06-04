"""
精确的 FK/IK 验证，基于 box.wbt 中的实际关节链
"""

import math

# ===== 从 box.wbt 提取的精确参数 =====
ARM_BASE_X = 0.156  # ARM Solid translation x

# arm1: anchor 0 0 0.077, axis 0 0 1 (绕Z轴)
L1 = 0.077

# arm2: anchor 0.033 0 0.07, axis 0 -1 0 (绕负Y轴)
# arm2 的 anchor 相对于 arm1 末端 Solid 的 translation
# arm1 末端 Solid translation: 0 0 0.077
# 所以 arm2 关节在基座坐标系中的位置: (0.156 + 0.033, 0, 0.077 + 0.07) = (0.189, 0, 0.147)
ARM2_ANCHOR_X = 0.033
ARM2_ANCHOR_Z = 0.070
L2 = 0.155  # arm2 长度

# arm3: anchor 0 0 0.155, axis 0 -1 0 (绕负Y轴)
L3 = 0.135

# arm4: anchor 0 0 0.135, axis 0 -1 0 (绕负Y轴)
L4 = 0.081

# arm5: anchor 0 0 0.081, axis 0 0 1 (绕Z轴)
L5 = 0.090  # 夹爪长度

# 注意：arm2 的 axis 是 0 -1 0（绕负Y轴）
# 这意味着正角度是向后摆，负角度是向前摆
# 但 Webots 中 arm2 的初始位置是 1.57（垂直向上）
# 当 arm2 增加时，上臂向后摆；当 arm2 减小时，上臂向前摆

def fk_exact(angles):
    """
    基于 box.wbt 的精确正运动学
    angles: (a1, a2, a3, a4, a5)
    """
    a1, a2, a3, a4, a5 = angles
    
    # 基座位置
    base_x = ARM_BASE_X
    base_z = 0.0
    
    # arm1 关节位置 (anchor 0 0 0.077)
    # arm1 绕 Z 轴旋转
    j1_x = base_x
    j1_z = base_z + L1
    
    # arm2 关节位置 (anchor 0.033 0 0.07 相对于 arm1 末端)
    # arm1 末端 Solid translation: 0 0 0.077
    # 所以 arm2 关节在基座坐标系中: (base_x + 0.033, 0, L1 + 0.07)
    j2_x = j1_x + ARM2_ANCHOR_X
    j2_z = j1_z + ARM2_ANCHOR_Z
    
    # arm2 绕 axis 0 -1 0 (负Y轴) 旋转
    # 当 a2=0 时，上臂垂直向上
    # 当 a2>0 时，上臂向后摆（因为 axis 是负Y轴）
    # 当 a2<0 时，上臂向前摆
    # arm2 长度 L2=0.155，末端在 (0, 0, L2) 相对于关节
    # 旋转后：x = L2 * sin(-a2) = -L2 * sin(a2)
    #         z = L2 * cos(-a2) = L2 * cos(a2)
    # 注意：因为 axis 是 0 -1 0，所以旋转方向相反
    # 实际上，对于 axis 0 -1 0，旋转矩阵是：
    # [cos(a2)  0  -sin(a2)]
    # [0        1   0      ]
    # [sin(a2)  0  cos(a2) ]
    # 所以点 (0, 0, L2) 旋转后变为 (-L2*sin(a2), 0, L2*cos(a2))
    elbow_x = j2_x - L2 * math.sin(a2)
    elbow_z = j2_z + L2 * math.cos(a2)
    
    # arm3 绕 axis 0 -1 0 (负Y轴) 旋转
    # a3 是相对于 arm2 的夹角
    # arm3 长度 L3=0.135
    # 总角度 = a2 + a3
    total_a3 = a2 + a3
    wrist_x = elbow_x - L3 * math.sin(total_a3)
    wrist_z = elbow_z + L3 * math.cos(total_a3)
    
    # arm4 绕 axis 0 -1 0 (负Y轴) 旋转
    total_a4 = a2 + a3 + a4
    gripper_base_x = wrist_x - L4 * math.sin(total_a4)
    gripper_base_z = wrist_z + L4 * math.cos(total_a4)
    
    # arm5 绕 axis 0 0 1 (Z轴) 旋转 - 不影响位置
    # 夹爪末端在 arm5 末端坐标系中: (0, 0.06, 0.09)
    # 但简化起见，我们只考虑 Z 方向
    total_a5 = a2 + a3 + a4  # arm5 绕 Z 轴，不影响 x 和 z
    tip_x = gripper_base_x - L5 * math.sin(total_a5)
    tip_z = gripper_base_z + L5 * math.cos(total_a5)
    
    # 考虑 arm1 的旋转 (绕 Z 轴)
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return (world_x, world_y, world_z)


def fk_simple(angles):
    """
    简化版 FK（当前 arm_controller.py 中的版本）
    用于对比
    """
    a1, a2, a3, a4, a5 = angles
    
    L1 = 0.077
    L2 = 0.155
    L3 = 0.135
    L4 = 0.081
    L5 = 0.090
    
    base_x = ARM_BASE_X
    
    # 当前版本：sin(a2) 为正，即 a2>0 时向前
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
print("FK 精确模型 vs 简化模型 对比")
print("=" * 80)

# 测试各种姿态
test_poses = [
    ("reset",       (0.0, 1.57, -2.635, 1.78, 0.0)),
    ("carry",       (0.0, 1.2, -1.5, 0.3, 0.0)),
    ("pre_grasp",   (0.0, 0.8, -1.2, 0.0, 0.0)),
    ("grasp",       (0.0, 1.0, -1.8, 0.0, 0.0)),
    ("grasp_low",   (0.0, 0.0, -2.4, -0.5, 0.0)),
    ("place_mid",   (0.0, 0.2, -0.5, 0.0, 0.0)),
    ("place_up",    (0.0, 0.0, 0.0, 0.0, 0.0)),
]

print(f"{'姿态':>12} {'a2':>8} {'a3':>8} {'a4':>8} -> ", end="")
print(f"{'精确前':>8} {'精确高':>8} | {'简化前':>8} {'简化高':>8}")
print("-" * 80)

for name, angles in test_poses:
    exact = fk_exact(angles)
    simple = fk_simple(angles)
    print(f"{name:>12} {angles[1]:8.3f} {angles[2]:8.3f} {angles[3]:8.3f} -> "
          f"{exact[0]:8.3f} {exact[2]:8.3f} | {simple[0]:8.3f} {simple[2]:8.3f}")

# 测试 arm2 方向
print(f"\n{'='*80}")
print("arm2 方向测试（精确模型）")
print(f"{'='*80}")
print(f"{'a2':>8} -> {'肘x':>8} {'肘z':>8} {'腕x':>8} {'腕z':>8}")
print("-" * 50)
for a2 in [-0.5, -0.3, -0.1, 0.0, 0.1, 0.3, 0.5, 1.0, 1.57]:
    angles = (0.0, a2, 0.0, 0.0, 0.0)
    pos = fk_exact(angles)
    print(f"{a2:8.3f} -> {pos[0]:8.3f} {pos[2]:8.3f}")

# 测试 IK 应该产生的角度
print(f"\n{'='*80}")
print("目标位置分析")
print(f"{'='*80}")
print("目标: 木块在机器人前方 0.12m，地面高度 0.05m")
print("夹爪需要到达木块中心正上方: 前=0.12m, 高=0.20m (0.05+0.15)")
print()

# 对于精确模型，我们需要找到 IK 解
# 已知 FK: tip_x = base_x - L2*sin(a2) - L3*sin(a2+a3) - (L4+L5)*sin(a2+a3+a4)
#        tip_z = L1 + ARM2_ANCHOR_Z + L2*cos(a2) + L3*cos(a2+a3) + (L4+L5)*cos(a2+a3+a4)
# 
# 其中 base_x = 0.156, L1=0.077, ARM2_ANCHOR_Z=0.07
# 所以 base_z_offset = L1 + ARM2_ANCHOR_Z = 0.147
#
# 令 target_x = 0.12, target_z = 0.20
# dx = target_x - base_x = 0.12 - 0.156 = -0.036
# dz = target_z - (L1 + ARM2_ANCHOR_Z) = 0.20 - 0.147 = 0.053
#
# 所以夹爪需要在 arm2 关节坐标系中到达 (-0.036, 0, 0.053)
# 即向后 0.036m，向上 0.053m

target_x = 0.12
target_z = 0.20
base_x = ARM_BASE_X
base_z_offset = L1 + ARM2_ANCHOR_Z  # 0.147

dx = target_x - base_x  # -0.036
dz = target_z - base_z_offset  # 0.053

print(f"相对于 arm2 关节: dx={dx:.3f}m, dz={dz:.3f}m")
print(f"距离: {math.sqrt(dx**2 + dz**2):.3f}m")
print(f"角度: {math.atan2(dz, dx):.3f}rad")
print()
print("由于 dx 为负（向后），arm2 需要向后摆（a2>0）")
print("但 arm2 向后摆时，夹爪会向后移动，更远离木块中心")
print()
print("这说明：木块在机器人前方 0.12m 时，")
print("机械臂基座在 0.156m，所以木块相对于基座在 -0.036m（后方）")
print("夹爪需要向后伸才能到达木块中心！")
print()
print("但当前 grasp_low 姿态 (a2=0, a3=-2.4, a4=-0.5) 的夹爪位置：")
pos = fk_exact((0.0, 0.0, -2.4, -0.5, 0.0))
print(f"  前={pos[0]:.3f}m, 高={pos[2]:.3f}m")
print(f"  相对于 arm2 关节: dx={pos[0]-base_x:.3f}m, dz={pos[2]-base_z_offset:.3f}m")
