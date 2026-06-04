"""
修正版 FK/IK - 使用水平夹爪策略
对于地面抓取，夹爪应该水平向前而不是垂直向下
"""

import math
import sys

# ===== 从 box.wbt 提取的精确参数 =====
ARM_BASE_X = 0.156
L1 = 0.077
ARM2_ANCHOR_X = 0.033
ARM2_ANCHOR_Z = 0.070
L2 = 0.155
L3 = 0.135
L4 = 0.081
L5 = 0.090

J2_X = ARM_BASE_X + ARM2_ANCHOR_X  # 0.189
J2_Z = L1 + ARM2_ANCHOR_Z  # 0.147
L45 = L4 + L5  # 0.171


def fk(angles):
    """FK: axis 0 -1 0 旋转"""
    a1, a2, a3, a4, a5 = angles
    
    elbow_x = J2_X - L2 * math.sin(a2)
    elbow_z = J2_Z + L2 * math.cos(a2)
    
    total_a3 = a2 + a3
    wrist_x = elbow_x - L3 * math.sin(total_a3)
    wrist_z = elbow_z + L3 * math.cos(total_a3)
    
    total_a4 = a2 + a3 + a4
    tip_x = wrist_x - L45 * math.sin(total_a4)
    tip_z = wrist_z + L45 * math.cos(total_a4)
    
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    return (tip_x * cos_a1, tip_x * sin_a1, tip_z)


def ik_ground_grasp(forward, left, up):
    """
    地面抓取 IK
    
    策略：让夹爪水平向前 (a2+a3+a4 = 0)
    这样 sin(a2+a3+a4) = 0, cos(a2+a3+a4) = 1
    tip_x = J2_X - L2*sin(a2) - L3*sin(a2+a3)
    tip_z = J2_Z + L2*cos(a2) + L3*cos(a2+a3) + L45
    
    dx = forward - J2_X = -L2*sin(a2) - L3*sin(a2+a3)
    dz = up - J2_Z - L45 = L2*cos(a2) + L3*cos(a2+a3)
    """
    dx = forward - J2_X
    dz = up - J2_Z - L45
    
    c = math.sqrt(dx*dx + dz*dz)
    
    if c > L2 + L3 or c < abs(L2 - L3):
        return None
    
    # 解二连杆 IK
    cos_a3 = (L2*L2 + L3*L3 - c*c) / (2*L2*L3)
    cos_a3 = max(-1, min(1, cos_a3))
    a3 = -math.acos(cos_a3)  # 前臂向下弯曲
    
    A = L2 + L3 * math.cos(a3)
    B = L3 * math.sin(a3)
    denom = A*A + B*B
    
    # dx = -A*sin(a2) - B*cos(a2)
    # dz = A*cos(a2) - B*sin(a2)
    sin_a2 = (-A*dx - B*dz) / denom
    cos_a2 = (A*dz - B*dx) / denom
    a2 = math.atan2(sin_a2, cos_a2)
    
    # 夹爪水平向前: a2 + a3 + a4 = 0
    a4 = -a2 - a3
    
    # arm1: 指向目标方向
    a1 = math.atan2(left, forward)
    a5 = 0.0
    
    # 限位
    a2 = max(-1.13446, min(1.5708, a2))
    a3 = max(-2.63545, min(-0.01, a3))
    a4 = max(-1.78024, min(1.78024, a4))
    
    return {"arm1": a1, "arm2": a2, "arm3": a3, "arm4": a4, "arm5": a5}


def ik_vertical_down(forward, left, up):
    """
    垂直向下 IK
    
    策略：让夹爪垂直向下 (a2+a3+a4 = -pi/2)
    sin(a2+a3+a4) = -1, cos(a2+a3+a4) = 0
    tip_x = J2_X - L2*sin(a2) - L3*sin(a2+a3) + L45
    tip_z = J2_Z + L2*cos(a2) + L3*cos(a2+a3)
    
    dx = forward - J2_X + L45 = -L2*sin(a2) - L3*sin(a2+a3)
    dz = up - J2_Z = L2*cos(a2) + L3*cos(a2+a3)
    """
    dx = forward - J2_X + L45
    dz = up - J2_Z
    
    c = math.sqrt(dx*dx + dz*dz)
    
    if c > L2 + L3 or c < abs(L2 - L3):
        return None
    
    cos_a3 = (L2*L2 + L3*L3 - c*c) / (2*L2*L3)
    cos_a3 = max(-1, min(1, cos_a3))
    a3 = -math.acos(cos_a3)
    
    A = L2 + L3 * math.cos(a3)
    B = L3 * math.sin(a3)
    denom = A*A + B*B
    
    sin_a2 = (-A*dx - B*dz) / denom
    cos_a2 = (A*dz - B*dx) / denom
    a2 = math.atan2(sin_a2, cos_a2)
    
    a4 = -math.pi/2 - a2 - a3
    
    a1 = math.atan2(left, forward)
    a5 = 0.0
    
    a2 = max(-1.13446, min(1.5708, a2))
    a3 = max(-2.63545, min(-0.01, a3))
    a4 = max(-1.78024, min(1.78024, a4))
    
    return {"arm1": a1, "arm2": a2, "arm3": a3, "arm4": a4, "arm5": a5}


# ===== 测试 =====
print("=" * 80)
print("FK/IK 对比测试")
print("=" * 80)

# 验证已知姿态
print("\n已知姿态验证:")
print(f"{'姿态':>12} -> {'前':>8} {'高':>8}")
print("-" * 40)

test_poses = [
    ("reset",       (0.0, 1.57, -2.635, 1.78, 0.0)),
    ("carry",       (0.0, 1.2, -1.5, 0.3, 0.0)),
    ("pre_grasp",   (0.0, 0.8, -1.2, 0.0, 0.0)),
    ("grasp",       (0.0, 1.0, -1.8, 0.0, 0.0)),
    ("grasp_low",   (0.0, 0.0, -2.4, -0.5, 0.0)),
    ("place_mid",   (0.0, 0.2, -0.5, 0.0, 0.0)),
    ("place_up",    (0.0, 0.0, 0.0, 0.0, 0.0)),
]

for name, angles in test_poses:
    pos = fk(angles)
    print(f"{name:>12} -> {pos[0]:8.3f} {pos[2]:8.3f}")

# 测试两种 IK 策略
print(f"\n{'='*80}")
print("IK 策略对比")
print(f"{'='*80}")

test_points = [
    (0.12, 0.0, 0.20),
    (0.12, 0.0, 0.15),
    (0.12, 0.0, 0.10),
    (0.10, 0.0, 0.20),
    (0.10, 0.0, 0.15),
    (0.10, 0.0, 0.10),
    (0.08, 0.0, 0.20),
    (0.08, 0.0, 0.15),
    (0.08, 0.0, 0.10),
]

for strategy_name, ik_func in [("水平夹爪", ik_ground_grasp), ("垂直向下", ik_vertical_down)]:
    print(f"\n--- {strategy_name} ---")
    print(f"{'目标前':>8} {'目标高':>8} -> {'a2':>8} {'a3':>8} {'a4':>8} -> {'验证前':>8} {'验证高':>8} {'误差':>8}")
    print("-" * 70)
    
    for fx, lf, up in test_points:
        angles = ik_func(fx, lf, up)
        if angles:
            pos = fk((angles["arm1"], angles["arm2"], angles["arm3"], angles["arm4"], angles["arm5"]))
            err = math.sqrt((pos[0]-fx)**2 + (pos[2]-up)**2)
            print(f"{fx:8.3f} {up:8.3f} -> "
                  f"{angles['arm2']:8.3f} {angles['arm3']:8.3f} {angles['arm4']:8.3f} -> "
                  f"{pos[0]:8.3f} {pos[2]:8.3f} {err:8.4f}")
        else:
            print(f"{fx:8.3f} {up:8.3f} -> IK无解")

# 分析 grasp_low 姿态
print(f"\n{'='*80}")
print("grasp_low 姿态分析")
print(f"{'='*80}")

# 当前 grasp_low: a2=0, a3=-2.4, a4=-0.5
# 夹爪方向: a2+a3+a4 = 0 - 2.4 - 0.5 = -2.9rad (-166°)
# 这几乎是垂直向下但稍微向后

# 对于木块在 0.12m 处，夹爪需要到达 0.12m
# 当前 grasp_low 的夹爪在 0.321m（太靠前）
# 需要让夹爪向后移动

# 方案：增大 a2（向后摆上臂）
print("尝试调整 grasp_low 姿态让夹爪到达 0.12m:")
print(f"{'a2':>8} {'a3':>8} {'a4':>8} -> {'前':>8} {'高':>8}")
print("-" * 50)

# 保持 a3=-2.4, a4=-0.5，改变 a2
for a2 in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]:
    pos = fk((0.0, a2, -2.4, -0.5, 0.0))
    print(f"{a2:8.3f} {'-2.400':>8} {'-0.500':>8} -> {pos[0]:8.3f} {pos[2]:8.3f}")

print()
print("最佳方案：使用水平夹爪 IK 计算精确角度")
print("这样夹爪可以精确到达木块中心正上方")
print("然后通过减小 arm2（向后摆）让夹爪下探")
