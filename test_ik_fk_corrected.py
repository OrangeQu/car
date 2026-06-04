"""
修正版 FK/IK 验证
基于 box.wbt 的实际关节链，axis 0 -1 0 表示绕负Y轴旋转
"""

import math

# ===== 从 box.wbt 提取的精确参数 =====
ARM_BASE_X = 0.156  # ARM Solid translation x

# arm1: anchor 0 0 0.077, axis 0 0 1 (绕Z轴)
L1 = 0.077

# arm2: anchor 0.033 0 0.07, axis 0 -1 0 (绕负Y轴)
# arm2 关节在基座坐标系: (0.156 + 0.033, 0, 0.077 + 0.07) = (0.189, 0, 0.147)
ARM2_ANCHOR_X = 0.033
ARM2_ANCHOR_Z = 0.070
L2 = 0.155

# arm3: anchor 0 0 0.155, axis 0 -1 0
L3 = 0.135

# arm4: anchor 0 0 0.135, axis 0 -1 0
L4 = 0.081

# arm5: anchor 0 0 0.081, axis 0 0 1 (绕Z轴)
L5 = 0.090  # 夹爪长度

# arm2 关节在基座坐标系中的位置
J2_X = ARM_BASE_X + ARM2_ANCHOR_X  # 0.189
J2_Z = L1 + ARM2_ANCHOR_Z  # 0.147


def fk_correct(angles):
    """
    修正版正运动学
    axis 0 -1 0 的旋转矩阵:
    [cos(a)  0  -sin(a)]
    [0       1   0     ]
    [sin(a)  0  cos(a) ]
    
    所以点 (0, 0, L) 旋转后变为 (-L*sin(a), 0, L*cos(a))
    """
    a1, a2, a3, a4, a5 = angles
    
    # arm2 关节位置
    j2_x = J2_X
    j2_z = J2_Z
    
    # arm2 末端（肘部）
    # a2>0 时向后摆，a2<0 时向前摆
    elbow_x = j2_x - L2 * math.sin(a2)
    elbow_z = j2_z + L2 * math.cos(a2)
    
    # arm3 末端（腕部）
    total_a3 = a2 + a3
    wrist_x = elbow_x - L3 * math.sin(total_a3)
    wrist_z = elbow_z + L3 * math.cos(total_a3)
    
    # arm4 末端（夹爪根部）
    total_a4 = a2 + a3 + a4
    gripper_base_x = wrist_x - L4 * math.sin(total_a4)
    gripper_base_z = wrist_z + L4 * math.cos(total_a4)
    
    # arm5 末端（夹爪尖端）
    total_a5 = a2 + a3 + a4
    tip_x = gripper_base_x - L5 * math.sin(total_a5)
    tip_z = gripper_base_z + L5 * math.cos(total_a5)
    
    # arm1 绕 Z 轴旋转
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return (world_x, world_y, world_z)


def ik_correct(forward, left, up):
    """
    修正版逆运动学
    
    已知 FK:
    tip_x = J2_X - L2*sin(a2) - L3*sin(a2+a3) - (L4+L5)*sin(a2+a3+a4)
    tip_z = J2_Z + L2*cos(a2) + L3*cos(a2+a3) + (L4+L5)*cos(a2+a3+a4)
    
    令 target_x = forward, target_z = up
    dx = forward - J2_X
    dz = up - J2_Z
    
    我们需要解:
    dx = -L2*sin(a2) - L3*sin(a2+a3) - L45*sin(a2+a3+a4)
    dz = L2*cos(a2) + L3*cos(a2+a3) + L45*cos(a2+a3+a4)
    
    其中 L45 = L4 + L5
    
    策略：让夹爪垂直向下 (a2+a3+a4 = -pi/2)
    这样 sin(a2+a3+a4) = -1, cos(a2+a3+a4) = 0
    dx = -L2*sin(a2) - L3*sin(a2+a3) + L45
    dz = L2*cos(a2) + L3*cos(a2+a3)
    
    令:
    dx' = dx - L45 = -L2*sin(a2) - L3*sin(a2+a3)
    dz' = dz = L2*cos(a2) + L3*cos(a2+a3)
    
    这是标准的二连杆 IK 问题
    """
    L45 = L4 + L5  # 0.171
    
    # 相对于 arm2 关节的偏移
    dx = forward - J2_X
    dz = up - J2_Z
    
    # 让夹爪垂直向下: a2+a3+a4 = -pi/2
    # 所以 sin(a2+a3+a4) = -1, cos(a2+a3+a4) = 0
    # dx = -L2*sin(a2) - L3*sin(a2+a3) + L45
    # dz = L2*cos(a2) + L3*cos(a2+a3)
    
    dx_prime = dx - L45  # 减去 L45 的贡献
    dz_prime = dz
    
    # 现在解二连杆 IK:
    # dx' = -L2*sin(a2) - L3*sin(a2+a3)
    # dz' = L2*cos(a2) + L3*cos(a2+a3)
    #
    # 令:
    # x1 = L2*sin(a2), z1 = L2*cos(a2)
    # x2 = L3*sin(a2+a3), z2 = L3*cos(a2+a3)
    # dx' = -(x1 + x2), dz' = z1 + z2
    
    # 余弦定理求 a3
    c_sq = dx_prime*dx_prime + dz_prime*dz_prime
    c = math.sqrt(c_sq)
    
    if c > L2 + L3 or c < abs(L2 - L3):
        print(f"  ⚠️ IK 无解: 目标 ({forward:.3f}, {left:.3f}, {up:.3f}) 超出工作空间")
        return None
    
    # a3 的余弦
    cos_a3 = (L2*L2 + L3*L3 - c_sq) / (2*L2*L3)
    cos_a3 = max(-1, min(1, cos_a3))
    
    # 选择 a3 < 0（前臂向下弯曲，适合抓取地面物体）
    a3 = -math.acos(cos_a3)
    
    # 求 a2
    # 从几何关系:
    # dx' = -L2*sin(a2) - L3*sin(a2+a3)
    # dz' = L2*cos(a2) + L3*cos(a2+a3)
    #
    # 展开:
    # dx' = -(L2 + L3*cos(a3))*sin(a2) - L3*sin(a3)*cos(a2)
    # dz' = (L2 + L3*cos(a3))*cos(a2) - L3*sin(a3)*sin(a2)
    #
    # 令:
    # A = L2 + L3*cos(a3)
    # B = L3*sin(a3)
    #
    # dx' = -A*sin(a2) - B*cos(a2)
    # dz' = A*cos(a2) - B*sin(a2)
    #
    # 解:
    # sin(a2) = (-A*dx' - B*dz') / (A*A + B*B)
    # cos(a2) = (A*dz' - B*dx') / (A*A + B*B)
    
    A = L2 + L3 * math.cos(a3)
    B = L3 * math.sin(a3)
    denom = A*A + B*B
    
    sin_a2 = (-A*dx_prime - B*dz_prime) / denom
    cos_a2 = (A*dz_prime - B*dx_prime) / denom
    
    a2 = math.atan2(sin_a2, cos_a2)
    
    # a4: 使夹爪垂直向下
    # a2 + a3 + a4 = -pi/2
    a4 = -math.pi/2 - a2 - a3
    
    # arm1: 指向目标方向
    a1 = math.atan2(left, forward)
    
    # arm5: 保持夹爪方向
    a5 = 0.0
    
    # 限位
    # arm2: [0.01, 3.14] 或 [-3.14, -0.01] 取决于方向
    # 对于地面抓取，a2 应该为负（向前摆）或小正（向后摆）
    # 实际上 arm2 限位是 [-1.13446, 1.5708]
    a2 = max(-1.13446, min(1.5708, a2))
    a3 = max(-3.14, min(-0.01, a3))
    a4 = max(-1.75, min(1.75, a4))
    
    return {"arm1": a1, "arm2": a2, "arm3": a3, "arm4": a4, "arm5": a5}


# ===== 测试 =====
print("=" * 80)
print("修正版 FK/IK 验证")
print("=" * 80)

# 测试各种姿态的 FK
test_poses = [
    ("reset",       (0.0, 1.57, -2.635, 1.78, 0.0)),
    ("carry",       (0.0, 1.2, -1.5, 0.3, 0.0)),
    ("pre_grasp",   (0.0, 0.8, -1.2, 0.0, 0.0)),
    ("grasp",       (0.0, 1.0, -1.8, 0.0, 0.0)),
    ("grasp_low",   (0.0, 0.0, -2.4, -0.5, 0.0)),
    ("place_mid",   (0.0, 0.2, -0.5, 0.0, 0.0)),
    ("place_up",    (0.0, 0.0, 0.0, 0.0, 0.0)),
]

print(f"\n{'姿态':>12} -> {'前':>8} {'左':>8} {'高':>8}")
print("-" * 50)
for name, angles in test_poses:
    pos = fk_correct(angles)
    print(f"{name:>12} -> {pos[0]:8.3f} {pos[1]:8.3f} {pos[2]:8.3f}")

# 测试 IK
print(f"\n{'='*80}")
print("IK 测试")
print(f"{'='*80}")
print(f"{'目标前':>8} {'目标左':>8} {'目标高':>8} -> {'a2':>8} {'a3':>8} {'a4':>8} -> {'验证前':>8} {'验证高':>8} {'误差':>8}")
print("-" * 80)

test_points = [
    (0.12, 0.0, 0.20),   # 木块中心上方 0.15m
    (0.12, 0.0, 0.15),   # 木块中心上方 0.10m
    (0.12, 0.0, 0.10),   # 木块中心上方 0.05m
    (0.10, 0.0, 0.20),
    (0.10, 0.0, 0.15),
    (0.10, 0.0, 0.10),
    (0.08, 0.0, 0.20),
    (0.08, 0.0, 0.15),
    (0.08, 0.0, 0.10),
    (0.15, 0.0, 0.20),
    (0.15, 0.0, 0.15),
    (0.15, 0.0, 0.10),
]

for fx, lf, up in test_points:
    angles = ik_correct(fx, lf, up)
    if angles:
        pos = fk_correct((angles["arm1"], angles["arm2"], angles["arm3"], angles["arm4"], angles["arm5"]))
        err_x = abs(pos[0] - fx)
        err_z = abs(pos[2] - up)
        total_err = math.sqrt(err_x**2 + err_z**2)
        print(f"{fx:8.3f} {lf:8.3f} {up:8.3f} -> "
              f"{angles['arm2']:8.3f} {angles['arm3']:8.3f} {angles['arm4']:8.3f} -> "
              f"{pos[0]:8.3f} {pos[2]:8.3f} {total_err:8.4f}")
    else:
        print(f"{fx:8.3f} {lf:8.3f} {up:8.3f} -> IK无解")

# 验证夹爪方向
print(f"\n{'='*80}")
print("夹爪方向验证")
print(f"{'='*80}")
for fx, lf, up in test_points[:3]:
    angles = ik_correct(fx, lf, up)
    if angles:
        a2, a3, a4 = angles["arm2"], angles["arm3"], angles["arm4"]
        total = a2 + a3 + a4
        print(f"目标({fx:.2f},{up:.2f}): a2+a3+a4={total:.3f}rad = {total*180/math.pi:.1f}° (期望 -90°)")
