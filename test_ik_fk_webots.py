"""
基于 Webots 实际关节模型的 FK/IK
axis 0 -1 0 的旋转矩阵推导
"""

import math

# ===== 从 box.wbt 提取的精确参数 =====
ARM_BASE_X = 0.156  # ARM Solid translation x

# arm1: anchor 0 0 0.077, axis 0 0 1 (绕Z轴)
L1 = 0.077

# arm2: anchor 0.033 0 0.07, axis 0 -1 0 (绕负Y轴)
# arm2 初始位置: 1.57 (垂直向上)
# 减小 arm2 -> 向前摆，增加 arm2 -> 向后摆
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


def rotation_y_neg(angle):
    """
    axis 0 -1 0 的旋转矩阵
    绕负Y轴旋转 angle 弧度
    
    标准绕Y轴旋转矩阵:
    [cos(a)  0  sin(a)]
    [0       1  0     ]
    [-sin(a) 0  cos(a)]
    
    绕负Y轴旋转 angle:
    = 绕Y轴旋转 -angle
    [cos(-a)  0  sin(-a)]   [cos(a)  0  -sin(a)]
    [0        1  0       ] = [0       1  0      ]
    [-sin(-a) 0  cos(-a)]   [sin(a)  0  cos(a) ]
    """
    ca = math.cos(angle)
    sa = math.sin(angle)
    # 点 (x, y, z) 旋转后:
    # x' = x*ca + z*(-sa) = x*ca - z*sa
    # y' = y
    # z' = x*sa + z*ca
    return (ca, -sa, sa, ca)  # 返回 (m00, m02, m20, m22)


def fk_webots(angles):
    """
    基于 Webots 实际关节模型的 FK
    angles: (a1, a2, a3, a4, a5)
    """
    a1, a2, a3, a4, a5 = angles
    
    # arm2 关节位置
    j2_x = J2_X
    j2_z = J2_Z
    
    # arm2 绕 axis 0 -1 0 旋转
    # 点 (0, 0, L2) 在 arm2 末端坐标系中
    # 旋转后: x' = 0*ca - L2*(-sa) = L2*sa
    #         z' = 0*sa + L2*ca = L2*ca
    # 注意：这里 sa = sin(a2), ca = cos(a2)
    # 所以 x' = L2*sin(a2), z' = L2*cos(a2)
    # 
    # 当 a2=1.57 (初始位置): x' = L2, z' = 0 (水平向前)
    # 当 a2=0: x' = 0, z' = L2 (垂直向上)
    # 当 a2<0: x' < 0, z' > 0 (向后倾斜)
    # 当 a2 从 1.57 减小: x' 减小, z' 增大 (向前摆)
    elbow_x = j2_x + L2 * math.sin(a2)
    elbow_z = j2_z + L2 * math.cos(a2)
    
    # arm3 绕 axis 0 -1 0 旋转
    # 总角度 = a2 + a3
    total_a3 = a2 + a3
    wrist_x = elbow_x + L3 * math.sin(total_a3)
    wrist_z = elbow_z + L3 * math.cos(total_a3)
    
    # arm4 绕 axis 0 -1 0 旋转
    total_a4 = a2 + a3 + a4
    gripper_base_x = wrist_x + L4 * math.sin(total_a4)
    gripper_base_z = wrist_z + L4 * math.cos(total_a4)
    
    # arm5 绕 axis 0 0 1 旋转 - 不影响 x 和 z
    total_a5 = a2 + a3 + a4
    tip_x = gripper_base_x + L5 * math.sin(total_a5)
    tip_z = gripper_base_z + L5 * math.cos(total_a5)
    
    # arm1 绕 Z 轴旋转
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return (world_x, world_y, world_z)


# ===== 验证已知姿态 =====
print("=" * 80)
print("Webots FK 验证")
print("=" * 80)

# 从 box.wbt 中读取的初始姿态
# arm2=1.57, arm3=-2.635, arm4=1.78
print("\n初始姿态 (arm2=1.57, arm3=-2.635, arm4=1.78):")
pos = fk_webots((0.0, 1.57, -2.635, 1.78, 0.0))
print(f"  前={pos[0]:.3f}m, 高={pos[2]:.3f}m")

print("\ngrasp_low 姿态 (arm2=0.0, arm3=-2.4, arm4=-0.5):")
pos = fk_webots((0.0, 0.0, -2.4, -0.5, 0.0))
print(f"  前={pos[0]:.3f}m, 高={pos[2]:.3f}m")

print("\narm2 方向测试:")
print(f"{'a2':>8} -> {'肘x':>8} {'肘z':>8} {'腕x':>8} {'腕z':>8} {'夹爪x':>8} {'夹爪z':>8}")
print("-" * 70)
for a2 in [1.57, 1.2, 0.8, 0.4, 0.0, -0.4, -0.8, -1.134]:
    pos = fk_webots((0.0, a2, 0.0, 0.0, 0.0))
    print(f"{a2:8.3f} -> {pos[0]:8.3f} {pos[2]:8.3f}")

# 测试 IK 应该产生的角度
print(f"\n{'='*80}")
print("目标位置分析")
print(f"{'='*80}")

# 目标: 木块在机器人前方 0.12m，地面高度 0.05m
# 夹爪需要到达木块中心正上方: 前=0.12m, 高=0.20m
target_x = 0.12
target_z = 0.20

print(f"目标: 前={target_x:.3f}m, 高={target_z:.3f}m")
print(f"arm2 关节位置: 前={J2_X:.3f}m, 高={J2_Z:.3f}m")
print(f"相对于 arm2 关节: dx={target_x - J2_X:.3f}m, dz={target_z - J2_Z:.3f}m")
print()

# 对于 Webots FK:
# tip_x = J2_X + L2*sin(a2) + L3*sin(a2+a3) + (L4+L5)*sin(a2+a3+a4)
# tip_z = J2_Z + L2*cos(a2) + L3*cos(a2+a3) + (L4+L5)*cos(a2+a3+a4)
#
# 令夹爪垂直向下: a2+a3+a4 = -pi/2
# sin(a2+a3+a4) = -1, cos(a2+a3+a4) = 0
# tip_x = J2_X + L2*sin(a2) + L3*sin(a2+a3) - L45
# tip_z = J2_Z + L2*cos(a2) + L3*cos(a2+a3)
#
# dx = target_x - J2_X + L45 = L2*sin(a2) + L3*sin(a2+a3)
# dz = target_z - J2_Z = L2*cos(a2) + L3*cos(a2+a3)

L45 = L4 + L5  # 0.171
dx = target_x - J2_X + L45  # 0.12 - 0.189 + 0.171 = 0.102
dz = target_z - J2_Z  # 0.20 - 0.147 = 0.053

print(f"二连杆 IK 输入: dx={dx:.3f}m, dz={dz:.3f}m")
print(f"距离: {math.sqrt(dx**2 + dz**2):.3f}m")
print(f"L2+L3={L2+L3:.3f}m, |L2-L3|={abs(L2-L3):.3f}m")
print()

# 解二连杆 IK
c_sq = dx*dx + dz*dz
c = math.sqrt(c_sq)

cos_a3 = (L2*L2 + L3*L3 - c_sq) / (2*L2*L3)
cos_a3 = max(-1, min(1, cos_a3))
a3 = -math.acos(cos_a3)  # 前臂向下弯曲

A = L2 + L3 * math.cos(a3)
B = L3 * math.sin(a3)
denom = A*A + B*B

sin_a2 = (A*dx + B*dz) / denom
cos_a2 = (A*dz - B*dx) / denom
a2 = math.atan2(sin_a2, cos_a2)

a4 = -math.pi/2 - a2 - a3

print(f"IK 解:")
print(f"  a2={a2:.3f}rad ({a2*180/math.pi:.1f}°)")
print(f"  a3={a3:.3f}rad ({a3*180/math.pi:.1f}°)")
print(f"  a4={a4:.3f}rad ({a4*180/math.pi:.1f}°)")
print(f"  a2+a3+a4={a2+a3+a4:.3f}rad (期望 -1.571)")

# 验证
pos = fk_webots((0.0, a2, a3, a4, 0.0))
print(f"\n验证: 前={pos[0]:.3f}m, 高={pos[2]:.3f}m")
print(f"误差: dx={abs(pos[0]-target_x):.4f}m, dz={abs(pos[2]-target_z):.4f}m")

# 批量测试
print(f"\n{'='*80}")
print("批量 IK 测试")
print(f"{'='*80}")
print(f"{'目标前':>8} {'目标高':>8} -> {'a2':>8} {'a3':>8} {'a4':>8} -> {'验证前':>8} {'验证高':>8} {'误差':>8}")
print("-" * 70)

test_points = [
    (0.12, 0.20), (0.12, 0.15), (0.12, 0.10),
    (0.10, 0.20), (0.10, 0.15), (0.10, 0.10),
    (0.08, 0.20), (0.08, 0.15), (0.08, 0.10),
    (0.15, 0.20), (0.15, 0.15), (0.15, 0.10),
]

for fx, up in test_points:
    dx = fx - J2_X + L45
    dz = up - J2_Z
    
    c_sq = dx*dx + dz*dz
    c = math.sqrt(c_sq)
    
    if c > L2 + L3 or c < abs(L2 - L3):
        print(f"{fx:8.3f} {up:8.3f} -> IK无解")
        continue
    
    cos_a3 = (L2*L2 + L3*L3 - c_sq) / (2*L2*L3)
    cos_a3 = max(-1, min(1, cos_a3))
    a3 = -math.acos(cos_a3)
    
    A = L2 + L3 * math.cos(a3)
    B = L3 * math.sin(a3)
    denom = A*A + B*B
    
    sin_a2 = (A*dx + B*dz) / denom
    cos_a2 = (A*dz - B*dx) / denom
    a2 = math.atan2(sin_a2, cos_a2)
    
    a4 = -math.pi/2 - a2 - a3
    
    # 限位检查
    a2_clamped = max(-1.13446, min(1.5708, a2))
    a3_clamped = max(-2.63545, min(-0.01, a3))
    a4_clamped = max(-1.78024, min(1.78024, a4))
    
    pos = fk_webots((0.0, a2_clamped, a3_clamped, a4_clamped, 0.0))
    err = math.sqrt((pos[0]-fx)**2 + (pos[2]-up)**2)
    
    in_limits = (a2 == a2_clamped and a3 == a3_clamped and a4 == a4_clamped)
    flag = " ✓" if in_limits else " ⚠"
    print(f"{fx:8.3f} {up:8.3f} -> "
          f"{a2_clamped:8.3f} {a3_clamped:8.3f} {a4_clamped:8.3f} -> "
          f"{pos[0]:8.3f} {pos[2]:8.3f} {err:8.4f}{flag}")
