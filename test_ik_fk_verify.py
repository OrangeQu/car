"""
IK/FK 一致性验证脚本
测试 IK 计算出的角度是否能通过 FK 验证到达目标位置
"""

import math

# ===== 从 arm_controller.py 中提取的参数 =====
ARM_LENGTHS = {
    "arm1": 0.077,  # 基座高度
    "arm2": 0.155,  # 上臂
    "arm3": 0.135,  # 前臂
    "arm4": 0.081,  # 腕部
    "arm5": 0.090   # 夹爪
}
ARM_BASE_OFFSET_X = 0.156

def forward_kinematics(angles_dict):
    """从 arm_controller.py 复制的正运动学"""
    a1 = angles_dict.get("arm1", 0.0)
    a2 = angles_dict.get("arm2", 0.0)
    a3 = angles_dict.get("arm3", 0.0)
    a4 = angles_dict.get("arm4", 0.0)
    a5 = angles_dict.get("arm5", 0.0)
    
    L1 = ARM_LENGTHS["arm1"]
    L2 = ARM_LENGTHS["arm2"]
    L3 = ARM_LENGTHS["arm3"]
    L4 = ARM_LENGTHS["arm4"]
    L5 = ARM_LENGTHS["arm5"]
    
    base_x = ARM_BASE_OFFSET_X
    
    elbow_x = base_x + L2 * math.sin(a2)
    elbow_z = L1 + L2 * math.cos(a2)
    
    total_angle_arm3 = a2 + a3
    wrist_x = elbow_x + L3 * math.sin(total_angle_arm3)
    wrist_z = elbow_z + L3 * math.cos(total_angle_arm3)
    
    total_angle_arm4 = a2 + a3 + a4
    gripper_base_x = wrist_x + L4 * math.sin(total_angle_arm4)
    gripper_base_z = wrist_z + L4 * math.cos(total_angle_arm4)
    
    total_angle_arm5 = a2 + a3 + a4
    tip_x = gripper_base_x + L5 * math.sin(total_angle_arm5)
    tip_z = gripper_base_z + L5 * math.cos(total_angle_arm5)
    
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return (world_x, world_y, world_z)

def ik_original(forward, left, up):
    """从 arm_controller.py 复制的原始 IK"""
    y_c = forward
    x_c = left
    z_c = up
    
    y1 = math.sqrt(x_c * x_c + y_c * y_c)
    z1 = z_c + ARM_LENGTHS["arm4"] + ARM_LENGTHS["arm5"] - ARM_LENGTHS["arm1"]
    
    a = ARM_LENGTHS["arm2"]
    b = ARM_LENGTHS["arm3"]
    c = math.sqrt(y1 * y1 + z1 * z1)
    
    if y1 < 0.001:
        y1 = 0.001
    if c > a + b or c < abs(a - b):
        return None
    
    alpha = -math.asin(x_c / y1) if y1 > 0.001 else 0.0
    beta = -(math.pi / 2.0 - math.acos((a * a + c * c - b * b) / (2.0 * a * c)) - math.atan2(z1, y1))
    gamma = -(math.pi - math.acos((a * a + b * b - c * c) / (2.0 * a * b)))
    delta = -(math.pi + (beta + gamma))
    epsilon = math.pi / 2.0 + alpha
    
    beta = max(0.01, min(3.14, beta))
    gamma = max(-3.14, min(-0.01, gamma))
    delta = max(-1.75, min(1.75, delta))
    
    return {"arm1": alpha, "arm2": beta, "arm3": gamma, "arm4": delta, "arm5": epsilon}

def ik_fixed(forward, left, up):
    """
    修正版 IK
    关键修改：z1 的计算公式和角度符号
    """
    y_c = forward
    x_c = left
    z_c = up
    
    y1 = math.sqrt(x_c * x_c + y_c * y_c)
    # 修正：z1 = z_c - (arm1基座高度) 
    # 因为 IK 的坐标系原点在 arm1 基座，而 FK 的坐标系原点在 ARM Solid
    # 但 FK 中 base_x = ARM_BASE_OFFSET_X，base_z = 0
    # 所以 IK 中 z1 应该是目标高度减去 arm1 基座到 arm2 关节的偏移
    # 原始 C 库中 arm1=0.253 包含了基座高度+偏移
    # 现在 arm1=0.077 只是基座高度，所以需要调整
    z1 = z_c - ARM_LENGTHS["arm1"]
    
    a = ARM_LENGTHS["arm2"]
    b = ARM_LENGTHS["arm3"]
    c = math.sqrt(y1 * y1 + z1 * z1)
    
    if y1 < 0.001:
        y1 = 0.001
    if c > a + b or c < abs(a - b):
        print(f"  ⚠️ IK 无解: ({forward:.3f}, {left:.3f}, {up:.3f})")
        return None
    
    # 修正符号：去掉外层的负号
    alpha = math.asin(x_c / y1) if y1 > 0.001 else 0.0
    beta = math.pi / 2.0 - math.acos((a * a + c * c - b * b) / (2.0 * a * c)) - math.atan2(z1, y1)
    gamma = math.pi - math.acos((a * a + b * b - c * c) / (2.0 * a * b))
    delta = math.pi + (beta + gamma)
    epsilon = math.pi / 2.0 + alpha
    
    # 限位
    beta = max(0.01, min(3.14, beta))
    gamma = max(-3.14, min(-0.01, gamma))
    delta = max(-1.75, min(1.75, delta))
    
    return {"arm1": alpha, "arm2": beta, "arm3": gamma, "arm4": delta, "arm5": epsilon}

def ik_fixed_v2(forward, left, up):
    """
    修正版 V2：保持原始公式结构，但调整参数
    原始 C 库 arm_ik(y, z, x) 中 arm1=0.253
    现在 arm1=0.077，所以需要补偿差值
    """
    y_c = forward
    x_c = left
    z_c = up
    
    y1 = math.sqrt(x_c * x_c + y_c * y_c)
    # 原始 C 库: z1 = z_c + arm4 + arm5 - arm1(0.253)
    # 现在 arm1=0.077，差值 = 0.253 - 0.077 = 0.176
    # 所以 z1 = z_c + arm4 + arm5 - arm1(0.077) - 0.176
    # 或者等价于 z1 = z_c + arm4 + arm5 - 0.253
    ARM1_OLD = 0.253  # C 库原始值
    z1 = z_c + ARM_LENGTHS["arm4"] + ARM_LENGTHS["arm5"] - ARM1_OLD
    
    a = ARM_LENGTHS["arm2"]
    b = ARM_LENGTHS["arm3"]
    c = math.sqrt(y1 * y1 + z1 * z1)
    
    if y1 < 0.001:
        y1 = 0.001
    if c > a + b or c < abs(a - b):
        print(f"  ⚠️ IK 无解: ({forward:.3f}, {left:.3f}, {up:.3f})")
        return None
    
    # 保持原始公式的符号
    alpha = -math.asin(x_c / y1) if y1 > 0.001 else 0.0
    beta = -(math.pi / 2.0 - math.acos((a * a + c * c - b * b) / (2.0 * a * c)) - math.atan2(z1, y1))
    gamma = -(math.pi - math.acos((a * a + b * b - c * c) / (2.0 * a * b)))
    delta = -(math.pi + (beta + gamma))
    epsilon = math.pi / 2.0 + alpha
    
    beta = max(0.01, min(3.14, beta))
    gamma = max(-3.14, min(-0.01, gamma))
    delta = max(-1.75, min(1.75, delta))
    
    return {"arm1": alpha, "arm2": beta, "arm3": gamma, "arm4": delta, "arm5": epsilon}

def ik_fixed_v3(forward, left, up):
    """
    修正版 V3：从零推导 IK
    已知 FK: tip_x = base_x + L2*sin(a2) + L3*sin(a2+a3) + (L4+L5)*sin(a2+a3+a4)
           tip_z = L1 + L2*cos(a2) + L3*cos(a2+a3) + (L4+L5)*cos(a2+a3+a4)
    
    令 target_x = forward, target_z = up
    减去 base_x 和 L1:
    dx = forward - ARM_BASE_OFFSET_X
    dz = up - ARM_LENGTHS["arm1"]
    
    然后解三角形
    """
    dx = forward - ARM_BASE_OFFSET_X
    dz = up - ARM_LENGTHS["arm1"]
    
    L2 = ARM_LENGTHS["arm2"]
    L3 = ARM_LENGTHS["arm3"]
    L45 = ARM_LENGTHS["arm4"] + ARM_LENGTHS["arm5"]
    
    # 水平距离和垂直高度
    y1 = math.sqrt(left * left + dx * dx)
    z1 = dz
    
    a = L2
    b = L3
    c = math.sqrt(y1 * y1 + z1 * z1)
    
    if y1 < 0.001:
        y1 = 0.001
    if c > a + b or c < abs(a - b):
        print(f"  ⚠️ IK 无解: ({forward:.3f}, {left:.3f}, {up:.3f})")
        return None
    
    # arm1: 指向目标方向
    alpha = math.atan2(left, dx)
    
    # 解三角形求 arm2, arm3, arm4
    # 余弦定理
    cos_beta = (a*a + c*c - b*b) / (2*a*c)
    cos_beta = max(-1, min(1, cos_beta))
    
    cos_gamma = (a*a + b*b - c*c) / (2*a*b)
    cos_gamma = max(-1, min(1, cos_gamma))
    
    # arm2: 从垂直向上到目标方向的角度
    beta = math.atan2(z1, y1) - math.acos(cos_beta)
    
    # arm3: 上臂和前臂的夹角
    gamma = math.pi - math.acos(cos_gamma)
    
    # arm4: 使夹爪垂直向下
    # 总角度 = a2 + a3 + a4 应该指向下方（-90度）
    # 所以 a4 = -pi/2 - a2 - a3
    delta = -math.pi/2 - beta - gamma
    
    # 限位
    beta = max(0.01, min(3.14, beta))
    gamma = max(-3.14, min(-0.01, gamma))
    delta = max(-1.75, min(1.75, delta))
    
    return {"arm1": alpha, "arm2": beta, "arm3": gamma, "arm4": delta, "arm5": 0.0}


# ===== 测试 =====
print("=" * 80)
print("IK/FK 一致性验证")
print("=" * 80)

test_points = [
    (0.12, 0.0, 0.20),   # 木块中心上方 0.15m (木块高0.05)
    (0.12, 0.0, 0.15),   # 木块中心上方 0.10m
    (0.12, 0.0, 0.10),   # 木块中心上方 0.05m
    (0.10, 0.0, 0.20),
    (0.10, 0.0, 0.15),
    (0.10, 0.0, 0.10),
    (0.08, 0.0, 0.20),
    (0.08, 0.0, 0.15),
    (0.08, 0.0, 0.10),
]

ik_versions = [
    ("原始IK", ik_original),
    ("修正V2(改arm1参数)", ik_fixed_v2),
    ("修正V3(从零推导)", ik_fixed_v3),
]

for name, ik_func in ik_versions:
    print(f"\n{'='*80}")
    print(f"测试: {name}")
    print(f"{'='*80}")
    print(f"{'目标前':>8} {'目标左':>8} {'目标高':>8} -> {'a2':>8} {'a3':>8} {'a4':>8} -> {'验证前':>8} {'验证高':>8} {'误差':>8}")
    print("-" * 80)
    
    for fx, lf, up in test_points:
        angles = ik_func(fx, lf, up)
        if angles:
            pos = forward_kinematics(angles)
            err_x = abs(pos[0] - fx)
            err_z = abs(pos[2] - up)
            total_err = math.sqrt(err_x**2 + err_z**2)
            print(f"{fx:8.3f} {lf:8.3f} {up:8.3f} -> "
                  f"{angles['arm2']:8.3f} {angles['arm3']:8.3f} {angles['arm4']:8.3f} -> "
                  f"{pos[0]:8.3f} {pos[2]:8.3f} {total_err:8.4f}")
        else:
            print(f"{fx:8.3f} {lf:8.3f} {up:8.3f} -> IK无解")

# 额外测试：grasp_low 姿态的 FK
print(f"\n{'='*80}")
print("grasp_low 姿态验证")
print(f"{'='*80}")
grasp_low = {"arm1": 0.0, "arm2": 0.0, "arm3": -2.4, "arm4": -0.5, "arm5": 0.0}
pos = forward_kinematics(grasp_low)
print(f"grasp_low: 前={pos[0]:.3f}m, 左={pos[1]:.3f}m, 高={pos[2]:.3f}m")
