import math

# 使用旧公式（没有 arm2 的 0.033 偏移）
ARM_BASE_OFFSET_X = 0.156
ARM_LENGTHS = {
    "arm1": 0.077,
    "arm2": 0.155,
    "arm3": 0.135,
    "arm4": 0.081,
    "arm5": 0.090
}

def fk_old(angles_dict):
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

# 测试各种姿态
print("=== 使用旧公式（无 0.033 偏移）===")
print(f"{'arm2':>6} {'arm3':>8} {'arm4':>8} -> {'前':>8} {'高':>8} {'绝对高':>8}")
print("=" * 55)

test_poses = [
    # 当前 grasp_low
    (0.10, -2.60, -0.70),
    # 尝试 arm2 更小
    (0.05, -2.60, -0.70),
    (0.00, -2.60, -0.70),
    (-0.05, -2.60, -0.70),
    (-0.10, -2.60, -0.70),
    (-0.20, -2.60, -0.70),
    (-0.30, -2.60, -0.70),
    (-0.50, -2.60, -0.70),
    # arm3 极限
    (0.10, -2.635, -0.70),
    (0.00, -2.635, -0.70),
    (-0.10, -2.635, -0.70),
    (-0.20, -2.635, -0.70),
    (-0.30, -2.635, -0.70),
    (-0.50, -2.635, -0.70),
    # arm4 调整
    (0.00, -2.60, -0.50),
    (0.00, -2.60, -0.60),
    (0.00, -2.60, -0.70),
    (0.00, -2.60, -0.80),
    (0.00, -2.60, -0.90),
    (0.00, -2.60, -1.00),
    # arm2 负值 + arm4 调整
    (-0.20, -2.60, -0.50),
    (-0.20, -2.60, -0.60),
    (-0.20, -2.60, -0.70),
    (-0.30, -2.60, -0.40),
    (-0.30, -2.60, -0.50),
    (-0.30, -2.60, -0.60),
]

for a2, a3, a4 in test_poses:
    angles = {"arm1": 0.0, "arm2": a2, "arm3": a3, "arm4": a4, "arm5": 0.0}
    pos = fk_old(angles)
    abs_z = pos[2] + 0.103
    print(f"{a2:6.3f} {a3:8.3f} {a4:8.3f} -> {pos[0]:8.3f} {pos[2]:8.3f} {abs_z:8.3f}")
