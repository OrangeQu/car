import math

ARM_BASE_X = 0.156
L1 = 0.077
L2_OFFSET_X = 0.033
L2_OFFSET_Z = 0.070
L2 = 0.155
L3 = 0.135
L4 = 0.081
L5 = 0.090

def fk_correct(angles):
    a1, a2, a3, a4, a5 = angles
    base_x = ARM_BASE_X
    base_z = 0.0
    arm1_end_x = base_x
    arm1_end_z = base_z + L1
    shoulder_x = arm1_end_x + L2_OFFSET_X
    shoulder_z = arm1_end_z + L2_OFFSET_Z
    elbow_x = shoulder_x + L2 * math.sin(a2)
    elbow_z = shoulder_z + L2 * math.cos(a2)
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

# arm3 极限是 -2.63545，arm4 极限是 [-1.78024, 1.78024]
# 测试 arm3 接近极限 + arm4 各种值
print("=== arm3=-2.63 (接近极限), arm4 变化 ===")
for a4 in [0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6, -1.78]:
    angles = (0.0, 0.0, -2.63, a4, 0.0)
    pos = fk_correct(angles)
    abs_z = pos[2] + 0.103
    print(f"arm2=0.0, arm3=-2.63, arm4={a4:6.2f} -> 前={pos[0]:.3f}, 高={pos[2]:.3f}, 绝对高={abs_z:.3f}")

print("\n=== arm3=-2.635 (极限), arm4 变化 ===")
for a4 in [0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6, -1.78]:
    angles = (0.0, 0.0, -2.635, a4, 0.0)
    pos = fk_correct(angles)
    abs_z = pos[2] + 0.103
    print(f"arm2=0.0, arm3=-2.635, arm4={a4:6.2f} -> 前={pos[0]:.3f}, 高={pos[2]:.3f}, 绝对高={abs_z:.3f}")

print("\n=== arm2=0.0, arm3=-2.63, arm4 变化 ===")
for a4 in [0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6, -1.78]:
    angles = (0.0, 0.0, -2.63, a4, 0.0)
    pos = fk_correct(angles)
    abs_z = pos[2] + 0.103
    print(f"arm2=0.0, arm3=-2.63, arm4={a4:6.2f} -> 前={pos[0]:.3f}, 高={pos[2]:.3f}, 绝对高={abs_z:.3f}")

print("\n=== 尝试 arm2 负值（向后摆）===")
# arm2 限位是 [-1.13446, 1.5708]，可以向后摆
for a2 in [-0.1, -0.2, -0.3, -0.5, -0.8, -1.0]:
    angles = (0.0, a2, -2.63, 0.0, 0.0)
    pos = fk_correct(angles)
    abs_z = pos[2] + 0.103
    print(f"arm2={a2:6.2f}, arm3=-2.63, arm4=0.0 -> 前={pos[0]:.3f}, 高={pos[2]:.3f}, 绝对高={abs_z:.3f}")

print("\n=== 最佳组合搜索 ===")
# arm2 向后摆 + arm3 极限 + arm4 调整
best = None
best_abs_z = 999
for a2 in [0.0, -0.1, -0.2, -0.3, -0.4, -0.5]:
    for a3 in [-2.60, -2.63, -2.635]:
        for a4 in [0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6, -1.78]:
            angles = (0.0, a2, a3, a4, 0.0)
            pos = fk_correct(angles)
            abs_z = pos[2] + 0.103
            if abs_z < best_abs_z:
                best_abs_z = abs_z
                best = (a2, a3, a4, pos[0], pos[2], abs_z)

print(f"最佳: arm2={best[0]:.3f}, arm3={best[1]:.3f}, arm4={best[2]:.3f}")
print(f"     前={best[3]:.3f}m, 高={best[4]:.3f}m, 绝对高={best[5]:.3f}m")
