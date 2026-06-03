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

# arm2 极限 -1.13446，arm3 极限 -2.63545
# 测试 arm2 接近极限
print("=== arm2 接近极限 (-1.13), arm3=-2.635, arm4 变化 ===")
for a4 in [0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6, -1.78]:
    angles = (0.0, -1.13, -2.635, a4, 0.0)
    pos = fk_correct(angles)
    abs_z = pos[2] + 0.103
    print(f"arm2=-1.13, arm3=-2.635, arm4={a4:6.2f} -> 前={pos[0]:.3f}, 高={pos[2]:.3f}, 绝对高={abs_z:.3f}")

print("\n=== arm2=-1.13, arm3=-2.635, arm4=0.0 详细 ===")
angles = (0.0, -1.13, -2.635, 0.0, 0.0)
pos = fk_correct(angles)
print(f"夹爪末端: 前={pos[0]:.3f}m, 高={pos[2]:.3f}m, 绝对高={pos[2]+0.103:.3f}m")

# 计算肩膀位置
shoulder_x = ARM_BASE_X + L2_OFFSET_X
shoulder_z = L1 + L2_OFFSET_Z
print(f"肩膀位置: 前={shoulder_x:.3f}m, 高={shoulder_z:.3f}m, 绝对高={shoulder_z+0.103:.3f}m")

# 计算 elbow 位置
elbow_x = shoulder_x + L2 * math.sin(-1.13)
elbow_z = shoulder_z + L2 * math.cos(-1.13)
print(f"肘部位置: 前={elbow_x:.3f}m, 高={elbow_z:.3f}m, 绝对高={elbow_z+0.103:.3f}m")

# 计算 wrist 位置
total_a3 = -1.13 + (-2.635)
wrist_x = elbow_x + L3 * math.sin(total_a3)
wrist_z = elbow_z + L3 * math.cos(total_a3)
print(f"腕部位置: 前={wrist_x:.3f}m, 高={wrist_z:.3f}m, 绝对高={wrist_z+0.103:.3f}m")
print(f"total_a3 = {total_a3:.3f} rad = {math.degrees(total_a3):.1f}°")
