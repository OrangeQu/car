import math

AL = [0.077, 0.155, 0.135, 0.081, 0.090]
ARM_BASE_OFFSET_X = 0.156

def fk_full(angles):
    """完整正运动学"""
    a1, a2, a3, a4, a5 = angles
    L = AL
    base_x = ARM_BASE_OFFSET_X
    ex = base_x + L[1] * math.sin(a2)
    ez = L[0] + L[1] * math.cos(a2)
    wx = ex + L[2] * math.sin(a2 + a3)
    wz = ez + L[2] * math.cos(a2 + a3)
    gx = wx + L[3] * math.sin(a2 + a3 + a4)
    gz = wz + L[3] * math.cos(a2 + a3 + a4)
    tx = gx + L[4] * math.sin(a2 + a3 + a4)
    tz = gz + L[4] * math.cos(a2 + a3 + a4)
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tx * cos_a1
    world_y = tx * sin_a1
    world_z = tz
    return (world_x, world_y, world_z)

# 测试不同的 grasp_low 姿态
test_poses = [
    # (arm2, arm3, arm4) - 当前
    (0.20, -2.60, -0.30),
    # 尝试 arm2 更小（上臂更垂直），arm3 更负（前臂更向下）
    (0.15, -2.63, -0.20),
    (0.10, -2.63, -0.10),
    (0.05, -2.63, 0.00),
    # 尝试 arm4 更负（腕部更向下）
    (0.20, -2.60, -0.50),
    (0.15, -2.60, -0.50),
    (0.10, -2.60, -0.40),
    # 尝试 arm3 最大负值
    (0.20, -2.635, -0.30),
    (0.15, -2.635, -0.20),
    (0.10, -2.635, -0.10),
]

print(f"{'arm2':>6} {'arm3':>8} {'arm4':>8} -> {'前':>8} {'高':>8} {'绝对高':>8} {'说明'}")
print("=" * 60)

for a2, a3, a4 in test_poses:
    angles = (0.0, a2, a3, a4, 0.0)
    pos = fk_full(angles)
    abs_z = pos[2] + 0.103  # 加上底盘高度
    note = ""
    if abs_z < 0.05:
        note = "✓ 可抓到木块中心"
    elif abs_z < 0.08:
        note = "~ 接近木块顶部"
    else:
        note = "✗ 太高"
    print(f"{a2:6.3f} {a3:8.3f} {a4:8.3f} -> {pos[0]:8.3f} {pos[2]:8.3f} {abs_z:8.3f} {note}")
