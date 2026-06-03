import math

AL = [0.077, 0.155, 0.135, 0.081, 0.090]
ARM_BASE_OFFSET_X = 0.156

def fk_full(angles):
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

# 测试 arm4 更负的值（腕部更向下弯）
test_poses = [
    # 最佳组合: arm2=0.15, arm3=-2.60, arm4 从 -0.5 到 -1.5
    (0.15, -2.60, -0.50),
    (0.15, -2.60, -0.70),
    (0.15, -2.60, -0.90),
    (0.15, -2.60, -1.10),
    (0.15, -2.60, -1.30),
    (0.15, -2.60, -1.50),
    (0.15, -2.60, -1.70),
    # arm2=0.10 的组合
    (0.10, -2.60, -0.50),
    (0.10, -2.60, -0.70),
    (0.10, -2.60, -0.90),
    (0.10, -2.60, -1.10),
    (0.10, -2.60, -1.30),
    (0.10, -2.60, -1.50),
    # arm2=0.20 的组合
    (0.20, -2.60, -0.50),
    (0.20, -2.60, -0.70),
    (0.20, -2.60, -0.90),
    (0.20, -2.60, -1.10),
    (0.20, -2.60, -1.30),
    (0.20, -2.60, -1.50),
]

print(f"{'arm2':>6} {'arm3':>8} {'arm4':>8} -> {'前':>8} {'高':>8} {'绝对高':>8}")
print("=" * 55)

for a2, a3, a4 in test_poses:
    angles = (0.0, a2, a3, a4, 0.0)
    pos = fk_full(angles)
    abs_z = pos[2] + 0.103
    print(f"{a2:6.3f} {a3:8.3f} {a4:8.3f} -> {pos[0]:8.3f} {pos[2]:8.3f} {abs_z:8.3f}")
