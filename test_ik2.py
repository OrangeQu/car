import math

AL = [0.253, 0.155, 0.135, 0.081, 0.105]

def ik(f, l, u):
    """从 C 库移植的 IK"""
    y1 = math.sqrt(l*l + f*f)
    z1 = u + AL[3] + AL[4] - AL[0]
    a = AL[1]
    b = AL[2]
    c = math.sqrt(y1*y1 + z1*z1)
    if y1 < 0.001:
        y1 = 0.001
    if c > a + b or c < abs(a - b):
        return None
    # C 库: alpha = -asin(x / y1)
    alpha = -math.asin(l / y1) if y1 > 0.001 else 0.0
    # C 库: beta = -(PI/2 - acos((a*a + c*c - b*b)/(2*a*c)) - atan2(z1, y1))
    beta = -(math.pi/2 - math.acos((a*a + c*c - b*b)/(2*a*c)) - math.atan2(z1, y1))
    # C 库: gamma = -(PI - acos((a*a + b*b - c*c)/(2*a*b)))
    gamma = -(math.pi - math.acos((a*a + b*b - c*c)/(2*a*b)))
    # C 库: delta = -(PI + (beta + gamma))
    delta = -(math.pi + (beta + gamma))
    # C 库: epsilon = PI/2 + alpha
    epsilon = math.pi/2 + alpha
    
    print(f"  IK原始: alpha={alpha:.3f}, beta={beta:.3f}, gamma={gamma:.3f}, delta={delta:.3f}, epsilon={epsilon:.3f}")
    
    beta = max(0.01, min(3.14, beta))
    gamma = max(-3.14, min(-0.01, gamma))
    delta = max(-1.75, min(1.75, delta))
    
    return (alpha, beta, gamma, delta, epsilon)

def fk_full(angles):
    """完整正运动学"""
    a1, a2, a3, a4, a5 = angles
    L = AL
    # arm2 关节位置（基座顶部）
    # arm2 绕 Y 轴旋转，a2=0 时上臂垂直向上
    # a2>0 时上臂向前倾斜
    ex = L[1] * math.sin(a2)
    ez = L[0] + L[1] * math.cos(a2)
    
    # arm3 关节（肘部）
    wx = ex + L[2] * math.sin(a2 + a3)
    wz = ez + L[2] * math.cos(a2 + a3)
    
    # arm4 关节（腕部）
    gx = wx + L[3] * math.sin(a2 + a3 + a4)
    gz = wz + L[3] * math.cos(a2 + a3 + a4)
    
    # arm5 末端（夹爪）
    tx = gx + L[4] * math.sin(a2 + a3 + a4)
    tz = gz + L[4] * math.cos(a2 + a3 + a4)
    
    # arm1 旋转
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tx * cos_a1
    world_y = tx * sin_a1
    world_z = tz
    
    return (world_x, world_y, world_z)

# 测试1: 用 reset 姿态验证 IK
print("=" * 60)
print("测试1: 用 reset 姿态验证 IK")
print("=" * 60)
reset_angles = (0.0, 1.57, -2.635, 1.78, 0.0)
reset_pos = fk_full(reset_angles)
print(f"reset 姿态末端位置: 前={reset_pos[0]:.3f}, 左={reset_pos[1]:.3f}, 高={reset_pos[2]:.3f}")

# 用这个位置作为 IK 输入
print(f"\nIK 输入: 前={reset_pos[0]:.3f}, 左={reset_pos[1]:.3f}, 高={reset_pos[2]:.3f}")
ik_result = ik(reset_pos[0], reset_pos[1], reset_pos[2])
if ik_result:
    print(f"IK 输出: a1={ik_result[0]:.3f}, a2={ik_result[1]:.3f}, a3={ik_result[2]:.3f}, a4={ik_result[3]:.3f}, a5={ik_result[4]:.3f}")
    print(f"期望值: a1=0.000, a2=1.570, a3=-2.635, a4=1.780, a5=0.000")
    
    # 验证
    fk_result = fk_full(ik_result)
    print(f"IK->FK 验证: 前={fk_result[0]:.3f}, 左={fk_result[1]:.3f}, 高={fk_result[2]:.3f}")

# 测试2: 尝试不同的目标点
print("\n" + "=" * 60)
print("测试2: 不同目标点")
print("=" * 60)
for f, u in [(0.15, 0.05), (0.15, 0.20), (0.10, 0.05), (0.10, 0.10)]:
    print(f"\n目标: 前={f:.2f}, 高={u:.2f}")
    r = ik(f, 0.0, u)
    if r:
        fk_r = fk_full(r)
        print(f"  IK->FK: 前={fk_r[0]:.3f}, 高={fk_r[1]:.3f}, 高={fk_r[2]:.3f}")
