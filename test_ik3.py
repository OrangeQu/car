import math

AL = [0.253, 0.155, 0.135, 0.081, 0.105]

def fk_full(angles):
    """完整正运动学"""
    a1, a2, a3, a4, a5 = angles
    L = AL
    ex = L[1] * math.sin(a2)
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

# 验证 reset 姿态
reset = (0.0, 1.57, -2.635, 1.78, 0.0)
reset_pos = fk_full(reset)
print(f"reset 位置: 前={reset_pos[0]:.3f}, 高={reset_pos[2]:.3f}")

# 现在手动推导 IK
# 已知: 目标位置 (fx, fy, fz) = (forward, left, up)
# 已知: 关节角度 (a1, a2, a3, a4, a5)
# 
# 从正运动学:
# tx = L2*sin(a2) + L3*sin(a2+a3) + (L4+L5)*sin(a2+a3+a4)
# tz = L1 + L2*cos(a2) + L3*cos(a2+a3) + (L4+L5)*cos(a2+a3+a4)
#
# 令:
# x_end = tx = 夹爪在 XZ 平面的水平投影
# z_end = tz - L1 = 夹爪相对于基座顶部的高度
#
# 实际上，IK 需要解:
# 给定 (fx, fy, fz)，求 (a1, a2, a3, a4, a5)
#
# 步骤1: a1 = atan2(fy, fx)  (绕 Z 轴旋转)
# 步骤2: 在 XZ 平面解 (a2, a3, a4)
#   r = sqrt(fx^2 + fy^2)  (水平距离)
#   h = fz - L1  (相对于基座顶部的高度)
#   
#   但夹爪还有 L4+L5 的长度，所以实际需要到达的点是:
#   r_target = r  (水平距离不变)
#   h_target = h - (L4+L5)  (减去夹爪和腕部长度)
#
#   然后解 2R 机械臂 (L2 和 L3):
#   c = sqrt(r_target^2 + h_target^2)
#   cos_a3 = (r_target^2 + h_target^2 - L2^2 - L3^2) / (2*L2*L3)
#   a3 = acos(cos_a3)
#   a2 = atan2(h_target, r_target) - atan2(L3*sin(a3), L2 + L3*cos(a3))

def ik_new(fx, fy, fz):
    """重新推导的 IK"""
    L = AL
    
    # 步骤1: arm1 (绕 Z 轴旋转)
    r = math.sqrt(fx*fx + fy*fy)
    if r < 0.001:
        a1 = 0.0
    else:
        a1 = math.atan2(fy, fx)
    
    # 步骤2: 在 XZ 平面解 (a2, a3, a4)
    # 目标点相对于基座顶部的高度
    h = fz - L[0]  # 基座高度 0.253
    
    # 减去夹爪和腕部长度 (L4+L5)
    # 注意: 夹爪方向由 a2+a3+a4 决定
    # 这里先假设夹爪水平 (a2+a3+a4 = 0)
    # 实际上 a4 用来调整夹爪方向
    
    # 先解 a2 和 a3 (忽略 a4 和夹爪长度)
    # 目标: 腕部位置 (r_w, h_w)
    # 腕部 = 夹爪位置 - (L4+L5) * (sin(a2+a3+a4), cos(a2+a3+a4))
    # 假设 a2+a3+a4 = 0 (夹爪水平向前)
    r_w = r  # 水平距离不变
    h_w = h - (L[3] + L[4])  # 减去腕部+夹爪
    
    c = math.sqrt(r_w*r_w + h_w*h_w)
    L2, L3 = L[1], L[2]
    
    if c > L2 + L3 or c < abs(L2 - L3):
        print(f"  无解: c={c:.3f}, L2+L3={L2+L3:.3f}")
        return None
    
    # 余弦定理求 a3
    cos_a3 = (L2*L2 + L3*L3 - c*c) / (2*L2*L3)
    cos_a3 = max(-1.0, min(1.0, cos_a3))
    a3 = math.acos(cos_a3)
    # a3 为负表示向下弯曲
    a3 = -a3
    
    # 求 a2
    # atan2(h_w, r_w) 是目标方向角
    # atan2(L3*sin(a3), L2 + L3*cos(a3)) 是 L2 到 L3 的偏移角
    a2 = math.atan2(h_w, r_w) - math.atan2(L3*math.sin(a3), L2 + L3*math.cos(a3))
    
    # a4: 调整夹爪方向，使夹爪水平 (a2+a3+a4 = 0)
    a4 = -(a2 + a3)
    
    # a5: 保持夹爪朝前
    a5 = 0.0
    
    # 限位
    a2 = max(0.01, min(3.14, a2))
    a3 = max(-3.14, min(-0.01, a3))
    a4 = max(-1.75, min(1.75, a4))
    
    return (a1, a2, a3, a4, a5)

# 测试
print("\n" + "=" * 60)
print("测试新 IK")
print("=" * 60)

# 测试 reset 位置
print(f"\n测试 reset 位置 (0.159, 0.0, 0.459):")
r = ik_new(0.159, 0.0, 0.459)
if r:
    print(f"  IK: a1={r[0]:.3f}, a2={r[1]:.3f}, a3={r[2]:.3f}, a4={r[3]:.3f}, a5={r[4]:.3f}")
    print(f"  期望: a1=0.000, a2=1.570, a3=-2.635, a4=1.780, a5=0.000")
    fk_r = fk_full(r)
    print(f"  FK验证: 前={fk_r[0]:.3f}, 高={fk_r[2]:.3f}")

# 测试地面位置
for f, u in [(0.15, 0.05), (0.15, 0.10), (0.15, 0.20), (0.10, 0.05), (0.10, 0.10), (0.08, 0.05)]:
    print(f"\n目标: 前={f:.2f}, 高={u:.2f}")
    r = ik_new(f, 0.0, u)
    if r:
        fk_r = fk_full(r)
        print(f"  IK: a2={r[1]:.3f}, a3={r[2]:.3f}, a4={r[3]:.3f}")
        print(f"  FK验证: 前={fk_r[0]:.3f}, 高={fk_r[2]:.3f}")
