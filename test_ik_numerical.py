"""
数值 IK 求解器
基于精确 FK，使用梯度下降法求解 IK
"""

import math
import numpy as np

# ===== 从 box.wbt 提取的精确参数 =====
ARM_BASE_X = 0.156

ANCHORS = {
    "arm1": (0, 0, 0.077),
    "arm2": (0.033, 0, 0.07),
    "arm3": (0, 0, 0.155),
    "arm4": (0, 0, 0.135),
    "arm5": (0, 0, 0.081),
}

AXES = {
    "arm1": (0, 0, 1),
    "arm2": (0, -1, 0),
    "arm3": (0, -1, 0),
    "arm4": (0, -1, 0),
    "arm5": (0, 0, 1),
}

GRIPPER_OFFSET = (0, 0.06, 0.09)

# 关节限位
LIMITS = {
    "arm1": (-2.9496, 2.9496),
    "arm2": (-1.13446, 1.5708),
    "arm3": (-2.63545, 2.54818),
    "arm4": (-1.78024, 1.78024),
    "arm5": (-2.92343, 2.92343),
}


def rotation_matrix(axis, angle):
    x, y, z = axis
    length = math.sqrt(x*x + y*y + z*z)
    if length < 1e-10:
        return np.eye(3)
    x /= length; y /= length; z /= length
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    return np.array([
        [t*x*x + c, t*x*y - z*s, t*x*z + y*s],
        [t*x*y + z*s, t*y*y + c, t*y*z - x*s],
        [t*x*z - y*s, t*y*z + x*s, t*z*z + c]
    ])


def fk_full(angles):
    """精确 FK，返回 (x, y, z)"""
    a1, a2, a3, a4, a5 = angles
    pos = np.array([ARM_BASE_X, 0.0, 0.0])
    
    # arm1
    pos += ANCHORS["arm1"]
    R1 = rotation_matrix(AXES["arm1"], a1)
    pos += R1 @ np.array([0, 0, 0.077])
    
    # arm2
    pos += R1 @ np.array(ANCHORS["arm2"])
    R2 = rotation_matrix(AXES["arm2"], a2)
    pos += R1 @ R2 @ np.array([0.033, 0, 0.07])
    
    # arm3
    pos += R1 @ R2 @ np.array(ANCHORS["arm3"])
    R3 = rotation_matrix(AXES["arm3"], a3)
    pos += R1 @ R2 @ R3 @ np.array([0, 0, 0.155])
    
    # arm4
    pos += R1 @ R2 @ R3 @ np.array(ANCHORS["arm4"])
    R4 = rotation_matrix(AXES["arm4"], a4)
    pos += R1 @ R2 @ R3 @ R4 @ np.array([0, 0, 0.135])
    
    # arm5
    pos += R1 @ R2 @ R3 @ R4 @ np.array(ANCHORS["arm5"])
    R5 = rotation_matrix(AXES["arm5"], a5)
    pos += R1 @ R2 @ R3 @ R4 @ R5 @ np.array([0, 0, 0.081])
    
    # 夹爪
    pos += R1 @ R2 @ R3 @ R4 @ R5 @ np.array(GRIPPER_OFFSET)
    
    return pos


def fk_simplified(angles):
    """
    简化版 FK（只考虑 XZ 平面，a1=0, a5=0）
    用于快速计算
    """
    a1, a2, a3, a4, a5 = angles
    
    # arm1_end
    arm1_end_x = ARM_BASE_X
    arm1_end_z = 0.154
    
    # arm2_joint
    j2_x = arm1_end_x + 0.033
    j2_z = arm1_end_z + 0.07
    
    # arm2_end
    ca2 = math.cos(a2)
    sa2 = math.sin(a2)
    t2_x = 0.033*ca2 - 0.07*sa2
    t2_z = 0.033*sa2 + 0.07*ca2
    arm2_end_x = j2_x + t2_x
    arm2_end_z = j2_z + t2_z
    
    # arm3_joint
    j3_x = arm2_end_x - 0.155 * sa2
    j3_z = arm2_end_z + 0.155 * ca2
    
    # arm3_end
    ca3 = math.cos(a2 + a3)
    sa3 = math.sin(a2 + a3)
    arm3_end_x = j3_x - 0.155 * sa3
    arm3_end_z = j3_z + 0.155 * ca3
    
    # arm4_joint
    j4_x = arm3_end_x - 0.135 * sa3
    j4_z = arm3_end_z + 0.135 * ca3
    
    # arm4_end
    ca4 = math.cos(a2 + a3 + a4)
    sa4 = math.sin(a2 + a3 + a4)
    arm4_end_x = j4_x - 0.135 * sa4
    arm4_end_z = j4_z + 0.135 * ca4
    
    # arm5_joint
    j5_x = arm4_end_x - 0.081 * sa4
    j5_z = arm4_end_z + 0.081 * ca4
    
    # arm5_end
    arm5_end_x = j5_x - 0.081 * sa4
    arm5_end_z = j5_z + 0.081 * ca4
    
    # 夹爪
    tip_x = arm5_end_x - 0.09 * sa4
    tip_z = arm5_end_z + 0.09 * ca4
    
    # arm1 绕 Z 轴旋转
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return np.array([world_x, world_y, world_z])


def ik_numerical(target_pos, initial_guess=None, max_iter=100, tol=1e-4, lr=0.01):
    """
    数值 IK 求解器
    使用梯度下降法
    
    target_pos: (x, y, z) 目标位置
    initial_guess: 初始角度猜测
    """
    if initial_guess is None:
        initial_guess = [0.0, 0.0, -2.0, 0.0, 0.0]
    
    angles = np.array(initial_guess, dtype=float)
    target = np.array(target_pos, dtype=float)
    
    for iteration in range(max_iter):
        # 当前 FK
        pos = fk_full(angles)
        error = pos - target
        err_norm = np.linalg.norm(error)
        
        if err_norm < tol:
            break
        
        # 数值雅可比矩阵
        J = np.zeros((3, 5))
        eps = 1e-6
        
        for i in range(5):
            angles_plus = angles.copy()
            angles_plus[i] += eps
            pos_plus = fk_full(angles_plus)
            J[:, i] = (pos_plus - pos) / eps
        
        # 梯度下降: d(angles) = -lr * J^T * error
        # 使用伪逆
        try:
            J_pinv = np.linalg.pinv(J)
            delta = -lr * J_pinv @ error
        except:
            delta = -lr * J.T @ error
        
        angles += delta
        
        # 限位
        for i, name in enumerate(["arm1", "arm2", "arm3", "arm4", "arm5"]):
            lo, hi = LIMITS[name]
            angles[i] = max(lo, min(hi, angles[i]))
    
    return {
        "arm1": angles[0],
        "arm2": angles[1],
        "arm3": angles[2],
        "arm4": angles[3],
        "arm5": angles[4],
        "error": err_norm,
        "iterations": iteration + 1
    }


# ===== 测试 =====
print("=" * 80)
print("数值 IK 求解器测试")
print("=" * 80)

# 测试各种目标位置
test_targets = [
    ("木块上方 0.15m", (0.12, 0.0, 0.20)),
    ("木块上方 0.10m", (0.12, 0.0, 0.15)),
    ("木块上方 0.05m", (0.12, 0.0, 0.10)),
    ("木块表面", (0.12, 0.0, 0.05)),
    ("桌面表面", (0.38, 0.0, 0.75)),
    ("桌面下方", (0.38, 0.0, 0.70)),
]

for name, target in test_targets:
    print(f"\n--- {name} ---")
    print(f"目标: 前={target[0]:.3f}m, 左={target[1]:.3f}m, 高={target[2]:.3f}m")
    
    # 尝试不同的初始猜测
    for init_name, init in [
        ("grasp_low", [0.0, 0.0, -2.4, -0.5, 0.0]),
        ("place_up", [0.0, 0.0, 0.0, 0.0, 0.0]),
        ("pre_grasp", [0.0, 0.8, -1.2, 0.0, 0.0]),
    ]:
        result = ik_numerical(target, initial_guess=init, max_iter=200, lr=0.5)
        
        # 验证
        angles = [result["arm1"], result["arm2"], result["arm3"], result["arm4"], result["arm5"]]
        pos = fk_full(angles)
        err = np.linalg.norm(pos - np.array(target))
        
        print(f"  初始={init_name:>12}: a2={result['arm2']:7.3f} a3={result['arm3']:7.3f} a4={result['arm4']:7.3f} "
              f"-> 前={pos[0]:.3f} 高={pos[2]:.3f} 误差={err:.4f} ({result['iterations']}次)")

# 测试 IK 能否找到 grasp_low 的精确解
print(f"\n{'='*80}")
print("IK 自洽性测试")
print(f"{'='*80}")

# 已知 grasp_low 的角度
grasp_low_angles = [0.0, 0.0, -2.4, -0.5, 0.0]
grasp_low_pos = fk_full(grasp_low_angles)
print(f"grasp_low 位置: 前={grasp_low_pos[0]:.3f}m, 左={grasp_low_pos[1]:.3f}m, 高={grasp_low_pos[2]:.3f}m")

result = ik_numerical(grasp_low_pos, initial_guess=[0.0, 0.0, -2.0, 0.0, 0.0], max_iter=500, lr=0.5)
angles = [result["arm1"], result["arm2"], result["arm3"], result["arm4"], result["arm5"]]
pos = fk_full(angles)
print(f"IK 求解: a2={result['arm2']:.3f} a3={result['arm3']:.3f} a4={result['arm4']:.3f}")
print(f"验证: 前={pos[0]:.3f}m, 高={pos[2]:.3f}m, 误差={result['error']:.6f}")
