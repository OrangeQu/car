"""
独立测试 arm_controller.py 中的 FK 和 IK 函数
不依赖 Webots 的 controller 模块
"""

import math
import numpy as np

# ===== 从 arm_controller.py 复制的 FK 和 IK 函数 =====

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

LIMITS = {
    "arm1": (-2.9496, 2.9496),
    "arm2": (-1.13446, 1.5708),
    "arm3": (-2.63545, 2.54818),
    "arm4": (-1.78024, 1.78024),
    "arm5": (-2.92343, 2.92343),
}

POSES = {
    "reset":       {"arm1": 0.0, "arm2": 1.57, "arm3": -2.635, "arm4": 1.78, "arm5": 0.0},
    "carry":       {"arm1": 0.0, "arm2": 1.2, "arm3": -1.5, "arm4": 0.3, "arm5": 0.0},
    "pre_grasp":   {"arm1": 0.0, "arm2": 0.8, "arm3": -1.2, "arm4": 0.0, "arm5": 0.0},
    "grasp":       {"arm1": 0.0, "arm2": 1.0, "arm3": -1.8, "arm4": 0.0, "arm5": 0.0},
    "grasp_low":   {"arm1": 0.0, "arm2": 0.0, "arm3": -2.4, "arm4": -0.5, "arm5": 0.0},
    "place_mid":   {"arm1": 0.0, "arm2": 0.2, "arm3": -0.5, "arm4": 0.0, "arm5": 0.0},
    "place_up":    {"arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": 0.0, "arm5": 0.0},
    "place_approach": {"arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": -1.50, "arm5": 0.0},
    "place_release":  {"arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": -1.670, "arm5": 0.0},
    "place_retract":  {"arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": 0.0, "arm5": 0.0},
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


def forward_kinematics(angles_dict):
    a1 = angles_dict.get("arm1", 0.0)
    a2 = angles_dict.get("arm2", 0.0)
    a3 = angles_dict.get("arm3", 0.0)
    a4 = angles_dict.get("arm4", 0.0)
    a5 = angles_dict.get("arm5", 0.0)
    
    pos = np.array([ARM_BASE_X, 0.0, 0.0])
    
    pos += ANCHORS["arm1"]
    R1 = rotation_matrix(AXES["arm1"], a1)
    pos += R1 @ np.array([0, 0, 0.077])
    
    pos += R1 @ np.array(ANCHORS["arm2"])
    R2 = rotation_matrix(AXES["arm2"], a2)
    pos += R1 @ R2 @ np.array([0.033, 0, 0.07])
    
    pos += R1 @ R2 @ np.array(ANCHORS["arm3"])
    R3 = rotation_matrix(AXES["arm3"], a3)
    pos += R1 @ R2 @ R3 @ np.array([0, 0, 0.155])
    
    pos += R1 @ R2 @ R3 @ np.array(ANCHORS["arm4"])
    R4 = rotation_matrix(AXES["arm4"], a4)
    pos += R1 @ R2 @ R3 @ R4 @ np.array([0, 0, 0.135])
    
    pos += R1 @ R2 @ R3 @ R4 @ np.array(ANCHORS["arm5"])
    R5 = rotation_matrix(AXES["arm5"], a5)
    pos += R1 @ R2 @ R3 @ R4 @ R5 @ np.array([0, 0, 0.081])
    
    pos += R1 @ R2 @ R3 @ R4 @ R5 @ np.array(GRIPPER_OFFSET)
    
    return (pos[0], pos[1], pos[2])


def ik_numerical(target_pos, initial_guess=None, max_iter=200, tol=1e-3, lr=0.5):
    if initial_guess is None:
        initial_guess = [0.0, 0.0, -2.0, 0.0, 0.0]
    
    angles = np.array(initial_guess, dtype=float)
    target = np.array(target_pos, dtype=float)
    
    for iteration in range(max_iter):
        angles_dict = {"arm1": angles[0], "arm2": angles[1], 
                       "arm3": angles[2], "arm4": angles[3], "arm5": angles[4]}
        pos = np.array(forward_kinematics(angles_dict))
        error = pos - target
        err_norm = np.linalg.norm(error)
        
        if err_norm < tol:
            break
        
        J = np.zeros((3, 5))
        eps = 1e-6
        
        for i in range(5):
            angles_plus = angles.copy()
            angles_plus[i] += eps
            ad = {"arm1": angles_plus[0], "arm2": angles_plus[1],
                  "arm3": angles_plus[2], "arm4": angles_plus[3], "arm5": angles_plus[4]}
            pos_plus = np.array(forward_kinematics(ad))
            J[:, i] = (pos_plus - pos) / eps
        
        try:
            J_pinv = np.linalg.pinv(J)
            delta = -lr * J_pinv @ error
        except:
            delta = -lr * J.T @ error
        
        angles += delta
        
        for i, name in enumerate(["arm1", "arm2", "arm3", "arm4", "arm5"]):
            lo, hi = LIMITS[name]
            angles[i] = max(lo, min(hi, angles[i]))
    
    if err_norm > 0.05:
        return None
    
    return {
        "arm1": angles[0],
        "arm2": angles[1],
        "arm3": angles[2],
        "arm4": angles[3],
        "arm5": angles[4],
    }


# ===== 测试 =====
print("=" * 80)
print("验证 FK（所有预设姿态）")
print("=" * 80)

print(f"\n{'姿态':>16} -> {'前':>8} {'左':>8} {'高':>8}")
print("-" * 55)

for name, angles in POSES.items():
    pos = forward_kinematics(angles)
    print(f"{name:>16} -> {pos[0]:8.3f} {pos[1]:8.3f} {pos[2]:8.3f}")

# 测试 IK 自洽性
print(f"\n{'='*80}")
print("IK 自洽性测试")
print(f"{'='*80}")

test_cases = [
    ("grasp_low 位置", (0.510, 0.060, -0.141)),
    ("桌面表面", (0.380, 0.000, 0.750)),
    ("木块表面", (0.120, 0.000, 0.050)),
    ("木块上方 0.15m", (0.120, 0.000, 0.200)),
]

for name, target in test_cases:
    print(f"\n--- {name} ---")
    print(f"目标: 前={target[0]:.3f}m, 左={target[1]:.3f}m, 高={target[2]:.3f}m")
    
    result = ik_numerical(target, max_iter=500, lr=0.5)
    if result:
        pos = forward_kinematics(result)
        err = math.sqrt((pos[0]-target[0])**2 + (pos[1]-target[1])**2 + (pos[2]-target[2])**2)
        print(f"IK 结果: a2={result['arm2']:.3f} a3={result['arm3']:.3f} a4={result['arm4']:.3f}")
        print(f"验证: 前={pos[0]:.3f}m, 左={pos[1]:.3f}m, 高={pos[2]:.3f}m, 误差={err:.4f}m")
    else:
        print(f"IK 无解")

# 测试 IK 能否找到 grasp_low 的精确解
print(f"\n{'='*80}")
print("IK 自洽性测试 - grasp_low")
print(f"{'='*80}")

grasp_low_angles = POSES["grasp_low"]
grasp_low_pos = forward_kinematics(grasp_low_angles)
print(f"grasp_low 位置: 前={grasp_low_pos[0]:.3f}m, 左={grasp_low_pos[1]:.3f}m, 高={grasp_low_pos[2]:.3f}m")

result = ik_numerical(grasp_low_pos, initial_guess=[0.0, 0.0, -2.0, 0.0, 0.0], max_iter=500, lr=0.5)
if result:
    pos = forward_kinematics(result)
    err = math.sqrt((pos[0]-grasp_low_pos[0])**2 + (pos[1]-grasp_low_pos[1])**2 + (pos[2]-grasp_low_pos[2])**2)
    print(f"IK 结果: a2={result['arm2']:.3f} a3={result['arm3']:.3f} a4={result['arm4']:.3f}")
    print(f"验证: 前={pos[0]:.3f}m, 左={pos[1]:.3f}m, 高={pos[2]:.3f}m, 误差={err:.6f}m")
else:
    print(f"IK 无解")

# 测试 IK 能否找到 place_up 的精确解
print(f"\n{'='*80}")
print("IK 自洽性测试 - place_up")
print(f"{'='*80}")

place_up_angles = POSES["place_up"]
place_up_pos = forward_kinematics(place_up_angles)
print(f"place_up 位置: 前={place_up_pos[0]:.3f}m, 左={place_up_pos[1]:.3f}m, 高={place_up_pos[2]:.3f}m")

result = ik_numerical(place_up_pos, initial_guess=[0.0, 0.0, 0.0, 0.0, 0.0], max_iter=500, lr=0.5)
if result:
    pos = forward_kinematics(result)
    err = math.sqrt((pos[0]-place_up_pos[0])**2 + (pos[1]-place_up_pos[1])**2 + (pos[2]-place_up_pos[2])**2)
    print(f"IK 结果: a2={result['arm2']:.3f} a3={result['arm3']:.3f} a4={result['arm4']:.3f}")
    print(f"验证: 前={pos[0]:.3f}m, 左={pos[1]:.3f}m, 高={pos[2]:.3f}m, 误差={err:.6f}m")
else:
    print(f"IK 无解")
