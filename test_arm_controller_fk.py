"""
验证 arm_controller.py 中的 forward_kinematics 和 ik_numerical
"""

import sys
sys.path.insert(0, 'd:\\qts10\\car\\controllers\\YouBot')

import math
import numpy as np

# 直接从 arm_controller.py 导入函数
from arm_controller import forward_kinematics, ik_numerical, POSES

print("=" * 80)
print("验证 arm_controller.py 中的 FK 和 IK")
print("=" * 80)

# 测试所有预设姿态的 FK
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
