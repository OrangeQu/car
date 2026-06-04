"""
基于精确 FK 的 IK 推导
使用 Rodrigues 旋转矩阵验证
"""

import math

# ===== 从 box.wbt 提取的精确参数 =====
ARM_BASE_X = 0.156

# 关节链参数
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


def rotation_matrix(axis, angle):
    x, y, z = axis
    length = math.sqrt(x*x + y*y + z*z)
    if length < 1e-10:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x /= length; y /= length; z /= length
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    return [
        [t*x*x + c, t*x*y - z*s, t*x*z + y*s],
        [t*x*y + z*s, t*y*y + c, t*y*z - x*s],
        [t*x*z - y*s, t*y*z + x*s, t*z*z + c]
    ]


def transform_point(matrix, point):
    x, y, z = point
    return (
        matrix[0][0]*x + matrix[0][1]*y + matrix[0][2]*z,
        matrix[1][0]*x + matrix[1][1]*y + matrix[1][2]*z,
        matrix[2][0]*x + matrix[2][1]*y + matrix[2][2]*z
    )


def fk_accurate(angles):
    """精确 FK"""
    a1, a2, a3, a4, a5 = angles
    pos = [ARM_BASE_X, 0.0, 0.0]
    
    # arm1
    anchor1 = ANCHORS["arm1"]
    pos[0] += anchor1[0]; pos[1] += anchor1[1]; pos[2] += anchor1[2]
    R1 = rotation_matrix(AXES["arm1"], a1)
    
    t1 = (0, 0, 0.077)
    t1_rot = transform_point(R1, t1)
    pos[0] += t1_rot[0]; pos[1] += t1_rot[1]; pos[2] += t1_rot[2]
    
    # arm2
    anchor2 = ANCHORS["arm2"]
    a2_rot = transform_point(R1, anchor2)
    pos[0] += a2_rot[0]; pos[1] += a2_rot[1]; pos[2] += a2_rot[2]
    
    R2 = rotation_matrix(AXES["arm2"], a2)
    
    t2 = (0.033, 0, 0.07)
    t2_rot = transform_point(R1, transform_point(R2, t2))
    pos[0] += t2_rot[0]; pos[1] += t2_rot[1]; pos[2] += t2_rot[2]
    
    # arm3
    anchor3 = ANCHORS["arm3"]
    a3_rot = transform_point(R1, transform_point(R2, anchor3))
    pos[0] += a3_rot[0]; pos[1] += a3_rot[1]; pos[2] += a3_rot[2]
    
    R3 = rotation_matrix(AXES["arm3"], a3)
    
    t3 = (0, 0, 0.155)
    t3_rot = transform_point(R1, transform_point(R2, transform_point(R3, t3)))
    pos[0] += t3_rot[0]; pos[1] += t3_rot[1]; pos[2] += t3_rot[2]
    
    # arm4
    anchor4 = ANCHORS["arm4"]
    a4_rot = transform_point(R1, transform_point(R2, transform_point(R3, anchor4)))
    pos[0] += a4_rot[0]; pos[1] += a4_rot[1]; pos[2] += a4_rot[2]
    
    R4 = rotation_matrix(AXES["arm4"], a4)
    
    t4 = (0, 0, 0.135)
    t4_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, t4))))
    pos[0] += t4_rot[0]; pos[1] += t4_rot[1]; pos[2] += t4_rot[2]
    
    # arm5
    anchor5 = ANCHORS["arm5"]
    a5_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, anchor5))))
    pos[0] += a5_rot[0]; pos[1] += a5_rot[1]; pos[2] += a5_rot[2]
    
    R5 = rotation_matrix(AXES["arm5"], a5)
    
    t5 = (0, 0, 0.081)
    t5_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, transform_point(R5, t5)))))
    pos[0] += t5_rot[0]; pos[1] += t5_rot[1]; pos[2] += t5_rot[2]
    
    # 夹爪
    g_rot = transform_point(R1, transform_point(R2, transform_point(R3, transform_point(R4, transform_point(R5, GRIPPER_OFFSET)))))
    pos[0] += g_rot[0]; pos[1] += g_rot[1]; pos[2] += g_rot[2]
    
    return tuple(pos)


def fk_simplified(angles):
    """
    简化版 FK（用于 IK 推导）
    只考虑 XZ 平面（a1=0 时）
    """
    a1, a2, a3, a4, a5 = angles
    
    # 对于 axis 0 -1 0:
    # 绕负Y轴旋转 angle 的矩阵:
    # [cos(a)  0  -sin(a)]
    # [0       1   0     ]
    # [sin(a)  0  cos(a) ]
    #
    # 点 (0, 0, L) 旋转后: (-L*sin(a), 0, L*cos(a))
    
    # 基座位置
    base_x = ARM_BASE_X
    base_z = 0.0
    
    # arm1 anchor + translation
    # anchor 0 0 0.077, translation 0 0 0.077
    # 总偏移: 0 0 0.154
    # 但 arm1 绕 Z 轴旋转，不影响 XZ 平面（当 a1=0 时）
    j2_x = base_x + 0.033  # anchor2.x
    j2_z = 0.077 + 0.07   # L1 + anchor2.z
    
    # arm2: axis 0 -1 0
    # 点 (0, 0, L2) 旋转后: (-L2*sin(a2), 0, L2*cos(a2))
    # 但 arm2 的 translation 是 0.033 0 0.07
    # 这个 translation 是在 arm2 旋转前的坐标系中
    # 所以先旋转再平移
    # 实际上，在 Webots 中，translation 是 endPoint Solid 的 translation
    # 它是在关节旋转后的坐标系中
    # 所以: 先旋转 (0,0,L2) -> (-L2*sin(a2), 0, L2*cos(a2))
    # 然后加上 translation (0.033, 0, 0.07)
    # 但 translation 是在旋转后的坐标系中
    # 所以: (-L2*sin(a2) + 0.033, 0, L2*cos(a2) + 0.07)
    
    # 等等，让我重新理解 Webots 的关节链
    # HingeJoint {
    #   jointParameters HingeJointParameters {
    #     anchor 0.033 0 0.07
    #   }
    #   endPoint Solid {
    #     translation 0.033 0 0.07
    #   }
    # }
    # anchor 是关节在父坐标系中的位置
    # translation 是 endPoint Solid 在关节坐标系中的位置
    # 关节坐标系 = 父坐标系平移到 anchor 位置，然后旋转
    
    # 所以:
    # 1. 从父坐标系平移到 anchor 位置
    # 2. 绕 axis 旋转
    # 3. 在旋转后的坐标系中，endPoint Solid 在 translation 位置
    
    # 对于 arm2:
    # 父坐标系 = arm1 的 endPoint Solid 坐标系
    # anchor = (0.033, 0, 0.07) 在 arm1 的 endPoint Solid 坐标系中
    # 旋转后，endPoint Solid 在 (0.033, 0, 0.07) 在旋转后的坐标系中
    
    # 所以 arm2 的 endPoint Solid 在世界坐标系中的位置:
    # = arm1_end_pos + R1 * (anchor + R2 * translation)
    # = arm1_end_pos + R1 * anchor + R1 * R2 * translation
    
    # 对于 a1=0, R1 = I:
    # = arm1_end_pos + anchor + R2 * translation
    # = (base_x + 0 + 0.033, 0, 0 + 0.077 + 0.07) + R2 * (0.033, 0, 0.07)
    # = (0.189, 0, 0.147) + R2 * (0.033, 0, 0.07)
    
    # R2 是绕 axis 0 -1 0 旋转 a2
    # R2 * (0.033, 0, 0.07):
    # x' = 0.033*cos(a2) + 0.07*(-sin(a2)) = 0.033*cos(a2) - 0.07*sin(a2)
    # z' = 0.033*sin(a2) + 0.07*cos(a2)
    
    # 所以 arm2_end = (0.189 + 0.033*cos(a2) - 0.07*sin(a2), 0, 0.147 + 0.033*sin(a2) + 0.07*cos(a2))
    
    # 但 arm2 的长度是 0.155，这个长度体现在 arm2 的 mesh 中
    # arm2 的 mesh 从 (0,0,0) 延伸到 (0,0,0.155) 在 arm2 的 endPoint Solid 坐标系中
    
    # 实际上，对于 FK 计算，我们关心的是关节位置，而不是 mesh
    # 关节位置由 anchor 和 translation 决定
    
    # 让我用更简单的方法：
    # 每个关节的位置 = 父关节位置 + 父旋转 * anchor + 父旋转 * 子旋转 * translation
    
    # 对于 a1=0（R1=I）:
    # arm1_end = (base_x, 0, 0) + (0, 0, 0.077) + I * (0, 0, 0.077) = (base_x, 0, 0.154)
    # arm2_joint = arm1_end + I * (0.033, 0, 0.07) = (base_x + 0.033, 0, 0.224)
    # arm2_end = arm2_joint + R2 * (0.033, 0, 0.07)
    # arm3_joint = arm2_end + R2 * (0, 0, 0.155)
    # arm3_end = arm3_joint + R2 * R3 * (0, 0, 0.155)
    # ...
    
    # 这太复杂了。让我直接用精确 FK 的结果来拟合简化模型
    
    # 从精确 FK 结果：
    # place_up (a2=0, a3=0, a4=0): 前=0.222, 高=1.126
    # 当 a2=0, a3=0, a4=0 时，所有关节垂直向上
    # 总高度 = 0.077 + 0.077 + 0.07 + 0.155 + 0.135 + 0.081 + 0.09 = 0.685
    # 但精确 FK 显示 1.126，说明我的理解不对
    
    # 实际上，当 a2=0 时，arm2 垂直向上
    # arm2 的 anchor 在 (0.033, 0, 0.07) 相对于 arm1_end
    # arm1_end 在 (base_x, 0, 0.154)
    # 所以 arm2_joint 在 (base_x + 0.033, 0, 0.154 + 0.07) = (0.189, 0, 0.224)
    # arm2 垂直向上，所以 arm2_end 在 (0.189, 0, 0.224 + 0.155) = (0.189, 0, 0.379)
    # arm3 垂直向上，arm3_end 在 (0.189, 0, 0.379 + 0.135) = (0.189, 0, 0.514)
    # arm4 垂直向上，arm4_end 在 (0.189, 0, 0.514 + 0.081) = (0.189, 0, 0.595)
    # arm5 垂直向上，arm5_end 在 (0.189, 0, 0.595 + 0.081) = (0.189, 0, 0.676)
    # 夹爪在 (0.189, 0.06, 0.676 + 0.09) = (0.189, 0.06, 0.766)
    
    # 但精确 FK 显示 1.126！差了很多
    
    # 问题在于：每个关节的 translation 是在旋转后的坐标系中
    # 当 a2=0 时，R2 = I，所以 translation 不变
    # 但 arm2 的 translation 是 (0.033, 0, 0.07)
    # 这个 translation 是在 arm2 旋转后的坐标系中
    # 当 a2=0 时，旋转后的坐标系 = 父坐标系
    # 所以 arm2_end = arm2_joint + (0.033, 0, 0.07)
    # = (0.189, 0, 0.224) + (0.033, 0, 0.07) = (0.222, 0, 0.294)
    
    # 啊！我明白了！每个关节的 translation 是 endPoint Solid 相对于关节的位置
    # 而关节在 anchor 位置
    # 所以 arm2_end = arm2_joint + R2 * translation
    # 当 a2=0: arm2_end = (0.189, 0, 0.224) + (0.033, 0, 0.07) = (0.222, 0, 0.294)
    
    # 然后 arm3 的 anchor 在 (0, 0, 0.155) 相对于 arm2_end
    # arm3_joint = arm2_end + R2 * (0, 0, 0.155)
    # 当 a2=0: arm3_joint = (0.222, 0, 0.294) + (0, 0, 0.155) = (0.222, 0, 0.449)
    # arm3_end = arm3_joint + R2 * R3 * (0, 0, 0.155)
    # 当 a3=0: arm3_end = (0.222, 0, 0.449) + (0, 0, 0.155) = (0.222, 0, 0.604)
    
    # 继续...
    # arm4_joint = (0.222, 0, 0.604) + (0, 0, 0.135) = (0.222, 0, 0.739)
    # arm4_end = (0.222, 0, 0.739) + (0, 0, 0.135) = (0.222, 0, 0.874)
    # arm5_joint = (0.222, 0, 0.874) + (0, 0, 0.081) = (0.222, 0, 0.955)
    # arm5_end = (0.222, 0, 0.955) + (0, 0, 0.081) = (0.222, 0, 1.036)
    # 夹爪 = (0.222, 0.06, 1.036) + (0, 0, 0.09) = (0.222, 0.06, 1.126)
    
    # ✓ 匹配精确 FK！
    
    # 所以简化 FK 公式（a1=0 时）:
    # 对于 axis 0 -1 0:
    # 点 (x, y, z) 绕负Y轴旋转 angle 后:
    # x' = x*cos(angle) - z*sin(angle)
    # z' = x*sin(angle) + z*cos(angle)
    
    # 但每个关节的 translation 是在旋转后的坐标系中
    # 所以我们需要累积旋转
    
    # 对于 a1=0:
    # arm1_end = (base_x, 0, 0.154)
    # 
    # arm2_joint = arm1_end + (0.033, 0, 0.07) = (base_x + 0.033, 0, 0.224)
    # arm2 旋转 a2: R2
    # arm2_end = arm2_joint + R2 * (0.033, 0, 0.07)
    # 
    # arm3_joint = arm2_end + R2 * (0, 0, 0.155)
    # arm3 旋转 a3: R3
    # arm3_end = arm3_joint + R2 * R3 * (0, 0, 0.155)
    # 
    # arm4_joint = arm3_end + R2 * R3 * (0, 0, 0.135)
    # arm4 旋转 a4: R4
    # arm4_end = arm4_joint + R2 * R3 * R4 * (0, 0, 0.135)
    # 
    # arm5_joint = arm4_end + R2 * R3 * R4 * (0, 0, 0.081)
    # arm5 旋转 a5: R5 (绕 Z 轴，不影响 XZ)
    # arm5_end = arm5_joint + R2 * R3 * R4 * R5 * (0, 0, 0.081)
    # 
    # 夹爪 = arm5_end + R2 * R3 * R4 * R5 * (0, 0.06, 0.09)
    
    # 对于 axis 0 -1 0 旋转 angle:
    # R = [cos(a)  0  -sin(a)]
    #     [0       1   0     ]
    #     [sin(a)  0  cos(a) ]
    # 
    # R * (x, y, z) = (x*cos(a) - z*sin(a), y, x*sin(a) + z*cos(a))
    
    # 对于 a1=0, a5=0:
    # R2 * R3 * R4 = R(总角度) 其中总角度 = a2 + a3 + a4
    # 因为所有 axis 都是 0 -1 0
    
    # 所以:
    # arm2_end = arm2_joint + R(a2) * (0.033, 0, 0.07)
    # arm3_joint = arm2_end + R(a2) * (0, 0, 0.155)
    # arm3_end = arm3_joint + R(a2+a3) * (0, 0, 0.155)
    # arm4_joint = arm3_end + R(a2+a3) * (0, 0, 0.135)
    # arm4_end = arm4_joint + R(a2+a3+a4) * (0, 0, 0.135)
    # arm5_joint = arm4_end + R(a2+a3+a4) * (0, 0, 0.081)
    # arm5_end = arm5_joint + R(a2+a3+a4) * (0, 0, 0.081)
    # 夹爪 = arm5_end + R(a2+a3+a4) * (0, 0.06, 0.09)
    
    # 对于 R(a) * (0, 0, L) = (-L*sin(a), 0, L*cos(a))
    # 对于 R(a) * (0.033, 0, 0.07) = (0.033*cos(a) - 0.07*sin(a), 0, 0.033*sin(a) + 0.07*cos(a))
    
    # 所以简化 FK:
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
    j3_x = arm2_end_x + 0 * ca2 - 0.155 * sa2
    j3_z = arm2_end_z + 0 * sa2 + 0.155 * ca2
    
    # arm3_end
    ca3 = math.cos(a2 + a3)
    sa3 = math.sin(a2 + a3)
    arm3_end_x = j3_x + 0 * ca3 - 0.155 * sa3
    arm3_end_z = j3_z + 0 * sa3 + 0.155 * ca3
    
    # arm4_joint
    j4_x = arm3_end_x + 0 * ca3 - 0.135 * sa3
    j4_z = arm3_end_z + 0 * sa3 + 0.135 * ca3
    
    # arm4_end
    ca4 = math.cos(a2 + a3 + a4)
    sa4 = math.sin(a2 + a3 + a4)
    arm4_end_x = j4_x + 0 * ca4 - 0.135 * sa4
    arm4_end_z = j4_z + 0 * sa4 + 0.135 * ca4
    
    # arm5_joint
    j5_x = arm4_end_x + 0 * ca4 - 0.081 * sa4
    j5_z = arm4_end_z + 0 * sa4 + 0.081 * ca4
    
    # arm5_end (a5 绕 Z 轴，不影响 XZ)
    arm5_end_x = j5_x + 0 * ca4 - 0.081 * sa4
    arm5_end_z = j5_z + 0 * sa4 + 0.081 * ca4
    
    # 夹爪
    tip_x = arm5_end_x + 0 * ca4 - 0.09 * sa4
    tip_z = arm5_end_z + 0 * sa4 + 0.09 * ca4
    
    # arm1 绕 Z 轴旋转
    cos_a1 = math.cos(a1)
    sin_a1 = math.sin(a1)
    world_x = tip_x * cos_a1
    world_y = tip_x * sin_a1
    world_z = tip_z
    
    return (world_x, world_y, world_z)


# ===== 验证 =====
print("=" * 80)
print("简化 FK vs 精确 FK")
print("=" * 80)

test_poses = [
    ("reset",       (0.0, 1.57, -2.635, 1.78, 0.0)),
    ("carry",       (0.0, 1.2, -1.5, 0.3, 0.0)),
    ("pre_grasp",   (0.0, 0.8, -1.2, 0.0, 0.0)),
    ("grasp",       (0.0, 1.0, -1.8, 0.0, 0.0)),
    ("grasp_low",   (0.0, 0.0, -2.4, -0.5, 0.0)),
    ("place_mid",   (0.0, 0.2, -0.5, 0.0, 0.0)),
    ("place_up",    (0.0, 0.0, 0.0, 0.0, 0.0)),
    ("place_approach", (0.0, 0.0, 0.0, -1.5, 0.0)),
    ("place_release",  (0.0, 0.0, 0.0, -1.67, 0.0)),
]

print(f"\n{'姿态':>16} -> {'精确前':>8} {'精确高':>8} | {'简化前':>8} {'简化高':>8} {'误差':>8}")
print("-" * 75)

for name, angles in test_poses:
    exact = fk_accurate(angles)
    simple = fk_simplified(angles)
    err = math.sqrt((exact[0]-simple[0])**2 + (exact[2]-simple[2])**2)
    print(f"{name:>16} -> {exact[0]:8.3f} {exact[2]:8.3f} | {simple[0]:8.3f} {simple[2]:8.3f} {err:8.4f}")
