"""
配置文件 - 包含所有可调参数
"""

# ==================== DeepSeek API 配置 ====================
# ⚠️ 请替换为你的 API Key
DEEPSEEK_API_KEY = "sk-4dde16c2236340cd8eb5aed6f83c0091"
DEEPSEEK_API_URL = "https://api.deepseek.com/v1/chat/completions"

# ==================== 机器人参数 ====================
# 麦克纳姆轮参数（来自 C 库 base.c）
WHEEL_RADIUS = 0.05       # 轮子半径 [m]
LX = 0.228                # 纵向距离（机器人中心到轮子）[m]
LY = 0.158                # 横向距离（机器人中心到轮子）[m]
MAX_WHEEL_SPEED = 8.0     # 最大轮子速度 [rad/s]

# 机械臂参数（来自 C 库 arm.c 的 arm_get_sub_arm_length）
ARM_LENGTHS = {
    "arm1": 0.253,  # 基座高度
    "arm2": 0.155,  # 上臂
    "arm3": 0.135,  # 前臂
    "arm4": 0.081,  # 腕部
    "arm5": 0.105   # 夹爪
}

# 夹爪参数
GRIPPER_MIN = 0.0       # 夹紧位置 [m]
GRIPPER_MAX = 0.06      # 张开位置 [m]（原为0.025，改为0.06以夹住10cm木块）
GRIPPER_SPEED = 0.05    # 夹爪速度 [m/s]

# ==================== 桌面参数 ====================
# 桌子 Solid translation: 0 0 0.325
# 桌面 Box size: 1.2 0.8 0.05 -> 表面在 0.325 + 0.05/2 = 0.350m
TABLE_HEIGHT = 0.350      # 桌面表面高度 [m]（可微调）
TABLE_CENTER = (0.0, 0.0) # 桌子中心位置 (x, y)
TABLE_SURFACE_Z = 0.350   # 桌面表面 Z 坐标 [m]（木块放置目标高度）

# 放置参数
PLACEMENT = {
    "approach_height": 0.55,      # 放置前夹爪高度 [m]（高于桌面，仅用于参考）
    "drop_step": 0.01,            # 每次下降步长 [m]（仅用于参考）
    "drop_delay_steps": 15,       # 每步下降后的等待步数（仅用于参考）
    "release_height_offset": 0.005, # 释放前再下降一点确保接触 [m]（仅用于参考）
    "retract_height": 0.50,       # 释放后抬升高度 [m]（仅用于参考）
    "max_force_during_place": 2.0, # 放置时夹爪最大力 [N]
    # 以下为实际使用的参数（基于正运动学精确计算）
    "arm4_approach": 1.3,         # 接近时 arm4 角度 [rad] → 绝对高=0.417m
    "arm4_release": 1.4,          # 释放时 arm4 角度 [rad] → 绝对高=0.400m
    "arm4_step": 0.01,            # arm4 下降步长 [rad]
    "arm4_step_delay": 10,        # 每步等待步数
}

# ==================== 导航参数 ====================

# 注意: 机械臂基座在机器人前方 0.156m 处
# 机械臂总伸展长度约 0.46m（arm2+arm3+arm4+arm5）
# 地面抓取时，机械臂几乎垂直向下，水平伸展仅约 0.02m
# 因此底盘必须非常靠近木块（距离 < 0.05m）
NAVIGATION = {
    "approach_distance": 0.30,      # 接近木块的目标距离 [m]（从远处导航到此距离）
    "grasp_distance": 0.05,         # 抓取距离 [m]（底盘需要靠近到木块前方 0.05m）
    "table_approach_distance": 0.66, # 接近桌子的距离 [m]（安全点距离桌子中心0.6m，到达时距离<0.65m即认为已到达）
    "position_tolerance": 0.03,     # 位置容差 [m]
    "angle_tolerance": 0.05,        # 角度容差 [rad]
    "obstacle_distance": 0.3,       # 障碍物检测距离 [m]
    "max_navigation_time": 60.0,    # 最大导航时间 [s]
}

# ==================== 视觉参数 ====================
# 颜色名称映射（中英文）
COLOR_NAMES = {
    "red": "红色",
    "blue": "蓝色",
    "green": "绿色",
    "yellow": "黄色"
}

# ==================== 有限状态机参数 ====================
FSM_STATES = [
    "INIT",
    "WAIT_ORDER",
    "NAVIGATE_TO_BLOCK",
    "APPROACH_BLOCK",
    "GRASP",
    "LIFT",
    "NAVIGATE_TO_TABLE",
    "PLACE_ON_TABLE",
    "RELEASE",
    "BACK_OFF",
    "COMPLETED",
    "ERROR"
]

# ==================== 仿真参数 ====================
TIME_STEP = 16  # 仿真步长 [ms]
