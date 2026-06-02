"""
配置文件 - 包含所有可调参数
"""

# ==================== DeepSeek API 配置 ====================
DEEPSEEK_API_KEY = "YOUR_DEEPSEEK_API_KEY_HERE"  # ⚠️ 请替换为你的 API Key
DEEPSEEK_API_URL = "https://api.deepseek.com/v1/chat/completions"

# ==================== 机器人参数 ====================
# 麦克纳姆轮参数（来自 C 库 base.c）
WHEEL_RADIUS = 0.05       # 轮子半径 [m]
LX = 0.228                # 纵向距离（机器人中心到轮子）[m]
LY = 0.158                # 横向距离（机器人中心到轮子）[m]
MAX_WHEEL_SPEED = 8.0     # 最大轮子速度 [rad/s]（C 库中 SPEED=4.0）

# 机械臂参数（来自 C 库 arm.c 的 arm_get_sub_arm_length）
ARM_LENGTHS = {
    "arm1": 0.253,  # 基座高度
    "arm2": 0.155,  # 上臂
    "arm3": 0.135,  # 前臂
    "arm4": 0.081,  # 腕部
    "arm5": 0.105   # 夹爪
}

# 夹爪参数（来自 C 库 gripper.c）
GRIPPER_MIN = 0.0       # 夹紧位置
GRIPPER_MAX = 0.025     # 张开位置
GRIPPER_SPEED = 0.03    # 夹爪速度

# ==================== 导航参数 ====================
NAVIGATION = {
    "approach_distance": 0.35,      # 接近木块的目标距离 [m]
    "grasp_distance": 0.35,         # 抓取距离 [m]（与 approach_distance 一致）
    "table_approach_distance": 0.4, # 接近桌子的距离 [m]
    "position_tolerance": 0.05,     # 位置容差 [m]
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
