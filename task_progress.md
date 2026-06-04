# 任务进度

## 已完成

- [x] 分析问题根因：车头碰撞木块导致偏移 → 夹爪只抓住前端 → 放置时倾倒
- [x] 1. perception.py: 新增 `locate_block_exact()` 和 `locate_block_fusion_enhanced()` 方法
- [x] 2. fsm.py: 修改 `_state_APPROACH_BLOCK` - 加入 Lidar 扫描 + 底盘微调
- [x] 3. arm_controller.py: 修改 `place_on_table()` - 分步释放 + 竖直向上姿态
- [x] 4. config.py: 调整导航参数（grasp_distance=0.12m, 新增 Lidar 参数）
- [x] 5. fsm.py: precision 阶段使用配置参数中的 Lidar 停车距离
- [x] 6. 分析新问题：夹爪只抓到木块后端（靠近小车一侧）的原因
- [x] 7. 修改 grasp_block() 方法：新增阶段2.5 - 用 IK 移动到木块中心正上方再下探
- [x] 8. **修复 IK 算法**：用基于精确 FK 的数值 IK 替换了错误的 C 库移植 IK

## 关键发现

### 1. 原始 IK 算法错误
- 原始 `arm_controller.py` 中的 `ik()` 函数从 C 库 `arm_ik()` 移植
- 该算法假设 arm2 的 axis 是 `0 1 0`（正Y轴），但 Webots 中实际是 `0 -1 0`（负Y轴）
- 导致 FK 和 IK 的计算结果与实际物理位置完全相反

### 2. 精确 FK 推导
- 使用 Rodrigues 旋转矩阵，从 box.wbt 的关节链参数精确计算
- 验证了所有预设姿态的实际位置：
  - `place_up` (a2=0, a3=0, a4=0): 前=0.222m, 高=1.126m
  - `grasp_low` (a2=0, a3=-2.4, a4=-0.5): 前=0.510m, 高=-0.141m
  - 当 a2 从 1.57 减小到 0 再到负值，上臂向前摆

### 3. 数值 IK 求解器
- 基于精确 FK，使用梯度下降法 + 伪逆
- 支持多个初始猜测，自动选择最优解
- 桌面放置 (0.380, 0, 0.750): 误差 0.0006m
- 木块表面 (0.120, 0, 0.050): 误差 0.0010m
- IK 自洽性: 误差 < 0.001m
