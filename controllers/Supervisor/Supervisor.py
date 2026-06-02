"""
Supervisor 控制器 - 基于YouBot的多目标物自主搜集与定点放置
功能：
1. 启动时随机生成四角木块的坐标
2. 调用 DeepSeek API 规划最优抓取顺序
3. 通过 Emitter 将木块位置和抓取顺序发送给 YouBot
4. 监控任务进度
"""

from controller import Supervisor
import random
import json
import requests
import math

# ==================== 配置区域 ====================
# DeepSeek API 配置（请替换为你的 API Key）
DEEPSEEK_API_KEY = "sk-37a5c467e6104e30a376292db0aa87ca"  # ⚠️ 请替换为你的 API Key
DEEPSEEK_API_URL = "https://api.deepseek.com/v1/chat/completions"

# 场地参数
ARENA_SIZE = 10.0  # 10m x 10m
TABLE_POSITION = (0.0, 0.0)  # 桌子在场地中央
BLOCK_SIZE = 0.1  # 木块尺寸 0.1m

# 四角基准位置（每个角区域的范围）
CORNERS = {
    "red": {"base": (-4.0, 4.0), "range": 1.5},    # 左上
    "blue": {"base": (4.0, 4.0), "range": 1.5},    # 右上
    "green": {"base": (4.0, -4.0), "range": 1.5},  # 右下
    "yellow": {"base": (-4.0, -4.0), "range": 1.5} # 左下
}

# 颜色名称映射
COLOR_NAMES = {
    "red": "红色",
    "blue": "蓝色",
    "green": "绿色",
    "yellow": "黄色"
}


class BlockManager:
    """木块管理器：负责随机生成坐标和位置管理"""
    
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.blocks = {}  # {color: {node, position, original_position}}
        self.block_positions = {}  # {color: (x, y)}
        
    def get_block_node(self, color):
        """获取木块的 Solid 节点"""
        node = self.supervisor.getFromDef(f"{color}_block")
        if node is None:
            # 尝试通过名称获取
            children = self.supervisor.getRoot().getField("children")
            for i in range(children.getCount()):
                child = children.getMFNode(i)
                if child.getTypeName() == "Solid" and child.getField("name").getSFString() == f"{color}_block":
                    node = child
                    break
        return node
    
    def randomize_positions(self):
        """随机生成四个木块的位置"""
        print("=" * 60)
        print("🔄 Supervisor: 随机生成木块位置...")
        print("=" * 60)
        
        positions = {}
        for color, corner in CORNERS.items():
            bx, by = corner["base"]
            r = corner["range"]
            
            # 在基准位置附近随机偏移
            x = bx + random.uniform(-r, r)
            y = by + random.uniform(-r, r)
            
            # 确保木块在场地内（留0.5m边距）
            x = max(-4.5, min(4.5, x))
            y = max(-4.5, min(4.5, y))
            
            positions[color] = (x, y)
            print(f"  {COLOR_NAMES[color]}木块: ({x:.2f}, {y:.2f})")
        
        self.block_positions = positions
        return positions
    
    def apply_positions(self, positions):
        """将随机位置应用到场景中的木块"""
        for color, (x, y) in positions.items():
            node = self.get_block_node(color)
            if node:
                # 获取 translation 字段并设置新位置
                trans_field = node.getField("translation")
                trans_field.setSFVec3f([x, y, BLOCK_SIZE / 2])
                print(f"  ✓ {COLOR_NAMES[color]}木块已移动到 ({x:.2f}, {y:.2f})")
            else:
                print(f"  ⚠️ 未找到 {COLOR_NAMES[color]}木块节点")
    
    def get_positions_dict(self):
        """获取位置字典供 LLM 使用"""
        result = {}
        for color, (x, y) in self.block_positions.items():
            result[color] = {
                "name_cn": COLOR_NAMES[color],
                "position": [x, y],
                "distance_to_center": math.sqrt(x**2 + y**2)
            }
        return result


class LLMPlanner:
    """LLM 路径规划器：调用 DeepSeek API 规划最优抓取顺序"""
    
    def __init__(self, api_key, api_url):
        self.api_key = api_key
        self.api_url = api_url
    
    def plan_order(self, block_positions):
        """调用 DeepSeek API 规划抓取顺序"""
        print("\n🧠 调用 DeepSeek API 规划抓取顺序...")
        
        # 构建位置描述
        positions_desc = ""
        for color, info in block_positions.items():
            pos = info["position"]
            positions_desc += f"- {info['name_cn']}({color}): ({pos[0]:.2f}, {pos[1]:.2f}), 距中心{info['distance_to_center']:.2f}m\n"
        
        prompt = f"""你是一个机器人路径规划专家。场地为10m×10m，中央桌子在(0,0)处。
机器人从(0,0)出发，需要依次抓取四角的彩色木块并放回中央桌子。

四角木块位置：
{positions_desc}

请规划最短路径的抓取顺序（考虑从当前位置到各木块的距离，以及木块之间的路径优化）。

输出严格的JSON格式（不要包含其他文字）：
{{
    "order": ["颜色1", "颜色2", "颜色3", "颜色4"],
    "reason": "简要说明规划理由"
}}

颜色可选: red, blue, green, yellow
"""
        
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        
        data = {
            "model": "deepseek-chat",
            "messages": [
                {"role": "system", "content": "你是一个机器人路径规划专家，只输出JSON格式。"},
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.1,
            "max_tokens": 500
        }
        
        try:
            response = requests.post(self.api_url, headers=headers, json=data, timeout=30)
            response.raise_for_status()
            result = response.json()
            
            if "choices" in result and len(result["choices"]) > 0:
                content = result["choices"][0]["message"]["content"]
                print(f"  LLM 响应: {content}")
                
                # 解析 JSON
                # 处理可能包含的 markdown 代码块
                if "```json" in content:
                    content = content.split("```json")[1].split("```")[0].strip()
                elif "```" in content:
                    content = content.split("```")[1].split("```")[0].strip()
                
                plan = json.loads(content)
                return plan
        except Exception as e:
            print(f"  ⚠️ LLM 调用失败: {e}")
        
        # 如果 LLM 调用失败，使用默认顺序（按距离排序）
        print("  使用默认顺序（按距离从近到远）")
        return self._default_order(block_positions)
    
    def _default_order(self, block_positions):
        """默认顺序：按距离从近到远"""
        sorted_blocks = sorted(block_positions.items(), key=lambda x: x[1]["distance_to_center"])
        order = [b[0] for b in sorted_blocks]
        return {
            "order": order,
            "reason": "默认顺序：按距离从近到远（LLM调用失败）"
        }


class TaskMonitor:
    """任务监控器：监控任务进度"""
    
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.task_start_time = None
        self.collected_blocks = []
        self.current_block_index = 0
        
    def start_task(self):
        """开始任务计时"""
        self.task_start_time = self.supervisor.getTime()
        print(f"\n⏱️ 任务开始于 t={self.task_start_time:.2f}s")
    
    def mark_block_collected(self, color):
        """标记一个木块已收集"""
        self.collected_blocks.append(color)
        self.current_block_index += 1
        elapsed = self.supervisor.getTime() - self.task_start_time
        print(f"  ✅ 已收集 {COLOR_NAMES.get(color, color)} ({self.current_block_index}/4), 耗时 {elapsed:.1f}s")
    
    def is_task_complete(self):
        """检查任务是否完成"""
        return len(self.collected_blocks) >= 4
    
    def get_status_text(self):
        """获取状态文本用于显示"""
        elapsed = self.supervisor.getTime() - self.task_start_time if self.task_start_time else 0
        collected = ", ".join([COLOR_NAMES.get(c, c) for c in self.collected_blocks])
        return f"进度: {self.current_block_index}/4 | 已收集: {collected} | 耗时: {elapsed:.1f}s"


class SupervisorController:
    """主 Supervisor 控制器"""
    
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        # 初始化各模块
        self.block_manager = BlockManager(self.supervisor)
        self.llm_planner = LLMPlanner(DEEPSEEK_API_KEY, DEEPSEEK_API_URL)
        self.task_monitor = TaskMonitor(self.supervisor)
        
        # 任务数据
        self.grasp_order = []  # 抓取顺序
        self.block_positions = {}  # 木块位置
        self.task_started = False
        self.data_sent = False
        
        # 创建 Emitter 用于发送数据给 YouBot
        self.emitter = self.supervisor.getDevice("emitter")
        if self.emitter is None:
            print("⚠️ 未找到 Emitter 设备，将使用标签显示数据")
        
        print("=" * 60)
        print("Supervisor 控制器已启动")
        print("=" * 60)
    
    def setup_scene(self):
        """初始化场景：随机生成木块位置并规划路径"""
        # 1. 随机生成木块位置
        positions = self.block_manager.randomize_positions()
        self.block_manager.apply_positions(positions)
        self.block_positions = self.block_manager.get_positions_dict()
        
        # 2. 调用 LLM 规划抓取顺序
        plan = self.llm_planner.plan_order(self.block_positions)
        self.grasp_order = plan["order"]
        
        print(f"\n📋 抓取顺序: {', '.join([COLOR_NAMES.get(c, c) for c in self.grasp_order])}")
        print(f"💡 规划理由: {plan.get('reason', 'N/A')}")
        
        # 3. 显示在场景中
        self._display_info()
        
        self.task_started = True
        self.task_monitor.start_task()
    
    def _display_info(self):
        """在场景中显示信息"""
        # 显示抓取顺序
        order_text = " → ".join([COLOR_NAMES.get(c, c) for c in self.grasp_order])
        self.supervisor.setLabel(0, f"📋 抓取顺序: {order_text}", 0.01, 0.01, 0.08, 0x00FF00, 0.0, "Arial")
        
        # 显示木块位置
        pos_text = ""
        for color, info in self.block_positions.items():
            p = info["position"]
            pos_text += f"{info['name_cn']}: ({p[0]:.1f}, {p[1]:.1f})  "
        self.supervisor.setLabel(1, f"📍 {pos_text}", 0.01, 0.06, 0.06, 0xFFFFFF, 0.0, "Arial")
    
    def send_data_to_robot(self):
        """通过 Emitter 发送数据给 YouBot"""
        if self.emitter is None:
            return
        
        data = {
            "type": "mission_data",
            "grasp_order": self.grasp_order,
            "block_positions": {color: info["position"] for color, info in self.block_positions.items()},
            "table_position": [0.0, 0.0]
        }
        
        message = json.dumps(data).encode('utf-8')
        self.emitter.send(message)
        self.data_sent = True
        print("\n📡 已发送任务数据给 YouBot")
    
    def update_display(self):
        """更新显示信息"""
        if self.task_started:
            status = self.task_monitor.get_status_text()
            self.supervisor.setLabel(2, f"⏱️ {status}", 0.01, 0.11, 0.06, 0xFFFF00, 0.0, "Arial")
    
    def run(self):
        """主循环"""
        # 第一步：设置场景
        self.setup_scene()
        
        # 等待几秒让场景稳定
        for _ in range(100):
            self.supervisor.step(self.timestep)
        
        # 发送数据给 YouBot
        self.send_data_to_robot()
        
        print("\n" + "=" * 60)
        print("✅ Supervisor 初始化完成，等待 YouBot 执行任务...")
        print("=" * 60)
        
        # 主循环
        while self.supervisor.step(self.timestep) != -1:
            self.update_display()
            
            # 检查任务是否完成（可以通过检测木块是否在桌子上来判断）
            # 这里简化处理：由 YouBot 通过 Emitter 报告进度
            # 或者检测木块位置变化
            
            # 每步都重新发送数据（确保 YouBot 能收到）
            if not self.data_sent:
                self.send_data_to_robot()


if __name__ == "__main__":
    controller = SupervisorController()
    controller.run()
