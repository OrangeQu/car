"""
多传感器融合感知模块
使用 CameraRecognition + Lidar 融合定位彩色木块
"""

from controller import Robot
import math
import ctypes
from config import LIDAR_CONFIG


class LidarProcessor:
    """激光雷达处理模块"""

    def __init__(self, robot, timestep):
        self.lidar = robot.getDevice("LidarTop")
        self.enabled = False
        if self.lidar:
            self.lidar.enable(timestep)
            self.lidar.enablePointCloud()
            self.enabled = True
            print("  ✓ Lidar 已启用")
        else:
            print("  ⚠️ 未找到 LidarTop 设备")

    def get_range_image(self):
        """获取距离图像"""
        if not self.enabled or not self.lidar:
            return None
        return self.lidar.getRangeImage()

    def get_block_position_from_lidar(self, expected_angle=0.0):
        """
        用 Lidar 在指定方向附近扫描，检测木块的精确位置

        参数:
            expected_angle: 期望木块所在的角度 [rad]（相对于 Lidar 正前方）

        返回:
            (distance, angle) 或 None
        """
        range_image = self.get_range_image()
        if range_image is None:
            return None

        # Lidar 270° 视野，360 个点，角度分辨率 = 270/360 = 0.75°
        fov = 3.14159  # 270° in rad
        angle_per_point = fov / 360

        center_index = 180  # 正前方对应的索引
        expected_index = center_index + int(expected_angle / angle_per_point)

        # 在期望角度附近 ±15° 搜索
        search_range = int(15 * 3.14159 / 180 / angle_per_point)

        best_dist = float('inf')
        best_idx = -1

        for i in range(max(0, expected_index - search_range),
                       min(360, expected_index + search_range)):
            dist = range_image[i]
            # 过滤无效值和地面反射
            if 0.1 < dist < 5.0:
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

        if best_idx >= 0:
            angle = (best_idx - center_index) * angle_per_point
            return (best_dist, angle)

        return None

    def get_min_distance_in_angle_range(self, angle_range_deg=30):
        """
        获取前方指定角度范围内的最近障碍物距离（用于防碰撞）

        参数:
            angle_range_deg: 角度范围 [度]（前方 ±angle_range_deg）

        返回:
            最小距离 [m]，如果没有障碍物返回 inf
        """
        range_image = self.get_range_image()
        if range_image is None:
            return float('inf')

        fov = 3.14159
        angle_per_point = fov / 360
        center = 180
        half_range = int(angle_range_deg * 3.14159 / 180 / angle_per_point)

        min_dist = float('inf')
        for i in range(center - half_range, center + half_range + 1):
            if 0 <= i < 360:
                dist = range_image[i]
                if 0.1 < dist < 5.0:
                    min_dist = min(min_dist, dist)

        return min_dist

    def get_min_distance_in_full_range(self):
        """
        获取全方向最近障碍物距离

        返回:
            最小距离 [m]，如果没有障碍物返回 inf
        """
        range_image = self.get_range_image()
        if range_image is None:
            return float('inf')

        min_dist = float('inf')
        for i in range(360):
            dist = range_image[i]
            if 0.1 < dist < 5.0:
                min_dist = min(min_dist, dist)

        return min_dist

    def detect_block_cluster(self):
        """
        用 Lidar 检测木块（0.1m 立方体）的轮廓

        原理：木块在地面上会形成一个 0.1m 宽的凸起
        通过检测距离突变来识别木块边缘

        返回: [(distance, angle), ...] 木块轮廓点列表
        """
        range_image = self.get_range_image()
        if range_image is None:
            return []

        # 检测距离突变点（木块边缘）
        edges = []
        for i in range(1, 360):
            if 0.1 < range_image[i] < 5.0 and 0.1 < range_image[i-1] < 5.0:
                diff = abs(range_image[i] - range_image[i-1])
                if diff > 0.05:  # 5cm 突变 → 可能是木块边缘
                    edges.append(i)

        return edges

    def scan_block_profile(self):
        """
        扫描木块轮廓，获取木块中心位置

        返回: (distance, angle) 木块中心位置，或 None
        """
        range_image = self.get_range_image()
        if range_image is None:
            return None

        fov = 3.14159
        angle_per_point = fov / 360
        center = 180

        # 找前方 0.1~0.5m 范围内的连续凸起
        # 木块 0.1m 立方体，在 0.5m 处约占 0.1/0.5 ≈ 0.2rad ≈ 11.5°
        # 对应约 15 个 Lidar 点
        block_points = []
        in_block = False
        block_start = 0

        for i in range(360):
            dist = range_image[i]
            if 0.05 < dist < 0.5:  # 5cm~50cm 范围内
                if not in_block:
                    in_block = True
                    block_start = i
                block_points.append((i, dist))
            else:
                if in_block:
                    # 检查这个连续段的长度是否匹配木块
                    if len(block_points) >= 5:  # 至少 5 个点
                        # 计算平均距离和中心角度
                        avg_dist = sum(p[1] for p in block_points) / len(block_points)
                        center_idx = (block_points[0][0] + block_points[-1][0]) // 2
                        angle = (center_idx - center) * angle_per_point
                        return (avg_dist, angle)
                    in_block = False
                    block_points = []

        return None


class Perception:
    """感知模块 - 使用 CameraRecognition + Lidar 融合定位"""

    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep

        # 尝试获取摄像头（通过 bodySlot 添加的 CameraTop）
        self.camera = None
        camera_names = ["CameraTop", "camera", "Camera", "CameraBottom"]
        for name in camera_names:
            cam = robot.getDevice(name)
            if cam:
                self.camera = cam
                self.camera.enable(timestep)
                # 启用物体识别
                self.camera.recognitionEnable(timestep)
                print(f"  ✓ 摄像头 {name} 已启用（带物体识别）")
                break

        if not self.camera:
            print("  ⚠️ 未找到摄像头设备")

        # 初始化 Lidar
        self.lidar = LidarProcessor(robot, timestep)

        # 机器人位姿回调
        self.get_position = None
        self.get_orientation = None

        print("  ✓ 感知模块初始化完成（Camera + Lidar）")

    def set_robot_pose_callback(self, get_position_func, get_orientation_func):
        """设置获取机器人位姿的回调函数"""
        self.get_position = get_position_func
        self.get_orientation = get_orientation_func

    def get_camera_image(self):
        """获取摄像头图像（如果有）"""
        if self.camera:
            return self.camera.getImage()
        return None

    def _ctypes_colors_to_list(self, obj):
        """安全转换 CameraRecognitionObject 的 colors 指针"""
        try:
            colors_ptr = obj.getColors()
            if not colors_ptr:
                return []

            # 获取颜色数组长度
            colors_count = obj.getColorsSize() if hasattr(obj, 'getColorsSize') else 3

            if colors_count <= 0:
                return []

            # ctypes 核心转换: 指针 → 定长数组 → Python list
            ColorArray = ctypes.c_double * colors_count
            colors_array = ctypes.cast(colors_ptr, ctypes.POINTER(ColorArray)).contents
            return [float(c) for c in colors_array]
        except Exception as e:
            print(f"  ⚠️ colors 转换失败: {e}")
            return []

    def _recognition_object_to_dict(self, obj):
        """将 CameraRecognitionObject 转换为安全字典"""
        # 位置 (3D 坐标)
        pos = obj.getPosition() if hasattr(obj, 'getPosition') else [0, 0, 0]
        pos = [float(p) if p is not None else 0.0 for p in (pos or [0, 0, 0])[:3]]

        # 方向 (四元数)
        orient = obj.getOrientation() if hasattr(obj, 'getOrientation') else [0, 0, 0, 1]
        orient = [float(o) if o is not None else 0.0 for o in (orient or [0, 0, 0, 1])[:4]]

        # 尺寸
        size = obj.getSize() if hasattr(obj, 'getSize') else [0, 0]
        size = [float(s) if s is not None else 0.0 for s in (size or [0, 0])[:2]]

        # 图像坐标
        pos_img = obj.getPositionOnImage() if hasattr(obj, 'getPositionOnImage') else [0, 0]
        pos_img = [int(p) for p in (pos_img or [0, 0])[:2]]

        # 颜色
        colors = self._ctypes_colors_to_list(obj)

        # 元数据
        name = obj.getModel() if hasattr(obj, 'getModel') else 'unknown'
        name = name if name else 'unknown'

        # 计算距离
        distance = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)

        return {
            'name': name,
            'position': pos,
            'orientation': orient,
            'size': size,
            'position_on_image': pos_img,
            'colors': colors,
            'distance': distance
        }

    def get_visual_objects(self):
        """获取识别物体列表"""
        if not self.camera or not self.camera.hasRecognition():
            return []

        try:
            objects = self.camera.getRecognitionObjects()
        except Exception as e:
            print(f"  ⚠️ 获取识别对象失败: {e}")
            return []

        obj_list = []
        for obj in objects:
            try:
                obj_dict = self._recognition_object_to_dict(obj)
                obj_list.append(obj_dict)
            except Exception as e:
                print(f"  ⚠️ 跳过单个对象转换错误: {e}")
                continue

        return obj_list

    def detect_blocks_by_color(self):
        """
        通过 CameraRecognition 检测彩色木块
        返回: [(color_name, x, y, z), ...]
        """
        if not self.camera:
            return []

        objects = self.get_visual_objects()
        blocks = []

        for obj in objects:
            colors = obj.get('colors', [])
            pos = obj.get('position', [0, 0, 0])

            # 根据颜色判断木块类型
            if len(colors) >= 3:
                r, g, b = colors[0], colors[1], colors[2]
                color_name = self._classify_color(r, g, b)
                if color_name:
                    blocks.append((color_name, pos[0], pos[1], pos[2]))

        return blocks

    def _classify_color(self, r, g, b):
        """根据 RGB 值分类颜色"""
        # 红色: (1, 0, 0)
        if r > 0.7 and g < 0.3 and b < 0.3:
            return "red"
        # 蓝色: (0, 0, 1)
        if r < 0.3 and g < 0.3 and b > 0.7:
            return "blue"
        # 绿色: (0, 1, 0)
        if r < 0.3 and g > 0.7 and b < 0.3:
            return "green"
        # 黄色: (1, 1, 0)
        if r > 0.7 and g > 0.7 and b < 0.3:
            return "yellow"
        return None

    def get_block_relative_position(self, block_x, block_y):
        """
        计算木块相对于机器人的位置
        block_x, block_y: 木块在世界坐标系下的位置
        返回: (rel_x, rel_y, distance)
        """
        if not self.get_position or not self.get_orientation:
            return (0.0, 0.0, 0.0)

        robot_pos = self.get_position()
        robot_angle = self.get_orientation()

        dx = block_x - robot_pos[0]
        dy = block_y - robot_pos[1]

        # 转换到机器人坐标系
        cos_a = math.cos(-robot_angle)
        sin_a = math.sin(-robot_angle)
        rel_x = dx * cos_a - dy * sin_a
        rel_y = dx * sin_a + dy * cos_a

        distance = math.sqrt(dx**2 + dy**2)

        return (rel_x, rel_y, distance)

    def is_block_in_grasp_range(self, block_x, block_y, grasp_distance=0.25):
        """检查木块是否在抓取范围内"""
        _, _, distance = self.get_block_relative_position(block_x, block_y)
        return distance <= grasp_distance

    def locate_block_fusion(self, target_color):
        """
        Camera + Lidar 融合定位木块

        流程:
        1. Camera 识别颜色 → 得到木块在图像中的像素位置
        2. 像素位置 → 角度（Camera 视野 1.064rad，160px）
        3. Lidar 在该角度方向扫描 → 得到精确距离
        4. 融合计算 → 木块在机器人坐标系下的精确坐标

        返回: (forward, left, height) 或 None
        """
        # 1. Camera 识别
        objects = self.get_visual_objects()
        target_obj = None
        for obj in objects:
            colors = obj.get('colors', [])
            if len(colors) >= 3:
                color_name = self._classify_color(colors[0], colors[1], colors[2])
                if color_name == target_color:
                    target_obj = obj
                    break

        if not target_obj:
            # Camera 没识别到，尝试用 Lidar 扫描
            lidar_result = self.lidar.scan_block_profile()
            if lidar_result:
                distance, angle = lidar_result
                forward = distance * math.cos(angle)
                left = distance * math.sin(angle)
                return (forward, left, 0.05)
            return None

        # 2. 从 Camera 获取物体在图像中的位置
        img_x, img_y = target_obj['position_on_image']
        img_width = 160  # Camera 宽度

        # 3. 像素坐标 → 角度
        # Camera 视野 1.064 rad，宽度 160px
        # 中心像素 = 80，角度 = (像素 - 中心) * 视野/宽度
        camera_fov = 1.064
        angle_per_pixel = camera_fov / img_width
        angle_from_center = (img_x - img_width/2) * angle_per_pixel

        # 4. 用 Lidar 获取该方向上的精确距离
        lidar_result = self.lidar.get_block_position_from_lidar(angle_from_center)

        if lidar_result:
            distance, lidar_angle = lidar_result
        else:
            # Lidar 没检测到，用 Camera 的估算距离
            distance = target_obj.get('distance', 0.5)

        # 5. 计算木块在机器人坐标系下的位置
        forward = distance * math.cos(angle_from_center)
        left = distance * math.sin(angle_from_center)
        height = 0.05  # 木块高度（地面）

        return (forward, left, height)
