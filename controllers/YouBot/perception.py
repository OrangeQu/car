"""
多传感器融合感知模块
使用 CameraRecognition 识别彩色木块
"""

from controller import Robot
import math
import ctypes


class Perception:
    """感知模块 - 使用 CameraRecognition 检测木块"""

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

        # 机器人位姿回调
        self.get_position = None
        self.get_orientation = None

        print("  ✓ 感知模块初始化完成")

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
