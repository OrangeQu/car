"""
多传感器融合感知模块
由于 YouBot 默认没有摄像头和激光雷达，此模块提供基于已知位置信息的感知
"""

from controller import Robot
import math


class Perception:
    """感知模块 - 提供目标检测和定位功能"""
    
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # 尝试获取摄像头（YouBot 默认没有）
        self.camera = None
        camera_names = ["CameraTop", "camera", "Camera", "CameraBottom"]
        for name in camera_names:
            cam = robot.getDevice(name)
            if cam:
                self.camera = cam
                self.camera.enable(timestep)
                print(f"  ✓ 摄像头 {name} 已启用")
                break
        
        if not self.camera:
            print("  ⚠️ 未找到摄像头设备")
        
        # 尝试获取激光雷达（YouBot 默认没有）
        self.lidar = None
        lidar_names = ["LDS-01", "lidar", "Lidar", "Hokuyo", "Sick"]
        for name in lidar_names:
            lid = robot.getDevice(name)
            if lid:
                self.lidar = lid
                self.lidar.enable(timestep)
                print(f"  ✓ 激光雷达 {name} 已启用")
                break
        
        if not self.lidar:
            print("  ⚠️ 未找到激光雷达设备")
        
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
    
    def get_lidar_data(self):
        """获取激光雷达数据（如果有）"""
        if self.lidar:
            return self.lidar.getRangeImage()
        return None
    
    def detect_blocks_by_color(self):
        """
        通过颜色检测木块（需要摄像头）
        返回: [(color_name, x, y), ...]
        """
        if not self.camera:
            return []
        
        # 这里可以实现 OpenCV 颜色阈值分割
        # 但由于 YouBot 默认没有摄像头，暂时返回空列表
        return []
    
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
        """
        检查木块是否在抓取范围内
        """
        _, _, distance = self.get_block_relative_position(block_x, block_y)
        return distance <= grasp_distance
