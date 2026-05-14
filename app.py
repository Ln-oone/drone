import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
import pandas as pd
from dataclasses import dataclass, field
import numpy as np


# ==================== 配置常量 ====================
@dataclass
class Config:
    """系统配置类"""
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.748807, 32.233931])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.750046, 32.236150])
    
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 5
    MAX_BACKUP_FILES: int = 10
    
    BASE_SPEED_MPS: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    VOLTAGE_VARIATION: float = 0.5
    SAT_RANGE: Tuple[int, int] = (8, 14)
    
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR_URL: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
    
    # 避障参数
    SAFETY_BUFFER_METERS: float = 15.0  # 安全缓冲区（米）
    NUM_WAYPOINTS: int = 5  # 中间绕行点数量（至少5个）
    MAX_ITERATIONS: int = 10  # 最大迭代次数


config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)


# ==================== 几何函数 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    """判断点是否在多边形内"""
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / (y2 - y1) + x1
        ):
            inside = not inside
    return inside


def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
    """判断点q是否在线段pr上"""
    return (
        min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
        min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
    )


def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    """计算三点方向"""
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-10:
        return 0
    return 1 if val > 0 else 2


def segments_intersect(
    p1: List[float], p2: List[float], 
    p3: List[float], p4: List[float]
) -> bool:
    """判断两线段是否相交"""
    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)
    
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and on_segment(p1, p3, p2):
        return True
    if o2 == 0 and on_segment(p1, p4, p2):
        return True
    if o3 == 0 and on_segment(p3, p1, p4):
        return True
    if o4 == 0 and on_segment(p3, p2, p4):
        return True
    return False


def line_intersects_polygon(
    p1: List[float], p2: List[float], polygon: List[List[float]]
) -> bool:
    """判断线段是否与多边形相交"""
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False


def distance(p1: List[float], p2: List[float]) -> float:
    """计算两点间距离（度）"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def get_polygon_bounds(polygon: List[List[float]]) -> Optional[Dict]:
    """获取多边形边界"""
    if not polygon:
        return None
    min_lng = min(p[0] for p in polygon)
    max_lng = max(p[0] for p in polygon)
    min_lat = min(p[1] for p in polygon)
    max_lat = max(p[1] for p in polygon)
    return {
        'min_lng': min_lng, 'max_lng': max_lng,
        'min_lat': min_lat, 'max_lat': max_lat,
        'center_lng': (min_lng + max_lng) / 2,
        'center_lat': (min_lat + max_lat) / 2
    }


def validate_polygon(polygon: List[List[float]]) -> bool:
    """验证多边形有效性"""
    return len(polygon) >= 3


def meters_to_deg(meters: float, lat: float = 32.23) -> Tuple[float, float]:
    """米转经纬度度数"""
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg


def point_to_segment_distance_meters(
    point: List[float], seg_start: List[float], seg_end: List[float]
) -> float:
    """点到线段距离（米）"""
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    
    if len_sq == 0:
        dist_deg = math.sqrt((px - x1)**2 + (py - y1)**2)
        return dist_deg * 111000
    
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0, min(1, t))
    
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    
    dist_deg = math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
    return dist_deg * 111000


def check_safety_radius(
    drone_pos: List[float], 
    obstacles_gcj: List[Dict], 
    flight_altitude: float, 
    safety_radius: float
) -> Tuple[bool, Optional[float], Optional[str]]:
    """检查无人机是否进入安全半径"""
    if not drone_pos:
        return True, None, None
    
    min_distance = float('inf')
    danger_name = None
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        obs_height = obs.get('height', 30)
        
        if obs_height <= flight_altitude:
            continue
        
        if coords and len(coords) >= 3:
            for i in range(len(coords)):
                p1 = coords[i]
                p2 = coords[(i + 1) % len(coords)]
                dist_m = point_to_segment_distance_meters(drone_pos, p1, p2)
                
                if dist_m < min_distance:
                    min_distance = dist_m
                    danger_name = obs.get('name', '障碍物')
    
    if min_distance < safety_radius:
        return False, min_distance, danger_name
    return True, min_distance if min_distance != float('inf') else None, None


# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    """清理旧备份文件"""
    try:
        backup_files = [
            f for f in os.listdir(config.BACKUP_DIR) 
            if f.startswith(config.CONFIG_FILE)
        ]
        if len(backup_files) > config.MAX_BACKUP_FILES:
            backup_files.sort()
            for old_file in backup_files[:-config.MAX_BACKUP_FILES]:
                os.remove(os.path.join(config.BACKUP_DIR, old_file))
    except Exception as e:
        st.warning(f"清理备份文件时出错: {e}")


def backup_config() -> Optional[str]:
    """备份配置文件"""
    if os.path.exists(config.CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{timestamp}.bak"
        try:
            import shutil
            shutil.copy(config.CONFIG_FILE, backup_name)
            cleanup_old_backups()
            return backup_name
        except Exception as e:
            st.error(f"备份失败: {e}")
    return None


def load_obstacles() -> List[Dict]:
    """加载障碍物配置"""
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles = data.get('obstacles', [])
                for obs in obstacles:
                    if 'selected' not in obs:
                        obs['selected'] = False
                    if 'height' not in obs:
                        obs['height'] = 30
                return obstacles
        except (json.JSONDecodeError, IOError) as e:
            st.error(f"加载配置文件失败: {e}")
            return []
    return []


def save_obstacles(obstacles: List[Dict]) -> bool:
    """保存障碍物配置"""
    try:
        backup_config()
        data = {
            'obstacles': obstacles,
            'count': len(obstacles),
            'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'version': 'v16.0'
        }
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        st.error(f"保存失败: {e}")
        return False


def get_latest_backup() -> Optional[str]:
    """获取最新备份文件"""
    try:
        backup_files = [
            f for f in os.listdir(config.BACKUP_DIR) 
            if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')
        ]
        if backup_files:
            backup_files.sort(reverse=True)
            return os.path.join(config.BACKUP_DIR, backup_files[0])
    except Exception as e:
        st.error(f"获取备份文件失败: {e}")
    return None


def restore_from_backup(backup_path: str) -> bool:
    """从备份恢复配置"""
    try:
        with open(backup_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            obstacles = data.get('obstacles', [])
            save_obstacles(obstacles)
            return True
    except Exception as e:
        st.error(f"恢复备份失败: {e}")
        return False


# ==================== 新版避障算法（带多个中间绕行点） ====================

def get_blocking_obstacles(
    start: List[float], end: List[float], 
    obstacles_gcj: List[Dict], flight_altitude: float
) -> List[Dict]:
    """获取阻挡航线的障碍物（高度高于飞行高度且与线段相交）"""
    blocking = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking.append(obs)
    return blocking


def can_fly_direct(
    start: List[float], end: List[float], 
    obstacles_gcj: List[Dict], flight_altitude: float
) -> bool:
    """判断是否可以直接穿行"""
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    return len(blocking) == 0


def get_combined_obstacle_bounds(obstacles: List[Dict]) -> Dict:
    """获取多个障碍物的组合边界"""
    min_lng = float('inf')
    max_lng = -float('inf')
    min_lat = float('inf')
    max_lat = -float('inf')
    
    for obs in obstacles:
        coords = obs.get('polygon', [])
        if coords:
            for point in coords:
                min_lng = min(min_lng, point[0])
                max_lng = max(max_lng, point[0])
                min_lat = min(min_lat, point[1])
                max_lat = max(max_lat, point[1])
    
    return {
        'min_lng': min_lng,
        'max_lng': max_lng,
        'min_lat': min_lat,
        'max_lat': max_lat,
        'center_lng': (min_lng + max_lng) / 2,
        'center_lat': (min_lat + max_lat) / 2
    }


def generate_c_shape_waypoints(
    start: List[float], end: List[float],
    offset_x: float, num_waypoints: int
) -> List[List[float]]:
    """
    生成C形路径的中间航点
    路径形状：起点 → 垂直向上/下 → 水平移动 → 垂直向下/上 → 终点
    使用多个中间点使路径更平滑
    """
    waypoints = []
    
    # 中间点的数量（不包括起点和终点）
    num_intermediate = num_waypoints
    
    # 计算每个阶段的点数
    # 第一阶段：从起点到偏移线（垂直移动）
    # 第二阶段：沿偏移线水平移动
    # 第三阶段：从偏移线到终点（垂直移动）
    
    points_per_stage = max(1, num_intermediate // 3)
    
    # 第一阶段：垂直移动（起点 → 偏移线）
    for i in range(1, points_per_stage + 1):
        t = i / points_per_stage
        lat = start[1] + (offset_x[1] - start[1]) * t
        waypoints.append([start[0], lat])
    
    # 第二阶段：水平移动（沿偏移线）
    for i in range(1, points_per_stage + 1):
        t = i / points_per_stage
        lng = start[0] + (offset_x[0] - start[0]) * t
        waypoints.append([lng, offset_x[1]])
    
    # 第三阶段：垂直移动（偏移线 → 终点）
    for i in range(1, points_per_stage + 1):
        t = i / points_per_stage
        lat = offset_x[1] + (end[1] - offset_x[1]) * t
        waypoints.append([offset_x[0], lat])
    
    # 去重（移除可能的重复点）
    unique_waypoints = []
    for wp in waypoints:
        if not unique_waypoints or distance(unique_waypoints[-1], wp) > 1e-10:
            unique_waypoints.append(wp)
    
    return unique_waypoints


def find_path_left_avoidance(
    start: List[float], end: List[float], 
    obstacles_gcj: List[Dict], flight_altitude: float,
    safety_buffer: float = 15.0,
    num_waypoints: int = 5
) -> List[List[float]]:
    """
    向左绕行算法（C形路径，多个中间点）
    路径形状：起点 → 向左偏移 → 垂直移动到终点纬度 → 向右移动到终点
    """
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking:
        return [start, end]
    
    # 获取障碍物组合边界
    bounds = get_combined_obstacle_bounds(blocking)
    
    # 计算安全偏移距离（米转度）
    safe_lng, safe_lat = meters_to_deg(safety_buffer)
    
    # 向左偏移量
    left_offset_lng = bounds['min_lng'] - safe_lng * 2
    
    # 偏移点（向左偏移后的点）
    # 使用起点和终点的中间纬度，使路径更平滑
    mid_lat = (start[1] + end[1]) / 2
    offset_point = [left_offset_lng, mid_lat]
    
    # 生成C形路径的中间航点
    waypoints = generate_c_shape_waypoints(start, offset_point, num_waypoints)
    
    # 构建完整路径：起点 → 中间航点 → 终点
    path = [start] + waypoints + [end]
    
    # 验证路径是否有效，如果还有碰撞则进一步扩大绕行距离
    for i in range(config.MAX_ITERATIONS):
        if is_path_clear(path, obstacles_gcj, flight_altitude):
            break
        # 扩大绕行距离
        left_offset_lng -= safe_lng * (i + 1)
        offset_point = [left_offset_lng, mid_lat]
        waypoints = generate_c_shape_waypoints(start, offset_point, num_waypoints)
        path = [start] + waypoints + [end]
    
    return path


def find_path_right_avoidance(
    start: List[float], end: List[float], 
    obstacles_gcj: List[Dict], flight_altitude: float,
    safety_buffer: float = 15.0,
    num_waypoints: int = 5
) -> List[List[float]]:
    """
    向右绕行算法（C形路径，多个中间点）
    路径形状：起点 → 向右偏移 → 垂直移动到终点纬度 → 向左移动到终点
    """
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking:
        return [start, end]
    
    # 获取障碍物组合边界
    bounds = get_combined_obstacle_bounds(blocking)
    
    # 计算安全偏移距离（米转度）
    safe_lng, safe_lat = meters_to_deg(safety_buffer)
    
    # 向右偏移量
    right_offset_lng = bounds['max_lng'] + safe_lng * 2
    
    # 偏移点（向右偏移后的点）
    mid_lat = (start[1] + end[1]) / 2
    offset_point = [right_offset_lng, mid_lat]
    
    # 生成C形路径的中间航点
    waypoints = generate_c_shape_waypoints(start, offset_point, num_waypoints)
    
    # 构建完整路径：起点 → 中间航点 → 终点
    path = [start] + waypoints + [end]
    
    # 验证路径是否有效，如果还有碰撞则进一步扩大绕行距离
    for i in range(config.MAX_ITERATIONS):
        if is_path_clear(path, obstacles_gcj, flight_altitude):
            break
        # 扩大绕行距离
        right_offset_lng += safe_lng * (i + 1)
        offset_point = [right_offset_lng, mid_lat]
        waypoints = generate_c_shape_waypoints(start, offset_point, num_waypoints)
        path = [start] + waypoints + [end]
    
    return path


def find_path_up_avoidance(
    start: List[float], end: List[float], 
    obstacles_gcj: List[Dict], flight_altitude: float,
    safety_buffer: float = 15.0,
    num_waypoints: int = 5
) -> List[List[float]]:
    """
    向上绕行算法（从上方绕过障碍物）
    当飞行高度可以增加时使用
    """
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking:
        return [start, end]
    
    # 获取障碍物组合边界
    bounds = get_combined_obstacle_bounds(blocking)
    
    # 计算安全偏移距离（米转度）
    safe_lng, safe_lat = meters_to_deg(safety_buffer)
    
    # 向上偏移量（增加纬度）
    up_offset_lat = bounds['max_lat'] + safe_lat * 2
    
    # 偏移点（向上偏移后的点）
    mid_lng = (start[0] + end[0]) / 2
    offset_point = [mid_lng, up_offset_lat]
    
    # 生成C形路径的中间航点
    waypoints = generate_c_shape_waypoints(start, offset_point, num_waypoints)
    
    # 构建完整路径
    path = [start] + waypoints + [end]
    
    # 验证路径
    for i in range(config.MAX_ITERATIONS):
        if is_path_clear(path, obstacles_gcj, flight_altitude):
            break
        up_offset_lat += safe_lat * (i + 1)
        offset_point = [mid_lng, up_offset_lat]
        waypoints = generate_c_shape_waypoints(start, offset_point, num_waypoints)
        path = [start] + waypoints + [end]
    
    return path


def find_path_best_avoidance(
    start: List[float], end: List[float], 
    obstacles_gcj: List[Dict], flight_altitude: float,
    safety_buffer: float = 15.0,
    num_waypoints: int = 5
) -> List[List[float]]:
    """
    最佳航线算法
    计算左、右、上三个方向的路径，选择最短的
    """
    if can_fly_direct(start, end, obstacles_gcj, flight_altitude):
        return [start, end]
    
    # 计算三个方向的路径
    left_path = find_path_left_avoidance(start, end, obstacles_gcj, flight_altitude, safety_buffer, num_waypoints)
    right_path = find_path_right_avoidance(start, end, obstacles_gcj, flight_altitude, safety_buffer, num_waypoints)
    up_path = find_path_up_avoidance(start, end, obstacles_gcj, flight_altitude, safety_buffer, num_waypoints)
    
    # 计算路径长度（米）
    left_length = calculate_path_length_meters(left_path)
    right_length = calculate_path_length_meters(right_path)
    up_length = calculate_path_length_meters(up_path)
    
    # 选择最短路径
    min_length = min(left_length, right_length, up_length)
    if min_length == left_length:
        return left_path
    elif min_length == right_length:
        return right_path
    else:
        return up_path


def is_path_clear(
    path: List[List[float]], 
    obstacles_gcj: List[Dict], 
    flight_altitude: float
) -> bool:
    """检查整条路径是否与任何需要避让的障碍物相交"""
    for i in range(len(path) - 1):
        p1, p2 = path[i], path[i + 1]
        for obs in obstacles_gcj:
            if obs.get('height', 30) > flight_altitude:
                polygon = obs.get('polygon', [])
                if polygon and line_intersects_polygon(p1, p2, polygon):
                    return False
    return True


def calculate_path_length_meters(path: List[List[float]]) -> float:
    """计算路径总长度（米）"""
    total = 0.0
    for i in range(len(path) - 1):
        total += distance(path[i], path[i + 1]) * 111000
    return total


def calculate_path_length_deg(path: List[List[float]]) -> float:
    """计算路径总长度（度）"""
    total = 0.0
    for i in range(len(path) - 1):
        total += distance(path[i], path[i + 1])
    return total


def create_avoidance_path(
    start: List[float], end: List[float], 
    obstacles_gcj: List[Dict], flight_altitude: float, 
    direction: str, safety_radius: float = 5
) -> List[List[float]]:
    """创建避障路径的统一接口"""
    # 使用安全缓冲区
    safety_buffer = max(safety_radius, config.SAFETY_BUFFER_METERS)
    num_waypoints = max(5, config.NUM_WAYPOINTS)  # 至少5个中间点
    
    # 首先检查是否可以直接飞行
    if can_fly_direct(start, end, obstacles_gcj, flight_altitude):
        return [start, end]
    
    # 根据方向选择算法
    if direction == "向左绕行":
        return find_path_left_avoidance(start, end, obstacles_gcj, flight_altitude, safety_buffer, num_waypoints)
    elif direction == "向右绕行":
        return find_path_right_avoidance(start, end, obstacles_gcj, flight_altitude, safety_buffer, num_waypoints)
    else:  # "最佳航线"
        return find_path_best_avoidance(start, end, obstacles_gcj, flight_altitude, safety_buffer, num_waypoints)


# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    """心跳包数据类"""
    timestamp: str
    flight_time: float
    lat: float
    lng: float
    altitude: float
    voltage: float
    satellites: int
    speed: float
    progress: float
    arrived: bool
    safety_violation: bool
    remaining_distance: float


class HeartbeatSimulator:
    """心跳包模拟器"""
    
    def __init__(self, start_point_gcj: List[float]):
        self.history: List[HeartbeatData] = []
        self.current_pos: List[float] = start_point_gcj.copy()
        self.path: List[List[float]] = [start_point_gcj.copy()]
        self.path_index: int = 0
        self.simulating: bool = False
        self.flight_altitude: float = 50
        self.speed: int = 50
        self.progress: float = 0.0
        self.total_distance: float = 0.0
        self.distance_traveled: float = 0.0
        self.safety_radius: float = config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation: bool = False
        self.start_time: Optional[datetime] = None
        self.flight_log: List[HeartbeatData] = []
        self.last_update_time: Optional[float] = None
        
    def set_path(
        self, path: List[List[float]], 
        altitude: float = 50, speed: int = 50, 
        safety_radius: float = 5
    ):
        """设置飞行路径"""
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update_time = None
        
        self.total_distance = 0.0
        for i in range(len(path) - 1):
            self.total_distance += distance(path[i], path[i + 1])
    
    def update_and_generate(self, obstacles_gcj: List[Dict]) -> Optional[HeartbeatData]:
        """更新位置并生成心跳包"""
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
            return None
        
        current_time = time.time()
        if self.last_update_time is None:
            delta_time = config.HEARTBEAT_INTERVAL
        else:
            delta_time = min(0.5, current_time - self.last_update_time)
        self.last_update_time = current_time
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = distance(start, end)
        
        speed_m_per_s = config.BASE_SPEED_MPS * (self.speed / 100)
        move_distance = speed_m_per_s * delta_time
        
        self.distance_traveled += move_distance
        
        if self.total_distance > 0:
            completed_distance = 0.0
            for i in range(self.path_index):
                completed_distance += distance(self.path[i], self.path[i + 1])
            
            if segment_distance > 0:
                segment_progress = min(1.0, self.distance_traveled / segment_distance)
                completed_distance += segment_distance * segment_progress
            
            self.progress = min(1.0, completed_distance / self.total_distance)
        
        if self.distance_traveled >= segment_distance and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        else:
            if segment_distance > 0:
                t = min(1.0, max(0.0, self.distance_traveled / segment_distance))
                lng = start[0] + (end[0] - start[0]) * t
                lat = start[1] + (end[1] - start[1]) * t
                self.current_pos = [lng, lat]
        
        safe, _, _ = check_safety_radius(
            self.current_pos, obstacles_gcj, 
            self.flight_altitude, self.safety_radius
        )
        if not safe:
            self.safety_violation = True
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived: bool = False) -> HeartbeatData:
        """生成心跳包数据"""
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        if arrived:
            remaining_dist = 0.0
        else:
            remaining_in_path = 0.0
            if self.path_index < len(self.path) - 1:
                segment_remaining = distance(self.current_pos, self.path[self.path_index + 1])
                remaining_in_path += max(0, segment_remaining)
                
                for i in range(self.path_index + 1, len(self.path) - 1):
                    remaining_in_path += distance(self.path[i], self.path[i + 1])
            remaining_dist = remaining_in_path * 111000
        
        heartbeat = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=flight_time,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-config.VOLTAGE_VARIATION, config.VOLTAGE_VARIATION), 1),
            satellites=random.randint(*config.SAT_RANGE),
            speed=round(config.BASE_SPEED_MPS * (self.speed / 100), 1),
            progress=self.progress,
            arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=remaining_dist
        )
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        
        self.flight_log.append(heartbeat)
        if len(self.flight_log) > 1000:
            self.flight_log.pop(0)
        
        return heartbeat
    
    def export_flight_data(self) -> pd.DataFrame:
        """导出飞行数据"""
        if not self.flight_log:
            return pd.DataFrame()
        
        data = [{
            'timestamp': h.timestamp,
            'flight_time': h.flight_time,
            'lat': h.lat,
            'lng': h.lng,
            'altitude': h.altitude,
            'voltage': h.voltage,
            'satellites': h.satellites,
            'speed': h.speed,
            'progress': h.progress,
            'arrived': h.arrived,
            'safety_violation': h.safety_violation,
            'remaining_distance': h.remaining_distance
        } for h in self.flight_log]
        
        return pd.DataFrame(data)


# ==================== 地图创建 ====================
def create_planning_map(
    center_gcj: List[float], points_gcj: Dict, 
    obstacles_gcj: List[Dict], flight_history: Optional[List] = None,
    planned_path: Optional[List] = None, map_type: str = "satellite", 
    straight_blocked: bool = True, flight_altitude: float = 50, 
    drone_pos: Optional[List] = None, direction: str = "最佳航线", 
    safety_radius: float = 5
) -> folium.Map:
    """创建规划地图"""
    if map_type == "satellite":
        tiles = config.GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = config.GAODE_VECTOR_URL
        attr = "高德矢量地图"
    
    m = folium.Map(
        location=[center_gcj[1], center_gcj[0]], 
        zoom_start=16, tiles=tiles, attr=attr
    )
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={
            'polygon': {
                'allowIntersection': False, 'showArea': True, 
                'color': '#ff0000', 'fillColor': '#ff0000', 
                'fillOpacity': 0.4
            },
            'polyline': False, 'rectangle': False, 
            'circle': False, 'marker': False, 'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    # 绘制障碍物
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "darkred" if height > flight_altitude else "orange"
            folium.Polygon(
                [[c[1], c[0]] for c in coords], 
                color=color, weight=3, fill=True, 
                fill_color=color, fill_opacity=0.4, 
                popup=f"🚧 {obs.get('name')}\n高度: {height}m"
            ).add_to(m)
    
    # 绘制起点终点
    if points_gcj.get('A'):
        folium.Marker(
            [points_gcj['A'][1], points_gcj['A'][0]], 
            popup="🟢 起点", 
            icon=folium.Icon(color="green", icon="play", prefix="fa")
        ).add_to(m)
    if points_gcj.get('B'):
        folium.Marker(
            [points_gcj['B'][1], points_gcj['B'][0]], 
            popup="🔴 终点", 
            icon=folium.Icon(color="red", icon="stop", prefix="fa")
        ).add_to(m)
    
    # 绘制规划路径
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        if "向左" in direction:
            line_color = "purple"
        elif "向右" in direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine(
            path_locations, color=line_color, weight=5, 
            opacity=0.9, popup=f"✈️ {direction}\n航点数量: {len(planned_path)}"
        ).add_to(m)
        
        # 标记中间航点
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker(
                [point[1], point[0]], radius=4, color=line_color, 
                fill=True, fill_color="white", fill_opacity=0.8, 
                popup=f"航点 {i+1}"
            ).add_to(m)
    
    # 绘制直线航线
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine(
                [[points_gcj['A'][1], points_gcj['A'][0]], 
                 [points_gcj['B'][1], points_gcj['B'][0]]], 
                color="blue", weight=2, opacity=0.5, dash_array='5, 5', 
                popup="直线航线（可行）"
            ).add_to(m)
        else:
            folium.PolyLine(
                [[points_gcj['A'][1], points_gcj['A'][0]], 
                 [points_gcj['B'][1], points_gcj['B'][0]]], 
                color="gray", weight=2, opacity=0.4, dash_array='5, 5', 
                popup="⚠️ 直线被阻挡"
            ).add_to(m)
    
    # 绘制安全半径
    if drone_pos:
        folium.Circle(
            radius=safety_radius, location=[drone_pos[1], drone_pos[0]], 
            color="blue", weight=2, fill=True, fill_color="blue", 
            fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米"
        ).add_to(m)
    
    # 绘制历史轨迹
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(
                trail, color="orange", weight=2, opacity=0.6, 
                popup="历史轨迹"
            ).add_to(m)
    
    return m


# ==================== 辅助UI函数 ====================
def init_session_state():
    """初始化session state"""
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(),
        'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'last_hb_time': time.time(),
        'simulation_running': False,
        'flight_history': [],
        'planned_path': None,
        'last_flight_altitude': 50,
        'pending_obstacle': None,
        'current_direction': "最佳航线",
        'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True,
        'show_rename_dialog': False,
        'waiting_for_start_point': False,
        'waiting_for_end_point': False,
        'temp_click_point': None
    }
    
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value
    
    for obs in st.session_state.obstacles_gcj:
        if 'height' not in obs:
            obs['height'] = 30
        if 'selected' not in obs:
            obs['selected'] = False


def check_straight_blocked(
    points_gcj: Dict, obstacles_gcj: List[Dict], flight_altitude: float
) -> Tuple[bool, int]:
    """检查直线航线是否被阻挡"""
    blocked = False
    high_count = 0
    
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            high_count += 1
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(points_gcj['A'], points_gcj['B'], coords):
                blocked = True
    
    return blocked, high_count


def render_sidebar() -> Tuple[str, str, int, float, bool]:
    """渲染侧边栏"""
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=10, max_value=200, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider(
        "安全半径 (米)", min_value=1, max_value=20, 
        value=st.session_state.safety_radius, step=1
    )
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🔒 安全缓冲区")
    safety_buffer = st.sidebar.slider(
        "避障缓冲区 (米)", min_value=5, max_value=50, 
        value=int(config.SAFETY_BUFFER_METERS), step=5
    )
    config.SAFETY_BUFFER_METERS = safety_buffer
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("📍 绕行点数量")
    num_waypoints = st.sidebar.slider(
        "中间航点数量", min_value=3, max_value=15, 
        value=config.NUM_WAYPOINTS, step=1
    )
    config.NUM_WAYPOINTS = max(5, num_waypoints)  # 确保至少5个
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    
    return page, map_type, drone_speed, flight_alt, auto_save


# ==================== 页面渲染函数 ====================
def render_planning_page(map_type: str, drone_speed: int, flight_alt: float, auto_save: bool):
    """渲染航线规划页面"""
    st.header("🗺️ 航线规划 - 智能避障")
    
    straight_blocked, high_obstacles = check_straight_blocked(
        st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt
    )
    
    if straight_blocked:
        st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
        st.info(f"🔒 当前避障安全缓冲区: {config.SAFETY_BUFFER_METERS}米 | 📍 中间航点数量: {config.NUM_WAYPOINTS}")
    else:
        st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
    
    st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
    
    col1, col2 = st.columns([1, 1.5])
    
    with col1:
        render_planning_controls(flight_alt, drone_speed, auto_save)
    
    with col2:
        render_planning_map_view(map_type, flight_alt, straight_blocked)


def render_planning_controls(flight_alt: float, drone_speed: int, auto_save: bool):
    """渲染规划控制面板"""
    st.subheader("🎮 控制面板")
    
    with st.expander("📍 起点/终点设置", expanded=True):
        render_point_settings()
    
    with st.expander("🤖 路径规划策略", expanded=True):
        render_path_strategy(flight_alt)
    
    with st.expander("✈️ 飞行控制", expanded=True):
        render_flight_controls(flight_alt, drone_speed)
    
    st.markdown("### 📍 当前坐标")
    st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
    st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
    
    a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
    dist = math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2) * 111000
    st.caption(f"📏 直线距离: {dist:.0f} 米")


def render_point_settings():
    """渲染起点终点设置"""
    st.markdown("#### 🎯 设置方式选择")
    
    setting_mode = st.radio(
        "选择设置方式",
        ["✏️ 经纬度输入", "🖱️ 鼠标点击设置"],
        horizontal=True,
        key="point_setting_mode"
    )
    
    if setting_mode == "✏️ 经纬度输入":
        render_coordinate_input()
    else:
        render_mouse_click_setting()


def render_coordinate_input():
    """渲染经纬度输入界面"""
    st.markdown("#### 🟢 起点 A")
    col_a1, col_a2 = st.columns(2)
    with col_a1:
        a_lat = st.number_input(
            "纬度", value=st.session_state.points_gcj['A'][1], 
            format="%.6f", key="a_lat", step=0.000001
        )
    with col_a2:
        a_lng = st.number_input(
            "经度", value=st.session_state.points_gcj['A'][0], 
            format="%.6f", key="a_lng", step=0.000001
        )
    
    if st.button("📍 设置 A 点", use_container_width=True):
        st.session_state.points_gcj['A'] = [a_lng, a_lat]
        update_path_after_point_change()
        st.success(f"✅ 起点已设置为 ({a_lng:.6f}, {a_lat:.6f})")
        st.rerun()
    
    st.markdown("#### 🔴 终点 B")
    col_b1, col_b2 = st.columns(2)
    with col_b1:
        b_lat = st.number_input(
            "纬度", value=st.session_state.points_gcj['B'][1], 
            format="%.6f", key="b_lat", step=0.000001
        )
    with col_b2:
        b_lng = st.number_input(
            "经度", value=st.session_state.points_gcj['B'][0], 
            format="%.6f", key="b_lng", step=0.000001
        )
    
    if st.button("📍 设置 B 点", use_container_width=True):
        st.session_state.points_gcj['B'] = [b_lng, b_lat]
        update_path_after_point_change()
        st.success(f"✅ 终点已设置为 ({b_lng:.6f}, {b_lat:.6f})")
        st.rerun()


def render_mouse_click_setting():
    """渲染鼠标点击设置界面"""
    st.info("💡 提示：点击地图上的任意位置来设置起点或终点")
    
    col_status1, col_status2 = st.columns(2)
    
    with col_status1:
        if st.button("🎯 设置起点 (点击地图)", use_container_width=True, type="primary"):
            st.session_state.waiting_for_start_point = True
            st.session_state.waiting_for_end_point = False
            st.info("👉 请在地图上点击选择起点位置")
            st.rerun()
    
    with col_status2:
        if st.button("📍 设置终点 (点击地图)", use_container_width=True, type="primary"):
            st.session_state.waiting_for_end_point = True
            st.session_state.waiting_for_start_point = False
            st.info("👉 请在地图上点击选择终点位置")
            st.rerun()
    
    if st.session_state.waiting_for_start_point:
        st.warning("⏳ 等待设置起点... 请点击地图")
    elif st.session_state.waiting_for_end_point:
        st.warning("⏳ 等待设置终点... 请点击地图")
    
    if st.session_state.waiting_for_start_point or st.session_state.waiting_for_end_point:
        if st.button("❌ 取消当前操作", use_container_width=True):
            st.session_state.waiting_for_start_point = False
            st.session_state.waiting_for_end_point = False
            st.rerun()
    
    st.markdown("---")
    st.markdown("#### 📍 快速设置")
    
    col_reset1, col_reset2 = st.columns(2)
    with col_reset1:
        if st.button("🔄 重置到默认起点", use_container_width=True):
            st.session_state.points_gcj['A'] = config.DEFAULT_A_GCJ.copy()
            update_path_after_point_change()
            st.success("✅ 起点已重置为默认值")
            st.rerun()
    
    with col_reset2:
        if st.button("🔄 重置到默认终点", use_container_width=True):
            st.session_state.points_gcj['B'] = config.DEFAULT_B_GCJ.copy()
            update_path_after_point_change()
            st.success("✅ 终点已重置为默认值")
            st.rerun()


def update_path_after_point_change():
    """更新路径（起点或终点改变后调用）"""
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
        st.session_state.current_direction, st.session_state.safety_radius
    )


def render_path_strategy(flight_alt: float):
    """渲染路径规划策略"""
    st.markdown("**选择绕行方向：**")
    
    col_dir1, col_dir2, col_dir3 = st.columns(3)
    
    can_direct = can_fly_direct(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, flight_alt
    )
    
    with col_dir1:
        if st.button(
            "🔄 最佳航线", use_container_width=True, 
            type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"
        ):
            st.session_state.current_direction = "最佳航线"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "最佳航线",
                st.session_state.safety_radius
            )
            path_length = calculate_path_length_meters(st.session_state.planned_path)
            waypoint_count = len(st.session_state.planned_path) - 2
            st.success(f"已切换到最佳航线模式，路径长度: {path_length:.0f}米，航点数量: {waypoint_count}")
            st.rerun()
    
    with col_dir2:
        if st.button(
            "⬅️ 向左绕行", use_container_width=True,
            type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"
        ):
            st.session_state.current_direction = "向左绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向左绕行",
                st.session_state.safety_radius
            )
            path_length = calculate_path_length_meters(st.session_state.planned_path)
            waypoint_count = len(st.session_state.planned_path) - 2
            st.success(f"已切换到向左绕行模式，路径长度: {path_length:.0f}米，航点数量: {waypoint_count}")
            st.rerun()
    
    with col_dir3:
        if st.button(
            "➡️ 向右绕行", use_container_width=True,
            type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"
        ):
            st.session_state.current_direction = "向右绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向右绕行",
                st.session_state.safety_radius
            )
            path_length = calculate_path_length_meters(st.session_state.planned_path)
            waypoint_count = len(st.session_state.planned_path) - 2
            st.success(f"已切换到向右绕行模式，路径长度: {path_length:.0f}米，航点数量: {waypoint_count}")
            st.rerun()
    
    st.info(f"📌 当前绕行策略: **{st.session_state.current_direction}**")
    
    if can_direct:
        st.success("✅ 当前飞行高度足够高，可以直接穿行！")
    else:
        st.warning("⚠️ 当前飞行高度低于部分障碍物，需要绕行")
        blocking = get_blocking_obstacles(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt
        )
        st.caption(f"🚧 阻挡航线的障碍物数量: {len(blocking)}")
    
    if st.button("🔄 重新规划路径", use_container_width=True):
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        if st.session_state.planned_path:
            waypoint_count = len(st.session_state.planned_path) - 2
            st.success(f"已按照「{st.session_state.current_direction}」规划路径，{waypoint_count}个绕行点")
        st.rerun()


def render_flight_controls(flight_alt: float, drone_speed: int):
    """渲染飞行控制"""
    col_met1, col_met2, col_met3 = st.columns(3)
    with col_met1:
        st.metric("当前飞行高度", f"{flight_alt} m")
    with col_met2:
        st.metric("速度系数", f"{drone_speed}%")
    with col_met3:
        st.metric("🛡️ 安全半径", f"{st.session_state.safety_radius} 米")
    
    if st.session_state.planned_path:
        waypoint_count = len(st.session_state.planned_path) - 2
        st.metric("🎯 绕行点数量", waypoint_count)
        
        total_dist = calculate_path_length_meters(st.session_state.planned_path)
        st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")
    
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(
                    path, flight_alt, drone_speed, st.session_state.safety_radius
                )
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                waypoint_count = len(path) - 2
                st.success(
                    f"🚁 飞行已开始！{'路径中有' + str(waypoint_count) + '个绕行点' if waypoint_count > 0 else '直线飞行'}"
                )
                st.rerun()
            else:
                st.error("请先设置起点和终点")
    
    with col_btn2:
        if st.button("⏹️ 停止飞行", use_container_width=True):
            st.session_state.simulation_running = False
            st.session_state.heartbeat_sim.simulating = False
            st.info("飞行已停止")


def render_planning_map_view(map_type: str, flight_alt: float, straight_blocked: bool):
    """渲染规划地图视图"""
    st.subheader("🗺️ 规划地图")
    st.caption("🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行")
    st.caption("💡 提示：在鼠标点击设置模式下，直接点击地图即可设置起点或终点")
    st.caption("🎨 深红色=需避让障碍物 | 🟠 橙色=安全障碍物（高度≤飞行高度）")
    st.caption(f"🔒 安全缓冲区: {config.SAFETY_BUFFER_METERS}米 | 📍 中间航点: {config.NUM_WAYPOINTS}个")
    
    flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
    center = st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ
    
    if st.session_state.planned_path is None:
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt,
            st.session_state.current_direction, st.session_state.safety_radius
        )
    
    drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
    
    m = create_planning_map(
        center, st.session_state.points_gcj, st.session_state.obstacles_gcj,
        flight_trail, st.session_state.planned_path, map_type,
        straight_blocked, flight_alt, drone_pos,
        st.session_state.current_direction, st.session_state.safety_radius
    )
    
    output = st_folium(
        m, 
        width=700, 
        height=550, 
        returned_objects=["last_active_drawing", "last_clicked"]
    )
    
    handle_map_click(output)
    handle_drawing_output(output)


def handle_map_click(output: Any):
    """处理地图点击事件"""
    if output and output.get("last_clicked"):
        clicked = output["last_clicked"]
        if clicked and isinstance(clicked, dict):
            lng = clicked.get("lng")
            lat = clicked.get("lat")
            
            if lng is not None and lat is not None:
                if st.session_state.waiting_for_start_point:
                    st.session_state.points_gcj['A'] = [lng, lat]
                    update_path_after_point_change()
                    st.session_state.waiting_for_start_point = False
                    st.success(f"✅ 起点已设置: ({lng:.6f}, {lat:.6f})")
                    st.rerun()
                
                elif st.session_state.waiting_for_end_point:
                    st.session_state.points_gcj['B'] = [lng, lat]
                    update_path_after_point_change()
                    st.session_state.waiting_for_end_point = False
                    st.success(f"✅ 终点已设置: ({lng:.6f}, {lat:.6f})")
                    st.rerun()


def handle_drawing_output(output: Any):
    """处理绘图输出"""
    if output and output.get("last_active_drawing"):
        last = output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [])
            if coords and len(coords) > 0:
                poly = [[p[0], p[1]] for p in coords[0]]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                    if validate_polygon(poly):
                        st.session_state.pending_obstacle = poly
                        st.rerun()
    
    if st.session_state.pending_obstacle is not None:
        render_obstacle_dialog()


def render_obstacle_dialog():
    """渲染障碍物对话框"""
    st.markdown("---")
    st.subheader("📝 添加新障碍物")
    st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
    
    col_name1, col_name2 = st.columns(2)
    with col_name1:
        new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
    with col_name2:
        new_height = st.number_input(
            "障碍物高度 (米)", min_value=1, max_value=200, 
            value=30, step=5, key="height_input"
        )
    
    col_ok, col_cancel = st.columns(2)
    with col_ok:
        if st.button("✅ 确认添加", use_container_width=True, type="primary"):
            new_obstacle = {
                "name": new_name,
                "polygon": st.session_state.pending_obstacle,
                "height": new_height,
                "selected": False,
                "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{len(st.session_state.obstacles_gcj)}",
                "created_time": datetime.now().isoformat()
            }
            st.session_state.obstacles_gcj.append(new_obstacle)
            if st.session_state.auto_backup:
                save_obstacles(st.session_state.obstacles_gcj)
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
                st.session_state.current_direction, st.session_state.safety_radius
            )
            st.session_state.pending_obstacle = None
            st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
            st.rerun()
    with col_cancel:
        if st.button("❌ 取消", use_container_width=True):
            st.session_state.pending_obstacle = None
            st.rerun()


# ==================== 飞行监控页面 ====================
def render_flight_monitoring_page(map_type: str, flight_alt: float, drone_speed: int):
    """渲染飞行监控页面"""
    st.header("📡 飞行监控 - 实时心跳包")
    
    update_flight_simulation()
    
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        
        current_waypoint = 0
        total_waypoints = 0
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            total_waypoints = len(st.session_state.planned_path)
            
            if latest.arrived:
                current_waypoint = total_waypoints
            elif latest.progress >= 0 and not latest.arrived:
                if latest.progress < 1.0:
                    segment_index = int(latest.progress * (len(st.session_state.planned_path) - 1))
                    current_waypoint = segment_index + 1
                    current_waypoint = min(current_waypoint, total_waypoints)
                else:
                    current_waypoint = total_waypoints
        
        remaining_distance = latest.remaining_distance
        if latest.arrived:
            remaining_distance = 0.0
        elif remaining_distance < 0:
            remaining_distance = 0.0
            
        estimated_arrival = "计算中..."
        if latest.arrived:
            estimated_arrival = "00:00"
        elif latest.speed > 0 and remaining_distance > 0:
            eta_seconds = remaining_distance / latest.speed
            if eta_seconds < 60:
                estimated_arrival = f"{eta_seconds:.0f}秒"
            elif eta_seconds < 3600:
                minutes = int(eta_seconds // 60)
                seconds = int(eta_seconds % 60)
                estimated_arrival = f"{minutes:02d}:{seconds:02d}"
            else:
                hours = int(eta_seconds // 3600)
                minutes = int((eta_seconds % 3600) // 60)
                estimated_arrival = f"{hours:02d}:{minutes:02d}"
        
        max_flight_time = 1800
        battery_percentage = max(0, min(100, (1 - latest.flight_time / max_flight_time) * 100))
        if latest.voltage:
            voltage_percentage = ((latest.voltage - 21.0) / (22.2 - 21.0)) * 100
            battery_percentage = max(0, min(100, (battery_percentage + voltage_percentage) / 2))
        
        st.markdown("### ✈️ 飞行进度")
        st.progress(latest.progress if not latest.arrived else 1.0, text=f"飞行进度：{int(latest.progress*100) if not latest.arrived else 100}%")
        
        st.markdown("### 📊 实时飞行数据")
        
        col1, col2, col3 = st.columns(3)
        
        with col1:
            waypoint_display = f"{current_waypoint} / {total_waypoints}"
            if total_waypoints > 0:
                waypoint_progress_value = current_waypoint / total_waypoints if current_waypoint <= total_waypoints else 1.0
                st.metric(
                    label="🎯 当前航点",
                    value=waypoint_display,
                    delta=f"进度 {int(waypoint_progress_value*100)}%" if not latest.arrived else "已完成"
                )
                st.progress(waypoint_progress_value, text=f"航点进度: {int(waypoint_progress_value*100)}%")
            else:
                st.metric(label="🎯 当前航点", value="0 / 0")
        
        with col2:
            st.metric(
                label="💨 飞行速度",
                value=f"{latest.speed:.1f} m/s",
                delta=f"{drone_speed}% 系数" if not latest.arrived else "已到达"
            )
        
        with col3:
            minutes = int(latest.flight_time // 60)
            seconds = int(latest.flight_time % 60)
            time_display = f"{minutes:02d}:{seconds:02d}"
            st.metric(
                label="⏰ 已用时间",
                value=time_display,
                delta=f"{latest.flight_time:.1f}秒" if not latest.arrived else "已完成"
            )
        
        col4, col5, col6 = st.columns(3)
        
        with col4:
            if remaining_distance >= 1000:
                distance_text = f"{remaining_distance/1000:.2f} km"
            else:
                distance_text = f"{remaining_distance:.0f} m"
            
            st.metric(
                label="📏 剩余距离",
                value=distance_text if not latest.arrived else "0 m",
                delta="已到达!" if latest.arrived else None
            )
        
        with col5:
            st.metric(
                label="🕐 预计到达",
                value=estimated_arrival
            )
            if remaining_distance < 100 and remaining_distance > 0 and not latest.arrived:
                st.info("🏁 即将到达目的地！")
            elif latest.arrived:
                st.success("✅ 已到达目的地！")
        
        with col6:
            battery_color = "🟢" if battery_percentage > 50 else "🟡" if battery_percentage > 20 else "🔴"
            st.metric(
                label="🔋 电量模拟",
                value=f"{battery_color} {battery_percentage:.0f}%",
                delta=f"{latest.voltage:.1f}V"
            )
            if battery_percentage < 20 and not latest.arrived:
                st.warning("⚠️ 电量不足，请尽快返航！")
        
        st.markdown("### 📍 位置与状态")
        col7, col8, col9, col10 = st.columns(4)
        
        with col7:
            st.metric(label="📍 当前位置", value=f"{latest.lat:.6f}, {latest.lng:.6f}")
        
        with col8:
            st.metric(label="📏 飞行高度", value=f"{latest.altitude} m")
        
        with col9:
            st.metric(label="🛰️ 卫星数量", value=f"{latest.satellites} 颗")
        
        with col10:
            if latest.arrived:
                status = "✅ 已完成"
            elif st.session_state.simulation_running:
                status = "✈️ 飞行中"
            else:
                status = "⏸️ 已停止"
            st.metric(label="📌 飞行状态", value=status)
        
        if latest.safety_violation and not latest.arrived:
            st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
        
        if latest.arrived:
            st.success("🎉 无人机已到达目的地！飞行任务完成！")
            with st.expander("📊 飞行任务总结", expanded=True):
                col_sum1, col_sum2, col_sum3 = st.columns(3)
                with col_sum1:
                    minutes = int(latest.flight_time // 60)
                    seconds = int(latest.flight_time % 60)
                    st.metric("总飞行时间", f"{minutes:02d}:{seconds:02d}")
                with col_sum2:
                    total_distance = st.session_state.heartbeat_sim.total_distance * 111000
                    st.metric("总飞行距离", f"{total_distance:.0f} m")
                with col_sum3:
                    avg_speed = latest.speed if latest.speed > 0 else drone_speed * config.BASE_SPEED_MPS / 100
                    st.metric("平均速度", f"{avg_speed:.1f} m/s")
        
        st.markdown("---")
        
        st.markdown("### 🗺️ 实时位置追踪")
        display_monitor_map(map_type, latest, flight_alt)
        
        st.markdown("---")
        
        st.markdown("### 📈 实时数据图表")
        
        col_ch1, col_ch2 = st.columns(2)
        
        with col_ch1:
            st.subheader("📊 速度 vs 时间")
            if len(st.session_state.heartbeat_sim.history) > 1:
                speed_data = []
                for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
                    speed_data.append({"时间(s)": i * config.HEARTBEAT_INTERVAL, "速度(m/s)": h.speed})
                speed_df = pd.DataFrame(speed_data)
                st.line_chart(speed_df, x="时间(s)", y="速度(m/s)")
        
        with col_ch2:
            st.subheader("📏 剩余距离 vs 时间")
            if len(st.session_state.heartbeat_sim.history) > 1:
                dist_data = []
                for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
                    display_remaining = max(0, h.remaining_distance)
                    dist_data.append({"时间(s)": i * config.HEARTBEAT_INTERVAL, "剩余距离(m)": display_remaining})
                dist_df = pd.DataFrame(dist_data)
                st.line_chart(dist_df, x="时间(s)", y="剩余距离(m)")
        
        st.markdown("---")
        
        st.markdown("### 📋 飞行日志记录")
        display_flight_history()
        
        st.markdown("---")
        col_export1, col_export2, col_export3, col_export4 = st.columns(4)
        with col_export1:
            if st.button("📊 导出完整飞行数据", use_container_width=True, type="primary"):
                df = st.session_state.heartbeat_sim.export_flight_data()
                if not df.empty:
                    csv = df.to_csv(index=False)
                    st.download_button(
                        label="📥 下载CSV文件",
                        data=csv,
                        file_name=f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                        mime="text/csv",
                        use_container_width=True
                    )
        
        with col_export2:
            if st.button("📊 导出航点数据", use_container_width=True):
                if st.session_state.planned_path:
                    waypoint_data = []
                    for i, wp in enumerate(st.session_state.planned_path):
                        waypoint_data.append({
                            "航点序号": i + 1,
                            "航点类型": "起点" if i == 0 else "终点" if i == len(st.session_state.planned_path)-1 else f"绕行点{i}",
                            "经度": wp[0],
                            "纬度": wp[1]
                        })
                    wp_df = pd.DataFrame(waypoint_data)
                    csv = wp_df.to_csv(index=False, encoding='utf-8-sig')
                    st.download_button(
                        label="📥 下载航点CSV",
                        data=csv,
                        file_name=f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                        mime="text/csv",
                        use_container_width=True
                    )
        
        with col_export3:
            if st.button("🔄 刷新数据", use_container_width=True):
                st.rerun()
        
        with col_export4:
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
                st.success("飞行已停止")
                st.rerun()
                
    else:
        st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        st.markdown("---")
        col_tip1, col_tip2, col_tip3 = st.columns(3)
        with col_tip1:
            st.info("💡 提示1：先在航线规划页面设置起点和终点")
        with col_tip2:
            st.info("💡 提示2：设置飞行高度和速度系数")
        with col_tip3:
            st.info("💡 提示3：点击「开始飞行」按钮启动模拟")
        
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            st.markdown("---")
            st.subheader("🗺️ 规划航线预览")
            total_waypoints = len(st.session_state.planned_path)
            st.success(f"📌 已规划 {total_waypoints} 个航点（包括起点和终点），点击开始飞行后将按此航线飞行")


def display_monitor_map(map_type: str, latest, flight_alt: float):
    """显示监控地图"""
    tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
    monitor_map = folium.Map(
        location=[latest.lat, latest.lng], zoom_start=18, 
        tiles=tiles, attr="高德地图"
    )
    
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "darkred" if height > flight_alt else "orange"
            folium.Polygon(
                [[c[1], c[0]] for c in coords], color=color, 
                weight=2, fill=True, fill_opacity=0.3,
                popup=f"🚧 {obs.get('name')}\n高度: {height}m"
            ).add_to(monitor_map)
    
    if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
        if "向左" in st.session_state.current_direction:
            line_color = "purple"
        elif "向右" in st.session_state.current_direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine(
            [[p[1], p[0]] for p in st.session_state.planned_path], 
            color=line_color, weight=3, opacity=0.7,
            popup=f"规划航线 - {st.session_state.current_direction}"
        ).add_to(monitor_map)
    
    folium.Circle(
        radius=st.session_state.safety_radius, location=[latest.lat, latest.lng],
        color="blue", weight=2, fill=True, fill_color="blue", 
        fill_opacity=0.2, popup=f"🛡️ 安全半径: {st.session_state.safety_radius}米"
    ).add_to(monitor_map)
    
    trail = [[hb.lat, hb.lng] for hb in st.session_state.heartbeat_sim.history[:50] if hb.lat and hb.lng]
    if len(trail) > 1:
        folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史飞行轨迹").add_to(monitor_map)
    
    folium.Marker(
        [latest.lat, latest.lng], 
        popup=f"当前位置\n高度: {latest.altitude}m\n速度: {latest.speed}m/s", 
        icon=folium.Icon(color='red', icon='plane', prefix='fa')
    ).add_to(monitor_map)
    
    if st.session_state.points_gcj['A']:
        folium.Marker(
            [st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
            popup="起点 A", icon=folium.Icon(color='green', icon='play', prefix='fa')
        ).add_to(monitor_map)
    
    if st.session_state.points_gcj['B']:
        folium.Marker(
            [st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
            popup="终点 B", icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
        ).add_to(monitor_map)
    
    if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
        for i, point in enumerate(st.session_state.planned_path[1:-1]):
            folium.CircleMarker(
                [point[1], point[0]], radius=4, color="yellow", 
                fill=True, fill_color="yellow", fill_opacity=0.8,
                popup=f"航点 {i+1}"
            ).add_to(monitor_map)
    
    folium_static(monitor_map, width=900, height=500)


def display_flight_history():
    """显示飞行历史记录"""
    history_df = st.session_state.heartbeat_sim.export_flight_data()
    
    if not history_df.empty:
        display_cols = [
            'timestamp', 'flight_time', 'lat', 'lng', 'altitude', 
            'speed', 'voltage', 'satellites', 'remaining_distance'
        ]
        display_cols = [col for col in display_cols if col in history_df.columns]
        
        recent_df = history_df[display_cols].head(10)
        
        column_names = {
            'timestamp': '时间',
            'flight_time': '飞行时间(s)',
            'lat': '纬度',
            'lng': '经度',
            'altitude': '高度(m)',
            'speed': '速度(m/s)',
            'voltage': '电压(V)',
            'satellites': '卫星数',
            'remaining_distance': '剩余距离(m)'
        }
        recent_df = recent_df.rename(columns=column_names)
        
        st.dataframe(recent_df, use_container_width=True)
        
        st.markdown("### 📊 飞行统计")
        col_stat1, col_stat2, col_stat3, col_stat4 = st.columns(4)
        
        with col_stat1:
            max_speed = history_df['speed'].max() if 'speed' in history_df.columns else 0
            st.metric("🏁 最高速度", f"{max_speed:.1f} m/s")
        
        with col_stat2:
            avg_speed = history_df['speed'].mean() if 'speed' in history_df.columns else 0
            st.metric("📈 平均速度", f"{avg_speed:.1f} m/s")
        
        with col_stat3:
            max_alt = history_df['altitude'].max() if 'altitude' in history_df.columns else 0
            st.metric("⛰️ 最高高度", f"{max_alt:.0f} m")
        
        with col_stat4:
            total_time = history_df['flight_time'].max() if 'flight_time' in history_df.columns else 0
            st.metric("⏱️ 总飞行时间", f"{total_time:.1f} s")
    else:
        st.info("暂无飞行数据")


def update_flight_simulation():
    """更新飞行模拟"""
    current_time = time.time()
    if st.session_state.simulation_running:
        if current_time - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            try:
                new_hb = st.session_state.heartbeat_sim.update_and_generate(
                    st.session_state.obstacles_gcj
                )
                if new_hb:
                    st.session_state.last_hb_time = current_time
                    st.session_state.flight_history.append([new_hb.lng, new_hb.lat])
                    if len(st.session_state.flight_history) > 200:
                        st.session_state.flight_history.pop(0)
                    if not st.session_state.heartbeat_sim.simulating:
                        st.session_state.simulation_running = False
                        st.success("🏁 无人机已安全到达目的地！")
                    st.rerun()
            except Exception as e:
                st.error(f"更新心跳时出错: {e}")
    else:
        st.session_state.last_hb_time = current_time


# ==================== 障碍物管理页面 ====================
def render_obstacle_management_page(flight_alt: float):
    """渲染障碍物管理页面"""
    st.header("🚧 障碍物管理")
    
    col_status1, col_status2, col_status3, col_status4 = st.columns(4)
    with col_status1:
        st.info(f"📊 当前共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    with col_status2:
        st.info(f"🛡️ 安全半径: {st.session_state.safety_radius}米")
    with col_status3:
        if os.path.exists(config.CONFIG_FILE):
            try:
                with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    save_time = data.get('save_time', '未知')
                    st.info(f"💾 最后保存: {save_time}")
            except:
                st.info("💾 未保存")
        else:
            st.info("💾 未保存")
    with col_status4:
        backup_count = len(
            [f for f in os.listdir(config.BACKUP_DIR) 
             if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')]
        )
        st.info(f"📦 备份数量: {backup_count}")
    
    st.markdown("---")
    
    col_data1, col_data2, col_data3, col_data4, col_data5 = st.columns(5)
    
    with col_data1:
        if st.button("💾 保存配置", use_container_width=True, type="primary"):
            if save_obstacles(st.session_state.obstacles_gcj):
                st.success(f"✅ 已保存 {len(st.session_state.obstacles_gcj)} 个障碍物")
                st.balloons()
                time.sleep(0.5)
                st.rerun()
    
    with col_data2:
        if st.button("📂 加载配置", use_container_width=True):
            loaded = load_obstacles()
            if loaded:
                st.session_state.obstacles_gcj = loaded
                update_path_after_obstacle_change(flight_alt)
                st.success(f"✅ 已加载 {len(loaded)} 个障碍物")
                st.rerun()
            else:
                st.warning("⚠️ 未找到配置文件")
    
    with col_data3:
        if st.session_state.obstacles_gcj:
            config_data = {
                'obstacles': st.session_state.obstacles_gcj,
                'count': len(st.session_state.obstacles_gcj),
                'export_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'version': 'v16.0'
            }
            json_str = json.dumps(config_data, ensure_ascii=False, indent=2)
            st.download_button(
                label="📥 导出配置",
                data=json_str,
                file_name=f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                mime="application/json",
                use_container_width=True
            )
        else:
            st.button("📥 导出配置", use_container_width=True, disabled=True)
    
    with col_data4:
        latest_backup = get_latest_backup()
        if latest_backup:
            if st.button("🔄 恢复备份", use_container_width=True):
                if restore_from_backup(latest_backup):
                    st.session_state.obstacles_gcj = load_obstacles()
                    update_path_after_obstacle_change(flight_alt)
                    st.success("✅ 已从备份恢复")
                    st.rerun()
                else:
                    st.error("❌ 恢复失败")
        else:
            st.button("🔄 恢复备份", use_container_width=True, disabled=True)
    
    with col_data5:
        if st.button("🗑️ 清除全部", use_container_width=True):
            if st.session_state.auto_backup:
                backup_config()
            st.session_state.obstacles_gcj = []
            save_obstacles([])
            update_path_after_obstacle_change(flight_alt)
            st.success("✅ 已清除所有障碍物")
            st.rerun()
    
    st.markdown("---")
    
    col_stats1, col_stats2, col_stats3, col_stats4 = st.columns(4)
    with col_stats1:
        high_obs = sum(1 for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt)
        st.metric("🔴 需避让障碍物", high_obs)
    with col_stats2:
        safe_obs = len(st.session_state.obstacles_gcj) - high_obs
        st.metric("🟠 安全障碍物", safe_obs)
    with col_stats3:
        total_vertices = sum(len(obs.get('polygon', [])) for obs in st.session_state.obstacles_gcj)
        st.metric("📍 总顶点数", total_vertices)
    with col_stats4:
        avg_height = sum(obs.get('height', 30) for obs in st.session_state.obstacles_gcj) / max(1, len(st.session_state.obstacles_gcj))
        st.metric("📏 平均高度", f"{avg_height:.1f}m")
    
    st.markdown("---")
    
    st.subheader("🎯 批量操作")
    
    for obs in st.session_state.obstacles_gcj:
        if 'selected' not in obs:
            obs['selected'] = False
    
    col_batch1, col_batch2, col_batch3, col_batch4 = st.columns(4)
    
    with col_batch1:
        select_all = st.checkbox("☑️ 全选所有障碍物")
        if select_all:
            for obs in st.session_state.obstacles_gcj:
                obs['selected'] = True
    
    with col_batch2:
        if st.button("🗑️ 批量删除", use_container_width=True, type="primary"):
            selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
            if selected_indices:
                for i in reversed(selected_indices):
                    st.session_state.obstacles_gcj.pop(i)
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.success(f"✅ 已删除 {len(selected_indices)} 个障碍物")
                st.rerun()
            else:
                st.warning("⚠️ 请先选择要删除的障碍物")
    
    with col_batch3:
        batch_height = st.number_input("批量高度(m)", min_value=1, max_value=200, value=30, step=5, key="batch_height")
        if st.button("📏 批量设置高度", use_container_width=True):
            selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
            if selected_indices:
                for i in selected_indices:
                    st.session_state.obstacles_gcj[i]['height'] = batch_height
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.success(f"✅ 已为 {len(selected_indices)} 个障碍物设置高度为 {batch_height}m")
                st.rerun()
            else:
                st.warning("⚠️ 请先选择要修改的障碍物")
    
    with col_batch4:
        if st.button("🏷️ 批量重命名", use_container_width=True):
            selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
            if selected_indices:
                st.session_state.show_rename_dialog = True
            else:
                st.warning("⚠️ 请先选择要重命名的障碍物")
    
    if st.session_state.get('show_rename_dialog', False):
        with st.expander("🏷️ 批量重命名", expanded=True):
            col_n1, col_n2 = st.columns(2)
            with col_n1:
                name_prefix = st.text_input("名称前缀", value="建筑物")
                start_number = st.number_input("起始编号", min_value=1, value=1, step=1)
            with col_n2:
                name_suffix = st.text_input("名称后缀", value="")
            
            col_confirm, col_cancel_r = st.columns(2)
            with col_confirm:
                if st.button("确认重命名", use_container_width=True, type="primary"):
                    selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
                    for idx, i in enumerate(selected_indices):
                        new_name = f"{name_prefix}{start_number + idx}{name_suffix}"
                        st.session_state.obstacles_gcj[i]['name'] = new_name
                    if st.session_state.auto_backup:
                        save_obstacles(st.session_state.obstacles_gcj)
                    st.session_state.show_rename_dialog = False
                    st.success(f"✅ 已重命名 {len(selected_indices)} 个障碍物")
                    st.rerun()
            with col_cancel_r:
                if st.button("取消"):
                    st.session_state.show_rename_dialog = False
                    st.rerun()
    
    st.markdown("---")
    
    tab_list, tab_map = st.tabs(["📋 列表视图", "🗺️ 地图视图"])
    
    with tab_list:
        render_obstacle_list_view(flight_alt)
    
    with tab_map:
        render_obstacle_map_view(flight_alt)


def render_obstacle_list_view(flight_alt: float):
    """渲染障碍物列表视图"""
    st.subheader("📝 障碍物列表")
    st.caption("💡 提示：勾选复选框后可使用批量操作功能")
    
    if st.session_state.obstacles_gcj:
        items_per_row = 2
        rows = (len(st.session_state.obstacles_gcj) + items_per_row - 1) // items_per_row
        
        for row in range(rows):
            cols = st.columns(items_per_row)
            for col_idx in range(items_per_row):
                idx = row * items_per_row + col_idx
                if idx < len(st.session_state.obstacles_gcj):
                    render_obstacle_card(idx, flight_alt, cols[col_idx])
    else:
        st.info("📭 暂无任何障碍物，可以在「地图视图」中绘制添加")


def render_obstacle_card(idx: int, flight_alt: float, container):
    """渲染单个障碍物卡片"""
    obs = st.session_state.obstacles_gcj[idx]
    with container:
        with st.container(border=True):
            height = obs.get('height', 30)
            color = "🔴" if height > flight_alt else "🟠"
            name = obs.get('name', f'障碍物{idx+1}')
            
            col_check, col_name = st.columns([1, 5])
            with col_check:
                checked = st.checkbox("", key=f"select_card_{idx}", value=obs.get('selected', False))
                st.session_state.obstacles_gcj[idx]['selected'] = checked
            with col_name:
                st.markdown(f"**{color} {name}**")
            
            col_h1, col_h2 = st.columns(2)
            with col_h1:
                st.caption(f"📏 高度: {height}m")
            with col_h2:
                st.caption(f"📍 顶点: {len(obs.get('polygon', []))}个")
            
            new_h = st.number_input(
                "调整高度", value=height, min_value=1, max_value=200, 
                step=5, key=f"quick_edit_{idx}", label_visibility="collapsed"
            )
            if new_h != height:
                obs['height'] = new_h
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.rerun()
            
            if st.button("🗑️ 删除", key=f"delete_card_{idx}", use_container_width=True):
                st.session_state.obstacles_gcj.pop(idx)
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.rerun()


def render_obstacle_map_view(flight_alt: float):
    """渲染障碍物地图视图"""
    st.subheader("🗺️ 地图视图")
    st.caption("✏️ 使用左上角绘制工具绘制新障碍物 | 🖱️ 点击障碍物查看详细信息 | 🎨 深红色=需避让，橙色=安全")
    
    map_view_type = st.radio("地图类型", ["卫星影像", "矢量街道"], index=0, horizontal=True)
    map_type_view = "satellite" if map_view_type == "卫星影像" else "vector"
    
    tiles = config.GAODE_SATELLITE_URL if map_type_view == "satellite" else config.GAODE_VECTOR_URL
    obs_map = folium.Map(
        location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], 
        zoom_start=16, tiles=tiles, attr="高德地图"
    )
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={
            'polygon': {
                'allowIntersection': False, 'showArea': True, 
                'color': '#ff0000', 'fillColor': '#ff0000', 
                'fillOpacity': 0.4
            },
            'polyline': False, 'rectangle': False, 
            'circle': False, 'marker': False, 'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    obs_map.add_child(draw)
    
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        color = "darkred" if height > flight_alt else "orange"
        if coords and len(coords) >= 3:
            popup_text = f"""
            <div style="font-family: sans-serif;">
                <b>🏢 {obs.get('name')}</b><br>
                高度: {height} 米<br>
                ID: {obs.get('id', 'N/A')}<br>
            </div>
            """
            folium.Polygon(
                [[c[1], c[0]] for c in coords], color=color, weight=3, 
                fill=True, fill_color=color, fill_opacity=0.5, 
                popup=folium.Popup(popup_text, max_width=300)
            ).add_to(obs_map)
    
    folium.Marker(
        [config.DEFAULT_A_GCJ[1], config.DEFAULT_A_GCJ[0]], 
        popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')
    ).add_to(obs_map)
    folium.Marker(
        [config.DEFAULT_B_GCJ[1], config.DEFAULT_B_GCJ[0]], 
        popup="终点", icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
    ).add_to(obs_map)
    
    map_output = st_folium(
        obs_map, width=800, height=550, key="obstacle_map_view", 
        returned_objects=["last_active_drawing"]
    )
    
    if map_output and map_output.get("last_active_drawing"):
        last = map_output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [])
            if coords and len(coords) > 0:
                poly = [[p[0], p[1]] for p in coords[0]]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                    if validate_polygon(poly):
                        st.session_state.pending_obstacle = poly
                        st.rerun()
    
    if st.session_state.pending_obstacle is not None:
        render_obstacle_dialog()


def update_path_after_obstacle_change(flight_alt: float):
    """障碍物变更后更新路径"""
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, flight_alt,
        st.session_state.current_direction, st.session_state.safety_radius
    )


# ==================== 主程序 ====================
def main():
    """主程序入口"""
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    
    init_session_state()
    
    st.title("🏫 无人机地面站系统")
    st.markdown("---")
    
    page, map_type, drone_speed, flight_alt, auto_save = render_sidebar()
    st.session_state.auto_backup = auto_save
    
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt,
                st.session_state.current_direction, st.session_state.safety_radius
            )
            st.rerun()
    
    if page == "🗺️ 航线规划":
        render_planning_page(map_type, drone_speed, flight_alt, auto_save)
    elif page == "📡 飞行监控":
        render_flight_monitoring_page(map_type, flight_alt, drone_speed)
    elif page == "🚧 障碍物管理":
        render_obstacle_management_page(flight_alt)


if __name__ == "__main__":
    main()
