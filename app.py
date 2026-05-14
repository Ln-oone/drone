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


# ==================== 配置常量 ====================
@dataclass
class Config:
    """系统配置类"""
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.748726, 32.233881])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.750110, 32.235460])
    
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 5
    MAX_BACKUP_FILES: int = 10
    
    BASE_SPEED_MPS: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    VOLTAGE_VARIATION: float = 0.5
    SAT_RANGE: Tuple[int, int] = (8, 14)
    
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    
    VERTICAL_OFFSET_MULTIPLIER: float = 3.0
    WAYPOINT_OFFSET_FACTOR: float = 10.0


config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)


# ==================== 几何函数 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside


def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))


def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-10:
        return 0
    return 1 if val > 0 else 2


def segments_intersect(p1: List[float], p2: List[float], p3: List[float], p4: List[float]) -> bool:
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


def line_intersects_polygon(p1: List[float], p2: List[float], polygon: List[List[float]]) -> bool:
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
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def get_polygon_bounds(polygon: List[List[float]]) -> Optional[Dict]:
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
    return len(polygon) >= 3


def meters_to_deg(meters: float, lat: float = 32.23) -> Tuple[float, float]:
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg


def point_to_segment_distance_deg(point: List[float], seg_start: List[float], seg_end: List[float]) -> float:
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2)
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0, min(1, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


def point_to_segment_distance_meters(point: List[float], seg_start: List[float], seg_end: List[float]) -> float:
    return point_to_segment_distance_deg(point, seg_start, seg_end) * 111000


def check_safety_radius(drone_pos: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float) -> Tuple[bool, Optional[float], Optional[str]]:
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
    try:
        backup_files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')]
        if len(backup_files) > config.MAX_BACKUP_FILES:
            backup_files.sort()
            for old_file in backup_files[:-config.MAX_BACKUP_FILES]:
                os.remove(os.path.join(config.BACKUP_DIR, old_file))
    except Exception as e:
        st.warning(f"清理备份文件时出错: {e}")


def backup_config() -> Optional[str]:
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
    try:
        backup_config()
        data = {
            'obstacles': obstacles,
            'count': len(obstacles),
            'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'version': 'v13.2'
        }
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        st.error(f"保存失败: {e}")
        return False


def get_latest_backup() -> Optional[str]:
    try:
        backup_files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')]
        if backup_files:
            backup_files.sort(reverse=True)
            return os.path.join(config.BACKUP_DIR, backup_files[0])
    except Exception as e:
        st.error(f"获取备份文件失败: {e}")
    return None


def restore_from_backup(backup_path: str) -> bool:
    try:
        with open(backup_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            obstacles = data.get('obstacles', [])
            save_obstacles(obstacles)
            return True
    except Exception as e:
        st.error(f"恢复备份失败: {e}")
        return False


# ==================== 绕行算法 ====================
def get_blocking_obstacles(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    blocking = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking.append(obs)
    return blocking


def is_path_safe(p1: List[float], p2: List[float], obstacles: List[Dict], flight_altitude: float) -> bool:
    for obs in obstacles:
        if obs.get('height', 30) > flight_altitude:
            poly = obs.get('polygon', [])
            if poly and line_intersects_polygon(p1, p2, poly):
                return False
    return True


def get_obstacle_extent(obstacles: List[Dict]) -> Tuple[float, float, float, float]:
    min_lng, max_lng = float('inf'), -float('inf')
    min_lat, max_lat = float('inf'), -float('inf')
    for obs in obstacles:
        for point in obs.get('polygon', []):
            min_lng = min(min_lng, point[0])
            max_lng = max(max_lng, point[0])
            min_lat = min(min_lat, point[1])
            max_lat = max(max_lat, point[1])
    return min_lng, max_lng, min_lat, max_lat


def find_left_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking_obs:
        return [start, end]
    
    min_lng, max_lng, min_lat, max_lat = get_obstacle_extent(blocking_obs)
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    
    if length < 1e-9:
        return [start, end]
    
    ux, uy = dx / length, dy / length
    left_x, left_y = -uy, ux
    mid_lat = (start[1] + end[1]) / 2
    deg_per_meter_lng = 1 / (111000 * math.cos(math.radians(mid_lat)))
    
    for attempt in range(1, 51):
        offset_meters = safety_radius * 3 * attempt
        offset_deg_lng = offset_meters * deg_per_meter_lng
        mid_x = (start[0] + end[0]) / 2
        mid_y = (start[1] + end[1]) / 2
        waypoint = [mid_x + left_x * offset_deg_lng, mid_y + left_y * offset_deg_lng]
        
        if is_path_safe(start, waypoint, blocking_obs, flight_altitude) and is_path_safe(waypoint, end, blocking_obs, flight_altitude):
            return [start, waypoint, end]
    
    leftmost_x = min_lng
    mid_y = (start[1] + end[1]) / 2
    offset_meters = safety_radius * 20
    offset_deg_lng = offset_meters * deg_per_meter_lng
    waypoint = [leftmost_x - offset_deg_lng, mid_y]
    return [start, waypoint, end]


def find_right_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking_obs:
        return [start, end]
    
    min_lng, max_lng, min_lat, max_lat = get_obstacle_extent(blocking_obs)
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    
    if length < 1e-9:
        return [start, end]
    
    ux, uy = dx / length, dy / length
    right_x, right_y = uy, -ux
    mid_lat = (start[1] + end[1]) / 2
    deg_per_meter_lng = 1 / (111000 * math.cos(math.radians(mid_lat)))
    
    for attempt in range(1, 51):
        offset_meters = safety_radius * 3 * attempt
        offset_deg_lng = offset_meters * deg_per_meter_lng
        mid_x = (start[0] + end[0]) / 2
        mid_y = (start[1] + end[1]) / 2
        waypoint = [mid_x + right_x * offset_deg_lng, mid_y + right_y * offset_deg_lng]
        
        if is_path_safe(start, waypoint, blocking_obs, flight_altitude) and is_path_safe(waypoint, end, blocking_obs, flight_altitude):
            return [start, waypoint, end]
    
    rightmost_x = max_lng
    mid_y = (start[1] + end[1]) / 2
    offset_meters = safety_radius * 20
    offset_deg_lng = offset_meters * deg_per_meter_lng
    waypoint = [rightmost_x + offset_deg_lng, mid_y]
    return [start, waypoint, end]


def find_best_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    if is_path_safe(start, end, obstacles_gcj, flight_altitude):
        return [start, end]
    
    left_path = find_left_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    right_path = find_right_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    
    left_len = sum(distance(left_path[i], left_path[i+1]) for i in range(len(left_path)-1))
    right_len = sum(distance(right_path[i], right_path[i+1]) for i in range(len(right_path)-1))
    
    return left_path if left_len <= right_len else right_path


def create_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, direction: str, safety_radius: float = 5) -> List[List[float]]:
    straight_safe = True
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                straight_safe = False
                break
    if straight_safe:
        return [start, end]
    
    if direction == "向左绕行":
        return find_left_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        return find_right_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    else:
        return find_best_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)


def calculate_path_length(path: List[List[float]]) -> float:
    total = 0.0
    for i in range(len(path) - 1):
        total += distance(path[i], path[i + 1])
    return total


# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
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
        
    def set_path(self, path: List[List[float]], altitude: float = 50, speed: int = 50, safety_radius: float = 5):
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
        
        if segment_distance < 1e-9:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
            return self._generate_heartbeat(False)
        
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
        
        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        if not safe:
            self.safety_violation = True
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived: bool = False) -> HeartbeatData:
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
        
        return HeartbeatData(
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
    
    def export_flight_data(self) -> pd.DataFrame:
        if not self.flight_log:
            return pd.DataFrame()
        data = [{'timestamp': h.timestamp, 'flight_time': h.flight_time, 'lat': h.lat, 'lng': h.lng,
                 'altitude': h.altitude, 'voltage': h.voltage, 'satellites': h.satellites, 'speed': h.speed,
                 'progress': h.progress, 'arrived': h.arrived, 'safety_violation': h.safety_violation,
                 'remaining_distance': h.remaining_distance} for h in self.flight_log]
        return pd.DataFrame(data)


# ==================== 地图创建 ====================
def create_planning_map(center_gcj: List[float], points_gcj: Dict, obstacles_gcj: List[Dict], 
                        flight_history: Optional[List] = None, planned_path: Optional[List] = None,
                        map_type: str = "satellite", straight_blocked: bool = True, flight_altitude: float = 50,
                        drone_pos: Optional[List] = None, direction: str = "最佳航线", safety_radius: float = 5) -> folium.Map:
    tiles = config.GAODE_SATELLITE_URL
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr="高德卫星地图")
    
    draw = plugins.Draw(export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000',
                      'fillColor': '#ff0000', 'fillOpacity': 0.4}, 'polyline': False, 'rectangle': False,
                      'circle': False, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True})
    m.add_child(draw)
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True,
                          fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点",
                     icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点",
                     icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        if "向左" in direction:
            line_color = "purple"
        elif "向右" in direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine(path_locations, color=line_color, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=5, color=line_color, fill=True,
                               fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        color = "blue" if not straight_blocked else "gray"
        folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]],
                       color=color, weight=2, opacity=0.4, dash_array='5, 5',
                       popup="直线航线" if not straight_blocked else "⚠️ 直线被阻挡").add_to(m)
    
    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]], color="blue",
                     weight=2, fill=True, fill_color="blue", fill_opacity=0.2,
                     popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)
    
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    
    return m


# ==================== 辅助UI函数 ====================
def init_session_state():
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


def check_straight_blocked(points_gcj: Dict, obstacles_gcj: List[Dict], flight_altitude: float) -> Tuple[bool, int]:
    blocked = False
    high_count = 0
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            high_count += 1
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(points_gcj['A'], points_gcj['B'], coords):
                blocked = True
    return blocked, high_count


def render_sidebar() -> Tuple[str, int, float, bool]:
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=10, max_value=200, value=50, step=5)
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", min_value=1, max_value=20, value=st.session_state.safety_radius, step=1)
    st.session_state.safety_radius = safety_radius
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    return page, drone_speed, flight_alt, auto_save


# ==================== 航线规划页面 ====================
def render_planning_page(drone_speed: int, flight_alt: float, auto_save: bool):
    st.header("🗺️ 航线规划 - 智能避障")
    
    straight_blocked, high_obstacles = check_straight_blocked(st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt)
    if straight_blocked:
        st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
    else:
        st.success("✅ 直线航线畅通无阻")
    
    st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
    
    col1, col2 = st.columns([1, 1.5])
    with col1:
        render_planning_controls(flight_alt, auto_save)
    with col2:
        render_planning_map_view(flight_alt, straight_blocked)


def render_planning_controls(flight_alt: float, auto_save: bool):
    st.subheader("🎮 控制面板")
    
    with st.expander("📍 起点/终点设置", expanded=True):
        render_point_settings()
    
    with st.expander("🤖 路径规划策略", expanded=True):
        render_path_strategy(flight_alt)
    
    with st.expander("✈️ 飞行参数", expanded=True):
        render_flight_params(flight_alt)
    
    st.markdown("### 📍 当前坐标")
    st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
    st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
    
    a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
    dist = math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2) * 111000
    st.caption(f"📏 直线距离: {dist:.0f} 米")


def render_point_settings():
    st.markdown("#### 🎯 设置方式选择")
    setting_mode = st.radio("选择设置方式", ["✏️ 经纬度输入", "🖱️ 鼠标点击设置"], horizontal=True, key="point_setting_mode")
    if setting_mode == "✏️ 经纬度输入":
        render_coordinate_input()
    else:
        render_mouse_click_setting()


def render_coordinate_input():
    st.markdown("#### 🟢 起点 A")
    col_a1, col_a2 = st.columns(2)
    with col_a1:
        a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat", step=0.000001)
    with col_a2:
        a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng", step=0.000001)
    if st.button("📍 设置 A 点", use_container_width=True):
        st.session_state.points_gcj['A'] = [a_lng, a_lat]
        update_path_after_point_change()
        st.success(f"✅ 起点已设置为 ({a_lng:.6f}, {a_lat:.6f})")
        st.rerun()
    
    st.markdown("#### 🔴 终点 B")
    col_b1, col_b2 = st.columns(2)
    with col_b1:
        b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat", step=0.000001)
    with col_b2:
        b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng", step=0.000001)
    if st.button("📍 设置 B 点", use_container_width=True):
        st.session_state.points_gcj['B'] = [b_lng, b_lat]
        update_path_after_point_change()
        st.success(f"✅ 终点已设置为 ({b_lng:.6f}, {b_lat:.6f})")
        st.rerun()


def render_mouse_click_setting():
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
    
    if st.session_state.waiting_for_start_point or st.session_state.waiting_for_end_point:
        if st.button("❌ 取消当前操作", use_container_width=True):
            st.session_state.waiting_for_start_point = False
            st.session_state.waiting_for_end_point = False
            st.rerun()
    
    st.markdown("---")
    col_reset1, col_reset2 = st.columns(2)
    with col_reset1:
        if st.button("🔄 重置到默认起点", use_container_width=True):
            st.session_state.points_gcj['A'] = config.DEFAULT_A_GCJ.copy()
            update_path_after_point_change()
            st.rerun()
    with col_reset2:
        if st.button("🔄 重置到默认终点", use_container_width=True):
            st.session_state.points_gcj['B'] = config.DEFAULT_B_GCJ.copy()
            update_path_after_point_change()
            st.rerun()


def update_path_after_point_change():
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
        st.session_state.current_direction, st.session_state.safety_radius)


def render_path_strategy(flight_alt: float):
    st.markdown("**选择绕行方向：**")
    col_dir1, col_dir2, col_dir3 = st.columns(3)
    
    with col_dir1:
        if st.button("🔄 最佳航线", use_container_width=True, type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"):
            st.session_state.current_direction = "最佳航线"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "最佳航线", st.session_state.safety_radius)
            st.success("已切换到最佳航线模式")
            st.rerun()
    with col_dir2:
        if st.button("⬅️ 向左绕行", use_container_width=True, type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"):
            st.session_state.current_direction = "向左绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向左绕行", st.session_state.safety_radius)
            st.success("已切换到向左绕行模式")
            st.rerun()
    with col_dir3:
        if st.button("➡️ 向右绕行", use_container_width=True, type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"):
            st.session_state.current_direction = "向右绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向右绕行", st.session_state.safety_radius)
            st.success("已切换到向右绕行模式")
            st.rerun()
    
    st.info(f"📌 当前策略: **{st.session_state.current_direction}**")
    
    if st.button("🔄 重新规划路径", use_container_width=True):
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
        if st.session_state.planned_path:
            waypoint_count = len(st.session_state.planned_path) - 2
            st.success(f"已按照「{st.session_state.current_direction}」规划路径，{waypoint_count}个绕行点")
        st.rerun()


def render_flight_params(flight_alt: float):
    """显示飞行参数（不包含控制按钮）"""
    col_met1, col_met2, col_met3 = st.columns(3)
    with col_met1:
        st.metric("当前飞行高度", f"{flight_alt} m")
    with col_met2:
        st.metric("速度系数", f"{st.session_state.get('drone_speed', 50)}%")
    with col_met3:
        st.metric("🛡️ 安全半径", f"{st.session_state.safety_radius} 米")
    
    if st.session_state.planned_path:
        waypoint_count = len(st.session_state.planned_path) - 2
        st.metric("🎯 绕行点数量", waypoint_count)
        total_dist = calculate_path_length(st.session_state.planned_path) * 111000
        st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")


def render_planning_map_view(flight_alt: float, straight_blocked: bool):
    st.subheader("🗺️ 规划地图")
    flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
    center = st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ
    
    if st.session_state.planned_path is None:
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt,
            st.session_state.current_direction, st.session_state.safety_radius)
    
    drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
    
    m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail,
                           st.session_state.planned_path, "satellite", straight_blocked, flight_alt,
                           drone_pos, st.session_state.current_direction, st.session_state.safety_radius)
    
    output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
    handle_map_click(output)
    handle_drawing_output(output)


def handle_map_click(output: Any):
    if output and output.get("last_clicked"):
        clicked = output["last_clicked"]
        if clicked and isinstance(clicked, dict):
            lng, lat = clicked.get("lng"), clicked.get("lat")
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
    if output and output.get("last_active_drawing"):
        last = output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [])
            if coords and len(coords) > 0:
                poly = [[p[0], p[1]] for p in coords[0]]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None and validate_polygon(poly):
                    st.session_state.pending_obstacle = poly
                    st.rerun()
    
    if st.session_state.pending_obstacle is not None:
        render_obstacle_dialog()


def render_obstacle_dialog():
    st.markdown("---")
    st.subheader("📝 添加新障碍物")
    st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
    
    col_name1, col_name2 = st.columns(2)
    with col_name1:
        new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
    with col_name2:
        new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, value=30, step=5)
    
    col_ok, col_cancel = st.columns(2)
    with col_ok:
        if st.button("✅ 确认添加", use_container_width=True, type="primary"):
            new_obstacle = {"name": new_name, "polygon": st.session_state.pending_obstacle, "height": new_height,
                           "selected": False, "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
                           "created_time": datetime.now().isoformat()}
            st.session_state.obstacles_gcj.append(new_obstacle)
            if st.session_state.auto_backup:
                save_obstacles(st.session_state.obstacles_gcj)
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
                st.session_state.current_direction, st.session_state.safety_radius)
            st.session_state.pending_obstacle = None
            st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
            st.rerun()
    with col_cancel:
        if st.button("❌ 取消", use_container_width=True):
            st.session_state.pending_obstacle = None
            st.rerun()


# ==================== 飞行监控页面 ====================
def render_flight_monitoring_page(flight_alt: float, drone_speed: int):
    st.header("📡 飞行监控 - 实时心跳包")
    
    # 飞行控制按钮区域
    st.subheader("🎮 飞行控制")
    col_btn1, col_btn2, col_btn3 = st.columns(3)
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
                st.success(f"🚁 飞行已开始！{'路径中有' + str(waypoint_count) + '个绕行点' if waypoint_count > 0 else '直线飞行'}")
                st.rerun()
            else:
                st.error("请先设置起点和终点")
    with col_btn2:
        if st.button("⏹️ 停止飞行", use_container_width=True):
            st.session_state.simulation_running = False
            st.session_state.heartbeat_sim.simulating = False
            st.info("飞行已停止")
            st.rerun()
    with col_btn3:
        if st.button("🔄 刷新数据", use_container_width=True):
            st.rerun()
    
    st.markdown("---")
    
    # 更新飞行模拟
    update_flight_simulation()
    
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        
        # 飞行进度条
        st.progress(latest.progress if not latest.arrived else 1.0, text=f"飞行进度：{int(latest.progress*100)}%")
        
        # 主要指标
        col1, col2, col3, col4 = st.columns(4)
        with col1:
            st.metric("💨 飞行速度", f"{latest.speed:.1f} m/s")
        with col2:
            minutes, seconds = int(latest.flight_time // 60), int(latest.flight_time % 60)
            st.metric("⏰ 已用时间", f"{minutes:02d}:{seconds:02d}")
        with col3:
            remaining = max(0, latest.remaining_distance)
            st.metric("📏 剩余距离", f"{remaining:.0f} m" if remaining < 1000 else f"{remaining/1000:.2f} km")
        with col4:
            status = "✈️ 飞行中" if st.session_state.simulation_running and not latest.arrived else "✅ 已完成" if latest.arrived else "⏸️ 已停止"
            st.metric("📌 飞行状态", status)
        
        if latest.safety_violation and not latest.arrived:
            st.error("⚠️ 警告：无人机进入安全半径危险区域！")
        
        if latest.arrived:
            st.success("🎉 无人机已到达目的地！")
        
        st.markdown("---")
        st.markdown("### 🗺️ 实时位置追踪")
        display_monitor_map(flight_alt, latest)
        
        # 导出数据按钮
        col_export1, col_export2 = st.columns(2)
        with col_export1:
            if st.button("📊 导出飞行数据", use_container_width=True):
                df = st.session_state.heartbeat_sim.export_flight_data()
                if not df.empty:
                    csv = df.to_csv(index=False)
                    st.download_button("📥 下载CSV", csv, f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv")
        with col_export2:
            if st.button("📊 导出航点数据", use_container_width=True):
                if st.session_state.planned_path:
                    wp_data = [{"航点序号": i+1, "经度": wp[0], "纬度": wp[1]} for i, wp in enumerate(st.session_state.planned_path)]
                    wp_df = pd.DataFrame(wp_data)
                    st.download_button("📥 下载航点", wp_df.to_csv(index=False), f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv")
    else:
        st.info("⏳ 等待心跳数据... 请点击「开始飞行」按钮")
        
        # 显示航线预览
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            st.markdown("---")
            st.subheader("🗺️ 规划航线预览")
            total_waypoints = len(st.session_state.planned_path)
            st.success(f"📌 已规划 {total_waypoints} 个航点（包括起点和终点）")
            
            with st.expander("📋 查看详细航点列表"):
                waypoint_table = []
                for i, wp in enumerate(st.session_state.planned_path):
                    wp_type = "🚁 起点" if i == 0 else "🏁 终点" if i == len(st.session_state.planned_path)-1 else f"📍 绕行点 {i}"
                    waypoint_table.append({"序号": i+1, "类型": wp_type, "经度": f"{wp[0]:.6f}", "纬度": f"{wp[1]:.6f}"})
                st.dataframe(pd.DataFrame(waypoint_table), use_container_width=True)


def display_monitor_map(flight_alt: float, latest):
    monitor_map = folium.Map(location=[latest.lat, latest.lng], zoom_start=18, tiles=config.GAODE_SATELLITE_URL, attr="高德卫星地图")
    
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_alt else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=2, fill=True, fill_opacity=0.3,
                          popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(monitor_map)
    
    if st.session_state.planned_path:
        line_color = "purple" if "向左" in st.session_state.current_direction else "orange" if "向右" in st.session_state.current_direction else "green"
        folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color=line_color, weight=3, opacity=0.7).add_to(monitor_map)
    
    folium.Circle(radius=st.session_state.safety_radius, location=[latest.lat, latest.lng], color="blue", weight=2, fill=True, fill_opacity=0.2).add_to(monitor_map)
    
    trail = [[hb.lat, hb.lng] for hb in st.session_state.heartbeat_sim.history[:50] if hb.lat and hb.lng]
    if len(trail) > 1:
        folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(monitor_map)
    
    folium.Marker([latest.lat, latest.lng], popup=f"当前位置\n高度: {latest.altitude}m\n速度: {latest.speed}m/s",
                 icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
    
    if st.session_state.points_gcj['A']:
        folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], popup="起点 A",
                     icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
    if st.session_state.points_gcj['B']:
        folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], popup="终点 B",
                     icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(monitor_map)
    
    folium_static(monitor_map, width=900, height=500)


def update_flight_simulation():
    current_time = time.time()
    if st.session_state.simulation_running:
        if current_time - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            try:
                new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
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
    st.header("🚧 障碍物管理")
    
    col_data1, col_data2, col_data3, col_data4 = st.columns(4)
    with col_data1:
        if st.button("💾 保存配置", use_container_width=True, type="primary"):
            if save_obstacles(st.session_state.obstacles_gcj):
                st.success(f"✅ 已保存 {len(st.session_state.obstacles_gcj)} 个障碍物")
                st.rerun()
    with col_data2:
        if st.button("📂 加载配置", use_container_width=True):
            loaded = load_obstacles()
            if loaded:
                st.session_state.obstacles_gcj = loaded
                update_path_after_obstacle_change(flight_alt)
                st.rerun()
    with col_data3:
        if st.button("🗑️ 清除全部", use_container_width=True):
            st.session_state.obstacles_gcj = []
            save_obstacles([])
            update_path_after_obstacle_change(flight_alt)
            st.rerun()
    with col_data4:
        st.info(f"📊 共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    
    st.markdown("---")
    
    if st.session_state.obstacles_gcj:
        for idx, obs in enumerate(st.session_state.obstacles_gcj):
            with st.container(border=True):
                col1, col2, col3, col4 = st.columns([3, 2, 2, 1])
                with col1:
                    st.write(f"**{obs.get('name', f'障碍物{idx+1}')}**")
                with col2:
                    st.write(f"高度: {obs.get('height', 30)}m")
                with col3:
                    new_h = st.number_input("", value=obs.get('height', 30), min_value=1, max_value=200, step=5, key=f"h_{idx}", label_visibility="collapsed")
                    if new_h != obs.get('height', 30):
                        obs['height'] = new_h
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        update_path_after_obstacle_change(flight_alt)
                        st.rerun()
                with col4:
                    if st.button("🗑️", key=f"del_{idx}"):
                        st.session_state.obstacles_gcj.pop(idx)
                        save_obstacles(st.session_state.obstacles_gcj)
                        update_path_after_obstacle_change(flight_alt)
                        st.rerun()
    else:
        st.info("📭 暂无任何障碍物，请在航线规划页面绘制添加")


def update_path_after_obstacle_change(flight_alt: float):
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, flight_alt,
        st.session_state.current_direction, st.session_state.safety_radius)


# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    init_session_state()
    st.title("🏫 无人机地面站系统")
    st.markdown("---")
    
    page, drone_speed, flight_alt, auto_save = render_sidebar()
    st.session_state.auto_backup = auto_save
    st.session_state.drone_speed = drone_speed
    
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt,
                st.session_state.current_direction, st.session_state.safety_radius)
            st.rerun()
    
    if page == "🗺️ 航线规划":
        render_planning_page(drone_speed, flight_alt, auto_save)
    elif page == "📡 飞行监控":
        render_flight_monitoring_page(flight_alt, drone_speed)
    elif page == "🚧 障碍物管理":
        render_obstacle_management_page(flight_alt)


if __name__ == "__main__":
    main()
