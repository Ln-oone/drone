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

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)


# ==================== 基础几何函数 ====================
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


def segments_intersect(p1, p2, p3, p4) -> bool:
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


def line_intersects_polygon(p1, p2, polygon) -> bool:
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    for i in range(len(polygon)):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % len(polygon)]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False


def distance(p1, p2) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def meters_to_deg(meters: float, lat: float = 32.23):
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg


def get_polygon_bounds(polygon):
    if not polygon:
        return None
    return {
        'min_lng': min(p[0] for p in polygon),
        'max_lng': max(p[0] for p in polygon),
        'min_lat': min(p[1] for p in polygon),
        'max_lat': max(p[1] for p in polygon),
    }


def point_to_segment_distance_meters(point, seg_start, seg_end):
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    dx, dy = x2 - x1, y2 - y1
    
    if dx == 0 and dy == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2) * 111000
    
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    proj_x, proj_y = x1 + t * dx, y1 + t * dy
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2) * 111000


def check_safety_radius(drone_pos, obstacles, flight_alt, safety_radius):
    if not drone_pos:
        return True, None, None
    min_dist = float('inf')
    danger_name = None
    for obs in obstacles:
        if obs.get('height', 30) <= flight_alt:
            continue
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            for i in range(len(coords)):
                p1, p2 = coords[i], coords[(i + 1) % len(coords)]
                dist = point_to_segment_distance_meters(drone_pos, p1, p2)
                if dist < min_dist:
                    min_dist = dist
                    danger_name = obs.get('name', '障碍物')
    if min_dist < safety_radius:
        return False, min_dist, danger_name
    return True, min_dist if min_dist != float('inf') else None, None


# ==================== 核心避障算法 ====================
def get_blocking_obstacles(start, end, obstacles, flight_alt):
    """获取阻挡航线的障碍物"""
    blocking = []
    for obs in obstacles:
        if obs.get('height', 30) > flight_alt:
            polygon = obs.get('polygon', [])
            if polygon and line_intersects_polygon(start, end, polygon):
                blocking.append(obs)
    return blocking


def can_fly_direct(start, end, obstacles, flight_alt):
    return len(get_blocking_obstacles(start, end, obstacles, flight_alt)) == 0


def get_combined_obstacle_bounds(obstacles):
    """合并多个障碍物的边界"""
    if not obstacles:
        return None
    all_bounds = [get_polygon_bounds(obs['polygon']) for obs in obstacles if obs.get('polygon')]
    all_bounds = [b for b in all_bounds if b]
    if not all_bounds:
        return None
    return {
        'min_lng': min(b['min_lng'] for b in all_bounds),
        'max_lng': max(b['max_lng'] for b in all_bounds),
        'min_lat': min(b['min_lat'] for b in all_bounds),
        'max_lat': max(b['max_lat'] for b in all_bounds),
    }


def find_left_path(start, end, obstacles, flight_alt, safety_radius):
    """向左绕行：从障碍物左侧绕过"""
    if can_fly_direct(start, end, obstacles, flight_alt):
        return [start, end]
    
    blocking = get_blocking_obstacles(start, end, obstacles, flight_alt)
    if not blocking:
        return [start, end]
    
    bounds = get_combined_obstacle_bounds(blocking)
    if not bounds:
        return [start, end]
    
    lng_off, lat_off = meters_to_deg(safety_radius * 2, start[1])
    
    # 绕行点经度 = 障碍物最左侧 - 安全距离
    waypoint_lng = bounds['min_lng'] - lng_off
    
    # 确保路径向前（不后退）- 比较起点和终点的经度
    if end[0] < start[0]:  # 终点在左边，不需要绕行
        return [start, end]
    
    # 生成绕行点：先水平向左，再垂直移动，最后水平向右到终点
    waypoint1 = [waypoint_lng, start[1]]
    waypoint2 = [waypoint_lng, end[1]]
    
    return [start, waypoint1, waypoint2, end]


def find_right_path(start, end, obstacles, flight_alt, safety_radius):
    """向右绕行：从障碍物右侧绕过"""
    if can_fly_direct(start, end, obstacles, flight_alt):
        return [start, end]
    
    blocking = get_blocking_obstacles(start, end, obstacles, flight_alt)
    if not blocking:
        return [start, end]
    
    bounds = get_combined_obstacle_bounds(blocking)
    if not bounds:
        return [start, end]
    
    lng_off, lat_off = meters_to_deg(safety_radius * 2, start[1])
    
    # 绕行点经度 = 障碍物最右侧 + 安全距离
    waypoint_lng = bounds['max_lng'] + lng_off
    
    # 确保路径向前（不后退）
    if end[0] > start[0]:  # 终点在右边，不需要绕行
        return [start, end]
    
    waypoint1 = [waypoint_lng, start[1]]
    waypoint2 = [waypoint_lng, end[1]]
    
    return [start, waypoint1, waypoint2, end]


def find_best_path(start, end, obstacles, flight_alt, safety_radius):
    """最佳航线：比较左右路径长度，选择较短的"""
    if can_fly_direct(start, end, obstacles, flight_alt):
        return [start, end]
    
    left_path = find_left_path(start, end, obstacles, flight_alt, safety_radius)
    right_path = find_right_path(start, end, obstacles, flight_alt, safety_radius)
    
    def path_length(path):
        return sum(distance(path[i], path[i+1]) for i in range(len(path)-1))
    
    left_len = path_length(left_path)
    right_len = path_length(right_path)
    
    st.session_state.path_lengths = {'left': left_len * 111000, 'right': right_len * 111000}
    
    return left_path if left_len <= right_len else right_path


def create_avoidance_path(start, end, obstacles, flight_alt, direction, safety_radius):
    """创建避障路径"""
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles, flight_alt, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles, flight_alt, safety_radius)
    else:
        return find_best_path(start, end, obstacles, flight_alt, safety_radius)


# ==================== 障碍物管理 ====================
def load_obstacles():
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles = data.get('obstacles', [])
                for obs in obstacles:
                    obs.setdefault('selected', False)
                    obs.setdefault('height', 30)
                return obstacles
        except:
            return []
    return []


def save_obstacles(obstacles):
    try:
        data = {'obstacles': obstacles, 'count': len(obstacles), 
                'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except:
        return False


def validate_polygon(polygon):
    return len(polygon) >= 3


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
    def __init__(self, start_point):
        self.history = []
        self.current_pos = start_point.copy()
        self.path = [start_point.copy()]
        self.path_index = 0
        self.simulating = False
        self.flight_altitude = 50
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.safety_radius = config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        self.start_time = None
        self.flight_log = []
        self.last_update_time = None
        
    def set_path(self, path, altitude=50, speed=50, safety_radius=5):
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
        self.total_distance = sum(distance(self.path[i], self.path[i+1]) for i in range(len(self.path)-1))
    
    def update_and_generate(self, obstacles):
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
        seg_dist = distance(start, end)
        
        move_dist = config.BASE_SPEED_MPS * (self.speed / 100) * delta_time
        self.distance_traveled += move_dist
        
        if self.total_distance > 0:
            completed = sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_index))
            if seg_dist > 0:
                completed += seg_dist * min(1.0, self.distance_traveled / seg_dist)
            self.progress = min(1.0, completed / self.total_distance)
        
        if self.distance_traveled >= seg_dist and seg_dist > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._gen_heartbeat(True)
        else:
            if seg_dist > 0:
                t = min(1.0, self.distance_traveled / seg_dist)
                self.current_pos = [start[0] + (end[0]-start[0])*t, start[1] + (end[1]-start[1])*t]
        
        safe, _, _ = check_safety_radius(self.current_pos, obstacles, self.flight_altitude, self.safety_radius)
        if not safe:
            self.safety_violation = True
        
        return self._gen_heartbeat(False)
    
    def _gen_heartbeat(self, arrived):
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        if arrived:
            remaining = 0
        else:
            remaining = sum(distance(self.current_pos, self.path[self.path_index+1]) if self.path_index < len(self.path)-1 else 0,
                           *[distance(self.path[i], self.path[i+1]) for i in range(self.path_index+1, len(self.path)-1)]) * 111000
        
        hb = HeartbeatData(
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
            remaining_distance=remaining
        )
        self.history.insert(0, hb)
        if len(self.history) > 100:
            self.history.pop()
        self.flight_log.append(hb)
        return hb
    
    def export_flight_data(self):
        if not self.flight_log:
            return pd.DataFrame()
        return pd.DataFrame([{
            'timestamp': h.timestamp, 'flight_time': h.flight_time, 'lat': h.lat, 'lng': h.lng,
            'altitude': h.altitude, 'voltage': h.voltage, 'satellites': h.satellites,
            'speed': h.speed, 'progress': h.progress, 'arrived': h.arrived,
            'safety_violation': h.safety_violation, 'remaining_distance': h.remaining_distance
        } for h in self.flight_log])


# ==================== 地图创建 ====================
def create_map(center, points, obstacles, planned_path, map_type, flight_alt, drone_pos, direction, safety_radius):
    tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
    m = folium.Map(location=[center[1], center[0]], zoom_start=16, tiles=tiles, attr="高德地图")
    
    draw = plugins.Draw(export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4}},
        edit_options={'edit': True, 'remove': True})
    m.add_child(draw)
    
    # 障碍物
    for obs in obstacles:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            color = "darkred" if obs.get('height', 30) > flight_alt else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.5,
                          popup=f"{obs.get('name')}\n高度: {obs.get('height', 30)}m").add_to(m)
            # 安全缓冲区
            if obs.get('height', 30) > flight_alt:
                bounds = get_polygon_bounds(coords)
                if bounds:
                    lo, la = meters_to_deg(safety_radius, center[1])
                    buf = [[bounds['min_lng']-lo, bounds['min_lat']-la], [bounds['max_lng']+lo, bounds['min_lat']-la],
                           [bounds['max_lng']+lo, bounds['max_lat']+la], [bounds['min_lng']-lo, bounds['max_lat']+la]]
                    folium.Polygon([[c[1], c[0]] for c in buf], color="yellow", weight=1, fill=True, fill_color="yellow", fill_opacity=0.15).add_to(m)
    
    # 起点终点
    if points.get('A'):
        folium.Marker([points['A'][1], points['A'][0]], popup="起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points.get('B'):
        folium.Marker([points['B'][1], points['B'][0]], popup="终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    # 规划路径
    if planned_path and len(planned_path) > 1:
        color = {"向左绕行": "purple", "向右绕行": "orange", "最佳航线": "green"}.get(direction, "green")
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color=color, weight=5, opacity=0.9).add_to(m)
        for p in planned_path[1:-1]:
            folium.CircleMarker([p[1], p[0]], radius=4, color=color, fill=True, fill_color="white").add_to(m)
    
    # 直线航线
    if points.get('A') and points.get('B'):
        blocked = any(line_intersects_polygon(points['A'], points['B'], obs['polygon']) 
                     for obs in obstacles if obs.get('height', 30) > flight_alt and obs.get('polygon'))
        color = "gray" if blocked else "blue"
        folium.PolyLine([[points['A'][1], points['A'][0]], [points['B'][1], points['B'][0]]], color=color, weight=2, dash_array='5,5').add_to(m)
    
    if drone_pos:
        folium.Circle([drone_pos[1], drone_pos[0]], radius=safety_radius, color="blue", weight=2, fill=True, fill_opacity=0.2).add_to(m)
    
    return m


# ==================== UI组件 ====================
def init_state():
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(),
        'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'last_hb_time': time.time(),
        'simulation_running': False,
        'flight_history': [],
        'planned_path': None,
        'last_flight_alt': 50,
        'pending_obstacle': None,
        'current_direction': "最佳航线",
        'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True,
        'waiting_for_start': False,
        'waiting_for_end': False,
        'path_lengths': {}
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v
    for obs in st.session_state.obstacles_gcj:
        obs.setdefault('height', 30)
        obs.setdefault('selected', False)


def render_sidebar():
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type = "satellite" if st.sidebar.radio("地图类型", ["卫星影像", "矢量街道"]) == "卫星影像" else "vector"
    drone_speed = st.sidebar.slider("飞行速度系数", 10, 100, 50, 5)
    flight_alt = st.sidebar.slider("飞行高度 (m)", 10, 200, 50, 5)
    safety_radius = st.sidebar.slider("安全半径 (米)", 1, 30, st.session_state.safety_radius, 1)
    auto_save = st.sidebar.checkbox("自动保存", st.session_state.auto_backup)
    return page, map_type, drone_speed, flight_alt, auto_save, safety_radius


def render_planning_page(map_type, drone_speed, flight_alt, auto_save, safety_radius):
    st.header("🗺️ 航线规划")
    
    # 更新安全半径
    if safety_radius != st.session_state.safety_radius:
        st.session_state.safety_radius = safety_radius
        if st.session_state.planned_path:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
    
    # 检查直线是否被阻挡
    blocked = any(line_intersects_polygon(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], obs['polygon'])
                 for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt and obs.get('polygon'))
    
    if blocked:
        st.warning(f"⚠️ 有障碍物高于{flight_alt}m，需要绕行")
    else:
        st.success("✅ 直线航线畅通")
    
    st.info(f"🛡️ 安全半径: {safety_radius}米 | 点击地图左上角📐绘制多边形添加障碍物")
    
    col1, col2 = st.columns([1, 1.5])
    with col1:
        render_controls(flight_alt, drone_speed)
    with col2:
        render_map_view(map_type, flight_alt, blocked)


def render_controls(flight_alt, drone_speed):
    st.subheader("🎮 控制面板")
    
    with st.expander("📍 起点/终点", expanded=True):
        mode = st.radio("设置方式", ["经纬度输入", "鼠标点击"], horizontal=True)
        if mode == "经纬度输入":
            col1, col2 = st.columns(2)
            with col1:
                a_lat = st.number_input("起点纬度", value=st.session_state.points_gcj['A'][1], format="%.6f")
                a_lng = st.number_input("起点经度", value=st.session_state.points_gcj['A'][0], format="%.6f")
                if st.button("设置A点"):
                    st.session_state.points_gcj['A'] = [a_lng, a_lat]
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
                    st.rerun()
            with col2:
                b_lat = st.number_input("终点纬度", value=st.session_state.points_gcj['B'][1], format="%.6f")
                b_lng = st.number_input("终点经度", value=st.session_state.points_gcj['B'][0], format="%.6f")
                if st.button("设置B点"):
                    st.session_state.points_gcj['B'] = [b_lng, b_lat]
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
                    st.rerun()
        else:
            col1, col2 = st.columns(2)
            with col1:
                if st.button("🎯 点击地图设起点"):
                    st.session_state.waiting_for_start = True
                    st.session_state.waiting_for_end = False
            with col2:
                if st.button("📍 点击地图设终点"):
                    st.session_state.waiting_for_end = True
                    st.session_state.waiting_for_start = False
            if st.session_state.waiting_for_start:
                st.info("请在地图上点击选择起点")
            elif st.session_state.waiting_for_end:
                st.info("请在地图上点击选择终点")
            if st.button("重置默认"):
                st.session_state.points_gcj = {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()}
                st.session_state.waiting_for_start = st.session_state.waiting_for_end = False
                st.rerun()
    
    with st.expander("🤖 绕行策略", expanded=True):
        col1, col2, col3 = st.columns(3)
        with col1:
            if st.button("最佳航线", use_container_width=True, type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"):
                st.session_state.current_direction = "最佳航线"
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, "最佳航线", st.session_state.safety_radius)
                st.rerun()
        with col2:
            if st.button("向左绕行", use_container_width=True):
                st.session_state.current_direction = "向左绕行"
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, "向左绕行", st.session_state.safety_radius)
                st.rerun()
        with col3:
            if st.button("向右绕行", use_container_width=True):
                st.session_state.current_direction = "向右绕行"
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, "向右绕行", st.session_state.safety_radius)
                st.rerun()
        
        if st.session_state.path_lengths:
            st.caption(f"📊 路径长度: 左{st.session_state.path_lengths.get('left', 0):.0f}m | 右{st.session_state.path_lengths.get('right', 0):.0f}m")
    
    with st.expander("✈️ 飞行控制", expanded=True):
        if st.session_state.planned_path:
            total = sum(distance(st.session_state.planned_path[i], st.session_state.planned_path[i+1]) for i in range(len(st.session_state.planned_path)-1)) * 111000
            st.metric("路径长度", f"{total:.0f}m")
            st.metric("绕行点", len(st.session_state.planned_path)-2)
        
        col1, col2 = st.columns(2)
        with col1:
            if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                st.rerun()
        with col2:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
    
    st.write(f"🟢 A: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
    st.write(f"🔴 B: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")


def render_map_view(map_type, flight_alt, blocked):
    st.subheader("🗺️ 规划地图")
    
    if st.session_state.planned_path is None:
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
    
    drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
    trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
    
    m = create_map(
        st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ,
        st.session_state.points_gcj, st.session_state.obstacles_gcj,
        st.session_state.planned_path, map_type, flight_alt, drone_pos,
        st.session_state.current_direction, st.session_state.safety_radius
    )
    
    output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
    
    # 处理鼠标点击
    if output and output.get("last_clicked"):
        click = output["last_clicked"]
        if click:
            lng, lat = click.get("lng"), click.get("lat")
            if lng and lat:
                if st.session_state.waiting_for_start:
                    st.session_state.points_gcj['A'] = [lng, lat]
                    st.session_state.waiting_for_start = False
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
                    st.rerun()
                elif st.session_state.waiting_for_end:
                    st.session_state.points_gcj['B'] = [lng, lat]
                    st.session_state.waiting_for_end = False
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
                    st.rerun()
    
    # 处理绘制多边形
    if output and output.get("last_active_drawing"):
        draw = output["last_active_drawing"]
        if draw and draw.get("geometry") and draw["geometry"].get("type") == "Polygon":
            coords = draw["geometry"].get("coordinates", [])
            if coords and len(coords[0]) >= 3:
                poly = [[p[0], p[1]] for p in coords[0]]
                if st.session_state.pending_obstacle is None:
                    st.session_state.pending_obstacle = poly
                    st.rerun()
    
    if st.session_state.pending_obstacle:
        st.markdown("---")
        st.subheader("📝 添加障碍物")
        col1, col2 = st.columns(2)
        with col1:
            name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
        with col2:
            height = st.number_input("高度(m)", 1, 200, 30, 5)
        if st.button("确认添加"):
            st.session_state.obstacles_gcj.append({
                "name": name, "polygon": st.session_state.pending_obstacle, "height": height, "selected": False
            })
            if st.session_state.auto_backup:
                save_obstacles(st.session_state.obstacles_gcj)
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
            st.session_state.pending_obstacle = None
            st.rerun()
        if st.button("取消"):
            st.session_state.pending_obstacle = None
            st.rerun()


def render_monitoring_page(map_type, flight_alt, drone_speed):
    st.header("📡 飞行监控")
    
    # 更新模拟
    if st.session_state.simulation_running:
        if time.time() - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
            if hb:
                st.session_state.last_hb_time = time.time()
                st.session_state.flight_history.append([hb.lng, hb.lat])
                if not st.session_state.heartbeat_sim.simulating:
                    st.session_state.simulation_running = False
                    st.success("🏁 已到达目的地！")
                st.rerun()
    
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        
        # 进度条
        st.progress(latest.progress if not latest.arrived else 1.0, text=f"飞行进度: {int(latest.progress*100)}%")
        
        # 数据卡片
        col1, col2, col3, col4 = st.columns(4)
        with col1:
            st.metric("速度", f"{latest.speed:.1f} m/s", f"{drone_speed}%")
        with col2:
            st.metric("高度", f"{latest.altitude} m")
        with col3:
            st.metric("剩余距离", f"{max(0, latest.remaining_distance):.0f} m")
        with col4:
            st.metric("卫星", f"{latest.satellites} 颗")
        
        col5, col6, col7, col8 = st.columns(4)
        with col5:
            st.metric("电压", f"{latest.voltage:.1f} V")
        with col6:
            st.metric("已用时间", f"{int(latest.flight_time//60):02d}:{int(latest.flight_time%60):02d}")
        with col7:
            progress = int(latest.progress * 100)
            st.metric("进度", f"{progress}%")
        with col8:
            status = "✅完成" if latest.arrived else ("✈️飞行" if st.session_state.simulation_running else "⏸️停止")
            st.metric("状态", status)
        
        if latest.safety_violation and not latest.arrived:
            st.error("⚠️ 安全距离警告！")
        
        # 地图
        m = folium.Map(location=[latest.lat, latest.lng], zoom_start=18)
        for obs in st.session_state.obstacles_gcj:
            if obs.get('polygon'):
                folium.Polygon([[c[1], c[0]] for c in obs['polygon']], color="red", fill=True, fill_opacity=0.3).add_to(m)
        if st.session_state.planned_path:
            folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color="green", weight=3).add_to(m)
        folium.Marker([latest.lat, latest.lng], popup="当前位置", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
        folium_static(m, width=900, height=400)
        
        # 导出
        if st.button("导出飞行数据"):
            df = st.session_state.heartbeat_sim.export_flight_data()
            if not df.empty:
                st.download_button("下载CSV", df.to_csv(index=False), "flight_data.csv", "text/csv")
    else:
        st.info("请先在航线规划页面开始飞行")


def render_obstacle_page(flight_alt):
    st.header("🚧 障碍物管理")
    
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.info(f"📊 共 {len(st.session_state.obstacles_gcj)} 个")
    with col2:
        high = sum(1 for o in st.session_state.obstacles_gcj if o.get('height', 30) > flight_alt)
        st.info(f"🔴 需避让: {high}")
    with col3:
        st.info(f"🛡️ 安全半径: {st.session_state.safety_radius}m")
    with col4:
        if st.button("💾 保存"):
            save_obstacles(st.session_state.obstacles_gcj)
            st.success("已保存")
    
    # 列表
    for i, obs in enumerate(st.session_state.obstacles_gcj):
        with st.container(border=True):
            col1, col2, col3, col4 = st.columns([2, 2, 1, 1])
            with col1:
                name = st.text_input("名称", obs.get('name', f'障碍物{i+1}'), key=f"name_{i}")
                obs['name'] = name
            with col2:
                height = st.number_input("高度(m)", 1, 200, obs.get('height', 30), 5, key=f"height_{i}")
                obs['height'] = height
            with col3:
                st.caption(f"顶点: {len(obs.get('polygon', []))}")
            with col4:
                if st.button("🗑️", key=f"del_{i}"):
                    st.session_state.obstacles_gcj.pop(i)
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
                    st.rerun()
    
    if st.button("🗑️ 清除全部"):
        st.session_state.obstacles_gcj = []
        save_obstacles([])
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            [], flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
        st.rerun()


# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="无人机地面站", layout="wide")
    init_state()
    st.title("🏫 无人机地面站系统")
    st.markdown("---")
    
    page, map_type, drone_speed, flight_alt, auto_save, safety_radius = render_sidebar()
    st.session_state.auto_backup = auto_save
    
    if flight_alt != st.session_state.last_flight_alt:
        st.session_state.last_flight_alt = flight_alt
        if st.session_state.planned_path:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
            st.rerun()
    
    if page == "🗺️ 航线规划":
        render_planning_page(map_type, drone_speed, flight_alt, auto_save, safety_radius)
    elif page == "📡 飞行监控":
        render_monitoring_page(map_type, flight_alt, drone_speed)
    else:
        render_obstacle_page(flight_alt)


if __name__ == "__main__":
    main()
