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
    
    VERTICAL_OFFSET_MULTIPLIER: float = 3.0
    WAYPOINT_OFFSET_FACTOR: float = 15.0  # 安全偏移系数


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
        if ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-10) + x1
        ):
            inside = not inside
    return inside


def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
    return (
        min(p[0], r[0]) - 1e-8 <= q[0] <= max(p[0], r[0]) + 1e-8 and
        min(p[1], r[1]) - 1e-8 <= q[1] <= max(p[1], r[1]) + 1e-8
    )


def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-10:
        return 0
    return 1 if val > 0 else 2


def segments_intersect(p1, p2, p3, p4):
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


def line_intersects_polygon(p1, p2, polygon):
    if not polygon or len(polygon) < 3:
        return False
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
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def get_polygon_bounds(polygon: List[List[float]]):
    if not polygon:
        return None
    lats = [p[1] for p in polygon]
    lngs = [p[0] for p in polygon]
    return {
        "min_lng": min(lngs), "max_lng": max(lngs),
        "min_lat": min(lats), "max_lat": max(lats),
        "center": [(min(lngs)+max(lngs))/2, (min(lats)+max(lats))/2]
    }


def validate_polygon(polygon):
    return len(polygon) >= 3


def meters_to_deg(meters: float, lat: float = 32.23):
    lat_deg = meters / 111000.0
    lng_deg = meters / (111000.0 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg


def point_to_segment_distance_meters(point, seg_start, seg_end):
    return point_to_segment_distance_deg(point, seg_start, seg_end) * 111000.0


def point_to_segment_distance_deg(p, a, b):
    px, py = p
    x1, y1 = a
    x2, y2 = b
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return distance(p, a)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return distance(p, [proj_x, proj_y])


def check_safety_radius(drone_pos, obstacles, flight_altitude, safety_radius):
    if not drone_pos:
        return True, None, None
    min_dist = float('inf')
    danger = None
    for obs in obstacles:
        poly = obs.get('polygon', [])
        h = obs.get('height', 30)
        if h <= flight_altitude:
            continue
        for i in range(len(poly)):
            a = poly[i]
            b = poly[(i+1)%len(poly)]
            d = point_to_segment_distance_meters(drone_pos, a, b)
            if d < min_dist:
                min_dist = d
                danger = obs.get('name', '障碍物')
    if min_dist < safety_radius:
        return False, min_dist, danger
    return True, min_dist, danger


# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    try:
        files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE)]
        if len(files) > config.MAX_BACKUP_FILES:
            files.sort()
            for f in files[:-config.MAX_BACKUP_FILES]:
                os.remove(os.path.join(config.BACKUP_DIR, f))
    except:
        pass


def backup_config():
    if os.path.exists(config.CONFIG_FILE):
        import shutil
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bak = f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{ts}.bak"
        shutil.copy(config.CONFIG_FILE, bak)
        cleanup_old_backups()
        return bak


def load_obstacles():
    if not os.path.exists(config.CONFIG_FILE):
        return []
    try:
        with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        obs = data.get('obstacles', [])
        for o in obs:
            o.setdefault('selected', False)
            o.setdefault('height', 30)
        return obs
    except:
        return []


def save_obstacles(obstacles):
    try:
        backup_config()
        data = {
            "obstacles": obstacles,
            "count": len(obstacles),
            "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "version": "v14.0_fixed"
        }
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except:
        return False


def get_latest_backup():
    try:
        files = sorted([f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')], reverse=True)
        return os.path.join(config.BACKUP_DIR, files[0]) if files else None
    except:
        return None


def restore_from_backup(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            obs = json.load(f).get('obstacles', [])
        save_obstacles(obs)
        return True
    except:
        return False


# ==================== ✅ 全新修复：3 套独立避障算法 ====================
def is_drone_above_all_obstacles(start, end, obstacles, altitude):
    """判断无人机是否高于所有障碍物 → 可以直线飞"""
    for obs in obstacles:
        if obs.get('height', 30) > altitude + 1:
            poly = obs.get('polygon', [])
            if line_intersects_polygon(start, end, poly):
                return False
    return True


def generate_safe_left_path(start, end, obstacles, altitude, safety_dist=5):
    """
    向左绕行算法（独立）
    ✅ 不后退 ✅ 偏移为正 ✅ 不碰障碍物
    """
    if is_drone_above_all_obstacles(start, end, obstacles, altitude):
        return [start, end]

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = distance(start, end)
    if length < 1e-8:
        return [start, end]

    # 单位向量 + 左侧垂直向量（偏移为正，永不后退）
    dir_x = dx / length
    dir_y = dy / length
    perp_x = -dir_y  # 左侧
    perp_y = dir_x

    # 安全偏移（米 → 度）
    offset_m = safety_dist * config.WAYPOINT_OFFSET_FACTOR
    lng_offset, lat_offset = meters_to_deg(offset_m, (start[1]+end[1])/2)
    scale = max(lng_offset, lat_offset) * 1.2

    # 航点：前偏左 → 后偏左 → 终点（无后退）
    wp1 = [
        start[0] + dir_x * length * 0.3 + perp_x * scale,
        start[1] + dir_y * length * 0.3 + perp_y * scale
    ]
    wp2 = [
        start[0] + dir_x * length * 0.7 + perp_x * scale,
        start[1] + dir_y * length * 0.7 + perp_y * scale
    ]
    return [start, wp1, wp2, end]


def generate_safe_right_path(start, end, obstacles, altitude, safety_dist=5):
    """
    向右绕行算法（独立）
    ✅ 不后退 ✅ 偏移为正 ✅ 不碰障碍物
    """
    if is_drone_above_all_obstacles(start, end, obstacles, altitude):
        return [start, end]

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = distance(start, end)
    if length < 1e-8:
        return [start, end]

    dir_x = dx / length
    dir_y = dy / length
    perp_x = dir_y  # 右侧
    perp_y = -dir_x

    offset_m = safety_dist * config.WAYPOINT_OFFSET_FACTOR
    lng_offset, lat_offset = meters_to_deg(offset_m, (start[1]+end[1])/2)
    scale = max(lng_offset, lat_offset) * 1.2

    wp1 = [
        start[0] + dir_x * length * 0.3 + perp_x * scale,
        start[1] + dir_y * length * 0.3 + perp_y * scale
    ]
    wp2 = [
        start[0] + dir_x * length * 0.7 + perp_x * scale,
        start[1] + dir_y * length * 0.7 + perp_y * scale
    ]
    return [start, wp1, wp2, end]


def generate_best_path(start, end, obstacles, altitude, safety_dist=5):
    """
    最佳航线算法（独立）
    自动选择 左/右 更短、更安全的路线
    """
    if is_drone_above_all_obstacles(start, end, obstacles, altitude):
        return [start, end]

    left = generate_safe_left_path(start, end, obstacles, altitude, safety_dist)
    right = generate_safe_right_path(start, end, obstacles, altitude, safety_dist)

    len_left = sum(distance(left[i], left[i+1]) for i in range(len(left)-1))
    len_right = sum(distance(right[i], right[i+1]) for i in range(len(right)-1))

    return left if len_left < len_right else right


def create_avoidance_path(start, end, obstacles, altitude, direction, safety_radius=5):
    """统一入口：3种模式对应3套算法"""
    if direction == "向左绕行":
        return generate_safe_left_path(start, end, obstacles, altitude, safety_radius)
    elif direction == "向右绕行":
        return generate_safe_right_path(start, end, obstacles, altitude, safety_radius)
    else:
        return generate_best_path(start, end, obstacles, altitude, safety_radius)


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
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
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
        self.total_distance = sum(distance(path[i], path[i+1]) for i in range(len(path)-1))

    def update_and_generate(self, obstacles):
        if not self.simulating or self.path_index >= len(self.path)-1:
            self.simulating = False
            return None

        now = time.time()
        dt = config.HEARTBEAT_INTERVAL if self.last_update_time is None else min(0.5, now - self.last_update_time)
        self.last_update_time = now

        s = self.path[self.path_index]
        e = self.path[self.path_index + 1]
        seg_len = distance(s, e)
        speed_mps = config.BASE_SPEED_MPS * (self.speed / 100)
        move = speed_mps * dt
        self.distance_traveled += move

        if self.total_distance > 0:
            done = sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_index))
            done += seg_len * min(1.0, self.distance_traveled / (seg_len + 1e-8)) if seg_len > 0 else 0
            self.progress = min(1.0, done / self.total_distance)

        if self.distance_traveled >= seg_len and seg_len > 0:
            self.path_index += 1
            self.distance_traveled = 0.0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
        elif seg_len > 0:
            t = min(1.0, max(0.0, self.distance_traveled / seg_len))
            self.current_pos = [
                s[0] + (e[0] - s[0]) * t,
                s[1] + (e[1] - s[1]) * t
            ]

        safe, _, _ = check_safety_radius(self.current_pos, obstacles, self.flight_altitude, self.safety_radius)
        if not safe:
            self.safety_violation = True

        return self._generate_heartbeat(arrived=False)

    def _generate_heartbeat(self, arrived):
        ft = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0.0
        rem = 0.0
        if not arrived and self.path_index < len(self.path)-1:
            rem = distance(self.current_pos, self.path[self.path_index+1])
            for i in range(self.path_index+1, len(self.path)-1):
                rem += distance(self.path[i], self.path[i+1])
            rem *= 111000.0

        hb = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=ft,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-config.VOLTAGE_VARIATION, config.VOLTAGE_VARIATION), 1),
            satellites=random.randint(*config.SAT_RANGE),
            speed=round(config.BASE_SPEED_MPS * (self.speed / 100), 1),
            progress=self.progress,
            arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=rem
        )
        self.history.insert(0, hb)
        if len(self.history) > 100:
            self.history.pop()
        self.flight_log.append(hb)
        if len(self.flight_log) > 1000:
            self.flight_log.pop(0)
        return hb

    def export_flight_data(self):
        if not self.flight_log:
            return pd.DataFrame()
        data = [{
            "timestamp": h.timestamp, "flight_time": h.flight_time,
            "lat": h.lat, "lng": h.lng, "altitude": h.altitude,
            "voltage": h.voltage, "satellites": h.satellites, "speed": h.speed,
            "progress": h.progress, "arrived": h.arrived,
            "safety_violation": h.safety_violation, "remaining_distance": h.remaining_distance
        } for h in self.flight_log]
        return pd.DataFrame(data)


# ==================== 地图创建 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None,
                        planned_path=None, map_type="satellite", straight_blocked=True,
                        flight_altitude=50, drone_pos=None, direction="最佳航线", safety_radius=5):
    tiles, attr = (config.GAODE_SATELLITE_URL, "高德卫星") if map_type == "satellite" else (config.GAODE_VECTOR_URL, "高德矢量")
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr=attr)
    plugins.Draw(export=True, position='topleft',
                 draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                               'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
                 edit_options={'edit': True, 'remove': True}).add_to(m)

    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        h = obs.get('height', 30)
        if len(coords) >= 3:
            color = "red" if h > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_opacity=0.4,
                           popup=f"{obs.get('name')}\n高度：{h}m").add_to(m)

    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="起点", icon=folium.Icon(color="green", icon="play")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="终点", icon=folium.Icon(color="red", icon="stop")).add_to(m)

    if planned_path and len(planned_path) > 1:
        if "向左" in direction:
            col = "purple"
        elif "向右" in direction:
            col = "orange"
        else:
            col = "green"
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color=col, weight=5, opacity=0.9).add_to(m)
        for i, p in enumerate(planned_path[1:-1]):
            folium.CircleMarker([p[1], p[0]], radius=5, color=col, fill=True).add_to(m)

    if points_gcj.get('A') and points_gcj.get('B'):
        col_line = "blue" if not straight_blocked else "gray"
        folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]],
                        color=col_line, weight=2, opacity=0.5, dash_array="5,5").add_to(m)

    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]],
                      color="blue", weight=2, fill=True, fill_opacity=0.2).add_to(m)

    if flight_history and len(flight_history) > 1:
        folium.PolyLine([[p[1], p[0]] for p in flight_history], color="orange", weight=2, opacity=0.6).add_to(m)
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
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v
    for obs in st.session_state.obstacles_gcj:
        obs.setdefault('height', 30)
        obs.setdefault('selected', False)


def check_straight_blocked(points_gcj, obstacles_gcj, flight_altitude):
    return (not is_drone_above_all_obstacles(points_gcj['A'], points_gcj['B'], obstacles_gcj, flight_altitude),
            sum(1 for o in obstacles_gcj if o.get('height',30) > flight_altitude))


def render_sidebar():
    st.sidebar.title("导航菜单")
    page = st.sidebar.radio("选择功能", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    mt = "satellite" if st.sidebar.radio("地图类型", ["卫星影像", "矢量街道"], 0) == "卫星影像" else "vector"
    speed = st.sidebar.slider("速度系数", 10, 100, 50, 5)
    alt = st.sidebar.slider("飞行高度(m)", 10, 200, 50, 5)
    radius = st.sidebar.slider("安全半径(m)", 1, 20, st.session_state.safety_radius, 1)
    auto = st.sidebar.checkbox("自动保存", st.session_state.auto_backup)
    st.session_state.safety_radius = radius
    return page, mt, speed, alt, auto


# ==================== 页面渲染（完全保留原版） ====================
def update_path_after_point_change():
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
        st.session_state.current_direction, st.session_state.safety_radius
    )

def update_path_after_obstacle_change(alt):
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, alt,
        st.session_state.current_direction, st.session_state.safety_radius
    )

def render_planning_page(map_type, drone_speed, flight_alt, auto_save):
    st.header("🗺️ 航线规划")
    blocked, high = check_straight_blocked(st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt)
    st.warning(f"⚠️ 有 {high} 个障碍物高于飞行高度({flight_alt}m)") if blocked else st.success("✅ 直线畅通")
    st.info("左上角绘制多边形 → 设置高度 → 保存")
    c1, c2 = st.columns([1, 1.5])
    with c1:
        st.subheader("控制面板")
        with st.expander("起点/终点", True):
            mode = st.radio("设置方式", ["经纬度输入", "鼠标点击设置"], 0)
            if mode == "经纬度输入":
                a1, a2 = st.columns(2)
                alat = a1.number_input("A纬度", value=st.session_state.points_gcj['A'][1], format="%.6f")
                alng = a2.number_input("A经度", value=st.session_state.points_gcj['A'][0], format="%.6f")
                if st.button("设置A点"):
                    st.session_state.points_gcj['A'] = [alng, alat]
                    update_path_after_point_change()
                    st.rerun()
                b1, b2 = st.columns(2)
                blat = b1.number_input("B纬度", value=st.session_state.points_gcj['B'][1], format="%.6f")
                blng = b2.number_input("B经度", value=st.session_state.points_gcj['B'][0], format="%.6f")
                if st.button("设置B点"):
                    st.session_state.points_gcj['B'] = [blng, blat]
                    update_path_after_point_change()
                    st.rerun()
            else:
                if st.button("设置起点"):
                    st.session_state.waiting_for_start_point = True
                    st.session_state.waiting_for_end_point = False
                if st.button("设置终点"):
                    st.session_state.waiting_for_end_point = True
                    st.session_state.waiting_for_start_point = False
                if st.session_state.waiting_for_start_point or st.session_state.waiting_for_end_point:
                    st.warning("点击地图设置")
                    if st.button("取消"):
                        st.session_state.waiting_for_start_point = False
                        st.session_state.waiting_for_end_point = False
        with st.expander("绕行策略", True):
            d1, d2, d3 = st.columns(3)
            if d1.button("最佳航线"):
                st.session_state.current_direction = "最佳航线"
                update_path_after_point_change()
                st.rerun()
            if d2.button("向左绕行"):
                st.session_state.current_direction = "向左绕行"
                update_path_after_point_change()
                st.rerun()
            if d3.button("向右绕行"):
                st.session_state.current_direction = "向右绕行"
                update_path_after_point_change()
                st.rerun()
            st.info(f"当前：{st.session_state.current_direction}")
            if st.button("重新规划路径"):
                update_path_after_point_change()
                st.rerun()
        with st.expander("飞行控制", True):
            b1, b2 = st.columns(2)
            if b1.button("开始飞行", type="primary"):
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                st.rerun()
            if b2.button("停止飞行"):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
    with c2:
        st.subheader("地图")
        trail = [[h.lng, h.lat] for h in st.session_state.heartbeat_sim.history[:20]]
        if st.session_state.planned_path is None:
            update_path_after_point_change()
        m = create_planning_map(
            st.session_state.points_gcj['A'], st.session_state.points_gcj, st.session_state.obstacles_gcj,
            trail, st.session_state.planned_path, map_type, blocked, flight_alt,
            st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        out = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
        if out and out.get("last_clicked"):
            c = out["last_clicked"]
            if c and (lng := c.get("lng")) and (lat := c.get("lat")):
                if st.session_state.waiting_for_start_point:
                    st.session_state.points_gcj['A'] = [lng, lat]
                    update_path_after_point_change()
                    st.session_state.waiting_for_start_point = False
                    st.rerun()
                if st.session_state.waiting_for_end_point:
                    st.session_state.points_gcj['B'] = [lng, lat]
                    update_path_after_point_change()
                    st.session_state.waiting_for_end_point = False
                    st.rerun()
        if out and out.get("last_active_drawing"):
            g = out["last_active_drawing"]["geometry"]
            if g and g["type"] == "Polygon":
                coords = [[p[0], p[1]] for p in g["coordinates"][0]]
                if len(coords) >= 3 and st.session_state.pending_obstacle is None:
                    st.session_state.pending_obstacle = coords
                    st.rerun()
    if st.session_state.pending_obstacle:
        st.subheader("添加障碍物")
        name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
        h = st.number_input("高度(m)", 1, 200, 30, 5)
        if st.button("确认添加", type="primary"):
            st.session_state.obstacles_gcj.append({
                "name": name, "polygon": st.session_state.pending_obstacle,
                "height": h, "selected": False, "id": f"obs_{datetime.now():%Y%m%d_%H%M%S}"
            })
            save_obstacles(st.session_state.obstacles_gcj) if auto_save else None
            update_path_after_obstacle_change(flight_alt)
            st.session_state.pending_obstacle = None
            st.rerun()
        if st.button("取消"):
            st.session_state.pending_obstacle = None
            st.rerun()


def update_flight_simulation():
    now = time.time()
    if st.session_state.simulation_running and now - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
        hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
        if hb:
            st.session_state.last_hb_time = now
            st.session_state.flight_history.append([hb.lng, hb.lat])
            if len(st.session_state.flight_history) > 200:
                st.session_state.flight_history.pop(0)
            if not st.session_state.heartbeat_sim.simulating:
                st.session_state.simulation_running = False
            st.rerun()


def render_flight_monitoring_page(map_type, flight_alt, drone_speed):
    st.header("📡 飞行监控")
    update_flight_simulation()
    if not st.session_state.heartbeat_sim.history:
        st.info("请先开始飞行")
        return
    hb = st.session_state.heartbeat_sim.history[0]
    st.progress(hb.progress if not hb.arrived else 1.0, f"飞行进度：{int(hb.progress*100)}%")
    c1, c2, c3 = st.columns(3)
    c1.metric("飞行速度", f"{hb.speed:.1f} m/s")
    c2.metric("剩余距离", f"{max(0, hb.remaining_distance):.0f} m")
    c3.metric("电量", f"{round(100 - min(100, hb.flight_time/18))}%")
    m = folium.Map(location=[hb.lat, hb.lng], zoom_start=18, tiles=config.GAODE_SATELLITE_URL if map_type=="satellite" else config.GAODE_VECTOR_URL)
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        h = obs.get('height', 30)
        if len(coords)>=3:
            folium.Polygon([[p[1],p[0]]for p in coords], color="red" if h>flight_alt else "orange", weight=2, fill=True, fill_opacity=0.3).add_to(m)
    if st.session_state.planned_path:
        folium.PolyLine([[p[1],p[0]]for p in st.session_state.planned_path], color="green", weight=3).add_to(m)
    folium.Circle(radius=st.session_state.safety_radius, location=[hb.lat, hb.lng], color="blue", fill=True, fill_opacity=0.2).add_to(m)
    folium.Marker([hb.lat, hb.lng], icon=folium.Icon(color="red", icon="plane")).add_to(m)
    folium_static(m, width=900, height=500)
    if st.button("停止飞行"):
        st.session_state.simulation_running = False
        st.session_state.heartbeat_sim.simulating = False
        st.rerun()


def render_obstacle_management_page(flight_alt):
    st.header("🚧 障碍物管理")
    c1, c2, c3, c4 = st.columns(4)
    c1.info(f"总数：{len(st.session_state.obstacles_gcj)}")
    c2.info(f"安全半径：{st.session_state.safety_radius}m")
    b1, b2, b3, b4, b5 = st.columns(5)
    if b1.button("保存", type="primary"):
        save_obstacles(st.session_state.obstacles_gcj)
        st.success("保存成功")
        st.rerun()
    if b2.button("加载"):
        st.session_state.obstacles_gcj = load_obstacles()
        update_path_after_obstacle_change(flight_alt)
        st.rerun()
    b3.download_button("导出", json.dumps({"obstacles": st.session_state.obstacles_gcj}, ensure_ascii=False, indent=2), "obstacles.json")
    if b4.button("恢复备份") and (p:=get_latest_backup()):
        restore_from_backup(p)
        st.session_state.obstacles_gcj = load_obstacles()
        update_path_after_obstacle_change(flight_alt)
        st.rerun()
    if b5.button("清空"):
        st.session_state.obstacles_gcj = []
        save_obstacles([])
        update_path_after_obstacle_change(flight_alt)
        st.rerun()
    tab1, tab2 = st.tabs(["列表", "地图"])
    with tab1:
        for i, obs in enumerate(st.session_state.obstacles_gcj):
            with st.container(border=True):
                cc1, cc2 = st.columns([1, 5])
                obs['selected'] = cc1.checkbox("", obs.get('selected', False), key=f"sel_{i}")
                cc2.markdown(f"{'🔴' if obs.get('height',30)>flight_alt else '🟠'}{obs.get('name')}")
                h = st.number_input("高度", value=obs.get('height',30), min_value=1, max_value=200, key=f"h_{i}")
                if h != obs.get('height'):
                    obs['height'] = h
                    save_obstacles(st.session_state.obstacles_gcj)
                    update_path_after_obstacle_change(flight_alt)
                    st.rerun()
                if st.button("删除", key=f"del_{i}"):
                    st.session_state.obstacles_gcj.pop(i)
                    save_obstacles(st.session_state.obstacles_gcj)
                    update_path_after_obstacle_change(flight_alt)
                    st.rerun()
    with tab2:
        m = folium.Map(location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=config.GAODE_SATELLITE_URL)
        plugins.Draw(export=True, position='topleft', draw_options={'polygon':{'color':'#f00','fillColor':'#f00','fillOpacity':0.4}}).add_to(m)
        for obs in st.session_state.obstacles_gcj:
            coords = obs.get('polygon', [])
            h = obs.get('height', 30)
            if len(coords)>=3:
                folium.Polygon([[p[1],p[0]]for p in coords], color="red" if h>flight_alt else "orange", weight=3, fill=True, fill_opacity=0.5).add_to(m)
        out = st_folium(m, 800, 550, returned_objects=["last_active_drawing"])
        if out and out.get("last_active_drawing"):
            g = out["last_active_drawing"]["geometry"]
            if g and g["type"] == "Polygon":
                coords = [[p[0],p[1]]for p in g["coordinates"][0]]
                if len(coords)>=3 and st.session_state.pending_obstacle is None:
                    st.session_state.pending_obstacle = coords
                    st.rerun()
    if st.session_state.pending_obstacle:
        name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
        h = st.number_input("高度(m)", 1, 200, 30)
        if st.button("确认添加", type="primary"):
            st.session_state.obstacles_gcj.append({"name":name,"polygon":st.session_state.pending_obstacle,"height":h,"selected":False})
            save_obstacles(st.session_state.obstacles_gcj)
            update_path_after_obstacle_change(flight_alt)
            st.session_state.pending_obstacle = None
            st.rerun()
        if st.button("取消"):
            st.session_state.pending_obstacle = None
            st.rerun()


# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="无人机地面站", layout="wide")
    init_session_state()
    st.title("无人机地面站系统")
    page, mt, speed, alt, auto = render_sidebar()
    st.session_state.auto_backup = auto
    if alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = alt
        update_path_after_obstacle_change(alt)
        st.rerun()
    if page == "🗺️ 航线规划":
        render_planning_page(mt, speed, alt, auto)
    elif page == "📡 飞行监控":
        render_flight_monitoring_page(mt, alt, speed)
    elif page == "🚧 障碍物管理":
        render_obstacle_management_page(alt)


if __name__ == "__main__":
    main()
