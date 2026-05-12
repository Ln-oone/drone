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

def segments_intersect(p1, p2, p3, p4) -> bool:
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        return 0 if abs(val) < 1e-10 else (1 if val > 0 else 2)
    def on_segment(p, q, r):
        return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
    o1, o2, o3, o4 = orientation(p1, p2, p3), orientation(p1, p2, p4), orientation(p3, p4, p1), orientation(p3, p4, p2)
    return (o1 != o2 and o3 != o4) or (o1 == 0 and on_segment(p1, p3, p2)) or (o2 == 0 and on_segment(p1, p4, p2)) or (o3 == 0 and on_segment(p3, p1, p4)) or (o4 == 0 and on_segment(p3, p2, p4))

def line_intersects_polygon(p1, p2, polygon) -> bool:
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    for i in range(len(polygon)):
        if segments_intersect(p1, p2, polygon[i], polygon[(i + 1) % len(polygon)]):
            return True
    return False

def distance(p1, p2) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def meters_to_deg(meters: float, lat: float = 32.23):
    return meters / (111000 * math.cos(math.radians(lat))), meters / 111000

def point_to_segment_distance_meters(point, seg_start, seg_end) -> float:
    px, py, x1, y1, x2, y2 = *point, *seg_start, *seg_end
    dx, dy = x2 - x1, y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / len_sq))
    proj_x, proj_y = x1 + t * dx, y1 + t * dy
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2) * 111000

def check_safety_radius(drone_pos, obstacles_gcj, flight_altitude, safety_radius):
    if not drone_pos:
        return True, None, None
    min_dist = float('inf')
    danger_name = None
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if obs.get('height', 30) <= flight_altitude or not coords or len(coords) < 3:
            continue
        for i in range(len(coords)):
            dist = point_to_segment_distance_meters(drone_pos, coords[i], coords[(i + 1) % len(coords)])
            if dist < min_dist:
                min_dist, danger_name = dist, obs.get('name', '障碍物')
    return (min_dist >= safety_radius, min_dist if min_dist != float('inf') else None, danger_name)

# ==================== 障碍物管理 ====================
def load_obstacles() -> List[Dict]:
    if os.path.exists(config.CONFIG_FILE):
        try:
            data = json.load(open(config.CONFIG_FILE, 'r', encoding='utf-8'))
            for obs in data.get('obstacles', []):
                obs.setdefault('selected', False)
                obs.setdefault('height', 30)
            return data.get('obstacles', [])
        except:
            return []
    return []

def save_obstacles(obstacles: List[Dict]) -> bool:
    try:
        json.dump({'obstacles': obstacles, 'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")}, 
                  open(config.CONFIG_FILE, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
        return True
    except:
        return False

# ==================== 绕行算法 ====================
def get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude):
    return [obs for obs in obstacles_gcj if obs.get('height', 30) > flight_altitude 
            and obs.get('polygon', []) and line_intersects_polygon(start, end, obs['polygon'])]

def find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking:
        return [start, end]
    max_lng, max_lat, min_lat = -float('inf'), -float('inf'), float('inf')
    for obs in blocking:
        for p in obs['polygon']:
            max_lng, max_lat, min_lat = max(max_lng, p[0]), max(max_lat, p[1]), min(min_lat, p[1])
    safe_lng, safe_lat = meters_to_deg(safety_radius * 3)
    obstacle_height = max_lat - min_lat
    point1 = [start[0] + 0.0012, max_lat + obstacle_height * 3 + safe_lat * 5 + 0.0002]
    point2 = [max_lng + obstacle_height * 2 + safe_lng * 3, point1[1]]
    return [start, point1, point2, end]

def find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking:
        return [start, end]
    mid_x, mid_y = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length == 0:
        return [start, end]
    perp_x, perp_y = dy / length, -dx / length
    offset_dist = safety_radius * 10
    lat_rad, lng_scale, lat_scale = math.radians(mid_y), 111000 * math.cos(math.radians(mid_y)), 111000
    waypoint = [mid_x + perp_x * offset_dist / lng_scale, mid_y + perp_y * offset_dist / lat_scale]
    return [start, waypoint, end]

def find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    left, right = find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius), find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    return left if sum(distance(left[i], left[i+1]) for i in range(len(left)-1)) < sum(distance(right[i], right[i+1]) for i in range(len(right)-1)) else right

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction, safety_radius=5):
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    return find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp: str; flight_time: float; lat: float; lng: float; altitude: float
    voltage: float; satellites: int; speed: float; progress: float; arrived: bool
    safety_violation: bool; remaining_distance: float

class HeartbeatSimulator:
    def __init__(self, start_point_gcj: List[float]):
        self.history, self.flight_log = [], []
        self.current_pos, self.path = start_point_gcj.copy(), [start_point_gcj.copy()]
        self.path_index, self.simulating = 0, False
        self.flight_altitude, self.speed, self.progress = 50, 50, 0.0
        self.total_distance, self.distance_traveled = 0.0, 0.0
        self.safety_radius, self.safety_violation = 5, False
        self.start_time, self.last_update_time = None, None

    def set_path(self, path, altitude=50, speed=50, safety_radius=5):
        self.path, self.path_index = path, 0
        self.current_pos = path[0].copy()
        self.flight_altitude, self.speed, self.safety_radius = altitude, speed, safety_radius
        self.simulating, self.progress, self.distance_traveled = True, 0.0, 0.0
        self.safety_violation, self.start_time, self.last_update_time = False, datetime.now(), None
        self.total_distance = sum(distance(self.path[i], self.path[i+1]) for i in range(len(path)-1))

    def update_and_generate(self, obstacles_gcj):
        if not self.simulating or self.path_index >= len(self.path) - 1:
            self.simulating = False
            return None
        now = time.time()
        delta = min(0.5, now - self.last_update_time) if self.last_update_time else 0.2
        self.last_update_time = now
        start, end = self.path[self.path_index], self.path[self.path_index + 1]
        seg_dist = distance(start, end)
        move_dist = config.BASE_SPEED_MPS * (self.speed / 100) * delta
        self.distance_traveled += move_dist
        completed = sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_index))
        completed += min(self.distance_traveled, seg_dist)
        self.progress = min(1.0, completed / self.total_distance) if self.total_distance > 0 else 0
        
        if self.distance_traveled >= seg_dist and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        else:
            t = min(1.0, max(0.0, self.distance_traveled / seg_dist)) if seg_dist > 0 else 0
            self.current_pos = [start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t]
        
        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        self.safety_violation = not safe
        return self._generate_heartbeat(False)

    def _generate_heartbeat(self, arrived):
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        remaining = 0
        if not arrived and self.path_index < len(self.path) - 1:
            remaining = distance(self.current_pos, self.path[self.path_index + 1])
            for i in range(self.path_index + 1, len(self.path) - 1):
                remaining += distance(self.path[i], self.path[i + 1])
            remaining *= 111000
        hb = HeartbeatData(datetime.now().strftime("%H:%M:%S"), flight_time, self.current_pos[1], self.current_pos[0],
                           self.flight_altitude, round(22.2 + random.uniform(-0.5, 0.5), 1), random.randint(8, 14),
                           round(config.BASE_SPEED_MPS * (self.speed / 100), 1), self.progress, arrived,
                           self.safety_violation, remaining)
        self.history.insert(0, hb)
        self.history = self.history[:100]
        self.flight_log.append(hb)
        self.flight_log = self.flight_log[-1000:]
        return hb

# ==================== 地图创建 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history, planned_path, map_type, straight_blocked, flight_altitude, drone_pos, direction, safety_radius):
    tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr="高德地图")
    m.add_child(plugins.Draw(export=True, position='topleft', draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4}}))
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            color = "red" if obs.get('height', 30) > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {obs.get('height', 30)}m").add_to(m)
    
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        line_color = {"向左绕行": "purple", "向右绕行": "orange"}.get(direction, "green")
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color=line_color, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], 
                        color="gray" if straight_blocked else "blue", weight=2, opacity=0.4, dash_array='5, 5').add_to(m)
    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]], color="blue", weight=2, fill=True, fill_opacity=0.2).add_to(m)
    if flight_history and len(flight_history) > 1:
        folium.PolyLine([[p[1], p[0]] for p in flight_history if len(p) >= 2], color="orange", weight=2, opacity=0.6).add_to(m)
    return m

# ==================== 主程序 UI ====================
st.set_page_config(page_title="无人机地面站系统", layout="wide")

# 初始化session state
if 'points_gcj' not in st.session_state:
    st.session_state.points_gcj = {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()}
    st.session_state.obstacles_gcj = load_obstacles()
    st.session_state.heartbeat_sim = HeartbeatSimulator(config.DEFAULT_A_GCJ.copy())
    st.session_state.last_hb_time = time.time()
    st.session_state.simulation_running = False
    st.session_state.flight_history = []
    st.session_state.planned_path = None
    st.session_state.last_flight_altitude = 50
    st.session_state.pending_obstacle = None
    st.session_state.current_direction = "最佳航线"
    st.session_state.safety_radius = config.DEFAULT_SAFETY_RADIUS_METERS
    st.session_state.waiting_for_start_point = False
    st.session_state.waiting_for_end_point = False
    for obs in st.session_state.obstacles_gcj:
        obs.setdefault('height', 30)
        obs.setdefault('selected', False)

st.title("🏫 无人机地面站系统")
st.markdown("---")

# 侧边栏
page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
map_type = "satellite" if st.sidebar.radio("地图类型", ["卫星影像", "矢量街道"], index=0) == "卫星影像" else "vector"
drone_speed = st.sidebar.slider("飞行速度系数", 10, 100, 50, 5)
flight_alt = st.sidebar.slider("飞行高度 (m)", 10, 200, 50, 5)
safety_radius = st.sidebar.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
st.session_state.safety_radius = safety_radius
auto_save = st.sidebar.checkbox("自动保存障碍物", value=True)

if flight_alt != st.session_state.last_flight_altitude:
    st.session_state.last_flight_altitude = flight_alt
    st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                                                           st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)

def update_path():
    st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                                                           st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)

# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.header("🗺️ 航线规划 - 智能避障")
    blocked, high = False, 0
    for obs in st.session_state.obstacles_gcj:
        if obs.get('height', 30) > flight_alt:
            high += 1
            if obs.get('polygon', []) and line_intersects_polygon(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], obs['polygon']):
                blocked = True
    if blocked:
        st.warning(f"⚠️ 有 {high} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
    else:
        st.success("✅ 直线航线畅通无阻")
    
    col1, col2 = st.columns([1, 1.5])
    with col1:
        st.subheader("🎮 控制面板")
        with st.expander("📍 起点/终点设置", expanded=True):
            mode = st.radio("设置方式", ["✏️ 经纬度输入", "🖱️ 鼠标点击"], horizontal=True)
            if mode == "✏️ 经纬度输入":
                a_lat, a_lng = st.number_input("起点纬度", value=st.session_state.points_gcj['A'][1], format="%.6f"), st.number_input("起点经度", value=st.session_state.points_gcj['A'][0], format="%.6f")
                if st.button("📍 设置 A 点", use_container_width=True):
                    st.session_state.points_gcj['A'] = [a_lng, a_lat]
                    update_path()
                    st.rerun()
                b_lat, b_lng = st.number_input("终点纬度", value=st.session_state.points_gcj['B'][1], format="%.6f"), st.number_input("终点经度", value=st.session_state.points_gcj['B'][0], format="%.6f")
                if st.button("📍 设置 B 点", use_container_width=True):
                    st.session_state.points_gcj['B'] = [b_lng, b_lat]
                    update_path()
                    st.rerun()
            else:
                if st.button("🎯 设置起点", use_container_width=True):
                    st.session_state.waiting_for_start_point, st.session_state.waiting_for_end_point = True, False
                    st.rerun()
                if st.button("📍 设置终点", use_container_width=True):
                    st.session_state.waiting_for_end_point, st.session_state.waiting_for_start_point = True, False
                    st.rerun()
                if st.session_state.waiting_for_start_point:
                    st.warning("⏳ 请点击地图设置起点")
                elif st.session_state.waiting_for_end_point:
                    st.warning("⏳ 请点击地图设置终点")
        
        with st.expander("🤖 路径规划策略", expanded=True):
            for d, label in [("最佳航线", "🔄"), ("向左绕行", "⬅️"), ("向右绕行", "➡️")]:
                if st.button(f"{label} {d}", use_container_width=True, type="primary" if st.session_state.current_direction == d else "secondary"):
                    st.session_state.current_direction = d
                    update_path()
                    st.rerun()
            if st.button("🔄 重新规划路径", use_container_width=True):
                update_path()
                st.success(f"已按照「{st.session_state.current_direction}」规划路径")
                st.rerun()
        
        with st.expander("✈️ 飞行控制", expanded=True):
            st.metric("当前飞行高度", f"{flight_alt} m")
            st.metric("速度系数", f"{drone_speed}%")
            if st.session_state.planned_path:
                st.metric("绕行点数量", len(st.session_state.planned_path) - 2)
            if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, safety_radius)
                    st.session_state.simulation_running, st.session_state.flight_history = True, []
                    st.success("🚁 飞行已开始")
                    st.rerun()
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = st.session_state.heartbeat_sim.simulating = False
                st.info("飞行已停止")
    
    with col2:
        st.subheader("🗺️ 规划地图")
        drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
        if st.session_state.planned_path is None:
            update_path()
        trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
        m = create_planning_map(st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ, st.session_state.points_gcj,
                                 st.session_state.obstacles_gcj, trail, st.session_state.planned_path, map_type,
                                 blocked, flight_alt, drone_pos, st.session_state.current_direction, safety_radius)
        output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
        
        if output and output.get("last_clicked") and isinstance(output["last_clicked"], dict):
            lng, lat = output["last_clicked"].get("lng"), output["last_clicked"].get("lat")
            if lng and lat:
                if st.session_state.waiting_for_start_point:
                    st.session_state.points_gcj['A'] = [lng, lat]
                    update_path()
                    st.session_state.waiting_for_start_point = False
                    st.rerun()
                elif st.session_state.waiting_for_end_point:
                    st.session_state.points_gcj['B'] = [lng, lat]
                    update_path()
                    st.session_state.waiting_for_end_point = False
                    st.rerun()
        
        if output and output.get("last_active_drawing") and output["last_active_drawing"].get("geometry", {}).get("type") == "Polygon" and st.session_state.pending_obstacle is None:
            coords = output["last_active_drawing"]["geometry"]["coordinates"][0]
            poly = [[p[0], p[1]] for p in coords]
            if len(poly) >= 3:
                st.session_state.pending_obstacle = poly
                st.rerun()
        
        if st.session_state.pending_obstacle:
            with st.container():
                st.subheader("📝 添加新障碍物")
                name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
                height = st.number_input("高度 (米)", 1, 200, 30, 5)
                if st.button("✅ 确认添加"):
                    st.session_state.obstacles_gcj.append({"name": name, "polygon": st.session_state.pending_obstacle, "height": height, "selected": False})
                    if auto_save: save_obstacles(st.session_state.obstacles_gcj)
                    update_path()
                    st.session_state.pending_obstacle = None
                    st.rerun()
                if st.button("❌ 取消"):
                    st.session_state.pending_obstacle = None
                    st.rerun()

# ==================== 飞行监控页面 ====================
elif page == "📡 飞行监控":
    st.header("📡 飞行监控 - 实时心跳包")
    now = time.time()
    if st.session_state.simulation_running and now - st.session_state.last_hb_time >= 0.2:
        new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
        if new_hb:
            st.session_state.last_hb_time = now
            st.session_state.flight_history.append([new_hb.lng, new_hb.lat])
            st.session_state.flight_history = st.session_state.flight_history[-200:]
            if not st.session_state.heartbeat_sim.simulating:
                st.session_state.simulation_running = False
                st.success("🏁 无人机已安全到达目的地！")
            st.rerun()
    
    if st.session_state.heartbeat_sim.history:
        hb = st.session_state.heartbeat_sim.history[0]
        waypoints = len(st.session_state.planned_path) if st.session_state.planned_path else 0
        current_wp = waypoints if hb.arrived else (int(hb.progress * (waypoints - 1)) + 1 if waypoints > 0 else 0)
        remaining = max(0, hb.remaining_distance)
        
        st.progress(hb.progress if not hb.arrived else 1.0, text=f"飞行进度：{int(hb.progress*100) if not hb.arrived else 100}%")
        
        c1, c2, c3 = st.columns(3)
        c1.metric("🎯 当前航点", f"{min(current_wp, waypoints)} / {waypoints}")
        c2.metric("💨 飞行速度", f"{hb.speed:.1f} m/s")
        c3.metric("⏰ 已用时间", f"{int(hb.flight_time//60):02d}:{int(hb.flight_time%60):02d}")
        
        c4, c5, c6 = st.columns(3)
        c4.metric("📏 剩余距离", f"{remaining:.0f} m")
        eta = "00:00" if hb.arrived else (f"{int(remaining/hb.speed):.0f}秒" if hb.speed > 0 and remaining < 60 else f"{int(remaining/hb.speed//60):02d}:{int(remaining/hb.speed%60):02d}" if hb.speed > 0 else "计算中...")
        c5.metric("🕐 预计到达", eta)
        bat = max(0, min(100, (1 - hb.flight_time / 1800) * 100))
        c6.metric("🔋 电量模拟", f"{bat:.0f}%", f"{hb.voltage:.1f}V")
        
        if hb.safety_violation and not hb.arrived:
            st.error("⚠️ 警告：无人机进入安全半径危险区域！")
        if hb.arrived:
            st.success("🎉 无人机已到达目的地！飞行任务完成！")
        
        tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
        m = folium.Map(location=[hb.lat, hb.lng], zoom_start=18, tiles=tiles)
        for obs in st.session_state.obstacles_gcj:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                color = "red" if obs.get('height', 30) > flight_alt else "orange"
                folium.Polygon([[c[1], c[0]] for c in coords], color=color, fill=True, fill_opacity=0.3).add_to(m)
        if st.session_state.planned_path:
            folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color="green", weight=3).add_to(m)
        folium.Circle(radius=safety_radius, location=[hb.lat, hb.lng], color="blue", fill=True, fill_opacity=0.2).add_to(m)
        trail = [[h.lat, h.lng] for h in st.session_state.heartbeat_sim.history[:50]]
        if len(trail) > 1: folium.PolyLine(trail, color="orange", weight=2).add_to(m)
        folium.Marker([hb.lat, hb.lng], popup=f"当前位置", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
        folium_static(m, width=900, height=400)
        
        if st.button("📊 导出飞行数据", use_container_width=True):
            df = pd.DataFrame([{'时间': h.timestamp, '飞行时间': h.flight_time, '纬度': h.lat, '经度': h.lng, '速度': h.speed, '剩余距离': h.remaining_distance} for h in st.session_state.heartbeat_sim.flight_log])
            if not df.empty:
                st.download_button("📥 下载CSV", df.to_csv(index=False), f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    else:
        st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")

# ==================== 障碍物管理页面 ====================
else:
    st.header("🚧 障碍物管理")
    c1, c2, c3, c4 = st.columns(4)
    c1.info(f"📊 共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    c2.info(f"🛡️ 安全半径: {safety_radius}米")
    
    col1, col2, col3, col4, col5 = st.columns(5)
    if col1.button("💾 保存配置", use_container_width=True):
        if save_obstacles(st.session_state.obstacles_gcj):
            st.success("✅ 已保存")
            st.rerun()
    if col2.button("📂 加载配置", use_container_width=True):
        loaded = load_obstacles()
        if loaded:
            st.session_state.obstacles_gcj = loaded
            update_path()
            st.rerun()
    if st.session_state.obstacles_gcj and col3.button("📥 导出配置", use_container_width=True):
        st.download_button("下载", json.dumps({'obstacles': st.session_state.obstacles_gcj}, ensure_ascii=False, indent=2), f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    if col4.button("🔄 恢复备份", use_container_width=True):
        if os.path.exists(config.CONFIG_FILE):
            st.session_state.obstacles_gcj = load_obstacles()
            update_path()
            st.rerun()
    if col5.button("🗑️ 清除全部", use_container_width=True):
        st.session_state.obstacles_gcj = []
        save_obstacles([])
        update_path()
        st.rerun()
    
    for obs in st.session_state.obstacles_gcj:
        obs.setdefault('selected', False)
    
    select_all = st.checkbox("☑️ 全选")
    if select_all:
        for obs in st.session_state.obstacles_gcj: obs['selected'] = True
    
    if st.button("🗑️ 批量删除", use_container_width=True):
        st.session_state.obstacles_gcj = [obs for obs in st.session_state.obstacles_gcj if not obs.get('selected', False)]
        if auto_save: save_obstacles(st.session_state.obstacles_gcj)
        update_path()
        st.rerun()
    
    batch_h = st.number_input("批量高度(m)", 1, 200, 30, 5)
    if st.button("📏 批量设置高度", use_container_width=True):
        for obs in st.session_state.obstacles_gcj:
            if obs.get('selected'): obs['height'] = batch_h
        if auto_save: save_obstacles(st.session_state.obstacles_gcj)
        update_path()
        st.rerun()
    
    tab_list, tab_map = st.tabs(["📋 列表视图", "🗺️ 地图视图"])
    with tab_list:
        for i, obs in enumerate(st.session_state.obstacles_gcj):
            with st.container(border=True):
                col_a, col_b = st.columns([1, 5])
                with col_a: obs['selected'] = st.checkbox("", key=f"sel_{i}", value=obs.get('selected', False))
                with col_b: st.markdown(f"**{obs.get('name', f'障碍物{i+1}')}**")
                st.caption(f"📏 高度: {obs.get('height', 30)}m | 📍 顶点: {len(obs.get('polygon', []))}个")
                new_h = st.number_input("高度", obs.get('height', 30), 1, 200, 5, key=f"h_{i}", label_visibility="collapsed")
                if new_h != obs.get('height', 30):
                    obs['height'] = new_h
                    if auto_save: save_obstacles(st.session_state.obstacles_gcj)
                    update_path()
                    st.rerun()
                if st.button("🗑️ 删除", key=f"del_{i}", use_container_width=True):
                    st.session_state.obstacles_gcj.pop(i)
                    if auto_save: save_obstacles(st.session_state.obstacles_gcj)
                    update_path()
                    st.rerun()
    
    with tab_map:
        tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
        m = folium.Map(location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles)
        m.add_child(plugins.Draw(export=True, draw_options={'polygon': {'allowIntersection': False, 'showArea': True}}))
        for obs in st.session_state.obstacles_gcj:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                color = "red" if obs.get('height', 30) > flight_alt else "orange"
                folium.Polygon([[c[1], c[0]] for c in coords], color=color, fill=True, fill_opacity=0.4, popup=obs.get('name')).add_to(m)
        output = st_folium(m, width=800, height=550, returned_objects=["last_active_drawing"])
        if output and output.get("last_active_drawing") and output["last_active_drawing"].get("geometry", {}).get("type") == "Polygon" and st.session_state.pending_obstacle is None:
            coords = output["last_active_drawing"]["geometry"]["coordinates"][0]
            poly = [[p[0], p[1]] for p in coords]
            if len(poly) >= 3:
                st.session_state.pending_obstacle = poly
                st.rerun()
        if st.session_state.pending_obstacle:
            name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
            height = st.number_input("高度", 1, 200, 30, 5)
            if st.button("确认"):
                st.session_state.obstacles_gcj.append({"name": name, "polygon": st.session_state.pending_obstacle, "height": height, "selected": False})
                if auto_save: save_obstacles(st.session_state.obstacles_gcj)
                update_path()
                st.session_state.pending_obstacle = None
                st.rerun()
