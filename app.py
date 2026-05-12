import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random, time, math, json, os, shutil
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
import pandas as pd
from dataclasses import dataclass, field

# ==================== 配置 ====================
@dataclass
class Config:
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.748807, 32.233931])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.750046, 32.236150])
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 5
    BASE_SPEED_MPS: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    VOLTAGE_VARIATION: float = 0.5
    SAT_RANGE: Tuple[int, int] = (8, 14)
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR_URL: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 几何工具函数 ====================
def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def on_segment(p, q, r):
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and 
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    return 0 if abs(val) < 1e-10 else (1 if val > 0 else 2)

def segments_intersect(p1, p2, p3, p4):
    o1, o2, o3, o4 = orientation(p1, p2, p3), orientation(p1, p2, p4), orientation(p3, p4, p1), orientation(p3, p4, p2)
    if o1 != o2 and o3 != o4: return True
    if o1 == 0 and on_segment(p1, p3, p2): return True
    if o2 == 0 and on_segment(p1, p4, p2): return True
    if o3 == 0 and on_segment(p3, p1, p4): return True
    if o4 == 0 and on_segment(p3, p2, p4): return True
    return False

def line_intersects_polygon(p1, p2, polygon):
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon): return True
    return any(segments_intersect(p1, p2, polygon[i], polygon[(i+1)%len(polygon)]) for i in range(len(polygon)))

def distance(p1, p2):
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def meters_to_deg(meters, lat=32.23):
    return meters/111000, meters/(111000*math.cos(math.radians(lat)))

def point_to_segment_distance_meters(point, seg_start, seg_end):
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    dx, dy = x2-x1, y2-y1
    len_sq = dx*dx + dy*dy
    if len_sq == 0: return math.hypot(px-x1, py-y1) * 111000
    t = max(0, min(1, ((px-x1)*dx + (py-y1)*dy)/len_sq))
    return math.hypot(px - (x1 + t*dx), py - (y1 + t*dy)) * 111000

def check_safety_radius(drone_pos, obstacles_gcj, flight_altitude, safety_radius):
    if not drone_pos: return True, None, None
    min_dist, danger_name = float('inf'), None
    for obs in obstacles_gcj:
        coords, obs_h = obs.get('polygon', []), obs.get('height', 30)
        if obs_h <= flight_altitude or not coords: continue
        for i in range(len(coords)):
            dist = point_to_segment_distance_meters(drone_pos, coords[i], coords[(i+1)%len(coords)])
            if dist < min_dist:
                min_dist, danger_name = dist, obs.get('name', '障碍物')
    return (min_dist >= safety_radius, min_dist if min_dist != float('inf') else None, danger_name)

# ==================== 障碍物管理 ====================
def load_obstacles():
    if os.path.exists(config.CONFIG_FILE):
        try:
            obstacles = json.load(open(config.CONFIG_FILE, 'r', encoding='utf-8')).get('obstacles', [])
            for obs in obstacles:
                obs.setdefault('selected', False)
                obs.setdefault('height', 30)
            return obstacles
        except: return []
    return []

def save_obstacles(obstacles):
    try:
        if os.path.exists(config.CONFIG_FILE):
            shutil.copy(config.CONFIG_FILE, f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{datetime.now().strftime('%Y%m%d_%H%M%S')}.bak")
        json.dump({'obstacles': obstacles, 'count': len(obstacles), 'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")}, 
                  open(config.CONFIG_FILE, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
        return True
    except: return False

# ==================== 绕行算法 ====================
def get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude):
    return [obs for obs in obstacles_gcj if obs.get('height', 30) > flight_altitude and 
            obs.get('polygon') and line_intersects_polygon(start, end, obs['polygon'])]

def find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking: return [start, end]
    max_lng, max_lat, min_lat = -float('inf'), -float('inf'), float('inf')
    for obs in blocking:
        for p in obs['polygon']:
            max_lng, max_lat, min_lat = max(max_lng, p[0]), max(max_lat, p[1]), min(min_lat, p[1])
    if max_lng == -float('inf'): return [start, end]
    safe_lng, safe_lat = meters_to_deg(safety_radius * 3)
    obstacle_h = max_lat - min_lat
    p1 = [start[0] + 0.0012, max_lat + obstacle_h * 3 + safe_lat * 5 + 0.0002]
    p2 = [max_lng + obstacle_h * 2 + safe_lng * 3, p1[1]]
    return [start, p1, p2, end]

def find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking: return [start, end]
    mid_x, mid_y = (start[0]+end[0])/2, (start[1]+end[1])/2
    dx, dy = end[0]-start[0], end[1]-start[1]
    length = math.hypot(dx, dy)
    if length == 0: return [start, end]
    perp_x, perp_y = dy/length, -dx/length
    offset_dist = safety_radius * 10
    lng_scale, lat_scale = 111000 * math.cos(math.radians(mid_y)), 111000
    offset_x, offset_y = perp_x * offset_dist / lng_scale, perp_y * offset_dist / lat_scale
    return [start, [mid_x + offset_x, mid_y + offset_y], end]

def find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    left, right = find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius), find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    return left if sum(distance(left[i], left[i+1]) for i in range(len(left)-1)) < sum(distance(right[i], right[i+1]) for i in range(len(right)-1)) else right

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction, safety_radius=5):
    if direction == "向左绕行": return find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    if direction == "向右绕行": return find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    return find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp: str; flight_time: float; lat: float; lng: float; altitude: float
    voltage: float; satellites: int; speed: float; progress: float
    arrived: bool; safety_violation: bool; remaining_distance: float

class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history, self.flight_log = [], []
        self.current_pos = start_point_gcj.copy()
        self.path, self.path_index = [start_point_gcj.copy()], 0
        self.simulating = False
        self.flight_altitude, self.speed, self.progress = 50, 50, 0.0
        self.total_distance, self.distance_traveled, self.safety_radius = 0.0, 0.0, config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation, self.start_time, self.last_update_time = False, None, None

    def set_path(self, path, altitude=50, speed=50, safety_radius=5):
        self.path, self.path_index = path, 0
        self.current_pos = path[0].copy()
        self.flight_altitude, self.speed, self.safety_radius = altitude, speed, safety_radius
        self.simulating, self.progress, self.distance_traveled, self.safety_violation = True, 0.0, 0.0, False
        self.start_time, self.last_update_time = datetime.now(), None
        self.total_distance = sum(distance(self.path[i], self.path[i+1]) for i in range(len(path)-1))

    def update_and_generate(self, obstacles_gcj):
        if not self.simulating or self.path_index >= len(self.path)-1:
            self.simulating = False
            return None
        now = time.time()
        delta = min(0.5, now - self.last_update_time) if self.last_update_time else config.HEARTBEAT_INTERVAL
        self.last_update_time = now
        start, end = self.path[self.path_index], self.path[self.path_index+1]
        seg_dist = distance(start, end)
        move = config.BASE_SPEED_MPS * (self.speed/100) * delta
        self.distance_traveled += move
        
        if self.total_distance > 0:
            completed = sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_index))
            completed += min(seg_dist, self.distance_traveled)
            self.progress = min(1.0, completed / self.total_distance)
        
        if self.distance_traveled >= seg_dist and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
                return self._generate_heartbeat(False)
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        else:
            t = min(1.0, max(0.0, self.distance_traveled / seg_dist)) if seg_dist > 0 else 0
            self.current_pos = [start[0] + (end[0]-start[0])*t, start[1] + (end[1]-start[1])*t]
        
        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        self.safety_violation = not safe
        return self._generate_heartbeat(False)

    def _generate_heartbeat(self, arrived):
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        remaining = 0.0
        if not arrived and self.path_index < len(self.path)-1:
            remaining = max(0, distance(self.current_pos, self.path[self.path_index+1]))
            for i in range(self.path_index+1, len(self.path)-1):
                remaining += distance(self.path[i], self.path[i+1])
            remaining *= 111000
        
        hb = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"), flight_time=flight_time,
            lat=self.current_pos[1], lng=self.current_pos[0], altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-config.VOLTAGE_VARIATION, config.VOLTAGE_VARIATION), 1),
            satellites=random.randint(*config.SAT_RANGE),
            speed=round(config.BASE_SPEED_MPS * (self.speed/100), 1),
            progress=self.progress, arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=remaining
        )
        self.history.insert(0, hb)
        if len(self.history) > 100: self.history.pop()
        self.flight_log.append(hb)
        if len(self.flight_log) > 1000: self.flight_log.pop(0)
        return hb

    def export_flight_data(self):
        if not self.flight_log: return pd.DataFrame()
        return pd.DataFrame([{
            'timestamp': h.timestamp, 'flight_time': h.flight_time, 'lat': h.lat, 'lng': h.lng,
            'altitude': h.altitude, 'voltage': h.voltage, 'satellites': h.satellites,
            'speed': h.speed, 'progress': h.progress, 'arrived': h.arrived,
            'safety_violation': h.safety_violation, 'remaining_distance': h.remaining_distance
        } for h in self.flight_log])

# ==================== 地图创建 ====================
def create_map(center_gcj, points_gcj, obstacles_gcj, map_type, flight_altitude, planned_path=None, 
               drone_pos=None, direction="最佳航线", safety_radius=5, straight_blocked=True, flight_history=None):
    tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr="高德地图")
    m.add_child(plugins.Draw(export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 'marker': False},
        edit_options={'edit': True, 'remove': True}))
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            color = "red" if obs.get('height', 30) > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, 
                          fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {obs.get('height',30)}m").add_to(m)
    
    for pt, color, icon, label in [(points_gcj.get('A'), "green", "play", "🟢 起点"), 
                                    (points_gcj.get('B'), "red", "stop", "🔴 终点")]:
        if pt:
            folium.Marker([pt[1], pt[0]], popup=label, icon=folium.Icon(color=color, icon=icon, prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        colors = {"向左绕行": "purple", "向右绕行": "orange", "最佳航线": "green"}
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color=colors.get(direction, "green"), 
                        weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        for i, p in enumerate(planned_path[1:-1]):
            folium.CircleMarker([p[1], p[0]], radius=5, color="white", fill=True, fill_color="white", 
                                fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        color, dash = ("gray", '5, 5') if straight_blocked else ("blue", None)
        folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], 
                        color=color, weight=2, opacity=0.4 if straight_blocked else 0.5, dash_array=dash).add_to(m)
    
    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]], color="blue", weight=2, 
                     fill=True, fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)
    
    if flight_history and len(flight_history) > 1:
        folium.PolyLine([[p[1], p[0]] for p in flight_history if len(p) >= 2], color="orange", weight=2, opacity=0.6).add_to(m)
    return m

# ==================== 辅助函数 ====================
def init_session_state():
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(), 'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'last_hb_time': time.time(), 'simulation_running': False, 'flight_history': [],
        'planned_path': None, 'last_flight_altitude': 50, 'pending_obstacle': None,
        'current_direction': "最佳航线", 'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True, 'show_rename_dialog': False, 'waiting_for_start_point': False,
        'waiting_for_end_point': False, 'temp_click_point': None
    }
    for k, v in defaults.items():
        if k not in st.session_state: st.session_state[k] = v
    for obs in st.session_state.obstacles_gcj:
        obs.setdefault('height', 30)
        obs.setdefault('selected', False)

def update_path():
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
        st.session_state.current_direction, st.session_state.safety_radius)

def check_straight_blocked(points_gcj, obstacles_gcj, flight_altitude):
    blocked, high_count = False, 0
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            high_count += 1
            if obs.get('polygon') and line_intersects_polygon(points_gcj['A'], points_gcj['B'], obs['polygon']):
                blocked = True
    return blocked, high_count

# ==================== UI页面 ====================
def main():
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    init_session_state()
    st.title("🏫 无人机地面站系统")
    st.markdown("---")
    
    # 侧边栏
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type = "satellite" if st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0) == "卫星影像" else "vector"
    drone_speed = st.sidebar.slider("飞行速度系数", 10, 100, 50, 5)
    flight_alt = st.sidebar.slider("飞行高度 (m)", 10, 200, 50, 5)
    safety_radius = st.sidebar.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
    st.session_state.auto_backup = st.sidebar.checkbox("自动保存障碍物", st.session_state.auto_backup)
    st.session_state.safety_radius = safety_radius
    
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path: update_path()
        st.rerun()
    
    # 页面路由
    if page == "🗺️ 航线规划":
        render_planning_page(map_type, drone_speed, flight_alt)
    elif page == "📡 飞行监控":
        render_monitoring_page(map_type, flight_alt, drone_speed)
    else:
        render_obstacle_page(flight_alt)

def render_planning_page(map_type, drone_speed, flight_alt):
    st.header("🗺️ 航线规划 - 智能避障")
    blocked, high_cnt = check_straight_blocked(st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt)
    if blocked: st.warning(f"⚠️ 有 {high_cnt} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
    else: st.success("✅ 直线航线畅通无阻")
    st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
    
    col1, col2 = st.columns([1, 1.5])
    with col1:
        with st.expander("📍 起点/终点设置", expanded=True):
            mode = st.radio("设置方式", ["✏️ 经纬度输入", "🖱️ 鼠标点击"], horizontal=True, key="mode")
            if mode == "✏️ 经纬度输入":
                col_a1, col_a2 = st.columns(2)
                with col_a1: a_lat = st.number_input("起点纬度", value=st.session_state.points_gcj['A'][1], format="%.6f")
                with col_a2: a_lng = st.number_input("起点经度", value=st.session_state.points_gcj['A'][0], format="%.6f")
                if st.button("📍 设置A点", use_container_width=True):
                    st.session_state.points_gcj['A'] = [a_lng, a_lat]; update_path(); st.success("✅ 已设置"); st.rerun()
                col_b1, col_b2 = st.columns(2)
                with col_b1: b_lat = st.number_input("终点纬度", value=st.session_state.points_gcj['B'][1], format="%.6f")
                with col_b2: b_lng = st.number_input("终点经度", value=st.session_state.points_gcj['B'][0], format="%.6f")
                if st.button("📍 设置B点", use_container_width=True):
                    st.session_state.points_gcj['B'] = [b_lng, b_lat]; update_path(); st.success("✅ 已设置"); st.rerun()
            else:
                col_btn1, col_btn2 = st.columns(2)
                with col_btn1:
                    if st.button("🎯 设置起点", type="primary", use_container_width=True):
                        st.session_state.waiting_for_start_point, st.session_state.waiting_for_end_point = True, False
                with col_btn2:
                    if st.button("📍 设置终点", type="primary", use_container_width=True):
                        st.session_state.waiting_for_end_point, st.session_state.waiting_for_start_point = True, False
                if st.session_state.waiting_for_start_point: st.warning("⏳ 请点击地图设置起点")
                if st.session_state.waiting_for_end_point: st.warning("⏳ 请点击地图设置终点")
                if st.button("❌ 取消", use_container_width=True):
                    st.session_state.waiting_for_start_point = st.session_state.waiting_for_end_point = False
        
        with st.expander("🤖 路径规划策略", expanded=True):
            cols = st.columns(3)
            for i, (name, color) in enumerate([("最佳航线", "primary"), ("向左绕行", "secondary"), ("向右绕行", "secondary")]):
                with cols[i]:
                    if st.button(name, use_container_width=True, type=color if st.session_state.current_direction == name else "secondary"):
                        st.session_state.current_direction = name; update_path(); st.success(f"已切换到{name}"); st.rerun()
            if st.button("🔄 重新规划路径", use_container_width=True):
                update_path(); st.success(f"已重新规划"); st.rerun()
        
        with st.expander("✈️ 飞行控制", expanded=True):
            st.metric("当前飞行高度", f"{flight_alt} m")
            st.metric("速度系数", f"{drone_speed}%")
            if st.session_state.planned_path:
                st.metric("绕行点数量", len(st.session_state.planned_path)-2)
                st.caption(f"📏 路径总长: {sum(distance(st.session_state.planned_path[i], st.session_state.planned_path[i+1]) for i in range(len(st.session_state.planned_path)-1)) * 111000:.0f}米")
            if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
                    st.session_state.simulation_running, st.session_state.flight_history = True, []
                    st.success("🚁 飞行已开始"); st.rerun()
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = st.session_state.heartbeat_sim.simulating = False
        
        st.markdown("### 📍 当前坐标")
        st.write(f"🟢 A: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
        st.write(f"🔴 B: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
        dist = distance(st.session_state.points_gcj['A'], st.session_state.points_gcj['B']) * 111000
        st.caption(f"📏 直线距离: {dist:.0f}米")
    
    with col2:
        st.subheader("🗺️ 规划地图")
        st.caption("🟢绿色=最佳 | 🟣紫色=向左 | 🟠橙色=向右 | 🔵蓝色=安全半径")
        if not st.session_state.planned_path: update_path()
        drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
        flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
        
        output = st_folium(create_map(st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ, 
            st.session_state.points_gcj, st.session_state.obstacles_gcj, map_type, flight_alt,
            st.session_state.planned_path, drone_pos, st.session_state.current_direction,
            st.session_state.safety_radius, not blocked, flight_trail), width=700, height=550, returned_objects=["last_clicked", "last_active_drawing"])
        
        if output:
            if output.get("last_clicked") and (st.session_state.waiting_for_start_point or st.session_state.waiting_for_end_point):
                lng, lat = output["last_clicked"]["lng"], output["last_clicked"]["lat"]
                if st.session_state.waiting_for_start_point:
                    st.session_state.points_gcj['A'] = [lng, lat]; update_path()
                    st.session_state.waiting_for_start_point = False; st.success(f"✅ 起点已设置"); st.rerun()
                elif st.session_state.waiting_for_end_point:
                    st.session_state.points_gcj['B'] = [lng, lat]; update_path()
                    st.session_state.waiting_for_end_point = False; st.success(f"✅ 终点已设置"); st.rerun()
            
            if output.get("last_active_drawing") and not st.session_state.pending_obstacle:
                geom = output["last_active_drawing"].get("geometry")
                if geom and geom.get("type") == "Polygon":
                    coords = geom.get("coordinates", [[]])[0]
                    if len(coords) >= 3:
                        st.session_state.pending_obstacle = [[p[0], p[1]] for p in coords]
                        st.rerun()
        
        if st.session_state.pending_obstacle:
            st.markdown("---")
            st.subheader("📝 添加新障碍物")
            col_n1, col_n2 = st.columns(2)
            with col_n1: name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
            with col_n2: height = st.number_input("高度(米)", 1, 200, 30, 5)
            if st.button("✅ 确认添加", use_container_width=True):
                st.session_state.obstacles_gcj.append({"name": name, "polygon": st.session_state.pending_obstacle, "height": height, "selected": False})
                if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
                update_path(); st.session_state.pending_obstacle = None; st.success(f"✅ 已添加 {name}"); st.rerun()
            if st.button("❌ 取消", use_container_width=True):
                st.session_state.pending_obstacle = None; st.rerun()

def render_monitoring_page(map_type, flight_alt, drone_speed):
    st.header("📡 飞行监控 - 实时心跳包")
    
    # 更新飞行模拟
    now = time.time()
    if st.session_state.simulation_running and now - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
        hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
        if hb:
            st.session_state.last_hb_time = now
            st.session_state.flight_history.append([hb.lng, hb.lat])
            if len(st.session_state.flight_history) > 200: st.session_state.flight_history.pop(0)
            if not st.session_state.heartbeat_sim.simulating:
                st.session_state.simulation_running = False
                st.success("🏁 无人机已安全到达目的地！")
            st.rerun()
    
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        
        # 计算航点信息
        total_waypoints = len(st.session_state.planned_path) if st.session_state.planned_path else 1
        if latest.arrived:
            current_waypoint = total_waypoints
        elif latest.progress >= 0 and not latest.arrived:
            if latest.progress < 1.0:
                segment_index = int(latest.progress * (total_waypoints - 1))
                current_waypoint = min(segment_index + 1, total_waypoints)
            else:
                current_waypoint = total_waypoints
        else:
            current_waypoint = 0
        
        # 计算电池电量
        max_flight_time = 1800
        battery_percentage = max(0, min(100, (1 - latest.flight_time / max_flight_time) * 100))
        if latest.voltage:
            voltage_percentage = ((latest.voltage - 21.0) / (22.2 - 21.0)) * 100
            battery_percentage = max(0, min(100, (battery_percentage + voltage_percentage) / 2))
        
        # 计算ETA
        if latest.arrived:
            estimated_arrival = "00:00"
            eta_display = "00:00"
        elif latest.speed > 0 and latest.remaining_distance > 0:
            eta_seconds = latest.remaining_distance / latest.speed
            if eta_seconds < 60:
                eta_display = f"{eta_seconds:.0f}秒"
            elif eta_seconds < 3600:
                minutes, seconds = int(eta_seconds // 60), int(eta_seconds % 60)
                eta_display = f"{minutes:02d}:{seconds:02d}"
            else:
                hours, minutes = int(eta_seconds // 3600), int((eta_seconds % 3600) // 60)
                eta_display = f"{hours:02d}:{minutes:02d}"
            estimated_arrival = eta_display
        else:
            estimated_arrival = "计算中..."
        
        # 飞行进度条
        st.markdown("### ✈️ 飞行进度")
        progress_percent = int(latest.progress * 100) if not latest.arrived else 100
        st.progress(latest.progress if not latest.arrived else 1.0, text=f"飞行进度：{progress_percent}%")
        
        # 主要指标卡片
        st.markdown("### 📊 实时飞行数据")
        col1, col2, col3 = st.columns(3)
        
        with col1:
            waypoint_progress = current_waypoint / total_waypoints if total_waypoints > 0 else 1.0
            st.metric("🎯 当前航点", f"{current_waypoint} / {total_waypoints}", 
                     delta=f"进度 {int(waypoint_progress*100)}%" if not latest.arrived else "已完成")
            st.progress(waypoint_progress, text=f"航点进度: {int(waypoint_progress*100)}%")
        
        with col2:
            st.metric("💨 飞行速度", f"{latest.speed:.1f} m/s", 
                     delta=f"{drone_speed}% 系数" if not latest.arrived else "已到达")
            st.caption(f"≈ {latest.speed * 3.6:.1f} km/h")
        
        with col3:
            minutes, seconds = int(latest.flight_time // 60), int(latest.flight_time % 60)
            st.metric("⏰ 已用时间", f"{minutes:02d}:{seconds:02d}", 
                     delta=f"{latest.flight_time:.1f}秒" if not latest.arrived else "已完成")
        
        col4, col5, col6 = st.columns(3)
        
        with col4:
            distance_text = f"{latest.remaining_distance/1000:.2f} km" if latest.remaining_distance >= 1000 else f"{latest.remaining_distance:.0f} m"
            st.metric("📏 剩余距离", distance_text if not latest.arrived else "0 m", 
                     delta="已到达!" if latest.arrived else None)
        
        with col5:
            st.metric("🕐 预计到达", estimated_arrival, delta=None)
            if latest.remaining_distance < 100 and latest.remaining_distance > 0 and not latest.arrived:
                st.info("🏁 即将到达目的地！")
            elif latest.arrived:
                st.success("✅ 已到达目的地！")
        
        with col6:
            battery_color = "🟢" if battery_percentage > 50 else "🟡" if battery_percentage > 20 else "🔴"
            st.metric("🔋 电量模拟", f"{battery_color} {battery_percentage:.0f}%", 
                     delta=f"{latest.voltage:.1f}V")
            if battery_percentage < 20 and not latest.arrived:
                st.warning("⚠️ 电量不足，请尽快返航！")
        
        # 位置与状态
        st.markdown("### 📍 位置与状态")
        col7, col8, col9, col10 = st.columns(4)
        
        with col7:
            st.metric("📍 当前位置", f"{latest.lat:.6f}, {latest.lng:.6f}")
        with col8:
            st.metric("📏 飞行高度", f"{latest.altitude} m")
        with col9:
            st.metric("🛰️ 卫星数量", f"{latest.satellites} 颗")
        with col10:
            status = "✅ 已完成" if latest.arrived else ("✈️ 飞行中" if st.session_state.simulation_running else "⏸️ 已停止")
            st.metric("📌 飞行状态", status)
        
        if latest.safety_violation and not latest.arrived:
            st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
        
        if latest.arrived:
            st.success("🎉 无人机已到达目的地！飞行任务完成！")
            with st.expander("📊 飞行任务总结", expanded=True):
                col_sum1, col_sum2, col_sum3 = st.columns(3)
                with col_sum1:
                    minutes, seconds = int(latest.flight_time // 60), int(latest.flight_time % 60)
                    st.metric("总飞行时间", f"{minutes:02d}:{seconds:02d}")
                with col_sum2:
                    total_distance = st.session_state.heartbeat_sim.total_distance * 111000
                    st.metric("总飞行距离", f"{total_distance:.0f} m")
                with col_sum3:
                    avg_speed = latest.speed if latest.speed > 0 else drone_speed * config.BASE_SPEED_MPS / 100
                    st.metric("平均速度", f"{avg_speed:.1f} m/s")
        
        st.markdown("---")
        
        # 实时地图
        st.markdown("### 🗺️ 实时位置追踪")
        monitor_map = create_map(
            [latest.lng, latest.lat], st.session_state.points_gcj, st.session_state.obstacles_gcj,
            map_type, flight_alt, st.session_state.planned_path, [latest.lng, latest.lat],
            st.session_state.current_direction, st.session_state.safety_radius, True,
            [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:50]]
        )
        folium_static(monitor_map, width=900, height=500)
        
        st.markdown("---")
        
        # 实时图表（使用Streamlit原生图表）
        st.markdown("### 📈 实时数据图表")
        
        if len(st.session_state.heartbeat_sim.history) > 2:
            # 准备数据
            history_data = st.session_state.heartbeat_sim.history[:30][::-1]
            chart_data = pd.DataFrame({
                '时间': [i * config.HEARTBEAT_INTERVAL for i in range(len(history_data))],
                '速度(m/s)': [h.speed for h in history_data],
                '剩余距离(m)': [max(0, h.remaining_distance) for h in history_data]
            })
            
            # 电池数据
            battery_pct = []
            for h in history_data:
                b = max(0, min(100, (1 - h.flight_time / 1800) * 100))
                if h.voltage:
                    v_pct = ((h.voltage - 21.0) / (22.2 - 21.0)) * 100
                    b = max(0, min(100, (b + v_pct) / 2))
                battery_pct.append(b)
            
            battery_data = pd.DataFrame({
                '时间': [i * config.HEARTBEAT_INTERVAL for i in range(len(history_data))],
                '电量(%)': battery_pct
            })
            
            # 航点数据
            waypoint_pct = []
            for h in history_data:
                if h.arrived or h.progress >= 1.0:
                    wp_pct = 100
                else:
                    wp_pct = (min(int(h.progress * (total_waypoints - 1)) + 1, total_waypoints) / total_waypoints) * 100 if total_waypoints > 0 else 100
                waypoint_pct.append(wp_pct)
            
            waypoint_data = pd.DataFrame({
                '时间': [i * config.HEARTBEAT_INTERVAL for i in range(len(history_data))],
                '航点进度(%)': waypoint_pct
            })
            
            # 速度图表
            st.subheader("📊 速度 vs 时间")
            st.line_chart(chart_data, x='时间', y='速度(m/s)', use_container_width=True)
            
            # 剩余距离图表
            st.subheader("📏 剩余距离 vs 时间")
            st.line_chart(chart_data, x='时间', y='剩余距离(m)', use_container_width=True)
            
            # 电量图表
            st.subheader("🔋 电量模拟 vs 时间")
            st.line_chart(battery_data, x='时间', y='电量(%)', use_container_width=True)
            st.caption("💡 电量基于电压和飞行时间综合计算")
            
            # 航点进度图表
            st.subheader("🎯 航点进度")
            st.line_chart(waypoint_data, x='时间', y='航点进度(%)', use_container_width=True)
        else:
            st.info("⏳ 等待更多数据... (需要至少3个数据点)")
        
        st.markdown("---")
        
        # 飞行日志
        st.markdown("### 📋 飞行日志记录")
        history_df = st.session_state.heartbeat_sim.export_flight_data()
        if not history_df.empty:
            display_cols = ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'voltage', 'satellites', 'remaining_distance']
            display_cols = [c for c in display_cols if c in history_df.columns]
            recent_df = history_df[display_cols].head(10)
            column_names = {'timestamp': '时间', 'flight_time': '飞行时间(s)', 'lat': '纬度', 'lng': '经度',
                           'altitude': '高度(m)', 'speed': '速度(m/s)', 'voltage': '电压(V)',
                           'satellites': '卫星数', 'remaining_distance': '剩余距离(m)'}
            recent_df = recent_df.rename(columns=column_names)
            st.dataframe(recent_df, use_container_width=True)
            
            st.markdown("### 📊 飞行统计")
            col_stat1, col_stat2, col_stat3, col_stat4 = st.columns(4)
            with col_stat1: st.metric("🏁 最高速度", f"{history_df['speed'].max():.1f} m/s")
            with col_stat2: st.metric("📈 平均速度", f"{history_df['speed'].mean():.1f} m/s")
            with col_stat3: st.metric("⛰️ 最高高度", f"{history_df['altitude'].max():.0f} m")
            with col_stat4: st.metric("⏱️ 总飞行时间", f"{history_df['flight_time'].max():.1f} s")
        else:
            st.info("暂无飞行数据")
        
        # 导出按钮
        st.markdown("---")
        col_export1, col_export2, col_export3 = st.columns(3)
        with col_export1:
            if st.button("📊 导出飞行数据", use_container_width=True, type="primary"):
                df = st.session_state.heartbeat_sim.export_flight_data()
                if not df.empty:
                    st.download_button("📥 下载CSV", df.to_csv(index=False), 
                                     f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with col_export2:
            if st.button("🔄 刷新数据", use_container_width=True):
                st.rerun()
        with col_export3:
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = st.session_state.heartbeat_sim.simulating = False
                st.rerun()
                
    else:
        st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        st.markdown("---")
        st.info("💡 提示：先在航线规划页面设置起点和终点，然后点击「开始飞行」按钮启动模拟")

def render_obstacle_page(flight_alt):
    st.header("🚧 障碍物管理")
    
    col1, col2, col3, col4 = st.columns(4)
    col1.info(f"📊 共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    col2.info(f"🛡️ 安全半径: {st.session_state.safety_radius}米")
    
    # 批量操作按钮
    col_btn1, col_btn2, col_btn3, col_btn4, col_btn5 = st.columns(5)
    if col_btn1.button("💾 保存", use_container_width=True):
        if save_obstacles(st.session_state.obstacles_gcj): st.success("✅ 已保存"); st.rerun()
    if col_btn2.button("📂 加载", use_container_width=True):
        loaded = load_obstacles()
        if loaded: st.session_state.obstacles_gcj = loaded; update_path(); st.success(f"✅ 已加载 {len(loaded)} 个"); st.rerun()
    if st.session_state.obstacles_gcj:
        json_str = json.dumps({'obstacles': st.session_state.obstacles_gcj, 'export_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")}, ensure_ascii=False, indent=2)
        col_btn3.download_button("📥 导出", json_str, f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    if col_btn4.button("🗑️ 清除全部", use_container_width=True):
        if st.session_state.auto_backup: save_obstacles([])
        st.session_state.obstacles_gcj = []; update_path(); st.success("✅ 已清除"); st.rerun()
    
    # 统计
    high_cnt = sum(1 for o in st.session_state.obstacles_gcj if o.get('height',30) > flight_alt)
    st.markdown("---")
    st.metric("🔴 需避让障碍物", high_cnt, f"{len(st.session_state.obstacles_gcj)-high_cnt}个安全")
    
    # 列表视图
    st.subheader("📝 障碍物列表")
    if st.checkbox("☑️ 全选"):
        for o in st.session_state.obstacles_gcj: o['selected'] = True
    if st.button("🗑️ 批量删除", use_container_width=True):
        st.session_state.obstacles_gcj = [o for o in st.session_state.obstacles_gcj if not o.get('selected', False)]
        save_obstacles(st.session_state.obstacles_gcj); update_path(); st.rerun()
    
    for i, obs in enumerate(st.session_state.obstacles_gcj):
        with st.container(border=True):
            col_c, col_n = st.columns([1, 5])
            with col_c: obs['selected'] = st.checkbox("", obs.get('selected', False), key=f"sel_{i}")
            with col_n: st.markdown(f"**{'🔴' if obs.get('height',30)>flight_alt else '🟠'} {obs.get('name', f'障碍物{i+1}')}**")
            st.caption(f"📏 高度: {obs.get('height',30)}m | 📍 顶点: {len(obs.get('polygon',[]))}个")
            new_h = st.number_input("调整高度", obs.get('height',30), 1, 200, 5, key=f"h_{i}", label_visibility="collapsed")
            if new_h != obs.get('height',30):
                obs['height'] = new_h
                if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
                update_path(); st.rerun()
            if st.button("🗑️ 删除", key=f"del_{i}", use_container_width=True):
                st.session_state.obstacles_gcj.pop(i); save_obstacles(st.session_state.obstacles_gcj); update_path(); st.rerun()

if __name__ == "__main__":
    main()
