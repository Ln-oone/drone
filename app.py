import streamlit as st
import folium
from streamlit_folium import st_folium, folium_static
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

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 坐标转换模块 ====================
class CoordinateConverter:
    a, ee = 6378245.0, 0.00669342162296594323

    @classmethod
    def _transform_lat(cls, lng: float, lat: float) -> float:
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
        return ret

    @classmethod
    def _transform_lng(cls, lng: float, lat: float) -> float:
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 * math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
        return ret

    @classmethod
    def out_of_china(cls, lng: float, lat: float) -> bool:
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

    @classmethod
    def wgs84_to_gcj02(cls, lng: float, lat: float) -> Tuple[float, float]:
        if cls.out_of_china(lng, lat):
            return lng, lat
        dlat = cls._transform_lat(lng - 105.0, lat - 35.0)
        dlng = cls._transform_lng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - cls.ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.a * (1 - cls.ee)) / (magic * sqrtmagic) * math.pi)
        dlng = (dlng * 180.0) / (cls.a / sqrtmagic * math.cos(radlat) * math.pi)
        return lng + dlng, lat + dlat

    @classmethod
    def gcj02_to_wgs84(cls, lng: float, lat: float) -> Tuple[float, float]:
        if cls.out_of_china(lng, lat):
            return lng, lat
        dlat = cls._transform_lat(lng - 105.0, lat - 35.0)
        dlng = cls._transform_lng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - cls.ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.a * (1 - cls.ee)) / (magic * sqrtmagic) * math.pi)
        dlng = (dlng * 180.0) / (cls.a / sqrtmagic * math.cos(radlat) * math.pi)
        return lng * 2 - (lng + dlng), lat * 2 - (lat + dlat)

    @classmethod
    def calculate_offset(cls, lng: float, lat: float) -> Tuple[float, float]:
        gcj_lng, gcj_lat = cls.wgs84_to_gcj02(lng, lat)
        return gcj_lng - lng, gcj_lat - lat

# ==================== 通信链路模拟器 ====================
@dataclass
class CommunicationLog:
    timestamp: str
    direction: str
    message: str
    details: str = ""

class CommunicationSimulator:
    def __init__(self):
        self.gcs_ip, self.obc_ip, self.fcu_ip = "192.168.1.100", "192.168.1.101", "192.168.1.102"
        self.gcs_online = self.obc_online = self.fcu_online = True
        self.gcs_obc_latency, self.obc_fcu_latency, self.packet_loss_rate = 25, 15, 0.001
        self.logs, self.planning_records = [], []
        self.total_packets_sent = self.total_packets_received = self.total_packets_lost = 0

    def send_message(self, src: str, dst: str, message: str, details: str = "") -> bool:
        self.total_packets_sent += 1
        if not self.check_link_status(src, dst) or random.random() < self.packet_loss_rate:
            self.total_packets_lost += 1
            return False
        time.sleep(self.get_link_delay(src, dst) / 1000)
        self.total_packets_received += 1
        self.logs.insert(0, CommunicationLog(datetime.now().strftime("%H:%M:%S"), f"{src}→{dst}", message, details))
        if len(self.logs) > 100:
            self.logs.pop()
        return True

    def check_link_status(self, src: str, dst: str) -> bool:
        if src in ["GCS", "OBC"] and dst in ["GCS", "OBC"]:
            return self.gcs_online and self.obc_online
        return self.obc_online and self.fcu_online if src in ["OBC", "FCU"] and dst in ["OBC", "FCU"] else False

    def get_link_delay(self, src: str, dst: str) -> float:
        if (src == "GCS" and dst == "OBC") or (src == "OBC" and dst == "GCS"):
            return self.gcs_obc_latency
        return self.obc_fcu_latency

    def get_statistics(self) -> Dict:
        success_rate = (self.total_packets_received / self.total_packets_sent * 100) if self.total_packets_sent else 0
        return {"sent": self.total_packets_sent, "received": self.total_packets_received, "lost": self.total_packets_lost,
                "success_rate": success_rate, "gcs_obc_latency": self.gcs_obc_latency,
                "obc_fcu_latency": self.obc_fcu_latency, "packet_loss_rate": self.packet_loss_rate}

    def add_planning_record(self, record: Dict):
        record["timestamp"] = datetime.now().strftime("%H:%M:%S")
        self.planning_records.insert(0, record)
        if len(self.planning_records) > 20:
            self.planning_records.pop()

    def reset_statistics(self):
        self.total_packets_sent = self.total_packets_received = self.total_packets_lost = 0
        self.logs.clear()
        self.planning_records.clear()

# ==================== 几何函数 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    x, y = point
    inside, n = False, len(polygon)
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

    o1, o2, o3, o4 = orientation(p1, p2, p3), orientation(p1, p2, p4), orientation(p3, p4, p1), orientation(p3, p4, p2)
    if (o1 != o2 and o3 != o4) or (o1 == 0 and min(p1[0], p2[0]) <= p3[0] <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= p3[1] <= max(p1[1], p2[1])):
        return True
    return any(o == 0 and min(p3[0], p4[0]) <= p[0] <= max(p3[0], p4[0]) and min(p3[1], p4[1]) <= p[1] <= max(p3[1], p4[1]) for o, p in [(o2, p4), (o3, p1), (o4, p2)])

def line_intersects_polygon(p1, p2, polygon) -> bool:
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        if segments_intersect(p1, p2, polygon[i], polygon[(i + 1) % n]):
            return True
    return False

def distance(p1, p2) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def point_to_segment_distance_meters(point, seg_start, seg_end) -> float:
    px, py = point
    x1, y1, x2, y2 = seg_start[0], seg_start[1], seg_end[0], seg_end[1]
    dx, dy = x2 - x1, y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0:
        return math.hypot(px - x1, py - y1) * 111000
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / len_sq))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy)) * 111000

def check_safety_radius(drone_pos, obstacles_gcj, flight_altitude, safety_radius):
    if not drone_pos:
        return True, None, None
    min_dist = float('inf')
    danger_name = None
    for obs in obstacles_gcj:
        if obs.get('height', 30) <= flight_altitude:
            continue
        coords = obs.get('polygon', [])
        for i in range(len(coords)):
            dist = point_to_segment_distance_meters(drone_pos, coords[i], coords[(i + 1) % len(coords)])
            if dist < min_dist:
                min_dist, danger_name = dist, obs.get('name', '障碍物')
    if min_dist < safety_radius:
        return False, min_dist, danger_name
    return True, min_dist if min_dist != float('inf') else None, None

# ==================== 障碍物管理 ====================
def load_obstacles() -> List[Dict]:
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                obstacles = json.load(f).get('obstacles', [])
                for obs in obstacles:
                    obs.setdefault('selected', False)
                    obs.setdefault('height', 30)
                return obstacles
        except (json.JSONDecodeError, IOError):
            return []
    return []

def save_obstacles(obstacles: List[Dict]) -> bool:
    try:
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump({'obstacles': obstacles, 'count': len(obstacles),
                       'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 'version': 'v13.2'}, f, ensure_ascii=False, indent=2)
        return True
    except Exception:
        return False

# ==================== 绕行算法 ====================
def get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude):
    return [obs for obs in obstacles_gcj if obs.get('height', 30) > flight_altitude and line_intersects_polygon(start, end, obs.get('polygon', []))]

def is_path_segment_clear(p1, p2, obstacles, flight_altitude, safety_radius):
    for obs in obstacles:
        if obs.get('height', 30) <= flight_altitude:
            continue
        poly = obs.get('polygon', [])
        if poly and line_intersects_polygon(p1, p2, poly):
            return False
        for k in range(51):
            t = k / 50
            point = [p1[0] + (p2[0] - p1[0]) * t, p1[1] + (p2[1] - p1[1]) * t]
            if point_in_polygon(point, poly):
                return False
            for i in range(len(poly)):
                if point_to_segment_distance_meters(point, poly[i], poly[(i + 1) % len(poly)]) < safety_radius:
                    return False
    return True

def find_asymmetric_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5, side="right"):
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking_obs:
        return [start, end]

    min_lng = min(p[0] for obs in blocking_obs for p in obs.get('polygon', []))
    max_lng = max(p[0] for obs in blocking_obs for p in obs.get('polygon', []))
    min_lat = min(p[1] for obs in blocking_obs for p in obs.get('polygon', []))
    max_lat = max(p[1] for obs in blocking_obs for p in obs.get('polygon', []))

    mid_lat = (start[1] + end[1]) / 2
    deg_per_meter_lng = 1 / (111000 * math.cos(math.radians(mid_lat)))

    boundary = max_lng if side == "right" else min_lng
    offset_sign = 1 if side == "right" else -1
    base_offset = safety_radius + (2.5 if side == "right" else 1.5)

    waypoint_lats = list(set([start[1], end[1]] + [min_lat - 0.00002, (min_lat + max_lat) / 2, max_lat + 0.00002]))
    waypoint_lats.sort()

    for attempt in range(1, 15):
        offset_meters = base_offset + safety_radius * attempt * 0.4
        offset_deg = offset_meters * deg_per_meter_lng
        candidate = [start] + [[boundary + offset_sign * offset_deg, lat] for lat in waypoint_lats] + [end]
        if all(is_path_segment_clear(candidate[i], candidate[i+1], blocking_obs, flight_altitude, safety_radius) for i in range(len(candidate)-1)):
            return candidate

    return [start] + [[boundary + offset_sign * (base_offset + safety_radius * 6) * deg_per_meter_lng,
                       start[1] + (end[1] - start[1]) * (i + 1) / (8 + 1)] for i in range(8)] + [end]

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction: str, safety_radius=5):
    if is_path_segment_clear(start, end, obstacles_gcj, flight_altitude, safety_radius):
        return [start, end]
    if direction == "向左绕行":
        return find_asymmetric_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius, "left")
    elif direction == "向右绕行":
        return find_asymmetric_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius, "right")
    else:
        left = find_asymmetric_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius, "left")
        right = find_asymmetric_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius, "right")
        left_len = sum(distance(left[i], left[i+1]) for i in range(len(left)-1))
        right_len = sum(distance(right[i], right[i+1]) for i in range(len(right)-1))
        return left if left_len <= right_len else right

def calculate_path_length(path) -> float:
    return sum(distance(path[i], path[i+1]) for i in range(len(path)-1))

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
        self.history, self.flight_log = [], []
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
        self.last_update_time = None

    def set_path(self, path, altitude=50, speed=50, safety_radius=5):
        self.path, self.path_index, self.current_pos = path, 0, path[0].copy()
        self.flight_altitude, self.speed, self.safety_radius = altitude, speed, safety_radius
        self.simulating, self.progress, self.distance_traveled, self.safety_violation = True, 0.0, 0.0, False
        self.start_time, self.last_update_time = datetime.now(), None
        self.total_distance = sum(distance(path[i], path[i+1]) for i in range(len(path)-1))

    def update_and_generate(self, obstacles_gcj, comm_sim=None):
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
                if comm_sim:
                    comm_sim.send_message("FCU", "OBC", "MISSION_COMPLETE", "任务完成")
            return None

        current_time = time.time()
        delta_time = min(0.5, current_time - self.last_update_time) if self.last_update_time else config.HEARTBEAT_INTERVAL
        self.last_update_time = current_time

        start, end = self.path[self.path_index], self.path[self.path_index + 1]
        segment_distance = distance(start, end)
        move_distance = config.BASE_SPEED_MPS * (self.speed / 100) * delta_time
        self.distance_traveled += move_distance

        # 更新进度
        if self.total_distance > 0:
            completed = sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_index))
            segment_progress = min(1.0, max(0.0, self.distance_traveled / segment_distance)) if segment_distance > 0 else 1.0
            self.progress = min(1.0, (completed + segment_distance * segment_progress) / self.total_distance)

        # 移动到下一段
        if self.distance_traveled >= segment_distance and self.distance_traveled > 0:
            if comm_sim and self.path_index + 1 < len(self.path):
                comm_sim.send_message("FCU", "OBC", f"WP_REACHED #{self.path_index + 1}", f"到达航点 {self.path_index + 1}/{len(self.path)-1}")
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        elif segment_distance > 0:
            t = min(1.0, max(0.0, self.distance_traveled / segment_distance))
            self.current_pos = [start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t]

        # 安全检查
        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        if not safe and not self.safety_violation:
            self.safety_violation = True
            if comm_sim:
                comm_sim.send_message("FCU", "OBC", "SAFETY_VIOLATION", "警告：进入危险区域")

        return self._generate_heartbeat(False)

    def _generate_heartbeat(self, arrived: bool = False) -> HeartbeatData:
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        remaining = 0.0
        if not arrived and self.path_index < len(self.path) - 1:
            remaining = sum(distance(self.current_pos, self.path[self.path_index + 1]) +
                            sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_index + 1, len(self.path)-1))) * 111000

        heartbeat = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"), flight_time=flight_time,
            lat=self.current_pos[1], lng=self.current_pos[0], altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-config.VOLTAGE_VARIATION, config.VOLTAGE_VARIATION), 1),
            satellites=random.randint(*config.SAT_RANGE),
            speed=round(config.BASE_SPEED_MPS * (self.speed / 100), 1),
            progress=self.progress, arrived=arrived, safety_violation=self.safety_violation, remaining_distance=remaining
        )
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        self.flight_log.append(heartbeat)
        return heartbeat

    def export_flight_data(self) -> pd.DataFrame:
        if not self.flight_log:
            return pd.DataFrame()
        return pd.DataFrame([{'timestamp': h.timestamp, 'flight_time': h.flight_time, 'lat': h.lat, 'lng': h.lng,
                              'altitude': h.altitude, 'voltage': h.voltage, 'satellites': h.satellites,
                              'speed': h.speed, 'progress': h.progress, 'arrived': h.arrived,
                              'safety_violation': h.safety_violation, 'remaining_distance': h.remaining_distance} for h in self.flight_log])

# ==================== 地图创建 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history, planned_path, straight_blocked, flight_altitude, drone_pos, direction, safety_radius):
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=config.GAODE_SATELLITE_URL, attr="高德卫星地图")
    m.add_child(plugins.Draw(export=True, position='topleft', draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4}}))

    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if len(coords) >= 3:
            color = "red" if obs.get('height', 30) > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {obs.get('height', 30)}m").add_to(m)

    for key, color, icon, label in [('A', 'green', 'play', '🟢 起点'), ('B', 'red', 'stop', '🔴 终点')]:
        if points_gcj.get(key):
            folium.Marker([points_gcj[key][1], points_gcj[key][0]], popup=label, icon=folium.Icon(color=color, icon=icon, prefix="fa")).add_to(m)

    if planned_path and len(planned_path) > 1:
        line_color = "purple" if "向左" in direction else "orange" if "向右" in direction else "green"
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color=line_color, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        for i, p in enumerate(planned_path[1:-1]):
            folium.CircleMarker([p[1], p[0]], radius=5, color=line_color, fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)

    pos = drone_pos if drone_pos else points_gcj.get('A')
    if pos:
        folium.Circle(radius=safety_radius, location=[pos[1], pos[0]], color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)

    if flight_history and len(flight_history) > 1:
        folium.PolyLine([[p[1], p[0]] for p in flight_history if len(p) >= 2], color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    return m

# ==================== 页面渲染 ====================
def init_session_state():
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(), 'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'comm_sim': CommunicationSimulator(), 'last_hb_time': time.time(), 'simulation_running': False,
        'flight_history': [], 'planned_path': None, 'last_flight_altitude': 50, 'pending_obstacle': None,
        'current_direction': "最佳航线", 'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True, 'show_rename_dialog': False, 'waiting_for_start_point': False,
        'waiting_for_end_point': False, 'temp_click_point': None
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v
    for obs in st.session_state.obstacles_gcj:
        obs.setdefault('height', 30)
        obs.setdefault('selected', False)

def render_sidebar():
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🔗 通信拓扑", "🚧 障碍物管理", "🔄 坐标转换"])
    st.sidebar.markdown("---")
    drone_speed = st.sidebar.slider("飞行速度系数", 10, 100, 50, 5)
    flight_alt = st.sidebar.slider("飞行高度 (m)", 10, 200, 50, 5)
    new_safety = st.sidebar.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
    if new_safety != st.session_state.safety_radius:
        st.session_state.safety_radius = new_safety
        st.session_state.heartbeat_sim.safety_radius = new_safety
        if st.session_state.planned_path and st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
                st.session_state.current_direction, new_safety)
    auto_save = st.sidebar.checkbox("自动保存障碍物", st.session_state.auto_backup)
    return page, drone_speed, flight_alt, auto_save

def main():
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    init_session_state()
    st.title("🏫 无人机地面站系统")
    st.markdown("---")

    page, drone_speed, flight_alt, auto_save = render_sidebar()
    st.session_state.auto_backup = auto_save

    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt,
                st.session_state.current_direction, st.session_state.safety_radius)

    # 简化的页面渲染
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        blocked = any(obs.get('height', 30) > flight_alt and line_intersects_polygon(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], obs.get('polygon', []))
            for obs in st.session_state.obstacles_gcj)
        if blocked:
            st.warning(f"⚠️ 有障碍物高于飞行高度({flight_alt}m)，需要绕行")
        else:
            st.success("✅ 直线航线畅通无阻")

        col1, col2 = st.columns([1, 1.5])
        with col1:
            st.subheader("🎮 控制面板")
            if st.button("🔄 重新规划路径", use_container_width=True):
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt,
                    st.session_state.current_direction, st.session_state.safety_radius)
            if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                st.success(f"🚁 飞行已开始！")
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
        with col2:
            m = create_planning_map(
                st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ,
                st.session_state.points_gcj, st.session_state.obstacles_gcj,
                [hb for hb in st.session_state.heartbeat_sim.history[:20]],
                st.session_state.planned_path, blocked, flight_alt,
                st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None,
                st.session_state.current_direction, st.session_state.safety_radius)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
            if output and output.get("last_clicked"):
                lng, lat = output["last_clicked"]["lng"], output["last_clicked"]["lat"]
                st.session_state.points_gcj['A'] = [lng, lat]
                st.rerun()

    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        if st.session_state.simulation_running and time.time() - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj, st.session_state.comm_sim)
            st.session_state.last_hb_time = time.time()
            st.rerun()

        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            st.progress(latest.progress if not latest.arrived else 1.0, text=f"飞行进度：{int(latest.progress*100) if not latest.arrived else 100}%")
            cols = st.columns(4)
            metrics = [("🎯 当前航点", f"{int(latest.progress * (len(st.session_state.planned_path)-1)) + 1 if st.session_state.planned_path else 0} / {len(st.session_state.planned_path)-1 if st.session_state.planned_path else 0}"),
                       ("💨 飞行速度", f"{latest.speed:.1f} m/s"), ("⏰ 已用时间", f"{int(latest.flight_time//60):02d}:{int(latest.flight_time%60):02d}"),
                       ("📏 剩余距离", f"{latest.remaining_distance:.0f} m")]
            for col, (label, value) in zip(cols, metrics):
                col.metric(label, value)

            if latest.safety_violation:
                st.error("⚠️ 无人机进入危险区域！")
            if latest.arrived:
                st.success("🎉 无人机已到达目的地！")
        else:
            st.info("⏳ 等待心跳数据... 请在航线规划页面点击开始飞行")

    elif page == "🔗 通信拓扑":
        st.header("🔗 通信链路拓扑与数据流")
        comm = st.session_state.comm_sim
        cols = st.columns(3)
        for col, (name, status, ip) in zip(cols, [("GCS", "🟢 在线" if comm.gcs_online else "🔴 离线", comm.gcs_ip),
                                                   ("OBC", "🟢 在线" if comm.obc_online else "🔴 离线", comm.obc_ip),
                                                   ("FCU", "🟢 在线" if comm.fcu_online else "🔴 离线", comm.fcu_ip)]):
            col.metric(f"📡 {name}", status)
            col.caption(f"IP: {ip}")
        st.dataframe(pd.DataFrame(comm.get_statistics(), index=["值"]), use_container_width=True)

    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        for i, obs in enumerate(st.session_state.obstacles_gcj):
            with st.container(border=True):
                col1, col2, col3 = st.columns([3, 1, 1])
                col1.write(f"**{obs.get('name', f'障碍物{i+1}')}** - 高度: {obs.get('height', 30)}m")
                if col2.button("删除", key=f"del_{i}"):
                    st.session_state.obstacles_gcj.pop(i)
                    if st.session_state.auto_backup:
                        save_obstacles(st.session_state.obstacles_gcj)
                    st.rerun()
                new_h = col3.number_input("高度", 1, 200, obs.get('height', 30), key=f"h_{i}", label_visibility="collapsed")
                if new_h != obs.get('height', 30):
                    obs['height'] = new_h
                    if st.session_state.auto_backup:
                        save_obstacles(st.session_state.obstacles_gcj)

    elif page == "🔄 坐标转换":
        st.header("🔄 WGS-84 ↔ GCJ-02 坐标转换")
        col1, col2 = st.columns(2)
        with col1:
            lng = st.number_input("经度", value=118.748726, format="%.6f")
            lat = st.number_input("纬度", value=32.233881, format="%.6f")
            if st.button("WGS-84 → GCJ-02"):
                out_lng, out_lat = CoordinateConverter.wgs84_to_gcj02(lng, lat)
                st.session_state.conv_result = (out_lng, out_lat)
        with col2:
            if "conv_result" in st.session_state:
                st.success(f"GCJ-02: ({st.session_state.conv_result[0]:.8f}, {st.session_state.conv_result[1]:.8f})")

if __name__ == "__main__":
    main()
