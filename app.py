import streamlit as st
import pandas as pd
import numpy as np
import json
import os
import time
import math
import uuid
import copy
from datetime import datetime, timedelta
from enum import Enum
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass, asdict
import folium
from folium import plugins
from streamlit_folium import st_folium, folium_static
from shapely.geometry import Point, Polygon, LineString, MultiPolygon
from shapely.ops import nearest_points
import branca.colormap as cm

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="无人机地面站仿真平台",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# ==================== 常量与几何工具 ====================
EARTH_RADIUS = 6371000  # 地球半径(米)

def meters_to_degrees(meters: float, lat: float) -> float:
    """将米转换为经纬度度数（近似）"""
    lat_rad = math.radians(lat)
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * math.cos(lat_rad)
    return meters / meters_per_deg_lon

def degrees_to_meters(delta_lat: float, delta_lon: float, lat: float) -> Tuple[float, float]:
    """将经纬度差值转换为米"""
    lat_rad = math.radians(lat)
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * math.cos(lat_rad)
    return delta_lat * meters_per_deg_lat, delta_lon * meters_per_deg_lon

def haversine(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """计算两点间距离(米)"""
    R = EARTH_RADIUS
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def point_in_polygon(lon: float, lat: float, polygon: List[Tuple[float, float]]) -> bool:
    """射线法判断点是否在多边形内"""
    point = Point(lon, lat)
    poly = Polygon(polygon)
    return poly.contains(point)

def line_intersects_polygon(p1: Tuple[float, float], p2: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
    """线段是否与多边形相交"""
    line = LineString([p1, p2])
    poly = Polygon(polygon)
    return line.intersects(poly)

def closest_point_on_polygon(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> Tuple[float, float]:
    """点到多边形最近点"""
    pt = Point(point)
    poly = Polygon(polygon)
    nearest = nearest_points(pt, poly)[1]
    return (nearest.x, nearest.y)

# ==================== 数据模型 ====================
@dataclass
class Obstacle:
    id: str
    name: str
    polygon: List[Tuple[float, float]]  # [(lon, lat), ...]
    height: float  # 米
    is_avoid: bool = True

@dataclass
class Waypoint:
    lon: float
    lat: float
    altitude: float
    index: int

@dataclass
class FlightPath:
    waypoints: List[Waypoint]
    strategy: str  # "best", "left", "right"
    is_planned: bool = True

@dataclass
class FlightState:
    current_wp_index: int
    position: Tuple[float, float]  # lon, lat
    altitude: float
    speed: float  # m/s
    distance_remaining: float
    elapsed_time: float
    battery_voltage: float
    gps_satellites: int
    status: str  # "idle", "flying", "completed", "warning"
    progress: float

class HeartbeatSimulator:
    """心跳包模拟器"""
    def __init__(self, total_distance: float, speed: float):
        self.total_distance = total_distance
        self.speed = speed
        self.start_time = None
        self.elapsed = 0.0
        self.distance_covered = 0.0
        
    def reset(self, total_distance: float, speed: float):
        self.total_distance = total_distance
        self.speed = speed
        self.start_time = time.time()
        self.elapsed = 0.0
        self.distance_covered = 0.0
        
    def update(self):
        if self.start_time is None:
            self.start_time = time.time()
        self.elapsed = time.time() - self.start_time
        self.distance_covered = min(self.speed * self.elapsed, self.total_distance)
        progress = self.distance_covered / self.total_distance if self.total_distance > 0 else 0
        return {
            "elapsed": self.elapsed,
            "distance_covered": self.distance_covered,
            "progress": progress,
            "remaining_distance": self.total_distance - self.distance_covered,
            "eta": (self.total_distance - self.distance_covered) / self.speed if self.speed > 0 else 0,
            "voltage": 25.2 - (self.distance_covered / self.total_distance) * 5.0 if self.total_distance > 0 else 25.2,
            "satellites": min(12, int(6 + self.distance_covered / 100)),
            "speed_current": self.speed
        }

# ==================== 避障算法核心 ====================
class ObstacleAvoider:
    @staticmethod
    def is_blocked(start: Tuple[float, float], end: Tuple[float, float], obstacles: List[Obstacle], 
                   safe_radius: float, base_lat: float) -> Tuple[bool, List[Obstacle]]:
        """检测直线是否被障碍物阻挡"""
        blocked_obstacles = []
        for obs in obstacles:
            if not obs.is_avoid:
                continue
            # 简化：检查线段与多边形是否相交
            if line_intersects_polygon(start, end, obs.polygon):
                blocked_obstacles.append(obs)
        return len(blocked_obstacles) > 0, blocked_obstacles
    
    @staticmethod
    def generate_bypass(start: Tuple[float, float], end: Tuple[float, float], 
                       blocking_obs: List[Obstacle], direction: str, 
                       safe_radius: float, base_lat: float) -> List[Tuple[float, float]]:
        """生成绕行路径点"""
        waypoints = [start]
        # 简化的绕行逻辑：对每个阻挡障碍物，计算其边界绕行点
        for obs in blocking_obs:
            # 找到多边形离线段最近的点
            closest = closest_point_on_polygon(start, obs.polygon)
            # 根据方向计算偏移
            offset_angle = math.radians(90 if direction == "left" else -90)
            # 简化偏移量
            offset_meters = safe_radius * 2
            offset_deg = meters_to_degrees(offset_meters, base_lat)
            # 计算垂直方向偏移
            vec_x = end[0] - start[0]
            vec_y = end[1] - start[1]
            length = math.hypot(vec_x, vec_y)
            if length > 0:
                perp_x = -vec_y / length * offset_deg
                perp_y = vec_x / length * offset_deg
                if direction == "right":
                    perp_x, perp_y = -perp_x, -perp_y
                bypass_point = (closest[0] + perp_x, closest[1] + perp_y)
                waypoints.append(bypass_point)
        waypoints.append(end)
        return waypoints
    
    @staticmethod
    def plan_path(start: Tuple[float, float], end: Tuple[float, float], 
                  obstacles: List[Obstacle], strategy: str, 
                  safe_radius: float, altitude: float) -> FlightPath:
        """主路径规划接口"""
        base_lat = (start[1] + end[1]) / 2
        blocked, blocking_obs = ObstacleAvoider.is_blocked(start, end, obstacles, safe_radius, base_lat)
        
        if not blocked:
            # 直线可达
            wp = [Waypoint(start[0], start[1], altitude, 0), 
                  Waypoint(end[0], end[1], altitude, 1)]
            return FlightPath(wp, strategy)
        
        # 根据策略生成绕行
        if strategy == "left":
            bypass_pts = ObstacleAvoider.generate_bypass(start, end, blocking_obs, "left", safe_radius, base_lat)
        elif strategy == "right":
            bypass_pts = ObstacleAvoider.generate_bypass(start, end, blocking_obs, "right", safe_radius, base_lat)
        else:  # best - 选择左右中较短路径
            left_pts = ObstacleAvoider.generate_bypass(start, end, blocking_obs, "left", safe_radius, base_lat)
            right_pts = ObstacleAvoider.generate_bypass(start, end, blocking_obs, "right", safe_radius, base_lat)
            left_dist = sum(haversine(left_pts[i][0], left_pts[i][1], left_pts[i+1][0], left_pts[i+1][1]) for i in range(len(left_pts)-1))
            right_dist = sum(haversine(right_pts[i][0], right_pts[i][1], right_pts[i+1][0], right_pts[i+1][1]) for i in range(len(right_pts)-1))
            bypass_pts = left_pts if left_dist <= right_dist else right_pts
            
        waypoints = [Waypoint(p[0], p[1], altitude, i) for i, p in enumerate(bypass_pts)]
        return FlightPath(waypoints, strategy)

# ==================== 配置管理 ====================
class ConfigManager:
    CONFIG_DIR = "flight_configs"
    MAX_BACKUPS = 10
    
    @classmethod
    def ensure_dir(cls):
        if not os.path.exists(cls.CONFIG_DIR):
            os.makedirs(cls.CONFIG_DIR)
    
    @classmethod
    def save_config(cls, config: Dict, name: str = "current"):
        cls.ensure_dir()
        # 备份轮换
        backup_name = f"{name}_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        current_path = os.path.join(cls.CONFIG_DIR, f"{name}.json")
        if os.path.exists(current_path):
            backup_path = os.path.join(cls.CONFIG_DIR, backup_name)
            with open(current_path, 'r') as f:
                with open(backup_path, 'w') as bf:
                    bf.write(f.read())
        # 清理旧备份
        backups = [f for f in os.listdir(cls.CONFIG_DIR) if f.startswith(f"{name}_backup_")]
        backups.sort(reverse=True)
        for old in backups[cls.MAX_BACKUPS:]:
            os.remove(os.path.join(cls.CONFIG_DIR, old))
        # 保存当前
        with open(current_path, 'w') as f:
            json.dump(config, f, indent=2, default=str)
    
    @classmethod
    def load_config(cls, name: str = "current") -> Dict:
        cls.ensure_dir()
        path = os.path.join(cls.CONFIG_DIR, f"{name}.json")
        if os.path.exists(path):
            with open(path, 'r') as f:
                return json.load(f)
        return None
    
    @classmethod
    def list_backups(cls, name: str = "current") -> List[str]:
        cls.ensure_dir()
        backups = [f for f in os.listdir(cls.CONFIG_DIR) if f.startswith(f"{name}_backup_")]
        return sorted(backups, reverse=True)

# ==================== 飞行仿真引擎 ====================
class FlightSimulator:
    def __init__(self):
        self.flight_path: Optional[FlightPath] = None
        self.state = FlightState(0, (0,0), 0, 0, 0, 0, 0, 0, "idle", 0)
        self.simulator = HeartbeatSimulator(0, 0)
        self.history = []  # 历史轨迹和日志
        
    def set_path(self, flight_path: FlightPath, speed: float, altitude: float):
        self.flight_path = flight_path
        if flight_path and flight_path.waypoints:
            total_dist = self._calculate_total_distance()
            self.simulator.reset(total_dist, speed)
            self.state.altitude = altitude
            self.state.current_wp_index = 0
            self.state.position = (flight_path.waypoints[0].lon, flight_path.waypoints[0].lat)
            self.state.status = "idle"
            
    def _calculate_total_distance(self):
        if not self.flight_path:
            return 0
        total = 0
        wps = self.flight_path.waypoints
        for i in range(len(wps)-1):
            total += haversine(wps[i].lon, wps[i].lat, wps[i+1].lon, wps[i+1].lat)
        return total
        
    def start(self):
        if self.flight_path:
            self.state.status = "flying"
            self.simulator.start_time = time.time()
            self.history = []
            
    def stop(self):
        self.state.status = "idle"
        
    def update(self, speed: float, safe_radius: float, check_obstacles: List[Obstacle]):
        if self.state.status != "flying":
            return
        data = self.simulator.update()
        self.state.elapsed_time = data["elapsed"]
        self.state.distance_remaining = data["remaining_distance"]
        self.state.progress = data["progress"]
        self.state.battery_voltage = data["voltage"]
        self.state.gps_satellites = data["satellites"]
        self.state.speed = data["speed_current"]
        
        # 更新位置插值
        if self.flight_path and self.flight_path.waypoints:
            wps = self.flight_path.waypoints
            total_dist = self.simulator.total_distance
            covered = data["distance_covered"]
            # 简化的位置插值
            if covered >= total_dist:
                self.state.position = (wps[-1].lon, wps[-1].lat)
                self.state.status = "completed"
                st.session_state.flight_completed_flag = True
            else:
                # 找到当前航段
                dist_accum = 0
                for i in range(len(wps)-1):
                    seg_dist = haversine(wps[i].lon, wps[i].lat, wps[i+1].lon, wps[i+1].lat)
                    if covered <= dist_accum + seg_dist:
                        t = (covered - dist_accum) / seg_dist if seg_dist>0 else 0
                        lon = wps[i].lon + t*(wps[i+1].lon - wps[i].lon)
                        lat = wps[i].lat + t*(wps[i+1].lat - wps[i].lat)
                        self.state.position = (lon, lat)
                        self.state.current_wp_index = i
                        break
                    dist_accum += seg_dist
        # 记录历史
        self.history.append({
            "time": datetime.now(),
            "lon": self.state.position[0],
            "lat": self.state.position[1],
            "altitude": self.state.altitude,
            "speed": self.state.speed,
            "battery": self.state.battery_voltage,
            "satellites": self.state.gps_satellites,
            "progress": self.state.progress
        })
        # 安全半径告警
        for obs in check_obstacles:
            if point_in_polygon(self.state.position[0], self.state.position[1], obs.polygon):
                self.state.status = "warning"
                st.session_state.warning_triggered = True
                break

# ==================== 地图构建器 ====================
class MapBuilder:
    @staticmethod
    def create_base_map(center_lon: float, center_lat: float, zoom: int = 15, 
                        map_type: str = "高德矢量街道"):
        """创建高德底图"""
        # 高德瓦片URL
        if map_type == "高德卫星影像":
            tiles = "https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
        else:
            tiles = "https://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
        m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom, tiles=tiles, attr="高德地图")
        return m
    
    @staticmethod
    def add_obstacles(m: folium.Map, obstacles: List[Obstacle], flight_altitude: float):
        for obs in obstacles:
            color = "red" if obs.height > flight_altitude else "orange"
            folium.Polygon(
                locations=obs.polygon,
                color=color,
                fill=True,
                fill_opacity=0.4,
                popup=f"{obs.name}<br>高度:{obs.height}m<br>{'需避让' if obs.height>flight_altitude else '安全'}"
            ).add_to(m)
            
    @staticmethod
    def add_path(m: folium.Map, path: FlightPath, strategy_color: Dict):
        if not path:
            return
        color = strategy_color.get(path.strategy, "blue")
        points = [(wp.lat, wp.lon) for wp in path.waypoints]
        folium.PolyLine(points, color=color, weight=4, opacity=0.8).add_to(m)
        # 添加航点标记
        for i, wp in enumerate(path.waypoints):
            folium.CircleMarker([wp.lat, wp.lon], radius=4, color="white", fill=True, fill_color="blue",
                                popup=f"航点{i+1}").add_to(m)
    
    @staticmethod
    def add_flight_position(m: folium.Map, position: Tuple[float, float], safe_radius: float, base_lat: float):
        folium.Marker([position[1], position[0]], tooltip="无人机", icon=folium.Icon(color="red", icon="plane", prefix="fa")).add_to(m)
        radius_deg = meters_to_degrees(safe_radius, base_lat)
        folium.Circle([position[1], position[0]], radius=radius_deg*111320, color="blue", fill=True, fill_opacity=0.1).add_to(m)

# ==================== Session State初始化 ====================
def init_session():
    defaults = {
        "obstacles": [],
        "flight_paths": {},
        "current_flight_path": None,
        "simulator": FlightSimulator(),
        "start_point": (116.397128, 39.916527),  # 天安门
        "end_point": (116.416383, 39.924049),
        "map_center": (116.397128, 39.916527),
        "drawing_mode": False,
        "temp_polygon": [],
        "flight_running": False,
        "warning_triggered": False,
        "flight_completed_flag": False,
        "selected_obstacles_ids": [],
        "map_type": "高德矢量街道",
        "speed_factor": 0.5,
        "flight_altitude": 100.0,
        "safe_radius": 5.0,
        "auto_save": True,
        "heartbeat_data": [],
        "waypoint_export": None
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v

# ==================== UI回调函数 ====================
def on_auto_save():
    if st.session_state.auto_save:
        config = {
            "obstacles": [asdict(o) for o in st.session_state.obstacles],
            "start": st.session_state.start_point,
            "end": st.session_state.end_point,
            "altitude": st.session_state.flight_altitude,
            "safe_radius": st.session_state.safe_radius
        }
        ConfigManager.save_config(config)

def plan_route():
    start = st.session_state.start_point
    end = st.session_state.end_point
    obstacles = st.session_state.obstacles
    strategy = st.session_state.route_strategy
    safe_radius = st.session_state.safe_radius
    altitude = st.session_state.flight_altitude
    
    # 检测阻挡
    base_lat = (start[1] + end[1])/2
    blocked, _ = ObstacleAvoider.is_blocked(start, end, obstacles, safe_radius, base_lat)
    if blocked:
        st.toast("⚠️ 直线航线被障碍物阻挡，已启用避障绕行", icon="⚠️")
    else:
        st.toast("✅ 直线航线无障碍，可直接飞行", icon="✅")
        
    path = ObstacleAvoider.plan_path(start, end, obstacles, strategy, safe_radius, altitude)
    st.session_state.current_flight_path = path
    st.session_state.flight_paths[strategy] = path
    on_auto_save()
    
def start_flight():
    if st.session_state.current_flight_path:
        st.session_state.simulator.set_path(st.session_state.current_flight_path, 
                                            st.session_state.speed_factor * 15,  # 基础速度15m/s
                                            st.session_state.flight_altitude)
        st.session_state.simulator.start()
        st.session_state.flight_running = True
        st.session_state.flight_completed_flag = False
        st.session_state.warning_triggered = False

def stop_flight():
    st.session_state.simulator.stop()
    st.session_state.flight_running = False

def export_flight_log():
    if st.session_state.simulator.history:
        df = pd.DataFrame(st.session_state.simulator.history)
        csv = df.to_csv(index=False)
        st.download_button("下载飞行日志", csv, "flight_log.csv", "text/csv", key="dl_log")
        
def export_waypoints():
    if st.session_state.current_flight_path:
        wps = st.session_state.current_flight_path.waypoints
        data = [{"index": wp.index, "lon": wp.lon, "lat": wp.lat, "altitude": wp.altitude} for wp in wps]
        df = pd.DataFrame(data)
        csv = df.to_csv(index=False)
        st.download_button("下载航点CSV", csv, "waypoints.csv", "text/csv", key="dl_wp")

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🚁 无人机地面站")
    st.caption("智能仿真平台 v1.0")
    module = st.radio("功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"], index=0)
    st.divider()
    st.session_state.map_type = st.selectbox("地图类型", ["高德卫星影像", "高德矢量街道"])
    st.session_state.speed_factor = st.slider("速度系数", 0.1, 1.0, 0.5, 0.05)
    st.session_state.flight_altitude = st.slider("飞行高度(米)", 10, 200, 100)
    st.session_state.safe_radius = st.slider("安全半径(米)", 1, 20, 5)
    st.session_state.auto_save = st.checkbox("自动保存配置", True)

# ==================== 主内容区域 ====================
init_session()

# 根据模块显示内容
if module == "🗺️ 航线规划":
    col1, col2 = st.columns([3, 1])
    with col1:
        st.subheader("🗺️ 航线规划地图")
        # 地图控件
        m = MapBuilder.create_base_map(st.session_state.map_center[0], st.session_state.map_center[1],
                                       map_type=st.session_state.map_type)
        # 添加障碍物
        MapBuilder.add_obstacles(m, st.session_state.obstacles, st.session_state.flight_altitude)
        # 添加所有规划路径
        colors = {"best": "green", "left": "purple", "right": "orange"}
        for strat, path in st.session_state.flight_paths.items():
            if path:
                MapBuilder.add_path(m, path, colors)
        # 添加飞行器位置
        if st.session_state.flight_running:
            MapBuilder.add_flight_position(m, st.session_state.simulator.state.position, 
                                          st.session_state.safe_radius, st.session_state.map_center[1])
        # 绘图工具
        draw_control = plugins.Draw(export=True, 
                                   draw_options={'polygon': {'allowIntersection': False, 'repeatMode': True}})
        m.add_child(draw_control)
        output = st_folium(m, width=800, height=500)
        
        # 处理地图绘图事件
        if output and output.get("last_active_drawing"):
            feature = output["last_active_drawing"]
            if feature["geometry"]["type"] == "Polygon":
                coords = feature["geometry"]["coordinates"][0]
                coords_lonlat = [(c[0], c[1]) for c in coords[:-1]]  # 闭合环去重最后一点
                # 弹出添加障碍物对话框
                with st.form("new_obs"):
                    name = st.text_input("障碍物名称", f"障碍物{len(st.session_state.obstacles)+1}")
                    height = st.number_input("高度(米)", 10, 300, 100)
                    if st.form_submit_button("添加"):
                        new_obs = Obstacle(id=str(uuid.uuid4()), name=name, polygon=coords_lonlat, height=height)
                        st.session_state.obstacles.append(new_obs)
                        on_auto_save()
                        st.rerun()
                        
    with col2:
        st.subheader("✈️ 航线设置")
        # 起点终点模式
        mode = st.radio("起点终点设置方式", ["手动输入经纬度", "鼠标点击地图"], index=0)
        if mode == "手动输入经纬度":
            start_lon = st.number_input("起点经度", value=st.session_state.start_point[0], format="%.6f")
            start_lat = st.number_input("起点纬度", value=st.session_state.start_point[1], format="%.6f")
            end_lon = st.number_input("终点经度", value=st.session_state.end_point[0], format="%.6f")
            end_lat = st.number_input("终点纬度", value=st.session_state.end_point[1], format="%.6f")
            st.session_state.start_point = (start_lon, start_lat)
            st.session_state.end_point = (end_lon, end_lat)
        else:
            # 点击地图获取坐标（简化：使用上次点击）
            if output and output.get("last_clicked"):
                click = output["last_clicked"]
                if st.button("设为起点"):
                    st.session_state.start_point = (click["lng"], click["lat"])
                if st.button("设为终点"):
                    st.session_state.end_point = (click["lng"], click["lat"])
        col_a, col_b = st.columns(2)
        with col_a:
            if st.button("重置默认起点"):
                st.session_state.start_point = (116.397128, 39.916527)
        with col_b:
            if st.button("重置默认终点"):
                st.session_state.end_point = (116.416383, 39.924049)
        dist = haversine(st.session_state.start_point[0], st.session_state.start_point[1],
                        st.session_state.end_point[0], st.session_state.end_point[1])
        st.metric("直线距离", f"{dist:.1f} 米")
        
        st.subheader("🛣️ 路径规划")
        st.session_state.route_strategy = st.selectbox("绕行策略", ["best", "left", "right"], 
                                                       format_func=lambda x: {"best":"最佳航线","left":"向左绕行","right":"向右绕行"}[x])
        if st.button("生成航线", type="primary"):
            plan_route()
        # 飞行控制
        st.subheader("🎮 飞行控制")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("开始飞行") and not st.session_state.flight_running:
                start_flight()
        with c2:
            if st.button("停止飞行"):
                stop_flight()
        st.info(f"绕行点数量: {len(st.session_state.current_flight_path.waypoints) if st.session_state.current_flight_path else 0}")
        
elif module == "📡 飞行监控":
    st.subheader("📈 实时飞行数据")
    tab1, tab2, tab3 = st.tabs(["实时数据", "监控图表", "飞行轨迹"])
    with tab1:
        # 心跳数据看板
        if st.session_state.flight_running:
            sim = st.session_state.simulator
            sim.update(st.session_state.speed_factor*15, st.session_state.safe_radius, st.session_state.obstacles)
            state = sim.state
            # 实时指标
            m1, m2, m3, m4 = st.columns(4)
            m1.metric("进度", f"{state.progress*100:.1f}%")
            m2.metric("当前速度", f"{state.speed:.1f} m/s")
            m3.metric("剩余距离", f"{state.distance_remaining:.0f} m")
            m4.metric("电压", f"{state.battery_voltage:.1f} V")
            m5, m6, m7, m8 = st.columns(4)
            m5.metric("卫星数", state.gps_satellites)
            m6.metric("已用时间", f"{state.elapsed_time:.0f}s")
            m7.metric("航点", f"{state.current_wp_index+1}/{len(sim.flight_path.waypoints) if sim.flight_path else 0}")
            m8.metric("状态", state.status)
            if st.session_state.warning_triggered:
                st.error("⚠️ 闯入安全半径！危险告警！")
            if st.session_state.flight_completed_flag:
                st.success("✅ 任务完成！已抵达终点。")
            # 导出按钮
            col_export1, col_export2 = st.columns(2)
            with col_export1:
                export_flight_log()
            with col_export2:
                export_waypoints()
        else:
            st.info("未在飞行中，请前往航线规划页面开始飞行")
    with tab2:
        # 可视化图表
        if st.session_state.simulator.history:
            df_hist = pd.DataFrame(st.session_state.simulator.history)
            st.line_chart(df_hist.set_index("time")[["speed", "battery", "progress"]])
        else:
            st.info("暂无飞行数据")
    with tab3:
        # 实时地图追踪
        m_track = MapBuilder.create_base_map(st.session_state.map_center[0], st.session_state.map_center[1],
                                             map_type=st.session_state.map_type)
        MapBuilder.add_obstacles(m_track, st.session_state.obstacles, st.session_state.flight_altitude)
        if st.session_state.current_flight_path:
            MapBuilder.add_path(m_track, st.session_state.current_flight_path, {"best":"green"})
        if st.session_state.flight_running:
            MapBuilder.add_flight_position(m_track, st.session_state.simulator.state.position,
                                          st.session_state.safe_radius, st.session_state.map_center[1])
        # 历史轨迹
        if st.session_state.simulator.history:
            hist_points = [(h["lat"], h["lon"]) for h in st.session_state.simulator.history]
            folium.PolyLine(hist_points, color="gray", weight=2, opacity=0.5).add_to(m_track)
        folium_static(m_track, width=700, height=400)
        
else:  # 障碍物管理
    st.subheader("🚧 障碍物批量管理")
    col_left, col_right = st.columns([2, 1])
    with col_left:
        # 列表视图
        st.write("**障碍物列表**")
        if not st.session_state.obstacles:
            st.info("暂无障碍物，请在航线规划页面绘制添加")
        else:
            # 批量操作栏
            batch_col1, batch_col2, batch_col3, batch_col4 = st.columns(4)
            with batch_col1:
                if st.button("全选"):
                    st.session_state.selected_obstacles_ids = [o.id for o in st.session_state.obstacles]
            with batch_col2:
                if st.button("批量删除"):
                    st.session_state.obstacles = [o for o in st.session_state.obstacles if o.id not in st.session_state.selected_obstacles_ids]
                    st.session_state.selected_obstacles_ids = []
                    on_auto_save()
                    st.rerun()
            with batch_col3:
                new_h = st.number_input("统一高度", value=50)
                if st.button("批量设置高度"):
                    for obs in st.session_state.obstacles:
                        if obs.id in st.session_state.selected_obstacles_ids:
                            obs.height = new_h
                    on_auto_save()
            with batch_col4:
                st.download_button("导出JSON配置", 
                                   data=json.dumps([asdict(o) for o in st.session_state.obstacles], indent=2),
                                   file_name="obstacles.json")
            # 卡片列表
            for obs in st.session_state.obstacles:
                with st.expander(f"{obs.name} (高度:{obs.height}m)", expanded=False):
                    col_a, col_b = st.columns(2)
                    with col_a:
                        new_name = st.text_input("名称", obs.name, key=f"name_{obs.id}")
                        new_h = st.number_input("高度", value=obs.height, key=f"height_{obs.id}")
                        if st.button("更新", key=f"upd_{obs.id}"):
                            obs.name = new_name
                            obs.height = new_h
                            on_auto_save()
                            st.rerun()
                    with col_b:
                        st.write(f"顶点数: {len(obs.polygon)}")
                        st.write(f"需避让: {'是' if obs.height>st.session_state.flight_altitude else '否'}")
                        if st.button("删除", key=f"del_{obs.id}"):
                            st.session_state.obstacles.remove(obs)
                            on_auto_save()
                            st.rerun()
                    # 地图预览小窗口
                    m_preview = MapBuilder.create_base_map(obs.polygon[0][0], obs.polygon[0][1], zoom=16)
                    folium.Polygon(obs.polygon, color="red", fill=True).add_to(m_preview)
                    folium_static(m_preview, width=300, height=200)
    with col_right:
        st.write("**统计与备份**")
        total = len(st.session_state.obstacles)
        avoid = sum(1 for o in st.session_state.obstacles if o.height > st.session_state.flight_altitude)
        safe_num = total - avoid
        vertices = sum(len(o.polygon) for o in st.session_state.obstacles)
        avg_h = np.mean([o.height for o in st.session_state.obstacles]) if total>0 else 0
        st.metric("障碍物总数", total)
        st.metric("需避让/安全", f"{avoid} / {safe_num}")
        st.metric("总顶点数", vertices)
        st.metric("平均高度", f"{avg_h:.1f} m")
        st.divider()
        if st.button("清空全部障碍物"):
            st.session_state.obstacles = []
            on_auto_save()
            st.rerun()
        if st.button("恢复最近备份"):
            backups = ConfigManager.list_backups()
            if backups:
                cfg = ConfigManager.load_config(backups[0].replace(".json",""))
                if cfg and "obstacles" in cfg:
                    st.session_state.obstacles = [Obstacle(**o) for o in cfg["obstacles"]]
                    st.rerun()
        st.write("**配置保存**")
        if st.button("手动保存配置"):
            on_auto_save()
            st.success("已保存")

# ==================== 自动保存定时器 ====================
if st.session_state.auto_save:
    # 利用session state 的变化触发保存，此处简单起见和主要操作联动
    pass

# 页面底部初始化完成
st.sidebar.markdown("---")
st.sidebar.caption("仿真平台 | 智能避障 | 实时监控")
