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


# ==================== 配置 ====================
@dataclass
class Config:
    SCHOOL_CENTER: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A: List[float] = field(default_factory=lambda: [118.748807, 32.233931])
    DEFAULT_B: List[float] = field(default_factory=lambda: [118.750046, 32.236150])
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    BASE_SPEED: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    GAODE_SATELLITE: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"


config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)


# ==================== 几何工具 ====================
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
    def orient(a, b, c):
        val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        return 0 if abs(val) < 1e-10 else (1 if val > 0 else 2)
    
    def on_segment(p, q, r):
        return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))
    
    o1, o2, o3, o4 = orient(p1, p2, p3), orient(p1, p2, p4), orient(p3, p4, p1), orient(p3, p4, p2)
    if o1 != o2 and o3 != o4: return True
    if o1 == 0 and on_segment(p1, p3, p2): return True
    if o2 == 0 and on_segment(p1, p4, p2): return True
    if o3 == 0 and on_segment(p3, p1, p4): return True
    if o4 == 0 and on_segment(p3, p2, p4): return True
    return False


def line_intersects_polygon(p1, p2, polygon) -> bool:
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    for i in range(len(polygon)):
        if segments_intersect(p1, p2, polygon[i], polygon[(i + 1) % len(polygon)]):
            return True
    return False


def distance_meters(p1, p2) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) * 111000


def meters_to_deg(meters: float, lat: float = 32.23) -> Tuple[float, float]:
    lat_deg = meters / 111000
    return (meters / (111000 * math.cos(math.radians(lat))), lat_deg)


# ==================== 障碍物管理 ====================
def load_obstacles() -> List[Dict]:
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                for obs in data.get('obstacles', []):
                    obs.setdefault('selected', False)
                    obs.setdefault('height', 30)
                return data.get('obstacles', [])
        except: return []
    return []


def save_obstacles(obstacles: List[Dict]) -> bool:
    try:
        if os.path.exists(config.CONFIG_FILE):
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            import shutil
            shutil.copy(config.CONFIG_FILE, f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{timestamp}.bak")
        
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump({'obstacles': obstacles, 'count': len(obstacles), 
                       'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")}, f, ensure_ascii=False, indent=2)
        return True
    except: return False


# ==================== 绕行算法 ====================
def get_blocking_obstacles(start, end, obstacles, flight_alt):
    return [obs for obs in obstacles if obs.get('height', 30) > flight_alt 
            and obs.get('polygon') and line_intersects_polygon(start, end, obs['polygon'])]


def create_avoidance_path(start, end, obstacles, flight_alt, direction="最佳航线", safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles, flight_alt)
    if not blocking:
        return [start, end]
    
    # 计算障碍物边界
    max_lng, max_lat, min_lat = -float('inf'), -float('inf'), float('inf')
    for obs in blocking:
        for p in obs['polygon']:
            max_lng, max_lat, min_lat = max(max_lng, p[0]), max(max_lat, p[1]), min(min_lat, p[1])
    
    if max_lng == -float('inf'):
        return [start, end]
    
    safe_lng, safe_lat = meters_to_deg(safety_radius * 3)
    obstacle_h = max_lat - min_lat
    
    if direction == "向左绕行":
        point1 = [start[0] + 0.0012, max_lat + obstacle_h * 3 + safe_lat * 5 + 0.0002]
        point2 = [max_lng + obstacle_h * 2 + safe_lng * 3, point1[1]]
        return [start, point1, point2, end]
    else:  # 向右绕行
        mid = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2]
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.sqrt(dx * dx + dy * dy)
        if length > 0:
            perp_x, perp_y = dy / length, -dx / length
            offset = safety_radius * 10
            lng_scale, lat_scale = 111000 * math.cos(math.radians(mid[1])), 111000
            waypoint = [mid[0] + perp_x * offset / lng_scale, mid[1] + perp_y * offset / lat_scale]
            return [start, waypoint, end]
        return [start, end]


def select_best_path(start, end, obstacles, flight_alt, safety_radius):
    left = create_avoidance_path(start, end, obstacles, flight_alt, "向左绕行", safety_radius)
    right = create_avoidance_path(start, end, obstacles, flight_alt, "向右绕行", safety_radius)
    return left if sum(distance_meters(left[i], left[i+1]) for i in range(len(left)-1)) < \
                   sum(distance_meters(right[i], right[i+1]) for i in range(len(right)-1)) else right


# ==================== 心跳模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp: str; flight_time: float; lat: float; lng: float; altitude: float
    voltage: float; satellites: int; speed: float; progress: float
    arrived: bool; safety_violation: bool; remaining_distance: float


class HeartbeatSimulator:
    def __init__(self, start_point):
        self.history = []
        self.current_pos = start_point.copy()
        self.path = [start_point.copy()]
        self.path_idx = 0
        self.simulating = False
        self.flight_alt = 50
        self.speed = 50
        self.progress = 0.0
        self.total_dist = 0.0
        self.traveled = 0.0
        self.safety_radius = 5
        self.safety_violation = False
        self.start_time = None
        self.flight_log = []
        self.last_update = None
    
    def set_path(self, path, altitude, speed, safety_radius):
        self.path = path
        self.path_idx = 0
        self.current_pos = path[0].copy()
        self.flight_alt = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.traveled = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update = None
        self.total_dist = sum(distance_meters(path[i], path[i+1]) for i in range(len(path)-1))
    
    def update(self, obstacles):
        if not self.simulating or self.path_idx >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
            return None
        
        now = time.time()
        delta = min(0.5, (now - self.last_update) if self.last_update else config.HEARTBEAT_INTERVAL)
        self.last_update = now
        
        start, end = self.path[self.path_idx], self.path[self.path_idx + 1]
        seg_dist = distance_meters(start, end) / 111000
        move_dist = config.BASE_SPEED * (self.speed / 100) * delta / 111000
        
        self.traveled += move_dist
        self.progress = min(1.0, sum(distance_meters(self.path[i], self.path[i+1]) for i in range(self.path_idx)) / 111000 + 
                                  min(seg_dist, self.traveled) / self.total_dist * 111000 if self.total_dist > 0 else 0)
        
        if self.traveled >= seg_dist and self.traveled > 0:
            self.path_idx += 1
            self.traveled = 0
            if self.path_idx < len(self.path):
                self.current_pos = self.path[self.path_idx].copy()
        elif seg_dist > 0:
            t = min(1.0, self.traveled / seg_dist)
            self.current_pos = [start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t]
        
        arrived = self.path_idx >= len(self.path) - 1
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        # 计算剩余距离
        remaining = 0
        if not arrived:
            remaining = distance_meters(self.current_pos, self.path[-1])
            for i in range(self.path_idx, len(self.path) - 1):
                remaining += distance_meters(self.path[i], self.path[i+1])
        
        hb = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"), flight_time=flight_time,
            lat=self.current_pos[1], lng=self.current_pos[0], altitude=self.flight_alt,
            voltage=round(22.2 + random.uniform(-0.5, 0.5), 1),
            satellites=random.randint(8, 14), speed=round(config.BASE_SPEED * (self.speed / 100), 1),
            progress=self.progress, arrived=arrived,
            safety_violation=self.safety_violation, remaining_distance=remaining
        )
        
        self.history.insert(0, hb)
        if len(self.history) > 100: self.history.pop()
        self.flight_log.append(hb)
        if len(self.flight_log) > 1000: self.flight_log.pop(0)
        
        if arrived: self.simulating = False
        return hb
    
    def export_data(self) -> pd.DataFrame:
        if not self.flight_log: return pd.DataFrame()
        return pd.DataFrame([{k: getattr(h, k) for k in ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 
                              'voltage', 'satellites', 'speed', 'progress', 'arrived', 'safety_violation', 'remaining_distance']} 
                             for h in self.flight_log])


# ==================== 地图创建 ====================
def create_map(center, points, obstacles, path=None, trail=None, drone_pos=None, direction="最佳航线", 
               safety_radius=5, flight_alt=50, map_type="satellite"):
    tiles = config.GAODE_SATELLITE if map_type == "satellite" else config.GAODE_VECTOR
    m = folium.Map(location=[center[1], center[0]], zoom_start=16, tiles=tiles, attr="高德地图")
    m.add_child(plugins.Draw(export=True, position='topleft', draw_options={'polygon': {'allowIntersection': False, 'showArea': True}}))
    
    # 障碍物
    for obs in obstacles:
        if obs.get('polygon') and len(obs['polygon']) >= 3:
            color = "red" if obs.get('height', 30) > flight_alt else "orange"
            folium.Polygon([[c[1], c[0]] for c in obs['polygon']], color=color, weight=3, 
                          fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}").add_to(m)
    
    # 起点终点
    if points.get('A'): folium.Marker([points['A'][1], points['A'][0]], popup="🟢 起点", 
                                      icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points.get('B'): folium.Marker([points['B'][1], points['B'][0]], popup="🔴 终点",
                                      icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    # 路径
    if path and len(path) > 1:
        colors = {"向左绕行": "purple", "向右绕行": "orange", "最佳航线": "green"}
        folium.PolyLine([[p[1], p[0]] for p in path], color=colors.get(direction, "green"), 
                       weight=5, opacity=0.9).add_to(m)
    
    # 直线
    if points.get('A') and points.get('B'):
        blocked = any(obs.get('height', 30) > flight_alt and obs.get('polygon') and 
                     line_intersects_polygon(points['A'], points['B'], obs['polygon']) for obs in obstacles)
        color = "gray" if blocked else "blue"
        folium.PolyLine([[points['A'][1], points['A'][0]], [points['B'][1], points['B'][0]]], 
                       color=color, weight=2, opacity=0.4, dash_array='5,5').add_to(m)
    
    if drone_pos: folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]], 
                                color="blue", fill=True, fill_opacity=0.2).add_to(m)
    if trail and len(trail) > 1: folium.PolyLine([[p[1], p[0]] for p in trail if len(p) >= 2], 
                                                  color="orange", weight=2, opacity=0.6).add_to(m)
    return m


# ==================== 主UI ====================
def init_state():
    defaults = {
        'points': {'A': config.DEFAULT_A.copy(), 'B': config.DEFAULT_B.copy()},
        'obstacles': load_obstacles(), 'sim': HeartbeatSimulator(config.DEFAULT_A.copy()),
        'sim_running': False, 'flight_history': [], 'planned_path': None,
        'direction': "最佳航线", 'safety_radius': 5, 'last_alt': 50,
        'waiting_point': None, 'pending_obstacle': None
    }
    for k, v in defaults.items():
        if k not in st.session_state: st.session_state[k] = v


def update_path():
    st.session_state.planned_path = select_best_path(
        st.session_state.points['A'], st.session_state.points['B'],
        st.session_state.obstacles, st.session_state.last_alt, st.session_state.safety_radius
    ) if st.session_state.direction == "最佳航线" else create_avoidance_path(
        st.session_state.points['A'], st.session_state.points['B'],
        st.session_state.obstacles, st.session_state.last_alt, st.session_state.direction, st.session_state.safety_radius
    )


def main():
    st.set_page_config(page_title="无人机地面站", layout="wide")
    init_state()
    st.title("🏫 无人机地面站系统")
    
    # 侧边栏
    with st.sidebar:
        page = st.radio("功能", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
        map_type = "satellite" if st.radio("地图类型", ["卫星影像", "矢量街道"], index=0) == "卫星影像" else "vector"
        speed = st.slider("速度系数", 10, 100, 50, 5)
        alt = st.slider("飞行高度(m)", 10, 200, st.session_state.last_alt, 5)
        safety = st.slider("安全半径(m)", 1, 20, st.session_state.safety_radius, 1)
        st.session_state.safety_radius = safety
        auto_save = st.checkbox("自动保存", True)
    
    if alt != st.session_state.last_alt:
        st.session_state.last_alt = alt
        update_path()
        st.rerun()
    
    # ==================== 航线规划 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划")
        
        # 检查直线是否被阻挡
        blocked = any(obs.get('height', 30) > alt and obs.get('polygon') and 
                     line_intersects_polygon(st.session_state.points['A'], st.session_state.points['B'], obs['polygon']) 
                     for obs in st.session_state.obstacles)
        if blocked: st.warning("⚠️ 有障碍物高于飞行高度，需要绕行")
        else: st.success("✅ 直线航线畅通")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            # 起点终点设置
            with st.expander("📍 起点/终点", expanded=True):
                mode = st.radio("设置方式", ["✏️ 经纬度输入", "🖱️ 鼠标点击"], horizontal=True)
                
                if mode == "✏️ 经纬度输入":
                    col_a, col_b = st.columns(2)
                    with col_a:
                        a_lat = st.number_input("起点纬度", value=st.session_state.points['A'][1], format="%.6f")
                        a_lng = st.number_input("起点经度", value=st.session_state.points['A'][0], format="%.6f")
                        if st.button("设置A点"): st.session_state.points['A'] = [a_lng, a_lat]; update_path(); st.rerun()
                    with col_b:
                        b_lat = st.number_input("终点纬度", value=st.session_state.points['B'][1], format="%.6f")
                        b_lng = st.number_input("终点经度", value=st.session_state.points['B'][0], format="%.6f")
                        if st.button("设置B点"): st.session_state.points['B'] = [b_lng, b_lat]; update_path(); st.rerun()
                else:
                    col_btn1, col_btn2 = st.columns(2)
                    with col_btn1:
                        if st.button("🎯 设置起点", type="primary"): st.session_state.waiting_point = 'A'; st.rerun()
                    with col_btn2:
                        if st.button("📍 设置终点", type="primary"): st.session_state.waiting_point = 'B'; st.rerun()
                    if st.session_state.waiting_point: st.info(f"👆 请在地图上点击设置{st.session_state.waiting_point}点")
            
            # 路径策略
            with st.expander("🤖 路径规划", expanded=True):
                dirs = {"最佳航线": "🔄", "向左绕行": "⬅️", "向右绕行": "➡️"}
                cols = st.columns(3)
                for i, (name, icon) in enumerate(dirs.items()):
                    with cols[i]:
                        if st.button(f"{icon} {name}", use_container_width=True, 
                                   type="primary" if st.session_state.direction == name else "secondary"):
                            st.session_state.direction = name
                            update_path()
                            st.rerun()
                if st.button("🔄 重新规划"): update_path(); st.rerun()
            
            # 飞行控制
            with st.expander("✈️ 飞行控制", expanded=True):
                if st.button("▶️ 开始飞行", type="primary", use_container_width=True):
                    path = st.session_state.planned_path or [st.session_state.points['A'], st.session_state.points['B']]
                    st.session_state.sim.set_path(path, alt, speed, safety)
                    st.session_state.sim_running = True
                    st.session_state.flight_history = []
                    st.rerun()
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.sim_running = False
                    st.session_state.sim.simulating = False
                    st.rerun()
            
            st.metric("A点", f"({st.session_state.points['A'][0]:.6f}, {st.session_state.points['A'][1]:.6f})")
            st.metric("B点", f"({st.session_state.points['B'][0]:.6f}, {st.session_state.points['B'][1]:.6f})")
            dist = distance_meters(st.session_state.points['A'], st.session_state.points['B'])
            st.caption(f"📏 直线距离: {dist:.0f}米")
        
        with col2:
            # 地图
            if st.session_state.planned_path is None: update_path()
            drone_pos = st.session_state.sim.current_pos if st.session_state.sim.simulating else None
            trail = [[hb.lng, hb.lat] for hb in st.session_state.sim.history[:20]]
            
            m = create_map(st.session_state.points['A'], st.session_state.points, st.session_state.obstacles,
                          st.session_state.planned_path, trail, drone_pos, st.session_state.direction,
                          safety, alt, map_type)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
            
            # 处理点击
            if st.session_state.waiting_point and output.get("last_clicked"):
                click = output["last_clicked"]
                if click and click.get('lng') and click.get('lat'):
                    st.session_state.points[st.session_state.waiting_point] = [click['lng'], click['lat']]
                    st.session_state.waiting_point = None
                    update_path()
                    st.rerun()
            
            # 处理绘图
            if output.get("last_active_drawing"):
                geo = output["last_active_drawing"].get("geometry", {})
                if geo.get("type") == "Polygon" and geo.get("coordinates"):
                    coords = geo["coordinates"][0]
                    poly = [[p[0], p[1]] for p in coords]
                    if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                        st.session_state.pending_obstacle = poly
                        st.rerun()
            
            # 添加障碍物对话框
            if st.session_state.pending_obstacle:
                st.markdown("---")
                st.subheader("📝 添加障碍物")
                name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles)+1}")
                height = st.number_input("高度(米)", 1, 200, 30, 5)
                if st.button("✅ 确认添加"):
                    st.session_state.obstacles.append({
                        "name": name, "polygon": st.session_state.pending_obstacle,
                        "height": height, "selected": False
                    })
                    if auto_save: save_obstacles(st.session_state.obstacles)
                    st.session_state.pending_obstacle = None
                    update_path()
                    st.rerun()
                if st.button("❌ 取消"):
                    st.session_state.pending_obstacle = None
                    st.rerun()
    
    # ==================== 飞行监控 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控")
        
        # 更新模拟
        if st.session_state.sim_running:
            if time.time() - getattr(st, '_last_update', 0) >= config.HEARTBEAT_INTERVAL:
                st._last_update = time.time()
                hb = st.session_state.sim.update(st.session_state.obstacles)
                if hb:
                    st.session_state.flight_history.append([hb.lng, hb.lat])
                    if len(st.session_state.flight_history) > 200: st.session_state.flight_history.pop(0)
                    if not st.session_state.sim.simulating:
                        st.session_state.sim_running = False
                        st.success("🏁 已到达目的地！")
                    st.rerun()
        
        if st.session_state.sim.history:
            latest = st.session_state.sim.history[0]
            
            # 计算航点进度
            total_wp = len(st.session_state.planned_path) if st.session_state.planned_path else 0
            current_wp = min(total_wp, int(latest.progress * (total_wp - 1)) + 1) if total_wp > 0 else 0
            
            # 进度条
            st.progress(latest.progress if not latest.arrived else 1.0, text=f"飞行进度: {int(latest.progress*100)}%")
            
            # 指标卡片
            c1, c2, c3 = st.columns(3)
            c1.metric("🎯 当前航点", f"{current_wp}/{total_wp}")
            c2.metric("💨 速度", f"{latest.speed:.1f} m/s", f"{speed}%系数")
            c3.metric("⏰ 已用时间", f"{int(latest.flight_time//60):02d}:{int(latest.flight_time%60):02d}")
            
            c4, c5, c6 = st.columns(3)
            dist_remaining = latest.remaining_distance
            c4.metric("📏 剩余距离", f"{dist_remaining:.0f}m" if dist_remaining < 1000 else f"{dist_remaining/1000:.2f}km")
            
            eta = "00:00" if latest.arrived else f"{int(dist_remaining/latest.speed//60):02d}:{int(dist_remaining/latest.speed%60):02d}" if latest.speed > 0 else "计算中"
            c5.metric("🕐 预计到达", eta)
            
            battery = max(0, min(100, (1 - latest.flight_time/1800) * 100))
            c6.metric("🔋 电量", f"{battery:.0f}%")
            
            if latest.safety_violation: st.error("⚠️ 进入安全半径危险区域！")
            if latest.arrived: st.success("🎉 飞行任务完成！")
            
            # 实时地图
            m = create_map([latest.lat, latest.lng], st.session_state.points, st.session_state.obstacles,
                          st.session_state.planned_path, st.session_state.flight_history[-50:],
                          [latest.lng, latest.lat], st.session_state.direction, safety, alt, map_type)
            folium_static(m, width=900, height=500)
            
            # 导出数据
            if st.button("📊 导出飞行数据"):
                df = st.session_state.sim.export_data()
                if not df.empty:
                    st.download_button("下载CSV", df.to_csv(index=False), f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv")
        else:
            st.info("⏳ 请在「航线规划」页面点击「开始飞行」")
    
    # ==================== 障碍物管理 ====================
    else:
        st.header("🚧 障碍物管理")
        
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("障碍物数量", len(st.session_state.obstacles))
        high_obs = sum(1 for o in st.session_state.obstacles if o.get('height', 30) > alt)
        c2.metric("需避让", high_obs)
        c3.metric("安全半径", f"{safety}米")
        c4.metric("备份数", len([f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE)]))
        
        # 批量操作
        for obs in st.session_state.obstacles: obs.setdefault('selected', False)
        
        col_ops = st.columns(6)
        with col_ops[0]:
            if st.button("💾 保存"): save_obstacles(st.session_state.obstacles); st.success("已保存")
        with col_ops[1]:
            if st.button("📂 加载"): st.session_state.obstacles = load_obstacles(); update_path(); st.rerun()
        with col_ops[2]:
            if st.button("🗑️ 清除全部"):
                if auto_save: save_obstacles([])
                st.session_state.obstacles = []
                update_path()
                st.rerun()
        
        # 列表视图
        st.subheader("📋 障碍物列表")
        for i, obs in enumerate(st.session_state.obstacles):
            with st.container(border=True):
                col1, col2, col3, col4, col5 = st.columns([0.5, 2, 1, 1, 1])
                with col1: obs['selected'] = st.checkbox("", key=f"sel_{i}", value=obs.get('selected', False))
                with col2: st.write(f"🏢 {obs.get('name')}")
                with col3: st.write(f"高度: {obs.get('height', 30)}m")
                with col4: st.write(f"顶点: {len(obs.get('polygon', []))}")
                with col5:
                    new_h = st.number_input("", value=obs.get('height', 30), key=f"h_{i}", label_visibility="collapsed", step=5)
                    if new_h != obs.get('height', 30):
                        obs['height'] = new_h
                        if auto_save: save_obstacles(st.session_state.obstacles)
                        update_path()
                        st.rerun()
                if st.button("🗑️ 删除", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    if auto_save: save_obstacles(st.session_state.obstacles)
                    update_path()
                    st.rerun()
        
        # 批量删除
        selected = [i for i, o in enumerate(st.session_state.obstacles) if o.get('selected')]
        if selected and st.button(f"🗑️ 批量删除 {len(selected)} 个"):
            for i in reversed(selected): st.session_state.obstacles.pop(i)
            if auto_save: save_obstacles(st.session_state.obstacles)
            update_path()
            st.rerun()


if __name__ == "__main__":
    main()
