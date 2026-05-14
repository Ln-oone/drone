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

# ==================== 配置 ====================
@dataclass
class Config:
    SCHOOL_CENTER: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A: List[float] = field(default_factory=lambda: [118.748807, 32.233931])
    DEFAULT_B: List[float] = field(default_factory=lambda: [118.750046, 32.236150])
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    SAFETY_RADIUS: int = 5
    BASE_SPEED: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    
    GAODE_SATELLITE: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 工具函数 ====================
def distance(p1: List[float], p2: List[float]) -> float:
    """两点间距离（度）"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def meters_to_deg(meters: float, lat: float = 32.23) -> Tuple[float, float]:
    """米转度数"""
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    """射线法判断点是否在多边形内"""
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
    """判断线段相交"""
    def orient(a, b, c):
        val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        return 0 if abs(val) < 1e-10 else (1 if val > 0 else 2)
    
    o1 = orient(p1, p2, p3)
    o2 = orient(p1, p2, p4)
    o3 = orient(p3, p4, p1)
    o4 = orient(p3, p4, p2)
    
    return (o1 != o2 and o3 != o4) or (o1 == 0 and min(p1[0],p2[0])<=p3[0]<=max(p1[0],p2[0]) and min(p1[1],p2[1])<=p3[1]<=max(p1[1],p2[1]))

def line_intersects_polygon(p1, p2, polygon) -> bool:
    """线段与多边形相交检测"""
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    for i in range(len(polygon)):
        if segments_intersect(p1, p2, polygon[i], polygon[(i+1)%len(polygon)]):
            return True
    return False

def point_to_segment_distance_meters(point, seg_start, seg_end) -> float:
    """点到线段距离（米）"""
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    dx, dy = x2 - x1, y2 - y1
    len_sq = dx*dx + dy*dy
    if len_sq == 0:
        return math.sqrt((px-x1)**2 + (py-y1)**2) * 111000
    t = max(0, min(1, ((px-x1)*dx + (py-y1)*dy) / len_sq))
    proj_x = x1 + t*dx
    proj_y = y1 + t*dy
    return math.sqrt((px-proj_x)**2 + (py-proj_y)**2) * 111000

# ==================== 障碍物管理 ====================
def load_obstacles() -> List[Dict]:
    """加载障碍物"""
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                return json.load(f).get('obstacles', [])
        except:
            return []
    return []

def save_obstacles(obstacles: List[Dict]) -> bool:
    """保存障碍物"""
    try:
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump({'obstacles': obstacles, 'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")}, f, ensure_ascii=False, indent=2)
        return True
    except:
        return False

# ==================== 避障算法 ====================
def get_blocking_obstacles(start, end, obstacles, altitude) -> List[Dict]:
    """获取阻挡航线的障碍物"""
    return [obs for obs in obstacles if obs.get('height', 30) > altitude and 
            line_intersects_polygon(start, end, obs.get('polygon', []))]

def create_avoidance_path(start, end, obstacles, altitude, direction="最佳航线", safety_radius=5) -> List[List[float]]:
    """创建避障路径"""
    blocking = get_blocking_obstacles(start, end, obstacles, altitude)
    if not blocking:
        return [start, end]
    
    # 计算障碍物边界
    min_lng, max_lng = float('inf'), -float('inf')
    min_lat, max_lat = float('inf'), -float('inf')
    for obs in blocking:
        for p in obs.get('polygon', []):
            min_lng, max_lng = min(min_lng, p[0]), max(max_lng, p[0])
            min_lat, max_lat = min(min_lat, p[1]), max(max_lat, p[1])
    
    mid_x, mid_y = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.sqrt(dx*dx + dy*dy) or 1
    
    # 计算偏移方向
    if direction == "向左绕行":
        perp_x, perp_y = -dy/length, dx/length
    elif direction == "向右绕行":
        perp_x, perp_y = dy/length, -dx/length
    else:  # 最佳航线：选择偏移较小的方向
        perp_x, perp_y = -dy/length, dx/length
    
    # 偏移距离
    obstacle_width = (max_lng - min_lng) * 111000 * math.cos(math.radians(mid_y))
    offset_dist = obstacle_width/2 + safety_radius * 10
    lat_rad = math.radians(mid_y)
    offset_x = perp_x * offset_dist / (111000 * math.cos(lat_rad))
    offset_y = perp_y * offset_dist / 111000
    
    waypoint = [mid_x + offset_x, mid_y + offset_y]
    return [start, waypoint, end]

# ==================== 心跳模拟器 ====================
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
    remaining_distance: float

class HeartbeatSimulator:
    def __init__(self):
        self.history: List[HeartbeatData] = []
        self.current_pos = config.DEFAULT_A.copy()
        self.path = [config.DEFAULT_A.copy()]
        self.path_index = 0
        self.simulating = False
        self.altitude = 50
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.safety_radius = config.SAFETY_RADIUS
        self.start_time = None
        self.flight_log = []
        
    def set_path(self, path, altitude=50, speed=50, safety_radius=5):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.start_time = datetime.now()
        self.total_distance = sum(distance(path[i], path[i+1]) for i in range(len(path)-1))
        
    def update(self, obstacles) -> Optional[HeartbeatData]:
        if not self.simulating or self.path_index >= len(self.path) - 1:
            self.simulating = False
            return None
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = distance(start, end)
        
        speed_mps = config.BASE_SPEED * (self.speed / 100)
        move_distance = speed_mps * config.HEARTBEAT_INTERVAL
        self.distance_traveled += move_distance
        
        # 更新进度
        if self.total_distance > 0:
            completed = sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_index))
            completed += min(segment_distance, self.distance_traveled)
            self.progress = min(1.0, completed / self.total_distance)
        
        # 更新位置
        if self.distance_traveled >= segment_distance:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
        else:
            t = self.distance_traveled / segment_distance
            self.current_pos = [start[0] + (end[0]-start[0])*t, start[1] + (end[1]-start[1])*t]
        
        return self._generate_heartbeat(self.path_index >= len(self.path) - 1)
    
    def _generate_heartbeat(self, arrived):
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        # 计算剩余距离
        if arrived:
            remaining = 0
        else:
            remaining = distance(self.current_pos, self.path[-1]) * 111000
        
        hb = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=flight_time,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.altitude,
            voltage=22.2 + random.uniform(-0.5, 0.5),
            satellites=random.randint(8, 14),
            speed=round(config.BASE_SPEED * (self.speed / 100), 1),
            progress=self.progress,
            arrived=arrived,
            remaining_distance=remaining
        )
        
        self.history.insert(0, hb)
        if len(self.history) > 100:
            self.history.pop()
        self.flight_log.append(hb)
        return hb

# ==================== 地图创建 ====================
def create_map(center, points, obstacles, path=None, map_type="satellite", drone_pos=None, direction="最佳航线"):
    """创建地图"""
    tiles = config.GAODE_SATELLITE if map_type == "satellite" else config.GAODE_VECTOR
    m = folium.Map(location=[center[1], center[0]], zoom_start=16, tiles=tiles, attr="高德地图")
    
    # 绘制障碍物
    for obs in obstacles:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=2, fill=True, 
                          fill_opacity=0.3, popup=f"{obs.get('name')}\n高度: {obs.get('height', 30)}m").add_to(m)
    
    # 绘制起点终点
    if points.get('A'):
        folium.Marker([points['A'][1], points['A'][0]], popup="起点", icon=folium.Icon(color="green")).add_to(m)
    if points.get('B'):
        folium.Marker([points['B'][1], points['B'][0]], popup="终点", icon=folium.Icon(color="red")).add_to(m)
    
    # 绘制路径
    if path and len(path) > 1:
        colors = {"向左绕行": "purple", "向右绕行": "orange", "最佳航线": "green"}
        folium.PolyLine([[p[1], p[0]] for p in path], color=colors.get(direction, "blue"), 
                       weight=4, opacity=0.8).add_to(m)
    
    # 绘制无人机
    if drone_pos:
        folium.Circle([drone_pos[1], drone_pos[0]], radius=config.SAFETY_RADIUS, color="blue", 
                     fill=True, fill_opacity=0.2).add_to(m)
    
    m.add_child(plugins.Draw(export=True))
    return m

# ==================== UI组件 ====================
def init_session():
    """初始化会话状态"""
    defaults = {
        'points': {'A': config.DEFAULT_A.copy(), 'B': config.DEFAULT_B.copy()},
        'obstacles': load_obstacles(),
        'simulator': HeartbeatSimulator(),
        'simulating': False,
        'planned_path': None,
        'direction': "最佳航线",
        'altitude': 50,
        'speed': 50,
        'safety_radius': config.SAFETY_RADIUS,
        'auto_save': True,
        'pending_obstacle': None
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v

def update_path():
    """更新路径"""
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points['A'], st.session_state.points['B'],
        st.session_state.obstacles, st.session_state.altitude,
        st.session_state.direction, st.session_state.safety_radius
    )

# ==================== 页面 ====================
def planning_page():
    """航线规划页面"""
    st.header("🗺️ 航线规划")
    
    # 检查直线是否被阻挡
    blocked = any(obs.get('height', 30) > st.session_state.altitude and 
                  line_intersects_polygon(st.session_state.points['A'], st.session_state.points['B'], obs.get('polygon', []))
                  for obs in st.session_state.obstacles)
    
    if blocked:
        st.warning("⚠️ 航线被阻挡，已自动规划绕行路径")
    else:
        st.success("✅ 航线畅通")
    
    # 控制面板
    col1, col2 = st.columns([1, 1.5])
    
    with col1:
        st.subheader("🎮 控制面板")
        
        # 起点终点设置
        with st.expander("📍 起点/终点", expanded=True):
            col_a1, col_a2 = st.columns(2)
            with col_a1:
                a_lat = st.number_input("起点纬度", value=st.session_state.points['A'][1], format="%.6f")
                a_lng = st.number_input("起点经度", value=st.session_state.points['A'][0], format="%.6f")
            if st.button("设置起点"):
                st.session_state.points['A'] = [a_lng, a_lat]
                update_path()
            
            col_b1, col_b2 = st.columns(2)
            with col_b1:
                b_lat = st.number_input("终点纬度", value=st.session_state.points['B'][1], format="%.6f")
                b_lng = st.number_input("终点经度", value=st.session_state.points['B'][0], format="%.6f")
            if st.button("设置终点"):
                st.session_state.points['B'] = [b_lng, b_lat]
                update_path()
        
        # 绕行策略
        with st.expander("🤖 绕行策略", expanded=True):
            direction = st.radio("选择方向", ["最佳航线", "向左绕行", "向右绕行"], horizontal=True)
            if direction != st.session_state.direction:
                st.session_state.direction = direction
                update_path()
            
            if st.button("🔄 重新规划"):
                update_path()
                st.success("路径已更新")
        
        # 飞行设置
        with st.expander("✈️ 飞行设置", expanded=True):
            st.session_state.altitude = st.slider("飞行高度(m)", 10, 200, st.session_state.altitude, 5)
            st.session_state.speed = st.slider("速度系数(%)", 10, 100, st.session_state.speed, 5)
            st.session_state.safety_radius = st.slider("安全半径(m)", 1, 20, st.session_state.safety_radius, 1)
            
            if st.button("▶️ 开始飞行", type="primary"):
                path = st.session_state.planned_path or [st.session_state.points['A'], st.session_state.points['B']]
                st.session_state.simulator.set_path(path, st.session_state.altitude, st.session_state.speed, st.session_state.safety_radius)
                st.session_state.simulating = True
                st.session_state.flight_history = []
                st.success("飞行已开始")
                st.rerun()
            
            if st.button("⏹️ 停止飞行"):
                st.session_state.simulating = False
                st.session_state.simulator.simulating = False
    
    with col2:
        st.subheader("🗺️ 规划地图")
        m = create_map(config.SCHOOL_CENTER, st.session_state.points, st.session_state.obstacles,
                      st.session_state.planned_path, st.session_state.get('map_type', 'satellite'),
                      st.session_state.simulator.current_pos if st.session_state.simulating else None,
                      st.session_state.direction)
        
        output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
        
        # 处理新绘制的障碍物
        if output and output.get("last_active_drawing"):
            geom = output["last_active_drawing"].get("geometry", {})
            if geom.get("type") == "Polygon":
                coords = geom.get("coordinates", [[]])[0]
                poly = [[p[0], p[1]] for p in coords]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                    st.session_state.pending_obstacle = poly
                    st.rerun()
        
        # 障碍物对话框
        if st.session_state.pending_obstacle:
            st.markdown("---")
            st.subheader("📝 添加障碍物")
            name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles)+1}")
            height = st.number_input("高度(m)", 1, 200, 30, 5)
            
            col_ok, col_cancel = st.columns(2)
            if col_ok.button("确认"):
                st.session_state.obstacles.append({"name": name, "polygon": st.session_state.pending_obstacle, "height": height})
                if st.session_state.auto_save:
                    save_obstacles(st.session_state.obstacles)
                update_path()
                st.session_state.pending_obstacle = None
                st.rerun()
            if col_cancel.button("取消"):
                st.session_state.pending_obstacle = None
                st.rerun()

def monitoring_page():
    """飞行监控页面"""
    st.header("📡 飞行监控")
    
    # 更新模拟
    if st.session_state.simulating:
        hb = st.session_state.simulator.update(st.session_state.obstacles)
        if hb:
            if not st.session_state.simulator.simulating:
                st.session_state.simulating = False
                st.success("🏁 已到达目的地！")
            st.rerun()
    
    if st.session_state.simulator.history:
        latest = st.session_state.simulator.history[0]
        
        # 进度条
        st.progress(latest.progress, text=f"飞行进度: {int(latest.progress*100)}%")
        
        # 指标卡片
        col1, col2, col3, col4 = st.columns(4)
        col1.metric("速度", f"{latest.speed:.1f} m/s")
        col2.metric("高度", f"{latest.altitude} m")
        col3.metric("剩余距离", f"{latest.remaining_distance:.0f} m")
        col4.metric("卫星", f"{latest.satellites} 颗")
        
        col5, col6, col7, col8 = st.columns(4)
        col5.metric("电压", f"{latest.voltage:.1f} V")
        col6.metric("飞行时间", f"{latest.flight_time:.0f} s")
        col7.metric("进度", f"{int(latest.progress*100)}%")
        col8.metric("状态", "已到达" if latest.arrived else "飞行中")
        
        # 地图
        m = create_map(config.SCHOOL_CENTER, st.session_state.points, st.session_state.obstacles,
                      st.session_state.planned_path, st.session_state.get('map_type', 'satellite'),
                      [latest.lng, latest.lat], st.session_state.direction)
        folium_static(m, width=900, height=450)
        
        # 实时图表
        if len(st.session_state.simulator.history) > 1:
            df = pd.DataFrame([{
                'time': i * config.HEARTBEAT_INTERVAL,
                'speed': h.speed,
                'remaining': max(0, h.remaining_distance)
            } for i, h in enumerate(reversed(st.session_state.simulator.history[-30:]))])
            
            col_ch1, col_ch2 = st.columns(2)
            col_ch1.line_chart(df, x='time', y='speed')
            col_ch2.line_chart(df, x='time', y='remaining')
    else:
        st.info("⏳ 等待飞行数据，请在航线规划页面开始飞行")

def obstacle_page():
    """障碍物管理页面"""
    st.header("🚧 障碍物管理")
    
    col1, col2, col3, col4 = st.columns(4)
    col1.metric("障碍物数量", len(st.session_state.obstacles))
    col2.metric("需避让", sum(1 for o in st.session_state.obstacles if o.get('height', 30) > st.session_state.altitude))
    
    if col3.button("💾 保存"):
        if save_obstacles(st.session_state.obstacles):
            st.success("已保存")
    if col4.button("🗑️ 清空"):
        st.session_state.obstacles = []
        save_obstacles([])
        update_path()
        st.rerun()
    
    # 障碍物列表
    for i, obs in enumerate(st.session_state.obstacles):
        with st.expander(f"{obs.get('name')} - 高度: {obs.get('height', 30)}m"):
            col_e1, col_e2 = st.columns([3, 1])
            with col_e1:
                new_name = st.text_input("名称", obs.get('name'), key=f"name_{i}")
                new_height = st.number_input("高度", 1, 200, obs.get('height', 30), 5, key=f"height_{i}")
            with col_e2:
                if st.button("删除", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    update_path()
                    st.rerun()
            
            if new_name != obs.get('name') or new_height != obs.get('height', 30):
                obs['name'] = new_name
                obs['height'] = new_height
                if st.session_state.auto_save:
                    save_obstacles(st.session_state.obstacles)
                update_path()

# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="无人机地面站", layout="wide")
    init_session()
    
    st.title("🚁 无人机地面站系统")
    st.markdown("---")
    
    # 侧边栏
    page = st.sidebar.radio("功能", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type = st.sidebar.radio("地图", ["卫星影像", "矢量街道"])
    st.session_state['map_type'] = "satellite" if map_type == "卫星影像" else "vector"
    st.session_state.auto_save = st.sidebar.checkbox("自动保存", st.session_state.auto_save)
    
    # 更新高度变化时的路径
    if st.session_state.altitude != st.session_state.get('_last_alt', 0):
        st.session_state['_last_alt'] = st.session_state.altitude
        update_path()
    
    # 页面路由
    if page == "🗺️ 航线规划":
        planning_page()
    elif page == "📡 飞行监控":
        monitoring_page()
    else:
        obstacle_page()

if __name__ == "__main__":
    main()
