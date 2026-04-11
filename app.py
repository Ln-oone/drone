import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
import random
import time
import math
import json
import os
from datetime import datetime
import pandas as pd

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统", layout="wide")

# ==================== 南京科技职业学院坐标 ====================
# 学校中心坐标 (GCJ-02 高德坐标系，便于输入)
SCHOOL_CENTER_GCJ = [118.6965, 32.2015]  # [经度, 纬度]

# A点 (教学楼区域) B点 (操场区域) - GCJ-02 坐标
DEFAULT_A_GCJ = [118.6940, 32.2000]
DEFAULT_B_GCJ = [118.6990, 32.2030]

# ==================== 配置文件路径 ====================
CONFIG_FILE = "obstacle_config.json"

# ==================== 坐标系转换函数 ====================
def gcj02_to_wgs84(lng, lat):
    """GCJ02转WGS84 (OpenStreetMap使用WGS84)"""
    a = 6378245.0
    ee = 0.00669342162296594323
    
    if out_of_china(lng, lat):
        return lng, lat
    
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def wgs84_to_gcj02(lng, lat):
    """WGS84转GCJ02"""
    a = 6378245.0
    ee = 0.00669342162296594323
    
    if out_of_china(lng, lat):
        return lng, lat
    
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return mglng, mglat

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 * math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

# ==================== 障碍物持久化管理 ====================
def load_obstacles_from_file():
    """从JSON文件加载障碍物"""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return data.get('obstacles', [])
        except:
            return []
    return []

def save_obstacles_to_file(obstacles):
    """保存障碍物到JSON文件"""
    data = {
        'obstacles': obstacles,
        'count': len(obstacles),
        'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        'version': 'v12.2'
    }
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point_wgs84):
        self.history = []
        self.current_pos = start_point_wgs84.copy()
        self.target_pos = None
        self.simulating = False
        self.flight_altitude = 50
        
    def set_target(self, target_wgs84, altitude=50):
        self.target_pos = target_wgs84
        self.flight_altitude = altitude
        self.simulating = True
        
    def generate(self):
        if self.simulating and self.target_pos:
            dx = self.target_pos[0] - self.current_pos[0]
            dy = self.target_pos[1] - self.current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.00005:
                self.simulating = False
            else:
                step_x = dx / 50
                step_y = dy / 50
                self.current_pos[0] += step_x
                self.current_pos[1] += step_y
        
        if not self.simulating:
            altitude = random.randint(0, 10)
            speed = 0
        else:
            altitude = self.flight_altitude + random.randint(-5, 5)
            speed = round(random.uniform(8, 15), 1)
        
        heartbeat = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
            "speed": speed,
        }
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 50:
            self.history.pop()
        return heartbeat

# ==================== 创建OpenStreetMap地图 ====================
def create_planning_map(center_wgs84, points_wgs84, obstacles_wgs84, flight_history=None):
    """创建航线规划地图 - 使用OpenStreetMap"""
    m = folium.Map(
        location=[center_wgs84[1], center_wgs84[0]],
        zoom_start=16,
        tiles="OpenStreetMap"
    )
    
    # 添加障碍物多边形
    for obs in obstacles_wgs84:
        if obs.get('polygon'):
            coords = obs['polygon']
            folium.Polygon(
                locations=[[c[1], c[0]] for c in coords],
                color="red",
                weight=2,
                fill=True,
                fill_color="red",
                fill_opacity=0.4,
                popup=f"🚧 {obs.get('name', '障碍物')}"
            ).add_to(m)
    
    # 添加 A 点
    if points_wgs84.get('A'):
        folium.Marker(
            [points_wgs84['A'][1], points_wgs84['A'][0]],
            popup="🟢 起点 A",
            icon=folium.Icon(color="green", icon="play", prefix="fa")
        ).add_to(m)
    
    # 添加 B 点
    if points_wgs84.get('B'):
        folium.Marker(
            [points_wgs84['B'][1], points_wgs84['B'][0]],
            popup="🔴 终点 B",
            icon=folium.Icon(color="red", icon="stop", prefix="fa")
        ).add_to(m)
    
    # 添加航线
    if points_wgs84.get('A') and points_wgs84.get('B'):
        folium.PolyLine(
            [[points_wgs84['A'][1], points_wgs84['A'][0]], 
             [points_wgs84['B'][1], points_wgs84['B'][0]]],
            color="blue",
            weight=3,
            opacity=0.8,
            popup="规划航线"
        ).add_to(m)
    
    # 添加飞行轨迹
    if flight_history and len(flight_history) > 1:
        folium.PolyLine(
            [[p[1], p[0]] for p in flight_history],
            color="orange",
            weight=2,
            opacity=0.6,
            popup="历史轨迹"
        ).add_to(m)
    
    return m

# ==================== 主程序 ====================
def main():
    st.title("✈️ 无人机地面站系统 - 南京科技职业学院")
    st.markdown("---")
    
    # 侧边栏导航
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio(
        "选择功能模块",
        ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"]
    )
    
    # ==================== 初始化 Session State ====================
    # 点坐标存储 (WGS84格式，用于地图显示)
    if "points_wgs84" not in st.session_state:
        a_wgs84 = gcj02_to_wgs84(DEFAULT_A_GCJ[0], DEFAULT_A_GCJ[1])
        b_wgs84 = gcj02_to_wgs84(DEFAULT_B_GCJ[0], DEFAULT_B_GCJ[1])
        st.session_state.points_wgs84 = {
            'A': [a_wgs84[0], a_wgs84[1]],
            'B': [b_wgs84[0], b_wgs84[1]]
        }
    
    # 障碍物 (WGS84格式)
    if "obstacles_wgs84" not in st.session_state:
        st.session_state.obstacles_wgs84 = load_obstacles_from_file()
        if not st.session_state.obstacles_wgs84:
            # 默认障碍物
            st.session_state.obstacles_wgs84 = [
                {
                    "name": "教学楼",
                    "polygon": [
                        [118.6935, 32.1998],
                        [118.6948, 32.1998],
                        [118.6948, 32.2010],
                        [118.6935, 32.2010],
                        [118.6935, 32.1998]
                    ]
                },
                {
                    "name": "实验楼",
                    "polygon": [
                        [118.6965, 32.2012],
                        [118.6978, 32.2012],
                        [118.6978, 32.2022],
                        [118.6965, 32.2022],
                        [118.6965, 32.2012]
                    ]
                }
            ]
    
    # 心跳包模拟器
    if "heartbeat_sim" not in st.session_state:
        start_wgs84 = [st.session_state.points_wgs84['A'][0], st.session_state.points_wgs84['A'][1]]
        st.session_state.heartbeat_sim = HeartbeatSimulator(start_wgs84)
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "simulation_running" not in st.session_state:
        st.session_state.simulation_running = False
    if "flight_altitude" not in st.session_state:
        st.session_state.flight_altitude = 50
    if "flight_history" not in st.session_state:
        st.session_state.flight_history = []
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划")
        st.info("💡 操作说明：输入GCJ-02坐标(高德/百度)，系统自动转换到地图显示")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            st.caption("📐 输入坐标系：GCJ-02 (高德/百度)")
            
            st.markdown("---")
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=DEFAULT_A_GCJ[1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=DEFAULT_A_GCJ[0], format="%.6f", key="a_lng")
            
            if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
                # GCJ-02 转 WGS-84
                wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
                st.session_state.points_wgs84['A'] = [wgs_lng, wgs_lat]
                st.success(f"✅ A点已设置")
            
            st.markdown("---")
            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=DEFAULT_B_GCJ[1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=DEFAULT_B_GCJ[0], format="%.6f", key="b_lng")
            
            if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
                wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
                st.session_state.points_wgs84['B'] = [wgs_lng, wgs_lat]
                st.success(f"✅ B点已设置")
            
            st.markdown("---")
            st.markdown("#### ✈️ 飞行参数")
            flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5, min_value=10, max_value=200)
            st.session_state.flight_altitude = flight_alt
            
            st.markdown("---")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    target = st.session_state.points_wgs84['B']
                    st.session_state.heartbeat_sim.set_target([target[0], target[1]], st.session_state.flight_altitude)
                    st.session_state.simulation_running = True
                    st.success("🚁 飞行已开始！")
            
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.info("飞行已停止")
            
            st.markdown("---")
            st.markdown("### 📍 当前航点 (WGS84)")
            st.write(f"🟢 A点: ({st.session_state.points_wgs84['A'][0]:.6f}, {st.session_state.points_wgs84['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points_wgs84['B'][0]:.6f}, {st.session_state.points_wgs84['B'][1]:.6f})")
        
        with col2:
            st.subheader("🗺️ 规划地图 (OpenStreetMap)")
            
            # 获取飞行轨迹
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            
            # 地图中心
            center = st.session_state.points_wgs84['A'] if st.session_state.points_wgs84['A'] else [118.6965, 32.2015]
            
            planning_map = create_planning_map(
                center,
                st.session_state.points_wgs84,
                st.session_state.obstacles_wgs84,
                flight_trail
            )
            folium_static(planning_map, width=700, height=550)
            
            st.caption("📌 **图例**：🟢 A点 | 🔴 B点 | 🔴 红色区域=障碍物 | 🔵 蓝色线=规划航线 | 🟠 橙色线=历史轨迹")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        
        # 自动生成心跳包
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= 1 and st.session_state.simulation_running:
            new_hb = st.session_state.heartbeat_sim.generate()
            st.session_state.last_hb_time = current_time
            st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
            if len(st.session_state.flight_history) > 100:
                st.session_state.flight_history.pop(0)
            st.rerun()
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            # 指标卡片
            col1, col2, col3, col4, col5, col6 = st.columns(6)
            with col1:
                st.metric("⏰ 时间", latest['timestamp'])
            with col2:
                st.metric("📍 纬度", f"{latest['lat']:.6f}")
            with col3:
                st.metric("📍 经度", f"{latest['lng']:.6f}")
            with col4:
                st.metric("📊 高度", f"{latest['altitude']} m")
            with col5:
                st.metric("🔋 电压", f"{latest['voltage']} V")
            with col6:
                st.metric("🛰️ 卫星", latest['satellites'])
            
            # 飞行进度条
            if st.session_state.points_wgs84['A'] and st.session_state.points_wgs84['B']:
                a = st.session_state.points_wgs84['A']
                b = st.session_state.points_wgs84['B']
                current = [latest['lng'], latest['lat']]
                total_dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
                current_dist = math.sqrt((current[0]-a[0])**2 + (current[1]-a[1])**2)
                if total_dist > 0:
                    progress = min(1.0, current_dist / total_dist)
                    st.progress(progress, text=f"✈️ 飞行进度: {progress*100:.1f}%")
            
            # 实时位置地图
            st.subheader("📍 实时位置")
            monitor_map = folium.Map(
                location=[latest['lat'], latest['lng']],
                zoom_start=17,
                tiles="OpenStreetMap"
            )
            
            # 轨迹
            trail_points = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30]]
            if len(trail_points) > 1:
                folium.PolyLine(trail_points, color="orange", weight=3, opacity=0.7).add_to(monitor_map)
            
            # 当前位置
            folium.Marker(
                [latest['lat'], latest['lng']],
                popup=f"📍 当前位置\n高度: {latest['altitude']}m\n速度: {latest['speed']}m/s",
                icon=folium.Icon(color='red', icon='plane', prefix='fa')
            ).add_to(monitor_map)
            
            # A/B点
            if st.session_state.points_wgs84['A']:
                folium.Marker([st.session_state.points_wgs84['A'][1], st.session_state.points_wgs84['A'][0]], 
                             popup="起点 A", icon=folium.Icon(color='green')).add_to(monitor_map)
            if st.session_state.points_wgs84['B']:
                folium.Marker([st.session_state.points_wgs84['B'][1], st.session_state.points_wgs84['B'][0]], 
                             popup="终点 B", icon=folium.Icon(color='blue')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            # 数据图表
            st.subheader("📈 数据图表")
            chart_col1, chart_col2 = st.columns(2)
            with chart_col1:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    alt_df = pd.DataFrame({
                        "序号": range(len(st.session_state.heartbeat_sim.history[:20])),
                        "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(alt_df, x="序号", y="高度(m)")
                    st.caption("📊 高度变化趋势")
            with chart_col2:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    speed_df = pd.DataFrame({
                        "序号": range(len(st.session_state.heartbeat_sim.history[:20])),
                        "速度(m/s)": [h.get("speed", 0) for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(speed_df, x="序号", y="速度(m/s)")
                    st.caption("📊 速度变化趋势")
            
            # 历史记录表格
            st.subheader("📋 历史心跳记录")
            df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            st.dataframe(df, use_container_width=True)
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        # 控制按钮
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("🔄 立即刷新", use_container_width=True):
                new_hb = st.session_state.heartbeat_sim.generate()
                st.rerun()
        with col_btn2:
            if st.button("🗑️ 清空历史", use_container_width=True):
                st.session_state.heartbeat_sim.history = []
                st.rerun()
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理 - 多边形圈选")
        st.info("💡 障碍物配置会保存到本地文件，刷新页面不丢失")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("📝 障碍物列表")
            st.caption(f"文件状态: 共 {len(st.session_state.obstacles_wgs84)} 个障碍物")
            
            if st.session_state.obstacles_wgs84:
                for i, obs in enumerate(st.session_state.obstacles_wgs84):
                    col_name, col_btn = st.columns([3, 1])
                    with col_name:
                        st.write(f"🚧 {obs.get('name', f'障碍物{i+1}')}")
                    with col_btn:
                        if st.button("删除", key=f"del_{i}"):
                            st.session_state.obstacles_wgs84.pop(i)
                            save_obstacles_to_file(st.session_state.obstacles_wgs84)
                            st.rerun()
            else:
                st.write("暂无障癢物")
            
            st.markdown("---")
            st.subheader("💾 持久化操作")
            
            col_save, col_load = st.columns(2)
            with col_save:
                if st.button("💾 保存到文件", use_container_width=True):
                    save_obstacles_to_file(st.session_state.obstacles_wgs84)
                    st.success(f"已保存 {len(st.session_state.obstacles_wgs84)} 个障碍物")
            with col_load:
                if st.button("📂 从文件加载", use_container_width=True):
                    loaded = load_obstacles_from_file()
                    if loaded:
                        st.session_state.obstacles_wgs84 = loaded
                        st.success(f"已加载 {len(loaded)} 个障碍物")
                        st.rerun()
                    else:
                        st.warning("无配置文件或文件为空")
            
            col_clear, col_download = st.columns(2)
            with col_clear:
                if st.button("🗑️ 清除全部", use_container_width=True):
                    st.session_state.obstacles_wgs84 = []
                    save_obstacles_to_file([])
                    st.rerun()
            with col_download:
                # 下载配置文件
                config_data = {
                    'obstacles': st.session_state.obstacles_wgs84,
                    'count': len(st.session_state.obstacles_wgs84),
                    'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    'version': 'v12.2'
                }
                st.download_button(
                    label="📥 下载配置",
                    data=json.dumps(config_data, ensure_ascii=False, indent=2),
                    file_name=CONFIG_FILE,
                    mime="application/json",
                    use_container_width=True
                )
            
            st.markdown("---")
            st.subheader("➕ 添加障碍物")
            obs_name = st.text_input("障碍物名称", value=f"障碍物{len(st.session_state.obstacles_wgs84) + 1}")
            
            st.caption("多边形坐标格式: [[lng, lat], [lng, lat], ...]")
            default_coords = f"[[118.6950, 32.2015], [118.6960, 32.2015], [118.6960, 32.2025], [118.6950, 32.2025]]"
            coords_input = st.text_area("多边形坐标", value=default_coords, height=100)
            
            if st.button("➕ 添加障碍物", use_container_width=True):
                try:
                    coords = json.loads(coords_input)
                    new_obs = {
                        "name": obs_name,
                        "polygon": coords
                    }
                    st.session_state.obstacles_wgs84.append(new_obs)
                    save_obstacles_to_file(st.session_state.obstacles_wgs84)
                    st.success(f"已添加障碍物: {obs_name}")
                    st.rerun()
                except:
                    st.error("坐标格式错误，请使用正确的JSON格式")
        
        with col2:
            st.subheader("🗺️ 障碍物地图 (OpenStreetMap)")
            
            obs_map = folium.Map(
                location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]],
                zoom_start=16,
                tiles="OpenStreetMap"
            )
            
            for obs in st.session_state.obstacles_wgs84:
                if obs.get('polygon'):
                    coords = obs['polygon']
                    folium.Polygon(
                        locations=[[c[1], c[0]] for c in coords],
                        color="red",
                        weight=2,
                        fill=True,
                        fill_color="red",
                        fill_opacity=0.5,
                        popup=f"🚧 {obs.get('name', '障碍物')}"
                    ).add_to(obs_map)
            
            folium_static(obs_map, width=700, height=500)
            st.caption("🔴 红色多边形为已设置的障碍物区域")

if __name__ == "__main__":
    main()
