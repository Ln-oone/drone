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
import pandas as pd

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统 - 南京科技职业学院", layout="wide")

# ==================== 南京科技职业学院真实坐标 (GCJ-02) ====================
SCHOOL_CENTER_GCJ = [118.6965, 32.2015]

# A点 (南门附近) B点 (北门附近)
DEFAULT_A_GCJ = [118.6960, 32.2005]
DEFAULT_B_GCJ = [118.6968, 32.2030]

# ==================== 配置文件路径 ====================
CONFIG_FILE = "obstacle_config.json"

# ==================== 高德地图瓦片地址 ====================
GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

# ==================== 坐标系转换函数 ====================
def gcj02_to_wgs84(lng, lat):
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
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return data.get('obstacles', [])
        except:
            return []
    return []

def save_obstacles_to_file(obstacles):
    data = {
        'obstacles': obstacles,
        'count': len(obstacles),
        'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        'version': 'v12.2'
    }
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ==================== 安全获取多边形坐标 ====================
def get_polygon_coords(polygon_data):
    if not polygon_data:
        return []
    try:
        if isinstance(polygon_data, list):
            if len(polygon_data) >= 3:
                if isinstance(polygon_data[0], list) and isinstance(polygon_data[0][0], list):
                    return polygon_data[0]
                elif isinstance(polygon_data[0], list) and len(polygon_data[0]) == 2:
                    return polygon_data
        return []
    except:
        return []

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.target_pos = None
        self.simulating = False
        self.flight_altitude = 50
        self.speed = 50
        
    def set_target(self, target_gcj, altitude=50, speed=50):
        self.target_pos = target_gcj
        self.flight_altitude = altitude
        self.speed = speed
        self.simulating = True
        
    def generate(self):
        if self.simulating and self.target_pos:
            dx = self.target_pos[0] - self.current_pos[0]
            dy = self.target_pos[1] - self.current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.00001:
                self.simulating = False
            else:
                step_scale = self.speed / 20
                step_x = dx / max(20, 100 - step_scale)
                step_y = dy / max(20, 100 - step_scale)
                self.current_pos[0] += step_x
                self.current_pos[1] += step_y
        
        if not self.simulating:
            altitude = random.randint(0, 10)
            speed_display = 0
        else:
            altitude = self.flight_altitude + random.randint(-5, 5)
            speed_display = round(self.speed * 0.3, 1)
        
        heartbeat = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
            "speed": speed_display,
        }
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 50:
            self.history.pop()
        return heartbeat

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None):
    """创建航线规划地图"""
    m = folium.Map(
        location=[center_gcj[1], center_gcj[0]],
        zoom_start=16,
        tiles=GAODE_SATELLITE_URL,
        attr="高德卫星地图"
    )
    
    # 添加绘图控件
    draw = plugins.Draw(
        export=True,
        position='topleft',
        draw_options={
            'polygon': {'allowIntersection': False, 'showArea': True},
            'polyline': False,
            'rectangle': False,
            'circle': False,
            'marker': True,
            'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    # 添加点击获取坐标的插件（修复：使用正确的方法名）
    m.add_child(plugins.LatLngPopup())
    
    # 添加已保存的障碍物多边形
    for i, obs in enumerate(obstacles_gcj):
        coords = get_polygon_coords(obs.get('polygon', []))
        if coords and len(coords) >= 3:
            locations = [[c[1], c[0]] for c in coords]
            folium.Polygon(
                locations=locations,
                color="red",
                weight=2,
                fill=True,
                fill_color="red",
                fill_opacity=0.4,
                popup=f"🚧 {obs.get('name', f'障碍物{i+1}')}"
            ).add_to(m)
    
    # 添加 A 点
    if points_gcj.get('A'):
        folium.Marker(
            [points_gcj['A'][1], points_gcj['A'][0]],
            popup="🟢 起点 A",
            icon=folium.Icon(color="green", icon="play", prefix="fa")
        ).add_to(m)
    
    # 添加 B 点
    if points_gcj.get('B'):
        folium.Marker(
            [points_gcj['B'][1], points_gcj['B'][0]],
            popup="🔴 终点 B",
            icon=folium.Icon(color="red", icon="stop", prefix="fa")
        ).add_to(m)
    
    # 添加航线
    if points_gcj.get('A') and points_gcj.get('B'):
        folium.PolyLine(
            [[points_gcj['A'][1], points_gcj['A'][0]], 
             [points_gcj['B'][1], points_gcj['B'][0]]],
            color="blue",
            weight=3,
            opacity=0.8,
            popup="规划航线"
        ).add_to(m)
    
    # 添加飞行轨迹
    if flight_history and len(flight_history) > 1:
        trail_locations = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail_locations) > 1:
            folium.PolyLine(
                trail_locations,
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
    
    # 地图类型选择
    map_type = st.sidebar.radio(
        "🗺️ 地图类型",
        ["卫星影像", "矢量地图"],
        index=0
    )
    
    # 无人机速度设置（全局）
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider(
        "飞行速度系数", 
        min_value=10, 
        max_value=100, 
        value=50,
        step=5,
        help="数值越大，飞行越快（10=很慢，50=正常，100=极快）"
    )
    
    # ==================== 初始化 Session State ====================
    if "points_gcj" not in st.session_state:
        st.session_state.points_gcj = {
            'A': DEFAULT_A_GCJ.copy(),
            'B': DEFAULT_B_GCJ.copy()
        }
    
    if "obstacles_gcj" not in st.session_state:
        loaded = load_obstacles_from_file()
        if loaded:
            st.session_state.obstacles_gcj = loaded
        else:
            st.session_state.obstacles_gcj = []
    
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'].copy())
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "simulation_running" not in st.session_state:
        st.session_state.simulation_running = False
    if "flight_altitude" not in st.session_state:
        st.session_state.flight_altitude = 50
    if "flight_history" not in st.session_state:
        st.session_state.flight_history = []
    
    # 选择地图瓦片
    GAODE_URL = GAODE_SATELLITE_URL if map_type == "卫星影像" else GAODE_VECTOR_URL
    tile_attr = "高德卫星地图" if map_type == "卫星影像" else "高德矢量地图"
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划")
        st.info("💡 **两种方式设置A/B点**：1️⃣ 左侧输入经纬度  2️⃣ 右侧地图点击选点（点击地图任意位置获取坐标）")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            # 方式选择
            st.markdown("#### 📍 设置航点方式")
            point_input_method = st.radio(
                "选择方式",
                ["✏️ 手动输入经纬度", "🖱️ 地图点击选点"],
                horizontal=True,
                help="手动输入：在下方输入坐标；地图点击：在地图上点击位置后点击下方按钮"
            )
            
            st.markdown("---")
            
            if point_input_method == "✏️ 手动输入经纬度":
                st.markdown("#### 🟢 起点 A (南门区域)")
                a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
                a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
                
                if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
                    st.session_state.points_gcj['A'] = [a_lng, a_lat]
                    st.session_state.heartbeat_sim.current_pos = [a_lng, a_lat]
                    st.success(f"✅ A点已设置 ({a_lng:.6f}, {a_lat:.6f})")
                
                st.markdown("---")
                st.markdown("#### 🔴 终点 B (北门区域)")
                b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
                b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
                
                if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
                    st.session_state.points_gcj['B'] = [b_lng, b_lat]
                    st.success(f"✅ B点已设置 ({b_lng:.6f}, {b_lat:.6f})")
            
            else:  # 地图点击选点
                st.info("🖱️ **操作说明**：在右侧地图上点击任意位置，会弹出坐标弹窗，将坐标复制到下方输入框")
                
                st.markdown("#### 🟢 起点 A")
                a_lat_click = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat_click")
                a_lng_click = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng_click")
                
                if st.button("📌 将点击坐标设为A点", use_container_width=True, type="primary"):
                    st.session_state.points_gcj['A'] = [a_lng_click, a_lat_click]
                    st.session_state.heartbeat_sim.current_pos = [a_lng_click, a_lat_click]
                    st.success(f"✅ A点已设置")
                
                st.markdown("#### 🔴 终点 B")
                b_lat_click = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat_click")
                b_lng_click = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng_click")
                
                if st.button("📌 将点击坐标设为B点", use_container_width=True, type="primary"):
                    st.session_state.points_gcj['B'] = [b_lng_click, b_lat_click]
                    st.success(f"✅ B点已设置")
            
            st.markdown("---")
            st.markdown("#### ✈️ 飞行参数")
            flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5, min_value=10, max_value=200)
            st.session_state.flight_altitude = flight_alt
            
            # 显示当前速度系数
            st.metric("⚡ 当前速度系数", f"{drone_speed}%", help="速度系数越大，飞行越快")
            
            st.markdown("---")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    target = st.session_state.points_gcj['B']
                    st.session_state.heartbeat_sim.set_target(
                        [target[0], target[1]], 
                        st.session_state.flight_altitude,
                        drone_speed
                    )
                    st.session_state.simulation_running = True
                    st.success("🚁 飞行已开始！")
            
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.info("飞行已停止")
            
            st.markdown("---")
            st.markdown("### 📍 当前航点 (GCJ-02)")
            st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            
            # 计算距离
            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                a = st.session_state.points_gcj['A']
                b = st.session_state.points_gcj['B']
                dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 111000
                st.caption(f"📏 航线距离: 约 {dist:.0f} 米")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            st.caption("💡 **点击地图任意位置** → 弹出坐标弹窗 → 复制坐标到左侧输入框")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] if st.session_state.points_gcj['A'] else SCHOOL_CENTER_GCJ
            
            # 创建支持点击的地图
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail)
            
            # 显示地图
            folium_static(m, width=700, height=550)
            
            st.caption("📌 **图例**：🟢 A点 | 🔴 B点 | 🔴 红色区域=障碍物 | 🔵 蓝色线=规划航线 | 🟠 橙色线=历史轨迹")
            st.caption("✏️ **手绘障碍物**：点击左上角📐图标 → 选择多边形 → 在地图上绘制 → 自动保存")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        
        # 更新心跳包时使用当前速度
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
            
            # 速度显示
            col7, col8 = st.columns(2)
            with col7:
                st.metric("💨 当前速度", f"{latest.get('speed', 0)} m/s")
            with col8:
                st.metric("⚡ 速度系数", f"{drone_speed}%")
            
            # 飞行进度条
            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                a = st.session_state.points_gcj['A']
                b = st.session_state.points_gcj['B']
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
                tiles=GAODE_URL,
                attr=tile_attr
            )
            
            trail_points = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] if hb.get('lat') and hb.get('lng')]
            if len(trail_points) > 1:
                folium.PolyLine(trail_points, color="orange", weight=3, opacity=0.7).add_to(monitor_map)
            
            folium.Marker(
                [latest['lat'], latest['lng']],
                popup=f"📍 当前位置\n高度: {latest['altitude']}m\n速度: {latest['speed']}m/s",
                icon=folium.Icon(color='red', icon='plane', prefix='fa')
            ).add_to(monitor_map)
            
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                             popup="起点 A", icon=folium.Icon(color='green')).add_to(monitor_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
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
            
            st.subheader("📋 历史心跳记录")
            df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            st.dataframe(df, use_container_width=True)
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
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
        st.header("🚧 障碍物管理")
        st.info("💡 障碍物配置会保存到本地文件，刷新页面不丢失。也可以在「航线规划」页面手绘添加。")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("📝 障碍物列表")
            st.caption(f"共 {len(st.session_state.obstacles_gcj)} 个障碍物")
            
            if st.session_state.obstacles_gcj:
                for i, obs in enumerate(st.session_state.obstacles_gcj):
                    col_name, col_btn = st.columns([3, 1])
                    with col_name:
                        st.write(f"🚧 {obs.get('name', f'障碍物{i+1}')}")
                    with col_btn:
                        if st.button("删除", key=f"del_{i}"):
                            st.session_state.obstacles_gcj.pop(i)
                            save_obstacles_to_file(st.session_state.obstacles_gcj)
                            st.rerun()
            else:
                st.write("暂无障碍物")
            
            st.markdown("---")
            st.subheader("💾 持久化操作")
            
            col_save, col_load = st.columns(2)
            with col_save:
                if st.button("💾 保存到文件", use_container_width=True):
                    save_obstacles_to_file(st.session_state.obstacles_gcj)
                    st.success(f"已保存 {len(st.session_state.obstacles_gcj)} 个障碍物")
            with col_load:
                if st.button("📂 从文件加载", use_container_width=True):
                    loaded = load_obstacles_from_file()
                    if loaded:
                        st.session_state.obstacles_gcj = loaded
                        st.success(f"已加载 {len(loaded)} 个障碍物")
                        st.rerun()
                    else:
                        st.warning("无配置文件或文件为空")
            
            col_clear, col_download = st.columns(2)
            with col_clear:
                if st.button("🗑️ 清除全部", use_container_width=True):
                    st.session_state.obstacles_gcj = []
                    save_obstacles_to_file([])
                    st.rerun()
            with col_download:
                config_data = {
                    'obstacles': st.session_state.obstacles_gcj,
                    'count': len(st.session_state.obstacles_gcj),
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
        
        with col2:
            st.subheader("🗺️ 障碍物地图")
            
            obs_map = folium.Map(
                location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]],
                zoom_start=16,
                tiles=GAODE_URL,
                attr=tile_attr
            )
            
            for i, obs in enumerate(st.session_state.obstacles_gcj):
                coords = get_polygon_coords(obs.get('polygon', []))
                if coords and len(coords) >= 3:
                    locations = [[c[1], c[0]] for c in coords]
                    folium.Polygon(
                        locations=locations,
                        color="red",
                        weight=2,
                        fill=True,
                        fill_color="red",
                        fill_opacity=0.5,
                        popup=f"🚧 {obs.get('name', f'障碍物{i+1}')}"
                    ).add_to(obs_map)
            
            folium_static(obs_map, width=700, height=500)
            st.caption("🔴 红色多边形为已设置的障碍物区域")

if __name__ == "__main__":
    main()
