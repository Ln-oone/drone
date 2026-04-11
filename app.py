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
SCHOOL_CENTER = [118.7490, 32.23775]  # [经度, 纬度] 默认中心点

# ==================== 坐标系转换函数 ====================
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

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    a = 6378245.0
    ee = 0.00669342162296594323
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
OBSTACLE_FILE = "obstacle_config.json"

def load_obstacles_from_file():
    """从JSON文件加载障碍物"""
    if os.path.exists(OBSTACLE_FILE):
        try:
            with open(OBSTACLE_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return data.get("obstacles", [])
        except:
            return []
    return []

def save_obstacles_to_file(obstacles):
    """保存障碍物到JSON文件"""
    data = {
        "version": "v12.2",
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "obstacles": obstacles
    }
    with open(OBSTACLE_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self):
        self.history = []
        self.current_pos = [SCHOOL_CENTER[1], SCHOOL_CENTER[0]]  # [lat, lng]
        
    def generate(self):
        # 模拟随机游走
        self.current_pos[0] += random.uniform(-0.0005, 0.0005)
        self.current_pos[1] += random.uniform(-0.0005, 0.0005)
        
        heartbeat = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lat": self.current_pos[0],
            "lng": self.current_pos[1],
            "altitude": random.randint(40, 60),
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
            "speed": round(random.uniform(0, 15), 1),
        }
        self.history.insert(0, heartbeat)
        if len(self.history) > 50:
            self.history.pop()
        return heartbeat

# ==================== 创建地图 ====================
def create_map(center_lat, center_lng, point_a=None, point_b=None, obstacles=None, flight_path=None):
    """创建OpenStreetMap地图"""
    m = folium.Map(
        location=[center_lat, center_lng], 
        zoom_start=16, 
        tiles="OpenStreetMap"
    )
    
    # 添加障碍物多边形
    if obstacles:
        for obs in obstacles:
            if obs.get("type") == "polygon" and obs.get("coordinates"):
                coords = obs["coordinates"]
                # 转换坐标格式为 [[lat, lng], ...]
                locations = [[c[1], c[0]] for c in coords]
                folium.Polygon(
                    locations=locations,
                    color="red",
                    weight=2,
                    fill=True,
                    fill_color="red",
                    fill_opacity=0.4,
                    popup=obs.get("name", "障碍物")
                ).add_to(m)
    
    # 添加 A 点
    if point_a:
        folium.Marker(
            point_a, 
            popup="🟢 起点 A", 
            icon=folium.Icon(color="green", icon="play", prefix="fa")
        ).add_to(m)
    
    # 添加 B 点
    if point_b:
        folium.Marker(
            point_b, 
            popup="🔴 终点 B", 
            icon=folium.Icon(color="red", icon="stop", prefix="fa")
        ).add_to(m)
    
    # 添加连线
    if point_a and point_b:
        folium.PolyLine(
            locations=[point_a, point_b],
            color="blue",
            weight=3,
            opacity=0.7,
            popup="规划航线"
        ).add_to(m)
    
    return m

# ==================== 主程序 ====================
def main():
    st.title("✈️ 无人机地面站系统")
    st.markdown("---")
    
    # 侧边栏导航
    st.sidebar.title("导航菜单")
    page = st.sidebar.radio(
        "选择功能模块",
        ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"]
    )
    
    # 初始化 Session State
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator()
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "point_a" not in st.session_state:
        st.session_state.point_a = None
    if "point_b" not in st.session_state:
        st.session_state.point_b = None
    if "flight_altitude" not in st.session_state:
        st.session_state.flight_altitude = 50
    
    # 加载障碍物
    if "obstacles" not in st.session_state:
        st.session_state.obstacles = load_obstacles_from_file()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划")
        st.info("💡 操作说明：设置起点(A)和终点(B)，系统将自动规划航线并避开红色障碍物区域")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            # 坐标系选择
            coord_sys = st.radio(
                "📐 输入坐标系",
                ["WGS-84", "GCJ-02 (高德/百度)"],
                index=1
            )
            
            st.markdown("---")
            
            # 起点 A
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
            
            if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
                if coord_sys == "GCJ-02 (高德/百度)":
                    wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
                else:
                    wgs_lng, wgs_lat = a_lng, a_lat
                st.session_state.point_a = [wgs_lat, wgs_lng]
                st.success(f"✅ A点已设置")
            
            st.markdown("---")
            
            # 终点 B
            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
            
            if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
                if coord_sys == "GCJ-02 (高德/百度)":
                    wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
                else:
                    wgs_lng, wgs_lat = b_lng, b_lat
                st.session_state.point_b = [wgs_lat, wgs_lng]
                st.success(f"✅ B点已设置")
            
            st.markdown("---")
            
            # 飞行参数
            st.markdown("#### ✈️ 飞行参数")
            st.session_state.flight_altitude = st.number_input(
                "设定飞行高度 (m)", 
                value=st.session_state.flight_altitude, 
                step=5, 
                min_value=10, 
                max_value=200
            )
            
            st.markdown("---")
            
            # 显示当前坐标
            st.markdown("### 📍 当前航点")
            if st.session_state.point_a:
                st.write(f"🟢 A点: ({st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f})")
            else:
                st.warning("A点未设置")
            if st.session_state.point_b:
                st.write(f"🔴 B点: ({st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f})")
            else:
                st.warning("B点未设置")
        
        with col2:
            st.subheader("🗺️ 规划地图 (OpenStreetMap)")
            
            # 确定地图中心
            if st.session_state.point_a:
                center = st.session_state.point_a
            else:
                center = [SCHOOL_CENTER[1], SCHOOL_CENTER[0]]
            
            planning_map = create_map(
                center[0], center[1],
                st.session_state.point_a,
                st.session_state.point_b,
                st.session_state.obstacles
            )
            folium_static(planning_map, width=700, height=500)
            
            st.caption("📌 **图例**：🟢 A点 | 🔴 B点 | 🔴 红色区域=障碍物 | 🔵 蓝色线=规划航线")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 心跳包接收")
        
        # 自动生成心跳包
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= 2:
            st.session_state.heartbeat_sim.generate()
            st.session_state.last_hb_time = current_time
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
            
            # 实时位置地图
            st.subheader("📍 实时位置")
            monitor_map = folium.Map(
                location=[latest['lat'], latest['lng']],
                zoom_start=17,
                tiles="OpenStreetMap"
            )
            
            # 绘制轨迹
            trail_points = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:20]]
            if len(trail_points) > 1:
                folium.PolyLine(trail_points, color="orange", weight=3, opacity=0.7).add_to(monitor_map)
            
            folium.Marker(
                [latest['lat'], latest['lng']],
                popup=f"当前位置\n高度: {latest['altitude']}m",
                icon=folium.Icon(color='red', icon='plane', prefix='fa')
            ).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            # 历史数据表格
            st.subheader("📋 历史心跳记录")
            df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            st.dataframe(df, use_container_width=True)
        else:
            st.info("⏳ 等待心跳数据...")
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理 - 多边形圈选")
        st.info("💡 使用多边形工具在地图上圈选障碍物，支持保存和加载")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("📝 障碍物配置")
            st.caption(f"配置文件: {os.path.abspath(OBSTACLE_FILE)} | 版本: v12.2 障碍物持久化版")
            
            # 显示当前障碍物列表
            if st.session_state.obstacles:
                for i, obs in enumerate(st.session_state.obstacles):
                    col_name, col_btn = st.columns([3, 1])
                    with col_name:
                        st.write(f"• {obs.get('name', f'障碍物{i+1}')}")
                    with col_btn:
                        if st.button("删除", key=f"del_{i}"):
                            st.session_state.obstacles.pop(i)
                            st.rerun()
            else:
                st.write("暂无障碍物")
            
            st.markdown("---")
            st.subheader("🔧 操作按钮")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("💾 保存到文件", use_container_width=True):
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.success(f"已保存 {len(st.session_state.obstacles)} 个障碍物")
            
            with col_btn2:
                if st.button("📂 从文件加载", use_container_width=True):
                    st.session_state.obstacles = load_obstacles_from_file()
                    st.success(f"已加载 {len(st.session_state.obstacles)} 个障碍物")
                    st.rerun()
            
            if st.button("🗑️ 清除全部", use_container_width=True):
                st.session_state.obstacles = []
                st.rerun()
            
            st.markdown("---")
            
            # 添加新障碍物
            st.subheader("➕ 添加障碍物")
            obs_name = st.text_input("障碍物名称", value=f"障碍物{len(st.session_state.obstacles)+1}")
            
            st.markdown("**多边形坐标** (格式: 经度,纬度 每行一个点)")
            coord_text = st.text_area(
                "坐标点",
                value="118.7480,32.2340\n118.7495,32.2340\n118.7495,32.2355\n118.7480,32.2355",
                height=120
            )
            
            if st.button("➕ 添加多边形", use_container_width=True):
                try:
                    coords = []
                    for line in coord_text.strip().split('\n'):
                        if ',' in line:
                            lng, lat = line.split(',')
                            coords.append([float(lng.strip()), float(lat.strip())])
                    if len(coords) >= 3:
                        new_obs = {
                            "type": "polygon",
                            "name": obs_name,
                            "coordinates": coords
                        }
                        st.session_state.obstacles.append(new_obs)
                        st.success(f"已添加障碍物: {obs_name}")
                        st.rerun()
                    else:
                        st.error("至少需要3个坐标点")
                except Exception as e:
                    st.error(f"坐标格式错误: {e}")
            
            # 下载配置
            st.markdown("---")
            st.subheader("📥 下载配置文件")
            
            config_data = {
                "version": "v12.2",
                "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "obstacles": st.session_state.obstacles
            }
            config_json = json.dumps(config_data, ensure_ascii=False, indent=2)
            st.download_button(
                label="📥 下载 obstacle_config.json",
                data=config_json,
                file_name="obstacle_config.json",
                mime="application/json",
                use_container_width=True
            )
            
            # 文件状态
            st.markdown("---")
            st.subheader("📊 文件状态")
            st.write(f"共 {len(st.session_state.obstacles)} 个障碍物")
            if os.path.exists(OBSTACLE_FILE):
                mtime = os.path.getmtime(OBSTACLE_FILE)
                st.write(f"保存时间: {datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')}")
            st.write("版本: v12.2 障碍物持久化版")
        
        with col2:
            st.subheader("🗺️ 障碍物地图 (OpenStreetMap)")
            
            obs_map = create_map(
                SCHOOL_CENTER[1], SCHOOL_CENTER[0],
                obstacles=st.session_state.obstacles
            )
            folium_static(obs_map, width=700, height=500)
            st.caption("🔴 红色区域为已设置的障碍物 | 使用左侧面板添加/管理")

if __name__ == "__main__":
    main()
