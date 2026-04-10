import streamlit as st
import folium
from streamlit_folium import folium_static
import random
import time
import math
from datetime import datetime

# ==================== 坐标系转换函数 ====================
# WGS-84 与 GCJ-02 互转（高德/百度使用GCJ-02，OpenStreetMap使用WGS-84）

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

def gcj02_to_wgs84(lng, lat):
    """GCJ02转WGS84"""
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

# ==================== 心跳包模拟 ====================
class HeartbeatSimulator:
    def __init__(self):
        self.history = []  # 存储(时间, 心跳数据)
        self.running = True

    def generate_heartbeat(self):
        """生成一条模拟心跳数据"""
        return {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lat": 32.23775 + random.uniform(-0.003, 0.003),
            "lng": 118.7490 + random.uniform(-0.003, 0.003),
            "altitude": random.randint(40, 60),
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
        }

    def update(self):
        if self.running:
            hb = self.generate_heartbeat()
            self.history.insert(0, hb)
            if len(self.history) > 20:
                self.history.pop()
            return hb
        return None

# 初始化心跳模拟器（放在session_state中）
if "heartbeat" not in st.session_state:
    st.session_state.heartbeat = HeartbeatSimulator()
if "last_heartbeat_time" not in st.session_state:
    st.session_state.last_heartbeat_time = time.time()

# ==================== 页面导航 ====================
st.set_page_config(page_title="无人机地面站", layout="wide")
st.sidebar.title("导航")
page = st.sidebar.radio("选择页面", ["航线规划", "飞行监控"])

# 公共状态：A/B点坐标（存储为WGS84，地图显示用）
if "point_a_wgs84" not in st.session_state:
    st.session_state.point_a_wgs84 = None  # [lng, lat]
if "point_b_wgs84" not in st.session_state:
    st.session_state.point_b_wgs84 = None

# 障碍物（校园内，WGS84坐标）
obstacles_wgs84 = [
    [118.7485, 32.2345],
    [118.7495, 32.2360],
    [118.7480, 32.2385],
    [118.7498, 32.2405],
    [118.7482, 32.2420],
]

# 默认地图中心（校园中心，WGS84）
CENTER_WGS84 = [118.7490, 32.23775]  # [lng, lat]

# ==================== 航线规划页面 ====================
if page == "航线规划":
    st.title("🗺️ 航线规划")

    # 坐标系选择与输入区域
    col1, col2 = st.columns([1, 2])
    with col1:
        st.subheader("控制面板")
        coord_sys = st.radio("输入坐标系", ["WGS-84", "GCJ-02 (高德/百度)"], index=1)
        st.markdown("---")
        st.markdown("**起点 A**")
        a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
        if st.button("设置 A 点"):
            # 根据选择的坐标系转换到WGS84存储
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
            else:
                wgs_lng, wgs_lat = a_lng, a_lat
            st.session_state.point_a_wgs84 = [wgs_lng, wgs_lat]
            st.success(f"A点已设置（WGS84: {wgs_lng:.6f}, {wgs_lat:.6f})")

        st.markdown("**终点 B**")
        b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
        if st.button("设置 B 点"):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
            else:
                wgs_lng, wgs_lat = b_lng, b_lat
            st.session_state.point_b_wgs84 = [wgs_lng, wgs_lat]
            st.success(f"B点已设置（WGS84: {wgs_lng:.6f}, {wgs_lat:.6f})")

        st.markdown("---")
        flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5)
        st.markdown("---")
        if st.button("清除 A/B 点"):
            st.session_state.point_a_wgs84 = None
            st.session_state.point_b_wgs84 = None
            st.info("已清除所有航点")

    with col2:
        st.subheader("地图 (OpenStreetMap)")
        # 创建地图（使用WGS84坐标系，OpenStreetMap默认WGS84）
        m = folium.Map(location=[CENTER_WGS84[1], CENTER_WGS84[0]], zoom_start=16, tiles="OpenStreetMap")

        # 添加障碍物（红色方块）
        for obs in obstacles_wgs84:
            folium.CircleMarker(
                location=[obs[1], obs[0]],
                radius=8,
                color="red",
                fill=True,
                fill_color="red",
                fill_opacity=0.8,
                popup="障碍物",
            ).add_to(m)
            folium.Rectangle(
                bounds=[[obs[1]-0.0003, obs[0]-0.0003], [obs[1]+0.0003, obs[0]+0.0003]],
                color="red",
                fill=True,
                fill_opacity=0.3,
            ).add_to(m)

        # 添加 A 点
        if st.session_state.point_a_wgs84:
            lng_a, lat_a = st.session_state.point_a_wgs84
            folium.Marker(
                location=[lat_a, lng_a],
                popup="起点 A",
                icon=folium.Icon(color="green", icon="play", prefix="fa"),
            ).add_to(m)
            folium.CircleMarker([lat_a, lng_a], radius=6, color="green", fill=True, popup="A").add_to(m)

        # 添加 B 点
        if st.session_state.point_b_wgs84:
            lng_b, lat_b = st.session_state.point_b_wgs84
            folium.Marker(
                location=[lat_b, lng_b],
                popup="终点 B",
                icon=folium.Icon(color="red", icon="stop", prefix="fa"),
            ).add_to(m)
            folium.CircleMarker([lat_b, lng_b], radius=6, color="red", fill=True, popup="B").add_to(m)

        # 连线
        if st.session_state.point_a_wgs84 and st.session_state.point_b_wgs84:
            points = [[st.session_state.point_a_wgs84[1], st.session_state.point_a_wgs84[0]],
                      [st.session_state.point_b_wgs84[1], st.session_state.point_b_wgs84[0]]]
            folium.PolyLine(points, color="blue", weight=3, opacity=0.7, popup="规划航线").add_to(m)

        # 显示地图
        folium_static(m, width=800, height=600)

        # 提示：3D地图说明（OpenStreetMap 2D，但可通过插件实现倾斜；这里仅说明）
        st.info("💡 提示：当前使用 OpenStreetMap 2D 地图。如需3D效果，可替换为 Mapbox GL 或 Cesium（需 token）。")

# ==================== 飞行监控页面 ====================
elif page == "飞行监控":
    st.title("📡 飞行监控 - 心跳包接收")

    # 自动刷新心跳（每2秒）
    current_time = time.time()
    if current_time - st.session_state.last_heartbeat_time >= 2:
        st.session_state.heartbeat.update()
        st.session_state.last_heartbeat_time = current_time
        st.rerun()  # 刷新页面以显示最新心跳

    # 显示最新心跳
    if st.session_state.heartbeat.history:
        latest = st.session_state.heartbeat.history[0]
        col1, col2, col3, col4, col5 = st.columns(5)
        col1.metric("时间", latest["timestamp"])
        col2.metric("纬度", f"{latest['lat']:.6f}")
        col3.metric("经度", f"{latest['lng']:.6f}")
        col4.metric("高度 (m)", latest["altitude"])
        col5.metric("电压 (V)", latest["voltage"])

        st.subheader("历史心跳记录")
        # 展示最近10条
        import pandas as pd
        df = pd.DataFrame(st.session_state.heartbeat.history[:10])
        st.dataframe(df, use_container_width=True)
    else:
        st.info("等待心跳数据...")

    # 手动刷新按钮
    if st.button("立即刷新心跳"):
        st.session_state.heartbeat.update()
        st.rerun()

    st.markdown("---")
    st.caption("心跳包模拟数据，每2秒自动更新。实际使用时可替换为真实串口/网络接收。")
