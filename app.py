import streamlit as st
import folium
from streamlit_folium import folium_static
import random
import time
import math
from datetime import datetime
import pandas as pd

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站", layout="wide")

# ==================== 坐标系转换函数 ====================
def gcj02_to_wgs84(lng, lat):
    """GCJ-02 转 WGS-84"""
    a = 6378245.0
    ee = 0.00669342162296594323
    
    def out_of_china(lng, lat):
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)
    
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
    """WGS-84 转 GCJ-02"""
    a = 6378245.0
    ee = 0.00669342162296594323
    
    def out_of_china(lng, lat):
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)
    
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

# ==================== 初始化 Session State ====================
if "point_a" not in st.session_state:
    st.session_state.point_a = None  # [lng, lat] WGS84
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "last_heartbeat_time" not in st.session_state:
    st.session_state.last_heartbeat_time = time.time()

# ==================== 障碍物（WGS84坐标）====================
obstacles = [
    {"lng": 118.7485, "lat": 32.2345, "name": "障碍物1"},
    {"lng": 118.7495, "lat": 32.2360, "name": "障碍物2"},
    {"lng": 118.7480, "lat": 32.2385, "name": "障碍物3"},
    {"lng": 118.7498, "lat": 32.2405, "name": "障碍物4"},
    {"lng": 118.7482, "lat": 32.2420, "name": "障碍物5"},
]

# 地图中心点（校园中心）
CENTER = [32.23775, 118.7490]  # [lat, lng]

# ==================== 侧边栏导航 ====================
st.sidebar.title("导航")
page = st.sidebar.radio("选择页面", ["航线规划", "飞行监控"])

# ==================== 航线规划页面 ====================
if page == "航线规划":
    st.title("🗺️ 航线规划")
    
    # 左侧控制面板
    with st.sidebar:
        st.markdown("---")
        st.subheader("坐标系设置")
        coord_type = st.radio("输入坐标系", ["WGS-84", "GCJ-02 (高德/百度)"], index=1)
        
        st.markdown("---")
        st.subheader("起点 A")
        a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
        
        if st.button("设置 A 点", use_container_width=True):
            if coord_type == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
            else:
                wgs_lng, wgs_lat = a_lng, a_lat
            st.session_state.point_a = [wgs_lng, wgs_lat]
            st.success(f"✅ A点已设置")
        
        st.markdown("---")
        st.subheader("终点 B")
        b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
        
        if st.button("设置 B 点", use_container_width=True):
            if coord_type == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
            else:
                wgs_lng, wgs_lat = b_lng, b_lat
            st.session_state.point_b = [wgs_lng, wgs_lat]
            st.success(f"✅ B点已设置")
        
        st.markdown("---")
        flight_height = st.number_input("飞行高度 (米)", value=50, step=10)
        
        if st.button("清除所有航点", use_container_width=True):
            st.session_state.point_a = None
            st.session_state.point_b = None
            st.info("已清除所有航点")
        
        st.markdown("---")
        st.write("**当前航点状态**")
        st.write(f"A点: {st.session_state.point_a if st.session_state.point_a else '未设置'}")
        st.write(f"B点: {st.session_state.point_b if st.session_state.point_b else '未设置'}")
    
    # 右侧地图显示
    # 使用高德地图瓦片（国内可访问）
    tiles = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
    attr = "高德地图"
    
    m = folium.Map(location=CENTER, zoom_start=16, tiles=tiles, attr=attr)
    
    # 添加障碍物
    for obs in obstacles:
        # 红色圆点
        folium.CircleMarker(
            location=[obs["lat"], obs["lng"]],
            radius=10,
            color="red",
            fill=True,
            fill_color="red",
            fill_opacity=0.7,
            popup=obs["name"]
        ).add_to(m)
        # 红色边框方块
        folium.Rectangle(
            bounds=[[obs["lat"]-0.0003, obs["lng"]-0.0003], [obs["lat"]+0.0003, obs["lng"]+0.0003]],
            color="red",
            weight=2,
            fill=True,
            fill_opacity=0.3
        ).add_to(m)
    
    # 添加 A 点
    if st.session_state.point_a:
        lng_a, lat_a = st.session_state.point_a
        folium.Marker(
            location=[lat_a, lng_a],
            popup="起点 A",
            icon=folium.Icon(color="green", icon="play", prefix="fa")
        ).add_to(m)
        folium.CircleMarker(
            [lat_a, lng_a],
            radius=8,
            color="green",
            fill=True,
            fill_opacity=0.8,
            popup="A"
        ).add_to(m)
    
    # 添加 B 点
    if st.session_state.point_b:
        lng_b, lat_b = st.session_state.point_b
        folium.Marker(
            location=[lat_b, lng_b],
            popup="终点 B",
            icon=folium.Icon(color="red", icon="stop", prefix="fa")
        ).add_to(m)
        folium.CircleMarker(
            [lat_b, lng_b],
            radius=8,
            color="orange",
            fill=True,
            fill_opacity=0.8,
            popup="B"
        ).add_to(m)
    
    # 连接 A 和 B
    if st.session_state.point_a and st.session_state.point_b:
        points = [
            [st.session_state.point_a[1], st.session_state.point_a[0]],
            [st.session_state.point_b[1], st.session_state.point_b[0]]
        ]
        folium.PolyLine(
            points,
            color="blue",
            weight=3,
            opacity=0.8,
            popup="规划航线"
        ).add_to(m)
        
        # 显示距离信息
        from math import radians, sin, cos, sqrt, asin
        def calc_distance(lng1, lat1, lng2, lat2):
            R = 6371
            dlat = radians(lat2 - lat1)
            dlng = radians(lng2 - lng1)
            a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlng/2)**2
            return R * 2 * asin(sqrt(a))
        
        dist = calc_distance(
            st.session_state.point_a[0], st.session_state.point_a[1],
            st.session_state.point_b[0], st.session_state.point_b[1]
        )
        st.info(f"📏 航线距离: {dist:.2f} km | 飞行高度: {flight_height} m")
    
    # 显示地图
    folium_static(m, width=900, height=650)

# ==================== 飞行监控页面 ====================
elif page == "飞行监控":
    st.title("📡 飞行监控")
    
    # 生成心跳数据（每2秒）
    def generate_heartbeat():
        return {
            "时间": datetime.now().strftime("%H:%M:%S"),
            "纬度": 32.23775 + random.uniform(-0.005, 0.005),
            "经度": 118.7490 + random.uniform(-0.005, 0.005),
            "高度(m)": random.randint(40, 70),
            "电压(V)": round(random.uniform(11.2, 12.6), 1),
            "卫星数": random.randint(8, 14),
            "速度(km/h)": random.randint(0, 30)
        }
    
    # 自动更新心跳
    current_time = time.time()
    if current_time - st.session_state.last_heartbeat_time >= 2:
        new_heartbeat = generate_heartbeat()
        st.session_state.heartbeat_history.insert(0, new_heartbeat)
        if len(st.session_state.heartbeat_history) > 20:
            st.session_state.heartbeat_history.pop()
        st.session_state.last_heartbeat_time = current_time
    
    # 显示最新心跳
    if st.session_state.heartbeat_history:
        latest = st.session_state.heartbeat_history[0]
        
        # 指标卡片
        col1, col2, col3, col4, col5, col6 = st.columns(6)
        col1.metric("⏰ 时间", latest["时间"])
        col2.metric("📍 纬度", f"{latest['纬度']:.6f}")
        col3.metric("📍 经度", f"{latest['经度']:.6f}")
        col4.metric("📊 高度", f"{latest['高度(m)']} m")
        col5.metric("🔋 电压", f"{latest['电压(V)']} V")
        col6.metric("🛰️ 卫星", latest["卫星数"])
        
        st.markdown("---")
        
        # 飞行轨迹地图
        st.subheader("飞行轨迹")
        tiles = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
        m = folium.Map(location=[latest["纬度"], latest["经度"]], zoom_start=16, tiles=tiles, attr="高德地图")
        
        # 添加轨迹点
        for i, hb in enumerate(st.session_state.heartbeat_history[:10]):
            color = "red" if i == 0 else "blue"
            folium.CircleMarker(
                [hb["纬度"], hb["经度"]],
                radius=6 if i == 0 else 4,
                color=color,
                fill=True,
                popup=f"时间: {hb['时间']}<br>高度: {hb['高度(m)']}m"
            ).add_to(m)
        
        folium_static(m, width=900, height=450)
        
        st.markdown("---")
        
        # 历史数据表格
        st.subheader("历史心跳记录")
        df = pd.DataFrame(st.session_state.heartbeat_history[:10])
        st.dataframe(df, use_container_width=True)
        
        # 手动刷新按钮
        if st.button("🔄 立即刷新", use_container_width=True):
            new_hb = generate_heartbeat()
            st.session_state.heartbeat_history.insert(0, new_hb)
            st.rerun()
    else:
        st.info("⏳ 等待心跳数据...")
        time.sleep(0.1)
        st.rerun()
    
    st.markdown("---")
    st.caption("💡 心跳包每2秒自动更新（模拟数据）")
