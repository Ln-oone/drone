import streamlit as st
import pandas as pd
import pydeck as pdk
import random
import time
import math
from datetime import datetime

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

# ==================== 心跳包模拟 ====================
def generate_heartbeat():
    return {
        "timestamp": datetime.now().strftime("%H:%M:%S.%f")[:-3],
        "lat": 32.23775 + random.uniform(-0.003, 0.003),
        "lng": 118.7490 + random.uniform(-0.003, 0.003),
        "altitude": random.randint(45, 55),
        "voltage": round(random.uniform(11.8, 12.5), 2),
        "satellites": random.randint(10, 15),
        "speed": round(random.uniform(0, 15), 1),
        "heading": random.randint(0, 360)
    }

# ==================== 初始化 Session State ====================
if "point_a" not in st.session_state:
    st.session_state.point_a = None  # [lng, lat] WGS84
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "last_heartbeat_time" not in st.session_state:
    st.session_state.last_heartbeat_time = time.time()

# 障碍物（位于 A 点 32.2322 和 B 点 32.2433 之间）
obstacles = [
    {"name": "图书馆", "lng": 118.7485, "lat": 32.2345, "height": 25},
    {"name": "教学楼", "lng": 118.7495, "lat": 32.2360, "height": 30},
    {"name": "实验楼", "lng": 118.7480, "lat": 32.2385, "height": 28},
    {"name": "学生食堂", "lng": 118.7498, "lat": 32.2405, "height": 20},
    {"name": "体育馆", "lng": 118.7482, "lat": 32.2420, "height": 22},
]

# 地图中心点
CENTER_LNG, CENTER_LAT = 118.7490, 32.23775

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统", layout="wide")
st.sidebar.title("🎛️ 导航菜单")
page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控"])

# 侧边栏状态显示
st.sidebar.markdown("---")
st.sidebar.info(
    f"**系统状态**\n\n"
    f"- A点: {'✅ 已设' if st.session_state.point_a else '❌ 未设'}\n"
    f"- B点: {'✅ 已设' if st.session_state.point_b else '❌ 未设'}\n"
    f"- 障碍物: {len(obstacles)}个\n"
    f"- 心跳包: {len(st.session_state.heartbeat_history)}条"
)

# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.title("🗺️ 航线规划 - 3D地图 (PyDeck)")

    col1, col2 = st.columns([1, 1.5])

    with col1:
        st.markdown("### 🎮 控制面板")

        coord_sys = st.radio(
            "📐 输入坐标系",
            ["WGS-84", "GCJ-02 (高德/百度)"],
            index=1,
            help="选择你输入的坐标所使用的坐标系"
        )

        st.markdown("---")
        st.markdown("#### 🟢 起点 A")
        a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")

        if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
            else:
                wgs_lng, wgs_lat = a_lng, a_lat
            st.session_state.point_a = [wgs_lng, wgs_lat]
            st.success(f"✅ A点已设置")

        st.markdown("---")
        st.markdown("#### 🔴 终点 B")
        b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")

        if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
            else:
                wgs_lng, wgs_lat = b_lng, b_lat
            st.session_state.point_b = [wgs_lng, wgs_lat]
            st.success(f"✅ B点已设置")

        st.markdown("---")
        st.markdown("#### ✈️ 飞行参数")
        flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5, min_value=20, max_value=200)

        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("🗑️ 清除所有", use_container_width=True):
                st.session_state.point_a = None
                st.session_state.point_b = None
                st.info("已清除所有航点")
        with col_btn2:
            if st.button("🔄 重置视图", use_container_width=True):
                st.rerun()

        # 显示当前坐标
        st.markdown("---")
        st.markdown("### 📍 当前航点")
        if st.session_state.point_a:
            st.success(f"A点: ({st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f})")
        else:
            st.warning("A点未设置")
        if st.session_state.point_b:
            st.success(f"B点: ({st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f})")
        else:
            st.warning("B点未设置")

    with col2:
        st.markdown("### 🗺️ 3D 地图视图")
        st.caption("💡 **操作提示**：鼠标拖拽旋转视角 | 右键拖拽平移 | 滚动缩放 | 支持3D地形")

        # 准备图层数据
        layers = []

        # 1. 障碍物图层（3D柱状）
        if obstacles:
            obstacle_df = pd.DataFrame(obstacles)
            obstacle_layer = pdk.Layer(
                "ColumnLayer",
                data=obstacle_df,
                get_position=["lng", "lat"],
                get_elevation="height",
                elevation_scale=1,
                radius=20,
                get_fill_color=[255, 0, 0, 180],
                pickable=True,
                auto_highlight=True,
                extruded=True,
            )
            layers.append(obstacle_layer)

        # 2. A 点标记
        if st.session_state.point_a:
            a_df = pd.DataFrame([{
                "lng": st.session_state.point_a[0],
                "lat": st.session_state.point_a[1],
                "name": "A点"
            }])
            a_layer = pdk.Layer(
                "ScatterplotLayer",
                data=a_df,
                get_position=["lng", "lat"],
                get_color=[0, 255, 0, 255],
                get_radius=30,
                pickable=True,
                auto_highlight=True,
            )
            layers.append(a_layer)

        # 3. B 点标记
        if st.session_state.point_b:
            b_df = pd.DataFrame([{
                "lng": st.session_state.point_b[0],
                "lat": st.session_state.point_b[1],
                "name": "B点"
            }])
            b_layer = pdk.Layer(
                "ScatterplotLayer",
                data=b_df,
                get_position=["lng", "lat"],
                get_color=[255, 0, 0, 255],
                get_radius=30,
                pickable=True,
                auto_highlight=True,
            )
            layers.append(b_layer)

        # 4. 航线（A 到 B 连线）
        if st.session_state.point_a and st.session_state.point_b:
            line_df = pd.DataFrame([
                {"lng": st.session_state.point_a[0], "lat": st.session_state.point_a[1]},
                {"lng": st.session_state.point_b[0], "lat": st.session_state.point_b[1]}
            ])
            line_layer = pdk.Layer(
                "LineLayer",
                data=line_df,
                get_source_position=["lng", "lat"],
                get_target_position=["lng", "lat"],
                get_color=[0, 255, 255, 200],
                get_width=5,
                pickable=True,
            )
            layers.append(line_layer)

        # 如果没有设置任何点，显示提示信息
        if not layers:
            st.info("👈 请在左侧设置 A 点和 B 点，地图将自动显示")

        # 创建 PyDeck 地图（不需要任何 Token！）
        if layers:
            view_state = pdk.ViewState(
                longitude=CENTER_LNG,
                latitude=CENTER_LAT,
                zoom=15,
                pitch=50,  # 倾斜角度，产生3D效果
                bearing=0,
            )

            deck = pdk.Deck(
                layers=layers,
                initial_view_state=view_state,
                tooltip={"text": "{name}" if "{name}" else "障碍物高度: {height}m"},
                map_style="mapbox://styles/mapbox/satellite-streets-v12",  # 卫星+街道底图
            )

            st.pydeck_chart(deck, use_container_width=True)

            st.caption("📌 **图例**：🟢 绿色=A点 | 🔴 红色=B点 | 🔴 红色柱状=障碍物 | 🔵 青色线=规划航线")

# ==================== 飞行监控页面 ====================
elif page == "📡 飞行监控":
    st.title("📡 飞行监控 - 心跳包接收")

    # 自动更新心跳
    current_time = time.time()
    if current_time - st.session_state.last_heartbeat_time >= 2:
        new_hb = generate_heartbeat()
        st.session_state.heartbeat_history.insert(0, new_hb)
        if len(st.session_state.heartbeat_history) > 50:
            st.session_state.heartbeat_history.pop()
        st.session_state.last_heartbeat_time = current_time
        st.rerun()

    if st.session_state.heartbeat_history:
        latest = st.session_state.heartbeat_history[0]

        # 指标卡片
        col1, col2, col3, col4, col5, col6 = st.columns(6)
        with col1:
            st.metric("⏰ 时间", latest["timestamp"])
        with col2:
            st.metric("📍 纬度", f"{latest['lat']:.6f}")
        with col3:
            st.metric("📍 经度", f"{latest['lng']:.6f}")
        with col4:
            st.metric("📊 高度", f"{latest['altitude']} m")
        with col5:
            st.metric("🔋 电压", f"{latest['voltage']} V")
        with col6:
            st.metric("🛰️ 卫星", latest["satellites"])

        col7, col8, col9 = st.columns(3)
        with col7:
            st.metric("💨 速度", f"{latest['speed']} m/s")
        with col8:
            st.metric("🧭 航向", f"{latest['heading']}°")
        with col9:
            if st.session_state.point_a:
                a_lng, a_lat = st.session_state.point_a
                distance = math.sqrt((latest['lng'] - a_lng)**2 + (latest['lat'] - a_lat)**2) * 111000
                st.metric("📏 距A点", f"{distance:.0f} m")
            else:
                st.metric("📏 距A点", "未设置")

        # 实时位置地图
        st.markdown("---")
        st.subheader("📍 实时位置")
        pos_df = pd.DataFrame([{"lat": latest['lat'], "lon": latest['lng']}])
        st.map(pos_df, size=200)

        # 飞行轨迹
        if len(st.session_state.heartbeat_history) > 1:
            st.subheader("✈️ 飞行轨迹")
            track_df = pd.DataFrame(st.session_state.heartbeat_history[:20])
            st.map(track_df, latitude='lat', longitude='lng', size=50)
            st.caption("显示最近20个轨迹点")

        # 历史记录表格
        st.markdown("---")
        st.subheader("📋 历史心跳记录")
        
        col_filter1, col_filter2 = st.columns(2)
        with col_filter1:
            show_count = st.selectbox("显示条数", [10, 20, 50], index=0)
        with col_filter2:
            if st.button("🗑️ 清空历史", use_container_width=True):
                st.session_state.heartbeat_history = []
                st.rerun()

        df = pd.DataFrame(st.session_state.heartbeat_history[:show_count])
        st.dataframe(df, use_container_width=True)

        # 图表
        st.markdown("---")
        st.subheader("📈 数据图表")
        
        if len(st.session_state.heartbeat_history) > 1:
            chart_col1, chart_col2 = st.columns(2)
            with chart_col1:
                alt_df = pd.DataFrame({
                    "序号": range(len(st.session_state.heartbeat_history[:20])),
                    "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_history[:20]]
                })
                st.line_chart(alt_df, x="序号", y="高度(m)")
                st.caption("高度变化趋势")
            with chart_col2:
                volt_df = pd.DataFrame({
                    "序号": range(len(st.session_state.heartbeat_history[:20])),
                    "电压(V)": [h["voltage"] for h in st.session_state.heartbeat_history[:20]]
                })
                st.line_chart(volt_df, x="序号", y="电压(V)")
                st.caption("电压变化趋势")
    else:
        st.info("⏳ 等待心跳数据...")
        st.caption("心跳包将每2秒自动更新一次")

    # 手动刷新
    col_btn1, col_btn2, col_btn3 = st.columns(3)
    with col_btn1:
        if st.button("🔄 立即刷新", use_container_width=True):
            new_hb = generate_heartbeat()
            st.session_state.heartbeat_history.insert(0, new_hb)
            if len(st.session_state.heartbeat_history) > 50:
                st.session_state.heartbeat_history.pop()
            st.rerun()

    st.markdown("---")
    st.caption("💡 心跳包每2秒自动更新，模拟无人机实时遥测数据")
