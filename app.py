import streamlit as st
import folium
from streamlit_folium import folium_static
import time

st.set_page_config(layout="wide")
st.title("校园路径规划 - 高德地图底图")

# 初始化 session_state
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None

# 地图中心点（校园内）
default_center = [32.23775, 118.7490]

# 障碍物列表（红色方块）
obstacles = [
    [32.2345, 118.7485],
    [32.2360, 118.7495],
    [32.2385, 118.7480],
    [32.2405, 118.7498],
    [32.2420, 118.7482],
]

# ------------------ 侧边栏控件 ------------------
st.sidebar.header("设置点坐标")
col_a1, col_a2 = st.sidebar.columns(2)
with col_a1:
    lat_a = st.number_input("A点纬度", value=32.2322, format="%.6f", key="lat_a")
with col_a2:
    lng_a = st.number_input("A点经度", value=118.7490, format="%.6f", key="lng_a")
if st.sidebar.button("设置A点 (输入值)", key="btn_a"):
    st.session_state.point_a = [lat_a, lng_a]
    st.sidebar.success(f"A点已设为 ({lat_a}, {lng_a})")

col_b1, col_b2 = st.sidebar.columns(2)
with col_b1:
    lat_b = st.number_input("B点纬度", value=32.2433, format="%.6f", key="lat_b")
with col_b2:
    lng_b = st.number_input("B点经度", value=118.7490, format="%.6f", key="lng_b")
if st.sidebar.button("设置B点 (输入值)", key="btn_b"):
    st.session_state.point_b = [lat_b, lng_b]
    st.sidebar.success(f"B点已设为 ({lat_b}, {lng_b})")

st.sidebar.markdown("---")
show_obstacles = st.sidebar.checkbox("显示障碍物 (红色方块)", value=True)
clear_points = st.sidebar.button("清除 A/B 点")
if clear_points:
    st.session_state.point_a = None
    st.session_state.point_b = None
    st.sidebar.info("已清除所有标记点")

# 显示当前已设置的点
st.sidebar.markdown("---")
st.sidebar.subheader("当前点状态")
st.sidebar.write(f"A点: {st.session_state.point_a if st.session_state.point_a else '未设置'}")
st.sidebar.write(f"B点: {st.session_state.point_b if st.session_state.point_b else '未设置'}")

# ------------------ 创建地图函数 ------------------
def create_map():
    # 高德地图矢量瓦片（中文，GCJ-02坐标系，适合国内）
    tiles = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
    attr = "高德地图"
    m = folium.Map(location=default_center, zoom_start=16, tiles=tiles, attr=attr)

    # 添加障碍物
    if show_obstacles:
        for obs in obstacles:
            folium.CircleMarker(
                location=obs,
                radius=8,
                color="red",
                fill=True,
                fill_color="red",
                fill_opacity=0.8,
                popup="障碍物",
            ).add_to(m)
            folium.Rectangle(
                bounds=[[obs[0]-0.0003, obs[1]-0.0003], [obs[0]+0.0003, obs[1]+0.0003]],
                color="red",
                fill=True,
                fill_opacity=0.3,
            ).add_to(m)

    # 添加 A 点
    if st.session_state.point_a:
        folium.Marker(
            location=st.session_state.point_a,
            popup="起点 A",
            icon=folium.Icon(color="green", icon="play", prefix="fa"),
        ).add_to(m)
        folium.CircleMarker(
            location=st.session_state.point_a,
            radius=6,
            color="green",
            fill=True,
            fill_opacity=0.7,
            popup="A",
        ).add_to(m)

    # 添加 B 点
    if st.session_state.point_b:
        folium.Marker(
            location=st.session_state.point_b,
            popup="终点 B",
            icon=folium.Icon(color="red", icon="stop", prefix="fa"),
        ).add_to(m)
        folium.CircleMarker(
            location=st.session_state.point_b,
            radius=6,
            color="red",
            fill=True,
            fill_opacity=0.7,
            popup="B",
        ).add_to(m)

    # 连线
    if st.session_state.point_a and st.session_state.point_b:
        folium.PolyLine(
            locations=[st.session_state.point_a, st.session_state.point_b],
            color="blue",
            weight=3,
            opacity=0.7,
            popup="A→B 直线",
        ).add_to(m)

    return m

# ------------------ 显示地图 ------------------
# 使用 folium_static 可以避免因按钮点击导致地图重新加载时闪烁
# 但每次 session_state 变化时，需要重新生成地图并刷新
map_obj = create_map()
folium_static(map_obj, width=1000, height=600)

# 额外添加一个心跳/状态显示（让用户知道页面正在运行）
st.info("✅ 地图已加载 | 使用左侧边栏设置 A/B 点 | 红色方块为障碍物")
