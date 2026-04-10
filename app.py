import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins

# 页面配置
st.set_page_config(layout="wide")
st.title("校园路径规划 - OpenStreetMap 底图")

# 初始化 session_state
if "point_a" not in st.session_state:
    st.session_state.point_a = None   # [lat, lng]
if "point_b" not in st.session_state:
    st.session_state.point_b = None

# 默认地图中心（校园内，A、B之间）
default_center = [32.23775, 118.7490]
default_zoom = 16

# 预设障碍物（红色方块）—— 示例：围绕A、B两点附近的一些坐标
# 你可以根据实际情况修改
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
    lat_a = st.number_input("A点纬度", value=32.2322, format="%.6f")
with col_a2:
    lng_a = st.number_input("A点经度", value=118.7490, format="%.6f")
if st.sidebar.button("设置A点 (输入值)"):
    st.session_state.point_a = [lat_a, lng_a]
    st.sidebar.success(f"A点已设为 ({lat_a}, {lng_a})")

col_b1, col_b2 = st.sidebar.columns(2)
with col_b1:
    lat_b = st.number_input("B点纬度", value=32.2433, format="%.6f")
with col_b2:
    lng_b = st.number_input("B点经度", value=118.7490, format="%.6f")
if st.sidebar.button("设置B点 (输入值)"):
    st.session_state.point_b = [lat_b, lng_b]
    st.sidebar.success(f"B点已设为 ({lat_b}, {lng_b})")

st.sidebar.markdown("---")
show_obstacles = st.sidebar.checkbox("显示障碍物 (红色方块)", value=True)
clear_points = st.sidebar.button("清除 A/B 点")
if clear_points:
    st.session_state.point_a = None
    st.session_state.point_b = None
    st.sidebar.info("已清除所有标记点")

# ------------------ 创建地图 ------------------
m = folium.Map(location=default_center, zoom_start=default_zoom, tiles="OpenStreetMap")

# 添加点击获取坐标的功能（点击地图任意位置，在控制台打印，也可用于设置点）
# 这里使用 LatLngPopup 插件，点击后会弹出坐标，方便手动记录
m.add_child(folium.LatLngPopup())

# 添加鼠标坐标显示
plugins.MousePosition().add_to(m)

# 添加障碍物（红色方块）：使用 folium.Rectangle 或者 Marker 带方块图标
# 为清晰起见，使用 CircleMarker 并设置为红色正方形样式
if show_obstacles:
    for obs in obstacles:
        folium.CircleMarker(
            location=obs,
            radius=8,
            color="red",
            fill=True,
            fill_color="red",
            fill_opacity=0.8,
            weight=2,
            popup="障碍物",
        ).add_to(m)
        # 可选：添加一个矩形框（更接近“方块”）
        folium.Rectangle(
            bounds=[[obs[0]-0.0003, obs[1]-0.0003], [obs[0]+0.0003, obs[1]+0.0003]],
            color="red",
            fill=True,
            fill_opacity=0.4,
        ).add_to(m)

# 添加 A 点标记
if st.session_state.point_a:
    folium.Marker(
        location=st.session_state.point_a,
        popup="起点 A",
        icon=folium.Icon(color="green", icon="play", prefix="fa"),
    ).add_to(m)
    # 也可用圆点标记
    folium.CircleMarker(
        location=st.session_state.point_a,
        radius=6,
        color="green",
        fill=True,
        popup="A",
    ).add_to(m)

# 添加 B 点标记
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
        popup="B",
    ).add_to(m)

# 如果两个点都存在，可以绘制一条连线（示例）
if st.session_state.point_a and st.session_state.point_b:
    folium.PolyLine(
        locations=[st.session_state.point_a, st.session_state.point_b],
        color="blue",
        weight=3,
        opacity=0.7,
        popup="A→B 直线",
    ).add_to(m)

# ------------------ 显示地图 ------------------
# 使用 st_folium 捕获地图点击事件（可选）
output = st_folium(m, width=900, height=600, returned_objects=[])

# 可选：从地图点击中获取坐标并设置点（高级功能）
# 因为 LatLngPopup 已经弹窗，此处不再重复实现自动赋值，有需要可扩展
