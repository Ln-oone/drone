
import streamlit as st
import pydeck as pdk
import pandas as pd
import numpy as np

# --------------------------
# 1. 坐标系转换工具 (WGS-84 ↔ GCJ-02)
# --------------------------
x_pi = 3.14159265358979324 * 3000.0 / 180.0
pi = 3.1415926535897932384626  # π
a = 6378245.0  # 长半轴
ee = 0.00669342162296594323  # 偏心率平方

def gcj02_to_wgs84(lng, lat):
    """GCJ-02 转 WGS-84"""
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * np.cos(radlat) * pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return [lng * 2 - mglng, lat * 2 - mglat]

def wgs84_to_gcj02(lng, lat):
    """WGS-84 转 GCJ-02"""
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * np.cos(radlat) * pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return [mglng, mglat]

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * np.sqrt(np.fabs(lng))
    ret += (20.0 * np.sin(6.0 * lng * pi) + 20.0 * np.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * np.sin(lat * pi) + 40.0 * np.sin(lat / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * np.sin(lat / 12.0 * pi) + 320.0 * np.sin(lat * pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * np.sqrt(np.fabs(lng))
    ret += (20.0 * np.sin(6.0 * lng * pi) + 20.0 * np.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * np.sin(lng * pi) + 40.0 * np.sin(lng / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 * np.sin(lng / 12.0 * pi) + 300.0 * np.sin(lng / 30.0 * pi)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    """判断是否在国内 (国内才需要偏移)"""
    return not (73.66 < lng < 135.05 and 3.86 < lat < 53.55)

# --------------------------
# 2. Streamlit 页面初始化
# --------------------------
st.set_page_config(layout="wide", page_title="无人机航线规划Demo")
st.title("分组作业3-项目Demo")

# 侧边栏导航
nav = st.sidebar.radio("导航", ["航线规划", "飞行监控"])
st.sidebar.subheader("坐标系设置")
coord_system = st.sidebar.radio("输入坐标系", ["WGS-84", "GCJ-02(高德/百度)"])
st.sidebar.subheader("系统状态")

# --------------------------
# 3. 航线规划页面 (主地图界面)
# --------------------------
if nav == "航线规划":
    st.subheader("🗺️ 地图")
    col1, col2 = st.columns([3, 1])

    with col2:
        st.subheader("⚙️ 控制面板")
        # 起点A设置
        st.markdown("**起点A**")
        lat_a = st.number_input("纬度A", value=32.2322, format="%.4f")
        lng_a = st.number_input("经度A", value=118.749, format="%.4f")
        set_a = st.button("设置A点")

        # 终点B设置
        st.markdown("**终点B**")
        lat_b = st.number_input("纬度B", value=32.2343, format="%.4f")
        lng_b = st.number_input("经度B", value=118.749, format="%.4f")
        set_b = st.button("设置B点")

        # 飞行高度
        st.subheader("✈️ 飞行参数")
        flight_height = st.slider("设定飞行高度(m)", 10, 200, 50)

    with col1:
        # 坐标转换逻辑
        if coord_system == "GCJ-02(高德/百度)":
            lng_a_wgs, lat_a_wgs = gcj02_to_wgs84(lng_a, lat_a)
            lng_b_wgs, lat_b_wgs = gcj02_to_wgs84(lng_b, lat_b)
        else:
            lng_a_wgs, lat_a_wgs = lng_a, lat_a
            lng_b_wgs, lat_b_wgs = lng_b, lat_b

        # 准备点数据
        points = []
        if set_a:
            points.append({"name": "A", "lat": lat_a_wgs, "lon": lng_a_wgs, "color": [255, 0, 0]})
            st.sidebar.success("A点已设")
        if set_b:
            points.append({"name": "B", "lat": lat_b_wgs, "lon": lng_b_wgs, "color": [0, 255, 0]})
            st.sidebar.success("B点已设")

        # 绘制3D地图 (pydeck)
        if points:
            df = pd.DataFrame(points)
            view_state = pdk.ViewState(
                latitude=np.mean([lat_a_wgs, lat_b_wgs]),
                longitude=np.mean([lng_a_wgs, lng_b_wgs]),
                zoom=17,
                pitch=45,
                bearing=0
            )
            layer = pdk.Layer(
               
