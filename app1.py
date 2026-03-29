import streamlit as st
import folium
from streamlit_folium import st_folium
from streamlit_drawable_layer import drawable_layer
import time
import random
from datetime import datetime
import numpy as np

# -------------------------- 全局配置 --------------------------
st.set_page_config(page_title="无人机智能航线规划系统", layout="wide", page_icon="✈️")
st.title("✈️ 无人机智能化应用 - 项目Demo")

# -------------------------- 坐标系转换引擎 --------------------------
class CoordTransformer:
    _a = 6378245.0
    _ee = 0.00669342162296594323

    @staticmethod
    def _transform_lat(lon, lat):
        x = lon - 105.0
        y = lat - 35.0
        val = -100.0 + 2.0*x + 3.0*y + 0.2*y*y + 0.1*x*y + 0.2*abs(x)
        val += (20.0*np.sin(6.0*x*np.pi) + 20.0*np.sin(2.0*x*np.pi)) * 2.0 / 3.0
        val += (20.0*np.sin(y*np.pi) + 40.0*np.sin(y/3.0*np.pi)) * 2.0 / 3.0
        val += (160.0*np.sin(y/12.0*np.pi) + 320*np.sin(y*np.pi/30.0)) * 2.0 / 3.0
        return val

    @staticmethod
    def _transform_lon(lon, lat):
        x = lon - 105.0
        y = lat - 35.0
        val = 30.0 + x + 2.0*y + 0.1*x*x + 0.1*x*y + 0.1*abs(x)
        val += (20.0*np.sin(6.0*x*np.pi) + 20.0*np.sin(2.0*x*np.pi)) * 2.0 / 3.0
        val += (20.0*np.sin(x*np.pi) + 40.0*np.sin(x/3.0*np.pi)) * 2.0 / 3.0
        val += (150.0*np.sin(x/12.0*np.pi) + 300.0*np.sin(x/30.0*np.pi)) * 2.0 / 3.0
        return val

    @classmethod
    def wgs84_to_gcj02(cls, lon, lat):
        dlat = cls._transform_lon(lon-105.0, lat-35.0)
        dlon = cls._transform_lat(lon-105.0, lat-35.0)
        radlat = lat / 180.0 * np.pi
        magic = np.sin(radlat)
        magic = 1 - cls._ee * magic * magic
        sqrtmagic = np.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls._a * (1 - cls._ee)) / (magic * sqrtmagic) * np.pi)
        dlon = (dlon * 180.0) / (cls._a / sqrtmagic * np.cos(radlat) * np.pi)
        mlat = lat + dlat
        mlon = lon + dlon
        return round(mlon, 6), round(mlat, 6)

    @classmethod
    def gcj02_to_wgs84(cls, lon, lat):
        dlon = cls._transform_lon(lon-105.0, lat-35.0)
        dlat = cls._transform_lat(lon-105.0, lat-35.0)
        radlat = lat / 180.0 * np.pi
        magic = np.sin(radlat)
        magic = 1 - cls._ee * magic * magic
        sqrtmagic = np.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls._a * (1 - cls._ee)) / (magic * sqrtmagic) * np.pi)
        dlon = (dlon * 180.0) / (cls._a / sqrtmagic * np.cos(radlat) * np.pi)
        mlat = lat + dlat
        mlon = lon + dlon
        return round(2*lon - mlon, 6), round(2*lat - mlat, 6)

# -------------------------- 侧边栏 --------------------------
with st.sidebar:
    st.header("⚙️ 控制面板")
    coord_mode = st.radio("坐标系设置", ["WGS-84 (GPS原始)", "GCJ-02 (高德/百度)"])
    st.divider()

    # 起点 A（南京科技职业学院）
    st.subheader("📍 起点 A")
    a_lat = st.number_input("纬度", value=32.3015, format="%.6f", key="a_lat")
    a_lon = st.number_input("经度", value=118.8260, format="%.6f", key="a_lon")
    if st.button("✅ 应用 A 点"):
        if coord_mode == "GCJ-02 (高德/百度)":
            a_lon, a_lat = CoordTransformer.gcj02_to_wgs84(a_lon, a_lat)
        st.session_state['A'] = (a_lat, a_lon)
        st.success("A点已设置")

    # 终点 B（南京科技职业学院）
    st.subheader("🏁 终点 B")
    b_lat = st.number_input("纬度", value=32.3035, format="%.6f", key="b_lat")
    b_lon = st.number_input("经度", value=118.8280, format="%.6f", key="b_lon")
    if st.button("✅ 应用 B 点"):
        if coord_mode == "GCJ-02 (高德/百度)":
            b_lon, b_lat = CoordTransformer.gcj02_to_wgs84(b_lon, b_lat)
        st.session_state['B'] = (b_lat, b_lon)
        st.success("B点已设置")

    st.divider()
    st.subheader("📊 飞行参数")
    fly_height = st.slider("飞行高度 (m)", 20, 200, 80, step=5)
    fly_speed = st.slider("飞行速度 (m/s)", 3, 15, 8, step=1)

# -------------------------- 标签页 --------------------------
tab1, tab2 = st.tabs(["📝 航线规划 (3D地图)", "📡 飞行监控 (心跳包)"])

# -------------------------- 航线规划 --------------------------
with tab1:
    col_map, col_obs = st.columns([3, 1])
    with col_map:
        st.subheader("地图视图")
        center_lat, center_lon = st.session_state.get('A', (32.3025, 118.8270))

        m = folium.Map(location=[center_lat, center_lon], zoom_start=19)
        folium.TileLayer(tiles='https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', name="OpenStreetMap").add_to(m)

        if 'A' in st.session_state:
            folium.Marker(st.session_state['A'], icon=folium.Icon(color="red", icon="plane"), popup="起点 A").add_to(m)
        if 'B' in st.session_state:
   
