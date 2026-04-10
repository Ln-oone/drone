import streamlit as st
import pandas as pd
import random
import time
import math
import json
from datetime import datetime
import requests

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统", layout="wide", initial_sidebar_state="expanded")

# ==================== 坐标系转换函数 ====================
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
def generate_heartbeat():
    """生成模拟心跳数据"""
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
if "mapbox_token" not in st.session_state:
    # 请在这里填入你的 Mapbox Token
    # 注册地址：https://account.mapbox.com/access-tokens/
    st.session_state.mapbox_token = "YOUR_MAPBOX_TOKEN_HERE"

# ==================== 障碍物定义（校园内，WGS84坐标） ====================
# A点(32.2322, 118.7490) 和 B点(32.2433, 118.7490) 之间的障碍物
obstacles = [
    {"name": "图书馆", "lng": 118.7485, "lat": 32.2345, "height": 25},
    {"name": "教学楼", "lng": 118.7495, "lat": 32.2360, "height": 30},
    {"name": "实验楼", "lng": 118.7480, "lat": 32.2385, "height": 28},
    {"name": "学生食堂", "lng": 118.7498, "lat": 32.2405, "height": 20},
    {"name": "体育馆", "lng": 118.7482, "lat": 32.2420, "height": 22},
]

# ==================== 侧边栏导航 ====================
st.sidebar.title("🎛️ 导航菜单")
page = st.sidebar.radio(
    "选择功能模块",
    ["🗺️ 航线规划", "📡 飞行监控"],
    format_func=lambda x: x
)

st.sidebar.markdown("---")
st.sidebar.info(
    "**系统状态**\n\n"
    f"- A点: {'✅ 已设' if st.session_state.point_a else '❌ 未设'}\n"
    f"- B点: {'✅ 已设' if st.session_state.point_b else '❌ 未设'}\n"
    f"- 障碍物: {len(obstacles)}个\n"
    f"- 心跳包: {len(st.session_state.heartbeat_history)}条"
)

# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.title("🗺️ 航线规划 - 3D地图")
    
    col1, col2 = st.columns([1, 2])
    
    with col1:
        st.markdown("### 🎮 控制面板")
        
        # 坐标系选择
        coord_sys = st.radio(
            "📐 输入坐标系",
            ["WGS-84", "GCJ-02 (高德/百度)"],
            index=1,
            help="选择你输入的坐标所使用的坐标系"
        )
        
        st.markdown("---")
        
        # 起点 A 设置
        st.markdown("#### 🟢 起点 A")
        a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
        
        if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
            else:
                wgs_lng, wgs_lat = a_lng, a_lat
            st.session_state.point_a = [wgs_lng, wgs_lat]
            st.success(f"✅ A点已设置 (WGS84: {wgs_lng:.6f}, {wgs_lat:.6f})")
        
        st.markdown("---")
        
        # 终点 B 设置
        st.markdown("#### 🔴 终点 B")
        b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
        
        if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
            else:
                wgs_lng, wgs_lat = b_lng, b_lat
            st.session_state.point_b = [wgs_lng, wgs_lat]
            st.success(f"✅ B点已设置 (WGS84: {wgs_lng:.6f}, {wgs_lat:.6f})")
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("#### ✈️ 飞行参数")
        flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5, min_value=20, max_value=200)
        
        st.markdown("---")
        
        # 操作按钮
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("🗑️ 清除所有", use_container_width=True):
                st.session_state.point_a = None
                st.session_state.point_b = None
                st.info("已清除所有航点")
        with col_btn2:
            if st.button("🔄 重置地图", use_container_width=True):
                st.rerun()
        
        # 显示当前设置的坐标
        st.markdown("---")
        st.markdown("### 📍 当前航点")
        if st.session_state.point_a:
            st.success(f"A点: {st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f}")
        else:
            st.warning("A点未设置")
        
        if st.session_state.point_b:
            st.success(f"B点: {st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f}")
        else:
            st.warning("B点未设置")
        
        # 障碍物列表
        st.markdown("---")
        st.markdown("### 🚧 障碍物列表")
        for obs in obstacles:
            st.caption(f"• {obs['name']}: ({obs['lng']:.4f}, {obs['lat']:.4f}) 高度{obs['height']}m")
    
    with col2:
        st.markdown("### 🗺️ 3D 地图视图")
        
        # 检查 Mapbox Token
        if st.session_state.mapbox_token == "YOUR_MAPBOX_TOKEN_HERE":
            st.error("""
            ⚠️ **需要配置 Mapbox Token**
            
            要使用3D地图功能，请：
            1. 访问 [Mapbox官网](https://account.mapbox.com/access-tokens/) 注册免费账号
            2. 获取你的 Access Token
            3. 在代码中替换 `YOUR_MAPBOX_TOKEN_HERE`
            
            或者使用下方2D地图作为替代：
            """)
            
            # 备选：2D 地图
            st.markdown("#### 备选 2D 地图")
            map_data = []
            if st.session_state.point_a:
                map_data.append({"lat": st.session_state.point_a[1], "lon": st.session_state.point_a[0], "type": "A"})
            if st.session_state.point_b:
                map_data.append({"lat": st.session_state.point_b[1], "lon": st.session_state.point_b[0], "type": "B"})
            for obs in obstacles:
                map_data.append({"lat": obs["lat"], "lon": obs["lng"], "type": "障碍物"})
            
            if map_data:
                df = pd.DataFrame(map_data)
                st.map(df, latitude='lat', longitude='lon', size=100)
            else:
                st.info("请先在左侧设置 A 点和 B 点")
        else:
            # 使用 Mapbox GL JS 实现 3D 地图
            mapbox_html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset="utf-8">
                <title>3D Map</title>
                <meta name="viewport" content="initial-scale=1,maximum-scale=1,user-scalable=no">
                <link href="https://api.mapbox.com/mapbox-gl-js/v3.0.0/mapbox-gl.css" rel="stylesheet">
                <script src="https://api.mapbox.com/mapbox-gl-js/v3.0.0/mapbox-gl.js"></script>
                <style>
                    body {{ margin: 0; padding: 0; }}
                    #map {{ position: absolute; top: 0; bottom: 0; width: 100%; height: 600px; }}
                </style>
            </head>
            <body>
                <div id="map"></div>
                <script>
                    mapboxgl.accessToken = '{st.session_state.mapbox_token}';
                    
                    const center = [118.7490, 32.23775];
                    
                    const map = new mapboxgl.Map({{
                        container: 'map',
                        style: 'mapbox://styles/mapbox/satellite-streets-v12',
                        center: center,
                        zoom: 16,
                        pitch: 60,
                        bearing: 45,
                        antialias: true
                    }});
                    
                    // 添加地形
                    map.on('load', () => {{
                        map.addSource('mapbox-dem', {{
                            'type': 'raster-dem',
                            'url': 'mapbox://mapbox.mapbox-terrain-dem-v1',
                            'tileSize': 512,
                            'maxzoom': 14
                        }});
                        map.setTerrain({{ 'source': 'mapbox-dem', 'exaggeration': 1.5 }});
                        
                        // 添加 3D 建筑物
                        map.addLayer({{
                            'id': '3d-buildings',
                            'source': 'composite',
                            'source-layer': 'building',
                            'filter': ['==', 'extrude', 'true'],
                            'type': 'fill-extrusion',
                            'minzoom': 15,
                            'paint': {{
                                'fill-extrusion-color': '#aaa',
                                'fill-extrusion-height': ['get', 'height'],
                                'fill-extrusion-base': ['get', 'min_height'],
                                'fill-extrusion-opacity': 0.6
                            }}
                        }});
                    }});
                    
                    // 添加障碍物标记
                    const obstacles = {json.dumps(obstacles)};
                    obstacles.forEach(obs => {{
                        const el = document.createElement('div');
                        el.className = 'marker';
                        el.style.backgroundColor = 'red';
                        el.style.width = '20px';
                        el.style.height = '20px';
                        el.style.borderRadius = '4px';
                        el.style.border = '2px solid white';
                        el.style.cursor = 'pointer';
                        
                        new mapboxgl.Marker(el)
                            .setLngLat([obs.lng, obs.lat])
                            .setPopup(new mapboxgl.Popup().setHTML(`
                                <strong>${{obs.name}}</strong><br>
                                高度: ${{obs.height}}m
                            `))
                            .addTo(map);
                    }});
                    
                    // 添加 A 点标记
                    {f'''
                    if ({json.dumps(st.session_state.point_a)}) {{
                        const aEl = document.createElement('div');
                        aEl.innerHTML = '🟢';
                        aEl.style.fontSize = '30px';
                        aEl.style.cursor = 'pointer';
                        new mapboxgl.Marker(aEl)
                            .setLngLat({json.dumps(st.session_state.point_a)})
                            .setPopup(new mapboxgl.Popup().setHTML('<strong>起点 A</strong>'))
                            .addTo(map);
                    }}
                    ''' if st.session_state.point_a else ''}
                    
                    // 添加 B 点标记
                    {f'''
                    if ({json.dumps(st.session_state.point_b)}) {{
                        const bEl = document.createElement('div');
                        bEl.innerHTML = '🔴';
                        bEl.style.fontSize = '30px';
                        bEl.style.cursor = 'pointer';
                        new mapboxgl.Marker(bEl)
                            .setLngLat({json.dumps(st.session_state.point_b)})
                            .setPopup(new mapboxgl.Popup().setHTML('<strong>终点 B</strong>'))
                            .addTo(map);
                    }}
                    ''' if st.session_state.point_b else ''}
                    
                    // 添加航线（如果 A 和 B 都存在）
                    {f'''
                    if ({json.dumps(st.session_state.point_a)} && {json.dumps(st.session_state.point_b)}) {{
                        map.on('load', () => {{
                            map.addSource('route', {{
                                'type': 'geojson',
                                'data': {{
                                    'type': 'Feature',
                                    'properties': {{}},
                                    'geometry': {{
                                        'type': 'LineString',
                                        'coordinates': [
                                            {json.dumps(st.session_state.point_a)},
                                            {json.dumps(st.session_state.point_b)}
                                        ]
                                    }}
                                }}
                            }});
                            map.addLayer({{
                                'id': 'route',
                                'type': 'line',
                                'source': 'route',
                                'layout': {{
                                    'line-join': 'round',
                                    'line-cap': 'round'
                                }},
                                'paint': {{
                                    'line-color': '#00ff00',
                                    'line-width': 4,
                                    'line-opacity': 0.8
                                }}
                            }});
                        }});
                    }}
                    ''' if st.session_state.point_a and st.session_state.point_b else ''}
                    
                    // 添加比例尺
                    map.addControl(new mapboxgl.NavigationControl());
                    map.addControl(new mapboxgl.ScaleControl());
                </script>
            </body>
            </html>
            """
            
            # 嵌入 HTML
            from streamlit.components.v1 import html
            html(mapbox_html, height=620)
        
        st.caption("💡 **提示**：3D地图支持倾斜视角（按住右键拖拽旋转），可放大查看地形和建筑物")

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
    
    # 显示最新心跳数据
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
        
        # 第二行指标
        col7, col8, col9 = st.columns(3)
        with col7:
            st.metric("💨 速度", f"{latest['speed']} m/s")
        with col8:
            st.metric("🧭 航向", f"{latest['heading']}°")
        with col9:
            # 计算与A点的距离
            if st.session_state.point_a:
                a_lng, a_lat = st.session_state.point_a
                distance = math.sqrt((latest['lng'] - a_lng)**2 + (latest['lat'] - a_lat)**2) * 111000
                st.metric("📏 距A点", f"{distance:.0f} m")
            else:
                st.metric("📏 距A点", "未设置")
        
        # 实时位置地图
        st.markdown("---")
        st.subheader("📍 实时位置")
        pos_df = pd.DataFrame([{
            "lat": latest['lat'],
            "lon": latest['lng']
        }])
        st.map(pos_df, size=200)
        
        # 飞行轨迹
        st.subheader("✈️ 飞行轨迹")
        if len(st.session_state.heartbeat_history) > 1:
            track_df = pd.DataFrame(st.session_state.heartbeat_history[:20])
            st.map(track_df, latitude='lat', longitude='lng', size=50)
            st.caption("显示最近20个轨迹点")
        
        # 历史记录表格
        st.markdown("---")
        st.subheader("📋 历史心跳记录")
        
        # 添加筛选功能
        col_filter1, col_filter2 = st.columns(2)
        with col_filter1:
            show_count = st.selectbox("显示条数", [10, 20, 50], index=0)
        with col_filter2:
            if st.button("🗑️ 清空历史", use_container_width=True):
                st.session_state.heartbeat_history = []
                st.rerun()
        
        df = pd.DataFrame(st.session_state.heartbeat_history[:show_count])
        st.dataframe(df, use_container_width=True)
        
        # 图表展示
        st.markdown("---")
        st.subheader("📈 数据图表")
        
        chart_col1, chart_col2 = st.columns(2)
        with chart_col1:
            if len(st.session_state.heartbeat_history) > 1:
                alt_df = pd.DataFrame({
                    "时间": range(len(st.session_state.heartbeat_history[:20])),
                    "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_history[:20]]
                })
                st.line_chart(alt_df, x="时间", y="高度(m)")
                st.caption("高度变化趋势")
        
        with chart_col2:
            if len(st.session_state.heartbeat_history) > 1:
                volt_df = pd.DataFrame({
                    "时间": range(len(st.session_state.heartbeat_history[:20])),
                    "电压(V)": [h["voltage"] for h in st.session_state.heartbeat_history[:20]]
                })
                st.line_chart(volt_df, x="时间", y="电压(V)")
                st.caption("电压变化趋势")
    else:
        st.info("⏳ 等待心跳数据...")
        st.caption("心跳包将每2秒自动更新一次")
    
    # 手动刷新按钮
    col_btn1, col_btn2, col_btn3 = st.columns(3)
    with col_btn1:
        if st.button("🔄 立即刷新", use_container_width=True):
            new_hb = generate_heartbeat()
            st.session_state.heartbeat_history.insert(0, new_hb)
            if len(st.session_state.heartbeat_history) > 50:
                st.session_state.heartbeat_history.pop()
            st.rerun()
    
    with col_btn2:
        if st.button("📊 导出数据", use_container_width=True):
            if st.session_state.heartbeat_history:
                df_export = pd.DataFrame(st.session_state.heartbeat_history)
                csv = df_export.to_csv(index=False)
                st.download_button(
                    label="下载 CSV",
                    data=csv,
                    file_name=f"heartbeat_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                    mime="text/csv"
                )
    
    st.markdown("---")
    st.caption("💡 心跳包每2秒自动更新一次，模拟无人机实时遥测数据")

# ==================== 页脚 ====================
st.sidebar.markdown("---")
st.sidebar.caption("© 2025 无人机地面站系统 | v1.0")
