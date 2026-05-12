import streamlit as st
import pandas as pd
import numpy as np
import json
import os
import time
import math
import uuid
from datetime import datetime
from streamlit.components.v1 import html
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import re

# ---------- 页面配置 ----------
st.set_page_config(page_title="无人机地面站仿真平台", layout="wide", page_icon="✈️")

# ---------- 会话状态初始化 ----------
session_defaults = {
    "module": "航线规划",
    "map_type": "satellite",
    "speed_ratio": 0.5,
    "flight_height": 80.0,
    "safety_radius": 10.0,
    "auto_save": True,
    "start_point": {"lng": 116.397428, "lat": 39.909187, "name": "起点"},
    "end_point": {"lng": 116.407526, "lat": 39.904203, "name": "终点"},
    "obstacles": [],
    "current_paths": {"best": [], "left": [], "right": []},
    "path_strategy": "best",
    "flight_simulation": False,
    "flight_data_log": [],
    "flight_position": None,
    "flight_start_time": None,
    "waypoints": [],
    "current_waypoint_idx": 0,
    "history_flights": [],
    "obstacle_next_id": 1,
}
for key, val in session_defaults.items():
    if key not in st.session_state:
        st.session_state[key] = val

# 备份恢复和持久化
CONFIG_FILE = "ground_station_config.json"
BACKUP_FOLDER = "config_backups"
MAX_BACKUPS = 10
os.makedirs(BACKUP_FOLDER, exist_ok=True)

def save_config_to_file():
    config = {
        "obstacles": st.session_state.obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "safety_radius": st.session_state.safety_radius,
        "flight_height": st.session_state.flight_height,
        "speed_ratio": st.session_state.speed_ratio,
    }
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f)
    if st.session_state.auto_save:
        backup_name = f"backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(os.path.join(BACKUP_FOLDER, backup_name), "w") as bf:
            json.dump(config, bf)
        backups = sorted([f for f in os.listdir(BACKUP_FOLDER) if f.endswith(".json")])
        while len(backups) > MAX_BACKUPS:
            os.remove(os.path.join(BACKUP_FOLDER, backups.pop(0)))

def load_config_from_file():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)
            st.session_state.obstacles = config.get("obstacles", [])
            st.session_state.start_point = config.get("start_point", session_defaults["start_point"])
            st.session_state.end_point = config.get("end_point", session_defaults["end_point"])
            st.session_state.safety_radius = config.get("safety_radius", 10.0)
            st.session_state.flight_height = config.get("flight_height", 80.0)
            st.session_state.speed_ratio = config.get("speed_ratio", 0.5)

# ---------- 几何辅助函数 ----------
def lat_lon_to_meters(lat, lon):
    """简易经纬度转米 (近似)"""
    x = lon * 111320 * math.cos(math.radians(lat))
    y = lat * 110540
    return x, y

def meters_to_lat_lon(x, y, ref_lat, ref_lon):
    dx = x / (111320 * math.cos(math.radians(ref_lat)))
    dy = y / 110540
    return ref_lat + dy, ref_lon + dx

def create_polygon_from_coords(coords):
    return Polygon(coords)

def is_intersecting_with_obstacles(line_coords, obstacle_polygons, safe_radius):
    line = LineString(line_coords)
    buffered_line = line.buffer(safe_radius)
    for poly in obstacle_polygons:
        if buffered_line.intersects(poly):
            return True
    return False

def point_to_polygon_distance(point, polygon):
    return Point(point).distance(polygon)

# ---------- 避障路径规划算法 ----------
def generate_avoidance_path(start, end, obstacles, safety_radius, flight_height, strategy="best"):
    start_pt = Point(start["lng"], start["lat"])
    end_pt = Point(end["lng"], end["lat"])
    direct_line = LineString([start_pt, end_pt])
    # 筛选需要避让的障碍物（高度超过飞行高度）
    threatening = [obs for obs in obstacles if obs.get("height", 0) > flight_height]
    threat_polygons = [Polygon(obs["coords"]) for obs in threatening if len(obs["coords"]) >= 3]
    # 检测直线是否碰撞
    if not is_intersecting_with_obstacles([(start["lng"], start["lat"]), (end["lng"], end["lat"])], threat_polygons, safety_radius):
        return [start, end]  # 无碰撞直接返回

    # 生成绕行点: 左绕/右绕基于障碍物凸包最左/最右切点简化
    waypoints = [start]
    current = start_pt
    target = end_pt
    max_iter = 20
    for _ in range(max_iter):
        # 找到从current到target路径上第一个阻挡的障碍物
        blocking = None
        for poly in threat_polygons:
            if LineString([current, target]).intersects(poly.buffer(safety_radius)):
                blocking = poly
                break
        if not blocking:
            waypoints.append({"lng": target.x, "lat": target.y})
            break
        # 计算绕行点（简化：取多边形最接近直线的点）
        nearest = nearest_points(blocking, LineString([current, target]))[0]
        if strategy == "left":
            # 取多边形质心左侧偏移点
            centroid = blocking.centroid
            angle = math.atan2(target.y - current.y, target.x - current.x) + math.pi/2
            offset = safety_radius * 2
            bypass = Point(centroid.x + offset * math.cos(angle), centroid.y + offset * math.sin(angle))
        elif strategy == "right":
            centroid = blocking.centroid
            angle = math.atan2(target.y - current.y, target.x - current.x) - math.pi/2
            offset = safety_radius * 2
            bypass = Point(centroid.x + offset * math.cos(angle), centroid.y + offset * math.sin(angle))
        else:  # best - 选最短路径绕行点
            boundary = blocking.exterior
            distances = [LineString([current, Point(p)]).length + LineString([Point(p), target]).length for p in boundary.coords]
            min_idx = np.argmin(distances)
            bypass = Point(boundary.coords[min_idx])
        waypoints.append({"lng": bypass.x, "lat": bypass.y})
        current = bypass
    else:
        waypoints.append(end)
    return waypoints

def recalc_paths():
    obstacles = st.session_state.obstacles
    start = st.session_state.start_point
    end = st.session_state.end_point
    safety = st.session_state.safety_radius
    height = st.session_state.flight_height
    st.session_state.current_paths = {
        "best": generate_avoidance_path(start, end, obstacles, safety, height, "best"),
        "left": generate_avoidance_path(start, end, obstacles, safety, height, "left"),
        "right": generate_avoidance_path(start, end, obstacles, safety, height, "right"),
    }

# ---------- 心跳模拟 ----------
def generate_heartbeat():
    if not st.session_state.flight_simulation or st.session_state.flight_position is None:
        return None
    waypoints = st.session_state.waypoints
    idx = st.session_state.current_waypoint_idx
    if idx >= len(waypoints):
        return None
    current_pos = st.session_state.flight_position
    target = waypoints[idx]
    # 距离检查
    dist = math.hypot(target["lng"] - current_pos["lng"], target["lat"] - current_pos["lat"]) * 111000
    speed_ms = 5 * st.session_state.speed_ratio  # 最大5m/s
    if dist < 1.0:
        st.session_state.current_waypoint_idx += 1
        if st.session_state.current_waypoint_idx >= len(waypoints):
            st.session_state.flight_simulation = False
            st.success("🎉 任务完成！到达终点！")
            return None
        return generate_heartbeat()
    # 移动到下一位置
    angle = math.atan2(target["lat"] - current_pos["lat"], target["lng"] - current_pos["lng"])
    step = speed_ms / 111000
    new_lng = current_pos["lng"] + step * math.cos(angle)
    new_lat = current_pos["lat"] + step * math.sin(angle)
    st.session_state.flight_position = {"lng": new_lng, "lat": new_lat}
    elapsed = time.time() - st.session_state.flight_start_time if st.session_state.flight_start_time else 0
    total_dist = sum(math.hypot(waypoints[i+1]["lng"]-waypoints[i]["lng"], waypoints[i+1]["lat"]-waypoints[i]["lat"]) for i in range(len(waypoints)-1))*111000
    remaining_dist = sum(math.hypot(waypoints[i+1]["lng"]-waypoints[i]["lng"], waypoints[i+1]["lat"]-waypoints[i]["lat"]) for i in range(idx, len(waypoints)-1))*111000 + dist
    progress = (total_dist - remaining_dist)/total_dist if total_dist>0 else 0
    voltage = 22.2 - (progress * 2)
    satellites = np.random.randint(6, 18)
    return {
        "timestamp": datetime.now(),
        "lat": new_lat, "lng": new_lng,
        "progress": progress*100,
        "current_waypoint": idx+1, "total_waypoints": len(waypoints),
        "speed": speed_ms, "elapsed_time": elapsed,
        "remaining_dist": remaining_dist, "eta": remaining_dist/speed_ms if speed_ms>0 else 0,
        "voltage": max(16, voltage), "satellites": satellites,
        "flight_status": "正常"
    }

# 每帧更新飞行
def update_flight():
    if not st.session_state.flight_simulation:
        return
    data = generate_heartbeat()
    if data:
        st.session_state.flight_data_log.append(data)
        # 闯入安全半径告警
        for obs in st.session_state.obstacles:
            if obs.get("height",0) > st.session_state.flight_height:
                poly = Polygon(obs["coords"])
                if point_to_polygon_distance((data["lng"], data["lat"]), poly) < st.session_state.safety_radius:
                    st.warning(f"⚠️ 闯入障碍物 {obs['name']} 安全半径内！")
                    break

# ---------- 地图HTML组件 ----------
def render_map(center=None, zoom=15, edit_mode=False):
    if center is None and "flight_position" in st.session_state and st.session_state.flight_position:
        center = st.session_state.flight_position
    elif center is None:
        center = st.session_state.start_point
    map_type = "satellite" if st.session_state.map_type == "satellite" else "vector"
    paths = st.session_state.current_paths
    strategy = st.session_state.path_strategy
    current_route = paths.get(strategy, [])
    obstacles = st.session_state.obstacles
    start = st.session_state.start_point
    end = st.session_state.end_point
    safety_radius = st.session_state.safety_radius
    flight_pos = st.session_state.flight_position
    # 生成多边形样式
    obstacle_geojson = []
    for obs in obstacles:
        is_danger = obs.get("height", 0) > st.session_state.flight_height
        color = "red" if is_danger else "orange"
        coords = [[p["lng"], p["lat"]] for p in obs["coords"]]
        if len(coords)>=3:
            obstacle_geojson.append({
                "type": "Feature", "properties": {"name": obs["name"], "height": obs["height"], "color": color},
                "geometry": {"type": "Polygon", "coordinates": [coords]}
            })
    route_coords = [[p["lng"], p["lat"]] for p in current_route]
    route_color = "green" if strategy=="best" else ("purple" if strategy=="left" else "orange")
    # 构建JS代码（简化）
    return f"""
    <div id="map" style="width:100%; height:500px;"></div>
    <link rel="stylesheet" href="https://webapi.amap.com/maps?v=2.0&key=YOUR_KEY">
    <script src="https://webapi.amap.com/maps?v=2.0&key=YOUR_KEY"></script>
    <script>
        var map = new AMap.Map('map', {{ zoom: {zoom}, center: [{center["lng"]}, {center["lat"}]], viewMode: '2D' }});
        // 添加卫星/矢量图层
        var layer = new AMap.{'TileLayer.Satellite' if map_type=='satellite' else 'TileLayer'}();
        map.add(layer);
        // 添加障碍物
        var obstacles = {json.dumps(obstacle_geojson)};
        obstacles.forEach(obs => {{
            var polygon = new AMap.Polygon({{ path: obs.geometry.coordinates[0], fillColor: obs.properties.color, strokeColor: "black", fillOpacity:0.4 }});
            map.add(polygon);
        }});
        // 绘制路线
        var routeLine = new AMap.Polyline({{ path: {json.dumps(route_coords)}, strokeColor: "{route_color}", lineWidth: 4 }});
        map.add(routeLine);
        // 起点终点标记
        var startMarker = new AMap.Marker({{ position: [{start["lng"]}, {start["lat"]}], title: "起点", icon: "//a.amap.com/jsapi_demos/static/demo-center/icons/start.png" }});
        var endMarker = new AMap.Marker({{ position: [{end["lng"]}, {end["lat"]}], title: "终点", icon: "//a.amap.com/jsapi_demos/static/demo-center/icons/end.png" }});
        map.add(startMarker); map.add(endMarker);
        // 飞行位置
        if ({json.dumps(flight_pos)}) {{
            var flightMarker = new AMap.Marker({{ position: [{flight_pos["lng"]}, {flight_pos["lat"]}], title: "无人机", icon: "https://webapi.amap.com/theme/v1.3/markers/n/mark_b.png" }});
            map.add(flightMarker);
        }}
        // 安全半径圆
        var circle = new AMap.Circle({{ center: [{center["lng"]}, {center["lat"]}], radius: {safety_radius}, strokeColor: "blue", fillColor: "blue", fillOpacity:0.1 }});
        map.add(circle);
    </script>
    """

# ---------- 侧边栏UI ----------
with st.sidebar:
    st.title("⚙️ 控制中心")
    st.session_state.module = st.radio("功能模块", ["航线规划", "飞行监控", "障碍物管理"])
    st.session_state.map_type = st.radio("地图类型", ["卫星影像", "矢量街道"], index=0, horizontal=True)
    st.session_state.speed_ratio = st.slider("速度系数", 0.1, 1.0, st.session_state.speed_ratio)
    st.session_state.flight_height = st.slider("飞行高度(m)", 10, 200, int(st.session_state.flight_height))
    st.session_state.safety_radius = st.slider("安全半径(m)", 1, 20, int(st.session_state.safety_radius))
    st.session_state.auto_save = st.checkbox("自动保存配置", st.session_state.auto_save)
    if st.button("保存当前配置"):
        save_config_to_file()
        st.success("配置已保存")
    if st.button("从备份恢复"):
        load_config_from_file()
        recalc_paths()
        st.success("配置已恢复")

# ---------- 三大模块 ----------
# 航线规划
if st.session_state.module == "航线规划":
    st.header("🗺️ 航线规划")
    col1, col2 = st.columns(2)
    with col1:
        mode = st.radio("起点终点设置方式", ["手动输入经纬度", "鼠标点击地图"])
        if mode == "手动输入经纬度":
            start_lng = st.number_input("起点经度", value=st.session_state.start_point["lng"], format="%.6f")
            start_lat = st.number_input("起点纬度", value=st.session_state.start_point["lat"], format="%.6f")
            end_lng = st.number_input("终点经度", value=st.session_state.end_point["lng"], format="%.6f")
            end_lat = st.number_input("终点纬度", value=st.session_state.end_point["lat"], format="%.6f")
            if st.button("应用手动坐标"):
                st.session_state.start_point = {"lng": start_lng, "lat": start_lat, "name": "起点"}
                st.session_state.end_point = {"lng": end_lng, "lat": end_lat, "name": "终点"}
                recalc_paths()
        else:
            st.info("暂不支持网页内捕获点击，请使用下方坐标重置或手动输入")
        if st.button("重置默认起点终点"):
            st.session_state.start_point = session_defaults["start_point"]
            st.session_state.end_point = session_defaults["end_point"]
            recalc_paths()
    with col2:
        st.subheader("规划策略")
        st.session_state.path_strategy = st.selectbox("绕行模式", ["best", "left", "right"], format_func=lambda x: {"best":"最佳航线","left":"向左绕行","right":"向右绕行"}[x])
        if st.button("✈️ 重新规划路径"):
            recalc_paths()
            st.success("路径规划完成")
        st.metric("直线距离(km)", f"{math.hypot(st.session_state.start_point['lng']-st.session_state.end_point['lng'], st.session_state.start_point['lat']-st.session_state.end_point['lat'])*111:.2f}")
        st.metric("绕行航点数", len(st.session_state.current_paths.get(st.session_state.path_strategy, [])))
    st.subheader("飞行控制")
    fly_col1, fly_col2 = st.columns(2)
    with fly_col1:
        if st.button("🚁 开始飞行", disabled=st.session_state.flight_simulation):
            if st.session_state.current_paths.get(st.session_state.path_strategy):
                st.session_state.waypoints = st.session_state.current_paths[st.session_state.path_strategy]
                st.session_state.flight_position = dict(st.session_state.start_point)
                st.session_state.current_waypoint_idx = 1
                st.session_state.flight_simulation = True
                st.session_state.flight_start_time = time.time()
                st.session_state.flight_data_log = []
                st.success("仿真飞行开始")
        if st.button("🛑 停止飞行"):
            st.session_state.flight_simulation = False
    # 显示地图
    render_map()
    update_flight()
    if st.session_state.flight_simulation:
        st.info("飞行仿真进行中，切换至飞行监控页面查看详细数据")

# 飞行监控
elif st.session_state.module == "飞行监控":
    st.header("📡 飞行监控")
    update_flight()
    if st.session_state.flight_data_log:
        last = st.session_state.flight_data_log[-1]
        col1, col2, col3, col4 = st.columns(4)
        col1.metric("进度", f"{last['progress']:.1f}%")
        col2.metric("速度", f"{last['speed']:.1f} m/s")
        col3.metric("剩余距离", f"{last['remaining_dist']:.0f}m")
        col4.metric("电压", f"{last['voltage']:.1f}V")
        df_log = pd.DataFrame(st.session_state.flight_data_log)
        st.line_chart(df_log[["progress", "speed"]])
        st.line_chart(df_log[["voltage", "satellites"]])
        if st.button("导出飞行日志CSV"):
            csv = df_log.to_csv(index=False)
            st.download_button("下载日志", csv, f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv")
        # 历史飞行统计
        st.subheader("飞行统计")
        if len(df_log) > 0:
            st.metric("最高速度", f"{df_log['speed'].max():.1f} m/s")
            st.metric("平均速度", f"{df_log['speed'].mean():.1f} m/s")
    else:
        st.info("无飞行数据，请先在航线规划模块开始仿真飞行")
    render_map()

# 障碍物管理
else:
    st.header("🚧 障碍物管理")
    col1, col2 = st.columns([2,1])
    with col2:
        st.subheader("数据统计")
        obstacle_list = st.session_state.obstacles
        total = len(obstacle_list)
        avoid = sum(1 for o in obstacle_list if o.get("height",0) > st.session_state.flight_height)
        st.metric("障碍物总数", total)
        st.metric("需避让数量", avoid)
        st.metric("平均高度", f"{np.mean([o['height'] for o in obstacle_list]) if obstacle_list else 0:.1f}m")
        if st.button("保存配置到文件"):
            save_config_to_file()
        if st.button("清空所有障碍物"):
            st.session_state.obstacles = []
            recalc_paths()
    with col1:
        # 批量管理
        st.subheader("批量管理")
        with st.form("batch_form"):
            new_height = st.number_input("批量设置高度(m)", value=50)
            if st.form_submit_button("批量修改选中障碍物高度"):
                for obs in st.session_state.obstacles:
                    if obs.get("selected", False):
                        obs["height"] = new_height
                recalc_paths()
        # 障碍物列表
        st.subheader("障碍物列表")
        for idx, obs in enumerate(st.session_state.obstacles):
            with st.expander(f"{obs['name']} (高:{obs['height']}m)"):
                obs["selected"] = st.checkbox("选择", obs.get("selected", False), key=f"sel_{idx}")
                new_h = st.number_input("高度", value=obs["height"], key=f"h_{idx}")
                if new_h != obs["height"]:
                    obs["height"] = new_h
                    recalc_paths()
                if st.button("删除", key=f"del_{idx}"):
                    st.session_state.obstacles.pop(idx)
                    recalc_paths()
                    st.rerun()
        # 新增障碍物简易表单
        st.subheader("➕ 新增障碍物")
        with st.form("new_obs"):
            obs_name = st.text_input("名称", f"建筑物_{st.session_state.obstacle_next_id}")
            obs_height = st.number_input("高度(m)", value=30)
            # 模拟多边形绘制（简单矩形点）
            lng_center = (st.session_state.start_point["lng"] + st.session_state.end_point["lng"])/2
            lat_center = (st.session_state.start_point["lat"] + st.session_state.end_point["lat"])/2
            coords = [{"lng": lng_center-0.001, "lat": lat_center-0.001},
                      {"lng": lng_center+0.001, "lat": lat_center-0.001},
                      {"lng": lng_center+0.001, "lat": lat_center+0.001},
                      {"lng": lng_center-0.001, "lat": lat_center+0.001}]
            if st.form_submit_button("添加障碍物"):
                st.session_state.obstacles.append({
                    "id": st.session_state.obstacle_next_id,
                    "name": obs_name,
                    "height": obs_height,
                    "coords": coords,
                    "selected": False
                })
                st.session_state.obstacle_next_id += 1
                recalc_paths()
                st.rerun()
    render_map()

# 自动存储和第一次加载
if os.path.exists(CONFIG_FILE) and not st.session_state.get("loaded_once"):
    load_config_from_file()
    recalc_paths()
    st.session_state.loaded_once = True

# 定时飞行更新
update_flight()
# 如果有飞行模拟则自动重绘
if st.session_state.flight_simulation:
    time.sleep(0.05)
    st.rerun()
