import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
import shutil
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple, Any
import pandas as pd
from dataclasses import dataclass, field


# ==================== 配置常量 ====================
@dataclass
class Config:
    """系统配置类"""
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.749155, 32.233767])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.750110, 32.235460])
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 5
    MAX_BACKUP_FILES: int = 10
    BASE_SPEED_MPS: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    VOLTAGE_VARIATION: float = 0.5
    SAT_RANGE: Tuple[int, int] = (8, 14)
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    PATH_SAMPLE_POINTS: int = 40
    MAX_AVOID_ATTEMPTS: int = 12

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)


# ==================== 坐标转换模块 ====================
class CoordinateConverter:
    """WGS-84 与 GCJ-02 坐标转换器"""
    a = 6378245.0
    ee = 0.00669342162296594323

    @classmethod
    def _transform_lat(cls, lng: float, lat: float) -> float:
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
        return ret

    @classmethod
    def _transform_lng(cls, lng: float, lat: float) -> float:
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 * math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
        return ret

    @classmethod
    def out_of_china(cls, lng: float, lat: float) -> bool:
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

    @classmethod
    def wgs84_to_gcj02(cls, lng: float, lat: float) -> Tuple[float, float]:
        if cls.out_of_china(lng, lat):
            return lng, lat
        dlat = cls._transform_lat(lng - 105.0, lat - 35.0)
        dlng = cls._transform_lng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - cls.ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.a * (1 - cls.ee)) / (magic * sqrtmagic) * math.pi)
        dlng = (dlng * 180.0) / (cls.a / sqrtmagic * math.cos(radlat) * math.pi)
        return lng + dlng, lat + dlat

    @classmethod
    def gcj02_to_wgs84(cls, lng: float, lat: float) -> Tuple[float, float]:
        if cls.out_of_china(lng, lat):
            return lng, lat
        dlat = cls._transform_lat(lng - 105.0, lat - 35.0)
        dlng = cls._transform_lng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - cls.ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.a * (1 - cls.ee)) / (magic * sqrtmagic) * math.pi)
        dlng = (dlng * 180.0) / (cls.a / sqrtmagic * math.cos(radlat) * math.pi)
        return lng * 2 - (lng + dlng), lat * 2 - (lat + dlat)

    @classmethod
    def convert_batch(cls, coords: List[Tuple[float, float]], direction: str = "wgs84_to_gcj02") -> List[Tuple[float, float]]:
        converter = cls.wgs84_to_gcj02 if direction == "wgs84_to_gcj02" else cls.gcj02_to_wgs84
        return [converter(lng, lat) for lng, lat in coords]

    @classmethod
    def calculate_offset(cls, lng: float, lat: float) -> Tuple[float, float]:
        gcj_lng, gcj_lat = cls.wgs84_to_gcj02(lng, lat)
        return gcj_lng - lng, gcj_lat - lat


# ==================== MAVLink 接口规划文档 ====================
class MAVLinkInterfaceSpec:
    """
    MAVLink 接口规划文档
    
    【预留接口说明】
    当前飞行监控模块使用模拟数据展示，已预留 MAVLink 消息解析接口。
    后续可通过替换数据源接入真实的 SITL（Software In The Loop）数据流。
    
    【支持的 MAVLink 消息类型】（规划中）
    1. HEARTBEAT (ID: 0) - 系统心跳状态
    2. SYS_STATUS (ID: 1) - 电池与系统状态
    3. GLOBAL_POSITION_INT (ID: 33) - 全球定位信息
    4. ATTITUDE (ID: 30) - 姿态信息
    5. VFR_HUD (ID: 74) - 飞行状态数据
    6. GPS_RAW_INT (ID: 24) - 原始GPS数据
    7. MISSION_ITEM_REACHED (ID: 46) - 航点到达通知
    8. COMMAND_ACK (ID: 77) - 命令确认
    """
    
    MESSAGE_TYPES = {
        "HEARTBEAT": {"id": 0, "frequency": "1Hz", "description": "系统心跳状态"},
        "SYS_STATUS": {"id": 1, "frequency": "1Hz", "description": "电池与系统状态"},
        "GLOBAL_POSITION_INT": {"id": 33, "frequency": "10Hz", "description": "全球定位信息"},
        "ATTITUDE": {"id": 30, "frequency": "10Hz", "description": "姿态信息"},
        "VFR_HUD": {"id": 74, "frequency": "10Hz", "description": "飞行状态数据"},
        "GPS_RAW_INT": {"id": 24, "frequency": "5Hz", "description": "原始GPS数据"},
        "MISSION_ITEM_REACHED": {"id": 46, "frequency": "事件触发", "description": "航点到达通知"},
        "COMMAND_ACK": {"id": 77, "frequency": "事件触发", "description": "命令确认"},
    }
    
    @classmethod
    def get_message_table(cls) -> pd.DataFrame:
        data = []
        for name, info in cls.MESSAGE_TYPES.items():
            data.append({
                "消息名称": name,
                "ID": info["id"],
                "频率": info["frequency"],
                "描述": info["description"]
            })
        return pd.DataFrame(data)


# ==================== 生成MAVLink报文 ====================
def generate_mavlink_messages(count: int = 20) -> List[Dict]:
    """生成模拟MAVLink报文数据"""
    messages = []
    msg_types = [
        ("HEARTBEAT", "🔵", "系统心跳"),
        ("SYS_STATUS", "🟢", "系统状态"),
        ("GLOBAL_POSITION_INT", "🟠", "位置信息"),
        ("ATTITUDE", "🔴", "姿态信息"),
        ("VFR_HUD", "🟣", "飞行数据"),
        ("GPS_RAW_INT", "🟡", "GPS数据"),
    ]
    
    base_time = datetime.now()
    
    for i in range(count):
        msg_idx = i % len(msg_types)
        msg_name, icon, desc = msg_types[msg_idx]
        
        timestamp = (base_time - timedelta(seconds=(count - i) * 0.5)).strftime("%H:%M:%S")
        
        if msg_name == "HEARTBEAT":
            data = f"system_status=ACTIVE, base_mode=AUTO, custom_mode=0x0000"
        elif msg_name == "SYS_STATUS":
            voltage = 22.2 + random.uniform(-0.5, 0.5)
            battery = random.randint(60, 100)
            data = f"voltage={voltage:.1f}V, battery={battery}%, load=0.8"
        elif msg_name == "GLOBAL_POSITION_INT":
            lat = 32.233767 + random.uniform(-0.0001, 0.0001)
            lon = 118.749155 + random.uniform(-0.0001, 0.0001)
            data = f"lat={lat:.8f}, lon={lon:.8f}, alt=50.0m"
        elif msg_name == "ATTITUDE":
            roll = random.uniform(-5, 5)
            pitch = random.uniform(-3, 3)
            yaw = random.uniform(0, 360)
            data = f"roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°"
        elif msg_name == "VFR_HUD":
            airspeed = random.uniform(5, 15)
            groundspeed = random.uniform(4, 12)
            heading = random.uniform(0, 360)
            data = f"airspeed={airspeed:.1f}m/s, groundspeed={groundspeed:.1f}m/s, heading={heading:.1f}°"
        else:
            satellites = random.randint(8, 14)
            fix_type = random.choice([3, 4, 5])
            data = f"satellites={satellites}, fix_type={fix_type}, hdop=0.8"
        
        messages.append({
            "时间": timestamp,
            "类型": f"{icon} {msg_name}",
            "ID": MAVLinkInterfaceSpec.MESSAGE_TYPES.get(msg_name, {}).get("id", 0),
            "频率": MAVLinkInterfaceSpec.MESSAGE_TYPES.get(msg_name, {}).get("frequency", "-"),
            "数据": data,
            "描述": desc
        })
    
    return messages


# ==================== 通信链路模拟器 ====================
@dataclass
class CommunicationLog:
    timestamp: str
    direction: str
    message: str
    details: str = ""


class CommunicationSimulator:
    def __init__(self):
        self.gcs_ip = "192.168.1.100"
        self.obc_ip = "192.168.1.101"
        self.fcu_ip = "192.168.1.102"
        self.gcs_online = True
        self.obc_online = True
        self.fcu_online = True
        self.gcs_obc_latency = 25
        self.obc_fcu_latency = 15
        self.packet_loss_rate = 0.001
        self.logs: List[CommunicationLog] = []
        self.total_packets_sent = 0
        self.total_packets_received = 0
        self.total_packets_lost = 0
        self.planning_records: List[Dict] = []

    def send_message(self, src: str, dst: str, message: str, details: str = "") -> bool:
        self.total_packets_sent += 1
        if not self.check_link_status(src, dst):
            self.total_packets_lost += 1
            return False
        if random.random() < self.packet_loss_rate:
            self.total_packets_lost += 1
            return False
        delay = self.get_link_delay(src, dst)
        time.sleep(delay / 1000)
        self.total_packets_received += 1
        log = CommunicationLog(datetime.now().strftime("%H:%M:%S"), f"{src}→{dst}", message, details)
        self.logs.insert(0, log)
        if len(self.logs) > 100:
            self.logs.pop()
        return True

    def send_relayed_message(self, src: str, relay: str, dst: str, message: str, details: str = "") -> bool:
        return self.send_message(src, relay, message, details) and self.send_message(relay, dst, message, details)

    def check_link_status(self, src: str, dst: str) -> bool:
        if src == "GCS" and dst == "OBC":
            return self.gcs_online and self.obc_online
        elif src == "OBC" and dst == "GCS":
            return self.obc_online and self.gcs_online
        elif src == "OBC" and dst == "FCU":
            return self.obc_online and self.fcu_online
        elif src == "FCU" and dst == "OBC":
            return self.fcu_online and self.obc_online
        return False

    def get_link_delay(self, src: str, dst: str) -> float:
        if (src == "GCS" and dst == "OBC") or (src == "OBC" and dst == "GCS"):
            return self.gcs_obc_latency
        elif (src == "OBC" and dst == "FCU") or (src == "FCU" and dst == "OBC"):
            return self.obc_fcu_latency
        return 10

    def get_statistics(self) -> Dict:
        success_rate = (self.total_packets_received / self.total_packets_sent * 100) if self.total_packets_sent > 0 else 0
        return {
            "sent": self.total_packets_sent,
            "received": self.total_packets_received,
            "lost": self.total_packets_lost,
            "success_rate": success_rate,
            "gcs_obc_latency": self.gcs_obc_latency,
            "obc_fcu_latency": self.obc_fcu_latency,
            "packet_loss_rate": self.packet_loss_rate
        }

    def reset_statistics(self):
        self.total_packets_sent = self.total_packets_received = self.total_packets_lost = 0
        self.logs.clear()
        self.planning_records.clear()

    def add_planning_record(self, record: Dict):
        record["timestamp"] = datetime.now().strftime("%H:%M:%S")
        self.planning_records.insert(0, record)
        if len(self.planning_records) > 20:
            self.planning_records.pop()


# ==================== 几何函数 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside


def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))


def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-10:
        return 0
    return 1 if val > 0 else 2


def segments_intersect(p1: List[float], p2: List[float], p3: List[float], p4: List[float]) -> bool:
    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and on_segment(p1, p3, p2):
        return True
    if o2 == 0 and on_segment(p1, p4, p2):
        return True
    if o3 == 0 and on_segment(p3, p1, p4):
        return True
    if o4 == 0 and on_segment(p3, p2, p4):
        return True
    return False


def line_intersects_polygon(p1: List[float], p2: List[float], polygon: List[List[float]]) -> bool:
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False


def distance(p1: List[float], p2: List[float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def get_polygon_bounds(polygon: List[List[float]]) -> Optional[Dict]:
    if not polygon:
        return None
    lngs = [p[0] for p in polygon]
    lats = [p[1] for p in polygon]
    return {
        'min_lng': min(lngs), 'max_lng': max(lngs),
        'min_lat': min(lats), 'max_lat': max(lats),
        'center_lng': (min(lngs) + max(lngs)) / 2,
        'center_lat': (min(lats) + max(lats)) / 2
    }


def validate_polygon(polygon: List[List[float]]) -> bool:
    return len(polygon) >= 3


def point_to_segment_distance_deg(point: List[float], seg_start: List[float], seg_end: List[float]) -> float:
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0, min(1, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def point_to_segment_distance_meters(point: List[float], seg_start: List[float], seg_end: List[float]) -> float:
    return point_to_segment_distance_deg(point, seg_start, seg_end) * 111000


def check_safety_radius(drone_pos: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float) -> Tuple[bool, Optional[float], Optional[str]]:
    if not drone_pos:
        return True, None, None
    min_distance = float('inf')
    danger_name = None
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        obs_height = obs.get('height', 30)
        if obs_height <= flight_altitude:
            continue
        if coords and len(coords) >= 3:
            for i in range(len(coords)):
                p1 = coords[i]
                p2 = coords[(i + 1) % len(coords)]
                dist_m = point_to_segment_distance_meters(drone_pos, p1, p2)
                if dist_m < min_distance:
                    min_distance = dist_m
                    danger_name = obs.get('name', '障碍物')
    if min_distance < safety_radius:
        return False, min_distance, danger_name
    return True, min_distance if min_distance != float('inf') else None, None


# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    try:
        backup_files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE)]
        if len(backup_files) > config.MAX_BACKUP_FILES:
            backup_files.sort()
            for old_file in backup_files[:-config.MAX_BACKUP_FILES]:
                os.remove(os.path.join(config.BACKUP_DIR, old_file))
    except Exception as e:
        st.warning(f"清理备份文件时出错: {e}")


def backup_config() -> Optional[str]:
    if os.path.exists(config.CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{timestamp}.bak"
        try:
            shutil.copy(config.CONFIG_FILE, backup_name)
            cleanup_old_backups()
            return backup_name
        except Exception as e:
            st.error(f"备份失败: {e}")
            return None
    return None


def load_obstacles() -> List[Dict]:
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles = data.get('obstacles', [])
                for obs in obstacles:
                    if 'selected' not in obs:
                        obs['selected'] = False
                    if 'height' not in obs:
                        obs['height'] = 30
                return obstacles
        except (json.JSONDecodeError, IOError) as e:
            st.error(f"加载配置文件失败: {e}")
            return []
    return []


def save_obstacles(obstacles: List[Dict]) -> bool:
    try:
        backup_config()
        data = {
            'obstacles': obstacles,
            'count': len(obstacles),
            'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'version': 'v13.2'
        }
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        st.error(f"保存失败: {e}")
        return False


def get_latest_backup() -> Optional[str]:
    try:
        backup_files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')]
        if backup_files:
            backup_files.sort(reverse=True)
            return os.path.join(config.BACKUP_DIR, backup_files[0])
    except Exception as e:
        st.error(f"获取备份文件失败: {e}")
    return None


def restore_from_backup(backup_path: str) -> bool:
    try:
        with open(backup_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            obstacles = data.get('obstacles', [])
            save_obstacles(obstacles)
            return True
    except Exception as e:
        st.error(f"恢复备份失败: {e}")
        return False


# ==================== 绕行算法 ====================
@dataclass
class ObstacleInfo:
    polygon: List[List[float]]
    name: str
    height: float
    center: List[float]
    bounding_box: Tuple[float, float, float, float]
    min_lat: float
    max_lat: float
    min_lng: float
    max_lng: float


def get_blocking_obstacles(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    blocking = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking.append(obs)
    return blocking


def get_obstacle_bounds(obstacle: Dict) -> Tuple[float, float, float, float]:
    poly = obstacle.get('polygon', [])
    if not poly:
        return 0, 0, 0, 0
    lngs = [p[0] for p in poly]
    lats = [p[1] for p in poly]
    return min(lngs), max(lngs), min(lats), max(lats)


def get_obstacle_info(obstacle: Dict) -> ObstacleInfo:
    min_lng, max_lng, min_lat, max_lat = get_obstacle_bounds(obstacle)
    return ObstacleInfo(
        polygon=obstacle.get('polygon', []),
        name=obstacle.get('name', '障碍物'),
        height=obstacle.get('height', 30),
        center=[(min_lng + max_lng) / 2, (min_lat + max_lat) / 2],
        bounding_box=(min_lng, max_lng, min_lat, max_lat),
        min_lat=min_lat,
        max_lat=max_lat,
        min_lng=min_lng,
        max_lng=max_lng
    )


def is_path_segment_clear(p1: List[float], p2: List[float], obstacles: List[Dict],
                          flight_altitude: float, safety_radius: float) -> bool:
    for obs in obstacles:
        if obs.get('height', 30) <= flight_altitude:
            continue
        poly = obs.get('polygon', [])
        if not poly:
            continue

        if line_intersects_polygon(p1, p2, poly):
            return False

        sample_count = max(20, int(distance(p1, p2) * 111000 / 3))
        for k in range(sample_count + 1):
            t = k / sample_count
            px = p1[0] + (p2[0] - p1[0]) * t
            py = p1[1] + (p2[1] - p1[1]) * t
            point = [px, py]

            if point_in_polygon(point, poly):
                return False

            for i in range(len(poly)):
                p3 = poly[i]
                p4 = poly[(i + 1) % len(poly)]
                dist_m = point_to_segment_distance_meters(point, p3, p4)
                if dist_m < safety_radius:
                    return False
    return True


def generate_adaptive_waypoints(start: List[float], end: List[float],
                                 obstacles: List[Dict], flight_altitude: float,
                                 safety_radius: float, side: str) -> List[List[float]]:
    if not obstacles:
        return [start, end]

    obstacle_infos = [get_obstacle_info(obs) for obs in obstacles]

    min_lng = min(obs.min_lng for obs in obstacle_infos)
    max_lng = max(obs.max_lng for obs in obstacle_infos)
    min_lat = min(obs.min_lat for obs in obstacle_infos)
    max_lat = max(obs.max_lat for obs in obstacle_infos)

    mid_lat = (start[1] + end[1]) / 2
    deg_per_meter_lng = 1 / (111000 * math.cos(math.radians(mid_lat)))
    deg_per_meter_lat = 1 / 111000

    safe_offset_m = safety_radius + 0.5
    safe_offset_lng = safe_offset_m * deg_per_meter_lng
    safe_offset_lat = safe_offset_m * deg_per_meter_lat

    if side == "right":
        boundary_lng = max_lng + safe_offset_lng
    else:
        boundary_lng = min_lng - safe_offset_lng

    waypoint_lats = []

    if start[1] < min_lat:
        waypoint_lats.append(start[1])
        waypoint_lats.append(start[1] + (min_lat - start[1]) * 0.3)
        waypoint_lats.append(start[1] + (min_lat - start[1]) * 0.6)

    all_boundary_lats = []
    for obs in obstacle_infos:
        all_boundary_lats.append(obs.min_lat - safe_offset_lat * 0.5)
        all_boundary_lats.append(obs.min_lat)
        all_boundary_lats.append(obs.max_lat)
        all_boundary_lats.append(obs.max_lat + safe_offset_lat * 0.5)
        all_boundary_lats.append(obs.center[1])

    waypoint_lats.extend(sorted(set(all_boundary_lats)))

    if end[1] > max_lat:
        waypoint_lats.append(end[1] - (end[1] - max_lat) * 0.4)
        waypoint_lats.append(end[1] - (end[1] - max_lat) * 0.7)
        waypoint_lats.append(end[1])
    else:
        waypoint_lats.append(end[1])

    waypoint_lats = sorted(set(waypoint_lats))

    min_valid_lat = min(start[1], end[1]) - safe_offset_lat
    max_valid_lat = max(start[1], end[1]) + safe_offset_lat
    waypoint_lats = [lat for lat in waypoint_lats if min_valid_lat <= lat <= max_valid_lat]

    if len(waypoint_lats) < 5:
        for i in range(1, 6):
            t = i / 6
            lat = start[1] + (end[1] - start[1]) * t
            waypoint_lats.append(lat)
        waypoint_lats = sorted(set(waypoint_lats))

    best_path = None
    best_score = float('inf')

    for factor_idx, factor in enumerate([0.8, 1.0, 1.2, 1.5, 2.0]):
        current_offset_m = safe_offset_m * factor
        current_offset_lng = current_offset_m * deg_per_meter_lng

        waypoints = []
        for lat in waypoint_lats:
            adjustment = 0
            for obs in obstacle_infos:
                if obs.min_lat - safe_offset_lat <= lat <= obs.max_lat + safe_offset_lat:
                    dist_to_center = abs(lat - obs.center[1])
                    if dist_to_center < safe_offset_lat * 2:
                        extra = (1 - dist_to_center / (safe_offset_lat * 2)) * 0.3
                        adjustment = max(adjustment, extra)

            final_lng = boundary_lng + (current_offset_lng * (1 + adjustment)) if side == "right" else boundary_lng - (current_offset_lng * (1 + adjustment))
            waypoints.append([final_lng, lat])

        candidate = [start] + waypoints + [end]

        is_valid = True
        for i in range(len(candidate) - 1):
            if not is_path_segment_clear(candidate[i], candidate[i+1], obstacles, flight_altitude, safety_radius):
                is_valid = False
                break

        if is_valid:
            path_len = sum(distance(candidate[i], candidate[i+1]) for i in range(len(candidate)-1))
            score = path_len * 111000 + factor_idx * 10
            if score < best_score:
                best_score = score
                best_path = candidate
                break

    if not best_path:
        large_offset_m = safe_offset_m * 3
        large_offset_lng = large_offset_m * deg_per_meter_lng
        waypoints = []
        step = max(1, len(waypoint_lats) // 8)
        for i in range(0, len(waypoint_lats), step):
            lat = waypoint_lats[i]
            waypoint_lng = boundary_lng + large_offset_lng if side == "right" else boundary_lng - large_offset_lng
            waypoints.append([waypoint_lng, lat])
        best_path = [start] + waypoints + [end]

    optimized = [best_path[0]]
    i = 0
    while i < len(best_path) - 1:
        furthest = i + 1
        for j in range(i + 2, len(best_path)):
            if is_path_segment_clear(best_path[i], best_path[j], obstacles, flight_altitude, safety_radius):
                furthest = j
            else:
                break
        optimized.append(best_path[furthest])
        i = furthest

    return optimized


def find_left_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict],
                              flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking:
        return [start, end]
    return generate_adaptive_waypoints(start, end, blocking, flight_altitude, safety_radius, "left")


def find_right_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict],
                               flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking:
        return [start, end]
    return generate_adaptive_waypoints(start, end, blocking, flight_altitude, safety_radius, "right")


def find_best_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict],
                              flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    if is_path_segment_clear(start, end, obstacles_gcj, flight_altitude, safety_radius):
        return [start, end]

    left_path = find_left_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    right_path = find_right_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

    left_len = sum(distance(left_path[i], left_path[i+1]) for i in range(len(left_path)-1))
    right_len = sum(distance(right_path[i], right_path[i+1]) for i in range(len(right_path)-1))

    return left_path if left_len <= right_len else right_path


def create_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict],
                          flight_altitude: float, direction: str, safety_radius: float = 5) -> Optional[List[List[float]]]:
    if not start or not end:
        return None

    straight_safe = True
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                straight_safe = False
                break

    if straight_safe:
        return [start, end]

    if direction == "向左绕行":
        result = find_left_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        result = find_right_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    else:
        result = find_best_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

    if not result or len(result) < 2:
        return [start, end]
    return result


def calculate_path_length(path: List[List[float]]) -> float:
    if not path or len(path) < 2:
        return 0.0
    return sum(distance(path[i], path[i + 1]) for i in range(len(path) - 1))


# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp: str
    flight_time: float
    lat: float
    lng: float
    altitude: float
    voltage: float
    satellites: int
    speed: float
    progress: float
    arrived: bool
    safety_violation: bool
    remaining_distance: float


class HeartbeatSimulator:
    def __init__(self, start_point_gcj: List[float]):
        self.history: List[HeartbeatData] = []
        self.current_pos: List[float] = start_point_gcj.copy()
        self.path: List[List[float]] = [start_point_gcj.copy()]
        self.path_index: int = 0
        self.simulating: bool = False
        self.flight_altitude: float = 50
        self.speed: int = 50
        self.progress: float = 0.0
        self.total_distance: float = 0.0
        self.distance_traveled: float = 0.0
        self.safety_radius: float = config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation: bool = False
        self.start_time: Optional[datetime] = None
        self.flight_log: List[HeartbeatData] = []
        self.last_update_time: Optional[float] = None

    def set_path(self, path: List[List[float]], altitude: float = 50, speed: int = 50, safety_radius: float = 5):
        if not path or len(path) < 2:
            return
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update_time = None
        self.total_distance = sum(distance(path[i], path[i + 1]) for i in range(len(path) - 1))

    def update_and_generate(self, obstacles_gcj: List[Dict], comm_sim: Optional[Any] = None) -> Optional[HeartbeatData]:
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
                if comm_sim:
                    comm_sim.send_relayed_message("FCU", "OBC", "GCS", "MISSION_COMPLETE", "任务完成")
            return None

        current_time = time.time()
        if self.last_update_time is None:
            delta_time = config.HEARTBEAT_INTERVAL
        else:
            delta_time = min(0.5, current_time - self.last_update_time)
        self.last_update_time = current_time

        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = distance(start, end)

        if segment_distance < 1e-9:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index >= len(self.path) - 1:
                self.simulating = False
                return self._generate_heartbeat(True)
            return self._generate_heartbeat(False)

        speed_m_per_s = config.BASE_SPEED_MPS * (self.speed / 100)
        move_distance = speed_m_per_s * delta_time
        self.distance_traveled += move_distance

        if self.distance_traveled < 0:
            self.distance_traveled = 0

        if self.total_distance > 0:
            completed_distance = 0.0
            for i in range(self.path_index):
                completed_distance += distance(self.path[i], self.path[i + 1])
            segment_progress = min(1.0, max(0.0, self.distance_traveled / segment_distance))
            completed_distance += segment_distance * segment_progress
            self.progress = min(1.0, completed_distance / self.total_distance)

        if self.distance_traveled >= segment_distance - 1e-9 and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                if comm_sim:
                    comm_sim.send_relayed_message("FCU", "OBC", "GCS", "MISSION_COMPLETE", "所有航点已完成")
                return self._generate_heartbeat(True)
        elif segment_distance > 0:
            t = min(1.0, max(0.0, self.distance_traveled / segment_distance))
            lng = start[0] + (end[0] - start[0]) * t
            lat = start[1] + (end[1] - start[1]) * t
            self.current_pos = [lng, lat]

        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        if not safe and not self.safety_violation:
            self.safety_violation = True
            if comm_sim:
                comm_sim.send_relayed_message("FCU", "OBC", "GCS", "SAFETY_VIOLATION", "警告：进入危险区域")

        return self._generate_heartbeat(False)

    def _generate_heartbeat(self, arrived: bool = False) -> HeartbeatData:
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0

        if arrived:
            remaining_dist = 0.0
        else:
            remaining_in_path = 0.0
            if self.path_index < len(self.path) - 1:
                segment_remaining = distance(self.current_pos, self.path[self.path_index + 1])
                remaining_in_path += max(0, segment_remaining)
                for i in range(self.path_index + 1, len(self.path) - 1):
                    remaining_in_path += distance(self.path[i], self.path[i + 1])
            remaining_dist = remaining_in_path * 111000

        heartbeat = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=flight_time,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-config.VOLTAGE_VARIATION, config.VOLTAGE_VARIATION), 1),
            satellites=random.randint(*config.SAT_RANGE),
            speed=round(config.BASE_SPEED_MPS * (self.speed / 100), 1),
            progress=self.progress,
            arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=remaining_dist
        )
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        self.flight_log.append(heartbeat)
        if len(self.flight_log) > 1000:
            self.flight_log.pop(0)
        return heartbeat

    def export_flight_data(self) -> pd.DataFrame:
        if not self.flight_log:
            return pd.DataFrame()
        data = [{
            'timestamp': h.timestamp,
            'flight_time': h.flight_time,
            'lat': h.lat,
            'lng': h.lng,
            'altitude': h.altitude,
            'voltage': h.voltage,
            'satellites': h.satellites,
            'speed': h.speed,
            'progress': h.progress,
            'arrived': h.arrived,
            'safety_violation': h.safety_violation,
            'remaining_distance': h.remaining_distance
        } for h in self.flight_log]
        return pd.DataFrame(data)


# ==================== 地图创建 ====================
def create_planning_map(center_gcj: List[float], points_gcj: Dict, obstacles_gcj: List[Dict],
                        flight_history: Optional[List] = None, planned_path: Optional[List] = None,
                        straight_blocked: bool = True, flight_altitude: float = 50,
                        drone_pos: Optional[List] = None, direction: str = "最佳航线",
                        safety_radius: float = 5) -> folium.Map:
    tiles = config.GAODE_SATELLITE_URL
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr="高德卫星地图")

    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={
            'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000',
                        'fillColor': '#ff0000', 'fillOpacity': 0.4},
            'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)

    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True,
                          fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)

    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点",
                     icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
        folium.Circle(
            radius=safety_radius,
            location=[points_gcj['A'][1], points_gcj['A'][0]],
            color="green", weight=2, fill=True,
            fill_color="green", fill_opacity=0.15,
            popup=f"🛡️ 起点安全半径: {safety_radius}米"
        ).add_to(m)

    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点",
                     icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
        folium.Circle(
            radius=safety_radius,
            location=[points_gcj['B'][1], points_gcj['B'][0]],
            color="red", weight=2, fill=True,
            fill_color="red", fill_opacity=0.15,
            popup=f"🛡️ 终点安全半径: {safety_radius}米"
        ).add_to(m)

    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        line_color = "purple" if "向左" in direction else "orange" if "向右" in direction else "green"
        folium.PolyLine(path_locations, color=line_color, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=5, color=line_color, fill=True,
                               fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)

    if points_gcj.get('A') and points_gcj.get('B'):
        line = [[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]]
        if straight_blocked:
            folium.PolyLine(line, color="gray", weight=2, opacity=0.4, dash_array='5,5', popup="⚠️ 直线被阻挡").add_to(m)
        else:
            folium.PolyLine(line, color="blue", weight=2, opacity=0.5, dash_array='5,5', popup="直线航线").add_to(m)

    pos = drone_pos if drone_pos else points_gcj.get('A')
    if pos:
        folium.Circle(radius=safety_radius, location=[pos[1], pos[0]], color="blue", weight=2, fill=True,
                     fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)

    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)

    return m


# ==================== MAVLink 接口规划页面组件 ====================
def render_mavlink_interface_plan():
    """渲染 MAVLink 接口规划文档"""
    st.markdown("### 📡 MAVLink 接口规划文档")

    st.info("""
    **📌 预留接口说明**

    当前飞行监控模块使用**模拟数据**展示，已预留 MAVLink 消息解析接口。
    后续可通过替换数据源接入真实的 SITL（Software In The Loop）数据流或真实无人机数据。
    """)

    col1, col2, col3 = st.columns(3)
    with col1:
        st.markdown("""
        <div style="background: #e8f5e9; border-radius: 10px; padding: 15px; text-align: center; border: 1px solid #4caf50;">
            <h4>📊 当前数据源</h4>
            <p style="font-size: 20px; font-weight: bold; color: #2e7d32;">模拟数据</p>
            <p style="font-size: 12px; color: #666;">Simulation Mode</p>
        </div>
        """, unsafe_allow_html=True)
    with col2:
        st.markdown("""
        <div style="background: #fff3e0; border-radius: 10px; padding: 15px; text-align: center; border: 1px solid #ff9800;">
            <h4>🔌 接口状态</h4>
            <p style="font-size: 20px; font-weight: bold; color: #e65100;">✅ 已预留</p>
            <p style="font-size: 12px; color: #666;">MAVLink Parser 接口</p>
        </div>
        """, unsafe_allow_html=True)
    with col3:
        st.markdown("""
        <div style="background: #e3f2fd; border-radius: 10px; padding: 15px; text-align: center; border: 1px solid #2196f3;">
            <h4>📋 消息类型</h4>
            <p style="font-size: 20px; font-weight: bold; color: #0d47a1;">8 种</p>
            <p style="font-size: 12px; color: #666;">已定义 MAVLink 消息</p>
        </div>
        """, unsafe_allow_html=True)

    st.markdown("---")

    st.markdown("#### 📋 支持的 MAVLink 消息类型")
    df = MAVLinkInterfaceSpec.get_message_table()
    st.dataframe(df, use_container_width=True, hide_index=True)

    st.markdown("#### 🔄 数据流架构")
    st.code("""
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │                         MAVLink 数据流架构                                 │
    ├─────────────────────────────────────────────────────────────────────────────┤
    │                                                                             │
    │   SITL/PX4 ──(UDP 14550)──▶ MAVLink Parser ──▶ 数据处理层 ──▶ 前端展示     │
    │       │                          │                    │                     │
    │       │                          │                    │                     │
    │   ──▶ HEARTBEAT ──────────────▶ 系统状态更新 ────▶ 状态指示器              │
    │   ──▶ SYS_STATUS ─────────────▶ 电量/电压更新 ────▶ 电量仪表盘             │
    │   ──▶ GLOBAL_POSITION_INT ────▶ 位置更新 ────────▶ 地图标记/轨迹           │
    │   ──▶ ATTITUDE ───────────────▶ 姿态更新 ────────▶ 姿态指示器              │
    │   ──▶ VFR_HUD ────────────────▶ 飞行数据更新 ────▶ 仪表盘显示              │
    │   ──▶ GPS_RAW_INT ────────────▶ GPS数据更新 ────▶ 卫星数量显示             │
    │   ──▶ MISSION_ITEM_REACHED ───▶ 航点更新 ────────▶ 航点进度条              │
    │                                                                             │
    └─────────────────────────────────────────────────────────────────────────────┘
    """, language="text")

    st.markdown("#### 🔧 接口实现规划")
    impl_tab1, impl_tab2 = st.tabs(["📝 待实现方法", "📊 数据转换映射"])

    with impl_tab1:
        st.markdown("""
        | 方法签名 | 功能描述 | 状态 |
        |---------|---------|------|
        | `mavlink_connect(connection_string: str) -> bool` | 连接 MAVLink 数据源 | ⏳ 待实现 |
        | `mavlink_disconnect() -> bool` | 断开 MAVLink 连接 | ⏳ 待实现 |
        | `mavlink_receive_message() -> Optional[MAVLinkMessage]` | 接收 MAVLink 消息 | ⏳ 待实现 |
        | `mavlink_parse_heartbeat(msg) -> HeartbeatData` | 解析 HEARTBEAT 消息 | ⏳ 待实现 |
        | `mavlink_parse_position(msg) -> PositionData` | 解析 GLOBAL_POSITION_INT | ⏳ 待实现 |
        | `mavlink_parse_status(msg) -> StatusData` | 解析 SYS_STATUS | ⏳ 待实现 |
        | `mavlink_parse_attitude(msg) -> AttitudeData` | 解析 ATTITUDE | ⏳ 待实现 |
        | `mavlink_parse_vfr_hud(msg) -> VFRHUDData` | 解析 VFR_HUD | ⏳ 待实现 |
        """)

    with impl_tab2:
        st.markdown("""
        | MAVLink 字段 | 前端显示 | 更新频率 |
        |-------------|---------|---------|
        | `HEARTBEAT.system_status` | 系统状态指示器 | 1Hz |
        | `SYS_STATUS.battery_remaining` | 电量百分比 | 1Hz |
        | `GLOBAL_POSITION_INT.lat/lon` | 地图位置标记 | 10Hz |
        | `GLOBAL_POSITION_INT.alt` | 高度显示 | 10Hz |
        | `ATTITUDE.roll/pitch/yaw` | 姿态指示器 | 10Hz |
        | `VFR_HUD.airspeed/groundspeed` | 速度显示 | 10Hz |
        | `GPS_RAW_INT.satellites_visible` | 卫星数量 | 5Hz |
        | `MISSION_ITEM_REACHED.seq` | 航点进度 | 事件触发 |
        """)

    st.markdown("#### ⚙️ 接入配置")
    config_col1, config_col2 = st.columns(2)
    with config_col1:
        st.markdown("""
        **🔌 连接方式**
        - UDP Socket: `udp://127.0.0.1:14550`
        - TCP Socket: `tcp://127.0.0.1:5760`
        - 串口: `/dev/ttyUSB0`
        """)
    with config_col2:
        st.markdown("""
        **🎯 支持飞控**
        - PX4 (Autopilot)
        - ArduPilot (Mission Planner)
        - 其他 MAVLink 兼容飞控
        """)


# ==================== 辅助UI函数 ====================
def init_session_state():
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(),
        'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'comm_sim': CommunicationSimulator(),
        'last_hb_time': time.time(),
        'simulation_running': False,
        'flight_history': [],
        'planned_path': None,
        'last_flight_altitude': 50,
        'pending_obstacle': None,
        'current_direction': "最佳航线",
        'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True,
        'show_rename_dialog': False,
        'waiting_for_start_point': False,
        'waiting_for_end_point': False,
        'temp_click_point': None,
        'conv_result': None,
        'batch_result': None,
        'offset_result': None,
        'mavlink_connected': False,
        'mavlink_data_source': 'simulation',
        'mavlink_connection_string': 'udp://127.0.0.1:14550',
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value
    for obs in st.session_state.obstacles_gcj:
        if 'height' not in obs:
            obs['height'] = 30
        if 'selected' not in obs:
            obs['selected'] = False


def check_straight_blocked(points_gcj: Dict, obstacles_gcj: List[Dict], flight_altitude: float) -> Tuple[bool, int]:
    blocked = False
    high_count = 0
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            high_count += 1
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(points_gcj['A'], points_gcj['B'], coords):
                blocked = True
    return blocked, high_count


def render_sidebar() -> Tuple[str, int, float, bool]:
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🔗 通信拓扑", "🚧 障碍物管理"])
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", 0, 100, 50, 5)
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", 0, 120, 30, 5)
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    new_safety_radius = st.sidebar.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
    if new_safety_radius != st.session_state.safety_radius:
        st.session_state.safety_radius = new_safety_radius
        st.session_state.heartbeat_sim.safety_radius = new_safety_radius
        if st.session_state.planned_path and st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
                st.session_state.current_direction, new_safety_radius)
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", st.session_state.auto_backup)
    return page, drone_speed, flight_alt, auto_save


# ==================== 通信拓扑页面 ====================
def render_communication_page():
    st.header("🔗 通信链路拓扑与数据流")
    comm = st.session_state.comm_sim

    # ========== 三层结构ASCII拓扑图 ==========
    st.markdown("### 🏗️ 三层通信拓扑结构")

    ascii_topology = """
┌─────────────────────────────────────────────────────────────────────────────┐
│                          三层通信链路拓扑                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                      第1层: GCS (地面控制站)                        │   │
│   │   ┌─────────────────────┐      ┌─────────────────────┐             │   │
│   │   │   QGroundControl    │      │   Streamlit GCS     │             │   │
│   │   │   (桌面地面站)      │      │   (Web前端界面)      │             │   │
│   │   └─────────┬───────────┘      └─────────┬───────────┘             │   │
│   └─────────────┼─────────────────────────────┼─────────────────────────┘   │
│                 │                             │                             │
│                 │     UDP:14550 (MAVLink)     │                             │
│                 │◄────────────────────────────►│                             │
│                 │                             │                             │
│   ┌─────────────┼─────────────────────────────┼─────────────────────────┐   │
│   │             ▼                             ▼                         │   │
│   │              第2层: OBC (机载计算机 - Companion Computer)           │   │
│   │         (Raspberry Pi / NVIDIA Jetson)                              │   │
│   │   功能: MAVLink转发 | 数据融合 | 机载路径规划 | 视觉避障           │   │
│   └───────────────────────────┬──────────────────────────────────────────┘   │
│                               │  MAVLink (串口/UART)                         │
│   ┌───────────────────────────▼──────────────────────────────────────────┐   │
│   │              第3层: FCU (飞控单元 - PX4 飞控固件)                    │   │
│   │         (STM32 / Pixhawk 硬件)                                       │   │
│   │   功能: 姿态估计(EKF) | 位置控制(PID) | 电机混控 | 故障检测        │   │
│   └──────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
"""
    st.code(ascii_topology, language="text")

    st.markdown("---")

    # ==================== MAVLink 数据流与报文显示 ====================
    st.markdown("### 📡 MAVLink 数据流与报文监控")
    st.caption("💡 当前为模拟MAVLink报文展示，已预留真实数据接口，后续可接入SITL或PX4飞控")

    col_status1, col_status2, col_status3, col_status4 = st.columns(4)
    with col_status1:
        st.metric("📊 数据源", "模拟数据 (Simulation)", delta="预留MAVLink接口")
    with col_status2:
        st.metric("📡 连接状态", "🟢 已就绪", delta="等待SITL接入")
    with col_status3:
        st.metric("📦 报文总数", f"{comm.total_packets_received + comm.total_packets_sent:,}")
    with col_status4:
        st.metric("📈 实时速率", f"{random.randint(5, 20)} msg/s")

    st.markdown("---")

    tab1, tab2, tab3 = st.tabs(["📋 实时报文列表", "📊 报文详情解析", "📈 流量统计"])

    with tab1:
        st.markdown("#### 📋 MAVLink 报文流")

        mavlink_messages = generate_mavlink_messages(20)
        df = pd.DataFrame(mavlink_messages)
        st.dataframe(df, use_container_width=True, hide_index=True)

        st.markdown("**🎨 报文类型标识**")
        legend_cols = st.columns(5)
        with legend_cols[0]:
            st.markdown("🔵 HEARTBEAT - 系统心跳")
        with legend_cols[1]:
            st.markdown("🟢 SYS_STATUS - 系统状态")
        with legend_cols[2]:
            st.markdown("🟠 POSITION - 位置信息")
        with legend_cols[3]:
            st.markdown("🔴 ATTITUDE - 姿态信息")
        with legend_cols[4]:
            st.markdown("🟣 VFR_HUD - 飞行数据")

        if st.button("🔄 刷新报文", use_container_width=True):
            st.rerun()

    with tab2:
        st.markdown("#### 📊 报文详情解析")
        st.caption("选择下方报文类型，查看详细解析")

        msg_type = st.selectbox(
            "选择报文类型",
            ["HEARTBEAT", "SYS_STATUS", "GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD", "GPS_RAW_INT"]
        )

        if msg_type == "HEARTBEAT":
            st.markdown("""
            ### ❤️ HEARTBEAT 报文 (ID: 0)

            **📝 报文结构**
