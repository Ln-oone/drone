import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random, time, math, json, os
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
import pandas as pd
from dataclasses import dataclass, field

# ==================== 配置常量 ====================
@dataclass
class Config:
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.748726, 32.233881])
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
    VERTICAL_OFFSET_MULTIPLIER: float = 3.0
    WAYPOINT_OFFSET_FACTOR: float = 10.0

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 通信链路模拟器 ====================
@dataclass
class CommunicationLog:
    timestamp: str; direction: str; message: str; details: str = ""

class CommunicationSimulator:
    def __init__(self):
        self.gcs_ip, self.obc_ip, self.fcu_ip = "192.168.1.100", "192.168.1.101", "192.168.1.102"
        self.gcs_online = self.obc_online = self.fcu_online = True
        self.gcs_obc_latency, self.obc_fcu_latency, self.packet_loss_rate = 25, 15, 0.001
        self.logs, self.planning_records = [], []
        self.total_packets_sent = self.total_packets_received = self.total_packets_lost = 0
    
    def send_message(self, src, dst, message, details=""):
        self.total_packets_sent += 1
        if not self.check_link_status(src, dst) or random.random() < self.packet_loss_rate:
            self.total_packets_lost += 1
            return False
        time.sleep(self.get_link_delay(src, dst) / 1000)
        self.total_packets_received += 1
        self.logs.insert(0, CommunicationLog(datetime.now().strftime("%H:%M:%S"), f"{src}→{dst}", message, details))
        if len(self.logs) > 100: self.logs.pop()
        return True
    
    def send_relayed_message(self, src, relay, dst, message, details=""):
        return self.send_message(src, relay, message, details) and self.send_message(relay, dst, message, details)
    
    def check_link_status(self, src, dst):
        return (src == "GCS" and dst == "OBC" and self.gcs_online and self.obc_online) or \
               (src == "OBC" and dst == "GCS" and self.obc_online and self.gcs_online) or \
               (src == "OBC" and dst == "FCU" and self.obc_online and self.fcu_online) or \
               (src == "FCU" and dst == "OBC" and self.fcu_online and self.obc_online)
    
    def get_link_delay(self, src, dst):
        if (src, dst) in [("GCS","OBC"),("OBC","GCS")]: return self.gcs_obc_latency
        if (src, dst) in [("OBC","FCU"),("FCU","OBC")]: return self.obc_fcu_latency
        return 10
    
    def get_statistics(self):
        sr = (self.total_packets_received / self.total_packets_sent * 100) if self.total_packets_sent > 0 else 0
        return {"sent": self.total_packets_sent, "received": self.total_packets_received, "lost": self.total_packets_lost,
                "success_rate": sr, "gcs_obc_latency": self.gcs_obc_latency, "obc_fcu_latency": self.obc_fcu_latency,
                "packet_loss_rate": self.packet_loss_rate}
    
    def reset_statistics(self):
        self.total_packets_sent = self.total_packets_received = self.total_packets_lost = 0
        self.logs.clear(); self.planning_records.clear()
    
    def add_planning_record(self, record):
        record["timestamp"] = datetime.now().strftime("%H:%M:%S")
        self.planning_records.insert(0, record)
        if len(self.planning_records) > 20: self.planning_records.pop()

# ==================== 几何函数 ====================
def point_in_polygon(point, polygon):
    x, y = point; inside = False
    for i in range(len(polygon)):
        x1, y1 = polygon[i]; x2, y2 = polygon[(i+1)%len(polygon)]
        if ((y1 > y) != (y2 > y)) and (x < (x2-x1)*(y-y1)/(y2-y1)+x1): inside = not inside
    return inside

def on_segment(p, q, r): return (min(p[0],r[0]) <= q[0] <= max(p[0],r[0]) and min(p[1],r[1]) <= q[1] <= max(p[1],r[1]))

def orientation(p, q, r):
    val = (q[1]-p[1])*(r[0]-q[0]) - (q[0]-p[0])*(r[1]-q[1])
    return 0 if abs(val)<1e-10 else (1 if val>0 else 2)

def segments_intersect(p1,p2,p3,p4):
    o1,o2,o3,o4 = orientation(p1,p2,p3), orientation(p1,p2,p4), orientation(p3,p4,p1), orientation(p3,p4,p2)
    if o1!=o2 and o3!=o4: return True
    if o1==0 and on_segment(p1,p3,p2): return True
    if o2==0 and on_segment(p1,p4,p2): return True
    if o3==0 and on_segment(p3,p1,p4): return True
    if o4==0 and on_segment(p3,p2,p4): return True
    return False

def line_intersects_polygon(p1,p2,polygon):
    if point_in_polygon(p1,polygon) or point_in_polygon(p2,polygon): return True
    for i in range(len(polygon)):
        if segments_intersect(p1,p2,polygon[i],polygon[(i+1)%len(polygon)]): return True
    return False

def distance(p1,p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
def get_polygon_bounds(polygon):
    if not polygon: return None
    lngs = [p[0] for p in polygon]; lats = [p[1] for p in polygon]
    return {'min_lng':min(lngs), 'max_lng':max(lngs), 'min_lat':min(lats), 'max_lat':max(lats),
            'center_lng':(min(lngs)+max(lngs))/2, 'center_lat':(min(lats)+max(lats))/2}
def validate_polygon(polygon): return len(polygon) >= 3
def meters_to_deg(meters, lat=32.23): return meters/111000, meters/(111000*math.cos(math.radians(lat)))

def point_to_segment_distance_meters(point, seg_start, seg_end):
    px,py = point; x1,y1 = seg_start; x2,y2 = seg_end
    dx,dy = x2-x1, y2-y1
    if dx*dx+dy*dy == 0: return math.hypot(px-x1, py-y1)
    t = max(0, min(1, ((px-x1)*dx+(py-y1)*dy)/(dx*dx+dy*dy)))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy)) * 111000

def check_safety_radius(drone_pos, obstacles_gcj, flight_altitude, safety_radius):
    if not drone_pos: return True, None, None
    min_dist, danger = float('inf'), None
    for obs in obstacles_gcj:
        if obs.get('height',30) <= flight_altitude: continue
        poly = obs.get('polygon',[])
        for i in range(len(poly)):
            d = point_to_segment_distance_meters(drone_pos, poly[i], poly[(i+1)%len(poly)])
            if d < min_dist: min_dist, danger = d, obs.get('name','障碍物')
    return (False, min_dist, danger) if min_dist < safety_radius else (True, min_dist if min_dist!=float('inf') else None, None)

# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    try:
        files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE)]
        if len(files) > config.MAX_BACKUP_FILES:
            for f in sorted(files)[:-config.MAX_BACKUP_FILES]: os.remove(os.path.join(config.BACKUP_DIR, f))
    except: pass

def backup_config():
    if os.path.exists(config.CONFIG_FILE):
        bn = f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{datetime.now().strftime('%Y%m%d_%H%M%S')}.bak"
        try:
            import shutil; shutil.copy(config.CONFIG_FILE, bn); cleanup_old_backups(); return bn
        except: return None
    return None

def load_obstacles():
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                obs = json.load(f).get('obstacles', [])
                for o in obs:
                    o.setdefault('selected', False)
                    o.setdefault('height', 30)
                return obs
        except: return []
    return []

def save_obstacles(obstacles):
    try:
        backup_config()
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump({'obstacles': obstacles, 'count': len(obstacles), 
                       'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 'version': 'v13.2'}, f, ensure_ascii=False, indent=2)
        return True
    except: return False

def get_latest_backup():
    try:
        files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')]
        return os.path.join(config.BACKUP_DIR, sorted(files)[-1]) if files else None
    except: return None

def restore_from_backup(backup_path):
    try:
        with open(backup_path, 'r', encoding='utf-8') as f:
            return save_obstacles(json.load(f).get('obstacles', []))
    except: return False

# ==================== 绕行算法 ====================
def get_blocking_obstacles(start, end, obstacles_gcj, flight_alt):
    return [obs for obs in obstacles_gcj if obs.get('height',30) > flight_alt and line_intersects_polygon(start, end, obs.get('polygon',[]))]

def get_obstacle_extent(obstacles):
    lngs = [p[0] for obs in obstacles for p in obs.get('polygon',[])]
    lats = [p[1] for obs in obstacles for p in obs.get('polygon',[])]
    return min(lngs), max(lngs), min(lats), max(lats) if lngs else (0,0,0,0)

def is_path_segment_clear(p1, p2, obstacles, flight_alt, safety_radius):
    for obs in obstacles:
        if obs.get('height',30) <= flight_alt: continue
        poly = obs.get('polygon',[])
        if not poly: continue
        if line_intersects_polygon(p1,p2,poly): return False
        for k in range(31):
            t = k/30
            point = [p1[0]+(p2[0]-p1[0])*t, p1[1]+(p2[1]-p1[1])*t]
            if point_in_polygon(point, poly): return False
            for i in range(len(poly)):
                if point_to_segment_distance_meters(point, poly[i], poly[(i+1)%len(poly)]) < safety_radius: return False
    return True

def find_optimal_avoidance_path(start, end, obstacles_gcj, flight_alt, safety_radius=5, side="right"):
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_alt)
    if not blocking_obs: return [start, end]
    
    min_lng, max_lng, min_lat, max_lat = get_obstacle_extent(blocking_obs)
    mid_lat = (start[1]+end[1])/2
    deg_per_meter_lng = 1/(111000*math.cos(math.radians(mid_lat)))
    dx, dy = end[0]-start[0], end[1]-start[1]
    path_len = math.hypot(dx, dy)
    if path_len < 1e-9: return [start, end]
    ux, uy = dx/path_len, dy/path_len
    
    if side == "right":
        perp_x, perp_y, boundary = uy, -ux, max_lng
        base_offset = safety_radius + 2.0
    else:
        perp_x, perp_y, boundary = -uy, ux, min_lng
        base_offset = safety_radius + 1.5
    
    if start[1] < min_lat and end[1] > max_lat:
        t_vals = [0.15, 0.3, 0.5, 0.7, 0.85]
    elif start[1] < min_lat:
        t_vals = [0.2, 0.4, 0.6, 0.8]
    elif end[1] > max_lat:
        t_vals = [0.2, 0.4, 0.6, 0.8]
    else:
        t_vals = [0.2, 0.5, 0.8]
    
    offset = base_offset
    best_path = None
    min_off = float('inf')
    
    for attempt in range(1, 12):
        off_deg = offset * deg_per_meter_lng
        waypoints = []
        for t in t_vals:
            if start[1] < min_lat and end[1] > max_lat:
                lat = start[1] + (end[1]-start[1])*t
            elif start[1] < min_lat:
                lat = start[1] + (min(end[1], min_lat-0.00003)-start[1])*t
            elif end[1] > max_lat:
                lat = start[1] + (max(start[1], max_lat+0.00003)-start[1])*t
            else:
                target = max_lat+0.00005 if side=="right" else min_lat-0.00005
                lat = start[1] + (target-start[1])*t
            t_orig = (lat-start[1])/dy if dy!=0 else t
            t_orig = max(0, min(1, t_orig))
            orig_x = start[0] + dx*t_orig
            waypoints.append([orig_x + perp_x*off_deg, lat])
        
        cand = [start] + waypoints + [end]
        if all(is_path_segment_clear(cand[i], cand[i+1], blocking_obs, flight_alt, safety_radius) for i in range(len(cand)-1)):
            if offset < min_off: min_off, best_path = offset, cand
        offset = base_offset + safety_radius * attempt * 0.6
    
    if best_path: return best_path
    
    # 保底方案
    off_deg = (base_offset + safety_radius*2) * deg_per_meter_lng
    fallback = []
    for t in [0.1,0.25,0.4,0.6,0.75,0.9]:
        lat = start[1] + (end[1]-start[1])*t
        t_orig = (lat-start[1])/dy if dy!=0 else t
        orig_x = start[0] + dx*max(0,min(1,t_orig))
        lng = boundary + off_deg*(0.8+0.2*math.sin(math.pi*t)) if side=="right" else boundary - off_deg*(0.8+0.2*math.sin(math.pi*t))
        fallback.append([lng, lat])
    return [start] + fallback + [end]

def find_left_avoidance_path(s,e,obs,alt,rad=5): return find_optimal_avoidance_path(s,e,obs,alt,rad,"left")
def find_right_avoidance_path(s,e,obs,alt,rad=5): return find_optimal_avoidance_path(s,e,obs,alt,rad,"right")
def find_best_avoidance_path(s,e,obs,alt,rad=5):
    if is_path_segment_clear(s,e,obs,alt,rad): return [s,e]
    left, right = find_left_avoidance_path(s,e,obs,alt,rad), find_right_avoidance_path(s,e,obs,alt,rad)
    return left if sum(distance(left[i],left[i+1]) for i in range(len(left)-1)) <= sum(distance(right[i],right[i+1]) for i in range(len(right)-1)) else right

def create_avoidance_path(s,e,obs,alt,dir,rad=5):
    if all(obs.get('height',30)<=alt or not line_intersects_polygon(s,e,obs.get('polygon',[])) for obs in obs): return [s,e]
    return {"向左绕行":find_left_avoidance_path, "向右绕行":find_right_avoidance_path}.get(dir, find_best_avoidance_path)(s,e,obs,alt,rad)

def calculate_path_length(path): return sum(distance(path[i],path[i+1]) for i in range(len(path)-1))

# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp, flight_time, lat, lng, altitude, voltage, satellites, speed, progress, arrived, safety_violation, remaining_distance

class HeartbeatSimulator:
    def __init__(self, start):
        self.history = []; self.current_pos = start.copy(); self.path = [start.copy()]
        self.path_idx = 0; self.simulating = False; self.flight_alt = 50; self.speed = 50
        self.progress = 0; self.total_dist = 0; self.dist_traveled = 0
        self.safety_radius = config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False; self.start_time = None; self.flight_log = []; self.last_update = None
    
    def set_path(self, path, altitude=50, speed=50, safety_radius=5):
        self.path = path; self.path_idx = 0; self.current_pos = path[0].copy()
        self.flight_alt = altitude; self.speed = speed; self.safety_radius = safety_radius
        self.simulating = True; self.progress = self.dist_traveled = 0
        self.safety_violation = False; self.start_time = datetime.now(); self.last_update = None
        self.total_dist = sum(distance(self.path[i], self.path[i+1]) for i in range(len(path)-1))
    
    def update_and_generate(self, obstacles, comm=None):
        if not self.simulating or self.path_idx >= len(self.path)-1:
            if self.simulating:
                self.simulating = False
                if comm: comm.send_relayed_message("FCU","OBC","GCS","MISSION_COMPLETE","任务完成")
            return None
        
        now = time.time()
        dt = config.HEARTBEAT_INTERVAL if self.last_update is None else min(0.5, now - self.last_update)
        self.last_update = now
        
        start, end = self.path[self.path_idx], self.path[self.path_idx+1]
        seg_dist = distance(start, end)
        move = config.BASE_SPEED_MPS * (self.speed/100) * dt
        self.dist_traveled += max(0, move)
        
        if self.total_dist > 0:
            completed = sum(distance(self.path[i], self.path[i+1]) for i in range(self.path_idx))
            if seg_dist > 0: completed += seg_dist * min(1, max(0, self.dist_traveled/seg_dist))
            self.progress = min(1, completed/self.total_dist)
        
        if self.dist_traveled >= seg_dist and seg_dist > 0:
            if comm and self.path_idx < len(self.path)-1:
                wp = self.path_idx+1
                comm.send_message("FCU","OBC",f"WP_REACHED #{wp}",f"到达航点 {wp}/{len(self.path)-1}")
                comm.send_relayed_message("FCU","OBC","GCS",f"WP_REACHED #{wp}",f"航点 {wp} 已到达")
            self.path_idx += 1; self.dist_traveled = 0
            if self.path_idx < len(self.path): self.current_pos = self.path[self.path_idx].copy()
            else:
                self.simulating = False
                if comm: comm.send_relayed_message("FCU","OBC","GCS","MISSION_COMPLETE","所有航点已完成")
                return self._gen_hb(True)
        else:
            if seg_dist > 0:
                t = min(1, max(0, self.dist_traveled/seg_dist))
                self.current_pos = [start[0]+(end[0]-start[0])*t, start[1]+(end[1]-start[1])*t]
        
        safe,_,_ = check_safety_radius(self.current_pos, obstacles, self.flight_alt, self.safety_radius)
        if not safe:
            self.safety_violation = True
            if comm: comm.send_relayed_message("FCU","OBC","GCS","SAFETY_VIOLATION","警告：进入危险区域")
        return self._gen_hb(False)
    
    def _gen_hb(self, arrived=False):
        ft = (datetime.now()-self.start_time).total_seconds() if self.start_time else 0
        rem = 0 if arrived else sum(distance(self.current_pos, self.path[self.path_idx+1]) if self.path_idx<len(self.path)-1 else 0 for _ in range(1)) * 111000
        if not arrived and self.path_idx < len(self.path)-1:
            rem = distance(self.current_pos, self.path[self.path_idx+1]) * 111000
            for i in range(self.path_idx+1, len(self.path)-1): rem += distance(self.path[i], self.path[i+1]) * 111000
        
        hb = HeartbeatData(datetime.now().strftime("%H:%M:%S"), ft, self.current_pos[1], self.current_pos[0],
                           self.flight_alt, round(22.2+random.uniform(-config.VOLTAGE_VARIATION,config.VOLTAGE_VARIATION),1),
                           random.randint(*config.SAT_RANGE), round(config.BASE_SPEED_MPS*(self.speed/100),1),
                           self.progress, arrived, self.safety_violation, max(0,rem))
        self.history.insert(0, hb)
        if len(self.history) > 100: self.history.pop()
        self.flight_log.append(hb)
        if len(self.flight_log) > 1000: self.flight_log.pop(0)
        return hb
    
    def export_flight_data(self):
        if not self.flight_log: return pd.DataFrame()
        return pd.DataFrame([{'timestamp':h.timestamp, 'flight_time':h.flight_time, 'lat':h.lat, 'lng':h.lng,
                              'altitude':h.altitude, 'voltage':h.voltage, 'satellites':h.satellites, 'speed':h.speed,
                              'progress':h.progress, 'arrived':h.arrived, 'safety_violation':h.safety_violation,
                              'remaining_distance':h.remaining_distance} for h in self.flight_log])

# ==================== 地图创建 ====================
def create_planning_map(center, points, obstacles, flight_history=None, planned_path=None, 
                        straight_blocked=True, flight_alt=50, drone_pos=None, direction="最佳航线", safety_radius=5):
    m = folium.Map(location=[center[1], center[0]], zoom_start=16, tiles=config.GAODE_SATELLITE_URL, attr="高德卫星地图")
    m.add_child(plugins.Draw(export=True, position='topleft', draw_options={'polygon':{'allowIntersection':False,'showArea':True,'color':'#ff0000','fillColor':'#ff0000','fillOpacity':0.4},
                       'polyline':False,'rectangle':False,'circle':False,'marker':False,'circlemarker':False}, edit_options={'edit':True,'remove':True}))
    
    for obs in obstacles:
        coords = obs.get('polygon',[])
        if len(coords)>=3:
            color = "red" if obs.get('height',30) > flight_alt else "orange"
            folium.Polygon([[c[1],c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {obs.get('height',30)}m").add_to(m)
    
    for pt,label,color,icon in [(points.get('A'),"🟢 起点","green","play"), (points.get('B'),"🔴 终点","red","stop")]:
        if pt: folium.Marker([pt[1], pt[0]], popup=label, icon=folium.Icon(color=color, icon=icon, prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path)>1:
        lc = "purple" if "向左" in direction else "orange" if "向右" in direction else "green"
        folium.PolyLine([[p[1],p[0]] for p in planned_path], color=lc, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        for i,p in enumerate(planned_path[1:-1]): folium.CircleMarker([p[1],p[0]], radius=5, color=lc, fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    
    if points.get('A') and points.get('B'):
        line = [[points['A'][1],points['A'][0]],[points['B'][1],points['B'][0]]]
        folium.PolyLine(line, color="gray" if straight_blocked else "blue", weight=2, opacity=0.5 if not straight_blocked else 0.4, dash_array='5,5', popup="⚠️ 直线被阻挡" if straight_blocked else "直线航线").add_to(m)
    
    pos = drone_pos if drone_pos else points.get('A')
    if pos: folium.Circle(radius=safety_radius, location=[pos[1],pos[0]], color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)
    
    if flight_history and len(flight_history)>1:
        trail = [[p[1],p[0]] for p in flight_history if len(p)>=2]
        if len(trail)>1: folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    return m

# ==================== 辅助UI函数 ====================
def init_session_state():
    defaults = {'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
                'obstacles_gcj': load_obstacles(), 'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
                'comm_sim': CommunicationSimulator(), 'last_hb_time': time.time(), 'simulation_running': False,
                'flight_history': [], 'planned_path': None, 'last_flight_altitude': 50, 'pending_obstacle': None,
                'current_direction': "最佳航线", 'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
                'auto_backup': True, 'show_rename_dialog': False, 'waiting_for_start_point': False,
                'waiting_for_end_point': False, 'temp_click_point': None}
    for k,v in defaults.items():
        if k not in st.session_state: st.session_state[k] = v
    for obs in st.session_state.obstacles_gcj:
        obs.setdefault('height', 30)
        obs.setdefault('selected', False)

def check_straight_blocked(points, obstacles, flight_alt):
    blocked = False; high_cnt = 0
    for obs in obstacles:
        if obs.get('height',30) > flight_alt:
            high_cnt += 1
            if line_intersects_polygon(points['A'], points['B'], obs.get('polygon',[])): blocked = True
    return blocked, high_cnt

def render_sidebar():
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🔗 通信拓扑", "🚧 障碍物管理"])
    st.sidebar.markdown("---")
    drone_speed = st.sidebar.slider("飞行速度系数", 10, 100, 50, 5)
    flight_alt = st.sidebar.slider("飞行高度 (m)", 10, 200, 50, 5)
    new_rad = st.sidebar.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
    if new_rad != st.session_state.safety_radius:
        st.session_state.safety_radius = new_rad
        st.session_state.heartbeat_sim.safety_radius = new_rad
        if st.session_state.planned_path and st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
            st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, st.session_state.last_flight_altitude, st.session_state.current_direction, new_rad)
    st.sidebar.markdown("---")
    auto_save = st.sidebar.checkbox("自动保存障碍物", st.session_state.auto_backup)
    return page, drone_speed, flight_alt, auto_save

# ==================== 通信拓扑页面 ====================
def render_communication_page():
    st.header("🔗 通信链路拓扑与数据流")
    comm = st.session_state.comm_sim
    
    col_status1, col_status2, col_status3 = st.columns(3)
    for col, name, ip, device in [(col_status1, "📡 GCS", comm.gcs_ip, "GCS"), (col_status2, "💻 OBC", comm.obc_ip, "OBC"), (col_status3, "🎮 FCU", comm.fcu_ip, "FCU")]:
        with col: st.metric(name, "🟢 在线" if getattr(comm, f"{device.lower()}_online") else "🔴 离线"); st.caption(f"IP: {ip}")
    
    st.markdown("---"); st.subheader("📡 通信链路拓扑")
    col_t1, col_t2, col_t3 = st.columns([1,2,1])
    with col_t1: st.markdown("### 🖥️ GCS\n**地面站**"); st.caption(comm.gcs_ip)
    with col_t2:
        st.markdown("### 🔗 链路状态")
        st.markdown(f"**GCS ↔ OBC**\nUDP:14550 | {'🟢 已连接' if comm.check_link_status('GCS','OBC') else '🔴 断开'}")
        st.caption(f"延迟: {comm.gcs_obc_latency}ms"); st.markdown("↓")
        st.markdown(f"**OBC ↔ FCU**\nMAVLink | {'🟢 已连接' if comm.check_link_status('OBC','FCU') else '🔴 断开'}")
        st.caption(f"延迟: {comm.obc_fcu_latency}ms")
    with col_t3: st.markdown("### 🎮 FCU\n**飞控**"); st.caption(comm.fcu_ip); st.markdown("PX4 / ArduPilot")
    
    st.markdown("---"); st.subheader("📊 链路统计")
    stats = comm.get_statistics()
    cols = st.columns(4) + st.columns(3)
    for col, (k, v) in zip(cols[:4], [("📤 发送包数",stats["sent"]),("📥 接收包数",stats["received"]),("❌ 丢包数",stats["lost"]),("✅ 成功率",f"{stats['success_rate']:.1f}%")]): col.metric(k, v)
    for col, (k, v) in zip(cols[4:], [("⚡ GCS-OBC延迟",f"{stats['gcs_obc_latency']}ms"),("⚡ OBC-FCU延迟",f"{stats['obc_fcu_latency']}ms"),("📉 丢包率",f"{stats['packet_loss_rate']*100:.1f}%")]): col.metric(k, v)
    
    st.markdown("---"); st.subheader("🎮 链路控制")
    col_c = st.columns(4)
    if col_c[0].button("🔄 重置统计", use_container_width=True): comm.reset_statistics(); st.rerun()
    new_gcs = col_c[1].slider("GCS-OBC延迟(ms)",5,100,comm.gcs_obc_latency,5)
    if new_gcs != comm.gcs_obc_latency: comm.gcs_obc_latency = new_gcs
    new_obc = col_c[2].slider("OBC-FCU延迟(ms)",5,100,comm.obc_fcu_latency,5)
    if new_obc != comm.obc_fcu_latency: comm.obc_fcu_latency = new_obc
    new_loss = col_c[3].slider("丢包率(%)",0.0,5.0,comm.packet_loss_rate*100,0.1)/100
    if new_loss != comm.packet_loss_rate: comm.packet_loss_rate = new_loss
    
    st.markdown("---"); st.subheader("📋 通信日志")
    show1 = st.button("📤 GCS → OBC → FCU", use_container_width=True, type="primary")
    show2 = st.button("📥 FCU → OBC → GCS", use_container_width=True, type="secondary")
    st.markdown("---")
    
    if show1 or (not show2 and not show1):
        st.markdown("### 📤 GCS → OBC → FCU"); st.caption("航线规划指令下发流程")
        if comm.planning_records:
            st.markdown("#### 航线规划记录")
            for r in comm.planning_records[:10]:
                st.text(f"[{r.get('timestamp','')}] {r.get('message','')}")
                if r.get('details'): st.caption(f"   {r['details']}")
        else: st.info("暂无航线规划记录")
        for title, direction in [("GCS → OBC","GCS→OBC"), ("OBC → FCU","OBC→FCU")]:
            logs = [l for l in comm.logs if l.direction == direction]
            if logs:
                st.markdown(f"#### {title}")
                for l in logs[:10]: st.text(f"[{l.timestamp}] {l.message}"); st.caption(f"   {l.details}")
    
    if show2:
        st.markdown("### 📥 FCU → OBC → GCS"); st.caption("飞行状态上报流程")
        for title, direction in [("FCU → OBC","FCU→OBC"), ("OBC → GCS","OBC→GCS")]:
            logs = [l for l in comm.logs if l.direction == direction]
            if logs:
                st.markdown(f"#### {title}")
                for l in logs[:20]: st.text(f"[{l.timestamp}] {l.message}"); st.caption(f"   {l.details}")
    
    st.markdown("---")
    if st.button("🗑️ 清空所有日志", use_container_width=True): comm.logs.clear(); comm.planning_records.clear(); st.rerun()

# ==================== 航线规划页面 ====================
def render_planning_page(drone_speed, flight_alt, auto_save):
    st.header("🗺️ 航线规划 - 智能避障")
    blocked, high = check_straight_blocked(st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt)
    if blocked: st.warning(f"⚠️ 有 {high} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
    else: st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
    st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
    
    col1, col2 = st.columns([1, 1.5])
    with col1: render_planning_controls(flight_alt, drone_speed, auto_save)
    with col2: render_planning_map_view(flight_alt, blocked)

def render_planning_controls(flight_alt, drone_speed, auto_save):
    st.subheader("🎮 控制面板")
    with st.expander("📍 起点/终点设置", expanded=True): render_point_settings()
    with st.expander("🤖 路径规划策略", expanded=True): render_path_strategy(flight_alt)
    with st.expander("✈️ 飞行控制", expanded=True): render_flight_controls(flight_alt, drone_speed)
    
    a,b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
    st.markdown("### 📍 当前坐标")
    st.write(f"🟢 A点: ({a[0]:.6f}, {a[1]:.6f})")
    st.write(f"🔴 B点: ({b[0]:.6f}, {b[1]:.6f})")
    st.caption(f"📏 直线距离: {math.hypot(b[0]-a[0], b[1]-a[1])*111000:.0f} 米")
    st.caption(f"🛡️ 当前安全半径: {st.session_state.safety_radius} 米")

def render_point_settings():
    st.markdown("#### 🎯 设置方式选择")
    mode = st.radio("选择设置方式", ["✏️ 经纬度输入", "🖱️ 鼠标点击设置"], horizontal=True, key="point_setting_mode")
    if mode == "✏️ 经纬度输入": render_coordinate_input()
    else: render_mouse_click_setting()

def render_coordinate_input():
    st.markdown("#### 🟢 起点 A")
    c1,c2 = st.columns(2)
    with c1: lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat", step=0.000001)
    with c2: lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng", step=0.000001)
    if st.button("📍 设置 A 点", use_container_width=True):
        st.session_state.points_gcj['A'] = [lng, lat]; update_path_after_point_change(); st.success(f"✅ 起点已设置为 ({lng:.6f}, {lat:.6f})"); st.rerun()
    
    st.markdown("#### 🔴 终点 B")
    c1,c2 = st.columns(2)
    with c1: lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat", step=0.000001)
    with c2: lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng", step=0.000001)
    if st.button("📍 设置 B 点", use_container_width=True):
        st.session_state.points_gcj['B'] = [lng, lat]; update_path_after_point_change(); st.success(f"✅ 终点已设置为 ({lng:.6f}, {lat:.6f})"); st.rerun()

def render_mouse_click_setting():
    st.info("💡 提示：点击地图上的任意位置来设置起点或终点")
    c1,c2 = st.columns(2)
    if c1.button("🎯 设置起点 (点击地图)", use_container_width=True, type="primary"):
        st.session_state.waiting_for_start_point = True; st.session_state.waiting_for_end_point = False; st.info("👉 请在地图上点击选择起点位置"); st.rerun()
    if c2.button("📍 设置终点 (点击地图)", use_container_width=True, type="primary"):
        st.session_state.waiting_for_end_point = True; st.session_state.waiting_for_start_point = False; st.info("👉 请在地图上点击选择终点位置"); st.rerun()
    
    if st.session_state.waiting_for_start_point: st.warning("⏳ 等待设置起点... 请点击地图")
    elif st.session_state.waiting_for_end_point: st.warning("⏳ 等待设置终点... 请点击地图")
    if st.session_state.waiting_for_start_point or st.session_state.waiting_for_end_point:
        if st.button("❌ 取消当前操作", use_container_width=True): st.session_state.waiting_for_start_point = st.session_state.waiting_for_end_point = False; st.session_state.temp_click_point = None; st.rerun()
    
    st.markdown("---"); st.markdown("#### 📍 快速设置")
    c1,c2 = st.columns(2)
    if c1.button("🔄 重置到默认起点", use_container_width=True): st.session_state.points_gcj['A'] = config.DEFAULT_A_GCJ.copy(); update_path_after_point_change(); st.rerun()
    if c2.button("🔄 重置到默认终点", use_container_width=True): st.session_state.points_gcj['B'] = config.DEFAULT_B_GCJ.copy(); update_path_after_point_change(); st.rerun()

def update_path_after_point_change():
    st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, st.session_state.last_flight_altitude, st.session_state.current_direction, st.session_state.safety_radius)

def render_path_strategy(flight_alt):
    st.markdown("**选择绕行方向：**")
    c1,c2,c3 = st.columns(3)
    for btn, dir_name in [(c1,"🔄 最佳航线","最佳航线"),(c2,"⬅️ 向左绕行","向左绕行"),(c3,"➡️ 向右绕行","向右绕行")]:
        if btn.button(btn[0], use_container_width=True, type="primary" if st.session_state.current_direction==dir_name else "secondary"):
            st.session_state.current_direction = dir_name
            st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, dir_name, st.session_state.safety_radius)
            st.success(f"已切换到{dir_name}模式"); st.rerun()
    st.info(f"📌 当前绕行策略: **{st.session_state.current_direction}**")
    st.info(f"🛡️ 当前安全半径: **{st.session_state.safety_radius} 米**")
    if st.button("🔄 重新规划路径", use_container_width=True):
        st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
        if st.session_state.planned_path: st.success(f"已按照「{st.session_state.current_direction}」规划路径，{len(st.session_state.planned_path)-2}个绕行点")
        st.rerun()

def render_flight_controls(flight_alt, drone_speed):
    col1, col2, col3 = st.columns(3)
    col1.metric("当前飞行高度", f"{flight_alt} m")
    col2.metric("速度系数", f"{drone_speed}%")
    col3.metric("🛡️ 安全半径", f"{st.session_state.safety_radius} 米")
    if st.session_state.planned_path:
        st.metric("🎯 绕行点数量", len(st.session_state.planned_path)-2)
        st.caption(f"📏 规划路径总长: {calculate_path_length(st.session_state.planned_path)*111000:.0f} 米")
    c1,c2 = st.columns(2)
    if c1.button("▶️ 开始飞行", use_container_width=True, type="primary"):
        if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
            path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
            comm = st.session_state.comm_sim
            total = calculate_path_length(path)*111000
            comm.add_planning_record({"message":"开始航线规划","details":f"算法: A* | 障碍物数量: {len(st.session_state.obstacles_gcj)}"})
            comm.add_planning_record({"message":"航线规划完成","details":f"类型: horizontal | 航点数: {len(path)} | 路径长度: {total:.1f}m"})
            comm.add_planning_record({"message":"导航目标","details":f"起点: {st.session_state.points_gcj['A']} | 终点: {st.session_state.points_gcj['B']} | 目标高度: {flight_alt}m"})
            comm.send_message("GCS","OBC","START_MISSION",f"起点: {st.session_state.points_gcj['A']}, 终点: {st.session_state.points_gcj['B']}")
            comm.send_message("OBC","FCU","UPLOAD_MISSION",f"航点数量: {len(path)}")
            st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
            st.session_state.simulation_running = True
            st.session_state.flight_history = []
            comm.send_message("FCU","OBC","ACK","Mode: AUTO")
            comm.send_message("OBC","GCS","ACK","任务已开始")
            st.success(f"🚁 飞行已开始！{'路径中有' + str(len(path)-2) + '个绕行点' if len(path)>2 else '直线飞行'}")
            st.rerun()
        else: st.error("请先设置起点和终点")
    if c2.button("⏹️ 停止飞行", use_container_width=True):
        st.session_state.simulation_running = False
        st.session_state.heartbeat_sim.simulating = False
        st.session_state.comm_sim.send_message("GCS","OBC","STOP_MISSION","用户停止飞行")
        st.info("飞行已停止")

def render_planning_map_view(flight_alt, straight_blocked):
    st.subheader("🗺️ 规划地图")
    if straight_blocked: st.caption(f"当前避障策略: {st.session_state.current_direction}")
    st.caption("🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行 | 🔵 蓝色圆圈=安全半径")
    st.caption("💡 提示：在鼠标点击设置模式下，直接点击地图即可设置起点或终点")
    
    flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
    center = st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ
    
    if st.session_state.planned_path is None:
        st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
    
    drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
    m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail,
        st.session_state.planned_path, straight_blocked, flight_alt, drone_pos, st.session_state.current_direction, st.session_state.safety_radius)
    
    output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
    handle_map_click(output); handle_drawing_output(output)

def handle_map_click(output):
    if output and output.get("last_clicked"):
        click = output["last_clicked"]
        if click and isinstance(click, dict) and click.get("lng") is not None:
            if st.session_state.waiting_for_start_point:
                st.session_state.points_gcj['A'] = [click["lng"], click["lat"]]
                update_path_after_point_change(); st.session_state.waiting_for_start_point = False
                st.success(f"✅ 起点已设置: ({click['lng']:.6f}, {click['lat']:.6f})"); st.rerun()
            elif st.session_state.waiting_for_end_point:
                st.session_state.points_gcj['B'] = [click["lng"], click["lat"]]
                update_path_after_point_change(); st.session_state.waiting_for_end_point = False
                st.success(f"✅ 终点已设置: ({click['lng']:.6f}, {click['lat']:.6f})"); st.rerun()

def handle_drawing_output(output):
    if output and output.get("last_active_drawing"):
        last = output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [[]])[0]
            poly = [[p[0], p[1]] for p in coords]
            if len(poly) >= 3 and st.session_state.pending_obstacle is None and validate_polygon(poly):
                st.session_state.pending_obstacle = poly; st.rerun()
    if st.session_state.pending_obstacle is not None: render_obstacle_dialog()

def render_obstacle_dialog():
    st.markdown("---"); st.subheader("📝 添加新障碍物")
    st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
    c1,c2 = st.columns(2)
    with c1: name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
    with c2: height = st.number_input("障碍物高度 (米)", 1, 200, 30, 5, key="height_input")
    c_ok, c_cancel = st.columns(2)
    if c_ok.button("✅ 确认添加", use_container_width=True, type="primary"):
        st.session_state.obstacles_gcj.append({"name": name, "polygon": st.session_state.pending_obstacle, "height": height,
            "selected": False, "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{len(st.session_state.obstacles_gcj)}",
            "created_time": datetime.now().isoformat()})
        if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
        st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, st.session_state.last_flight_altitude, st.session_state.current_direction, st.session_state.safety_radius)
        st.session_state.pending_obstacle = None; st.success(f"✅ 已添加 {name}，高度 {height} 米"); st.rerun()
    if c_cancel.button("❌ 取消", use_container_width=True): st.session_state.pending_obstacle = None; st.rerun()

# ==================== 飞行监控页面 ====================
def render_flight_monitoring_page(flight_alt, drone_speed):
    st.header("📡 飞行监控 - 实时心跳包")
    update_flight_simulation()
    
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        total_wp = len(st.session_state.planned_path) if st.session_state.planned_path else 0
        curr_wp = 0
        if st.session_state.planned_path:
            if latest.arrived: curr_wp = total_wp
            elif latest.progress < 1:
                curr_wp = min(total_wp, int(latest.progress * (total_wp - 1)) + 1)
            else: curr_wp = total_wp
        
        rem = max(0, latest.remaining_distance)
        eta = "00:00" if latest.arrived else (f"{rem/latest.speed:.0f}秒" if latest.speed>0 and rem>0 and rem/latest.speed<60 else
            f"{int(rem/latest.speed//60):02d}:{int(rem/latest.speed%60):02d}" if latest.speed>0 and rem>0 else "计算中...")
        
        battery = max(0, min(100, (1 - latest.flight_time/1800)*100))
        if latest.voltage: battery = max(0, min(100, (battery + ((latest.voltage-21)/1.2)*100)/2))
        
        st.markdown("### ✈️ 飞行进度")
        st.progress(latest.progress if not latest.arrived else 1.0, text=f"飞行进度：{int(latest.progress*100) if not latest.arrived else 100}%")
        
        st.markdown("### 📊 实时飞行数据")
        c1,c2,c3 = st.columns(3)
        with c1: st.metric("🎯 当前航点", f"{curr_wp}/{total_wp}"); st.progress(curr_wp/total_wp if total_wp>0 else 0, text=f"航点进度: {int(curr_wp/total_wp*100) if total_wp>0 else 0}%")
        with c2: st.metric("💨 飞行速度", f"{latest.speed:.1f} m/s", f"{drone_speed}% 系数" if not latest.arrived else "已到达")
        with c3: st.metric("⏰ 已用时间", f"{int(latest.flight_time//60):02d}:{int(latest.flight_time%60):02d}")
        
        c4,c5,c6 = st.columns(3)
        with c4: st.metric("📏 剩余距离", f"{rem/1000:.2f} km" if rem>=1000 else f"{rem:.0f} m")
        with c5: st.metric("🕐 预计到达", eta)
        with c6: st.metric("🔋 电量模拟", f"{'🟢' if battery>50 else '🟡' if battery>20 else '🔴'} {battery:.0f}%", f"{latest.voltage:.1f}V")
        
        st.markdown("### 📍 位置与状态")
        c7,c8,c9,c10 = st.columns(4)
        with c7: st.metric("📍 当前位置", f"{latest.lat:.6f}, {latest.lng:.6f}")
        with c8: st.metric("📏 飞行高度", f"{latest.altitude} m")
        with c9: st.metric("🛰️ 卫星数量", f"{latest.satellites} 颗")
        with c10: st.metric("📌 飞行状态", "✅ 已完成" if latest.arrived else "✈️ 飞行中" if st.session_state.simulation_running else "⏸️ 已停止")
        
        if latest.safety_violation and not latest.arrived: st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
        if latest.arrived: st.success("🎉 无人机已到达目的地！飞行任务完成！")
        
        st.markdown("---")
        st.markdown("### 🗺️ 实时位置追踪 & 🎮 飞行控制")
        c_left, c_right = st.columns([2,1])
        with c_left: display_monitor_map(flight_alt, latest)
        with c_right:
            st.markdown("#### 🎮 飞行控制")
            c1,c2 = st.columns(2)
            c1.metric("当前飞行高度", f"{latest.altitude} m")
            c1.metric("速度系数", f"{drone_speed}%")
            c2.metric("安全半径", f"{st.session_state.safety_radius} 米")
            if st.session_state.planned_path:
                st.metric("🎯 绕行点数量", len(st.session_state.planned_path)-2)
                st.caption(f"📏 规划路径总长: {calculate_path_length(st.session_state.planned_path)*111000:.0f} 米")
            st.markdown("**📍 当前坐标**")
            a,b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
            st.write(f"🟢 A点: ({a[0]:.6f}, {a[1]:.6f})")
            st.write(f"🔴 B点: ({b[0]:.6f}, {b[1]:.6f})")
            st.caption(f"📏 直线距离: {math.hypot(b[0]-a[0], b[1]-a[1])*111000:.0f} 米")
            st.caption(f"🛡️ 当前安全半径: {st.session_state.safety_radius} 米")
            if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                if a and b:
                    path = st.session_state.planned_path or [a,b]
                    comm = st.session_state.comm_sim
                    total = calculate_path_length(path)*111000
                    comm.add_planning_record({"message":"开始航线规划","details":f"算法: A* | 障碍物数量: {len(st.session_state.obstacles_gcj)}"})
                    comm.add_planning_record({"message":"航线规划完成","details":f"类型: horizontal | 航点数: {len(path)} | 路径长度: {total:.1f}m"})
                    comm.add_planning_record({"message":"导航目标","details":f"起点: {a} | 终点: {b} | 目标高度: {flight_alt}m"})
                    comm.send_message("GCS","OBC","START_MISSION",f"起点: {a}, 终点: {b}")
                    comm.send_message("OBC","FCU","UPLOAD_MISSION",f"航点数量: {len(path)}")
                    st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    comm.send_message("FCU","OBC","ACK","Mode: AUTO")
                    comm.send_message("OBC","GCS","ACK","任务已开始")
                    st.success(f"🚁 飞行已开始！{'路径中有' + str(len(path)-2) + '个绕行点' if len(path)>2 else '直线飞行'}")
                    st.rerun()
                else: st.error("请先设置起点和终点")
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
                st.session_state.comm_sim.send_message("GCS","OBC","STOP_MISSION","用户停止飞行")
                st.info("飞行已停止"); st.rerun()
            st.markdown("**📊 数据导出**")
            c1,c2 = st.columns(2)
            if c1.button("📊 导出飞行数据", use_container_width=True):
                df = st.session_state.heartbeat_sim.export_flight_data()
                if not df.empty: st.download_button("📥 下载CSV", df.to_csv(index=False), f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv", use_container_width=True)
            if c2.button("📊 导出航点数据", use_container_width=True) and st.session_state.planned_path:
                wp_data = [{"航点序号":i+1,"航点类型":"起点" if i==0 else "终点" if i==len(st.session_state.planned_path)-1 else f"绕行点{i}",
                            "经度":wp[0],"纬度":wp[1]} for i,wp in enumerate(st.session_state.planned_path)]
                st.download_button("📥 下载航点CSV", pd.DataFrame(wp_data).to_csv(index=False, encoding='utf-8-sig'),
                    f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv", use_container_width=True)
            if st.button("🔄 刷新数据", use_container_width=True): st.rerun()
        
        st.markdown("---"); st.markdown("### 📈 实时数据图表")
        hist = st.session_state.heartbeat_sim.history[:30]
        if len(hist)>1:
            c1,c2 = st.columns(2)
            c1.line_chart(pd.DataFrame([{"时间(s)":i*0.2,"速度(m/s)":h.speed} for i,h in enumerate(hist)]), x="时间(s)", y="速度(m/s)")
            c2.line_chart(pd.DataFrame([{"时间(s)":i*0.2,"剩余距离(m)":max(0,h.remaining_distance)} for i,h in enumerate(hist)]), x="时间(s)", y="剩余距离(m)")
            c3,c4 = st.columns(2)
            battery_data = []
            for i,h in enumerate(hist):
                b = max(0, min(100, (1 - h.flight_time/1800)*100))
                if h.voltage: b = max(0, min(100, (b + ((h.voltage-21)/1.2)*100)/2))
                battery_data.append({"时间(s)":i*0.2,"电量(%)":b})
            c3.line_chart(pd.DataFrame(battery_data), x="时间(s)", y="电量(%)")
            if st.session_state.planned_path:
                wp_data = [{"时间(s)":i*0.2,"已完成航点": min(len(st.session_state.planned_path), int(h.progress*(len(st.session_state.planned_path)-1))+1) if not h.arrived else len(st.session_state.planned_path)} for i,h in enumerate(hist)]
                c4.line_chart(pd.DataFrame(wp_data), x="时间(s)", y="已完成航点")
        
        st.markdown("---"); st.markdown("### 📋 飞行日志记录")
        display_flight_history()
    else:
        st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        col_tip1, col_tip2, col_tip3 = st.columns(3)
        col_tip1.info("💡 提示1：先在航线规划页面设置起点和终点")
        col_tip2.info("💡 提示2：设置飞行高度和速度系数")
        col_tip3.info("💡 提示3：点击「开始飞行」按钮启动模拟")
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            st.markdown("---"); st.subheader("🗺️ 规划航线预览")
            st.success(f"📌 已规划 {len(st.session_state.planned_path)} 个航点（包括起点和终点），点击开始飞行后将按此航线飞行")
            with st.expander("📋 查看详细航点列表"):
                st.table(pd.DataFrame([{"序号":i+1,"类型":"🚁 起点" if i==0 else "🏁 终点" if i==len(st.session_state.planned_path)-1 else f"📍 绕行点 {i}",
                                        "经度":f"{wp[0]:.6f}","纬度":f"{wp[1]:.6f}"} for i,wp in enumerate(st.session_state.planned_path)]))

def display_monitor_map(flight_alt, latest):
    tiles = config.GAODE_SATELLITE_URL
    m = folium.Map(location=[latest.lat, latest.lng], zoom_start=18, tiles=tiles, attr="高德卫星地图")
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon',[])
        if len(coords)>=3:
            color = "red" if obs.get('height',30) > flight_alt else "orange"
            folium.Polygon([[c[1],c[0]] for c in coords], color=color, weight=2, fill=True, fill_opacity=0.3, popup=f"🚧 {obs.get('name')}\n高度: {obs.get('height',30)}m").add_to(m)
    if st.session_state.planned_path and len(st.session_state.planned_path)>1:
        lc = "purple" if "向左" in st.session_state.current_direction else "orange" if "向右" in st.session_state.current_direction else "green"
        folium.PolyLine([[p[1],p[0]] for p in st.session_state.planned_path], color=lc, weight=3, opacity=0.7, popup=f"规划航线 - {st.session_state.current_direction}").add_to(m)
    folium.Circle(radius=st.session_state.safety_radius, location=[latest.lat, latest.lng], color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {st.session_state.safety_radius}米").add_to(m)
    trail = [[hb.lat, hb.lng] for hb in st.session_state.heartbeat_sim.history[:50] if hb.lat and hb.lng]
    if len(trail)>1: folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史飞行轨迹").add_to(m)
    folium.Marker([latest.lat, latest.lng], popup=f"当前位置\n高度: {latest.altitude}m\n速度: {latest.speed}m/s", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
    if st.session_state.points_gcj['A']: folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], popup="起点 A", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(m)
    if st.session_state.points_gcj['B']: folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], popup="终点 B", icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(m)
    if st.session_state.planned_path and len(st.session_state.planned_path)>2:
        for i,p in enumerate(st.session_state.planned_path[1:-1]): folium.CircleMarker([p[1],p[0]], radius=4, color="yellow", fill=True, fill_color="yellow", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    folium_static(m, width=900, height=500)

def display_flight_history():
    df = st.session_state.heartbeat_sim.export_flight_data()
    if not df.empty:
        display_cols = ['timestamp','flight_time','lat','lng','altitude','speed','voltage','satellites','remaining_distance']
        display_cols = [c for c in display_cols if c in df.columns]
        rename = {'timestamp':'时间','flight_time':'飞行时间(s)','lat':'纬度','lng':'经度','altitude':'高度(m)','speed':'速度(m/s)','voltage':'电压(V)','satellites':'卫星数','remaining_distance':'剩余距离(m)'}
        st.dataframe(df[display_cols].head(10).rename(columns=rename), use_container_width=True)
    else: st.info("暂无飞行数据")

def update_flight_simulation():
    if st.session_state.simulation_running:
        now = time.time()
        if now - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            try:
                hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj, st.session_state.comm_sim)
                if hb:
                    st.session_state.last_hb_time = now
                    st.session_state.flight_history.append([hb.lng, hb.lat])
                    if len(st.session_state.flight_history) > 200: st.session_state.flight_history.pop(0)
                    if not st.session_state.heartbeat_sim.simulating:
                        st.session_state.simulation_running = False
                        st.success("🏁 无人机已安全到达目的地！")
                    st.rerun()
            except Exception as e: st.error(f"更新心跳时出错: {e}")

# ==================== 障碍物管理页面 ====================
def render_obstacle_management_page(flight_alt):
    st.header("🚧 障碍物管理")
    c1,c2,c3,c4 = st.columns(4)
    c1.info(f"📊 当前共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    c2.info(f"🛡️ 安全半径: {st.session_state.safety_radius}米")
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE,'r',encoding='utf-8') as f: save_time = json.load(f).get('save_time','未知')
            c3.info(f"💾 最后保存: {save_time}")
        except: c3.info("💾 未保存")
    else: c3.info("💾 未保存")
    c4.info(f"📦 备份数量: {len([f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')])}")
    
    st.markdown("---")
    cols = st.columns(5)
    if cols[0].button("💾 保存配置", use_container_width=True, type="primary") and save_obstacles(st.session_state.obstacles_gcj):
        st.success(f"✅ 已保存 {len(st.session_state.obstacles_gcj)} 个障碍物"); st.balloons(); time.sleep(0.5); st.rerun()
    if cols[1].button("📂 加载配置", use_container_width=True):
        loaded = load_obstacles()
        if loaded: st.session_state.obstacles_gcj = loaded; update_path_after_obstacle_change(flight_alt); st.success(f"✅ 已加载 {len(loaded)} 个障碍物"); st.rerun()
        else: st.warning("⚠️ 未找到配置文件")
    if cols[2].button("📥 导出配置", use_container_width=True) and st.session_state.obstacles_gcj:
        st.download_button("📥 下载", json.dumps({'obstacles':st.session_state.obstacles_gcj,'count':len(st.session_state.obstacles_gcj),
            'export_time':datetime.now().strftime("%Y-%m-%d %H:%M:%S"),'version':'v13.2'}, ensure_ascii=False, indent=2),
            f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json", "application/json", use_container_width=True)
    if cols[3].button("🔄 恢复备份", use_container_width=True) and get_latest_backup():
        if restore_from_backup(get_latest_backup()): st.session_state.obstacles_gcj = load_obstacles(); update_path_after_obstacle_change(flight_alt); st.success("✅ 已从备份恢复"); st.rerun()
        else: st.error("❌ 恢复失败")
    if cols[4].button("🗑️ 清除全部", use_container_width=True):
        if st.session_state.auto_backup: backup_config()
        st.session_state.obstacles_gcj = []; save_obstacles([]); update_path_after_obstacle_change(flight_alt); st.success("✅ 已清除所有障碍物"); st.rerun()
    
    st.markdown("---")
    high_obs = sum(1 for o in st.session_state.obstacles_gcj if o.get('height',30) > flight_alt)
    c1,c2,c3,c4 = st.columns(4)
    c1.metric("🔴 需避让障碍物", high_obs)
    c2.metric("🟠 安全障碍物", len(st.session_state.obstacles_gcj)-high_obs)
    c3.metric("📍 总顶点数", sum(len(o.get('polygon',[])) for o in st.session_state.obstacles_gcj))
    c4.metric("📏 平均高度", f"{sum(o.get('height',30) for o in st.session_state.obstacles_gcj)/max(1,len(st.session_state.obstacles_gcj)):.1f}m")
    
    st.markdown("---"); st.subheader("🎯 批量操作")
    for o in st.session_state.obstacles_gcj: o.setdefault('selected', False)
    c1,c2,c3,c4 = st.columns(4)
    if c1.checkbox("☑️ 全选所有障碍物"):
        for o in st.session_state.obstacles_gcj: o['selected'] = True
    if c2.button("🗑️ 批量删除", use_container_width=True, type="primary"):
        sel = [i for i,o in enumerate(st.session_state.obstacles_gcj) if o.get('selected',False)]
        if sel:
            for i in reversed(sel): st.session_state.obstacles_gcj.pop(i)
            if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
            update_path_after_obstacle_change(flight_alt); st.success(f"✅ 已删除 {len(sel)} 个障碍物"); st.rerun()
        else: st.warning("⚠️ 请先选择要删除的障碍物")
    batch_h = c3.number_input("批量高度(m)",1,200,30,5,key="batch_height")
    if c3.button("📏 批量设置高度", use_container_width=True):
        sel = [i for i,o in enumerate(st.session_state.obstacles_gcj) if o.get('selected',False)]
        if sel:
            for i in sel: st.session_state.obstacles_gcj[i]['height'] = batch_h
            if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
            update_path_after_obstacle_change(flight_alt); st.success(f"✅ 已为 {len(sel)} 个障碍物设置高度为 {batch_h}m"); st.rerun()
        else: st.warning("⚠️ 请先选择要修改的障碍物")
    if c4.button("🏷️ 批量重命名", use_container_width=True):
        if [i for i,o in enumerate(st.session_state.obstacles_gcj) if o.get('selected',False)]: st.session_state.show_rename_dialog = True
        else: st.warning("⚠️ 请先选择要重命名的障碍物")
    
    if st.session_state.get('show_rename_dialog', False):
        with st.expander("🏷️ 批量重命名", expanded=True):
            c1,c2 = st.columns(2)
            with c1: prefix = st.text_input("名称前缀", "建筑物"); start = st.number_input("起始编号",1,1,1)
            with c2: suffix = st.text_input("名称后缀", "")
            cc1,cc2 = st.columns(2)
            if cc1.button("确认重命名", use_container_width=True, type="primary"):
                sel = [i for i,o in enumerate(st.session_state.obstacles_gcj) if o.get('selected',False)]
                for idx,i in enumerate(sel): st.session_state.obstacles_gcj[i]['name'] = f"{prefix}{start+idx}{suffix}"
                if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
                st.session_state.show_rename_dialog = False; st.success(f"✅ 已重命名 {len(sel)} 个障碍物"); st.rerun()
            if cc2.button("取消"): st.session_state.show_rename_dialog = False; st.rerun()
    
    st.markdown("---")
    tab1, tab2 = st.tabs(["📋 列表视图", "🗺️ 地图视图"])
    with tab1: render_obstacle_list_view(flight_alt)
    with tab2: render_obstacle_map_view(flight_alt)

def render_obstacle_list_view(flight_alt):
    st.subheader("📝 障碍物列表"); st.caption("💡 提示：勾选复选框后可使用批量操作功能")
    if st.session_state.obstacles_gcj:
        items_per_row = 2
        rows = (len(st.session_state.obstacles_gcj) + items_per_row - 1) // items_per_row
        for row in range(rows):
            cols = st.columns(items_per_row)
            for ci in range(items_per_row):
                idx = row * items_per_row + ci
                if idx < len(st.session_state.obstacles_gcj): render_obstacle_card(idx, flight_alt, cols[ci])
    else: st.info("📭 暂无任何障碍物，可以在「地图视图」中绘制添加")

def render_obstacle_card(idx, flight_alt, container):
    obs = st.session_state.obstacles_gcj[idx]
    with container:
        with st.container(border=True):
            height = obs.get('height',30)
            color = "🔴" if height > flight_alt else "🟠"
            name = obs.get('name', f'障碍物{idx+1}')
            c1,c2 = st.columns([1,5])
            with c1: checked = st.checkbox("", key=f"select_card_{idx}", value=obs.get('selected', False)); st.session_state.obstacles_gcj[idx]['selected'] = checked
            with c2: st.markdown(f"**{color} {name}**")
            c1,c2 = st.columns(2)
            with c1: st.caption(f"📏 高度: {height}m")
            with c2: st.caption(f"📍 顶点: {len(obs.get('polygon',[]))}个")
            new_h = st.number_input("调整高度", value=height, 1, 200, 5, key=f"quick_edit_{idx}", label_visibility="collapsed")
            if new_h != height:
                obs['height'] = new_h
                if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt); st.rerun()
            if st.button("🗑️ 删除", key=f"delete_card_{idx}", use_container_width=True):
                st.session_state.obstacles_gcj.pop(idx)
                if st.session_state.auto_backup: save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt); st.rerun()

def render_obstacle_map_view(flight_alt):
    st.subheader("🗺️ 地图视图")
    st.caption("✏️ 使用左上角绘制工具绘制新障碍物 | 🖱️ 点击障碍物查看详细信息 | 🎨 红色=需避让，橙色=安全")
    m = folium.Map(location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=config.GAODE_SATELLITE_URL, attr="高德卫星地图")
    m.add_child(plugins.Draw(export=True, position='topleft', draw_options={'polygon':{'allowIntersection':False,'showArea':True,'color':'#ff0000','fillColor':'#ff0000','fillOpacity':0.4},
                       'polyline':False,'rectangle':False,'circle':False,'marker':False,'circlemarker':False}, edit_options={'edit':True,'remove':True}))
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon',[])
        if len(coords)>=3:
            color = "red" if obs.get('height',30) > flight_alt else "orange"
            folium.Polygon([[c[1],c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.5,
                popup=folium.Popup(f"<div><b>🏢 {obs.get('name')}</b><br>高度: {obs.get('height',30)} 米<br>ID: {obs.get('id','N/A')}</div>", max_width=300)).add_to(m)
    folium.Marker([config.DEFAULT_A_GCJ[1], config.DEFAULT_A_GCJ[0]], popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(m)
    folium.Marker([config.DEFAULT_B_GCJ[1], config.DEFAULT_B_GCJ[0]], popup="终点", icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(m)
    out = st_folium(m, width=800, height=550, key="obstacle_map_view", returned_objects=["last_active_drawing"])
    if out and out.get("last_active_drawing"):
        last = out["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [[]])[0]
            poly = [[p[0],p[1]] for p in coords]
            if len(poly)>=3 and st.session_state.pending_obstacle is None and validate_polygon(poly):
                st.session_state.pending_obstacle = poly; st.rerun()
    if st.session_state.pending_obstacle is not None: render_obstacle_dialog()

def update_path_after_obstacle_change(flight_alt):
    st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)

# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    init_session_state()
    st.title("🏫 无人机地面站系统")
    st.markdown("---")
    
    page, drone_speed, flight_alt, auto_save = render_sidebar()
    st.session_state.auto_backup = auto_save
    
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path:
            st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, st.session_state.safety_radius)
            st.rerun()
    
    {"🗺️ 航线规划": render_planning_page, "📡 飞行监控": render_flight_monitoring_page,
     "🔗 通信拓扑": render_communication_page, "🚧 障碍物管理": render_obstacle_management_page}[page](drone_speed, flight_alt, auto_save)

if __name__ == "__main__":
    main()
