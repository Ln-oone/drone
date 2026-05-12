import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random, time, math, json, os
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
import pandas as pd
from dataclasses import dataclass, field

# ==================== 配置 ====================
@dataclass
class Config:
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.748807, 32.233931])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.750046, 32.236150])
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 5
    MAX_BACKUP_FILES: int = 10
    BASE_SPEED_MPS: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    VOLTAGE_VARIATION: float = 0.5
    SAT_RANGE: Tuple[int, int] = (8, 14)
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR_URL: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
    VERTICAL_OFFSET_MULTIPLIER: float = 3.0
    WAYPOINT_OFFSET_FACTOR: float = 10.0

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 几何工具 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    x, y, inside, n = point[0], point[1], False, len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i+1)%n]
        if ((y1 > y) != (y2 > y)) and (x < (x2-x1)*(y-y1)/(y2-y1)+x1):
            inside = not inside
    return inside

def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
    return min(p[0],r[0])<=q[0]<=max(p[0],r[0]) and min(p[1],r[1])<=q[1]<=max(p[1],r[1])

def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    val = (q[1]-p[1])*(r[0]-q[0])-(q[0]-p[0])*(r[1]-q[1])
    return 0 if abs(val)<1e-10 else 1 if val>0 else 2

def segments_intersect(p1,p2,p3,p4):
    o1,o2,o3,o4 = orientation(p1,p2,p3),orientation(p1,p2,p4),orientation(p3,p4,p1),orientation(p3,p4,p2)
    if o1!=o2 and o3!=o4: return True
    if o1==0 and on_segment(p1,p3,p2): return True
    if o2==0 and on_segment(p1,p4,p2): return True
    if o3==0 and on_segment(p3,p1,p4): return True
    if o4==0 and on_segment(p3,p2,p4): return True
    return False

def line_intersects_polygon(p1,p2,poly):
    if point_in_polygon(p1,poly) or point_in_polygon(p2,poly): return True
    n = len(poly)
    for i in range(n):
        if segments_intersect(p1,p2,poly[i],poly[(i+1)%n]): return True
    return False

def distance(p1,p2): return math.hypot(p1[0]-p2[0],p1[1]-p2[1])
def get_polygon_bounds(p): return None if not p else {'min_lng':min(x[0]for x in p),'max_lng':max(x[0]for x in p),'min_lat':min(x[1]for x in p),'max_lat':max(x[1]for x in p),'center_lng':sum(x[0]for x in p)/len(p),'center_lat':sum(x[1]for x in p)/len(p)}
def validate_polygon(p): return len(p)>=3

def meters_to_deg(m, lat=32.23):
    return m/111000/math.cos(math.radians(lat)), m/111000

def point_to_segment_distance_deg(p, a, b):
    px,py,x1,y1,x2,y2 = *p,*a,*b
    dx,dy,len_sq = x2-x1,y2-y1,(x2-x1)**2+(y2-y1)**2
    if len_sq==0: return math.hypot(px-x1,py-y1)
    t = max(0,min(1,((px-x1)*dx+(py-y1)*dy)/len_sq))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))

def point_to_segment_distance_meters(p,a,b):
    return point_to_segment_distance_deg(p,a,b)*111000

def check_safety_radius(drone_pos, obstacles, alt, radius):
    if not drone_pos: return True,None,None
    min_d, name = float('inf'), None
    for obs in obstacles:
        coords, h = obs.get('polygon',[]), obs.get('height',30)
        if h<=alt or len(coords)<3: continue
        for i in range(len(coords)):
            d = point_to_segment_distance_meters(drone_pos, coords[i], coords[(i+1)%len(coords)])
            if d<min_d: min_d, name = d, obs.get('name')
    return (False,min_d,name) if min_d<radius else (True,min_d if min_d!=float('inf')else None,name)

# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    try:
        files = sorted(f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE))
        if len(files)>config.MAX_BACKUP_FILES:
            for f in files[:-config.MAX_BACKUP_FILES]: os.remove(os.path.join(config.BACKUP_DIR,f))
    except: pass

def backup_config():
    if os.path.exists(config.CONFIG_FILE):
        import shutil
        bak = f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{datetime.now():%Y%m%d_%H%M%S}.bak"
        shutil.copy(config.CONFIG_FILE, bak)
        cleanup_old_backups()
        return bak

def load_obstacles():
    if not os.path.exists(config.CONFIG_FILE): return []
    try:
        with open(config.CONFIG_FILE,'r',encoding='utf-8')as f: d=json.load(f)
        for o in d.get('obstacles',[]):
            o.setdefault('selected',False)
            o.setdefault('height',30)
        return d.get('obstacles',[])
    except: return []

def save_obstacles(obs):
    try:
        backup_config()
        with open(config.CONFIG_FILE,'w',encoding='utf-8')as f:
            json.dump({'obstacles':obs,'count':len(obs),'save_time':datetime.now().strftime("%Y-%m-%d %H:%M:%S"),'version':'v13.1'},f,ensure_ascii=False,indent=2)
        return True
    except: return False

def get_latest_backup():
    try:
        files = sorted((f for f in os.listdir(config.BACKUP_DIR)if f.startswith(config.CONFIG_FILE)and f.endswith('.bak')),reverse=True)
        return os.path.join(config.BACKUP_DIR,files[0])if files else None
    except: return None

def restore_from_backup(path):
    try:
        with open(path,'r',encoding='utf-8')as f: obs=json.load(f).get('obstacles',[])
        save_obstacles(obs)
        return True
    except: return False

# ==================== 绕行算法 ====================
def get_blocking_obstacles(s,e,obs,alt):
    return [o for o in obs if o.get('height',30)>alt and o.get('polygon')and line_intersects_polygon(s,e,o.get('polygon'))]

def find_left_path(s,e,obs,alt,r=5):
    b = get_blocking_obstacles(s,e,obs,alt)
    if not b: return [s,e]
    max_lng,max_lat,min_lat=-1e9,-1e9,1e9
    for o in b:
        for p in o.get('polygon',[]):
            max_lng=max(max_lng,p[0])
            max_lat=max(max_lat,p[1])
            min_lat=min(min_lat,p[1])
    sl,sa = meters_to_deg(r*3)
    oh = max_lat-min_lat
    p1 = [s[0]+0.0012, max_lat+oh*3+sa*5+0.0002]
    p2 = [max_lng+oh*2+sl*3, p1[1]]
    return [s,p1,p2,e]

def find_right_path(s,e,obs,alt,r=5):
    if not get_blocking_obstacles(s,e,obs,alt): return [s,e]
    dx,dy = e[0]-s[0],e[1]-s[1]
    l = math.hypot(dx,dy)
    if l==0: return [s,e]
    mx,my = (s[0]+e[0])/2,(s[1]+e[1])/2
    ox,oy = dy/l,-dx/l
    o = r*config.WAYPOINT_OFFSET_FACTOR
    scl = 111000*math.cos(math.radians(my))
    return [s,[mx+ox*o/scl,my+oy*o/111000],e]

def calculate_path_length(p): return sum(distance(p[i],p[i+1])for i in range(len(p)-1))
def find_best_path(s,e,obs,alt,r=5):
    lp,rp = find_left_path(s,e,obs,alt,r),find_right_path(s,e,obs,alt,r)
    return lp if calculate_path_length(lp)<calculate_path_length(rp)else rp
def create_avoidance_path(s,e,obs,alt,d,r=5):
    if d=="向左绕行":return find_left_path(s,e,obs,alt,r)
    if d=="向右绕行":return find_right_path(s,e,obs,alt,r)
    return find_best_path(s,e,obs,alt,r)

# ==================== 心跳模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp:str;flight_time:float;lat:float;lng:float;altitude:float;voltage:float;satellites:int;speed:float;progress:float;arrived:bool;safety_violation:bool;remaining_distance:float

class HeartbeatSimulator:
    def __init__(self,start):
        self.history,self.current_pos,self.path,self.idx,self.simulating = [],start.copy(),[start.copy()],0,False
        self.alt,self.speed,self.progress,self.total_dist,self.traveled = 50,50,0,0,0
        self.radius,self.violation,self.start,self.log,self.last = config.DEFAULT_SAFETY_RADIUS_METERS,False,None,[],None

    def set_path(self,path,alt=50,speed=50,r=5):
        self.path,self.idx,self.current_pos = path,0,path[0].copy()
        self.alt,self.speed,self.radius = alt,speed,r
        self.simulating,self.progress,self.traveled,self.violation = True,0,0,False
        self.start = datetime.now()
        self.last = None
        self.total_dist = sum(distance(path[i],path[i+1])for i in range(len(path)-1))

    def update_and_generate(self,obs):
        if not self.simulating or self.idx>=len(self.path)-1:
            self.simulating=False
            return None
        now = time.time()
        dt = config.HEARTBEAT_INTERVAL if self.last is None else min(0.5,now-self.last)
        self.last = now
        s,e = self.path[self.idx],self.path[self.idx+1]
        seg = distance(s,e)
        move = config.BASE_SPEED_MPS*(self.speed/100)*dt
        self.traveled += move
        if self.total_dist>0:
            done=sum(distance(self.path[i],self.path[i+1])for i in range(self.idx))
            done+=seg*min(1,self.traveled/seg)if seg>0 else 0
            self.progress=min(1,done/self.total_dist)
        if self.traveled>=seg and self.traveled>0:
            self.idx+=1
            self.traveled=0
            if self.idx<len(self.path):self.current_pos=self.path[self.idx].copy()
            else:self.simulating=False
        elif seg>0:
            t=min(1,max(0,self.traveled/seg))
            self.current_pos=[s[0]+(e[0]-s[0])*t,s[1]+(e[1]-s[1])*t]
        safe,_,_=check_safety_radius(self.current_pos,obs,self.alt,self.radius)
        if not safe:self.violation=True
        return self._gen(False)

    def _gen(self,arrived):
        ft=(datetime.now()-self.start).total_seconds()if self.start else 0
        rem=0.0
        if not arrived and self.idx<len(self.path)-1:
            rem=distance(self.current_pos,self.path[self.idx+1])
            for i in range(self.idx+1,len(self.path)-1):rem+=distance(self.path[i],self.path[i+1])
            rem*=111000
        hb=HeartbeatData(
            datetime.now().strftime("%H:%M:%S"),ft,self.current_pos[1],self.current_pos[0],self.alt,
            round(22.2+random.uniform(-config.VOLTAGE_VARIATION,config.VOLTAGE_VARIATION),1),
            random.randint(*config.SAT_RANGE),round(config.BASE_SPEED_MPS*(self.speed/100),1),
            self.progress,arrived,self.violation,rem
        )
        self.history.insert(0,hb)
        if len(self.history)>100:self.history.pop()
        self.log.append(hb)
        if len(self.log)>1000:self.log.pop(0)
        return hb

    def export(self):
        return pd.DataFrame([{'timestamp':h.timestamp,'flight_time':h.flight_time,'lat':h.lat,'lng':h.lng,'altitude':h.altitude,'voltage':h.voltage,'satellites':h.satellites,'speed':h.speed,'progress':h.progress,'arrived':h.arrived,'safety_violation':h.safety_violation,'remaining_distance':h.remaining_distance}for h in self.log])

# ==================== 地图 ====================
def create_map(center,points,obs,trail=None,path=None,mt='satellite',blocked=True,alt=50,drone=None,dir='最佳',r=5):
    t,a=(config.GAODE_SATELLITE_URL,'高德卫星')if mt=='satellite'else(config.GAODE_VECTOR_URL,'高德矢量')
    m=folium.Map([center[1],center[0]],zoom_start=16,tiles=t,attr=a)
    plugins.Draw(export=True,position='topleft',draw_options={'polygon':{'allowIntersection':False,'showArea':True,'color':'#f00','fillColor':'#f00','fillOpacity':0.4},'polyline':False,'rectangle':False,'circle':False,'marker':False,'circlemarker':False},edit_options={'edit':True,'remove':True}).add_to(m)
    for o in obs:
        c=o.get('polygon',[])
        if len(c)>=3:folium.Polygon([[p[1],p[0]]for p in c],color='red'if o.get('height',30)>alt else'orange',weight=3,fill=True,fill_opacity=0.4,popup=f"{o.get('name')}\n高度:{o.get('height')}m").add_to(m)
    if points.get('A'):folium.Marker([points['A'][1],points['A'][0]],popup='起点',icon=folium.Icon(color='green',icon='play')).add_to(m)
    if points.get('B'):folium.Marker([points['B'][1],points['B'][0]],popup='终点',icon=folium.Icon(color='red',icon='stop')).add_to(m)
    if path and len(path)>1:
        clr='purple'if'左'in dir else'orange'if'右'in dir else'green'
        folium.PolyLine([[p[1],p[0]]for p in path],color=clr,weight=5,opacity=0.9).add_to(m)
        for i,p in enumerate(path[1:-1]):folium.CircleMarker([p[1],p[0]],radius=5,color=clr,fill=True).add_to(m)
    if points.get('A')and points.get('B'):
        folium.PolyLine([[points['A'][1],points['A'][0]],[points['B'][1],points['B'][0]]],color='blue'if not blocked else'gray',weight=2,opacity=0.5,dash_array='5,5').add_to(m)
    if drone:folium.Circle(radius=r,location=[drone[1],drone[0]],color='blue',weight=2,fill=True,fill_opacity=0.2).add_to(m)
    if trail and len(trail)>1:folium.PolyLine([[p[1],p[0]]for p in trail],color='orange',weight=2,opacity=0.6).add_to(m)
    return m

# ==================== 初始化 ====================
def init():
    d={'points':{'A':config.DEFAULT_A_GCJ.copy(),'B':config.DEFAULT_B_GCJ.copy()},'obs':load_obstacles(),'sim':HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),'last_hb':time.time(),'running':False,'trail':[],'path':None,'last_alt':50,'pending':None,'dir':'最佳航线','radius':config.DEFAULT_SAFETY_RADIUS_METERS,'auto':True,'rename':False,'wait_start':False,'wait_end':False,'temp':None}
    for k,v in d.items():
        if k not in st.session_state:st.session_state[k]=v
    for o in st.session_state.obs:o.setdefault('height',30);o.setdefault('selected',False)

def check_blocked(points,obs,alt):
    blocked,high=False,0
    for o in obs:
        if o.get('height',30)>alt:
            high+=1
            if line_intersects_polygon(points['A'],points['B'],o.get('polygon',[])):blocked=True
    return blocked,high

def sidebar():
    st.sidebar.title('导航')
    page=st.sidebar.radio('模块',['🗺️航线规划','📡飞行监控','🚧障碍物管理'])
    mt='satellite'if st.sidebar.radio('地图',['卫星影像','矢量街道'],0)=='卫星影像'else'vector'
    speed=st.sidebar.slider('速度系数',10,100,50,5)
    alt=st.sidebar.slider('飞行高度(m)',10,200,50,5)
    radius=st.sidebar.slider('安全半径(m)',1,20,st.session_state.radius,1)
    auto=st.sidebar.checkbox('自动保存',st.session_state.auto)
    st.session_state.radius=radius
    return page,mt,speed,alt,auto

# ==================== 航线规划 ====================
def update_path(alt):
    st.session_state.path=create_avoidance_path(st.session_state.points['A'],st.session_state.points['B'],st.session_state.obs,alt,st.session_state.dir,st.session_state.radius)

def planning(mt,speed,alt,auto):
    st.header('航线规划')
    blocked,high=check_blocked(st.session_state.points,st.session_state.obs,alt)
    st.warning(f'有{high}个障碍物高于{alt}m')if blocked else st.success('直线畅通')
    st.info('左上角绘制多边形→设置高度→保存')
    c1,c2=st.columns([1,1.5])
    with c1:
        st.subheader('控制面板')
        with st.expander('起点/终点',True):
            mode=st.radio('设置方式',['经纬度输入','鼠标点击设置'],0)
            if mode=='经纬度输入':
                a1,a2=st.columns(2)
                alat,alon=a1.number_input('A纬度',value=st.session_state.points['A'][1],format='%.6f'),a2.number_input('A经度',value=st.session_state.points['A'][0],format='%.6f')
                if st.button('设置A点'):st.session_state.points['A']=[alon,alat];update_path(alt);st.rerun()
                b1,b2=st.columns(2)
                blat,blon=b1.number_input('B纬度',value=st.session_state.points['B'][1],format='%.6f'),b2.number_input('B经度',value=st.session_state.points['B'][0],format='%.6f')
                if st.button('设置B点'):st.session_state.points['B']=[blon,blat];update_path(alt);st.rerun()
            else:
                if st.button('设置起点'):st.session_state.wait_start=True;st.session_state.wait_end=False
                if st.button('设置终点'):st.session_state.wait_end=True;st.session_state.wait_start=False
                if st.session_state.wait_start or st.session_state.wait_end:
                    st.warning('等待点击地图')
                    if st.button('取消'):st.session_state.wait_start=st.session_state.wait_end=False
        with st.expander('绕行策略',True):
            d1,d2,d3=st.columns(3)
            if d1.button('最佳航线'):st.session_state.dir='最佳航线';update_path(alt);st.rerun()
            if d2.button('向左绕行'):st.session_state.dir='向左绕行';update_path(alt);st.rerun()
            if d3.button('向右绕行'):st.session_state.dir='向右绕行';update_path(alt);st.rerun()
            st.info(f'当前：{st.session_state.dir}')
            if st.button('重新规划'):update_path(alt);st.rerun()
        with st.expander('飞行控制',True):
            b1,b2=st.columns(2)
            if b1.button('开始飞行',type='primary'):
                st.session_state.sim.set_path(st.session_state.path or [st.session_state.points['A'],st.session_state.points['B']],alt,speed,st.session_state.radius)
                st.session_state.running=True;st.session_state.trail=[];st.rerun()
            if b2.button('停止飞行'):st.session_state.running=False;st.session_state.sim.simulating=False
    with c2:
        st.subheader('地图')
        trail=[[h.lng,h.lat]for h in st.session_state.sim.history[:20]]
        if not st.session_state.path:update_path(alt)
        m=create_map(st.session_state.points['A'],st.session_state.points,st.session_state.obs,trail,st.session_state.path,mt,blocked,alt,st.session_state.sim.current_pos if st.session_state.sim.simulating else None,st.session_state.dir,st.session_state.radius)
        out=st_folium(m,width=700,height=550,returned_objects=['last_active_drawing','last_clicked'])
        if out and out.get('last_clicked'):
            c=out['last_clicked']
            if c and (lng:=c.get('lng'))and(lat:=c.get('lat')):
                if st.session_state.wait_start:
                    st.session_state.points['A']=[lng,lat];update_path(alt);st.session_state.wait_start=False;st.rerun()
                if st.session_state.wait_end:
                    st.session_state.points['B']=[lng,lat];update_path(alt);st.session_state.wait_end=False;st.rerun()
        if out and out.get('last_active_drawing'):
            g=out['last_active_drawing']['geometry']
            if g and g['type']=='Polygon':
                coords=[[p[0],p[1]]for p in g['coordinates'][0]]
                if len(coords)>=3 and not st.session_state.pending:
                    st.session_state.pending=coords;st.rerun()
    if st.session_state.pending:
        st.subheader('添加障碍物')
        name=st.text_input('名称',f'建筑物{len(st.session_state.obs)+1}')
        h=st.number_input('高度(m)',1,200,30,5)
        if st.button('确认添加',type='primary'):
            st.session_state.obs.append({'name':name,'polygon':st.session_state.pending,'height':h,'selected':False,'id':f"obs_{datetime.now():%Y%m%d_%H%M%S}"});save_obstacles(st.session_state.obs)if auto else None;update_path(alt);st.session_state.pending=None;st.rerun()
        if st.button('取消'):st.session_state.pending=None;st.rerun()

# ==================== 监控 ====================
def update_sim():
    now=time.time()
    if st.session_state.running and now-st.session_state.last_hb>=config.HEARTBEAT_INTERVAL:
        hb=st.session_state.sim.update_and_generate(st.session_state.obs)
        if hb:
            st.session_state.last_hb=now
            st.session_state.trail.append([hb.lng,hb.lat])
            if len(st.session_state.trail)>200:st.session_state.trail.pop(0)
            if not st.session_state.sim.simulating:st.session_state.running=False
            st.rerun()

def monitor(mt,alt,speed):
    st.header('飞行监控')
    update_sim()
    if not st.session_state.sim.history:
        st.info('请先开始飞行');return
    hb=st.session_state.sim.history[0]
    total=len(st.session_state.path)if st.session_state.path else 0
    curr=0
    if total>0:
        if hb.arrived:curr=total
        else:curr=min(int(hb.progress*(total-1))+1,total)
    rem=max(0,hb.remaining_distance if not hb.arrived else 0)
    eta='00:00'
    if not hb.arrived and hb.speed>0 and rem>0:
        s=rem/hb.speed
        eta=f"{int(s//60):02d}:{int(s%60):02d}"if s<3600 else f"{int(s//3600):02d}:{int((s%3600)//60):02d}"
    bat=max(0,min(100,(1-hb.flight_time/1800)*100))
    if hb.voltage:bat=((bat+((hb.voltage-21)/(22.2-21))*100)/2)
    st.progress(hb.progress if not hb.arrived else 1,f'进度{int(hb.progress*100)}%')
    c1,c2,c3=st.columns(3)
    c1.metric('航点',f'{curr}/{total}')
    c2.metric('速度',f'{hb.speed:.1f}m/s')
    c3.metric('已用时间',f'{int(hb.flight_time//60):02d}:{int(hb.flight_time%60):02d}')
    c4,c5,c6=st.columns(3)
    c4.metric('剩余距离',f'{rem:.0f}m')
    c5.metric('预计到达',eta)
    c6.metric('电量',f'{"🟢"if bat>50 else"🟡"if bat>20 else"🔴"}{bat:.0f}%')
    if hb.safety_violation and not hb.arrived:st.error('进入危险区域！')
    m=folium.Map([hb.lat,hb.lng],18,config.GAODE_SATELLITE_URL if mt=='satellite'else config.GAODE_VECTOR_URL)
    for o in st.session_state.obs:
        c=o.get('polygon',[])
        if len(c)>=3:folium.Polygon([[p[1],p[0]]for p in c],color='red'if o.get('height',30)>alt else'orange',weight=2,fill=True,fill_opacity=0.3).add_to(m)
    if st.session_state.path:folium.PolyLine([[p[1],p[0]]for p in st.session_state.path],color='purple'if'左'in st.session_state.dir else'orange'if'右'in st.session_state.dir else'green',weight=3).add_to(m)
    folium.Circle(radius=st.session_state.radius,location=[hb.lat,hb.lng],color='blue',fill=True,fill_opacity=0.2).add_to(m)
    folium.Marker([hb.lat,hb.lng],icon=folium.Icon(color='red',icon='plane')).add_to(m)
    folium_static(m,900,500)

# ==================== 障碍物管理 ====================
def obs_manage(alt):
    st.header('障碍物管理')
    c1,c2,c3,c4=st.columns(4)
    c1.info(f'总数：{len(st.session_state.obs)}')
    c2.info(f'安全半径：{st.session_state.radius}m')
    c4.info(f'备份：{len([f for f in os.listdir(config.BACKUP_DIR)if f.startswith(config.CONFIG_FILE)])}')
    b1,b2,b3,b4,b5=st.columns(5)
    if b1.button('保存',type='primary'):save_obstacles(st.session_state.obs);st.success('保存成功');st.rerun()
    if b2.button('加载'):st.session_state.obs=load_obstacles();update_path(alt);st.rerun()
    b3.download_button('导出',json.dumps({'obstacles':st.session_state.obs},ensure_ascii=False,indent=2),'obs.json')
    if b4.button('恢复备份')and(p:=get_latest_backup()):restore_from_backup(p);st.session_state.obs=load_obstacles();update_path(alt);st.rerun()
    if b5.button('清空'):st.session_state.obs=[];save_obstacles([]);update_path(alt);st.rerun()
    tab1,tab2=st.tabs(['列表','地图'])
    with tab1:
        for i,o in enumerate(st.session_state.obs):
            with st.container(border=True):
                c1,c2=st.columns([1,5])
                o['selected']=c1.checkbox('',o.get('selected',False),key=f's{i}')
                c2.markdown(f"{'🔴'if o.get('height',30)>alt else'🟠'}{o.get('name')}")
                h=st.number_input('高度',value=o.get('height',30),min_value=1,max_value=200,key=f'h{i}')
                if h!=o.get('height'):o['height']=h;save_obstacles(st.session_state.obs);update_path(alt);st.rerun()
                if st.button('删除',key=f'd{i}'):st.session_state.obs.pop(i);save_obstacles(st.session_state.obs);update_path(alt);st.rerun()
    with tab2:
        m=folium.Map([config.SCHOOL_CENTER_GCJ[1],config.SCHOOL_CENTER_GCJ[0]],16,config.GAODE_SATELLITE_URL)
        plugins.Draw(export=True,position='topleft',draw_options={'polygon':{'color':'#f00','fillColor':'#f00','fillOpacity':0.4}}).add_to(m)
        for o in st.session_state.obs:
            c=o.get('polygon',[])
            if len(c)>=3:folium.Polygon([[p[1],p[0]]for p in c],color='red'if o.get('height',30)>alt else'orange',weight=3,fill=True,fill_opacity=0.5).add_to(m)
        out=st_folium(m,800,550,returned_objects=['last_active_drawing'])
        if out and out.get('last_active_drawing'):
            g=out['last_active_drawing']['geometry']
            if g and g['type']=='Polygon':
                coords=[[p[0],p[1]]for p in g['coordinates'][0]]
                if len(coords)>=3 and not st.session_state.pending:
                    st.session_state.pending=coords;st.rerun()
    if st.session_state.pending:
        name=st.text_input('名称',f'建筑物{len(st.session_state.obs)+1}')
        h=st.number_input('高度(m)',1,200,30)
        if st.button('确认添加',type='primary'):
            st.session_state.obs.append({'name':name,'polygon':st.session_state.pending,'height':h,'selected':False});save_obstacles(st.session_state.obs);update_path(alt);st.session_state.pending=None;st.rerun()
        if st.button('取消'):st.session_state.pending=None;st.rerun()

# ==================== 主程序 ====================
def main():
    st.set_page_config('无人机地面站',layout='wide')
    init()
    st.title('无人机地面站系统')
    page,mt,speed,alt,auto=sidebar()
    if alt!=st.session_state.last_alt:
        st.session_state.last_alt=alt;update_path(alt);st.rerun()
    if page=='🗺️航线规划':planning(mt,speed,alt,auto)
    elif page=='📡飞行监控':monitor(mt,alt,speed)
    elif page=='🚧障碍物管理':obs_manage(alt)

if __name__=='__main__':main()
