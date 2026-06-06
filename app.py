# ==================== 改进的绕行算法 ====================
import math
import heapq
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass

@dataclass
class Point2D:
    """二维点"""
    x: float  # 经度
    y: float  # 纬度
    
    def distance_to(self, other: 'Point2D') -> float:
        """计算距离（度）"""
        return math.hypot(self.x - other.x, self.y - other.y)
    
    def to_list(self) -> List[float]:
        return [self.x, self.y]

@dataclass
class Edge:
    """边（连接两个点）"""
    p1: Point2D
    p2: Point2D
    
    def length(self) -> float:
        return self.p1.distance_to(self.p2)

class VisibilityGraph:
    """可见性图 - 用于路径规划的核心数据结构"""
    
    def __init__(self, start: Point2D, end: Point2D, obstacles: List[Dict], 
                 flight_altitude: float, safety_radius: float, extra_margin: float = 1.0):
        self.start = start
        self.end = end
        self.obstacles = obstacles
        self.flight_altitude = flight_altitude
        self.safety_radius = safety_radius
        self.extra_margin = extra_margin
        self.required_distance = safety_radius + extra_margin
        
        # 所有节点（起点、终点、障碍物顶点）
        self.nodes: List[Point2D] = [start, end]
        self.node_origins: List[str] = ["start", "end"]  # 标记节点来源
        
        # 添加障碍物顶点（向外扩展安全距离）
        self._add_obstacle_vertices()
        
        # 构建邻接表
        self.adjacency: Dict[int, List[Tuple[int, float]]] = {}
        self._build_graph()
    
    def _expand_polygon(self, polygon: List[List[float]]) -> List[Point2D]:
        """向外扩展多边形（安全距离）"""
        if len(polygon) < 3:
            return []
        
        # 转换为中心点
        center_lng = sum(p[0] for p in polygon) / len(polygon)
        center_lat = sum(p[1] for p in polygon) / len(polygon)
        center = Point2D(center_lng, center_lat)
        
        # 计算扩展系数（将米转换为度）
        mid_lat = (polygon[0][1] + polygon[1][1]) / 2
        deg_per_meter_lng = 1 / (111000 * math.cos(math.radians(mid_lat)))
        deg_per_meter_lat = 1 / 111000
        
        expand_lng = self.required_distance * deg_per_meter_lng
        expand_lat = self.required_distance * deg_per_meter_lat
        
        expanded = []
        for p in polygon:
            # 沿径向向外扩展
            point = Point2D(p[0], p[1])
            dx = point.x - center.x
            dy = point.y - center.y
            dist = math.hypot(dx, dy)
            
            if dist > 1e-10:
                # 扩展方向
                expand_x = dx / dist * expand_lng
                expand_y = dy / dist * expand_lat
            else:
                expand_x, expand_y = expand_lng, expand_lat
            
            expanded.append(Point2D(point.x + expand_x, point.y + expand_y))
        
        return expanded
    
    def _add_obstacle_vertices(self):
        """添加障碍物顶点作为节点"""
        for obs in self.obstacles:
            # 只考虑高于飞行高度的障碍物
            if obs.get('height', 30) <= self.flight_altitude:
                continue
            
            polygon = obs.get('polygon', [])
            if not polygon or len(polygon) < 3:
                continue
            
            # 扩展多边形（安全距离）
            expanded = self._expand_polygon(polygon)
            
            # 添加扩展后的顶点
            for point in expanded:
                self.nodes.append(point)
                self.node_origins.append(f"obstacle_{id(obs)}")
    
    def _is_visible(self, p1: Point2D, p2: Point2D) -> bool:
        """检查两点之间是否可见（不被障碍物阻挡）"""
        # 检查所有障碍物
        for obs in self.obstacles:
            if obs.get('height', 30) <= self.flight_altitude:
                continue
            
            polygon = obs.get('polygon', [])
            if not polygon or len(polygon) < 3:
                continue
            
            # 检查线段是否与障碍物相交
            if self._line_intersects_polygon(p1, p2, polygon):
                return False
            
            # 检查线段是否离障碍物太近
            if self._line_too_close_to_polygon(p1, p2, polygon):
                return False
        
        return True
    
    def _line_intersects_polygon(self, p1: Point2D, p2: Point2D, polygon: List[List[float]]) -> bool:
        """检查线段是否与多边形相交"""
        # 检查端点是否在多边形内部
        if self._point_in_polygon(p1, polygon) or self._point_in_polygon(p2, polygon):
            return True
        
        # 检查线段是否与多边形的边相交
        for i in range(len(polygon)):
            p3 = Point2D(polygon[i][0], polygon[i][1])
            p4 = Point2D(polygon[(i+1) % len(polygon)][0], polygon[(i+1) % len(polygon)][1])
            
            if self._segments_intersect(p1, p2, p3, p4):
                return True
        
        return False
    
    def _line_too_close_to_polygon(self, p1: Point2D, p2: Point2D, polygon: List[List[float]]) -> bool:
        """检查线段是否离多边形太近"""
        # 对线段进行采样检查
        sample_count = max(20, int(p1.distance_to(p2) * 111000 / 2))
        
        for k in range(sample_count + 1):
            t = k / sample_count
            px = p1.x + (p2.x - p1.x) * t
            py = p1.y + (p2.y - p1.y) * t
            point = Point2D(px, py)
            
            # 检查点到多边形每条边的距离
            for i in range(len(polygon)):
                p3 = Point2D(polygon[i][0], polygon[i][1])
                p4 = Point2D(polygon[(i+1) % len(polygon)][0], polygon[(i+1) % len(polygon)][1])
                
                dist = self._point_to_segment_distance(point, p3, p4) * 111000  # 转换为米
                if dist < self.required_distance:
                    return True
        
        return False
    
    def _point_in_polygon(self, point: Point2D, polygon: List[List[float]]) -> bool:
        """射线法判断点是否在多边形内"""
        x, y = point.x, point.y
        inside = False
        n = len(polygon)
        
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i+1) % n]
            
            # 检查是否在顶点上
            if (x == x1 and y == y1) or (x == x2 and y == y2):
                return True
            
            # 检查射线是否穿过边
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                inside = not inside
        
        return inside
    
    def _segments_intersect(self, p1: Point2D, p2: Point2D, p3: Point2D, p4: Point2D) -> bool:
        """检查两条线段是否相交"""
        def orientation(p, q, r):
            val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
            if abs(val) < 1e-10:
                return 0
            return 1 if val > 0 else 2
        
        o1 = orientation(p1, p2, p3)
        o2 = orientation(p1, p2, p4)
        o3 = orientation(p3, p4, p1)
        o4 = orientation(p3, p4, p2)
        
        # 一般情况
        if o1 != o2 and o3 != o4:
            return True
        
        # 特殊情况（共线）
        if o1 == 0 and self._on_segment(p1, p3, p2):
            return True
        if o2 == 0 and self._on_segment(p1, p4, p2):
            return True
        if o3 == 0 and self._on_segment(p3, p1, p4):
            return True
        if o4 == 0 and self._on_segment(p3, p2, p4):
            return True
        
        return False
    
    def _on_segment(self, p: Point2D, q: Point2D, r: Point2D) -> bool:
        """检查点q是否在线段pr上"""
        return (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and
                q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y))
    
    def _point_to_segment_distance(self, p: Point2D, a: Point2D, b: Point2D) -> float:
        """点到线段的最短距离（度）"""
        # 计算向量
        ab = Point2D(b.x - a.x, b.y - a.y)
        ap = Point2D(p.x - a.x, p.y - a.y)
        
        # 计算投影参数
        dot = ap.x * ab.x + ap.y * ab.y
        if dot <= 0:
            return p.distance_to(a)
        
        ab_len_sq = ab.x * ab.x + ab.y * ab.y
        if dot >= ab_len_sq:
            return p.distance_to(b)
        
        # 投影点在线段上
        t = dot / ab_len_sq
        proj = Point2D(a.x + t * ab.x, a.y + t * ab.y)
        return p.distance_to(proj)
    
    def _build_graph(self):
        """构建可见性图"""
        n = len(self.nodes)
        
        for i in range(n):
            self.adjacency[i] = []
            for j in range(n):
                if i == j:
                    continue
                
                # 检查两点之间是否可见
                if self._is_visible(self.nodes[i], self.nodes[j]):
                    dist = self.nodes[i].distance_to(self.nodes[j])
                    self.adjacency[i].append((j, dist))
    
    def find_shortest_path(self) -> Optional[List[Point2D]]:
        """使用Dijkstra算法找最短路径"""
        n = len(self.nodes)
        dist = [float('inf')] * n
        prev = [-1] * n
        dist[0] = 0  # 起点是索引0
        
        pq = [(0, 0)]  # (距离, 节点索引)
        
        while pq:
            d, u = heapq.heappop(pq)
            
            if d > dist[u]:
                continue
            
            # 到达终点
            if u == 1:  # 终点是索引1
                break
            
            for v, weight in self.adjacency[u]:
                if dist[v] > dist[u] + weight:
                    dist[v] = dist[u] + weight
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))
        
        # 重建路径
        if prev[1] == -1:
            return None
        
        path = []
        u = 1
        while u != -1:
            path.append(self.nodes[u])
            u = prev[u]
        path.reverse()
        
        return path


def get_blocking_obstacles(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    """获取阻挡航线的障碍物（改进版）"""
    blocking = []
    start_point = Point2D(start[0], start[1])
    end_point = Point2D(end[0], end[1])
    
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            polygon = obs.get('polygon', [])
            if polygon and len(polygon) >= 3:
                # 检查直线是否与障碍物相交
                vg = VisibilityGraph(start_point, end_point, [obs], flight_altitude, 5.0, 0)
                if not vg._is_visible(start_point, end_point):
                    blocking.append(obs)
    
    return blocking


def find_best_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict],
                              flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    """基于可见性图的最佳绕行路径"""
    
    start_point = Point2D(start[0], start[1])
    end_point = Point2D(end[0], end[1])
    
    # 获取相关障碍物（高于飞行高度的）
    relevant_obstacles = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            polygon = obs.get('polygon', [])
            if polygon and len(polygon) >= 3:
                relevant_obstacles.append(obs)
    
    if not relevant_obstacles:
        return [start, end]
    
    # 构建可见性图并找最短路径
    vg = VisibilityGraph(start_point, end_point, relevant_obstacles, flight_altitude, safety_radius)
    path = vg.find_shortest_path()
    
    if path and len(path) >= 2:
        # 转换回列表格式
        result = [p.to_list() for p in path]
        
        # 路径简化（去除冗余的共线点）
        result = simplify_path(result)
        
        return result
    
    # 如果找不到路径，使用简单的偏移绕行
    return _fallback_avoidance_path(start, end, relevant_obstacles, flight_altitude, safety_radius)


def simplify_path(path: List[List[float]], epsilon: float = 1e-8) -> List[List[float]]:
    """简化路径，去除共线的冗余点"""
    if len(path) <= 2:
        return path
    
    simplified = [path[0]]
    
    for i in range(1, len(path) - 1):
        # 检查三点是否共线
        p1 = Point2D(simplified[-1][0], simplified[-1][1])
        p2 = Point2D(path[i][0], path[i][1])
        p3 = Point2D(path[i+1][0], path[i+1][1])
        
        # 计算叉积判断共线性
        cross = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x)
        
        if abs(cross) > epsilon:
            # 不共线，保留该点
            simplified.append(path[i])
    
    simplified.append(path[-1])
    return simplified


def _fallback_avoidance_path(start: List[float], end: List[float], obstacles: List[Dict],
                              flight_altitude: float, safety_radius: float) -> List[List[float]]:
    """保底方案：简单的偏移绕行"""
    
    # 计算所有障碍物的边界
    min_lng, max_lng = float('inf'), -float('inf')
    min_lat, max_lat = float('inf'), -float('inf')
    
    for obs in obstacles:
        for point in obs.get('polygon', []):
            min_lng = min(min_lng, point[0])
            max_lng = max(max_lng, point[0])
            min_lat = min(min_lat, point[1])
            max_lat = max(max_lat, point[1])
    
    # 计算扩展距离
    mid_lat = (start[1] + end[1]) / 2
    deg_per_meter_lng = 1 / (111000 * math.cos(math.radians(mid_lat)))
    deg_per_meter_lat = 1 / 111000
    
    offset_m = (safety_radius + 5) * 2  # 增加偏移量
    offset_lng = offset_m * deg_per_meter_lng
    offset_lat = offset_m * deg_per_meter_lat
    
    # 判断走哪一侧更短
    left_distance = abs(start[0] - min_lng) + abs(end[0] - min_lng)
    right_distance = abs(start[0] - max_lng) + abs(end[0] - max_lng)
    
    if left_distance <= right_distance:
        # 从左侧绕行
        bypass_lng = min_lng - offset_lng
    else:
        # 从右侧绕行
        bypass_lng = max_lng + offset_lng
    
    # 生成绕行点
    waypoints = []
    waypoints.append([bypass_lng, start[1]])
    
    # 添加中间纬度点
    lat_step = (max_lat - min_lat) / 4
    for i in range(1, 5):
        lat = min_lat + i * lat_step
        waypoints.append([bypass_lng, lat])
    
    waypoints.append([bypass_lng, end[1]])
    
    return [start] + waypoints + [end]


def create_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict],
                          flight_altitude: float, direction: str, safety_radius: float = 5) -> Optional[List[List[float]]]:
    """创建绕行路径的主入口函数"""
    if not start or not end:
        return None
    
    # 检查直线是否安全
    if is_path_segment_clear(start, end, obstacles_gcj, flight_altitude, safety_radius, 1.0):
        return [start, end]
    
    # 使用改进的路径规划算法
    result = find_best_avoidance_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    
    if result and len(result) >= 2:
        # 验证路径安全性
        all_clear = True
        for i in range(len(result) - 1):
            if not is_path_segment_clear(result[i], result[i+1], obstacles_gcj, flight_altitude, safety_radius, 1.0):
                all_clear = False
                break
        
        if all_clear:
            return result
    
    # 如果路径不安全或不存在，返回直线（后续会有安全检测）
    return [start, end]


def is_path_segment_clear(p1: List[float], p2: List[float], obstacles: List[Dict], 
                          flight_altitude: float, safety_radius: float, extra_margin: float = 1.0) -> bool:
    """检查线段是否安全（改进版，更精确）"""
    required_distance = safety_radius + extra_margin
    start_point = Point2D(p1[0], p1[1])
    end_point = Point2D(p2[0], p2[1])
    
    for obs in obstacles:
        if obs.get('height', 30) <= flight_altitude:
            continue
        polygon = obs.get('polygon', [])
        if not polygon or len(polygon) < 3:
            continue
        
        # 使用改进的几何检查
        vg = VisibilityGraph(start_point, end_point, [obs], flight_altitude, safety_radius, extra_margin)
        if not vg._is_visible(start_point, end_point):
            return False
    
    return True
