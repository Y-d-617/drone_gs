import streamlit as st
import pandas as pd
import time
import folium
from folium.plugins import Draw
from streamlit_folium import st_folium
from heartbeat_sim import HeartbeatSimulator
import math
import json
import os
import heapq
import random
from datetime import datetime
from collections import deque

# ========== 障碍物持久化 ==========
OBSTACLE_FILE = "obstacles.json"

def save_obstacles_to_file(obstacles):
    try:
        with open(OBSTACLE_FILE, 'w', encoding='utf-8') as f:
            json.dump(obstacles, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        st.error(f"保存障碍物失败: {e}")
        return False

def load_obstacles_from_file():
    if os.path.exists(OBSTACLE_FILE):
        try:
            with open(OBSTACLE_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, list):
                    cleaned = []
                    for obs in data:
                        if isinstance(obs, dict) and "vertices" in obs and "height" in obs:
                            if isinstance(obs["vertices"], list) and len(obs["vertices"]) >= 3:
                                cleaned.append(obs)
                    return cleaned
                return []
        except Exception as e:
            st.error(f"加载障碍物失败: {e}")
    return []

# ========== 通信链路管理类 ==========
class CommunicationLink:
    def __init__(self):
        self.link_status = {
            "GCS": "online",
            "OBC": "online", 
            "FCU": "online"
        }
        self.data_flow_history = deque(maxlen=100)
        self.pending_commands = deque(maxlen=50)
        self.current_flow = None
        
    def send_command(self, source, target, command_type, data=None):
        timestamp = datetime.now()
        command_id = f"{timestamp.strftime('%H%M%S')}_{random.randint(1000,9999)}"
        
        log_entry = {
            "timestamp": timestamp,
            "command_id": command_id,
            "direction": "downlink",
            "source": source,
            "target": target,
            "type": command_type,
            "data": data,
            "status": "sent"
        }
        
        if self.link_status["GCS"] == "online" and self.link_status["OBC"] == "online":
            log_entry["status"] = "relayed_to_OBC"
            self.data_flow_history.append(log_entry)
            
            if self.link_status["FCU"] == "online":
                fcu_log = {
                    "timestamp": datetime.now(),
                    "command_id": command_id,
                    "direction": "downlink",
                    "source": "OBC",
                    "target": "FCU",
                    "type": command_type,
                    "data": data,
                    "status": "delivered"
                }
                self.data_flow_history.append(fcu_log)
                self.current_flow = {
                    "path": ["GCS", "OBC", "FCU"],
                    "command": command_type,
                    "timestamp": timestamp
                }
                return True
            else:
                log_entry["status"] = "failed_FCU_offline"
                self.data_flow_history.append(log_entry)
                return False
        else:
            log_entry["status"] = "failed_link_down"
            self.data_flow_history.append(log_entry)
            return False
    
    def send_telemetry(self, source, target, data_type, data=None):
        timestamp = datetime.now()
        data_id = f"{timestamp.strftime('%H%M%S')}_{random.randint(1000,9999)}"
        
        log_entry = {
            "timestamp": timestamp,
            "data_id": data_id,
            "direction": "uplink",
            "source": source,
            "target": target,
            "type": data_type,
            "data": data,
            "status": "sent"
        }
        
        if self.link_status["FCU"] == "online" and self.link_status["OBC"] == "online":
            log_entry["status"] = "relayed_to_OBC"
            self.data_flow_history.append(log_entry)
            
            if self.link_status["GCS"] == "online":
                gcs_log = {
                    "timestamp": datetime.now(),
                    "data_id": data_id,
                    "direction": "uplink",
                    "source": "OBC",
                    "target": "GCS",
                    "type": data_type,
                    "data": data,
                    "status": "delivered"
                }
                self.data_flow_history.append(gcs_log)
                self.current_flow = {
                    "path": ["FCU", "OBC", "GCS"],
                    "data_type": data_type,
                    "timestamp": timestamp
                }
                return True
            else:
                log_entry["status"] = "failed_GCS_offline"
                self.data_flow_history.append(log_entry)
                return False
        else:
            log_entry["status"] = "failed_link_down"
            self.data_flow_history.append(log_entry)
            return False
    
    def update_link_status(self, component, status):
        if component in self.link_status:
            old_status = self.link_status[component]
            self.link_status[component] = status
            log_entry = {
                "timestamp": datetime.now(),
                "type": "status_change",
                "component": component,
                "old_status": old_status,
                "new_status": status
            }
            self.data_flow_history.append(log_entry)
            return True
        return False
    
    def get_link_quality(self):
        quality = {}
        online_count = sum(1 for s in self.link_status.values() if s == "online")
        quality["overall"] = (online_count / 3) * 100
        
        quality["GCS_OBC"] = random.randint(85, 99) if self.link_status["GCS"] == "online" and self.link_status["OBC"] == "online" else 0
        quality["OBC_FCU"] = random.randint(80, 98) if self.link_status["OBC"] == "online" and self.link_status["FCU"] == "online" else 0
        
        return quality
    
    def get_recent_logs(self, count=20):
        return list(self.data_flow_history)[-count:]

# ========== GCJ-02 转 WGS-84 ==========
def gcj02_to_wgs84(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    PI = math.pi
    def transform_lat(lng, lat):
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * PI) + 20.0 * math.sin(2.0 * lng * PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * PI) + 40.0 * math.sin(lat / 3.0 * PI)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * PI) + 320 * math.sin(lat * PI / 30.0)) * 2.0 / 3.0
        return ret
    def transform_lng(lng, lat):
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * PI) + 20.0 * math.sin(2.0 * lng * PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * PI) + 40.0 * math.sin(lng / 3.0 * PI)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * PI) + 300.0 * math.sin(lng * PI / 30.0)) * 2.0 / 3.0
        return ret
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * PI)
    wgs_lat = lat - dlat
    wgs_lng = lng - dlng
    return wgs_lng, wgs_lat

# ========== 几何辅助函数 ==========
def segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
    def cross(ax, ay, bx, by):
        return ax*by - ay*bx
    def on_segment(px, py, qx, qy, rx, ry):
        return min(px, qx) <= rx <= max(px, qx) and min(py, qy) <= ry <= max(py, qy)
    o1 = cross(x2-x1, y2-y1, x3-x1, y3-y1)
    o2 = cross(x2-x1, y2-y1, x4-x1, y4-y1)
    o3 = cross(x4-x3, y4-y3, x1-x3, y1-y3)
    o4 = cross(x4-x3, y4-y3, x2-x3, y2-y3)
    if o1 == 0 and on_segment(x1, y1, x2, y2, x3, y3): return True
    if o2 == 0 and on_segment(x1, y1, x2, y2, x4, y4): return True
    if o3 == 0 and on_segment(x3, y3, x4, y4, x1, y1): return True
    if o4 == 0 and on_segment(x3, y3, x4, y4, x2, y2): return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)

def polygon_intersects_segment(poly_vertices, seg_start, seg_end):
    try:
        n = len(poly_vertices)
        if n < 3:
            return False
        for i in range(n):
            x1, y1 = poly_vertices[i]
            x2, y2 = poly_vertices[(i+1)%n]
            if segments_intersect(seg_start[0], seg_start[1], seg_end[0], seg_end[1], x1, y1, x2, y2):
                return True
        mid_x = (seg_start[0] + seg_end[0]) / 2
        mid_y = (seg_start[1] + seg_end[1]) / 2
        inside = False
        for i in range(n):
            x1, y1 = poly_vertices[i]
            x2, y2 = poly_vertices[(i+1)%n]
            if ((y1 > mid_y) != (y2 > mid_y)) and (mid_x < (x2 - x1) * (mid_y - y1) / (y2 - y1) + x1):
                inside = not inside
        return inside
    except:
        return False

def get_bounding_box(poly_vertices):
    xs = [v[0] for v in poly_vertices]
    ys = [v[1] for v in poly_vertices]
    return min(xs), min(ys), max(xs), max(ys)

def catmull_rom_spline(points, num_segments=30):
    if len(points) < 2:
        return points
    if len(points) == 2:
        return [points[0] + (points[1]-points[0]) * t for t in [i/num_segments for i in range(num_segments+1)]]
    result = []
    for i in range(len(points)-1):
        p0 = points[max(i-1, 0)]
        p1 = points[i]
        p2 = points[i+1]
        p3 = points[min(i+2, len(points)-1)]
        for t in [j/num_segments for j in range(num_segments)]:
            t2 = t*t
            t3 = t2*t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
            result.append((x, y))
    result.append(points[-1])
    return result

def get_expanded_rect_polygon(obs, safety_meters):
    try:
        vertices = obs["vertices"]
        minx, miny, maxx, maxy = get_bounding_box(vertices)
        center_lat = (miny + maxy) / 2.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(center_lat))
        expand_lon = safety_meters / meters_per_deg_lon if meters_per_deg_lon > 0 else safety_meters / 111320.0
        expand_lat = safety_meters / 111000.0
        minx -= expand_lon
        miny -= expand_lat
        maxx += expand_lon
        maxy += expand_lat
        return [(minx, miny), (minx, maxy), (maxx, maxy), (maxx, miny)]
    except Exception as e:
        return [(0, 0), (0, 0.001), (0.001, 0.001), (0.001, 0)]

def segment_collides_with_obstacle(obs, safety_meters, seg_start, seg_end):
    try:
        if not isinstance(obs, dict) or "vertices" not in obs:
            return False
        vertices = obs["vertices"]
        if not vertices or len(vertices) < 3:
            return False
        expanded = get_expanded_rect_polygon(obs, safety_meters)
        return polygon_intersects_segment(expanded, seg_start, seg_end)
    except Exception:
        return False

def detour_single(A, B, obs, safety_meters, side="auto"):
    expanded = get_expanded_rect_polygon(obs, safety_meters)
    rect_pts = expanded
    if side == "left":
        p1, p2 = rect_pts[0], rect_pts[1]
        if math.hypot(p1[0]-A[0], p1[1]-A[1]) > math.hypot(p2[0]-A[0], p2[1]-A[1]):
            p1, p2 = p2, p1
        return [A, p1, p2, B]
    elif side == "right":
        p1, p2 = rect_pts[3], rect_pts[2]
        if math.hypot(p1[0]-A[0], p1[1]-A[1]) > math.hypot(p2[0]-A[0], p2[1]-A[1]):
            p1, p2 = p2, p1
        return [A, p1, p2, B]
    else:
        paths = [
            ([A, rect_pts[0], rect_pts[1], B]),
            ([A, rect_pts[1], rect_pts[2], B]),
            ([A, rect_pts[2], rect_pts[3], B]),
            ([A, rect_pts[3], rect_pts[0], B]),
        ]
        def path_len(path):
            total = math.hypot(path[1][0]-path[0][0], path[1][1]-path[0][1])
            total += math.hypot(path[2][0]-path[1][0], path[2][1]-path[1][1])
            total += math.hypot(path[3][0]-path[2][0], path[3][1]-path[2][1])
            return total
        best = min(paths, key=path_len)
        return best

def sequential_detour(A, B, obstacles, flight_height, safety_meters, side="auto", max_iters=10):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    current_route = [A, B]
    for _ in range(max_iters):
        new_route = [current_route[0]]
        conflict = False
        for i in range(len(current_route)-1):
            seg_start = current_route[i]
            seg_end = current_route[i+1]
            target_obs = None
            for obs in relevant:
                if segment_collides_with_obstacle(obs, safety_meters, seg_start, seg_end):
                    target_obs = obs
                    break
            if target_obs is None:
                new_route.append(seg_end)
            else:
                conflict = True
                seg_detour = detour_single(seg_start, seg_end, target_obs, safety_meters, side)
                new_route.extend(seg_detour[1:])
        current_route = new_route
        if not conflict:
            ok = True
            for i in range(len(current_route)-1):
                for obs in relevant:
                    if segment_collides_with_obstacle(obs, safety_meters, current_route[i], current_route[i+1]):
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                return current_route
    return current_route

def simplify_route(route, obstacles, flight_height, safety_meters):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if len(route) <= 2:
        return route
    simplified = [route[0]]
    current_idx = 0
    while current_idx < len(route)-1:
        furthest = current_idx + 1
        for j in range(len(route)-1, current_idx, -1):
            safe = True
            for obs in relevant:
                if segment_collides_with_obstacle(obs, safety_meters, route[current_idx], route[j]):
                    safe = False
                    break
            if safe:
                furthest = j
                break
        simplified.append(route[furthest])
        current_idx = furthest
    return simplified

def safe_smooth_route(route_points, obstacles, flight_height, safety_meters):
    if len(route_points) <= 2:
        return route_points
    smooth = catmull_rom_spline(route_points, num_segments=30)
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    for i in range(len(smooth)-1):
        for obs in relevant:
            if segment_collides_with_obstacle(obs, safety_meters, smooth[i], smooth[i+1]):
                return route_points
    for pt in smooth:
        for obs in relevant:
            expanded = get_expanded_rect_polygon(obs, safety_meters)
            if polygon_intersects_segment(expanded, pt, pt):
                return route_points
    return smooth

def generate_detour_route(A, B, obstacles, flight_height, safety_meters, detour_side="auto", max_attempts=3):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if not relevant:
        return [A, B]
    for attempt in range(max_attempts):
        current_safety = safety_meters * (1 + attempt * 0.5)
        route = sequential_detour(A, B, relevant, flight_height, current_safety, detour_side, max_iters=10)
        ok = True
        for i in range(len(route)-1):
            for obs in relevant:
                if segment_collides_with_obstacle(obs, current_safety, route[i], route[i+1]):
                    ok = False
                    break
            if not ok:
                break
        if ok:
            simplified = simplify_route(route, obstacles, flight_height, current_safety)
            return safe_smooth_route(simplified, obstacles, flight_height, current_safety)
    return [A, B]

def optimal_detour_route(A, B, obstacles, flight_height, safety_meters, max_attempts=3):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if not relevant:
        return [A, B]
    for attempt in range(max_attempts):
        current_safety = safety_meters * (1 + attempt * 0.5)
        points = [A, B]
        for obs in relevant:
            for v in get_expanded_rect_polygon(obs, current_safety):
                points.append(v)
        unique = []
        for p in points:
            if not any(math.hypot(p[0]-q[0], p[1]-q[1]) < 1e-9 for q in unique):
                unique.append(p)
        points = unique
        n = len(points)
        graph = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                safe = True
                for obs in relevant:
                    if segment_collides_with_obstacle(obs, current_safety, points[i], points[j]):
                        safe = False
                        break
                if safe:
                    dist = math.hypot(points[j][0]-points[i][0], points[j][1]-points[i][1])
                    graph[i].append((j, dist))
                    graph[j].append((i, dist))
        start_idx = points.index(A)
        end_idx = points.index(B)
        dist = [float('inf')] * n
        prev = [-1] * n
        dist[start_idx] = 0
        pq = [(0, start_idx)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for v, w in graph[u]:
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))
        if dist[end_idx] != float('inf'):
            path_idx = []
            cur = end_idx
            while cur != -1:
                path_idx.append(cur)
                cur = prev[cur]
            path_idx.reverse()
            path_pts = [points[i] for i in path_idx]
            simplified = simplify_route(path_pts, obstacles, flight_height, current_safety)
            return safe_smooth_route(simplified, obstacles, flight_height, current_safety)
    return [A, B]

def haversine(lng1, lat1, lng2, lat2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lng2 - lng1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def interpolate_pos(p1, p2, speed, elapsed):
    d = haversine(p1[0], p1[1], p2[0], p2[1])
    if d < 0.1:
        return p2
    travel = speed * elapsed
    t = min(travel / d, 1.0)
    lng = p1[0] + (p2[0] - p1[0]) * t
    lat = p1[1] + (p2[1] - p1[1]) * t
    return (lng, lat)

# ========== 通信链路可视化组件 ==========
def render_communication_topology(comm_link):
    st.subheader("🌐 通信链路拓扑")
    link_quality = comm_link.get_link_quality()
    
    col1, col2, col3 = st.columns(3)
    with col1:
        status_color = "🟢" if comm_link.link_status["GCS"] == "online" else "🔴"
        st.markdown(f"""
        <div style="text-align: center; padding: 20px; border: 2px solid #ddd; border-radius: 10px;">
            <h3>{status_color} 地面站 (GCS)</h3>
            <p>状态: <b>{comm_link.link_status["GCS"].upper()}</b></p>
            <p>链路质量: {link_quality["GCS_OBC"]}%</p>
        </div>
        """, unsafe_allow_html=True)
    with col2:
        status_color = "🟢" if comm_link.link_status["OBC"] == "online" else "🔴"
        st.markdown(f"""
        <div style="text-align: center; padding: 20px; border: 2px solid #ddd; border-radius: 10px;">
            <h3>{status_color} 机载计算机 (OBC)</h3>
            <p>状态: <b>{comm_link.link_status["OBC"].upper()}</b></p>
            <p>链路质量: {link_quality["GCS_OBC"]}% / {link_quality["OBC_FCU"]}%</p>
        </div>
        """, unsafe_allow_html=True)
    with col3:
        status_color = "🟢" if comm_link.link_status["FCU"] == "online" else "🔴"
        st.markdown(f"""
        <div style="text-align: center; padding: 20px; border: 2px solid #ddd; border-radius: 10px;">
            <h3>{status_color} 飞行控制器 (FCU)</h3>
            <p>状态: <b>{comm_link.link_status["FCU"].upper()}</b></p>
            <p>链路质量: {link_quality["OBC_FCU"]}%</p>
        </div>
        """, unsafe_allow_html=True)
    
    st.markdown("---")
    st.subheader("📡 当前数据流")
    if comm_link.current_flow:
        flow = comm_link.current_flow
        if "path" in flow:
            if flow["path"] == ["GCS", "OBC", "FCU"]:
                st.info(f"⬇️ 下行命令流: GCS → OBC → FCU ({flow.get('command', '未知命令')})")
            elif flow["path"] == ["FCU", "OBC", "GCS"]:
                st.info(f"⬆️ 上行数据流: FCU → OBC → GCS ({flow.get('data_type', '未知数据')})")
    else:
        st.caption("等待数据传输...")

def render_communication_controls(comm_link):
    st.subheader("🎮 通信控制")
    col1, col2 = st.columns(2)
    with col1:
        st.markdown("**📤 发送命令 (GCS → OBC → FCU)**")
        command_type = st.selectbox("命令类型", ["起飞", "悬停", "返航", "设置航点", "降落", "紧急停止"], key="cmd_type")
        if st.button("🚀 发送命令", key="send_cmd"):
            success = comm_link.send_command("GCS", "FCU", command_type, {"timestamp": datetime.now().isoformat()})
            if success:
                st.success(f"命令 '{command_type}' 已发送")
                st.rerun()
            else:
                st.error("命令发送失败：链路中断")
    with col2:
        st.markdown("**📊 请求遥测数据 (FCU → OBC → GCS)**")
        data_type = st.selectbox("数据类型", ["位置信息", "姿态数据", "电池状态", "速度信息", "传感器数据"], key="data_type")
        if st.button("📡 请求数据", key="req_data"):
            mock_data = {
                "位置信息": {"lat": 32.2322, "lng": 118.7490, "alt": 50},
                "姿态数据": {"roll": 0.5, "pitch": 0.3, "yaw": 45.0},
                "电池状态": {"voltage": 22.8, "current": 15.2, "remaining": 85},
                "速度信息": {"vx": 8.5, "vy": 0, "vz": 0},
                "传感器数据": {"gps": "OK", "imu": "OK", "baro": "OK"}
            }
            success = comm_link.send_telemetry("FCU", "GCS", data_type, mock_data.get(data_type, {}))
            if success:
                st.success(f"遥测数据 '{data_type}' 已接收")
                st.rerun()
            else:
                st.error("数据请求失败：链路中断")
    
    st.markdown("---")
    st.markdown("**🔧 链路模拟控制**")
    col3, col4, col5 = st.columns(3)
    with col3:
        if st.button("📡 GCS 离线", key="gcs_offline"):
            comm_link.update_link_status("GCS", "offline")
            st.warning("GCS 已离线")
            st.rerun()
    with col4:
        if st.button("💻 OBC 离线", key="obc_offline"):
            comm_link.update_link_status("OBC", "offline")
            st.warning("OBC 已离线")
            st.rerun()
    with col5:
        if st.button("🛸 FCU 离线", key="fcu_offline"):
            comm_link.update_link_status("FCU", "offline")
            st.warning("FCU 已离线")
            st.rerun()
    if st.button("🔄 恢复所有链路", key="restore_links"):
        comm_link.update_link_status("GCS", "online")
        comm_link.update_link_status("OBC", "online")
        comm_link.update_link_status("FCU", "online")
        st.success("所有链路已恢复")
        st.rerun()

def render_communication_logs(comm_link):
    st.subheader("📋 通信日志")
    col1, col2 = st.columns(2)
    with col1:
        log_filter = st.selectbox("日志类型", ["全部", "命令下发", "数据上传", "状态变更"], key="log_filter")
    with col2:
        log_limit = st.slider("显示条数", 10, 100, 30, key="log_limit")
    
    logs = comm_link.get_recent_logs(log_limit)
    if logs:
        for log in reversed(logs):
            timestamp = log.get("timestamp", datetime.now())
            time_str = timestamp.strftime("%H:%M:%S.%f")[:-3] if hasattr(timestamp, 'strftime') else str(timestamp)
            if "direction" in log:
                direction_icon = "⬇️" if log["direction"] == "downlink" else "⬆️"
                if log["direction"] == "downlink":
                    msg = f"{direction_icon} [{time_str}] 命令 {log['command_id']}: {log['source']} → {log['target']} [{log['type']}]"
                else:
                    msg = f"{direction_icon} [{time_str}] 数据 {log['data_id']}: {log['source']} → {log['target']} [{log['type']}]"
                if log["status"] == "delivered":
                    st.success(msg)
                elif "failed" in log["status"]:
                    st.error(msg + f" - 失败: {log['status']}")
                else:
                    st.info(msg)
            elif "type" in log and log["type"] == "status_change":
                status_icon = "🔴" if log["new_status"] == "offline" else "🟢"
                msg = f"{status_icon} [{time_str}] {log['component']} 状态变更: {log['old_status']} → {log['new_status']}"
                if log["new_status"] == "offline":
                    st.error(msg)
                else:
                    st.success(msg)
    else:
        st.info("暂无通信日志")

# ========== Streamlit 页面配置 ==========
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

# 初始化 session state
if "app_version" not in st.session_state:
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    loaded = load_obstacles_from_file()
    st.session_state.obstacles = loaded if loaded else []
    st.session_state.default_obstacle_height = 30.0
    st.session_state.safety_distance = 3.0
    st.session_state.detour_route = None
    st.session_state.detour_side = "auto"
    st.session_state.flash_message = None
    st.session_state.app_version = "v37_comm_link"
    st.session_state.mission_waypoints = None
    st.session_state.mission_active = False
    st.session_state.mission_paused = False
    st.session_state.mission_start_time = 0.0
    st.session_state.current_waypoint_index = 0
    st.session_state.aircraft_position = None
    st.session_state.flight_speed = 8.5
    st.session_state.battery = 96.0
    st.session_state.stop_mission = False
    st.session_state.comm_link = CommunicationLink()

def show_flash():
    if "flash_message" in st.session_state and st.session_state.flash_message is not None:
        msg_type, msg_text = st.session_state.flash_message
        if msg_type == "success":
            st.success(msg_text)
        elif msg_type == "warning":
            st.warning(msg_text)
        elif msg_type == "error":
            st.error(msg_text)
        st.session_state.flash_message = None

# 侧边栏
st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控", "通信链路"], key="page_radio")
st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系设置", ["WGS-84", "GCJ-02"], index=0, key="coord_radio")
st.sidebar.info("✅ 卫星图底图：Esri World Imagery (WGS-84)")

# 简易的航线规划页面（简化版，确保能运行）
if page == "航线规划":
    st.header("🗺️ 航线规划")
    show_flash()
    
    st.info("请在右侧地图上绘制障碍物（多边形或矩形）")
    
    col1, col2 = st.columns([1, 2])
    with col1:
        lat_a = st.number_input("起点纬度", value=32.2322, format="%.6f")
        lon_a = st.number_input("起点经度", value=118.7490, format="%.6f")
        lat_b = st.number_input("终点纬度", value=32.2343, format="%.6f")
        lon_b = st.number_input("终点经度", value=118.7495, format="%.6f")
        flight_height = st.slider("飞行高度(m)", 0, 100, 50)
        safety_dist = st.number_input("安全距离(m)", value=3.0, step=1.0)
        
        if st.button("生成绕行路径"):
            A = (lon_a, lat_a)
            B = (lon_b, lat_b)
            route = generate_detour_route(A, B, st.session_state.obstacles, flight_height, safety_dist)
            st.session_state.detour_route = route
            st.session_state.mission_waypoints = route
            st.success(f"生成路径成功，共{len(route)}个航点")
    
    with col2:
        center = [lat_a, lon_a]
        m = folium.Map(location=center, zoom_start=17,
                      tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                      attr='Esri')
        
        # 画障碍物
        for idx, obs in enumerate(st.session_state.obstacles):
            if isinstance(obs, dict) and "vertices" in obs:
                folium.Polygon([[lat, lng] for lng, lat in obs["vertices"]],
                              color="red", weight=2, fill=True, fill_opacity=0.3,
                              popup=f"障碍物{idx+1}").add_to(m)
        
        # 画路径
        if st.session_state.detour_route:
            folium.PolyLine([[lat, lng] for lng, lat in st.session_state.detour_route],
                           color="blue", weight=3).add_to(m)
        
        folium.Marker([lat_a, lon_a], popup="起点", icon=folium.Icon(color='green')).add_to(m)
        folium.Marker([lat_b, lon_b], popup="终点", icon=folium.Icon(color='red')).add_to(m)
        
        draw = Draw(draw_options={"polygon": True, "rectangle": True})
        draw.add_to(m)
        
        output = st_folium(m, width=700, height=500, returned_objects=["last_active_drawing"])
        
        if output and output.get("last_active_drawing"):
            drawing = output["last_active_drawing"]
            geom_type = drawing.get("geometry", {}).get("type")
            coords = drawing.get("geometry", {}).get("coordinates")
            if geom_type == "Polygon" and coords:
                ring = coords[0]
                poly = [(lng, lat) for lng, lat in ring]
                new_obs = {"vertices": poly, "height": st.session_state.default_obstacle_height}
                st.session_state.obstacles.append(new_obs)
                save_obstacles_to_file(st.session_state.obstacles)
                st.rerun()

# 飞行监控页面
elif page == "飞行监控":
    st.header("✈️ 飞行监控")
    if st.session_state.mission_waypoints is None:
        st.warning("请先在航线规划页面生成路径")
    else:
        st.info("飞行监控功能正常")
        route = st.session_state.mission_waypoints
        st.write(f"路径航点数: {len(route)}")

# 通信链路页面
elif page == "通信链路":
    st.header("📡 通信链路")
    comm_link = st.session_state.comm_link
    render_communication_topology(comm_link)
    st.markdown("---")
    render_communication_controls(comm_link)
    st.markdown("---")
    render_communication_logs(comm_link)
