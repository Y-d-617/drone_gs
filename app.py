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

# ========== 安全区域扩展（生成缓冲多边形） ==========
def get_expanded_rect_polygon(obs, safety_meters):
    """返回障碍物的安全扩展矩形四个顶点（逆时针），用于代替原障碍物进行碰撞检测"""
    minx, miny, maxx, maxy = get_bounding_box(obs["vertices"])
    center_lat = (miny + maxy) / 2.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(center_lat))
    expand_lon = safety_meters / meters_per_deg_lon
    expand_lat = safety_meters / 111000.0
    minx -= expand_lon
    miny -= expand_lat
    maxx += expand_lon
    maxy += expand_lat
    return [(minx, miny), (minx, maxy), (maxx, maxy), (maxx, miny)]

def segment_collides_with_obstacle(obs, safety_meters, seg_start, seg_end):
    """线段是否与障碍物的安全扩展区域相交"""
    expanded = get_expanded_rect_polygon(obs, safety_meters)
    return polygon_intersects_segment(expanded, seg_start, seg_end)

# ========== 顺序绕行函数（已升级为基于扩展矩形的碰撞检测） ==========
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

# ========== 路径简化（减少航点） ==========
def simplify_route(route, obstacles, flight_height, safety_meters):
    """贪心可见性简化：保留起点和终点，尽量跳过中间点"""
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if len(route) <= 2:
        return route
    simplified = [route[0]]
    current_idx = 0
    while current_idx < len(route)-1:
        # 寻找从当前点可以直接到达的最远点
        furthest = current_idx + 1
        for j in range(len(route)-1, current_idx, -1):
            seg_start = route[current_idx]
            seg_end = route[j]
            safe = True
            for obs in relevant:
                if segment_collides_with_obstacle(obs, safety_meters, seg_start, seg_end):
                    safe = False
                    break
            if safe:
                furthest = j
                break
        simplified.append(route[furthest])
        current_idx = furthest
    return simplified

# ========== 安全平滑（含扩展区域检查） ==========
def safe_smooth_route(route_points, obstacles, flight_height, safety_meters):
    if len(route_points) <= 2:
        return route_points
    smooth = catmull_rom_spline(route_points, num_segments=30)
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    # 检查平滑线段是否与扩展区域相交
    for i in range(len(smooth)-1):
        for obs in relevant:
            if segment_collides_with_obstacle(obs, safety_meters, smooth[i], smooth[i+1]):
                return route_points
    # 检查平滑点是否落入扩展矩形内部
    for pt in smooth:
        for obs in relevant:
            expanded = get_expanded_rect_polygon(obs, safety_meters)
            if polygon_intersects_segment(expanded, pt, pt):  # 点包含判断
                return route_points
    return smooth

# ========== 生成绕行路径（含简化） ==========
def generate_detour_route(A, B, obstacles, flight_height, safety_meters, detour_side="auto", max_attempts=3):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if not relevant:
        return [A, B]
    for attempt in range(max_attempts):
        current_safety = safety_meters * (1 + attempt * 0.5)
        route = sequential_detour(A, B, relevant, flight_height, current_safety, detour_side, max_iters=10)
        # 验证
        ok = True
        for i in range(len(route)-1):
            for obs in relevant:
                if segment_collides_with_obstacle(obs, current_safety, route[i], route[i+1]):
                    ok = False
                    break
            if not ok:
                break
        if ok:
            # 简化路径
            simplified = simplify_route(route, obstacles, flight_height, current_safety)
            return safe_smooth_route(simplified, obstacles, flight_height, current_safety)
    st.warning("⚠️ 无法找到完全避障路径，请增加安全距离或调整障碍物位置")
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
                p1 = points[i]
                p2 = points[j]
                safe = True
                for obs in relevant:
                    if segment_collides_with_obstacle(obs, current_safety, p1, p2):
                        safe = False
                        break
                if safe:
                    dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
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
    st.warning("⚠️ 最优路径搜索失败，请增加安全距离或调整障碍物")
    return [A, B]

# ========== 飞行模拟辅助函数 ==========
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

# ========== Streamlit 页面配置 ==========
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

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
    st.session_state.app_version = "v35_simplified"
    st.session_state.mission_waypoints = None
    st.session_state.mission_active = False
    st.session_state.mission_paused = False
    st.session_state.mission_start_time = 0.0
    st.session_state.current_waypoint_index = 0
    st.session_state.aircraft_position = None
    st.session_state.flight_speed = 8.5
    st.session_state.battery = 96.0
    st.session_state.stop_mission = False
else:
    if st.session_state.obstacles and isinstance(st.session_state.obstacles[0], list):
        new_obs = []
        for poly in st.session_state.obstacles:
            new_obs.append({"vertices": poly, "height": 30.0})
        st.session_state.obstacles = new_obs
        save_obstacles_to_file(st.session_state.obstacles)
    for key in ["mission_waypoints", "mission_active", "mission_paused", "mission_start_time",
                "current_waypoint_index", "aircraft_position", "flight_speed", "battery", "stop_mission"]:
        if key not in st.session_state:
            if key == "flight_speed":
                st.session_state.flight_speed = 8.5
            elif key == "battery":
                st.session_state.battery = 96.0
            else:
                st.session_state[key] = None if key not in ["mission_active", "mission_paused", "stop_mission"] else False

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

# ========== 侧边栏 ==========
st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控"], key="page_radio")
st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系设置", ["WGS-84", "GCJ-02"], index=0, key="coord_radio")
st.sidebar.info("✅ 卫星图底图：Esri World Imagery (WGS-84)\n若选择 GCJ-02，系统会自动转换为 WGS-84 匹配卫星图。")

if page == "航线规划":
    st.header("🗺️ 航线规划 + 多障碍物可靠绕行 (左侧/右侧/自动/最优)")
    show_flash()

    st.sidebar.subheader("🚧 障碍物默认高度")
    default_h = st.sidebar.number_input(
        "新绘制障碍物的默认高度 (米)", 
        min_value=0.0, max_value=200.0, 
        value=st.session_state.default_obstacle_height, step=5.0,
        key="default_height"
    )
    st.session_state.default_obstacle_height = default_h
    st.sidebar.divider()

    st.sidebar.subheader("🛡️ 安全距离 (米)")
    safety = st.sidebar.number_input(
        "绕行安全距离", 
        min_value=0.0, max_value=200.0, 
        value=st.session_state.safety_distance, step=5.0,
        help="绕行路径与障碍物的最小距离（若找不到路径会自动增加）",
        key="safety_dist"
    )
    st.session_state.safety_distance = safety
    st.sidebar.divider()

    st.sidebar.subheader("↪️ 全局绕行侧偏好（仅对“自动绕行”有效）")
    side_option = st.sidebar.selectbox(
        "偏好绕行侧",
        options=["auto", "left", "right"],
        index=["auto", "left", "right"].index(st.session_state.detour_side),
        format_func=lambda x: {"auto": "自动选择最短路径", "left": "强制从左侧绕过", "right": "强制从右侧绕过"}[x],
        key="side_select"
    )
    st.session_state.detour_side = side_option
    st.sidebar.divider()

    st.sidebar.subheader("📋 已添加的障碍物")
    if not st.session_state.obstacles:
        st.sidebar.write("暂无障碍物")
    else:
        for idx, obs in enumerate(st.session_state.obstacles):
            with st.sidebar.expander(f"障碍物 {idx+1} (高度: {obs['height']} m)"):
                new_height = st.number_input(
                    f"高度 (m)", min_value=0.0, max_value=200.0, value=obs['height'],
                    key=f"obs_height_{idx}", step=5.0
                )
                if new_height != obs['height']:
                    obs['height'] = new_height
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.rerun()
                if st.button(f"🗑️ 删除障碍物 {idx+1}", key=f"del_obs_{idx}"):
                    st.session_state.obstacles.pop(idx)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.session_state.detour_route = None
                    st.rerun()
                st.caption(f"顶点数: {len(obs['vertices'])}")
    st.sidebar.metric("障碍物总数", len(st.session_state.obstacles))
    st.sidebar.divider()
    col_save1, col_save2 = st.sidebar.columns(2)
    with col_save1:
        if st.button("💾 保存障碍物", key="save_btn"):
            if save_obstacles_to_file(st.session_state.obstacles):
                st.session_state.flash_message = ("success", "已保存")
                st.rerun()
    with col_save2:
        if st.button("📂 加载障碍物", key="load_btn"):
            loaded = load_obstacles_from_file()
            if loaded:
                st.session_state.obstacles = loaded
                st.session_state.flash_message = ("success", f"加载 {len(loaded)} 个")
                st.rerun()
            else:
                st.session_state.flash_message = ("warning", "无备份文件或文件损坏")
                st.rerun()
    if st.sidebar.button("🧹 清空所有障碍物", key="clear_all"):
        st.session_state.obstacles = []
        if os.path.exists(OBSTACLE_FILE):
            os.remove(OBSTACLE_FILE)
        st.session_state.detour_route = None
        st.session_state.flash_message = ("success", "已清空")
        st.rerun()
    if st.sidebar.button("🔄 重置应用", key="reset_all"):
        for key in list(st.session_state.keys()):
            del st.session_state[key]
        st.rerun()

    col1, col2 = st.columns([1, 2])
    with col1:
        st.subheader("📍 坐标输入")
        lat_a = st.number_input("起点 A 纬度", value=32.2322, format="%.6f", key="lat_a")
        lon_a = st.number_input("起点 A 经度", value=118.7490, format="%.6f", key="lon_a")
        lat_b = st.number_input("终点 B 纬度", value=32.2343, format="%.6f", key="lat_b")
        lon_b = st.number_input("终点 B 经度", value=118.7495, format="%.6f", key="lon_b")
        flight_height = st.slider("设定飞行高度 (m)", 0, 100, 50, key="flight_h")

        if coord_mode == "GCJ-02":
            display_lon_a, display_lat_a = gcj02_to_wgs84(lon_a, lat_a)
            display_lon_b, display_lat_b = gcj02_to_wgs84(lon_b, lat_b)
            st.success("已自动将 GCJ-02 坐标转换为 WGS-84")
        else:
            display_lon_a, display_lat_a = lon_a, lat_a
            display_lon_b, display_lat_b = lon_b, lat_b
            st.info("直接使用 WGS-84 坐标")

        def compute_and_set_route(route_func, **kwargs):
            A_wgs = (display_lon_a, display_lat_a)
            B_wgs = (display_lon_b, display_lat_b)
            try:
                route = route_func(A_wgs, B_wgs, **kwargs)
                if len(route) == 2:
                    st.session_state.flash_message = ("success", "✅ 无冲突，无需绕行")
                    st.session_state.detour_route = None
                    st.session_state.mission_waypoints = [A_wgs, B_wgs]
                else:
                    st.session_state.flash_message = ("success", f"✅ 已生成绕行航线，共 {len(route)} 个航点")
                    st.session_state.detour_route = route
                    st.session_state.mission_waypoints = route
                st.session_state.mission_start_point = A_wgs
                st.session_state.mission_end_point = B_wgs
            except Exception as e:
                st.session_state.flash_message = ("error", f"计算失败: {str(e)}")
                st.session_state.detour_route = None
            st.rerun()

        col_btn1, col_btn2, col_btn3, col_btn4 = st.columns(4)
        with col_btn1:
            if st.button("✈️ 自动绕行", key="btn_auto", use_container_width=True):
                with st.spinner("正在计算自动绕行路径..."):
                    compute_and_set_route(generate_detour_route,
                                          obstacles=st.session_state.obstacles,
                                          flight_height=flight_height,
                                          safety_meters=st.session_state.safety_distance,
                                          detour_side=st.session_state.detour_side)
        with col_btn2:
            if st.button("⬅️ 左侧绕行", key="btn_left", use_container_width=True):
                with st.spinner("正在计算左侧绕行路径..."):
                    compute_and_set_route(generate_detour_route,
                                          obstacles=st.session_state.obstacles,
                                          flight_height=flight_height,
                                          safety_meters=st.session_state.safety_distance,
                                          detour_side="left")
        with col_btn3:
            if st.button("➡️ 右侧绕行", key="btn_right", use_container_width=True):
                with st.spinner("正在计算右侧绕行路径..."):
                    compute_and_set_route(generate_detour_route,
                                          obstacles=st.session_state.obstacles,
                                          flight_height=flight_height,
                                          safety_meters=st.session_state.safety_distance,
                                          detour_side="right")
        with col_btn4:
            if st.button("🏆 最优路径", key="btn_optimal", use_container_width=True):
                with st.spinner("正在计算全局最优最短路径..."):
                    compute_and_set_route(optimal_detour_route,
                                          obstacles=st.session_state.obstacles,
                                          flight_height=flight_height,
                                          safety_meters=st.session_state.safety_distance)

        if st.button("清除绕行航线", key="clear_route"):
            st.session_state.detour_route = None
            st.rerun()

        if st.button("清除所有障碍物", key="clear_obs"):
            st.session_state.obstacles = []
            save_obstacles_to_file(st.session_state.obstacles)
            st.session_state.detour_route = None
            st.rerun()

    with col2:
        map_center = [display_lat_a, display_lon_a]
        m = folium.Map(
            location=map_center, zoom_start=17,
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri World Imagery',
        )
        folium.PolyLine(
            locations=[[display_lat_a, display_lon_a], [display_lat_b, display_lon_b]],
            color="yellow", weight=5, opacity=0.8, popup="原始航线"
        ).add_to(m)
        if st.session_state.get("detour_route"):
            detour_locs = [[lat, lng] for lng, lat in st.session_state.detour_route]
            folium.PolyLine(
                locations=detour_locs, color="blue", weight=4, opacity=0.9,
                popup="绕行航线"
            ).add_to(m)
            start_pt = st.session_state.detour_route[0]
            end_pt = st.session_state.detour_route[-1]
            folium.Marker([start_pt[1], start_pt[0]], popup="绕行起点", icon=folium.Icon(color='blue', icon='play')).add_to(m)
            folium.Marker([end_pt[1], end_pt[0]], popup="绕行终点", icon=folium.Icon(color='blue', icon='stop')).add_to(m)
        folium.Marker([display_lat_a, display_lon_a], popup=f"起点 A (高度:{flight_height}m)", icon=folium.Icon(color='red', icon='play')).add_to(m)
        folium.Marker([display_lat_b, display_lon_b], popup="终点 B", icon=folium.Icon(color='green', icon='stop')).add_to(m)
        for idx, obs in enumerate(st.session_state.obstacles):
            poly_folium = [[lat, lng] for lng, lat in obs["vertices"]]
            folium.Polygon(
                locations=poly_folium, color="red", weight=3, fill=True, fill_color="red", fill_opacity=0.3,
                popup=f"障碍物 {idx+1}\n高度: {obs['height']} m"
            ).add_to(m)
        draw = Draw(
            draw_options={"polyline": False, "rectangle": True, "circle": False, "marker": False, "circlemarker": False, "polygon": True},
            edit_options={"edit": True, "remove": True}
        )
        draw.add_to(m)
        all_lats = [display_lat_a, display_lat_b]
        all_lons = [display_lon_a, display_lon_b]
        if st.session_state.get("detour_route"):
            for lng, lat in st.session_state.detour_route:
                all_lons.append(lng)
                all_lats.append(lat)
        for obs in st.session_state.obstacles:
            for lng, lat in obs["vertices"]:
                all_lons.append(lng)
                all_lats.append(lat)
        if all_lats and all_lons:
            m.fit_bounds([[min(all_lats), min(all_lons)], [max(all_lats), max(all_lons)]])
        output = st_folium(m, width=800, height=500, returned_objects=["last_active_drawing"])
        if output and output.get("last_active_drawing"):
            drawing = output["last_active_drawing"]
            geom_type = drawing.get("geometry", {}).get("type")
            coords = drawing.get("geometry", {}).get("coordinates")
            if geom_type == "Polygon" and coords:
                ring = coords[0]
                poly_wgs84 = [(lng, lat) for lng, lat in ring]
                exists = any(obs["vertices"] == poly_wgs84 for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": poly_wgs84, "height": st.session_state.default_obstacle_height}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.session_state.flash_message = ("success", f"已添加障碍物（高度 {new_obs['height']} m）")
                    st.rerun()
            elif geom_type == "Rectangle" and coords:
                lng1, lat1 = coords[0]; lng2, lat2 = coords[1]
                rect = [(lng1, lat1), (lng2, lat1), (lng2, lat2), (lng1, lat2)]
                exists = any(obs["vertices"] == rect for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": rect, "height": st.session_state.default_obstacle_height}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.session_state.flash_message = ("success", "已添加矩形障碍物")
                    st.rerun()

elif page == "飞行监控":
    st.header("✈️ 飞行任务实时监控")
    show_flash()

    if st.session_state.mission_waypoints is None:
        st.warning("⚠️ 尚未规划航线，请先在“航线规划”页面生成绕行路径。")
        st.stop()

    waypoints = st.session_state.mission_waypoints
    route = waypoints

    col_ctrl1, col_ctrl2, col_ctrl3, col_ctrl4 = st.columns(4)
    with col_ctrl1:
        if st.button("▶️ 开始任务", disabled=st.session_state.mission_active, key="btn_start_mon"):
            st.session_state.mission_active = True
            st.session_state.mission_paused = False
            st.session_state.mission_start_time = time.time()
            st.session_state.current_waypoint_index = 0
            st.session_state.aircraft_position = route[0]
            st.session_state.battery = 96.0
            st.session_state.stop_mission = False
            st.rerun()
    with col_ctrl2:
        pause_label = "⏸️ 暂停" if not st.session_state.mission_paused else "▶️ 继续"
        if st.button(pause_label, disabled=not st.session_state.mission_active, key="btn_pause_mon"):
            st.session_state.mission_paused = not st.session_state.mission_paused
            st.rerun()
    with col_ctrl3:
        if st.button("⏹️ 停止", disabled=not st.session_state.mission_active, key="btn_stop_mon"):
            st.session_state.mission_active = False
            st.session_state.mission_paused = False
            st.session_state.stop_mission = True
            st.rerun()
    with col_ctrl4:
        if st.button("🔄 重置", key="btn_reset_mon"):
            st.session_state.mission_active = False
            st.session_state.mission_paused = False
            st.session_state.current_waypoint_index = 0
            st.session_state.aircraft_position = route[0]
            st.session_state.mission_start_time = 0.0
            st.session_state.battery = 96.0
            st.session_state.stop_mission = False
            st.rerun()

    speed = st.slider("设定飞行速度 (m/s)", 1.0, 20.0, st.session_state.flight_speed, 0.5, key="speed_slider_mon")
    st.session_state.flight_speed = speed

    if st.session_state.mission_active and not st.session_state.mission_paused:
        if st.session_state.current_waypoint_index < len(route) - 1:
            wp_start = route[st.session_state.current_waypoint_index]
            wp_end = route[st.session_state.current_waypoint_index + 1]
            now = time.time()
            if st.session_state.mission_start_time == 0.0:
                st.session_state.mission_start_time = now
            elapsed = now - st.session_state.mission_start_time
            new_pos = interpolate_pos(wp_start, wp_end, st.session_state.flight_speed, elapsed)
            st.session_state.aircraft_position = new_pos
            d = haversine(wp_start[0], wp_start[1], wp_end[0], wp_end[1])
            if d > 0 and haversine(wp_start[0], wp_start[1], new_pos[0], new_pos[1]) >= d * 0.9999:
                st.session_state.current_waypoint_index += 1
                st.session_state.mission_start_time = now
                if st.session_state.current_waypoint_index >= len(route) - 1:
                    st.session_state.aircraft_position = route[-1]
                    st.session_state.mission_active = False
                    st.session_state.flash_message = ("success", "✅ 飞行任务已完成！")
        else:
            st.session_state.aircraft_position = route[-1]
            st.session_state.mission_active = False
            st.session_state.flash_message = ("success", "✅ 飞行任务已完成！")
        st.session_state.battery = max(0.0, st.session_state.battery - 0.02)
    elif not st.session_state.mission_active and st.session_state.current_waypoint_index >= len(route)-1:
        st.session_state.aircraft_position = route[-1]

    pos = st.session_state.aircraft_position if st.session_state.aircraft_position else route[0]
    wp_idx = st.session_state.current_waypoint_index
    total_wp = len(route)
    dist_remaining = 0.0
    if wp_idx < total_wp - 1:
        dist_remaining += haversine(pos[0], pos[1], route[wp_idx+1][0], route[wp_idx+1][1])
    for i in range(wp_idx+1, total_wp-1):
        dist_remaining += haversine(route[i][0], route[i][1], route[i+1][0], route[i+1][1])
    elapsed_time = 0.0
    if st.session_state.mission_start_time > 0 and st.session_state.mission_active:
        elapsed_time = time.time() - st.session_state.mission_start_time + (wp_idx * 30)
    eta = dist_remaining / st.session_state.flight_speed if st.session_state.flight_speed > 0 else 0.0

    col_gauges, col_map = st.columns([1, 2])
    with col_gauges:
        st.subheader("📊 飞行数据")
        st.metric("当前航点", f"{wp_idx+1}/{total_wp}")
        st.metric("飞行速度", f"{st.session_state.flight_speed:.1f} m/s")
        mins, secs = divmod(int(elapsed_time), 60) if elapsed_time else (0,0)
        st.metric("已用时间", f"{mins:02d}:{secs:02d}")
        st.metric("剩余距离", f"{dist_remaining:.1f} m")
        mins_e, secs_e = divmod(int(eta), 60) if eta else (0,0)
        st.metric("预计到达", f"{mins_e:02d}:{secs_e:02d}")
        st.metric("电量模拟", f"{st.session_state.battery:.1f}%")
        st.subheader("📡 通信链路")
        cols = st.columns(3)
        cols[0].success("GCS 在线")
        cols[1].success("OBC 在线")
        cols[2].success("FCU 在线")
        if st.session_state.sim:
            packet = st.session_state.sim.generate_packet()
            st.session_state.history.append(packet)
            avg_rtt, loss_rate = st.session_state.sim.get_summary(st.session_state.history)
            st.caption(f"RTT: {packet['rtt']:.3f}s | 丢包率: {loss_rate:.1f}%")
            if packet['is_timeout']:
                st.error("通信超时！")

    with col_map:
        center_lat = pos[1]
        center_lng = pos[0]
        m2 = folium.Map(location=[center_lat, center_lng], zoom_start=17,
                        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                        attr='Esri World Imagery')
        folium.PolyLine(locations=[[lat, lng] for lng, lat in route], color="blue", weight=3).add_to(m2)
        for idx, obs in enumerate(st.session_state.obstacles):
            poly_folium = [[lat, lng] for lng, lat in obs["vertices"]]
            folium.Polygon(locations=poly_folium, color="red", weight=2, fill=True, fill_opacity=0.3).add_to(m2)
        folium.Marker([pos[1], pos[0]], icon=folium.Icon(color="blue", icon="plane", prefix="fa"),
                      popup=f"航点 {wp_idx+1}/{total_wp}").add_to(m2)
        folium.Marker([route[0][1], route[0][0]], icon=folium.Icon(color="green", icon="play")).add_to(m2)
        folium.Marker([route[-1][1], route[-1][0]], icon=folium.Icon(color="red", icon="stop")).add_to(m2)
        lats = [p[1] for p in route] + [pos[1]]
        lngs = [p[0] for p in route] + [pos[0]]
        for obs in st.session_state.obstacles:
            for lng, lat in obs["vertices"]:
                lats.append(lat)
                lngs.append(lng)
        m2.fit_bounds([[min(lats), min(lngs)], [max(lats), max(lngs)]])
        st_folium(m2, width=800, height=500)

    if st.session_state.mission_active and not st.session_state.mission_paused:
        time.sleep(0.5)
        st.rerun()
