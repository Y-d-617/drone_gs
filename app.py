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
import datetime

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

# ========== 现有顺序绕行函数 ==========
def detour_single(A, B, obs, safety_meters, side="auto"):
    minx, miny, maxx, maxy = get_bounding_box(obs["vertices"])
    expand = safety_meters / 111000.0
    minx -= expand
    miny -= expand
    maxx += expand
    maxy += expand
    rect_pts = [(minx, miny), (minx, maxy), (maxx, maxy), (maxx, miny)]
    
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
    current_route = [A, B]
    for _ in range(max_iters):
        new_route = [current_route[0]]
        conflict = False
        for i in range(len(current_route)-1):
            seg_start = current_route[i]
            seg_end = current_route[i+1]
            target_obs = None
            for obs in obstacles:
                if flight_height < obs["height"] and polygon_intersects_segment(obs["vertices"], seg_start, seg_end):
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
                for obs in obstacles:
                    if flight_height < obs["height"] and polygon_intersects_segment(obs["vertices"], current_route[i], current_route[i+1]):
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                return current_route
    return current_route

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
                if polygon_intersects_segment(obs["vertices"], route[i], route[i+1]):
                    ok = False
                    break
            if not ok:
                break
        if ok:
            if len(route) > 2:
                return catmull_rom_spline(route, num_segments=30)
            else:
                return route
    st.warning("⚠️ 无法找到完全避障路径，请增加安全距离或调整障碍物位置")
    return [A, B]

# ========== 最优路径基于 Dijkstra ==========
def optimal_detour_route(A, B, obstacles, flight_height, safety_meters, max_attempts=3):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if not relevant:
        return [A, B]
    for attempt in range(max_attempts):
        current_safety = safety_meters * (1 + attempt * 0.5)
        expand = current_safety / 111000.0
        points = [A, B]
        for obs in relevant:
            minx, miny, maxx, maxy = get_bounding_box(obs["vertices"])
            minx -= expand
            miny -= expand
            maxx += expand
            maxy += expand
            points.extend([(minx, miny), (minx, maxy), (maxx, maxy), (maxx, miny)])
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
                    if polygon_intersects_segment(obs["vertices"], p1, p2):
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
        visited = [False] * n
        for _ in range(n):
            u = -1
            min_d = float('inf')
            for i in range(n):
                if not visited[i] and dist[i] < min_d:
                    min_d = dist[i]
                    u = i
            if u == -1:
                break
            visited[u] = True
            for v, w in graph[u]:
                if not visited[v] and dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
        if dist[end_idx] != float('inf'):
            path_idx = []
            cur = end_idx
            while cur != -1:
                path_idx.append(cur)
                cur = prev[cur]
            path_idx.reverse()
            path_pts = [points[i] for i in path_idx]
            if len(path_pts) > 2:
                return catmull_rom_spline(path_pts, num_segments=30)
            else:
                return path_pts
    st.warning("⚠️ 最优路径搜索失败，请增加安全距离或调整障碍物")
    return [A, B]

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
    # 飞行监控相关状态
    st.session_state.flight_route = None
    st.session_state.flight_current_index = 0
    st.session_state.flight_active = False
    st.session_state.flight_paused = False
    st.session_state.flight_start_time = None
    st.session_state.flight_speed = 10.0
    st.session_state.flight_battery = 100.0
    st.session_state.flight_last_update = None
    st.session_state.flight_travelled = 0.0
    st.session_state.app_version = "v34_enhanced_monitor"
else:
    if st.session_state.obstacles and isinstance(st.session_state.obstacles[0], list):
        new_obs = []
        for poly in st.session_state.obstacles:
            new_obs.append({"vertices": poly, "height": 30.0})
        st.session_state.obstacles = new_obs
        save_obstacles_to_file(st.session_state.obstacles)

st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控"], key="page_radio")
st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系设置", ["WGS-84", "GCJ-02"], index=0, key="coord_radio")
st.sidebar.info("✅ 卫星图底图：Esri World Imagery (WGS-84)")

# ========== 页面1：航线规划 ==========
if page == "航线规划":
    st.header("🗺️ 航线规划 + 多障碍物可靠绕行 (左侧/右侧/自动/最优)")

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

    st.sidebar.subheader("↪️ 全局绕行侧偏好（仅对下方“自动绕行”有效）")
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
                    st.session_state.flight_route = None
                    st.rerun()
                st.caption(f"顶点数: {len(obs['vertices'])}")
    st.sidebar.metric("障碍物总数", len(st.session_state.obstacles))
    st.sidebar.divider()
    col_save1, col_save2 = st.sidebar.columns(2)
    with col_save1:
        if st.button("💾 保存障碍物", key="save_btn"):
            if save_obstacles_to_file(st.session_state.obstacles):
                st.sidebar.success("已保存")
    with col_save2:
        if st.button("📂 加载障碍物", key="load_btn"):
            loaded = load_obstacles_from_file()
            if loaded:
                st.session_state.obstacles = loaded
                st.sidebar.success(f"加载 {len(loaded)} 个")
                st.rerun()
            else:
                st.sidebar.warning("无备份文件或文件损坏")
    if st.sidebar.button("🧹 清空所有障碍物", key="clear_all"):
        st.session_state.obstacles = []
        if os.path.exists(OBSTACLE_FILE):
            os.remove(OBSTACLE_FILE)
        st.session_state.detour_route = None
        st.session_state.flight_route = None
        st.sidebar.success("已清空")
        st.rerun()
    if st.sidebar.button("🔄 重置应用", key="reset_all"):
        st.session_state.obstacles = []
        if os.path.exists(OBSTACLE_FILE):
            os.remove(OBSTACLE_FILE)
        st.session_state.detour_route = None
        st.session_state.flight_route = None
        st.session_state.history = []
        st.session_state.sim = HeartbeatSimulator()
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

        # 生成航线按钮
        col_btn1, col_btn2, col_btn3, col_btn4 = st.columns(4)
        with col_btn1:
            if st.button("✈️ 自动绕行", key="btn_auto", use_container_width=True):
                with st.spinner("正在计算自动绕行路径..."):
                    A_wgs = (display_lon_a, display_lat_a)
                    B_wgs = (display_lon_b, display_lat_b)
                    route = generate_detour_route(
                        A_wgs, B_wgs,
                        st.session_state.obstacles,
                        flight_height,
                        st.session_state.safety_distance,
                        detour_side=st.session_state.detour_side
                    )
                    if len(route) == 2:
                        st.success("✅ 无冲突，无需绕行")
                        st.session_state.detour_route = None
                        st.session_state.flight_route = None
                    else:
                        st.success(f"✅ 已生成自动绕行航线，共 {len(route)} 个航点")
                        st.session_state.detour_route = route
                        st.session_state.flight_route = route
                    st.rerun()
        with col_btn2:
            if st.button("⬅️ 左侧绕行", key="btn_left", use_container_width=True):
                with st.spinner("正在计算左侧绕行路径..."):
                    A_wgs = (display_lon_a, display_lat_a)
                    B_wgs = (display_lon_b, display_lat_b)
                    route = generate_detour_route(
                        A_wgs, B_wgs,
                        st.session_state.obstacles,
                        flight_height,
                        st.session_state.safety_distance,
                        detour_side="left"
                    )
                    if len(route) == 2:
                        st.success("✅ 无冲突，无需绕行")
                        st.session_state.detour_route = None
                        st.session_state.flight_route = None
                    else:
                        st.success(f"✅ 已生成左侧绕行航线，共 {len(route)} 个航点")
                        st.session_state.detour_route = route
                        st.session_state.flight_route = route
                    st.rerun()
        with col_btn3:
            if st.button("➡️ 右侧绕行", key="btn_right", use_container_width=True):
                with st.spinner("正在计算右侧绕行路径..."):
                    A_wgs = (display_lon_a, display_lat_a)
                    B_wgs = (display_lon_b, display_lat_b)
                    route = generate_detour_route(
                        A_wgs, B_wgs,
                        st.session_state.obstacles,
                        flight_height,
                        st.session_state.safety_distance,
                        detour_side="right"
                    )
                    if len(route) == 2:
                        st.success("✅ 无冲突，无需绕行")
                        st.session_state.detour_route = None
                        st.session_state.flight_route = None
                    else:
                        st.success(f"✅ 已生成右侧绕行航线，共 {len(route)} 个航点")
                        st.session_state.detour_route = route
                        st.session_state.flight_route = route
                    st.rerun()
        with col_btn4:
            if st.button("🏆 最优路径", key="btn_optimal", use_container_width=True):
                with st.spinner("正在计算全局最优最短路径..."):
                    A_wgs = (display_lon_a, display_lat_a)
                    B_wgs = (display_lon_b, display_lat_b)
                    route = optimal_detour_route(
                        A_wgs, B_wgs,
                        st.session_state.obstacles,
                        flight_height,
                        st.session_state.safety_distance
                    )
                    if len(route) == 2:
                        st.success("✅ 无冲突，无需绕行")
                        st.session_state.detour_route = None
                        st.session_state.flight_route = None
                    else:
                        st.success(f"✅ 已生成最优路径航线，共 {len(route)} 个航点")
                        st.session_state.detour_route = route
                        st.session_state.flight_route = route
                    st.rerun()

        if st.button("清除绕行航线", key="clear_route"):
            st.session_state.detour_route = None
            st.session_state.flight_route = None
            st.rerun()

        if st.button("清除所有障碍物", key="clear_obs"):
            st.session_state.obstacles = []
            save_obstacles_to_file(st.session_state.obstacles)
            st.session_state.detour_route = None
            st.session_state.flight_route = None
            st.rerun()

    with col2:
        map_center = [display_lat_a, display_lon_a]
        m = folium.Map(
            location=map_center, zoom_start=17,
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri World Imagery',
        )
        # 原始航线
        folium.PolyLine(
            locations=[[display_lat_a, display_lon_a], [display_lat_b, display_lon_b]],
            color="yellow", weight=5, opacity=0.8, popup="原始航线"
        ).add_to(m)
        # 绕行航线
        if st.session_state.get("detour_route"):
            detour_locs = [[lat, lng] for lng, lat in st.session_state.detour_route]
            folium.PolyLine(
                locations=detour_locs, color="blue", weight=4, opacity=0.9,
                popup="规划航线"
            ).add_to(m)
            start_pt = st.session_state.detour_route[0]
            end_pt = st.session_state.detour_route[-1]
            folium.Marker([start_pt[1], start_pt[0]], popup="起点", icon=folium.Icon(color='blue', icon='play')).add_to(m)
            folium.Marker([end_pt[1], end_pt[0]], popup="终点", icon=folium.Icon(color='blue', icon='stop')).add_to(m)
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
                    st.success(f"已添加障碍物（高度 {new_obs['height']} m）")
                    st.rerun()
            elif geom_type == "Rectangle" and coords:
                lng1, lat1 = coords[0]; lng2, lat2 = coords[1]
                rect = [(lng1, lat1), (lng2, lat1), (lng2, lat2), (lng1, lat2)]
                exists = any(obs["vertices"] == rect for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": rect, "height": st.session_state.default_obstacle_height}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.success("已添加矩形障碍物")
                    st.rerun()

# ========== 页面2：飞行监控（增强版）==========
elif page == "飞行监控":
    st.header("✈️ 飞行实时画面 - 任务执行监控")

    if st.session_state.flight_route is None or len(st.session_state.flight_route) < 2:
        st.warning("⚠️ 尚未规划航线，请先到“航线规划”页面生成一条绕行航线。")
        st.stop()

    route_points = st.session_state.flight_route
    total_points = len(route_points)
    total_distance = 0.0
    for i in range(total_points-1):
        total_distance += math.hypot(route_points[i+1][0]-route_points[i][0], route_points[i+1][1]-route_points[i][1]) * 111000.0

    # 飞行控制栏
    col1, col2, col3, col4, col5 = st.columns(5)
    with col1:
        if st.button("▶️ 开始任务", use_container_width=True):
            if not st.session_state.flight_active and st.session_state.flight_current_index < total_points-1:
                st.session_state.flight_active = True
                st.session_state.flight_paused = False
                if st.session_state.flight_start_time is None:
                    st.session_state.flight_start_time = time.time()
                    st.session_state.flight_last_update = time.time()
                    st.session_state.flight_travelled = 0.0
                st.rerun()
    with col2:
        if st.button("⏸️ 暂停", use_container_width=True):
            if st.session_state.flight_active and not st.session_state.flight_paused:
                st.session_state.flight_paused = True
                st.session_state.flight_active = False
                st.rerun()
    with col3:
        if st.button("⏹️ 停止", use_container_width=True):
            st.session_state.flight_active = False
            st.session_state.flight_paused = False
            st.session_state.flight_current_index = 0
            st.session_state.flight_start_time = None
            st.session_state.flight_travelled = 0.0
            st.session_state.flight_battery = 100.0
            st.session_state.current_position = route_points[0]
            st.rerun()
    with col4:
        if st.button("🔄 重置", use_container_width=True):
            st.session_state.flight_active = False
            st.session_state.flight_paused = False
            st.session_state.flight_current_index = 0
            st.session_state.flight_start_time = None
            st.session_state.flight_travelled = 0.0
            st.session_state.flight_battery = 100.0
            st.session_state.current_position = route_points[0]
            st.rerun()
    with col5:
        speed = st.number_input("速度 (m/s)", min_value=1.0, max_value=30.0, value=st.session_state.flight_speed, step=1.0, key="speed_input")
        st.session_state.flight_speed = speed

    # 初始化当前飞行位置
    if "current_position" not in st.session_state or st.session_state.current_position is None:
        st.session_state.current_position = route_points[0]
        st.session_state.flight_current_index = 0
        st.session_state.flight_travelled = 0.0
        st.session_state.flight_battery = 100.0

    # 实时更新飞行位置
    if st.session_state.flight_active and not st.session_state.flight_paused and st.session_state.flight_current_index < total_points-1:
        now = time.time()
        if st.session_state.flight_last_update is None:
            st.session_state.flight_last_update = now
        dt = now - st.session_state.flight_last_update
        if dt > 0:
            move_dist = st.session_state.flight_speed * dt
            remaining_in_segment = math.hypot(route_points[st.session_state.flight_current_index+1][0] - route_points[st.session_state.flight_current_index][0],
                                              route_points[st.session_state.flight_current_index+1][1] - route_points[st.session_state.flight_current_index][1]) * 111000.0
            while move_dist > remaining_in_segment and st.session_state.flight_current_index < total_points-1:
                move_dist -= remaining_in_segment
                st.session_state.flight_travelled += remaining_in_segment
                st.session_state.flight_current_index += 1
                if st.session_state.flight_current_index < total_points-1:
                    remaining_in_segment = math.hypot(route_points[st.session_state.flight_current_index+1][0] - route_points[st.session_state.flight_current_index][0],
                                                      route_points[st.session_state.flight_current_index+1][1] - route_points[st.session_state.flight_current_index][1]) * 111000.0
                else:
                    break
            if st.session_state.flight_current_index < total_points-1:
                ratio = move_dist / remaining_in_segment if remaining_in_segment > 0 else 0
                current_lng = route_points[st.session_state.flight_current_index][0] + ratio * (route_points[st.session_state.flight_current_index+1][0] - route_points[st.session_state.flight_current_index][0])
                current_lat = route_points[st.session_state.flight_current_index][1] + ratio * (route_points[st.session_state.flight_current_index+1][1] - route_points[st.session_state.flight_current_index][1])
                st.session_state.current_position = (current_lng, current_lat)
                st.session_state.flight_travelled += move_dist
            else:
                st.session_state.flight_active = False
                st.session_state.current_position = route_points[-1]
                st.success("🎉 任务完成！")
            st.session_state.flight_last_update = now
            # 模拟电量消耗（每飞行50米消耗1%）
            battery_consumed = move_dist / 50.0
            st.session_state.flight_battery = max(0, st.session_state.flight_battery - battery_consumed)

    # 计算显示指标
    current_index = st.session_state.flight_current_index
    travelled = st.session_state.flight_travelled
    remaining_distance = max(0.0, total_distance - travelled)
    progress = travelled / total_distance if total_distance > 0 else 1.0
    eta_seconds = remaining_distance / st.session_state.flight_speed if st.session_state.flight_speed > 0 else 0
    eta_str = str(datetime.timedelta(seconds=int(eta_seconds))) if eta_seconds < 86400 else ">1天"
    # 已用时间（从开始任务算起，含暂停）
    if st.session_state.flight_start_time is not None:
        elapsed_seconds = time.time() - st.session_state.flight_start_time
    else:
        elapsed_seconds = 0
    elapsed_str = str(datetime.timedelta(seconds=int(elapsed_seconds)))

    # 仪表盘区域
    st.markdown("---")
    metric_col1, metric_col2, metric_col3, metric_col4, metric_col5 = st.columns(5)
    with metric_col1:
        st.metric("当前航点", f"{current_index+1} / {total_points}")
        st.write(f"任务进度: {progress*100:.1f}%")
    with metric_col2:
        st.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
    with metric_col3:
        st.metric("已用时间", elapsed_str)
    with metric_col4:
        st.metric("剩余距离", f"{remaining_distance:.0f} m")
    with metric_col5:
        st.metric("预计到达", eta_str)
    st.markdown("---")
    col_batt1, col_batt2 = st.columns([1, 3])
    with col_batt1:
        st.metric("电量模拟", f"{st.session_state.flight_battery:.1f}%")
    with col_batt2:
        st.progress(st.session_state.flight_battery/100.0)

    # 通信链路拓扑
    st.subheader("通信链路拓扑与数据流")
    comm_col1, comm_col2, comm_col3 = st.columns(3)
    with comm_col1:
        st.success("GCS 在线")
        st.caption("地面站\n192.168.1.100")
    with comm_col2:
        st.success("OBC 在线")
        st.caption("机载计算机\nRaspberry Pi 4")
    with comm_col3:
        st.success("FCU 在线")
        st.caption("飞控\nPX4 / ArduPilot")
    st.info("MAVLink 已连接")

    # 实时飞行地图
    st.subheader("实时飞行地图")
    center_lat = st.session_state.current_position[1]
    center_lng = st.session_state.current_position[0]
    m = folium.Map(location=[center_lat, center_lng], zoom_start=17, tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', attr='Esri World Imagery')
    folium.PolyLine(locations=[[lat, lng] for lng, lat in route_points], color="blue", weight=4, opacity=0.8, popup="规划航线").add_to(m)
    # 已飞轨迹
    if current_index > 0:
        flown_points = route_points[:current_index+1]
        if st.session_state.flight_active and current_index < total_points-1:
            flown_points.append(st.session_state.current_position)
        folium.PolyLine(locations=[[lat, lng] for lng, lat in flown_points], color="green", weight=3, opacity=0.7, popup="已飞航迹").add_to(m)
    folium.Marker([st.session_state.current_position[1], st.session_state.current_position[0]], popup="无人机", icon=folium.Icon(color='darkblue', icon='plane', prefix='fa')).add_to(m)
    start_pt = route_points[0]
    end_pt = route_points[-1]
    folium.Marker([start_pt[1], start_pt[0]], popup="起点", icon=folium.Icon(color='green', icon='play')).add_to(m)
    folium.Marker([end_pt[1], end_pt[0]], popup="终点", icon=folium.Icon(color='red', icon='stop')).add_to(m)
    st_folium(m, width=800, height=500, returned_objects=[])

    # 可选：保留心跳包监控（可折叠）
    with st.expander("通讯延迟监控 (RTT)"):
        placeholder = st.empty()
        if st.button("开始接收实时数据", key="monitor_start"):
            for _ in range(50):
                packet = st.session_state.sim.generate_packet()
                st.session_state.history.append(packet)
                plot_df = pd.DataFrame(st.session_state.history[-20:])
                with placeholder.container():
                    avg_rtt, loss_rate = st.session_state.sim.get_summary(st.session_state.history)
                    col_r1, col_r2, col_r3 = st.columns(3)
                    col_r1.metric("实时 RTT", f"{packet['rtt']:.3f}s", delta=packet['status'], delta_color="inverse")
                    col_r2.metric("平均 RTT", f"{avg_rtt:.3f}s")
                    col_r3.metric("累计丢包率", f"{loss_rate:.1f}%")
                    st.line_chart(plot_df.set_index("time")["rtt"])
                    if packet['is_timeout']:
                        st.error(f"警报：北京时间 {packet['time']} 发生通讯超时！")
                time.sleep(0.4)
        else:
            st.info("点击按钮可查看心跳包模拟数据（不影响飞行模拟）")
