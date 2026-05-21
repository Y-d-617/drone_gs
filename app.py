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

def point_in_polygon(px, py, poly):
    """射线法判断点是否在多边形内部（包括边界）"""
    n = len(poly)
    inside = False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i+1)%n]
        # 检查点是否在边上
        if min(x1, x2) <= px <= max(x1, x2) and min(y1, y2) <= py <= max(y1, y2):
            if abs((y2-y1)*(px-x1) - (x2-x1)*(py-y1)) < 1e-12:
                return True
        if ((y1 > py) != (y2 > py)) and (px < (x2 - x1) * (py - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def polygon_intersects_segment(poly_vertices, seg_start, seg_end):
    """线段是否与多边形相交（包括穿过或端点在内部）"""
    # 先检查端点是否在多边形内
    if point_in_polygon(seg_start[0], seg_start[1], poly_vertices):
        return True
    if point_in_polygon(seg_end[0], seg_end[1], poly_vertices):
        return True
    # 检查线段与多边形的每条边是否相交
    n = len(poly_vertices)
    for i in range(n):
        x1, y1 = poly_vertices[i]
        x2, y2 = poly_vertices[(i+1)%n]
        if segments_intersect(seg_start[0], seg_start[1], seg_end[0], seg_end[1], x1, y1, x2, y2):
            return True
    return False

def get_polygon_centroid(poly):
    xs = [v[0] for v in poly]
    ys = [v[1] for v in poly]
    return sum(xs)/len(xs), sum(ys)/len(ys)

def offset_point(lng, lat, distance_m, angle_rad):
    """将点沿指定方向移动 distance_m 米（近似）"""
    meters_per_deg_lat = 111000.0
    meters_per_deg_lng = 111320.0 * math.cos(math.radians(lat))
    dlng = (distance_m * math.sin(angle_rad)) / meters_per_deg_lng
    dlat = (distance_m * math.cos(angle_rad)) / meters_per_deg_lat
    return (lng + dlng, lat + dlat)

# ========== 生成障碍物的安全偏移点 ==========
def get_offset_vertices(obs, safety_meters):
    """返回障碍物每个顶点向外偏移 safety_meters 后的点列表"""
    vertices = obs["vertices"]
    cx, cy = get_polygon_centroid(vertices)
    offset_pts = []
    for lng, lat in vertices:
        # 从中心指向顶点的方向角
        dx = lng - cx
        dy = lat - cy
        if dx == 0 and dy == 0:
            angle = 0
        else:
            angle = math.atan2(dy, dx)
        offset_pt = offset_point(lng, lat, safety_meters, angle)
        offset_pts.append(offset_pt)
    return offset_pts

# ========== 碰撞检测（使用原始多边形） ==========
def segment_collides_with_obstacle(obs, safety_meters, seg_start, seg_end):
    """检查线段是否与障碍物的原始多边形相交（安全距离在生成候选点时已考虑）"""
    # 注意：这里不再使用扩展矩形，直接用原始多边形判断碰撞
    # 如果线段与原始多边形相交，则视为碰撞（后面会用偏移点来绕开）
    return polygon_intersects_segment(obs["vertices"], seg_start, seg_end)

# ========== 最优路径搜索（改进版） ==========
def optimal_detour_route(A, B, obstacles, flight_height, safety_meters, max_attempts=3):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if not relevant:
        return [A, B]

    for attempt in range(max_attempts):
        current_safety = safety_meters * (1 + attempt * 0.5)
        # 收集候选点：起点、终点、每个障碍物的安全偏移顶点
        points = [A, B]
        for obs in relevant:
            for v in get_offset_vertices(obs, current_safety):
                points.append(v)
        # 去重
        unique = []
        for p in points:
            if not any(math.hypot(p[0]-q[0], p[1]-q[1]) < 1e-9 for q in unique):
                unique.append(p)
        points = unique

        n = len(points)
        # 构建邻接图：两点之间若线段不与任何障碍物（原始多边形）相交，则连通
        graph = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                safe = True
                for obs in relevant:
                    if polygon_intersects_segment(obs["vertices"], points[i], points[j]):
                        safe = False
                        break
                if safe:
                    dist = math.hypot(points[j][0]-points[i][0], points[j][1]-points[i][1])
                    graph[i].append((j, dist))
                    graph[j].append((i, dist))

        # Dijkstra 找最短路径
        try:
            start_idx = points.index(A)
            end_idx = points.index(B)
        except ValueError:
            st.error("起点或终点不在候选点集中")
            return [A, B]

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
            # 进一步简化和平滑
            simplified = simplify_route(path_pts, obstacles, flight_height, current_safety)
            return safe_smooth_route(simplified, obstacles, flight_height, current_safety)

    # 所有尝试失败
    st.error("❌ 所有尝试均失败，请增大安全距离或调整障碍物位置")
    return [A, B]  # 返回直线但保留错误提示

# ========== 以下为原有辅助函数（部分修改以适应新的碰撞逻辑） ==========
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
            if point_in_polygon(pt[0], pt[1], obs["vertices"]):
                return route_points
    return smooth

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

# ========== 通信日志生成函数 ==========
def generate_comm_log_entry():
    direction = random.choice(["downlink", "uplink"])
    if direction == "uplink":
        cmd_list = ["起飞指令", "悬停指令", "航点上传", "云台控制", "拍照指令"]
        content = random.choice(cmd_list)
        delays = {
            "gcs_obc": round(random.uniform(0.02, 0.15), 3),
            "obc_fcu": round(random.uniform(0.02, 0.1), 3)
        }
        success = random.random() > 0.05
        entry = {
            "time": datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "direction": "GCS → OBC → FCU",
            "content": content,
            "gcs_obc_delay": delays["gcs_obc"],
            "obc_fcu_delay": delays["obc_fcu"],
            "total_delay": round(delays["gcs_obc"] + delays["obc_fcu"], 3),
            "status": "✅ 成功" if success else "❌ 超时"
        }
    else:
        telem_list = ["GPS位置", "电池电压", "飞行速度", "高度", "姿态角"]
        content = random.choice(telem_list)
        delays = {
            "fcu_obc": round(random.uniform(0.02, 0.1), 3),
            "obc_gcs": round(random.uniform(0.02, 0.15), 3)
        }
        success = random.random() > 0.05
        entry = {
            "time": datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "direction": "FCU → OBC → GCS",
            "content": content,
            "fcu_obc_delay": delays["fcu_obc"],
            "obc_gcs_delay": delays["obc_gcs"],
            "total_delay": round(delays["fcu_obc"] + delays["obc_gcs"], 3),
            "status": "✅ 成功" if success else "❌ 超时"
        }
    st.session_state.comm_log.append(entry)
    if len(st.session_state.comm_log) > 100:
        st.session_state.comm_log.pop(0)
    return entry

# ========== Streamlit 页面配置 ==========
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

if "app_version" not in st.session_state:
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    loaded = load_obstacles_from_file()
    st.session_state.obstacles = loaded if loaded else []
    st.session_state.default_obstacle_height = 30.0
    st.session_state.safety_distance = 5.0          # 默认安全距离改为5米
    st.session_state.detour_route = None
    st.session_state.detour_side = "auto"
    st.session_state.flash_message = None
    st.session_state.app_version = "v37_robust"
    st.session_state.mission_waypoints = None
    st.session_state.mission_active = False
    st.session_state.mission_paused = False
    st.session_state.mission_start_time = 0.0
    st.session_state.current_waypoint_index = 0
    st.session_state.aircraft_position = None
    st.session_state.flight_speed = 8.5
    st.session_state.battery = 96.0
    st.session_state.stop_mission = False
    st.session_state.comm_log = []
else:
    # 数据格式升级
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
    st.header("🗺️ 航线规划 + 多障碍物可靠绕行 (最优路径)")
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

    # 不再需要左右侧选择，因为改用最优路径

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
        flight_height = st.slider("设定飞行高度 (m)", 0, 100, 30, key="flight_h")   # 默认30米，低于默认障碍物高度30米，确保能触发绕行

        if coord_mode == "GCJ-02":
            display_lon_a, display_lat_a = gcj02_to_wgs84(lon_a, lat_a)
            display_lon_b, display_lat_b = gcj02_to_wgs84(lon_b, lat_b)
            st.success("已自动将 GCJ-02 坐标转换为 WGS-84")
        else:
            display_lon_a, display_lat_a = lon_a, lat_a
            display_lon_b, display_lat_b = lon_b, lat_b
            st.info("直接使用 WGS-84 坐标")

        if st.button("✈️ 计算绕行路径", use_container_width=True):
            with st.spinner("正在计算最优绕行路径..."):
                A_wgs = (display_lon_a, display_lat_a)
                B_wgs = (display_lon_b, display_lat_b)
                route = optimal_detour_route(A_wgs, B_wgs,
                                             st.session_state.obstacles,
                                             flight_height,
                                             st.session_state.safety_distance)
                # 根据 route 长度决定成功与否
                relevant = [obs for obs in st.session_state.obstacles if flight_height < obs["height"]]
                if len(route) == 2 and relevant:
                    # 有障碍物但绕行失败（返回了直线）
                    st.session_state.flash_message = ("error", "❌ 绕行失败，无法找到安全路径，请增大安全距离或调整障碍物")
                    st.session_state.detour_route = None
                    st.session_state.mission_waypoints = None
                elif len(route) == 2:
                    # 没有相关障碍物，无需绕行
                    st.session_state.flash_message = ("success", "✅ 无冲突，无需绕行")
                    st.session_state.detour_route = None
                    st.session_state.mission_waypoints = [A_wgs, B_wgs]
                else:
                    st.session_state.flash_message = ("success", f"✅ 已生成绕行航线，共 {len(route)} 个航点")
                    st.session_state.detour_route = route
                    st.session_state.mission_waypoints = route
                st.rerun()

        if st.button("清除绕行航线", key="clear_route"):
            st.session_state.detour_route = None
            st.session_state.mission_waypoints = None
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
    generate_comm_log_entry()

    if st.session_state.mission_waypoints is None:
        st.warning("⚠️ 尚未规划航线，请先在“航线规划”页面生成绕行路径。")
        st.stop()

    waypoints = st.session_state.mission_waypoints
    route = waypoints

    # 飞行控制按钮、数据仪表、地图等（保持不变，略作整合）
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

    # 通信链路拓扑图
    st.subheader("📡 通信链路拓扑")
    if st.session_state.sim and st.session_state.history:
        avg_rtt, loss_rate = st.session_state.sim.get_summary(st.session_state.history)
        if loss_rate > 30 or avg_rtt > 1.0:
            status_color = "red"
            status_text = "异常"
        elif loss_rate > 10 or avg_rtt > 0.5:
            status_color = "orange"
            status_text = "警告"
        else:
            status_color = "green"
            status_text = "正常"
    else:
        status_color = "gray"
        status_text = "待测"

    graph = f"""
    digraph comm {{
        rankdir=LR;
        node [shape=box, style=filled, fontname="Arial"];
        GCS [label="GCS\\n地面站", fillcolor={status_color}, fontcolor=white];
        OBC [label="OBC\\n机载计算机", fillcolor={status_color}, fontcolor=white];
        FCU [label="FCU\\n飞控", fillcolor={status_color}, fontcolor=white];
        GCS -> OBC [label="CMD/遥测", color=blue];
        OBC -> FCU [label="MAVLink", color=blue];
        FCU -> OBC [label="遥测/状态", color=red];
        OBC -> GCS [label="遥测/日志", color=red];
        label="通信链路状态: {status_text}";
        fontsize=14;
    }}
    """
    try:
        st.graphviz_chart(graph, use_container_width=True)
    except Exception as e:
        st.warning("Graphviz 不可用，显示文本拓扑。")
        st.text("GCS --(CMD/遥测)--> OBC --(MAVLink)--> FCU")
        st.text("FCU --(遥测/状态)--> OBC --(遥测/日志)--> GCS")

    # 通信日志
    with st.expander("📜 通信日志 (最近50条)", expanded=False):
        if st.session_state.comm_log:
            df_log = pd.DataFrame(st.session_state.comm_log[-50:])
            cols_show = ["time", "direction", "content", "total_delay", "status"]
            st.dataframe(df_log[cols_show], use_container_width=True, hide_index=True)
        else:
            st.info("暂无通信记录")

    if st.session_state.mission_active and not st.session_state.mission_paused:
        time.sleep(0.5)
        st.rerun()
