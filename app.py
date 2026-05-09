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

# ========== 单个障碍物平滑绕行 ==========
def smooth_detour_segment(A, B, obs, safety_meters, side="auto"):
    minx, miny, maxx, maxy = get_bounding_box(obs["vertices"])
    expand = safety_meters / 111000.0
    minx -= expand
    miny -= expand
    maxx += expand
    maxy += expand
    rect_pts = [(minx, miny), (minx, maxy), (maxx, maxy), (maxx, miny)]
    if side == "left":
        p1, p2 = rect_pts[0], rect_pts[1]
    elif side == "right":
        p1, p2 = rect_pts[3], rect_pts[2]
    else:
        # 自动模式：选择最短的相邻边
        paths = [
            ([A, rect_pts[0], rect_pts[1], B]),
            ([A, rect_pts[1], rect_pts[2], B]),
            ([A, rect_pts[2], rect_pts[3], B]),
            ([A, rect_pts[3], rect_pts[0], B]),
        ]
        best = min(paths, key=lambda p: math.hypot(p[1][0]-p[0][0], p[1][1]-p[0][1]) +
                                        math.hypot(p[2][0]-p[1][0], p[2][1]-p[1][1]) +
                                        math.hypot(p[3][0]-p[2][0], p[3][1]-p[2][1]))
        return catmull_rom_spline(best, num_segments=30)
    # 按距离A排序
    if math.hypot(p1[0]-A[0], p1[1]-A[1]) > math.hypot(p2[0]-A[0], p2[1]-A[1]):
        p1, p2 = p2, p1
    control = [A, p1, p2, B]
    return catmull_rom_spline(control, num_segments=30)

def generate_route(A, B, obstacles, flight_height, safety_meters, side="auto", max_iters=10):
    relevant = [obs for obs in obstacles if flight_height < obs["height"]]
    if not relevant:
        return [A, B]
    current = [A, B]
    for _ in range(max_iters):
        new_route = [current[0]]
        conflict = False
        for i in range(len(current)-1):
            s_start = current[i]
            s_end = current[i+1]
            target = None
            for obs in relevant:
                if polygon_intersects_segment(obs["vertices"], s_start, s_end):
                    target = obs
                    break
            if target is None:
                new_route.append(s_end)
            else:
                conflict = True
                seg = smooth_detour_segment(s_start, s_end, target, safety_meters, side)
                new_route.extend(seg[1:])
        current = new_route
        if not conflict:
            return current
    return current

# ========== Streamlit 页面 ==========
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

if "app_version" not in st.session_state:
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    loaded = load_obstacles_from_file()
    st.session_state.obstacles = loaded if loaded else []
    st.session_state.default_height = 30.0
    st.session_state.safety = 3.0
    st.session_state.route = None
    st.session_state.side = "auto"
    # 飞行监控状态
    st.session_state.flight_route = None
    st.session_state.flight_idx = 0
    st.session_state.flight_active = False
    st.session_state.flight_paused = False
    st.session_state.flight_start = None
    st.session_state.flight_speed = 10.0
    st.session_state.flight_battery = 100.0
    st.session_state.flight_last_update = None
    st.session_state.flight_travelled = 0.0
    st.session_state.app_version = "v38_final"
else:
    if st.session_state.obstacles and isinstance(st.session_state.obstacles[0], list):
        new_obs = []
        for poly in st.session_state.obstacles:
            new_obs.append({"vertices": poly, "height": 30.0})
        st.session_state.obstacles = new_obs
        save_obstacles_to_file(st.session_state.obstacles)

st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("页面", ["航线规划", "飞行监控"], key="page")
st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系", ["WGS-84", "GCJ-02"], index=0, key="coord")
st.sidebar.info("卫星图底图: Esri World Imagery (WGS-84)")

# ========== 航线规划页面 ==========
if page == "航线规划":
    st.header("🗺️ 航线规划 + 障碍物圈选")

    st.sidebar.subheader("障碍物默认高度")
    default_h = st.sidebar.number_input("米", 0.0, 200.0, st.session_state.default_height, 5.0, key="dh")
    st.session_state.default_height = default_h
    st.sidebar.divider()

    st.sidebar.subheader("安全距离 (米)")
    safety = st.sidebar.number_input("绕行安全距离", 0.0, 200.0, st.session_state.safety, 5.0, key="sf")
    st.session_state.safety = safety
    st.sidebar.divider()

    st.sidebar.subheader("绕行侧")
    side_opt = st.sidebar.selectbox("偏好", ["auto", "left", "right"], index=["auto","left","right"].index(st.session_state.side), key="side")
    st.session_state.side = side_opt
    st.sidebar.divider()

    st.sidebar.subheader("已添加的障碍物")
    if not st.session_state.obstacles:
        st.sidebar.write("暂无")
    else:
        for idx, obs in enumerate(st.session_state.obstacles):
            with st.sidebar.expander(f"障碍物 {idx+1} (高度: {obs['height']} m)"):
                nh = st.number_input("高度(m)", 0.0, 200.0, obs['height'], 5.0, key=f"h_{idx}")
                if nh != obs['height']:
                    obs['height'] = nh
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.rerun()
                if st.button(f"删除", key=f"del_{idx}"):
                    st.session_state.obstacles.pop(idx)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.session_state.route = None
                    st.session_state.flight_route = None
                    st.rerun()
    st.sidebar.metric("障碍物总数", len(st.session_state.obstacles))
    st.sidebar.divider()
    if st.sidebar.button("💾 保存"):
        save_obstacles_to_file(st.session_state.obstacles)
        st.sidebar.success("已保存")
    if st.sidebar.button("📂 加载"):
        loaded = load_obstacles_from_file()
        if loaded:
            st.session_state.obstacles = loaded
            st.rerun()
    if st.sidebar.button("🧹 清空所有"):
        st.session_state.obstacles = []
        if os.path.exists(OBSTACLE_FILE): os.remove(OBSTACLE_FILE)
        st.session_state.route = None
        st.session_state.flight_route = None
        st.rerun()

    col1, col2 = st.columns([1,2])
    with col1:
        st.subheader("坐标输入")
        lat_a = st.number_input("起点纬度", value=32.2322, format="%.6f", key="lat_a")
        lon_a = st.number_input("起点经度", value=118.7490, format="%.6f", key="lon_a")
        lat_b = st.number_input("终点纬度", value=32.2343, format="%.6f", key="lat_b")
        lon_b = st.number_input("终点经度", value=118.7495, format="%.6f", key="lon_b")
        fly_h = st.slider("飞行高度(m)", 0, 100, 50, key="fh")

        if coord_mode == "GCJ-02":
            d_lon_a, d_lat_a = gcj02_to_wgs84(lon_a, lat_a)
            d_lon_b, d_lat_b = gcj02_to_wgs84(lon_b, lat_b)
        else:
            d_lon_a, d_lat_a = lon_a, lat_a
            d_lon_b, d_lat_b = lon_b, lat_b

        if st.button("✈️ 生成平滑绕行航线", use_container_width=True):
            with st.spinner("计算中..."):
                A = (d_lon_a, d_lat_a)
                B = (d_lon_b, d_lat_b)
                route = generate_route(A, B, st.session_state.obstacles, fly_h, st.session_state.safety, st.session_state.side)
                if len(route) == 2:
                    st.success("无冲突，无需绕行")
                    st.session_state.route = None
                    st.session_state.flight_route = None
                else:
                    st.success(f"已生成平滑绕行航线，共 {len(route)} 个航点")
                    st.session_state.route = route
                    st.session_state.flight_route = route
                st.rerun()

        if st.button("清除航线"):
            st.session_state.route = None
            st.session_state.flight_route = None
            st.rerun()

    with col2:
        center = [d_lat_a, d_lon_a]
        m = folium.Map(location=center, zoom_start=17,
                       tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                       attr='Esri World Imagery')
        folium.PolyLine([[d_lat_a, d_lon_a], [d_lat_b, d_lon_b]], color="yellow", weight=5, opacity=0.8, popup="原始航线").add_to(m)
        if st.session_state.route:
            route_locs = [[lat, lng] for lng, lat in st.session_state.route]
            folium.PolyLine(route_locs, color="blue", weight=4, opacity=0.9, popup="规划航线").add_to(m)
            spt = st.session_state.route[0]
            ept = st.session_state.route[-1]
            folium.Marker([spt[1], spt[0]], popup="起点", icon=folium.Icon(color='blue', icon='play')).add_to(m)
            folium.Marker([ept[1], ept[0]], popup="终点", icon=folium.Icon(color='blue', icon='stop')).add_to(m)
        folium.Marker([d_lat_a, d_lon_a], popup=f"起点A (高度:{fly_h}m)", icon=folium.Icon(color='red', icon='play')).add_to(m)
        folium.Marker([d_lat_b, d_lon_b], popup="终点B", icon=folium.Icon(color='green', icon='stop')).add_to(m)
        for obs in st.session_state.obstacles:
            poly = [[lat, lng] for lng, lat in obs["vertices"]]
            folium.Polygon(poly, color="red", weight=3, fill=True, fill_color="red", fill_opacity=0.3,
                           popup=f"高度 {obs['height']} m").add_to(m)
        draw = Draw(draw_options={"polyline":False,"rectangle":True,"circle":False,"marker":False,"circlemarker":False,"polygon":True},
                    edit_options={"edit":True,"remove":True})
        draw.add_to(m)
        output = st_folium(m, width=800, height=500, returned_objects=["last_active_drawing"])
        if output and output.get("last_active_drawing"):
            drawing = output["last_active_drawing"]
            geom = drawing.get("geometry", {}).get("type")
            coords = drawing.get("geometry", {}).get("coordinates")
            if geom == "Polygon" and coords:
                ring = coords[0]
                poly_wgs84 = [(lng, lat) for lng, lat in ring]
                exists = any(obs["vertices"] == poly_wgs84 for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": poly_wgs84, "height": st.session_state.default_height}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.success(f"已添加障碍物 (高度 {new_obs['height']} m)")
                    st.rerun()
            elif geom == "Rectangle" and coords:
                lng1, lat1 = coords[0]; lng2, lat2 = coords[1]
                rect = [(lng1, lat1), (lng2, lat1), (lng2, lat2), (lng1, lat2)]
                exists = any(obs["vertices"] == rect for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": rect, "height": st.session_state.default_height}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.success("已添加矩形障碍物")
                    st.rerun()

# ========== 飞行监控页面 ==========
elif page == "飞行监控":
    st.header("✈️ 飞行实时画面 - 任务执行监控")

    if st.session_state.flight_route is None or len(st.session_state.flight_route) < 2:
        st.warning("请先在航线规划页面生成绕行航线")
        st.stop()

    route = st.session_state.flight_route
    total_dist = 0.0
    for i in range(len(route)-1):
        total_dist += math.hypot(route[i+1][0]-route[i][0], route[i+1][1]-route[i][1]) * 111000.0

    if "current_pos" not in st.session_state:
        st.session_state.current_pos = route[0]
        st.session_state.flight_idx = 0
        st.session_state.flight_travelled = 0.0
        st.session_state.flight_battery = 100.0
        st.session_state.flight_active = False
        st.session_state.flight_paused = False
        st.session_state.flight_start = None
        st.session_state.flight_last_update = None

    col_a, col_b, col_c, col_d, col_e = st.columns(5)
    with col_a:
        start_btn = st.button("▶️ 开始任务", use_container_width=True)
    with col_b:
        pause_btn = st.button("⏸️ 暂停", use_container_width=True)
    with col_c:
        stop_btn = st.button("⏹️ 停止", use_container_width=True)
    with col_d:
        reset_btn = st.button("🔄 重置", use_container_width=True)
    with col_e:
        spd = st.number_input("速度(m/s)", 1.0, 30.0, st.session_state.flight_speed, 1.0, key="fsp")
        st.session_state.flight_speed = spd

    if start_btn:
        if st.session_state.flight_idx >= len(route)-1:
            st.session_state.flight_idx = 0
            st.session_state.flight_travelled = 0.0
            st.session_state.current_pos = route[0]
            st.session_state.flight_battery = 100.0
            st.session_state.flight_start = None
            st.session_state.flight_last_update = None
        st.session_state.flight_active = True
        st.session_state.flight_paused = False
        if st.session_state.flight_start is None:
            st.session_state.flight_start = time.time()
            st.session_state.flight_last_update = time.time()
        st.rerun()
    if pause_btn and st.session_state.flight_active and not st.session_state.flight_paused:
        st.session_state.flight_paused = True
        st.session_state.flight_active = False
        st.rerun()
    if stop_btn:
        st.session_state.flight_active = False
        st.session_state.flight_paused = False
        st.session_state.flight_idx = 0
        st.session_state.flight_travelled = 0.0
        st.session_state.flight_battery = 100.0
        st.session_state.current_pos = route[0]
        st.session_state.flight_start = None
        st.session_state.flight_last_update = None
        st.rerun()
    if reset_btn:
        st.session_state.flight_active = False
        st.session_state.flight_paused = False
        st.session_state.flight_idx = 0
        st.session_state.flight_travelled = 0.0
        st.session_state.flight_battery = 100.0
        st.session_state.current_pos = route[0]
        st.session_state.flight_start = None
        st.session_state.flight_last_update = None
        st.rerun()

    if st.session_state.flight_active and not st.session_state.flight_paused:
        now = time.time()
        if st.session_state.flight_last_update is None:
            st.session_state.flight_last_update = now
        dt = now - st.session_state.flight_last_update
        if dt > 0:
            move = st.session_state.flight_speed * dt
            while move > 0 and st.session_state.flight_idx < len(route)-1:
                seg_start = route[st.session_state.flight_idx]
                seg_end = route[st.session_state.flight_idx+1]
                seg_len = math.hypot(seg_end[0]-seg_start[0], seg_end[1]-seg_start[1]) * 111000.0
                if move < seg_len:
                    ratio = move / seg_len
                    cur_lng = seg_start[0] + ratio * (seg_end[0]-seg_start[0])
                    cur_lat = seg_start[1] + ratio * (seg_end[1]-seg_start[1])
                    st.session_state.current_pos = (cur_lng, cur_lat)
                    st.session_state.flight_travelled += move
                    st.session_state.flight_battery = max(0, st.session_state.flight_battery - move/50.0)
                    move = 0
                else:
                    move -= seg_len
                    st.session_state.flight_travelled += seg_len
                    st.session_state.flight_battery = max(0, st.session_state.flight_battery - seg_len/50.0)
                    st.session_state.flight_idx += 1
                    st.session_state.current_pos = route[st.session_state.flight_idx]
            st.session_state.flight_last_update = now
            if st.session_state.flight_idx >= len(route)-1:
                st.session_state.flight_active = False
                st.success("🎉 任务完成！")
            else:
                time.sleep(0.05)
                st.rerun()

    idx = st.session_state.flight_idx
    travelled = st.session_state.flight_travelled
    remaining = max(0.0, total_dist - travelled)
    prog = travelled / total_dist if total_dist>0 else 1.0
    eta = remaining / st.session_state.flight_speed if st.session_state.flight_speed>0 else 0
    eta_str = str(datetime.timedelta(seconds=int(eta))) if eta<86400 else ">1天"
    elapsed = time.time() - st.session_state.flight_start if st.session_state.flight_start else 0
    elapsed_str = str(datetime.timedelta(seconds=int(elapsed)))

    st.markdown("---")
    c1,c2,c3,c4,c5 = st.columns(5)
    c1.metric("当前航点", f"{idx+1}/{len(route)}")
    c1.write(f"进度: {prog*100:.1f}%")
    c2.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
    c3.metric("已用时间", elapsed_str)
    c4.metric("剩余距离", f"{remaining:.0f} m")
    c5.metric("预计到达", eta_str)
    st.markdown("---")
    batt_col1, batt_col2 = st.columns([1,3])
    batt_col1.metric("电量模拟", f"{st.session_state.flight_battery:.1f}%")
    batt_col2.progress(st.session_state.flight_battery/100.0)

    st.subheader("通信链路拓扑与数据流")
    cc1,cc2,cc3 = st.columns(3)
    cc1.success("GCS 在线\n地面站\n192.168.1.100")
    cc2.success("OBC 在线\n机载计算机\nRaspberry Pi 4")
    cc3.success("FCU 在线\n飞控\nPX4 / ArduPilot")
    st.info("MAVLink 已连接")

    st.subheader("实时飞行地图")
    map_center = [st.session_state.current_pos[1], st.session_state.current_pos[0]]
    m = folium.Map(location=map_center, zoom_start=17,
                   tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                   attr='Esri World Imagery')
    folium.PolyLine([[lat, lng] for lng, lat in route], color="blue", weight=4, opacity=0.8, popup="规划航线").add_to(m)
    if idx > 0:
        flown = route[:idx+1]
        if st.session_state.flight_active and idx < len(route)-1:
            flown.append(st.session_state.current_pos)
        folium.PolyLine([[lat, lng] for lng, lat in flown], color="green", weight=3, opacity=0.7, popup="已飞航迹").add_to(m)
    folium.Marker([st.session_state.current_pos[1], st.session_state.current_pos[0]], popup="无人机", icon=folium.Icon(color='darkblue', icon='plane', prefix='fa')).add_to(m)
    folium.Marker([route[0][1], route[0][0]], popup="起点", icon=folium.Icon(color='green', icon='play')).add_to(m)
    folium.Marker([route[-1][1], route[-1][0]], popup="终点", icon=folium.Icon(color='red', icon='stop')).add_to(m)
    st_folium(m, width=800, height=500, returned_objects=[])
