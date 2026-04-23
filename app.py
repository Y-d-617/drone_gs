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

# ========== 障碍物持久化 ==========
OBSTACLE_FILE = "obstacles.json"

def validate_obstacles(obstacles):
    """验证障碍物数据，转换为统一格式 {vertices, height, center, radius}"""
    valid = []
    for obs in obstacles:
        if not isinstance(obs, dict):
            continue
        vertices = obs.get("vertices", [])
        if not isinstance(vertices, list) or len(vertices) < 3:
            continue
        valid_vertices = []
        for v in vertices:
            if isinstance(v, (list, tuple)) and len(v) == 2:
                try:
                    lng, lat = float(v[0]), float(v[1])
                    valid_vertices.append((lng, lat))
                except:
                    pass
        if len(valid_vertices) < 3:
            continue
        height = obs.get("height", 30.0)
        try:
            height = float(height)
        except:
            height = 30.0
        # 计算外接圆圆心和半径
        xs = [p[0] for p in valid_vertices]
        ys = [p[1] for p in valid_vertices]
        cx = (min(xs) + max(xs)) / 2
        cy = (min(ys) + max(ys)) / 2
        radius = max(max(xs)-cx, cx-min(xs), max(ys)-cy, cy-min(ys))  # 半边长
        valid.append({
            "vertices": valid_vertices,
            "height": height,
            "center": (cx, cy),
            "radius": radius
        })
    return valid

def save_obstacles_to_file(obstacles):
    # 保存前需要移除计算字段，只保存原始顶点和高度
    to_save = []
    for obs in obstacles:
        to_save.append({"vertices": obs["vertices"], "height": obs["height"]})
    try:
        with open(OBSTACLE_FILE, 'w', encoding='utf-8') as f:
            json.dump(to_save, f, ensure_ascii=False, indent=2)
        return True
    except:
        return False

def load_obstacles_from_file():
    if os.path.exists(OBSTACLE_FILE):
        try:
            with open(OBSTACLE_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, list):
                    return validate_obstacles(data)
        except:
            pass
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

# ========== 圆形绕行算法（稳定）==========
def circle_line_intersection(center, radius, A, B):
    """返回线段AB与圆的交点（从A到B方向，最多两个），若不相交返回空列表"""
    cx, cy = center
    ax, ay = A
    bx, by = B
    dx = bx - ax
    dy = by - ay
    fx = ax - cx
    fy = ay - cy
    a = dx*dx + dy*dy
    if a < 1e-12:
        return []
    b = 2*(fx*dx + fy*dy)
    c = (fx*fx + fy*fy) - radius*radius
    delta = b*b - 4*a*c
    if delta < 0:
        return []
    sqrt_delta = math.sqrt(delta)
    t1 = (-b - sqrt_delta) / (2*a)
    t2 = (-b + sqrt_delta) / (2*a)
    intersections = []
    for t in (t1, t2):
        if 0 <= t <= 1:
            x = ax + t*dx
            y = ay + t*dy
            intersections.append((x, y))
    # 按距离A排序
    intersections.sort(key=lambda p: math.hypot(p[0]-ax, p[1]-ay))
    return intersections

def get_tangent_points(center, radius, point):
    """从圆外一点到圆的两条切线切点（返回两个点）"""
    cx, cy = center
    px, py = point
    dx = px - cx
    dy = py - cy
    dist = math.hypot(dx, dy)
    if dist <= radius:
        return []  # 点在圆内或圆上，无法作切线
    angle = math.atan2(dy, dx)
    delta = math.asin(radius / dist)
    angle1 = angle + delta
    angle2 = angle - delta
    t1 = (cx + radius * math.cos(angle1), cy + radius * math.sin(angle1))
    t2 = (cx + radius * math.cos(angle2), cy + radius * math.sin(angle2))
    return [t1, t2]

def generate_detour_route(A, B, obstacles, flight_height, safety_meters):
    """基于圆形近似生成绕行航线，确保平滑且稳定"""
    # 筛选需要绕行的障碍物（高度低于飞行高度且与线段相交）
    relevant = []
    for obs in obstacles:
        if flight_height < obs["height"]:
            # 扩大半径作为安全距离
            expanded_radius = obs["radius"] + safety_meters / 111000.0
            inters = circle_line_intersection(obs["center"], expanded_radius, A, B)
            if inters:
                relevant.append((obs, inters, expanded_radius))
    if not relevant:
        return [A, B]
    # 按交点距离A排序，先处理第一个
    relevant.sort(key=lambda x: math.hypot(x[1][0][0]-A[0], x[1][0][1]-A[1]))
    # 迭代处理：对第一个障碍物生成绕行路径，然后递归处理后续段
    def detour_single(A, B, obs, inters, radius):
        # 取第一个交点和最后一个交点
        p_in = inters[0]
        p_out = inters[-1] if len(inters) > 1 else inters[0]
        # 计算从A到p_in的路径（直接直线，但为了平滑，可能添加切线点？）
        # 更优雅：从A到圆上入切点，然后沿圆弧到出切点，再到B
        # 简化：找到从A到圆的两条切线，选择与p_in最近的那个切点作为入口
        tangents = get_tangent_points(obs["center"], radius, A)
        if not tangents:
            # 如果A在圆内，直接取p_in作为入口
            entry = p_in
        else:
            # 选择离p_in最近的切点作为入口
            entry = min(tangents, key=lambda t: math.hypot(t[0]-p_in[0], t[1]-p_in[1]))
        tangents2 = get_tangent_points(obs["center"], radius, B)
        if not tangents2:
            exit = p_out
        else:
            exit = min(tangents2, key=lambda t: math.hypot(t[0]-p_out[0], t[1]-p_out[1]))
        # 控制点序列：A -> entry -> exit -> B
        control = [A, entry, exit, B]
        # 插值平滑
        return catmull_rom_spline(control, num_segments=30)

    # 处理第一个障碍物
    first_obs, first_inters, first_radius = relevant[0]
    # 生成绕行该障碍物的路径（可能为平滑曲线点集）
    detour_path = detour_single(A, B, first_obs, first_inters, first_radius)
    # 递归处理剩余障碍物：将detour_path作为新航线，再次检查冲突
    # 为了避免无限递归，只迭代两次
    current_route = detour_path
    for _ in range(5):
        new_route = [current_route[0]]
        conflict = False
        for i in range(len(current_route)-1):
            seg_start = current_route[i]
            seg_end = current_route[i+1]
            # 检查本段是否与任何障碍物相交
            found = None
            for obs in obstacles:
                if flight_height < obs["height"]:
                    radius = obs["radius"] + safety_meters / 111000.0
                    inters = circle_line_intersection(obs["center"], radius, seg_start, seg_end)
                    if inters:
                        found = obs
                        break
            if found is None:
                new_route.append(seg_end)
            else:
                conflict = True
                # 对这一段重新绕行
                radius = found["radius"] + safety_meters / 111000.0
                inters = circle_line_intersection(found["center"], radius, seg_start, seg_end)
                if inters:
                    # 简单绕行：添加两个切线点
                    tangents = get_tangent_points(found["center"], radius, seg_start)
                    tangents2 = get_tangent_points(found["center"], radius, seg_end)
                    if tangents and tangents2:
                        p1 = tangents[0] if len(tangents)>0 else inters[0]
                        p2 = tangents2[0] if len(tangents2)>0 else inters[-1]
                        sub = [seg_start, p1, p2, seg_end]
                        sub_smooth = catmull_rom_spline(sub, num_segments=20)
                        new_route.extend(sub_smooth[1:])
                    else:
                        new_route.append(seg_end)
                else:
                    new_route.append(seg_end)
        if not conflict:
            return current_route
        current_route = new_route
    return current_route

def catmull_rom_spline(points, num_segments=30):
    if not points or len(points) < 2:
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

# ========== Streamlit 页面配置 ==========
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

if "app_version" not in st.session_state or st.session_state.app_version != "v19_circle_detour":
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    loaded = load_obstacles_from_file()
    st.session_state.obstacles = loaded if loaded else []
    st.session_state.default_obstacle_height = 30.0
    st.session_state.safety_distance = 3.0
    st.session_state.detour_route = None
    st.session_state.app_version = "v19_circle_detour"
else:
    if st.session_state.obstacles:
        # 确保障碍物包含计算字段（如果从旧版加载可能缺失，重新验证）
        if "center" not in st.session_state.obstacles[0]:
            st.session_state.obstacles = validate_obstacles(st.session_state.obstacles)

st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控"])
st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系设置（输入坐标的原始类型）", ["WGS-84", "GCJ-02"], index=0)
st.sidebar.info("✅ 卫星图底图：Esri World Imagery (WGS-84)\n若选择 GCJ-02，系统会自动转换为 WGS-84 匹配卫星图。")

if page == "航线规划":
    st.header("🗺️ 航线规划 + 障碍物圈选 (稳定圆形绕行)")

    st.sidebar.subheader("🚧 障碍物默认高度")
    default_h = st.sidebar.number_input(
        "新绘制障碍物的默认高度 (米)", 
        min_value=0.0, max_value=200.0, 
        value=st.session_state.default_obstacle_height, step=5.0
    )
    st.session_state.default_obstacle_height = default_h
    st.sidebar.divider()

    st.sidebar.subheader("🛡️ 安全距离 (米)")
    safety = st.sidebar.number_input(
        "绕行安全距离", 
        min_value=0.0, max_value=200.0, 
        value=st.session_state.safety_distance, step=5.0,
        help="绕行路径与障碍物的最小距离"
    )
    st.session_state.safety_distance = safety
    st.sidebar.divider()

    st.sidebar.subheader("📋 已添加的障碍物")
    if not st.session_state.obstacles:
        st.sidebar.write("暂无障碍物")
    else:
        for idx, obs in enumerate(st.session_state.obstacles):
            with st.sidebar.expander(f"障碍物 {idx+1} (高度: {obs['height']} m)"):
                new_height = st.number_input(
                    f"高度 (m)", min_value=0.0, max_value=200.0, value=obs['height'],
                    key=f"height_{idx}", step=5.0
                )
                if new_height != obs['height']:
                    obs['height'] = new_height
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.rerun()
                if st.button(f"🗑️ 删除障碍物 {idx+1}", key=f"del_{idx}"):
                    st.session_state.obstacles.pop(idx)
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.session_state.detour_route = None
                    st.rerun()
                st.caption(f"顶点数: {len(obs['vertices'])}")

    st.sidebar.metric("障碍物总数", len(st.session_state.obstacles))
    st.sidebar.divider()
    col_save1, col_save2 = st.sidebar.columns(2)
    with col_save1:
        if st.button("💾 保存障碍物"):
            if save_obstacles_to_file(st.session_state.obstacles):
                st.sidebar.success("已保存")
    with col_save2:
        if st.button("📂 加载障碍物"):
            loaded = load_obstacles_from_file()
            if loaded:
                st.session_state.obstacles = loaded
                st.sidebar.success(f"加载 {len(loaded)} 个")
                st.rerun()
            else:
                st.sidebar.warning("无备份文件或文件损坏")
    if st.sidebar.button("🧹 清空所有障碍物并重置文件"):
        st.session_state.obstacles = []
        if os.path.exists(OBSTACLE_FILE):
            os.remove(OBSTACLE_FILE)
        st.session_state.detour_route = None
        st.sidebar.success("已清空障碍物")
        st.rerun()

    col1, col2 = st.columns([1, 2])
    with col1:
        st.subheader("📍 坐标输入")
        lat_a = st.number_input("起点 A 纬度", value=32.2322, format="%.6f")
        lon_a = st.number_input("起点 A 经度", value=118.7490, format="%.6f")
        lat_b = st.number_input("终点 B 纬度", value=32.2343, format="%.6f")
        lon_b = st.number_input("终点 B 经度", value=118.7495, format="%.6f")
        flight_height = st.slider("设定飞行高度 (m)", 0, 100, 50)

        if coord_mode == "GCJ-02":
            display_lon_a, display_lat_a = gcj02_to_wgs84(lon_a, lat_a)
            display_lon_b, display_lat_b = gcj02_to_wgs84(lon_b, lat_b)
            st.success("已自动将 GCJ-02 坐标转换为 WGS-84")
        else:
            display_lon_a, display_lat_a = lon_a, lat_a
            display_lon_b, display_lat_b = lon_b, lat_b
            st.info("直接使用 WGS-84 坐标")

        if st.button("✈️ 生成绕行航线"):
            with st.spinner("正在计算绕行路径..."):
                A_wgs = (display_lon_a, display_lat_a)
                B_wgs = (display_lon_b, display_lat_b)
                try:
                    detour = generate_detour_route(
                        A_wgs, B_wgs, 
                        st.session_state.obstacles, 
                        flight_height,
                        st.session_state.safety_distance
                    )
                    if len(detour) == 2:
                        st.success("✅ 无冲突，无需绕行")
                        st.session_state.detour_route = None
                    else:
                        st.success(f"✅ 已生成绕行航线，共 {len(detour)} 个航点")
                        st.session_state.detour_route = detour
                except Exception as e:
                    st.error(f"计算失败: {e}")
                    st.session_state.detour_route = None
                st.rerun()

        if st.button("清除绕行航线"):
            st.session_state.detour_route = None
            st.rerun()

        if st.button("清除所有障碍物"):
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
            # 显示多边形（原始障碍物形状）
            poly_folium = [[lat, lng] for lng, lat in obs["vertices"]]
            folium.Polygon(
                locations=poly_folium, color="red", weight=3, fill=True, fill_color="red", fill_opacity=0.3,
                popup=f"障碍物 {idx+1}\n高度: {obs['height']} m"
            ).add_to(m)
            # 可选：显示外接圆（调试用，这里注释掉）
            # folium.Circle(radius=obs["radius"]*111000, location=[obs["center"][1], obs["center"][0]], color='gray', fill=False).add_to(m)

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
                    # 重新验证以添加 center/radius
                    validated = validate_obstacles([new_obs])
                    if validated:
                        st.session_state.obstacles.append(validated[0])
                        save_obstacles_to_file(st.session_state.obstacles)
                        st.success(f"已添加障碍物（高度 {new_obs['height']} m）")
                        st.rerun()
            elif geom_type == "Rectangle" and coords:
                lng1, lat1 = coords[0]; lng2, lat2 = coords[1]
                rect = [(lng1, lat1), (lng2, lat1), (lng2, lat2), (lng1, lat2)]
                exists = any(obs["vertices"] == rect for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": rect, "height": st.session_state.default_obstacle_height}
                    validated = validate_obstacles([new_obs])
                    if validated:
                        st.session_state.obstacles.append(validated[0])
                        save_obstacles_to_file(st.session_state.obstacles)
                        st.success("已添加矩形障碍物")
                        st.rerun()

elif page == "飞行监控":
    st.header("✈️ 飞行监控 (心跳包实时状态)")
    placeholder = st.empty()
    if st.button("开始接收实时数据", key="btn_monitor_v19"):
        for _ in range(50):
            packet = st.session_state.sim.generate_packet()
            st.session_state.history.append(packet)
            plot_df = pd.DataFrame(st.session_state.history[-20:])
            with placeholder.container():
                m1, m2, m3 = st.columns(3)
                avg_rtt, loss_rate = st.session_state.sim.get_summary(st.session_state.history)
                m1.metric("实时 RTT", f"{packet['rtt']:.3f}s", delta=packet['status'], delta_color="inverse")
                m2.metric("平均 RTT", f"{avg_rtt:.3f}s")
                m3.metric("累计丢包率", f"{loss_rate:.1f}%")
                st.subheader("通讯延迟 (RTT) 变化曲线")
                st.line_chart(plot_df.set_index("time")["rtt"])
                if packet['is_timeout']:
                    st.error(f"警报：北京时间 {packet['time']} 发生通讯超时！")
            time.sleep(0.4)
    else:
        st.info("请点击按钮开始模拟监控。")
