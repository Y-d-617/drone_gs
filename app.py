import streamlit as st
import pandas as pd
import time
import folium
from folium.plugins import Draw
from streamlit_folium import st_folium
from heartbeat_sim import HeartbeatSimulator
import math

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

# ========== 纯数学几何函数（无 shapely）==========
def segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
    """判断线段 (x1,y1)-(x2,y2) 和 (x3,y3)-(x4,y4) 是否相交"""
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
    """检查多边形是否与线段相交（包括包含关系）"""
    # 检查线段是否与任意边相交
    n = len(poly_vertices)
    for i in range(n):
        x1, y1 = poly_vertices[i]
        x2, y2 = poly_vertices[(i+1)%n]
        if segments_intersect(seg_start[0], seg_start[1], seg_end[0], seg_end[1], x1, y1, x2, y2):
            return True
    # 检查线段中点是否在多边形内部（射线法）
    mid_x = (seg_start[0] + seg_end[0]) / 2
    mid_y = (seg_start[1] + seg_end[1]) / 2
    inside = False
    for i in range(n):
        x1, y1 = poly_vertices[i]
        x2, y2 = poly_vertices[(i+1)%n]
        if ((y1 > mid_y) != (y2 > mid_y)) and (mid_x < (x2 - x1) * (mid_y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def get_bounding_box(poly_vertices):
    """返回多边形外接矩形 (minx, miny, maxx, maxy)"""
    xs = [v[0] for v in poly_vertices]
    ys = [v[1] for v in poly_vertices]
    return min(xs), min(ys), max(xs), max(ys)

def generate_detour_route(A, B, obstacles, flight_height):
    """生成绕行航线（绕过第一个相交障碍物的外接矩形）"""
    # 找出第一个需要绕行的障碍物
    target_obs = None
    for obs in obstacles:
        if flight_height < obs["height"]:
            if polygon_intersects_segment(obs["vertices"], A, B):
                target_obs = obs
                break
    if target_obs is None:
        return [A, B]
    
    # 获取外接矩形中心
    minx, miny, maxx, maxy = get_bounding_box(target_obs["vertices"])
    cx = (minx + maxx) / 2
    cy = (miny + maxy) / 2
    
    # AB方向单位向量
    dx = B[0] - A[0]
    dy = B[1] - A[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return [A, B]
    dx /= length
    dy /= length
    
    # 垂直向量
    perp_x = -dy
    perp_y = dx
    offset = 0.0005  # 约50米
    
    left = (cx + perp_x * offset, cy + perp_y * offset)
    right = (cx - perp_x * offset, cy - perp_y * offset)
    
    # 按距离起点排序
    dist_left = math.hypot(left[0]-A[0], left[1]-A[1])
    dist_right = math.hypot(right[0]-A[0], right[1]-A[1])
    if dist_left < dist_right:
        return [A, left, right, B]
    else:
        return [A, right, left, B]

# ========== Streamlit 页面配置 ==========
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

# 初始化 session_state
if "app_version" not in st.session_state or st.session_state.app_version != "v10_noshapely":
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    st.session_state.obstacles = []
    st.session_state.default_obstacle_height = 30.0
    st.session_state.detour_route = None
    st.session_state.app_version = "v10_noshapely"
else:
    # 兼容旧数据：将纯列表转换为带高度的字典
    if st.session_state.obstacles and isinstance(st.session_state.obstacles[0], list):
        new_obstacles = []
        for poly in st.session_state.obstacles:
            new_obstacles.append({"vertices": poly, "height": 30.0})
        st.session_state.obstacles = new_obstacles

# 侧边栏
st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控"])
st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系设置（输入坐标的原始类型）", ["WGS-84", "GCJ-02"], index=0)
st.sidebar.info("✅ 卫星图底图：Esri World Imagery (WGS-84)\n若选择 GCJ-02，系统会自动转换为 WGS-84 匹配卫星图。")

# ========== 页面1：航线规划 ==========
if page == "航线规划":
    st.header("🗺️ 航线规划 + 障碍物圈选 (WGS-84 卫星图)")

    # 侧边栏：障碍物默认高度
    st.sidebar.subheader("🚧 障碍物默认高度")
    default_height = st.sidebar.number_input(
        "新绘制障碍物的默认高度 (米)", 
        min_value=0.0, max_value=200.0, 
        value=st.session_state.default_obstacle_height, step=5.0
    )
    st.session_state.default_obstacle_height = default_height
    st.sidebar.divider()
    
    # 侧边栏：已有障碍物管理
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
                    st.rerun()
                if st.button(f"🗑️ 删除障碍物 {idx+1}", key=f"del_{idx}"):
                    st.session_state.obstacles.pop(idx)
                    st.session_state.detour_route = None
                    st.rerun()
                st.caption(f"顶点数: {len(obs['vertices'])}")
    st.sidebar.metric("障碍物总数", len(st.session_state.obstacles))

    col1, col2 = st.columns([1, 2])
    with col1:
        st.subheader("📍 坐标输入")
        lat_a = st.number_input("起点 A 纬度", value=32.2322, format="%.6f")
        lon_a = st.number_input("起点 A 经度", value=118.7490, format="%.6f")
        lat_b = st.number_input("终点 B 纬度", value=32.2343, format="%.6f")
        lon_b = st.number_input("终点 B 经度", value=118.7495, format="%.6f")
        flight_height = st.slider("设定飞行高度 (m)", 0, 100, 50)

        # 坐标转换（自动适配卫星图）
        if coord_mode == "GCJ-02":
            display_lon_a, display_lat_a = gcj02_to_wgs84(lon_a, lat_a)
            display_lon_b, display_lat_b = gcj02_to_wgs84(lon_b, lat_b)
            st.success("已自动将 GCJ-02 坐标转换为 WGS-84")
        else:
            display_lon_a, display_lat_a = lon_a, lat_a
            display_lon_b, display_lat_b = lon_b, lat_b
            st.info("直接使用 WGS-84 坐标")

        # 绕行控制按钮
        if st.button("✈️ 检测冲突并生成绕行航线"):
            with st.spinner("正在计算绕行路径..."):
                A_wgs = (display_lon_a, display_lat_a)
                B_wgs = (display_lon_b, display_lat_b)
                detour = generate_detour_route(A_wgs, B_wgs, st.session_state.obstacles, flight_height)
                if len(detour) == 2:
                    st.success("✅ 无冲突，无需绕行")
                    st.session_state.detour_route = None
                else:
                    st.success(f"✅ 已生成绕行航线，共 {len(detour)} 个航点")
                    st.session_state.detour_route = detour
                st.rerun()
        
        if st.button("清除绕行航线"):
            st.session_state.detour_route = None
            st.rerun()
        
        if st.button("清除所有障碍物"):
            st.session_state.obstacles = []
            st.session_state.detour_route = None
            st.rerun()

    with col2:
        map_center = [display_lat_a, display_lon_a]
        m = folium.Map(
            location=map_center, zoom_start=17,
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri World Imagery',
        )

        # 原始航线（黄色）
        folium.PolyLine(
            locations=[[display_lat_a, display_lon_a], [display_lat_b, display_lon_b]],
            color="yellow", weight=5, opacity=0.8, popup="原始航线"
        ).add_to(m)

        # 绕行航线（蓝色）
        if st.session_state.get("detour_route"):
            detour_locs = [[lat, lng] for lng, lat in st.session_state.detour_route]
            folium.PolyLine(
                locations=detour_locs, color="blue", weight=4, opacity=0.9,
                popup="绕行航线"
            ).add_to(m)
            for i, (lng, lat) in enumerate(st.session_state.detour_route):
                folium.Marker(
                    [lat, lng], popup=f"绕行航点 {i}", icon=folium.Icon(color='blue', icon='info-sign')
                ).add_to(m)

        # 起点终点标记
        folium.Marker([display_lat_a, display_lon_a], popup=f"起点 A (高度:{flight_height}m)", icon=folium.Icon(color='red', icon='play')).add_to(m)
        folium.Marker([display_lat_b, display_lon_b], popup="终点 B", icon=folium.Icon(color='green', icon='stop')).add_to(m)

        # 绘制所有障碍物
        for idx, obs in enumerate(st.session_state.obstacles):
            poly_folium = [[lat, lng] for lng, lat in obs["vertices"]]
            folium.Polygon(
                locations=poly_folium, color="red", weight=3, fill=True, fill_color="red", fill_opacity=0.3,
                popup=f"障碍物 {idx+1}\n高度: {obs['height']} m"
            ).add_to(m)

        # 绘图工具（多边形、矩形）
        draw = Draw(
            draw_options={"polyline": False, "rectangle": True, "circle": False, "marker": False, "circlemarker": False, "polygon": True},
            edit_options={"edit": True, "remove": True}
        )
        draw.add_to(m)
        output = st_folium(m, width=800, height=500, returned_objects=["last_active_drawing"])

        # 处理新绘制的图形
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
                    st.success(f"已添加障碍物（高度 {new_obs['height']} m）")
                    st.rerun()
            elif geom_type == "Rectangle" and coords:
                lng1, lat1 = coords[0]; lng2, lat2 = coords[1]
                rect = [(lng1, lat1), (lng2, lat1), (lng2, lat2), (lng1, lat2)]
                exists = any(obs["vertices"] == rect for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": rect, "height": st.session_state.default_obstacle_height}
                    st.session_state.obstacles.append(new_obs)
                    st.success("已添加矩形障碍物")
                    st.rerun()

# ========== 页面2：飞行监控 ==========
elif page == "飞行监控":
    st.header("✈️ 飞行监控 (心跳包实时状态)")
    placeholder = st.empty()
    if st.button("开始接收实时数据", key="btn_monitor_v10"):
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
