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

def save_obstacles(obs):
    try:
        with open(OBSTACLE_FILE, 'w', encoding='utf-8') as f:
            json.dump(obs, f, ensure_ascii=False, indent=2)
        return True
    except:
        return False

def load_obstacles():
    if os.path.exists(OBSTACLE_FILE):
        try:
            with open(OBSTACLE_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, list):
                    cleaned = []
                    for o in data:
                        if isinstance(o, dict) and "vertices" in o and "height" in o and len(o["vertices"])>=3:
                            cleaned.append(o)
                    return cleaned
        except:
            pass
    return []

# ========== GCJ-02 to WGS-84 ==========
def gcj02_to_wgs84(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    PI = math.pi
    def transform_lat(lng, lat):
        ret = -100.0 + 2.0*lng + 3.0*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
        ret += (20.0*math.sin(6.0*lng*PI) + 20.0*math.sin(2.0*lng*PI))*2.0/3.0
        ret += (20.0*math.sin(lat*PI) + 40.0*math.sin(lat/3.0*PI))*2.0/3.0
        ret += (160.0*math.sin(lat/12.0*PI) + 320*math.sin(lat*PI/30.0))*2.0/3.0
        return ret
    def transform_lng(lng, lat):
        ret = 300.0 + lng + 2.0*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
        ret += (20.0*math.sin(6.0*lng*PI) + 20.0*math.sin(2.0*lng*PI))*2.0/3.0
        ret += (20.0*math.sin(lng*PI) + 40.0*math.sin(lng/3.0*PI))*2.0/3.0
        ret += (150.0*math.sin(lng/12.0*PI) + 300.0*math.sin(lng*PI/30.0))*2.0/3.0
        return ret
    dlat = transform_lat(lng-105.0, lat-35.0)
    dlng = transform_lng(lng-105.0, lat-35.0)
    radlat = lat/180.0*PI
    magic = math.sin(radlat)
    magic = 1 - ee*magic*magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat*180.0) / ((a*(1-ee))/(magic*sqrtmagic)*PI)
    dlng = (dlng*180.0) / (a/sqrtmagic*math.cos(radlat)*PI)
    return lng-dlng, lat-dlat

# ========== 几何函数 ==========
def seg_intersect(x1,y1,x2,y2,x3,y3,x4,y4):
    def cross(ax,ay,bx,by): return ax*by - ay*bx
    def on_seg(px,py,qx,qy,rx,ry): return min(px,qx)<=rx<=max(px,qx) and min(py,qy)<=ry<=max(py,qy)
    o1 = cross(x2-x1,y2-y1,x3-x1,y3-y1)
    o2 = cross(x2-x1,y2-y1,x4-x1,y4-y1)
    o3 = cross(x4-x3,y4-y3,x1-x3,y1-y3)
    o4 = cross(x4-x3,y4-y3,x2-x3,y2-y3)
    if o1==0 and on_seg(x1,y1,x2,y2,x3,y3): return True
    if o2==0 and on_seg(x1,y1,x2,y2,x4,y4): return True
    if o3==0 and on_seg(x3,y3,x4,y4,x1,y1): return True
    if o4==0 and on_seg(x3,y3,x4,y4,x2,y2): return True
    return (o1>0)!=(o2>0) and (o3>0)!=(o4>0)

def poly_intersects_seg(poly, s, e):
    try:
        n = len(poly)
        if n<3: return False
        for i in range(n):
            x1,y1 = poly[i]
            x2,y2 = poly[(i+1)%n]
            if seg_intersect(s[0],s[1],e[0],e[1], x1,y1,x2,y2):
                return True
        mx = (s[0]+e[0])/2
        my = (s[1]+e[1])/2
        inside = False
        for i in range(n):
            x1,y1 = poly[i]
            x2,y2 = poly[(i+1)%n]
            if ((y1>my)!=(y2>my)) and (mx < (x2-x1)*(my-y1)/(y2-y1)+x1):
                inside = not inside
        return inside
    except:
        return False

def bbox(poly):
    xs = [v[0] for v in poly]
    ys = [v[1] for v in poly]
    return min(xs), min(ys), max(xs), max(ys)

def catmull_rom_spline(pts, seg=30):
    if len(pts)<2: return pts
    if len(pts)==2:
        return [pts[0] + (pts[1]-pts[0])*t for t in [i/seg for i in range(seg+1)]]
    res = []
    for i in range(len(pts)-1):
        p0 = pts[max(i-1,0)]
        p1 = pts[i]
        p2 = pts[i+1]
        p3 = pts[min(i+2,len(pts)-1)]
        for t in [j/seg for j in range(seg)]:
            t2 = t*t
            t3 = t2*t
            x = 0.5 * ((2*p1[0]) + (-p0[0]+p2[0])*t + (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2 + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
            y = 0.5 * ((2*p1[1]) + (-p0[1]+p2[1])*t + (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2 + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
            res.append((x,y))
    res.append(pts[-1])
    return res

# ========== 绕行生成 ==========
def detour_segment(A, B, obs, safe_m, side):
    mx, my, Mx, My = bbox(obs["vertices"])
    exp = safe_m / 111000.0
    mx -= exp; my -= exp; Mx += exp; My += exp
    pts = [(mx,my), (mx,My), (Mx,My), (Mx,my)]
    if side == "left":
        p1, p2 = pts[0], pts[1]
    elif side == "right":
        p1, p2 = pts[3], pts[2]
    else:
        paths = [([A,pts[0],pts[1],B]), ([A,pts[1],pts[2],B]), ([A,pts[2],pts[3],B]), ([A,pts[3],pts[0],B])]
        best = min(paths, key=lambda p: math.hypot(p[1][0]-p[0][0], p[1][1]-p[0][1])+math.hypot(p[2][0]-p[1][0], p[2][1]-p[1][1])+math.hypot(p[3][0]-p[2][0], p[3][1]-p[2][1]))
        return catmull_rom_spline(best, 30)
    if math.hypot(p1[0]-A[0], p1[1]-A[1]) > math.hypot(p2[0]-A[0], p2[1]-A[1]):
        p1, p2 = p2, p1
    return catmull_rom_spline([A,p1,p2,B], 30)

def generate_route(A, B, obstacles, flight_h, safe_m, side="auto", max_iter=10):
    relevant = [o for o in obstacles if flight_h < o["height"]]
    if not relevant: return [A,B]
    route = [A,B]
    for _ in range(max_iter):
        new = [route[0]]
        conflict = False
        for i in range(len(route)-1):
            s = route[i]
            e = route[i+1]
            target = None
            for o in relevant:
                if poly_intersects_seg(o["vertices"], s, e):
                    target = o
                    break
            if target is None:
                new.append(e)
            else:
                conflict = True
                seg = detour_segment(s, e, target, safe_m, side)
                new.extend(seg[1:])
        route = new
        if not conflict:
            return route
    return route

# ========== Streamlit 页面 ==========
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

# 初始化 session_state (使用带前缀的键避免冲突)
if "init_done" not in st.session_state:
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    st.session_state.obstacles = load_obstacles()
    st.session_state.default_h = 30.0
    st.session_state.safe_m = 3.0
    st.session_state.route = None
    st.session_state.route_side = "auto"
    st.session_state.flight_route = None
    st.session_state.flight_idx = 0
    st.session_state.flight_active = False
    st.session_state.flight_paused = False
    st.session_state.flight_start = None
    st.session_state.flight_speed = 10.0
    st.session_state.flight_battery = 100.0
    st.session_state.flight_last_update = None
    st.session_state.flight_travelled = 0.0
    st.session_state.current_pos = None
    st.session_state.init_done = True
else:
    # 兼容旧数据
    if st.session_state.obstacles and isinstance(st.session_state.obstacles[0], list):
        new = []
        for poly in st.session_state.obstacles:
            new.append({"vertices": poly, "height": 30.0})
        st.session_state.obstacles = new
        save_obstacles(new)

st.sidebar.title("导航控制")
page = st.sidebar.radio("页面", ["航线规划", "飞行监控"])
coord_mode = st.sidebar.radio("坐标系", ["WGS-84", "GCJ-02"], index=0)
st.sidebar.info("卫星图: Esri World Imagery (WGS-84)")

# ========== 航线规划页面 ==========
if page == "航线规划":
    st.header("🗺️ 航线规划 + 障碍物圈选")

    st.sidebar.subheader("障碍物默认高度")
    dh = st.sidebar.number_input("米", 0.0, 200.0, st.session_state.default_h, 5.0)
    st.session_state.default_h = dh

    st.sidebar.subheader("安全距离 (米)")
    sm = st.sidebar.number_input("绕行安全距离", 0.0, 200.0, st.session_state.safe_m, 5.0)
    st.session_state.safe_m = sm

    st.sidebar.subheader("绕行侧")
    side_opt = st.sidebar.selectbox("偏好", ["auto", "left", "right"], index=["auto","left","right"].index(st.session_state.route_side))
    st.session_state.route_side = side_opt

    st.sidebar.subheader("已添加的障碍物")
    if not st.session_state.obstacles:
        st.sidebar.write("暂无")
    else:
        for idx, obs in enumerate(st.session_state.obstacles):
            with st.sidebar.expander(f"障碍物 {idx+1} (高度: {obs['height']} m)"):
                nh = st.number_input("高度(m)", 0.0, 200.0, obs['height'], 5.0, key=f"oh_{idx}")
                if nh != obs['height']:
                    obs['height'] = nh
                    save_obstacles(st.session_state.obstacles)
                    st.rerun()
                if st.button(f"删除", key=f"del_{idx}"):
                    st.session_state.obstacles.pop(idx)
                    save_obstacles(st.session_state.obstacles)
                    st.session_state.route = None
                    st.session_state.flight_route = None
                    st.rerun()
    st.sidebar.metric("障碍物总数", len(st.session_state.obstacles))
    if st.sidebar.button("💾 保存"):
        save_obstacles(st.session_state.obstacles)
        st.sidebar.success("已保存")
    if st.sidebar.button("📂 加载"):
        loaded = load_obstacles()
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
        lat_a = st.number_input("起点纬度", value=32.2322, format="%.6f")
        lon_a = st.number_input("起点经度", value=118.7490, format="%.6f")
        lat_b = st.number_input("终点纬度", value=32.2343, format="%.6f")
        lon_b = st.number_input("终点经度", value=118.7495, format="%.6f")
        fly_h = st.slider("飞行高度(m)", 0, 100, 50)

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
                route = generate_route(A, B, st.session_state.obstacles, fly_h, st.session_state.safe_m, st.session_state.route_side)
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
                    new_obs = {"vertices": poly_wgs84, "height": st.session_state.default_h}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles(st.session_state.obstacles)
                    st.success(f"已添加障碍物 (高度 {new_obs['height']} m)")
                    st.rerun()
            elif geom == "Rectangle" and coords:
                lng1, lat1 = coords[0]; lng2, lat2 = coords[1]
                rect = [(lng1, lat1), (lng2, lat1), (lng2, lat2), (lng1, lat2)]
                exists = any(obs["vertices"] == rect for obs in st.session_state.obstacles)
                if not exists:
                    new_obs = {"vertices": rect, "height": st.session_state.default_h}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles(st.session_state.obstacles)
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

    if st.session_state.current_pos is None:
        st.session_state.current_pos = route[0]
        st.session_state.flight_idx = 0
        st.session_state.flight_travelled = 0.0
        st.session_state.flight_battery = 100.0

    c1,c2,c3,c4,c5 = st.columns(5)
    with c1:
        if st.button("▶️ 开始任务", use_container_width=True):
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
    with c2:
        if st.button("⏸️ 暂停", use_container_width=True):
            if st.session_state.flight_active and not st.session_state.flight_paused:
                st.session_state.flight_paused = True
                st.session_state.flight_active = False
                st.rerun()
    with c3:
        if st.button("⏹️ 停止", use_container_width=True):
            st.session_state.flight_active = False
            st.session_state.flight_paused = False
            st.session_state.flight_idx = 0
            st.session_state.flight_travelled = 0.0
            st.session_state.flight_battery = 100.0
            st.session_state.current_pos = route[0]
            st.session_state.flight_start = None
            st.session_state.flight_last_update = None
            st.rerun()
    with c4:
        if st.button("🔄 重置", use_container_width=True):
            st.session_state.flight_active = False
            st.session_state.flight_paused = False
            st.session_state.flight_idx = 0
            st.session_state.flight_travelled = 0.0
            st.session_state.flight_battery = 100.0
            st.session_state.current_pos = route[0]
            st.session_state.flight_start = None
            st.session_state.flight_last_update = None
            st.rerun()
    with c5:
        spd = st.number_input("速度(m/s)", 1.0, 30.0, st.session_state.flight_speed, 1.0)
        st.session_state.flight_speed = spd

    if st.session_state.flight_active and not st.session_state.flight_paused:
        now = time.time()
        if st.session_state.flight_last_update is None:
            st.session_state.flight_last_update = now
        dt = now - st.session_state.flight_last_update
        if dt > 0:
            move = st.session_state.flight_speed * dt
            while move > 0 and st.session_state.flight_idx < len(route)-1:
                s = route[st.session_state.flight_idx]
                e = route[st.session_state.flight_idx+1]
                seg_len = math.hypot(e[0]-s[0], e[1]-s[1]) * 111000.0
                if move < seg_len:
                    ratio = move / seg_len
                    cur_lng = s[0] + ratio*(e[0]-s[0])
                    cur_lat = s[1] + ratio*(e[1]-s[1])
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
    m1,m2,m3,m4,m5 = st.columns(5)
    m1.metric("当前航点", f"{idx+1}/{len(route)}")
    m1.write(f"进度: {prog*100:.1f}%")
    m2.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
    m3.metric("已用时间", elapsed_str)
    m4.metric("剩余距离", f"{remaining:.0f} m")
    m5.metric("预计到达", eta_str)
    st.markdown("---")
    bc1, bc2 = st.columns([1,3])
    bc1.metric("电量模拟", f"{st.session_state.flight_battery:.1f}%")
    bc2.progress(st.session_state.flight_battery/100.0)

    st.subheader("通信链路拓扑与数据流")
    t1,t2,t3 = st.columns(3)
    t1.success("GCS 在线\n地面站\n192.168.1.100")
    t2.success("OBC 在线\n机载计算机\nRaspberry Pi 4")
    t3.success("FCU 在线\n飞控\nPX4 / ArduPilot")
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
