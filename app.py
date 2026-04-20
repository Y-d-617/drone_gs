import streamlit as st
import pandas as pd
import time
import folium
from folium.plugins import Draw
from streamlit_folium import st_folium
from utils import wgs84_to_gcj02
from heartbeat_sim import HeartbeatSimulator
import math

# ========== 坐标转换辅助函数 ==========
def gcj02_to_wgs84(lng, lat):
    """
    将 GCJ-02 坐标转换为 WGS-84 坐标（近似可逆算法）
    参考：https://github.com/wandergis/coordTransform_py
    """
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

# 页面基础配置
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

# --- 强制重置逻辑 (防止旧代码缓存导致的 NameError) ---
if "app_version" not in st.session_state or st.session_state.app_version != "v4_obstacle":
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    st.session_state.obstacles = []          # 存储障碍物多边形顶点列表 (WGS-84)
    st.session_state.app_version = "v4_obstacle"

# --- 侧边栏导航 ---
st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控"])

st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系设置", ["WGS-84", "GCJ-02"], index=1)

# --- 页面1：航线规划（含障碍物圈选）---
if page == "航线规划":
    st.header("🗺️ 航线规划 + 障碍物圈选 (高德卫星地图)")

    col1, col2 = st.columns([1, 2])
    with col1:
        st.subheader("📍 坐标输入")
        lat_a = st.number_input("起点 A 纬度 (WGS-84)", value=32.2322, format="%.6f")
        lon_a = st.number_input("起点 A 经度 (WGS-84)", value=118.7490, format="%.6f")
        lat_b = st.number_input("终点 B 纬度 (WGS-84)", value=32.2343, format="%.6f")
        lon_b = st.number_input("终点 B 经度 (WGS-84)", value=118.7495, format="%.6f")
        height = st.slider("设定飞行高度 (m)", 0, 100, 50)

        # 将用户输入的 WGS-84 坐标转换为 GCJ-02 用于地图显示
        if coord_mode == "GCJ-02":
            # 强制使用 GCJ-02 显示（高德底图要求）
            display_lon_a, display_lat_a = wgs84_to_gcj02(lon_a, lat_a)
            display_lon_b, display_lat_b = wgs84_to_gcj02(lon_b, lat_b)
            st.success("已启用 GCJ-02 坐标转换，位置与卫星底图匹配")
        else:
            # WGS-84 模式也显示转换后的坐标（否则偏移）——为了地图正确，总是转换
            display_lon_a, display_lat_a = wgs84_to_gcj02(lon_a, lat_a)
            display_lon_b, display_lat_b = wgs84_to_gcj02(lon_b, lat_b)
            st.info("当前为 WGS-84 输入，但地图已自动转换为 GCJ-02 显示")

        # 障碍物管理按钮
        st.subheader("🚧 障碍物管理")
        if st.button("清除所有障碍物"):
            st.session_state.obstacles = []
            st.success("已清除所有障碍物")
            st.rerun()

    with col2:
        # 创建地图中心点（使用起点转换后的 GCJ-02 坐标）
        map_center = [display_lat_a, display_lon_a]

        m = folium.Map(
            location=map_center,
            zoom_start=17,
            tiles=None
        )

        # 高德卫星图底图
        folium.TileLayer(
            tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
            attr='高德-卫星图',
            name='高德卫星图',
            overlay=False,
            control=True
        ).add_to(m)

        # 绘制航线（使用转换后的 GCJ-02 坐标）
        folium.PolyLine(
            locations=[[display_lat_a, display_lon_a], [display_lat_b, display_lon_b]],
            color="yellow",
            weight=5,
            opacity=0.8,
            popup="规划航线"
        ).add_to(m)

        # 添加起点/终点标记（GCJ-02）
        folium.Marker(
            [display_lat_a, display_lon_a],
            popup=f"起点 A (高度:{height}m)",
            icon=folium.Icon(color='red', icon='play')
        ).add_to(m)
        folium.Marker(
            [display_lat_b, display_lon_b],
            popup="终点 B",
            icon=folium.Icon(color='green', icon='stop')
        ).add_to(m)

        # ========== 绘制已存储的障碍物多边形 ==========
        for idx, poly_wgs84 in enumerate(st.session_state.obstacles):
            # 将多边形顶点从 WGS-84 转换为 GCJ-02
            poly_gcj02 = []
            for lng, lat in poly_wgs84:
                gcj_lng, gcj_lat = wgs84_to_gcj02(lng, lat)
                poly_gcj02.append([gcj_lat, gcj_lng])  # folium 使用 [lat, lng]
            folium.Polygon(
                locations=poly_gcj02,
                color="red",
                weight=3,
                fill=True,
                fill_color="red",
                fill_opacity=0.3,
                popup=f"障碍物 {idx+1}"
            ).add_to(m)

        # 添加 Draw 控件（支持绘制多边形、矩形、圆等，我们只关心多边形）
        draw = Draw(
            draw_options={
                "polyline": False,
                "rectangle": True,      # 矩形可作为快速圈选
                "circle": False,
                "marker": False,
                "circlemarker": False,
                "polygon": True,        # 多边形
            },
            edit_options={"edit": True, "remove": True}
        )
        draw.add_to(m)

        # 渲染地图并获取交互数据
        output = st_folium(m, width=800, height=500, returned_objects=["last_active_drawing"])

        # 处理用户新绘制的图形（多边形或矩形）
        if output and output.get("last_active_drawing"):
            drawing = output["last_active_drawing"]
            geom_type = drawing.get("geometry", {}).get("type")
            coords = drawing.get("geometry", {}).get("coordinates")
            if geom_type == "Polygon" and coords:
                # 提取多边形顶点 (GCJ-02 坐标，格式: [[lng, lat], ...])
                # coords 是一个三维数组：[[[lng, lat], ...]]，取第一个环
                ring = coords[0]
                # 转换为 WGS-84 并存储
                poly_wgs84 = []
                for lng, lat in ring:
                    wgs_lng, wgs_lat = gcj02_to_wgs84(lng, lat)
                    poly_wgs84.append((wgs_lng, wgs_lat))
                # 避免重复添加（简单检查是否已存在相同多边形）
                if poly_wgs84 not in st.session_state.obstacles:
                    st.session_state.obstacles.append(poly_wgs84)
                    st.success(f"已添加障碍物多边形，共 {len(poly_wgs84)} 个顶点")
                    st.rerun()
            elif geom_type == "Rectangle" and coords:
                # 矩形转换为多边形存储（四个顶点）
                # coords 格式: [[lng1, lat1], [lng2, lat2]] 表示对角线
                lng1, lat1 = coords[0]
                lng2, lat2 = coords[1]
                rect_gcj02 = [
                    [lng1, lat1],
                    [lng2, lat1],
                    [lng2, lat2],
                    [lng1, lat2]
                ]
                poly_wgs84 = []
                for lng, lat in rect_gcj02:
                    wgs_lng, wgs_lat = gcj02_to_wgs84(lng, lat)
                    poly_wgs84.append((wgs_lng, wgs_lat))
                if poly_wgs84 not in st.session_state.obstacles:
                    st.session_state.obstacles.append(poly_wgs84)
                    st.success("已添加矩形障碍物")
                    st.rerun()

    # 在侧边栏显示当前障碍物数量
    st.sidebar.metric("已圈选障碍物数量", len(st.session_state.obstacles))

# --- 页面2：飞行监控（保持不变）---
elif page == "飞行监控":
    st.header("✈️ 飞行监控 (心跳包实时状态)")

    placeholder = st.empty()

    if st.button("开始接收实时数据", key="btn_monitor_v4"):
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
