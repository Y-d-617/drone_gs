import streamlit as st
import pandas as pd
import time
import folium
from streamlit_folium import st_folium
from utils import wgs84_to_gcj02
from heartbeat_sim import HeartbeatSimulator

# 页面基础配置
st.set_page_config(page_title="无人机地面站监控系统", layout="wide")

# --- 强制重置逻辑 (防止旧代码缓存导致的 NameError) ---
if "app_version" not in st.session_state or st.session_state.app_version != "v3_folium":
    st.session_state.sim = HeartbeatSimulator()
    st.session_state.history = []
    st.session_state.app_version = "v3_folium"

# --- 侧边栏导航 ---
st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控"])

st.sidebar.divider()
coord_mode = st.sidebar.radio("坐标系设置", ["WGS-84", "GCJ-02"], index=1)

# --- 页面1：航线规划 ---
if page == "航线规划":
    st.header("🗺️ 航线规划 (高德卫星地图)")
    
    col1, col2 = st.columns([1, 2])
    with col1:
        st.subheader("📍 坐标输入")
        lat_a = st.number_input("起点 A 纬度", value=32.2322, format="%.6f")
        lon_a = st.number_input("起点 A 经度", value=118.7490, format="%.6f")
        lat_b = st.number_input("终点 B 纬度", value=32.2343, format="%.6f")
        lon_b = st.number_input("终点 B 经度", value=118.7495, format="%.6f")
        height = st.slider("设定飞行高度 (m)", 0, 100, 50)
        
        # 坐标纠偏
        process_lon_a, process_lat_a = lon_a, lat_a
        process_lon_b, process_lat_b = lon_b, lat_b
        if coord_mode == "GCJ-02":
            process_lon_a, process_lat_a = wgs84_to_gcj02(lon_a, lat_a)
            process_lon_b, process_lat_b = wgs84_to_gcj02(lon_b, lat_b)
            st.success("已启用 GCJ-02 纠偏")

    with col2:
        # 创建 Folium 地图 (使用高德卫星图底图)
        # style=6 为卫星图，style=7 为街道图
        m = folium.Map(
            location=[process_lat_a, process_lon_a], 
            zoom_start=17, 
            tiles=None
        )
        
        folium.TileLayer(
            tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
            attr='高德-卫星图',
            name='高德卫星图',
            overlay=False,
            control=True
        ).add_to(m)

        # 添加 A、B 点标记
        folium.Marker(
            [process_lat_a, process_lon_a], 
            popup=f"起点 A (高度:{height}m)", 
            icon=folium.Icon(color='red', icon='play')
        ).add_to(m)
        
        folium.Marker(
            [process_lat_b, process_lon_b], 
            popup="终点 B", 
            icon=folium.Icon(color='green', icon='stop')
        ).add_to(m)

        # 绘制航线连线
        folium.PolyLine(
            locations=[[process_lat_a, process_lon_a], [process_lat_b, process_lon_b]],
            color="yellow",
            weight=5,
            opacity=0.8
        ).add_to(m)

        # 渲染地图
        st_folium(m, width=800, height=500, returned_objects=[])

# --- 页面2：飞行监控 ---
elif page == "飞行监控":
    st.header("✈️ 飞行监控 (心跳包实时状态)")
    
    # 动态显示容器
    placeholder = st.empty()
    
    if st.button("开始接收实时数据", key="btn_monitor_v3"):
        # 每次点击按钮时可以清空旧历史，也可以保留
        # st.session_state.history = [] 
        
        for _ in range(50):
            # 调用最新的 generate_packet，确保内部有 rtt 变量
            packet = st.session_state.sim.generate_packet()
            st.session_state.history.append(packet)
            
            # 取最近 20 条数据
            plot_df = pd.DataFrame(st.session_state.history[-20:])
            
            with placeholder.container():
                m1, m2, m3 = st.columns(3)
                avg_rtt, loss_rate = st.session_state.sim.get_summary(st.session_state.history)
                
                m1.metric("实时 RTT", f"{packet['rtt']:.3f}s", delta=packet['status'], delta_color="inverse")
                m2.metric("平均 RTT", f"{avg_rtt:.3f}s")
                m3.metric("累计丢包率", f"{loss_rate:.1f}%")
                
                st.subheader("通讯延迟 (RTT) 变化曲线")
                # 使用北京时间 time 作为横坐标
                st.line_chart(plot_df.set_index("time")["rtt"])
                
                if packet['is_timeout']:
                    st.error(f"警报：北京时间 {packet['time']} 发生通讯超时！")
            
            time.sleep(0.4)
    else:
        st.info("请点击按钮开始模拟监控。")