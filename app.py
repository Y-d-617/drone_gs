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
    # 初始化通信链路管理器
    st.session_state.comm_link = CommunicationLink()
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
    if "comm_link" not in st.session_state:
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

# ========== 侧边栏 ==========
st.sidebar.title("🧭 导航控制")
page = st.sidebar.radio("请选择功能页面", ["航线规划", "飞行监控", "通信链路"], key="page_radio")
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
        
        # 显示通信链路状态（简化版）
        st.subheader("📡 通信链路状态")
        comm = st.session_state.comm_link
        status_color_gcs = "🟢" if comm.link_status["GCS"] == "online" else "🔴"
        status_color_obc = "🟢" if comm.link_status["OBC"] == "online" else "🔴"
        status_color_fcu = "🟢" if comm.link_status["FCU"] == "online" else "🔴"
        st.markdown(f"{status_color_gcs} GCS  |  {status_color_obc} OBC  |  {status_color_fcu} FCU")
        
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

# ========== 通信链路页面 ==========
elif page == "通信链路":
    st.header("📡 通信链路拓扑与数据流监控")
    show_flash()
    
    comm_link = st.session_state.comm_link
    
    # 创建两列布局
    col_left, col_right = st.columns([1, 1])
    
    with col_left:
        # 渲染通信拓扑
        render_communication_topology(comm_link)
        
        st.markdown("---")
        # 渲染通信控制
        render_communication_controls(comm_link)
    
    with col_right:
        # 渲染通信日志
        render_communication_logs(comm_link)
    
    # 添加业务流程说明
    st.markdown("---")
    st.subheader("🔄 数据流业务流程")
    
    col_flow1, col_flow2 = st.columns(2)
    
    with col_flow1:
        st.markdown("""
        ### ⬇️ 下行命令流 (GCS → OBC → FCU)
        
        1. **地面站(GCS)** 生成控制命令
        2. 命令通过数据链路发送至 **机载计算机(OBC)**
        3. OBC 解析并验证命令
        4. 命令转发至 **飞行控制器(FCU)**
        5. FCU 执行命令并返回执行结果
        
        **典型命令示例：**
        - 起飞命令
        - 航点设置
        - 返航命令
        - 紧急停止
        """)
    
    with col_flow2:
        st.markdown("""
        ### ⬆️ 上行数据流 (FCU → OBC → GCS)
        
        1. **飞行控制器(FCU)** 采集遥测数据
        2. 数据打包发送至 **机载计算机(OBC)**
        3. OBC 进行数据融合与缓存
        4. 数据通过下行链路发送至 **地面站(GCS)**
        5. GCS 解析并显示实时状态
        
        **典型数据示例：**
        - 位置/姿态信息
        - 电池状态
        - 传感器数据
        - 任务进度
        """)
    
    # 链路质量指示器
    st.markdown("---")
    st.subheader("📊 实时链路质量")
    
    quality = comm_link.get_link_quality()
    
    col_q1, col_q2, col_q3 = st.columns(3)
    
    with col_q1:
        st.metric("整体链路质量", f"{quality['overall']:.0f}%")
    
    with col_q2:
        st.metric("GCS ↔ OBC 链路质量", f"{quality['GCS_OBC']}%")
    
    with col_q3:
        st.metric("OBC ↔ FCU 链路质量", f"{quality['OBC_FCU']}%")
    
    # 链路状态建议
    if quality['overall'] < 50:
        st.warning("⚠️ 链路质量较差，建议检查通信设备或切换信道")
    elif quality['overall'] < 80:
        st.info("📡 链路质量良好，可正常执行任务")
    else:
        st.success("✅ 链路质量优秀，通信稳定")

# 自动刷新通信链路状态（在飞行监控页面）
if page == "飞行监控" and st.session_state.mission_active:
    # 模拟遥测数据自动上报
    if random.random() < 0.3:  # 30%概率在飞行时上报数据
        mock_data = {"lat": 32.2322, "lng": 118.7490, "alt": 50, "speed": st.session_state.flight_speed}
        st.session_state.comm_link.send_telemetry("FCU", "GCS", "飞行状态", mock_data)
