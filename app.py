# 在文件开头的 session_state 初始化部分，修复障碍物数据结构转换

# 找到这部分代码（大约在第706行附近）：
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
    st.session_state.comm_link = CommunicationLink()
else:
    # 修复：确保障碍物数据格式正确
    if st.session_state.obstacles:
        # 检查第一个障碍物的格式
        if isinstance(st.session_state.obstacles[0], list):
            # 旧格式：直接是顶点列表，转换为新格式
            new_obs = []
            for poly in st.session_state.obstacles:
                if isinstance(poly, list):
                    new_obs.append({"vertices": poly, "height": st.session_state.default_obstacle_height})
                elif isinstance(poly, dict) and "vertices" in poly:
                    new_obs.append(poly)
            st.session_state.obstacles = new_obs
            save_obstacles_to_file(st.session_state.obstacles)
        elif isinstance(st.session_state.obstacles[0], dict) and "vertices" not in st.session_state.obstacles[0]:
            # 错误格式，重新初始化
            st.session_state.obstacles = []
    
    # 确保其他必要的键存在
    for key in ["mission_waypoints", "mission_active", "mission_paused", "mission_start_time",
                "current_waypoint_index", "aircraft_position", "flight_speed", "battery", "stop_mission", "comm_link"]:
        if key not in st.session_state:
            if key == "flight_speed":
                st.session_state.flight_speed = 8.5
            elif key == "battery":
                st.session_state.battery = 96.0
            elif key == "comm_link":
                st.session_state.comm_link = CommunicationLink()
            else:
                st.session_state[key] = None if key not in ["mission_active", "mission_paused", "stop_mission"] else False

# 添加一个调试函数来检查障碍物
def debug_obstacles():
    """调试函数：打印障碍物信息"""
    st.sidebar.divider()
    st.sidebar.subheader("🐛 调试信息")
    st.sidebar.write(f"障碍物数量: {len(st.session_state.obstacles)}")
    for idx, obs in enumerate(st.session_state.obstacles):
        st.sidebar.write(f"障碍物 {idx+1}: 类型={type(obs)}, 高度={obs.get('height', 'N/A') if isinstance(obs, dict) else '格式错误'}")
        if isinstance(obs, dict) and "vertices" in obs:
            st.sidebar.write(f"  顶点数: {len(obs['vertices'])}")
            st.sidebar.write(f"  前两个顶点: {obs['vertices'][:2] if obs['vertices'] else '无'}")

# 在航线规划页面的开头添加调试信息（可选）
# 在 if page == "航线规划": 下面添加：
if page == "航线规划":
    # 添加调试开关
    show_debug = st.sidebar.checkbox("显示调试信息", value=False, key="show_debug")
    if show_debug:
        debug_obstacles()
    
    st.header("🗺️ 航线规划 + 多障碍物可靠绕行 (左侧/右侧/自动/最优)")
    show_flame()
    
    # ... 其余代码 ...

# 修复 generate_detour_route 函数，添加调试输出
def generate_detour_route(A, B, obstacles, flight_height, safety_meters, detour_side="auto", max_attempts=3):
    # 添加调试信息
    st.write(f"调试: 障碍物数量 = {len(obstacles)}")
    st.write(f"调试: 飞行高度 = {flight_height}")
    
    # 过滤相关障碍物（高度大于飞行高度的）
    relevant = []
    for obs in obstacles:
        if isinstance(obs, dict) and "height" in obs and "vertices" in obs:
            obs_height = obs["height"]
            st.write(f"调试: 障碍物高度 = {obs_height}, 飞行高度 = {flight_height}")
            if flight_height < obs_height:
                relevant.append(obs)
                st.write(f"调试: 添加障碍物，高度 {obs_height} > {flight_height}")
    
    if not relevant:
        st.info("ℹ️ 没有高于飞行高度的障碍物，使用直线航线")
        return [A, B]
    
    st.write(f"调试: 相关障碍物数量 = {len(relevant)}")
    
    for attempt in range(max_attempts):
        current_safety = safety_meters * (1 + attempt * 0.5)
        st.write(f"调试: 尝试 {attempt + 1}, 安全距离 = {current_safety}")
        
        route = sequential_detour(A, B, relevant, flight_height, current_safety, detour_side, max_iters=10)
        
        # 验证路径
        ok = True
        for i in range(len(route)-1):
            for obs in relevant:
                if segment_collides_with_obstacle(obs, current_safety, route[i], route[i+1]):
                    st.write(f"调试: 线段 {i} 与障碍物碰撞")
                    ok = False
                    break
            if not ok:
                break
        
        if ok:
            st.write(f"调试: 找到有效路径，共 {len(route)} 个点")
            simplified = simplify_route(route, obstacles, flight_height, current_safety)
            result = safe_smooth_route(simplified, obstacles, flight_height, current_safety)
            return result
    
    st.error("❌ 所有尝试均失败，请增大安全距离或调整障碍物位置")
    return [A, B]

# 修复 segment_collides_with_obstacle 函数，添加安全检查
def segment_collides_with_obstacle(obs, safety_meters, seg_start, seg_end):
    """线段是否与障碍物的安全扩展区域相交"""
    try:
        # 确保 obs 是字典格式
        if not isinstance(obs, dict):
            st.warning(f"障碍物格式错误: {type(obs)}")
            return False
        
        if "vertices" not in obs:
            st.warning(f"障碍物缺少 vertices 字段")
            return False
        
        vertices = obs["vertices"]
        if not vertices or len(vertices) < 3:
            return False
        
        expanded = get_expanded_rect_polygon(obs, safety_meters)
        return polygon_intersects_segment(expanded, seg_start, seg_end)
    except Exception as e:
        st.warning(f"碰撞检测错误: {e}")
        return False

# 修复 get_expanded_rect_polygon 函数
def get_expanded_rect_polygon(obs, safety_meters):
    """根据障碍物bbox生成外扩安全距离的矩形（逆时针）"""
    try:
        vertices = obs["vertices"]
        minx, miny, maxx, maxy = get_bounding_box(vertices)
        center_lat = (miny + maxy) / 2.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(center_lat))
        expand_lon = safety_meters / meters_per_deg_lon if meters_per_deg_lon > 0 else safety_meters / 111320.0
        expand_lat = safety_meters / 111000.0
        minx -= expand_lon
        miny -= expand_lat
        maxx += expand_lon
        maxy += expand_lat
        return [(minx, miny), (minx, maxy), (maxx, maxy), (maxx, miny)]
    except Exception as e:
        st.error(f"扩展矩形计算错误: {e}")
        # 返回一个小矩形作为默认值
        return [(0, 0), (0, 0.001), (0.001, 0.001), (0.001, 0)]

# 在添加障碍物时，确保格式正确
# 找到添加障碍物的代码部分（大约在 folium 绘图回调处），修改为：
if output and output.get("last_active_drawing"):
    drawing = output["last_active_drawing"]
    geom_type = drawing.get("geometry", {}).get("type")
    coords = drawing.get("geometry", {}).get("coordinates")
    if geom_type == "Polygon" and coords:
        ring = coords[0]
        # 确保顶点是 (lng, lat) 格式
        poly_wgs84 = [(float(lng), float(lat)) for lng, lat in ring]
        # 检查是否已存在相同顶点
        exists = False
        for obs in st.session_state.obstacles:
            if isinstance(obs, dict) and "vertices" in obs:
                if obs["vertices"] == poly_wgs84:
                    exists = True
                    break
        if not exists:
            new_obs = {"vertices": poly_wgs84, "height": float(st.session_state.default_obstacle_height)}
            st.session_state.obstacles.append(new_obs)
            save_obstacles_to_file(st.session_state.obstacles)
            st.session_state.flash_message = ("success", f"已添加障碍物（高度 {new_obs['height']} m）")
            st.rerun()
    elif geom_type == "Rectangle" and coords:
        lng1, lat1 = coords[0]
        lng2, lat2 = coords[1]
        rect = [(float(lng1), float(lat1)), (float(lng2), float(lat1)), (float(lng2), float(lat2)), (float(lng1), float(lat2))]
        exists = False
        for obs in st.session_state.obstacles:
            if isinstance(obs, dict) and "vertices" in obs:
                if obs["vertices"] == rect:
                    exists = True
                    break
        if not exists:
            new_obs = {"vertices": rect, "height": float(st.session_state.default_obstacle_height)}
            st.session_state.obstacles.append(new_obs)
            save_obstacles_to_file(st.session_state.obstacles)
            st.session_state.flash_message = ("success", "已添加矩形障碍物")
            st.rerun()

# 在页面底部添加一个按钮来显示当前障碍物（用于调试）
if page == "航线规划":
    if st.sidebar.button("显示当前障碍物详情", key="show_obs_details"):
        st.sidewrite("### 当前障碍物列表")
        if not st.session_state.obstacles:
            st.sidebar.write("暂无障е物")
        else:
            for idx, obs in enumerate(st.session_state.obstacles):
                st.sidebar.write(f"**障碍物 {idx+1}:**")
                st.sidebar.write(f"  类型: {type(obs)}")
                if isinstance(obs, dict):
                    st.sidebar.write(f"  高度: {obs.get('height', 'N/A')}")
                    st.sidebar.write(f"  顶点数: {len(obs.get('vertices', []))}")
                    st.sidebar.write(f"  前三个顶点: {obs.get('vertices', [])[:3]}")
                else:
                    st.sidebar.write(f"  数据: {obs}")
