# ========== 通信链路拓扑与数据流（动态版）==========
st.subheader("📡 通信链路拓扑与数据流")

# 刷新按钮
col_refresh, _ = st.columns([1, 4])
with col_refresh:
    refresh_link = st.button("🔄 刷新链路状态", key="refresh_link_stats", use_container_width=True)

if refresh_link:
    # 更新 GCS ↔ OBC 链路统计
    packet_gobc = st.session_state.sim_gcs_obc.generate_packet()
    # 将模拟器的历史记录追加（用于计算丢包率）
    if "hist_gcs_obc" not in st.session_state:
        st.session_state.hist_gcs_obc = []
    st.session_state.hist_gcs_obc.append(packet_gobc)
    if len(st.session_state.hist_gcs_obc) > 100:
        st.session_state.hist_gcs_obc.pop(0)
    avg_rtt_gobc, loss_gobc = st.session_state.sim_gcs_obc.get_summary(st.session_state.hist_gcs_obc)
    st.session_state.link_stats["GCS-OBC"]["delay_ms"] = avg_rtt_gobc * 1000
    st.session_state.link_stats["GCS-OBC"]["loss_rate"] = loss_gobc
    st.session_state.link_stats["GCS-OBC"]["last_heartbeat"] = time.time()
    st.session_state.link_stats["GCS-OBC"]["normal"] = (loss_gobc < 0.1 and avg_rtt_gobc < 0.5)  # 丢包<10% 且 延迟<500ms

    # 更新 OBC ↔ FCU 链路统计
    packet_ofcu = st.session_state.sim_obc_fcu.generate_packet()
    if "hist_obc_fcu" not in st.session_state:
        st.session_state.hist_obc_fcu = []
    st.session_state.hist_obc_fcu.append(packet_ofcu)
    if len(st.session_state.hist_obc_fcu) > 100:
        st.session_state.hist_obc_fcu.pop(0)
    avg_rtt_ofcu, loss_ofcu = st.session_state.sim_obc_fcu.get_summary(st.session_state.hist_obc_fcu)
    st.session_state.link_stats["OBC-FCU"]["delay_ms"] = avg_rtt_ofcu * 1000
    st.session_state.link_stats["OBC-FCU"]["loss_rate"] = loss_ofcu
    st.session_state.link_stats["OBC-FCU"]["last_heartbeat"] = time.time()
    st.session_state.link_stats["OBC-FCU"]["normal"] = (loss_ofcu < 0.1 and avg_rtt_ofcu < 0.5)

# 显示节点状态（根据最后心跳时间判断在线，这里简化：只要有刷新动作且在最近5秒内即视为在线）
now_time = time.time()
gcs_online = (now_time - st.session_state.link_stats["GCS-OBC"]["last_heartbeat"]) < 5.0
obc_online = gcs_online and (now_time - st.session_state.link_stats["OBC-FCU"]["last_heartbeat"]) < 5.0
fcu_online = (now_time - st.session_state.link_stats["OBC-FCU"]["last_heartbeat"]) < 5.0

comm_col1, comm_col2, comm_col3 = st.columns(3)
with comm_col1:
    if gcs_online:
        st.success("✅ GCS 在线")
    else:
        st.error("❌ GCS 离线")
    st.caption("地面站\n192.168.1.100")
with comm_col2:
    if obc_online:
        st.success("✅ OBC 在线")
    else:
        st.error("❌ OBC 离线")
    st.caption("机载计算机\nRaspberry Pi 4")
with comm_col3:
    if fcu_online:
        st.success("✅ FCU 在线")
    else:
        st.error("❌ FCU 离线")
    st.caption("飞控\nPX4 / ArduPilot")

# 链路统计表
st.markdown("#### 链路统计")
link_col1, link_col2 = st.columns(2)
with link_col1:
    st.markdown("**GCS ↔ OBC**")
    stats = st.session_state.link_stats["GCS-OBC"]
    status_text = "🟢 正常" if stats["normal"] else "🔴 异常"
    st.metric("状态", status_text)
    st.metric("延迟", f"{stats['delay_ms']:.1f} ms" if stats['delay_ms'] > 0 else "--")
    st.metric("丢包率", f"{stats['loss_rate']*100:.1f}%" if stats['loss_rate'] > 0 else "--")
with link_col2:
    st.markdown("**OBC ↔ FCU**")
    stats = st.session_state.link_stats["OBC-FCU"]
    status_text = "🟢 正常" if stats["normal"] else "🔴 异常"
    st.metric("状态", status_text)
    st.metric("延迟", f"{stats['delay_ms']:.1f} ms" if stats['delay_ms'] > 0 else "--")
    st.metric("丢包率", f"{stats['loss_rate']*100:.1f}%" if stats['loss_rate'] > 0 else "--")

st.info("MAVLink 已连接 | 点击「刷新链路状态」更新实时数据")
