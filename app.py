
**📊 当前解析值**
| 字段 | 值 | 说明 |
|------|-----|------|
| 卫星数量 | {satellites} 颗 | 可见卫星 |
| 定位类型 | {fix_names.get(fix_type, '未知')} | 定位精度 |
| 水平精度 | 1.2 m | EPH |
| 垂直精度 | 1.5 m | EPV |

**🛰️ GPS状态**
- 定位精度: 良好
- 信号质量: {'强' if satellites > 10 else '中等'}
""")

with tab3:
st.markdown("#### 📈 报文流量统计")

col_chart1, col_chart2 = st.columns(2)

with col_chart1:
st.markdown("**📊 各类型报文频率**")
freq_data = {
    "报文类型": ["HEARTBEAT", "SYS_STATUS", "POSITION", "ATTITUDE", "VFR_HUD", "GPS_RAW"],
    "频率(Hz)": [1, 1, 10, 10, 10, 5]
}
freq_df = pd.DataFrame(freq_data)
st.bar_chart(freq_df.set_index("报文类型"), height=250)
st.caption("📌 HEARTBEAT(1Hz) | POSITION(10Hz) | ATTITUDE(10Hz) | VFR_HUD(10Hz)")

with col_chart2:
st.markdown("**📊 累计报文数量**")
count_data = {
    "报文类型": ["HEARTBEAT", "SYS_STATUS", "POSITION", "ATTITUDE", "VFR_HUD", "GPS_RAW"],
    "累计数量": [random.randint(30, 50), random.randint(30, 45),
                random.randint(140, 170), random.randint(130, 160),
                random.randint(120, 150), random.randint(60, 80)]
}
count_df = pd.DataFrame(count_data)
st.bar_chart(count_df.set_index("报文类型"), height=250)

st.markdown("---")
st.markdown("#### 🔄 MAVLink 数据流架构")
st.code("""
┌─────────────────────────────────────────────────────────────────────────────┐
│                         MAVLink 数据流架构                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   SITL/PX4 ──(UDP 14550)──▶ MAVLink Parser ──▶ 数据处理层 ──▶ 前端展示     │
│       │                          │                    │                     │
│       │                          │                    │                     │
│   ┌───▼───┐                ┌─────▼─────┐      ┌──────▼──────┐              │
│   │原始数据│                │ 消息解析   │      │  UI渲染     │              │
│   │MAVLink│ ──────────────▶ │ •HEARTBEAT│ ──▶ │ •仪表盘     │              │
│   │二进制 │                │ •SYS_STATUS│      │ •地图标记   │              │
│   │流     │                │ •POSITION  │      │ •状态指示   │              │
│   └───────┘                │ •ATTITUDE  │      │ •数据图表   │              │
│                            │ •VFR_HUD   │      │ •日志记录   │              │
│                            └────────────┘      └─────────────┘              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
""", language="text")

st.markdown("---")

# ==================== 系统节点状态 ====================
st.markdown("### 🖥️ 系统节点状态")
col1, col2, col3 = st.columns(3)

with col1:
gcs_status = "🟢 在线" if comm.gcs_online else "🔴 离线"
st.markdown(f"""
<div style="background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        border-radius: 15px; padding: 20px; text-align: center; color: white;">
<h2>📡 GCS</h2>
<h3>{gcs_status}</h3>
<p style="font-size: 12px; margin: 5px 0;">地面站</p>
<p style="font-size: 11px; opacity: 0.8;">{comm.gcs_ip}</p>
</div>
""", unsafe_allow_html=True)

with col2:
obc_status = "🟢 在线" if comm.obc_online else "🔴 离线"
st.markdown(f"""
<div style="background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
        border-radius: 15px; padding: 20px; text-align: center; color: white;">
<h2>💻 OBC</h2>
<h3>{obc_status}</h3>
<p style="font-size: 12px; margin: 5px 0;">机载计算机</p>
<p style="font-size: 11px; opacity: 0.8;">{comm.obc_ip} | Raspberry Pi 4</p>
</div>
""", unsafe_allow_html=True)

with col3:
fcu_status = "🟢 在线" if comm.fcu_online else "🔴 离线"
st.markdown(f"""
<div style="background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
        border-radius: 15px; padding: 20px; text-align: center; color: white;">
<h2>🎮 FCU</h2>
<h3>{fcu_status}</h3>
<p style="font-size: 12px; margin: 5px 0;">飞控</p>
<p style="font-size: 11px; opacity: 0.8;">{comm.fcu_ip} | PX4 / ArduPilot</p>
</div>
""", unsafe_allow_html=True)

st.markdown("---")

st.subheader("📊 链路统计")
stats = comm.get_statistics()

metric_cols = st.columns(4)
with metric_cols[0]:
st.metric("📤 发送包数", f"{stats['sent']:,}")
with metric_cols[1]:
st.metric("📥 接收包数", f"{stats['received']:,}")
with metric_cols[2]:
st.metric("❌ 丢包数", f"{stats['lost']:,}")
with metric_cols[3]:
success_rate = stats['success_rate']
st.metric("✅ 成功率", f"{success_rate:.1f}%")

stat_cols = st.columns(3)
with stat_cols[0]:
st.metric("⚡ GCS-OBC延迟", f"{stats['gcs_obc_latency']}ms")
with stat_cols[1]:
st.metric("⚡ OBC-FCU延迟", f"{stats['obc_fcu_latency']}ms")
with stat_cols[2]:
loss_rate = stats['packet_loss_rate'] * 100
st.metric("📉 丢包率", f"{loss_rate:.1f}%")

st.markdown("---")

st.subheader("🎮 链路控制")
control_cols = st.columns(4)
with control_cols[0]:
if st.button("🔄 重置统计", use_container_width=True, type="primary"):
comm.reset_statistics()
st.success("✅ 统计已重置")
st.rerun()
with control_cols[1]:
new_gcs_latency = st.slider("GCS-OBC延迟(ms)", 5, 100, comm.gcs_obc_latency, 5, key="gcs_latency")
if new_gcs_latency != comm.gcs_obc_latency:
comm.gcs_obc_latency = new_gcs_latency
with control_cols[2]:
new_obc_latency = st.slider("OBC-FCU延迟(ms)", 5, 100, comm.obc_fcu_latency, 5, key="obc_latency")
if new_obc_latency != comm.obc_fcu_latency:
comm.obc_fcu_latency = new_obc_latency
with control_cols[3]:
new_loss_rate = st.slider("丢包率(%)", 0.0, 5.0, comm.packet_loss_rate * 100, 0.1, key="loss_rate") / 100
if new_loss_rate != comm.packet_loss_rate:
comm.packet_loss_rate = new_loss_rate

st.markdown("---")

st.subheader("📋 通信日志")

col_flow1, col_flow2 = st.columns(2)
with col_flow1:
st.info("📤 **GCS → OBC → FCU**\n\n航线规划指令下发流程")
with col_flow2:
st.info("📥 **FCU → OBC → GCS**\n\n飞行状态上报流程")
st.markdown("---")

log_mode = st.radio("显示模式", ["📋 表格视图", "📝 详细视图"], horizontal=True)

if log_mode == "📋 表格视图":
if comm.logs:
log_data = []
for i, log in enumerate(comm.logs[:50]):
    log_data.append({
        "序号": i + 1,
        "时间": log.timestamp,
        "方向": log.direction,
        "消息": log.message,
        "详情": log.details if log.details else "-"
    })
df = pd.DataFrame(log_data)
st.dataframe(df, use_container_width=True, height=400)
else:
st.info("📭 暂无通信日志")

col_clear1, col_clear2, col_clear3 = st.columns([1, 1, 1])
with col_clear2:
if st.button("🗑️ 清空所有日志", use_container_width=True, type="secondary"):
    comm.logs.clear()
    comm.planning_records.clear()
    st.success("✅ 日志已清空")
    st.rerun()
else:
tab1, tab2, tab3 = st.tabs(["📤 下行指令 (GCS→OBC→FCU)", "📥 上行状态 (FCU→OBC→GCS)", "📋 规划记录"])

with tab1:
st.caption("航线规划指令下发流程")
gcs_obc_logs = [log for log in comm.logs if log.direction == "GCS→OBC"]
if gcs_obc_logs:
    st.markdown("#### 📡 GCS → OBC")
    for log in gcs_obc_logs[:15]:
        st.markdown(f"""
        <div style="background: #e3f2fd; border-left: 4px solid #2196f3; padding: 8px; margin: 5px 0; border-radius: 5px;">
            <code>[{log.timestamp}]</code> <strong>{log.message}</strong>
            {f'<br><span style="color: #666; font-size: 12px;">📝 {log.details}</span>' if log.details else ''}
        </div>
        """, unsafe_allow_html=True)
else:
    st.info("暂无 GCS → OBC 日志")

obc_fcu_logs = [log for log in comm.logs if log.direction == "OBC→FCU"]
if obc_fcu_logs:
    st.markdown("#### 🖥️ OBC → FCU")
    for log in obc_fcu_logs[:15]:
        st.markdown(f"""
        <div style="background: #e8f5e9; border-left: 4px solid #4caf50; padding: 8px; margin: 5px 0; border-radius: 5px;">
            <code>[{log.timestamp}]</code> <strong>{log.message}</strong>
            {f'<br><span style="color: #666; font-size: 12px;">📝 {log.details}</span>' if log.details else ''}
        </div>
        """, unsafe_allow_html=True)
else:
    st.info("暂无 OBC → FCU 日志")

with tab2:
st.caption("飞行状态上报流程")
fcu_obc_logs = [log for log in comm.logs if log.direction == "FCU→OBC"]
if fcu_obc_logs:
    st.markdown("#### 🎮 FCU → OBC")
    for log in fcu_obc_logs[:20]:
        st.markdown(f"""
        <div style="background: #fff3e0; border-left: 4px solid #ff9800; padding: 8px; margin: 5px 0; border-radius: 5px;">
            <code>[{log.timestamp}]</code> <strong>{log.message}</strong>
            {f'<br><span style="color: #666; font-size: 12px;">📝 {log.details}</span>' if log.details else ''}
        </div>
        """, unsafe_allow_html=True)
else:
    st.info("暂无 FCU → OBC 日志")

obc_gcs_logs = [log for log in comm.logs if log.direction == "OBC→GCS"]
if obc_gcs_logs:
    st.markdown("#### 💻 OBC → GCS")
    for log in obc_gcs_logs[:20]:
        st.markdown(f"""
        <div style="background: #f3e5f5; border-left: 4px solid #9c27b0; padding: 8px; margin: 5px 0; border-radius: 5px;">
            <code>[{log.timestamp}]</code> <strong>{log.message}</strong>
            {f'<br><span style="color: #666; font-size: 12px;">📝 {log.details}</span>' if log.details else ''}
        </div>
        """, unsafe_allow_html=True)
else:
    st.info("暂无 OBC → GCS 日志")

with tab3:
st.caption("航线规划记录")
if comm.planning_records:
    for record in comm.planning_records[:15]:
        st.markdown(f"""
        <div style="background: #e0f7fa; border-left: 4px solid #00bcd4; padding: 10px; margin: 8px 0; border-radius: 5px;">
            <code>[{record.get('timestamp', '')}]</code>
            <strong>✈️ {record.get('message', '')}</strong>
            {f'<br><span style="color: #006064; font-size: 12px;">📊 {record.get("details", "")}</span>' if record.get('details') else ''}
        </div>
        """, unsafe_allow_html=True)
else:
    st.info("暂无航线规划记录")

col_clear1, col_clear2, col_clear3 = st.columns([1, 1, 1])
with col_clear2:
if st.button("🗑️ 清空所有日志", use_container_width=True, type="secondary"):
    comm.logs.clear()
    comm.planning_records.clear()
    st.success("✅ 日志已清空")
    st.rerun()


# ==================== 航线规划页面 ====================
def render_planning_page(drone_speed: int, flight_alt: float, auto_save: bool):
st.header("🗺️ 航线规划")

tab1, tab2 = st.tabs(["✈️ 航线规划", "🔄 坐标转换工具"])

with tab1:
render_planning_tab(drone_speed, flight_alt, auto_save)
with tab2:
render_coordinate_conversion_tab()


def render_planning_tab(drone_speed: int, flight_alt: float, auto_save: bool):
blocked, high = check_straight_blocked(st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt)

status_cols = st.columns([2, 1])
with status_cols[0]:
if blocked:
st.warning(f"⚠️ 有 {high} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
else:
st.success("✅ 直线航线畅通无阻")
with status_cols[1]:
st.info(f"🛡️ 安全半径: {st.session_state.safety_radius}m")

st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")

col1, col2 = st.columns([1, 1.5])
with col1:
render_planning_controls(flight_alt, drone_speed, auto_save)
with col2:
render_planning_map_view(flight_alt, blocked)


def render_planning_controls(flight_alt: float, drone_speed: int, auto_save: bool):
with st.expander("📍 起点/终点设置", expanded=True):
render_point_settings()

with st.expander("🤖 路径规划策略", expanded=True):
render_path_strategy(flight_alt)

st.subheader("✈️ 飞行控制")
param_cols = st.columns(3)
with param_cols[0]:
st.metric("飞行高度", f"{flight_alt} m")
with param_cols[1]:
st.metric("速度系数", f"{drone_speed}%")
with param_cols[2]:
st.metric("安全半径", f"{st.session_state.safety_radius} m")

btn_col1, btn_col2 = st.columns(2)
with btn_col1:
if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
start_flight(flight_alt, drone_speed)
with btn_col2:
if st.button("⏹️ 停止飞行", use_container_width=True):
stop_flight()

st.markdown("---")
render_current_coords_info()


def render_current_coords_info():
st.subheader("📍 当前坐标信息")
a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']

coord_cols = st.columns(2)
with coord_cols[0]:
st.markdown(f"""
<div style="background: #f0f9f0; border-radius: 10px; padding: 10px; border-left: 4px solid #4CAF50;">
<span style="font-size: 12px; color: #666;">🟢 起点 A</span><br>
<code style="font-size: 12px;">经度: {a[0]:.8f}</code><br>
<code style="font-size: 12px;">纬度: {a[1]:.8f}</code>
</div>
""", unsafe_allow_html=True)

with coord_cols[1]:
st.markdown(f"""
<div style="background: #fff0f0; border-radius: 10px; padding: 10px; border-left: 4px solid #f44336;">
<span style="font-size: 12px; color: #666;">🔴 终点 B</span><br>
<code style="font-size: 12px;">经度: {b[0]:.8f}</code><br>
<code style="font-size: 12px;">纬度: {b[1]:.8f}</code>
</div>
""", unsafe_allow_html=True)

dist = math.hypot(b[0] - a[0], b[1] - a[1]) * 111000
info_cols = st.columns(2)
with info_cols[0]:
st.metric("📏 直线距离", f"{dist:.0f} 米")
with info_cols[1]:
if st.session_state.planned_path:
total_dist = calculate_path_length(st.session_state.planned_path) * 111000
delta_dist = total_dist - dist
st.metric("🛣️ 规划路径总长", f"{total_dist:.0f} 米", delta=f"+{delta_dist:.0f}m" if delta_dist > 0 else None)
else:
st.metric("🛣️ 规划路径总长", "未规划")

if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
waypoint_count = len(st.session_state.planned_path) - 2
st.caption(f"🎯 包含 {waypoint_count} 个绕行航点")


def render_point_settings():
st.markdown("#### 🎯 设置方式")
mode = st.radio("选择方式", ["✏️ 经纬度输入", "🖱️ 鼠标点击"], horizontal=True, key="point_setting_mode", label_visibility="collapsed")

if mode == "✏️ 经纬度输入":
render_coordinate_input()
else:
render_mouse_click_setting()


def render_coordinate_input():
st.markdown("**🟢 起点 A**")
col_a1, col_a2, col_a3 = st.columns([1, 1, 1])
with col_a1:
a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat", step=0.000001, label_visibility="collapsed", placeholder="纬度")
with col_a2:
a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng", step=0.000001, label_visibility="collapsed", placeholder="经度")
with col_a3:
if st.button("📍 设置A点", use_container_width=True, key="set_a"):
st.session_state.points_gcj['A'] = [a_lng, a_lat]
update_path_after_point_change()
st.success("✅ 起点已更新")
st.rerun()

st.markdown("**🔴 终点 B**")
col_b1, col_b2, col_b3 = st.columns([1, 1, 1])
with col_b1:
b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat", step=0.000001, label_visibility="collapsed", placeholder="纬度")
with col_b2:
b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng", step=0.000001, label_visibility="collapsed", placeholder="经度")
with col_b3:
if st.button("📍 设置B点", use_container_width=True, key="set_b"):
st.session_state.points_gcj['B'] = [b_lng, b_lat]
update_path_after_point_change()
st.success("✅ 终点已更新")
st.rerun()

col_r1, col_r2 = st.columns(2)
with col_r1:
if st.button("🔄 重置默认起点", use_container_width=True):
st.session_state.points_gcj['A'] = config.DEFAULT_A_GCJ.copy()
update_path_after_point_change()
st.rerun()
with col_r2:
if st.button("🔄 重置默认终点", use_container_width=True):
st.session_state.points_gcj['B'] = config.DEFAULT_B_GCJ.copy()
update_path_after_point_change()
st.rerun()


def render_mouse_click_setting():
st.info("💡 点击地图上的任意位置设置起点或终点")

col1, col2 = st.columns(2)
with col1:
if st.button("🎯 设置起点", use_container_width=True, type="primary"):
st.session_state.waiting_for_start_point = True
st.session_state.waiting_for_end_point = False
st.rerun()
with col2:
if st.button("📍 设置终点", use_container_width=True, type="primary"):
st.session_state.waiting_for_end_point = True
st.session_state.waiting_for_start_point = False
st.rerun()

if st.session_state.waiting_for_start_point:
st.warning("⏳ 等待设置起点... 请点击地图")
elif st.session_state.waiting_for_end_point:
st.warning("⏳ 等待设置终点... 请点击地图")

if st.session_state.waiting_for_start_point or st.session_state.waiting_for_end_point:
if st.button("❌ 取消", use_container_width=True):
st.session_state.waiting_for_start_point = False
st.session_state.waiting_for_end_point = False
st.rerun()


def update_path_after_point_change():
st.session_state.planned_path = create_avoidance_path(
st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
st.session_state.current_direction, st.session_state.safety_radius)


def render_path_strategy(flight_alt: float):
st.markdown("**绕行方向**")
col1, col2, col3 = st.columns(3)

with col1:
is_best = st.session_state.current_direction == "最佳航线"
if st.button("🔄 最佳航线", use_container_width=True, type="primary" if is_best else "secondary"):
st.session_state.current_direction = "最佳航线"
st.session_state.planned_path = create_avoidance_path(
    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
    st.session_state.obstacles_gcj, flight_alt, "最佳航线", st.session_state.safety_radius)
st.success("✅ 已切换到最佳航线")
st.rerun()

with col2:
is_left = st.session_state.current_direction == "向左绕行"
if st.button("⬅️ 向左绕行", use_container_width=True, type="primary" if is_left else "secondary"):
st.session_state.current_direction = "向左绕行"
st.session_state.planned_path = create_avoidance_path(
    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
    st.session_state.obstacles_gcj, flight_alt, "向左绕行", st.session_state.safety_radius)
st.success("✅ 已切换到向左绕行")
st.rerun()

with col3:
is_right = st.session_state.current_direction == "向右绕行"
if st.button("➡️ 向右绕行", use_container_width=True, type="primary" if is_right else "secondary"):
st.session_state.current_direction = "向右绕行"
st.session_state.planned_path = create_avoidance_path(
    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
    st.session_state.obstacles_gcj, flight_alt, "向右绕行", st.session_state.safety_radius)
st.success("✅ 已切换到向右绕行")
st.rerun()

st.info(f"📌 当前策略: **{st.session_state.current_direction}**")

if st.button("🔄 重新规划路径", use_container_width=True):
with st.spinner("规划中..."):
st.session_state.planned_path = create_avoidance_path(
    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
    st.session_state.obstacles_gcj, flight_alt,
    st.session_state.current_direction, st.session_state.safety_radius)
if st.session_state.planned_path:
st.success("✅ 路径已重新规划")
st.rerun()


def start_flight(flight_alt: float, drone_speed: int):
if not st.session_state.points_gcj['A'] or not st.session_state.points_gcj['B']:
st.error("请先设置起点和终点")
return

path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
comm = st.session_state.comm_sim
total = calculate_path_length(path) * 111000

comm.add_planning_record({"message": "开始航线规划", "details": f"障碍物数量: {len(st.session_state.obstacles_gcj)}"})
comm.add_planning_record({"message": "航线规划完成", "details": f"航点数: {len(path)} | 路径长度: {total:.1f}m"})
comm.add_planning_record({"message": "导航目标", "details": f"起点→终点 | 目标高度: {flight_alt}m"})
comm.send_message("GCS", "OBC", "START_MISSION")
comm.send_message("OBC", "FCU", "UPLOAD_MISSION", f"航点数量: {len(path)}")

st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
st.session_state.simulation_running = True
st.session_state.flight_history = []

comm.send_message("FCU", "OBC", "ACK", "Mode: AUTO")
comm.send_message("OBC", "GCS", "ACK", "任务已开始")

waypoint_msg = f'路径中有 {len(path)-2} 个绕行点' if len(path) > 2 else '直线飞行'
st.success(f"🚁 飞行已开始！{waypoint_msg}")
st.rerun()


def stop_flight():
st.session_state.simulation_running = False
st.session_state.heartbeat_sim.simulating = False
st.session_state.comm_sim.send_message("GCS", "OBC", "STOP_MISSION", "用户停止飞行")
st.info("✈️ 飞行已停止")
st.rerun()


def render_planning_map_view(flight_alt: float, straight_blocked: bool):
st.subheader("🗺️ 规划地图")

if straight_blocked:
st.caption(f"🎯 当前避障策略: {st.session_state.current_direction}")
st.caption("🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行 | 🔵 蓝色=安全半径")

flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
center = st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ

if st.session_state.planned_path is None:
st.session_state.planned_path = create_avoidance_path(
st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
st.session_state.obstacles_gcj, flight_alt,
st.session_state.current_direction, st.session_state.safety_radius)

drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None

m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj,
                flight_trail, st.session_state.planned_path, straight_blocked,
                flight_alt, drone_pos, st.session_state.current_direction,
                st.session_state.safety_radius)

output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing", "last_clicked"])
handle_map_click(output)
handle_drawing_output(output)


def handle_map_click(output):
if output and output.get("last_clicked"):
clicked = output["last_clicked"]
if clicked and isinstance(clicked, dict):
lng = clicked.get("lng")
lat = clicked.get("lat")
if lng is not None and lat is not None:
    if st.session_state.waiting_for_start_point:
        st.session_state.points_gcj['A'] = [lng, lat]
        update_path_after_point_change()
        st.session_state.waiting_for_start_point = False
        st.success(f"✅ 起点已设置: ({lng:.6f}, {lat:.6f})")
        st.rerun()
    elif st.session_state.waiting_for_end_point:
        st.session_state.points_gcj['B'] = [lng, lat]
        update_path_after_point_change()
        st.session_state.waiting_for_end_point = False
        st.success(f"✅ 终点已设置: ({lng:.6f}, {lat:.6f})")
        st.rerun()


def handle_drawing_output(output):
if output and output.get("last_active_drawing"):
last = output["last_active_drawing"]
if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
coords = last["geometry"].get("coordinates", [])
if coords and len(coords) > 0:
    poly = [[p[0], p[1]] for p in coords[0]]
    if len(poly) >= 3 and st.session_state.pending_obstacle is None:
        if validate_polygon(poly):
            st.session_state.pending_obstacle = poly
            st.rerun()

if st.session_state.pending_obstacle is not None:
render_obstacle_dialog()


def render_obstacle_dialog():
st.markdown("---")
st.subheader("📝 添加新障碍物")
st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")

col1, col2 = st.columns(2)
with col1:
new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
with col2:
new_height = st.number_input("障碍物高度 (米)", 1, 200, 30, 5, key="height_input")

col_ok, col_cancel = st.columns(2)
with col_ok:
if st.button("✅ 确认添加", use_container_width=True, type="primary"):
new_obstacle = {
    "name": new_name,
    "polygon": st.session_state.pending_obstacle,
    "height": new_height,
    "selected": False,
    "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{len(st.session_state.obstacles_gcj)}",
    "created_time": datetime.now().isoformat()
}
st.session_state.obstacles_gcj.append(new_obstacle)
if st.session_state.auto_backup:
    save_obstacles(st.session_state.obstacles_gcj)
st.session_state.planned_path = create_avoidance_path(
    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
    st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
    st.session_state.current_direction, st.session_state.safety_radius)
st.session_state.pending_obstacle = None
st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
st.rerun()
with col_cancel:
if st.button("❌ 取消", use_container_width=True):
st.session_state.pending_obstacle = None
st.rerun()


# ==================== 坐标转换工具标签页 ====================
def render_coordinate_conversion_tab():
st.subheader("🔄 WGS-84 ↔ GCJ-02 坐标转换")
st.caption("WGS-84 (GPS) ↔ GCJ-02 (高德/腾讯/谷歌中国)")

convert_type = st.radio("转换模式", ["📍 单点转换", "📊 批量转换"], horizontal=True, key="conv_type")
st.markdown("---")

if convert_type == "📍 单点转换":
render_single_point_conversion()
else:
render_batch_conversion()


def render_single_point_conversion():
col1, col2 = st.columns([1, 1])

with col1:
st.markdown("#### 📥 输入坐标")
direction = st.radio("方向", ["WGS-84 → GCJ-02", "GCJ-02 → WGS-84"], horizontal=True, key="single_direction")
lng = st.number_input("经度", value=118.748726, format="%.6f", key="single_lng")
lat = st.number_input("纬度", value=32.233881, format="%.6f", key="single_lat")

if st.button("🔄 执行转换", type="primary", use_container_width=True):
try:
    if direction == "WGS-84 → GCJ-02":
        out_lng, out_lat = CoordinateConverter.wgs84_to_gcj02(lng, lat)
        st.session_state.conv_result = {
            "input": (lng, lat), "output": (out_lng, out_lat),
            "direction": "WGS-84 → GCJ-02"
        }
    else:
        out_lng, out_lat = CoordinateConverter.gcj02_to_wgs84(lng, lat)
        st.session_state.conv_result = {
            "input": (lng, lat), "output": (out_lng, out_lat),
            "direction": "GCJ-02 → WGS-84"
        }
except Exception as e:
    st.error(f"转换失败: {e}")

with col2:
st.markdown("#### 📤 转换结果")
if st.session_state.get("conv_result"):
res = st.session_state.conv_result
st.success(f"**{res['direction']}**")

delta_lng = res['output'][0] - res['input'][0]
delta_lat = res['output'][1] - res['input'][1]
delta_lng_m = delta_lng * 111000 * math.cos(math.radians(res['input'][1]))
delta_lat_m = delta_lat * 111000

st.markdown(f"""
| 项目 | 经度 | 纬度 |
|------|------|------|
| 输入 | `{res['input'][0]:.8f}` | `{res['input'][1]:.8f}` |
| 输出 | `{res['output'][0]:.8f}` | `{res['output'][1]:.8f}` |
| 偏移 | {delta_lng_m:.2f}米 | {delta_lat_m:.2f}米 |
""")

st.markdown("---")
st.markdown("#### 🎯 应用到航线")
col_apply1, col_apply2 = st.columns(2)
with col_apply1:
    if st.button("📌 设为起点", use_container_width=True):
        st.session_state.points_gcj['A'] = [res['output'][0], res['output'][1]]
        update_path_after_point_change()
        st.success("✅ 已设为起点")
        st.rerun()
with col_apply2:
    if st.button("📍 设为终点", use_container_width=True):
        st.session_state.points_gcj['B'] = [res['output'][0], res['output'][1]]
        update_path_after_point_change()
        st.success("✅ 已设为终点")
        st.rerun()
else:
st.info("点击「执行转换」查看结果")


def render_batch_conversion():
st.markdown("#### 📥 输入坐标")
st.caption("每行格式：经度,纬度")

col1, col2 = st.columns([1, 1])

with col1:
direction = st.radio("方向", ["WGS-84 → GCJ-02", "GCJ-02 → WGS-84"], horizontal=True, key="batch_direction")
batch_input = st.text_area("坐标列表", height=250,
                       placeholder="118.748726,32.233881\n118.750110,32.235460\n118.749000,32.234000",
                       key="batch_input")

if st.button("📊 执行批量转换", type="primary", use_container_width=True):
if batch_input.strip():
    lines = batch_input.strip().split('\n')
    coords = []
    invalid_lines = []

    for i, line in enumerate(lines, 1):
        line = line.strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) >= 2:
            try:
                lng = float(parts[0].strip())
                lat = float(parts[1].strip())
                coords.append((lng, lat))
            except ValueError:
                invalid_lines.append(i)
        else:
            invalid_lines.append(i)

    if invalid_lines:
        st.warning(f"跳过无效行: 第{', '.join(map(str, invalid_lines))}行")

    if coords:
        try:
            if direction == "WGS-84 → GCJ-02":
                results = CoordinateConverter.convert_batch(coords, "wgs84_to_gcj02")
            else:
                results = CoordinateConverter.convert_batch(coords, "gcj02_to_wgs84")

            st.session_state.batch_result = {
                "input": coords, "output": results, "direction": direction
            }
        except Exception as e:
            st.error(f"批量转换失败: {e}")

with col2:
st.markdown("#### 📤 转换结果")
if st.session_state.get("batch_result"):
res = st.session_state.batch_result
st.success(f"**{res['direction']}** - 共 {len(res['input'])} 个点")

result_data = []
for i, (in_coord, out_coord) in enumerate(zip(res['input'], res['output'])):
    delta_lng = out_coord[0] - in_coord[0]
    delta_lat = out_coord[1] - in_coord[1]
    delta_lng_m = delta_lng * 111000 * math.cos(math.radians(in_coord[1]))
    delta_lat_m = delta_lat * 111000

    result_data.append({
        "序号": i + 1,
        "输入经度": f"{in_coord[0]:.8f}",
        "输入纬度": f"{in_coord[1]:.8f}",
        "输出经度": f"{out_coord[0]:.8f}",
        "输出纬度": f"{out_coord[1]:.8f}",
        "Δ经度(米)": f"{delta_lng_m:.2f}",
        "Δ纬度(米)": f"{delta_lat_m:.2f}"
    })

df = pd.DataFrame(result_data)
st.dataframe(df, use_container_width=True, height=250)

csv = df.to_csv(index=False, encoding='utf-8-sig')
st.download_button(label="📥 导出CSV", data=csv,
                   file_name=f"coordinate_conversion_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                   mime="text/csv", use_container_width=True)
else:
st.info("点击「执行批量转换」查看结果")


# ==================== 飞行监控页面 ====================
def render_flight_monitoring_page(flight_alt: float, drone_speed: int):
st.header("📡 飞行监控 - 实时心跳包")

with st.expander("📡 MAVLink 接口规划文档 (预留接口说明)", expanded=False):
render_mavlink_interface_plan()

st.markdown("---")

auto_refresh = st.checkbox("🔄 自动刷新 (2秒)", value=True, key="auto_refresh_monitor")

col_refresh1, col_refresh2 = st.columns([1, 3])
with col_refresh1:
if st.button("🔄 手动刷新", use_container_width=True):
st.rerun()

update_flight_simulation()

if st.session_state.heartbeat_sim.history:
latest = st.session_state.heartbeat_sim.history[0]

current_waypoint = 0
total_waypoints = 0

if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
total_waypoints = len(st.session_state.planned_path)

if latest.arrived:
    current_waypoint = total_waypoints
else:
    if latest.progress >= 0:
        segment_count = len(st.session_state.planned_path) - 1
        segment_index = int(latest.progress * segment_count)
        segment_index = min(segment_index, segment_count - 1) if segment_count > 0 else 0

        if latest.progress >= 0.999:
            current_waypoint = total_waypoints
        else:
            current_waypoint = segment_index + 1

        current_waypoint = min(current_waypoint, total_waypoints)
        current_waypoint = max(current_waypoint, 1)
    else:
        current_waypoint = 1

waypoint_progress_value = current_waypoint / total_waypoints if total_waypoints > 0 else 0
remaining_distance = max(0, latest.remaining_distance if not latest.arrived else 0)

estimated_arrival = "00:00" if latest.arrived else "计算中..."
if not latest.arrived and latest.speed > 0 and remaining_distance > 0:
eta_seconds = remaining_distance / latest.speed
if eta_seconds < 60:
    estimated_arrival = f"{eta_seconds:.0f}秒"
elif eta_seconds < 3600:
    estimated_arrival = f"{int(eta_seconds // 60):02d}:{int(eta_seconds % 60):02d}"
else:
    estimated_arrival = f"{int(eta_seconds // 3600):02d}:{int((eta_seconds % 3600) // 60):02d}"

max_flight_time = 1800
battery_percentage = max(0, min(100, (1 - latest.flight_time / max_flight_time) * 100))
if latest.voltage:
voltage_percentage = ((latest.voltage - 21.0) / (22.2 - 21.0)) * 100
battery_percentage = max(0, min(100, (battery_percentage + voltage_percentage) / 2))

st.markdown("### ✈️ 飞行进度")
st.progress(latest.progress if not latest.arrived else 1.0,
        text=f"飞行进度：{int(latest.progress*100) if not latest.arrived else 100}%")

st.markdown("### 📊 实时飞行数据")
c1, c2, c3 = st.columns(3)
with c1:
waypoint_display = f"{current_waypoint} / {total_waypoints}"
if total_waypoints > 0:
    st.metric("🎯 当前航点", waypoint_display,
              delta=f"进度 {int(waypoint_progress_value*100)}%" if not latest.arrived else "已完成")
    st.progress(waypoint_progress_value, text=f"航点进度: {int(waypoint_progress_value*100)}%")

    if not latest.arrived and current_waypoint < total_waypoints:
        next_wp = st.session_state.planned_path[current_waypoint]
        st.caption(f"📍 下一航点: ({next_wp[0]:.6f}, {next_wp[1]:.6f})")
    elif not latest.arrived and current_waypoint == total_waypoints:
        st.caption("🎯 即将到达终点")
else:
    st.metric("🎯 当前航点", "0 / 0")
with c2:
st.metric("💨 飞行速度", f"{latest.speed:.1f} m/s", delta=f"{drone_speed}% 系数" if not latest.arrived else "已到达")
with c3:
st.metric("⏰ 已用时间", f"{int(latest.flight_time//60):02d}:{int(latest.flight_time%60):02d}")

c4, c5, c6 = st.columns(3)
with c4:
distance_text = f"{remaining_distance/1000:.2f} km" if remaining_distance >= 1000 else f"{remaining_distance:.0f} m"
st.metric("📏 剩余距离", distance_text if not latest.arrived else "0 m", delta="已到达!" if latest.arrived else None)
with c5:
st.metric("🕐 预计到达", estimated_arrival)
if remaining_distance < 100 and remaining_distance > 0 and not latest.arrived:
    st.info("🏁 即将到达目的地！")
elif latest.arrived:
    st.success("✅ 已到达目的地！")
with c6:
battery_color = "🟢" if battery_percentage > 50 else "🟡" if battery_percentage > 20 else "🔴"
st.metric("🔋 电量模拟", f"{battery_color} {battery_percentage:.0f}%", delta=f"{latest.voltage:.1f}V")

st.markdown("### 📍 位置与状态")
c7, c8, c9, c10 = st.columns(4)
with c7:
st.metric("📍 当前位置", f"{latest.lat:.6f}, {latest.lng:.6f}")
with c8:
st.metric("📏 飞行高度", f"{latest.altitude} m")
with c9:
st.metric("🛰️ 卫星数量", f"{latest.satellites} 颗")
with c10:
status = "✅ 已完成" if latest.arrived else "✈️ 飞行中" if st.session_state.simulation_running else "⏸️ 已停止"
st.metric("📌 飞行状态", status)

if latest.safety_violation and not latest.arrived:
st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
if latest.arrived:
st.success("🎉 无人机已到达目的地！飞行任务完成！")

with st.expander("📊 飞行任务总结", expanded=True):
c_sum1, c_sum2, c_sum3 = st.columns(3)
with c_sum1:
    st.metric("总飞行时间", f"{int(latest.flight_time//60):02d}:{int(latest.flight_time%60):02d}")
with c_sum2:
    total_distance = st.session_state.heartbeat_sim.total_distance * 111000
    st.metric("总飞行距离", f"{total_distance:.0f} m")
with c_sum3:
    avg_speed = latest.speed if latest.speed > 0 else drone_speed * config.BASE_SPEED_MPS / 100
    st.metric("平均速度", f"{avg_speed:.1f} m/s")

st.markdown("---")
st.markdown("### 🗺️ 实时位置追踪 & 🎮 飞行控制")
col_left, col_right = st.columns([2, 1])
with col_left:
display_monitor_map(flight_alt, latest)
with col_right:
st.markdown("#### 🎮 飞行控制")
p1, p2 = st.columns(2)
with p1:
    st.metric("当前飞行高度", f"{latest.altitude} m")
    st.metric("速度系数", f"{drone_speed}%")
with p2:
    st.metric("安全半径", f"{st.session_state.safety_radius} 米")
if st.session_state.planned_path:
    st.metric("🎯 绕行点数量", len(st.session_state.planned_path) - 2)
    total_dist = calculate_path_length(st.session_state.planned_path) * 111000
    st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")

st.markdown("**📍 当前坐标**")
a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
st.write(f"🟢 A点: ({a[0]:.6f}, {a[1]:.6f})")
st.write(f"🔴 B点: ({b[0]:.6f}, {b[1]:.6f})")
dist = math.hypot(b[0] - a[0], b[1] - a[1]) * 111000
st.caption(f"📏 直线距离: {dist:.0f} 米")
st.caption(f"🛡️ 当前安全半径: {st.session_state.safety_radius} 米")

col_btn1, col_btn2 = st.columns(2)
with col_btn1:
    if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
        if a and b:
            path = st.session_state.planned_path or [a, b]
            comm = st.session_state.comm_sim
            total = calculate_path_length(path) * 111000
            comm.add_planning_record({"message": "开始航线规划", "details": f"算法: A* | 障碍物数量: {len(st.session_state.obstacles_gcj)}"})
            comm.add_planning_record({"message": "航线规划完成", "details": f"类型: horizontal | 航点数: {len(path)} | 路径长度: {total:.1f}m"})
            comm.add_planning_record({"message": "导航目标", "details": f"起点: {a} | 终点: {b} | 目标高度: {flight_alt}m"})
            comm.send_message("GCS", "OBC", "START_MISSION", f"起点: {a}, 终点: {b}")
            comm.send_message("OBC", "FCU", "UPLOAD_MISSION", f"航点数量: {len(path)}")
            st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
            st.session_state.simulation_running = True
            st.session_state.flight_history = []
            comm.send_message("FCU", "OBC", "ACK", "Mode: AUTO")
            comm.send_message("OBC", "GCS", "ACK", "任务已开始")
            st.success(f"🚁 飞行已开始！{'路径中有' + str(len(path)-2) + '个绕行点' if len(path)>2 else '直线飞行'}")
            st.rerun()
        else:
            st.error("请先设置起点和终点")
with col_btn2:
    if st.button("⏹️ 停止飞行", use_container_width=True):
        st.session_state.simulation_running = False
        st.session_state.heartbeat_sim.simulating = False
        st.session_state.comm_sim.send_message("GCS", "OBC", "STOP_MISSION", "用户停止飞行")
        st.info("飞行已停止")
        st.rerun()

st.markdown("**📊 数据导出**")
col_exp1, col_exp2 = st.columns(2)
with col_exp1:
if st.button("📊 导出飞行数据", use_container_width=True):
    df = st.session_state.heartbeat_sim.export_flight_data()
    if not df.empty:
        csv = df.to_csv(index=False)
        st.download_button(label="📥 下载CSV", data=csv,
                           file_name=f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                           mime="text/csv")
with col_exp2:
if st.button("📊 导出航点数据", use_container_width=True) and st.session_state.planned_path:
    waypoint_data = [{"航点序号": i+1, "航点类型": "起点" if i==0 else "终点" if i==len(st.session_state.planned_path)-1 else f"绕行点{i}",
                     "经度": wp[0], "纬度": wp[1]} for i, wp in enumerate(st.session_state.planned_path)]
    csv = pd.DataFrame(waypoint_data).to_csv(index=False, encoding='utf-8-sig')
    st.download_button(label="📥 下载航点CSV", data=csv,
                       file_name=f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                       mime="text/csv")

st.markdown("---")
st.markdown("### 📈 实时数据图表")
c_ch1, c_ch2 = st.columns(2)
with c_ch1:
st.subheader("📊 速度 vs 时间")
if len(st.session_state.heartbeat_sim.history) > 1:
    speed_data = [{"时间(s)": i * config.HEARTBEAT_INTERVAL, "速度(m/s)": h.speed}
                  for i, h in enumerate(st.session_state.heartbeat_sim.history[:30])]
    st.line_chart(pd.DataFrame(speed_data), x="时间(s)", y="速度(m/s)")
with c_ch2:
st.subheader("📏 剩余距离 vs 时间")
if len(st.session_state.heartbeat_sim.history) > 1:
    dist_data = [{"时间(s)": i * config.HEARTBEAT_INTERVAL, "剩余距离(m)": max(0, h.remaining_distance)}
                 for i, h in enumerate(st.session_state.heartbeat_sim.history[:30])]
    st.line_chart(pd.DataFrame(dist_data), x="时间(s)", y="剩余距离(m)")

c_ch3, c_ch4 = st.columns(2)
with c_ch3:
st.subheader("🔋 电量模拟 vs 时间")
if len(st.session_state.heartbeat_sim.history) > 1:
    battery_data = []
    for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
        hist_battery = max(0, min(100, (1 - h.flight_time / 1800) * 100))
        if h.voltage:
            hist_voltage_pct = ((h.voltage - 21.0) / (22.2 - 21.0)) * 100
            hist_battery = max(0, min(100, (hist_battery + hist_voltage_pct) / 2))
        battery_data.append({"时间(s)": i * config.HEARTBEAT_INTERVAL, "电量(%)": hist_battery})
    st.line_chart(pd.DataFrame(battery_data), x="时间(s)", y="电量(%)")
st.caption("💡 电量基于电压和飞行时间综合计算")
with c_ch4:
st.subheader("🎯 航点进度")
if len(st.session_state.heartbeat_sim.history) > 1 and total_waypoints > 0:
    waypoint_data = []
    for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
        if h.arrived:
            hist_waypoint = total_waypoints
        else:
            segment_count = total_waypoints - 1
            if segment_count > 0 and h.progress >= 0:
                segment_index = int(h.progress * segment_count)
                segment_index = min(segment_index, segment_count - 1) if segment_count > 0 else 0
                if h.progress >= 0.999:
                    hist_waypoint = total_waypoints
                else:
                    hist_waypoint = segment_index + 1
                hist_waypoint = min(hist_waypoint, total_waypoints)
                hist_waypoint = max(hist_waypoint, 1)
            else:
                hist_waypoint = 1
        waypoint_data.append({"时间(s)": i * config.HEARTBEAT_INTERVAL, "已完成航点": hist_waypoint})
    st.line_chart(pd.DataFrame(waypoint_data), x="时间(s)", y="已完成航点")

st.markdown("---")
st.markdown("### 📋 飞行日志记录")
display_flight_history()
else:
st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
col_tip1, col_tip2, col_tip3 = st.columns(3)
with col_tip1:
st.info("💡 提示1：先在航线规划页面设置起点和终点")
with col_tip2:
st.info("💡 提示2：设置飞行高度和速度系数")
with col_tip3:
st.info("💡 提示3：点击「开始飞行」按钮启动模拟")

if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
st.markdown("---")
st.subheader("🗺️ 规划航线预览")
st.success(f"📌 已规划 {len(st.session_state.planned_path)} 个航点（包括起点和终点），点击开始飞行后将按此航线飞行")
with st.expander("📋 查看详细航点列表"):
waypoint_table = [{"序号": i+1, "类型": "🚁 起点" if i==0 else "🏁 终点" if i==len(st.session_state.planned_path)-1 else f"📍 绕行点 {i}",
                  "经度": f"{wp[0]:.6f}", "纬度": f"{wp[1]:.6f}"} for i, wp in enumerate(st.session_state.planned_path)]
st.table(pd.DataFrame(waypoint_table))

if auto_refresh and st.session_state.simulation_running:
import time
time.sleep(2)
st.rerun()


def display_monitor_map(flight_alt: float, latest):
tiles = config.GAODE_SATELLITE_URL
m = folium.Map(location=[latest.lat, latest.lng], zoom_start=18, tiles=tiles, attr="高德卫星地图")

for obs in st.session_state.obstacles_gcj:
coords = obs.get('polygon', [])
height = obs.get('height', 30)
if coords and len(coords) >= 3:
color = "red" if height > flight_alt else "orange"
folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=2, fill=True,
              fill_opacity=0.3, popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)

if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
line_color = "purple" if "向左" in st.session_state.current_direction else "orange" if "向右" in st.session_state.current_direction else "green"
folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color=line_color,
           weight=3, opacity=0.7, popup=f"规划航线 - {st.session_state.current_direction}").add_to(m)

folium.Circle(radius=st.session_state.safety_radius, location=[latest.lat, latest.lng],
     color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2,
     popup=f"🛡️ 安全半径: {st.session_state.safety_radius}米").add_to(m)

trail = [[hb.lat, hb.lng] for hb in st.session_state.heartbeat_sim.history[:50] if hb.lat and hb.lng]
if len(trail) > 1:
folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史飞行轨迹").add_to(m)

folium.Marker([latest.lat, latest.lng], popup=f"当前位置\n高度: {latest.altitude}m\n速度: {latest.speed}m/s",
     icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)

if st.session_state.points_gcj['A']:
folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], popup="起点 A",
         icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(m)
if st.session_state.points_gcj['B']:
folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], popup="终点 B",
         icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(m)

if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
for i, point in enumerate(st.session_state.planned_path[1:-1]):
folium.CircleMarker([point[1], point[0]], radius=4, color="yellow", fill=True,
                   fill_color="yellow", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)

folium_static(m, width=900, height=500)


def display_flight_history():
df = st.session_state.heartbeat_sim.export_flight_data()
if not df.empty:
display_cols = ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'voltage', 'satellites', 'remaining_distance']
display_cols = [c for c in display_cols if c in df.columns]
rename = {'timestamp': '时间', 'flight_time': '飞行时间(s)', 'lat': '纬度', 'lng': '经度',
      'altitude': '高度(m)', 'speed': '速度(m/s)', 'voltage': '电压(V)', 'satellites': '卫星数', 'remaining_distance': '剩余距离(m)'}
st.dataframe(df[display_cols].head(10).rename(columns=rename), use_container_width=True)
else:
st.info("暂无飞行数据")


def update_flight_simulation():
if st.session_state.simulation_running:
if time.time() - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
try:
    new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj, st.session_state.comm_sim)
    if new_hb:
        st.session_state.last_hb_time = time.time()
        st.session_state.flight_history.append([new_hb.lng, new_hb.lat])
        if len(st.session_state.flight_history) > 200:
            st.session_state.flight_history.pop(0)
        if not st.session_state.heartbeat_sim.simulating:
            st.session_state.simulation_running = False
            st.success("🏁 无人机已安全到达目的地！")
            st.rerun()
except Exception as e:
    st.error(f"更新心跳时出错: {e}")


# ==================== 障碍物管理页面 ====================
def render_obstacle_management_page(flight_alt: float):
st.header("🚧 障碍物管理")

col1, col2, col3, col4 = st.columns(4)
with col1:
st.markdown(f"""
<div style="background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        border-radius: 12px; padding: 15px; text-align: center; color: white;">
<div style="font-size: 28px; font-weight: bold;">{len(st.session_state.obstacles_gcj)}</div>
<div style="font-size: 12px; opacity: 0.9;">📊 障碍物总数</div>
</div>
""", unsafe_allow_html=True)
with col2:
high_obs = sum(1 for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt)
st.markdown(f"""
<div style="background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
        border-radius: 12px; padding: 15px; text-align: center; color: white;">
<div style="font-size: 28px; font-weight: bold;">{high_obs}</div>
<div style="font-size: 12px; opacity: 0.9;">🔴 需避让障碍物</div>
</div>
""", unsafe_allow_html=True)
with col3:
safe_obs = len(st.session_state.obstacles_gcj) - high_obs
st.markdown(f"""
<div style="background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
        border-radius: 12px; padding: 15px; text-align: center; color: white;">
<div style="font-size: 28px; font-weight: bold;">{safe_obs}</div>
<div style="font-size: 12px; opacity: 0.9;">🟠 安全障碍物</div>
</div>
""", unsafe_allow_html=True)
with col4:
total_vertices = sum(len(obs.get('polygon', [])) for obs in st.session_state.obstacles_gcj)
st.markdown(f"""
<div style="background: linear-gradient(135deg, #43e97b 0%, #38f9d7 100%);
        border-radius: 12px; padding: 15px; text-align: center; color: white;">
<div style="font-size: 28px; font-weight: bold;">{total_vertices}</div>
<div style="font-size: 12px; opacity: 0.9;">📍 总顶点数</div>
</div>
""", unsafe_allow_html=True)

st.markdown("---")
st.markdown("### 🛠️ 工具栏")
tool_cols = st.columns([1, 1, 1, 1, 2])
with tool_cols[0]:
if st.button("💾 保存配置", use_container_width=True, type="primary"):
if save_obstacles(st.session_state.obstacles_gcj):
    st.success(f"✅ 已保存 {len(st.session_state.obstacles_gcj)} 个障碍物")
    st.balloons()
with tool_cols[1]:
if st.session_state.obstacles_gcj:
config_data = {
    'obstacles': st.session_state.obstacles_gcj,
    'count': len(st.session_state.obstacles_gcj),
    'export_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    'version': 'v13.2'
}
st.download_button(label="📥 导出配置", data=json.dumps(config_data, ensure_ascii=False, indent=2),
                   file_name=f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                   mime="application/json", use_container_width=True)
else:
st.download_button(label="📥 导出配置", data=json.dumps({"obstacles": [], "count": 0}, ensure_ascii=False, indent=2),
                   file_name=f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                   mime="application/json", use_container_width=True, disabled=True)
st.caption("📭 暂无障碍物")
with tool_cols[2]:
latest_backup = get_latest_backup()
if latest_backup:
if st.button("🔄 恢复备份", use_container_width=True):
    if restore_from_backup(latest_backup):
        st.session_state.obstacles_gcj = load_obstacles()
        for obs in st.session_state.obstacles_gcj:
            obs['selected'] = False
        update_path_after_obstacle_change(flight_alt)
        st.success("✅ 已从备份恢复")
        st.rerun()
    else:
        st.error("❌ 恢复失败")
else:
st.button("🔄 恢复备份", use_container_width=True, disabled=True)
st.caption("📭 暂无备份")
with tool_cols[3]:
if st.button("🗑️ 清除全部", use_container_width=True):
if st.session_state.obstacles_gcj:
    if st.session_state.auto_backup:
        backup_config()
    st.session_state.obstacles_gcj = []
    save_obstacles([])
    update_path_after_obstacle_change(flight_alt)
    st.success("✅ 已清除所有障碍物")
    st.rerun()
else:
    st.warning("⚠️ 无障碍物")
with tool_cols[4]:
if os.path.exists(config.CONFIG_FILE):
try:
    with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
        save_time = json.load(f).get('save_time', '未知')
    st.info(f"💾 最后保存: {save_time}")
except:
    st.info("💾 未保存")
else:
st.info("💾 未保存")

st.markdown("---")
info_cols = st.columns(3)
with info_cols[0]:
avg_height = sum(obs.get('height', 30) for obs in st.session_state.obstacles_gcj) / max(1, len(st.session_state.obstacles_gcj))
st.metric("📏 平均高度", f"{avg_height:.1f} m")
with info_cols[1]:
backup_count = len([f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')])
st.metric("📦 备份数量", backup_count)
with info_cols[2]:
st.metric("🛡️ 安全半径", f"{st.session_state.safety_radius} 米")

st.markdown("---")
with st.expander("🎯 批量操作", expanded=False):
for obs in st.session_state.obstacles_gcj:
if 'selected' not in obs:
    obs['selected'] = False

batch_cols = st.columns([1, 1, 1, 2])
with batch_cols[0]:
select_all = st.checkbox("☑️ 全选", key="select_all_obs")
if select_all:
    for obs in st.session_state.obstacles_gcj:
        obs['selected'] = True
with batch_cols[1]:
if st.button("🗑️ 批量删除", use_container_width=True, type="primary"):
    selected = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
    if selected:
        if st.session_state.auto_backup:
            backup_config()
        for i in reversed(selected):
            st.session_state.obstacles_gcj.pop(i)
        save_obstacles(st.session_state.obstacles_gcj)
        update_path_after_obstacle_change(flight_alt)
        st.success(f"✅ 已删除 {len(selected)} 个障碍物")
        st.rerun()
    else:
        st.warning("⚠️ 请先选择障碍物")
with batch_cols[2]:
batch_height = st.number_input("批量高度(m)", 1, 200, 30, 5, key="batch_height", label_visibility="collapsed")
if st.button("📏 批量设置", use_container_width=True):
    selected = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
    if selected:
        for i in selected:
            st.session_state.obstacles_gcj[i]['height'] = batch_height
        if st.session_state.auto_backup:
            save_obstacles(st.session_state.obstacles_gcj)
        update_path_after_obstacle_change(flight_alt)
        st.success(f"✅ 已设置 {len(selected)} 个障碍物")
        st.rerun()
    else:
        st.warning("⚠️ 请先选择障碍物")
with batch_cols[3]:
if st.button("🏷️ 批量重命名", use_container_width=True):
    selected = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
    if selected:
        st.session_state.show_rename_dialog = True
    else:
        st.warning("⚠️ 请先选择障碍物")

if st.session_state.get('show_rename_dialog', False):
with st.container():
    st.markdown("---")
    st.markdown("#### 🏷️ 批量重命名")
    rename_cols = st.columns([1, 1, 1, 1])
    with rename_cols[0]:
        name_prefix = st.text_input("前缀", value="建筑物")
    with rename_cols[1]:
        start_number = st.number_input("起始编号", 1, 100, 1)
    with rename_cols[2]:
        name_suffix = st.text_input("后缀", value="")
    with rename_cols[3]:
        col_confirm, col_cancel = st.columns(2)
        with col_confirm:
            if st.button("确认", use_container_width=True, type="primary"):
                selected = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
                for idx, i in enumerate(selected):
                    st.session_state.obstacles_gcj[i]['name'] = f"{name_prefix}{start_number + idx}{name_suffix}"
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                st.session_state.show_rename_dialog = False
                st.success(f"✅ 已重命名 {len(selected)} 个障碍物")
                st.rerun()
        with col_cancel:
            if st.button("取消", use_container_width=True):
                st.session_state.show_rename_dialog = False
                st.rerun()

st.markdown("---")
tab1, tab2 = st.tabs(["📋 列表视图", "🗺️ 地图视图"])
with tab1:
render_obstacle_list_view(flight_alt)
with tab2:
render_obstacle_map_view(flight_alt)


def render_obstacle_list_view(flight_alt: float):
st.subheader("📝 障碍物列表")
st.caption("💡 提示：勾选复选框后可使用批量操作功能")

if not st.session_state.obstacles_gcj:
st.info("📭 暂无任何障碍物，可以在「地图视图」中绘制添加")
return

items_per_row = 3
rows = (len(st.session_state.obstacles_gcj) + items_per_row - 1) // items_per_row

for row in range(rows):
cols = st.columns(items_per_row)
for col_idx in range(items_per_row):
idx = row * items_per_row + col_idx
if idx < len(st.session_state.obstacles_gcj):
    render_obstacle_card(idx, flight_alt, cols[col_idx])


def render_obstacle_card(idx: int, flight_alt: float, container):
obs = st.session_state.obstacles_gcj[idx]
with container:
with st.container(border=True):
height = obs.get('height', 30)
color = "🔴" if height > flight_alt else "🟠"
name = obs.get('name', f'障碍物{idx+1}')

header_cols = st.columns([1, 5])
with header_cols[0]:
    checked = st.checkbox("", key=f"select_card_{idx}", value=obs.get('selected', False))
    st.session_state.obstacles_gcj[idx]['selected'] = checked
with header_cols[1]:
    st.markdown(f"**{color} {name}**")

info_cols = st.columns(2)
with info_cols[0]:
    st.caption(f"📏 高度: {height}m")
with info_cols[1]:
    st.caption(f"📍 顶点: {len(obs.get('polygon', []))}个")

new_h = st.number_input("高度", min_value=1, max_value=200, value=height, step=5,
                        key=f"quick_edit_{idx}", label_visibility="collapsed")
if new_h != height:
    obs['height'] = new_h
    if st.session_state.auto_backup:
        save_obstacles(st.session_state.obstacles_gcj)
    update_path_after_obstacle_change(flight_alt)
    st.rerun()

if st.button("🗑️ 删除", key=f"delete_card_{idx}", use_container_width=True):
    if st.session_state.auto_backup:
        backup_config()
    st.session_state.obstacles_gcj.pop(idx)
    save_obstacles(st.session_state.obstacles_gcj)
    update_path_after_obstacle_change(flight_alt)
    st.success(f"✅ 已删除 {name}")
    st.rerun()


def render_obstacle_map_view(flight_alt: float):
st.subheader("🗺️ 地图视图")
st.caption("✏️ 使用左上角绘制工具绘制新障碍物 | 🖱️ 点击障碍物查看详细信息 | 🎨 红色=需避让，橙色=安全")

tiles = config.GAODE_SATELLITE_URL
m = folium.Map(location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles, attr="高德卫星地图")

draw = plugins.Draw(
export=True, position='topleft',
draw_options={
'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000',
            'fillColor': '#ff0000', 'fillOpacity': 0.4},
'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False
},
edit_options={'edit': True, 'remove': True}
)
m.add_child(draw)

for obs in st.session_state.obstacles_gcj:
coords = obs.get('polygon', [])
height = obs.get('height', 30)
color = "red" if height > flight_alt else "orange"
if coords and len(coords) >= 3:
popup_text = f"""
<div style="font-family: sans-serif; min-width: 150px;">
    <b>🏢 {obs.get('name', '未知')}</b><br>
    📏 高度: {height} 米<br>
    📍 顶点: {len(coords)} 个<br>
    🆔 ID: {obs.get('id', 'N/A')[:12]}
</div>
"""
folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True,
              fill_color=color, fill_opacity=0.5, popup=folium.Popup(popup_text, max_width=250)).add_to(m)

folium.Marker([config.DEFAULT_A_GCJ[1], config.DEFAULT_A_GCJ[0]], popup="🟢 起点 (默认)",
     icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(m)
folium.Marker([config.DEFAULT_B_GCJ[1], config.DEFAULT_B_GCJ[0]], popup="🔴 终点 (默认)",
     icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(m)

output = st_folium(m, width=850, height=550, key="obstacle_map_view", returned_objects=["last_active_drawing"])

if output and output.get("last_active_drawing"):
last = output["last_active_drawing"]
if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
coords = last["geometry"].get("coordinates", [[]])[0]
poly = [[p[0], p[1]] for p in coords]
if len(poly) >= 3 and st.session_state.pending_obstacle is None and validate_polygon(poly):
    st.session_state.pending_obstacle = poly
    st.rerun()

if st.session_state.pending_obstacle is not None:
render_obstacle_dialog()


def update_path_after_obstacle_change(flight_alt: float):
if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
st.session_state.planned_path = create_avoidance_path(
st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
st.session_state.obstacles_gcj, flight_alt,
st.session_state.current_direction, st.session_state.safety_radius)


# ==================== 主程序 ====================
def main():
st.set_page_config(page_title="无人机地面站系统", layout="wide")
init_session_state()
st.title("🏫 无人机地面站系统")
st.markdown("---")

page, drone_speed, flight_alt, auto_save = render_sidebar()
st.session_state.auto_backup = auto_save

if flight_alt != st.session_state.last_flight_altitude:
st.session_state.last_flight_altitude = flight_alt
if st.session_state.planned_path:
st.session_state.planned_path = create_avoidance_path(
    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
    st.session_state.obstacles_gcj, flight_alt,
    st.session_state.current_direction, st.session_state.safety_radius)

if page == "🗺️ 航线规划":
render_planning_page(drone_speed, flight_alt, auto_save)
elif page == "📡 飞行监控":
render_flight_monitoring_page(flight_alt, drone_speed)
elif page == "🔗 通信拓扑":
render_communication_page()
elif page == "🚧 障碍物管理":
render_obstacle_management_page(flight_alt)


if __name__ == "__main__":
main()
