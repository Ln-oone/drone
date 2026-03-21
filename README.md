import time
import threading
import streamlit as st
import pandas as pd
from datetime import datetime, timedelta

# 全局状态
heartbeat_data = []
last_receive_time = datetime.now()
is_connected = True

# 模拟无人机发送心跳包
def send_heartbeat():
    global last_receive_time, is_connected
    seq = 1
    while True:
        now = datetime.now()
        heartbeat_data.append({"序号": seq, "时间": now})
        last_receive_time = now
        is_connected = True
        print(f"心跳包 {seq} 发送: {now}")
        seq += 1
        time.sleep(1)

# 检测掉线（3秒超时）
def check_connection():
    global is_connected
    while True:
        now = datetime.now()
        if (now - last_receive_time).total_seconds() > 3:
            is_connected = False
            print("⚠️ 连接超时！未收到心跳包超过3秒")
        time.sleep(0.5)

# Streamlit 可视化界面
def run_streamlit():
    st.title("无人机心跳包监控")
    st.subheader("实时连接状态")
    
    # 状态显示
    if is_connected:
        st.success("✅ 连接正常")
    else:
        st.error("❌ 连接超时")
    
    # 数据表格
    st.subheader("心跳包数据")
    df = pd.DataFrame(heartbeat_data)
    st.dataframe(df)
    
    # 折线图
    st.subheader("序号随时间变化")
    if not df.empty:
        st.line_chart(df, x="时间", y="序号")

# 启动线程
if __name__ == "__main__":
    # 启动心跳发送线程
    threading.Thread(target=send_heartbeat, daemon=True).start()
    # 启动连接检测线程
    threading.Thread(target=check_connection, daemon=True).start()
    # 启动Streamlit可视化
    run_streamlit()
