import streamlit as st
import pandas as pd
import plotly.express as px

# --- 1. 页面配置：必须放在最前面，设置页面标题和布局 ---
st.set_page_config(page_title="心跳监控看板", layout="wide")

# --- 2. 标题和说明 ---
st.title("💓 心跳包序号趋势监控")
st.markdown("该应用展示心跳包序号（Sequence Number）随时间的变化趋势。")

# --- 3. 数据加载逻辑 (支持本地文件 & GitHub) ---
st.sidebar.header("📂 数据源配置")

# 创建一个缓存函数，避免每次交互都重新加载数据，提升性能
@st.cache_data
def load_data(uploaded_file, github_url):
    """
    根据用户选择加载数据：
    优先使用上传的文件，如果没有上传但填了GitHub URL，则从网络加载。
    """
    # 情况1：用户上传了本地CSV文件
    if uploaded_file is not None:
        try:
            df = pd.read_csv(uploaded_file)
            st.sidebar.success("✅ 已加载本地文件")
            return df
        except Exception as e:
            st.sidebar.error(f"文件读取失败: {e}")
            return None
    
    # 情况2：用户提供了GitHub原始链接
    elif github_url:
        try:
            # 注意：GitHub 的 raw 链接通常以 /raw/ 开头，确保你使用的是正确的原始数据链接
            df = pd.read_csv(github_url)
            st.sidebar.success("✅ 已从 GitHub 加载数据")
            return df
        except Exception as e:
            st.sidebar.error(f"网络数据加载失败: {e}\n请检查链接是否为原始数据链接（Raw）")
            return None
    
    # 情况3：演示模式：如果没有输入任何东西，自动生成模拟数据，方便预览效果
    else:
        st.sidebar.info("💡 演示模式：未提供数据，正在生成随机示例数据...")
        # 生成模拟数据：从当前时间开始，连续100个心跳点
        date_range = pd.date_range(start="2024-01-01", periods=100, freq="S")
        # 生成序号：初始1000，逐渐增加，并加入少量波动模拟网络延迟
        seq_nums = 1000 + range(100) + (pd.Series(range(100)) * 0.5).round(0)
        df = pd.DataFrame({
            "timestamp": date_range,
            "heartbeat_seq": seq_nums
        })
        return df

# --- 4. 侧边栏控件：让用户选择数据来源 ---
with st.sidebar:
    st.subheader("1. 上传本地数据")
    uploaded_file = st.file_uploader("选择 CSV 文件", type="csv")
    
    st.subheader("2. 或使用 GitHub 链接")
    github_url = st.text_input("粘贴 GitHub 原始数据链接 (Raw URL)", 
                               placeholder="https://raw.githubusercontent.com/.../data.csv")
    
    st.markdown("---")
    st.caption("CSV文件需包含 'timestamp' (时间) 和 'heartbeat_seq' (序号) 列")

# --- 5. 加载数据 ---
df = load_data(uploaded_file, github_url)

# --- 6. 数据预处理与校验 ---
if df is not None:
    # 假设数据列名为 'timestamp' 和 'heartbeat_seq'
    # 如果你的列名不同，可以在这里修改，或者利用st.dataframe查看结构
    # 如果数据中没有时间列，我们假设索引为时间（演示用）
    if 'timestamp' not in df.columns:
        st.warning("⚠️ 数据中未找到 'timestamp' 列，将使用行索引作为X轴")
        df['timestamp'] = df.index
    
    # 确保时间列是datetime格式，方便Plotly处理
    try:
        df['timestamp'] = pd.to_datetime(df['timestamp'])
    except:
        pass # 如果转换失败，保持原样，Plotly也能处理字符串
# --- 7. 主界面布局与图表绘制 ---
if df is not None:
    # 显示数据预览
    st.subheader("📊 数据预览")
    st.dataframe(df.head(10), use_container_width=True)
    
    # 关键指标卡片 (KPI)
    col1, col2, col3 = st.columns(3)
    with col1:
        st.metric("最新序号", f"{df['heartbeat_seq'].iloc[-1]:.0f}")
    with col2:
        st.metric("总心跳次数", len(df))
    with col3:
        # 计算总变化量
        delta = df['heartbeat_seq'].iloc[-1] - df['heartbeat_seq'].iloc[0]
        st.metric("序号总增量", f"{delta:.0f}")
    
    # --- 核心：绘制折线图 ---
    st.subheader("📈 心跳序号变化趋势")
    
    # 使用 Plotly 创建交互式图表
    fig = px.line(
        df, 
        x='timestamp', 
        y='heartbeat_seq',
        title='心跳包序号随时间变化曲线',
        labels={'timestamp': '时间', 'heartbeat_seq': '心跳序号 (Seq)'},
        markers=True  # 显示数据点
    )
    
    # 美化图表布局
    fig.update_layout(
        hovermode='x unified',  # 统一悬停模式
        xaxis_title="时间戳",
        yaxis_title="心跳序号",
        template="plotly_white"  # 白底风格，清爽
    )
    
    # 在 Streamlit 中渲染 Plotly 图表
    st.plotly_chart(fig, use_container_width=True)
    
    # 可选：添加一个滑块来筛选时间范围
    st.subheader("🎚️ 交互式筛选 (可选)")
    min_time = df['timestamp'].min()
    max_time = df['timestamp'].max()
    
    # 只有时间范围有效且不是模拟索引时才显示时间滑块
    if min_time != max_time:
        time_range = st.slider(
            "选择时间范围",
            min_value=min_time,
            max_value=max_time,
            value=(min_time, max_time)
        )
        # 筛选数据
        mask = (df['timestamp'] >= time_range[0]) & (df['timestamp'] <= time_range[1])
        filtered_df = df[mask]
        
        if not filtered_df.empty:
            st.write(f"筛选后共有 {len(filtered_df)} 条记录")
            # 绘制筛选后的局部放大图
            fig_filtered = px.line(filtered_df, x='timestamp', y='heartbeat_seq', markers=True)
            st.plotly_chart(fig_filtered, use_container_width=True)
        else:
            st.warning("筛选范围内无数据")
else:
    st.error("❌ 无法加载数据，请检查文件格式或网络链接。")
