import streamlit as st

st.set_page_config(
    page_title="无人机飞行规划与监控系统",
    page_icon="✈️",
    layout="wide"
)

# ================== 航线规划 Tab 中的 3D 地图 HTML ==================
# 该 HTML 包含了完整的 Cesium 地图、坐标系转换、A/B点设置、障碍物生成及圈选功能
map_html = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>3D 航线规划</title>
    <link rel="stylesheet" href="https://cesium.com/downloads/cesiumjs/releases/1.113/Build/Cesium/Widgets/widgets.css">
    <script src="https://cesium.com/downloads/cesiumjs/releases/1.113/Build/Cesium/Cesium.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/coordtransform@2.1.2/index.js"></script>
    <style>
        html, body, #cesiumContainer {
            width: 100%;
            height: 100%;
            margin: 0;
            padding: 0;
            overflow: hidden;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }
        /* 控制面板浮层 */
        .control-panel {
            position: absolute;
            top: 20px;
            right: 20px;
            width: 320px;
            background: rgba(20, 30, 45, 0.85);
            backdrop-filter: blur(10px);
            border-radius: 16px;
            padding: 16px;
            color: white;
            border: 1px solid rgba(59,130,246,0.5);
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
            z-index: 100;
            pointer-events: auto;
            font-size: 14px;
        }
        .control-panel h3 {
            margin: 0 0 12px 0;
            font-size: 18px;
            text-align: center;
            color: #90caf9;
        }
        .section {
            margin-bottom: 16px;
            border-bottom: 1px solid #2c3e66;
            padding-bottom: 12px;
        }
        .section-title {
            font-weight: bold;
            margin-bottom: 8px;
            color: #bbd4ff;
        }
        .input-group {
            display: flex;
            gap: 8px;
            margin-bottom: 8px;
        }
        .input-group input, .input-group select {
            flex: 1;
            background: #0f121c;
            border: 1px solid #2d3a5e;
            padding: 6px 8px;
            border-radius: 8px;
            color: white;
        }
        button {
            background: #2c3e66;
            border: none;
            padding: 6px 12px;
            border-radius: 20px;
            color: white;
            cursor: pointer;
            font-size: 12px;
            width: 100%;
            margin-top: 6px;
            transition: all 0.2s;
        }
        button.primary {
            background: #3b82f6;
        }
        button.primary:hover {
            background: #2563eb;
        }
        .coord-radio {
            display: flex;
            gap: 16px;
            margin: 8px 0;
        }
        .coord-radio label {
            display: flex;
            align-items: center;
            gap: 5px;
        }
        .status-text {
            font-size: 12px;
            color: #a0c4ff;
            margin-top: 8px;
            text-align: center;
        }
        button.obstacle-btn {
            background: #a855f7;
        }
        .note {
            font-size: 11px;
            color: #aaa;
            text-align: center;
            margin-top: 10px;
        }
    </style>
</head>
<body>
    <div id="cesiumContainer"></div>
    <div class="control-panel">
        <h3>🗺️ 航线规划 · 南京科技职业学院</h3>
        <div class="section">
            <div class="section-title">🌐 坐标系设置</div>
            <div class="coord-radio" id="coordSysGroup">
                <label><input type="radio" name="inputCoord" value="GCJ02" checked> GCJ-02 (高德/百度)</label>
                <label><input type="radio" name="inputCoord" value="WGS84"> WGS-84</label>
            </div>
        </div>
        <div class="section">
            <div class="section-title">📍 起点 A</div>
            <div class="input-group">
                <input type="number" id="aLat" value="32.2322" step="any" placeholder="纬度">
                <input type="number" id="aLng" value="118.7490" step="any" placeholder="经度">
            </div>
            <button id="setABtn" class="primary">✈️ 设置 A 点</button>
        </div>
        <div class="section">
            <div class="section-title">📍 终点 B</div>
            <div class="input-group">
                <input type="number" id="bLat" value="32.2343" step="any" placeholder="纬度">
                <input type="number" id="bLng" value="118.7490" step="any" placeholder="经度">
            </div>
            <button id="setBBtn" class="primary">✈️ 设置 B 点</button>
        </div>
        <div class="section">
            <div class="section-title">🚁 飞行参数</div>
            <div class="input-group">
                <input type="number" id="flightHeight" value="50" step="5" placeholder="高度(米)">
            </div>
        </div>
        <div class="section">
            <div class="section-title">🧱 障碍物</div>
            <button id="highlightObstacleBtn" class="obstacle-btn">🔍 圈选 / 高亮障碍物</button>
            <div class="status-text" id="obstacleStatus">✅ 自动生成于 A-B 连线中点</div>
        </div>
        <div class="note">💡 坐标自动转换 → 地图显示为 WGS84 | 障碍物为红色立方体</div>
    </div>

    <script>
        // 初始化 Cesium (使用官方推荐地形和影像)
        Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiIyY2Q3ZjMxZi1mOTg4LTRkNjEtYWEzMy00OTJiMzI5YjVkOTUiLCJpZCI6NDExNTg1LCJpYXQiOjE3NzQ5MjY2MTR9.ONB5PcSqUb071ws3IFko5Sg-_0hvmOwo2r7-sF6G7eE';
        // 如果上述 Token 失效，请前往 https://cesium.com/ion/signup 免费申请并替换
        
        const viewer = new Cesium.Viewer('cesiumContainer', {
            terrain: Cesium.Terrain.fromWorldTerrain(),
            baseLayerPicker: false,
            geocoder: false,
            navigationHelpButton: false,
            sceneModePicker: true,
            shadows: true,
            infoBox: false
        });
        viewer.scene.globe.enableLighting = true;
        // 添加 3D 建筑
        viewer.scene.primitives.add(Cesium.createOsmBuildings());
        
        // 相机飞到校园区域 (南京科技职业学院 约 118.749, 32.233)
        viewer.camera.flyTo({
            destination: Cesium.Cartesian3.fromDegrees(118.749, 32.2333, 400),
            orientation: { heading: Cesium.Math.toRadians(0), pitch: Cesium.Math.toRadians(-35) }
        });
        
        // 存储实体
        let entities = {
            markerA: null,
            markerB: null,
            line: null,
            obstacle: null
        };
        let currentAWgs84 = null;
        let currentBWgs84 = null;
        
        // 坐标转换辅助
        function convertGCJ02ToWGS84(lng, lat) {
            if (typeof coordtransform !== 'undefined') {
                const wgs = coordtransform.gcj02towgs84(lng, lat);
                return { lng: wgs[0], lat: wgs[1] };
            }
            return { lng: lng - 0.0012, lat: lat - 0.0005 };
        }
        
        function getInputCoordAsWGS84(lngVal, latVal, coordType) {
            let lng = parseFloat(lngVal);
            let lat = parseFloat(latVal);
            if (isNaN(lng) || isNaN(lat)) return null;
            if (coordType === 'GCJ02') {
                return convertGCJ02ToWGS84(lng, lat);
            } else {
                return { lng, lat };
            }
        }
        
        function clearMapEntities() {
            if (entities.markerA) viewer.entities.remove(entities.markerA);
            if (entities.markerB) viewer.entities.remove(entities.markerB);
            if (entities.line) viewer.entities.remove(entities.line);
            if (entities.obstacle) viewer.entities.remove(entities.obstacle);
            entities = { markerA: null, markerB: null, line: null, obstacle: null };
        }
        
        function createObstacleBetweenPoints(p1, p2) {
            if (!p1 || !p2) return null;
            const midLng = (p1.lng + p2.lng) / 2;
            const midLat = (p1.lat + p2.lat) / 2;
            const heightAboveGround = 25.0;
            const position = Cesium.Cartesian3.fromDegrees(midLng, midLat, heightAboveGround);
            const box = viewer.entities.add({
                name: '障碍物',
                position: position,
                box: {
                    dimensions: new Cesium.Cartesian3(12.0, 12.0, heightAboveGround),
                    material: Cesium.Color.RED.withAlpha(0.85),
                    outline: true,
                    outlineColor: Cesium.Color.YELLOW,
                    outlineWidth: 2
                },
                description: '校园障碍物 (位于A-B连线中点)'
            });
            // 添加标签
            viewer.entities.add({
                position: Cesium.Cartesian3.fromDegrees(midLng, midLat, heightAboveGround + 5),
                label: {
                    text: '🚧 障碍物',
                    font: '14px sans-serif',
                    fillColor: Cesium.Color.WHITE,
                    outlineColor: Cesium.Color.BLACK,
                    outlineWidth: 2,
                    pixelOffset: new Cesium.Cartesian2(0, -15)
                }
            });
            return box;
        }
        
        function updateMapPoints() {
            const coordType = document.querySelector('input[name="inputCoord"]:checked').value;
            const aLat = document.getElementById('aLat').value;
            const aLng = document.getElementById('aLng').value;
            const bLat = document.getElementById('bLat').value;
            const bLng = document.getElementById('bLng').value;
            
            const aWgs = getInputCoordAsWGS84(aLng, aLat, coordType);
            const bWgs = getInputCoordAsWGS84(bLng, bLat, coordType);
            if (!aWgs || !bWgs) {
                alert("坐标无效，请检查输入");
                return;
            }
            currentAWgs84 = aWgs;
            currentBWgs84 = bWgs;
            
            clearMapEntities();
            
            // 添加 A、B 点标记
            entities.markerA = viewer.entities.add({
                position: Cesium.Cartesian3.fromDegrees(aWgs.lng, aWgs.lat, 8),
                point: { pixelSize: 12, color: Cesium.Color.LIME, outlineColor: Cesium.Color.WHITE, outlineWidth: 2 },
                label: { text: 'A 起点', font: '14px sans-serif', fillColor: Cesium.Color.LIME, pixelOffset: new Cesium.Cartesian2(0, -20) }
            });
            entities.markerB = viewer.entities.add({
                position: Cesium.Cartesian3.fromDegrees(bWgs.lng, bWgs.lat, 8),
                point: { pixelSize: 12, color: Cesium.Color.ORANGERED, outlineColor: Cesium.Color.WHITE, outlineWidth: 2 },
                label: { text: 'B 终点', font: '14px sans-serif', fillColor: Cesium.Color.ORANGERED, pixelOffset: new Cesium.Cartesian2(0, -20) }
            });
            // 连线
            entities.line = viewer.entities.add({
                polyline: {
                    positions: Cesium.Cartesian3.fromDegreesArray([aWgs.lng, aWgs.lat, bWgs.lng, bWgs.lat]),
                    width: 4,
                    material: Cesium.Color.RED,
                    arcType: Cesium.ArcType.GEODESIC
                }
            });
            // 障碍物
            entities.obstacle = createObstacleBetweenPoints(aWgs, bWgs);
            
            // 更新状态显示
            document.getElementById('obstacleStatus').innerHTML = '✅ A/B 点已更新，障碍物已生成';
        }
        
        function highlightObstacle() {
            if (entities.obstacle) {
                const pos = entities.obstacle.position.getValue(Cesium.JulianDate.now());
                if (pos) {
                    viewer.camera.flyTo({
                        destination: pos,
                        duration: 0.8,
                        complete: () => {
                            if (entities.obstacle.box) {
                                const original = entities.obstacle.box.material;
                                entities.obstacle.box.material = Cesium.Color.GOLD.withAlpha(0.9);
                                setTimeout(() => { if(entities.obstacle) entities.obstacle.box.material = Cesium.Color.RED.withAlpha(0.85); }, 800);
                            }
                        }
                    });
                } else {
                    alert("障碍物位置未就绪");
                }
            } else {
                alert("请先设置 A、B 点生成障碍物");
            }
        }
        
        // 绑定按钮事件
        document.getElementById('setABtn').addEventListener('click', updateMapPoints);
        document.getElementById('setBBtn').addEventListener('click', updateMapPoints);
        document.getElementById('highlightObstacleBtn').addEventListener('click', highlightObstacle);
        // 坐标系切换时自动刷新地图
        const radios = document.querySelectorAll('input[name="inputCoord"]');
        radios.forEach(radio => radio.addEventListener('change', updateMapPoints));
        
        // 初次加载默认生成 A、B 点
        setTimeout(() => { updateMapPoints(); }, 1000);
    </script>
</body>
</html>
"""

# ================== 飞行监控 Tab 中的心跳包模拟 HTML ==================
# 前端模拟心跳包实时数据，独立于 Streamlit 后端，避免页面重载
heartbeat_html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>心跳包模拟</title>
    <style>
        body {
            background: #0b0f17;
            font-family: 'Segoe UI', monospace;
            padding: 20px;
            margin: 0;
            color: #eef2ff;
        }
        .container {
            max-width: 1000px;
            margin: 0 auto;
        }
        .stats {
            display: flex;
            gap: 20px;
            margin-bottom: 25px;
            flex-wrap: wrap;
        }
        .stat-card {
            background: #1e293b;
            border-radius: 20px;
            padding: 15px 25px;
            flex: 1;
            text-align: center;
            border-left: 4px solid #3b82f6;
        }
        .stat-value {
            font-size: 28px;
            font-weight: bold;
            color: #facc15;
        }
        .log-area {
            background: #0a0e16;
            border-radius: 20px;
            padding: 15px;
            height: 55vh;
            overflow-y: auto;
            font-size: 13px;
        }
        .log-entry {
            border-left: 3px solid #3b82f6;
            padding: 8px 12px;
            margin: 8px 0;
            background: #111827;
            border-radius: 12px;
            font-family: monospace;
        }
        .log-time {
            color: #6c86a3;
        }
        button {
            background: #2c3e66;
            border: none;
            padding: 8px 20px;
            border-radius: 40px;
            color: white;
            cursor: pointer;
            margin-top: 15px;
        }
        h2 {
            margin-top: 0;
        }
    </style>
</head>
<body>
<div class="container">
    <h2>📡 实时心跳数据流 (模拟无人机遥测)</h2>
    <div class="stats">
        <div class="stat-card">
            <div>📶 信号强度</div>
            <div class="stat-value" id="signalValue">-52 dBm</div>
        </div>
        <div class="stat-card">
            <div>🔋 电池电量</div>
            <div class="stat-value" id="batteryValue">98%</div>
        </div>
        <div class="stat-card">
            <div>📍 无人机位置</div>
            <div class="stat-value" id="posValue" style="font-size: 16px;">等待定位</div>
        </div>
    </div>
    <div style="text-align: right;">
        <button id="clearLogBtn">清空日志</button>
    </div>
    <div class="log-area" id="logArea">
        <div class="log-entry">✨ 心跳监控已启动，等待数据包...</div>
    </div>
</div>
<script>
    let intervalId = null;
    let seq = 1;
    // 模拟位置在 A(32.2322,118.749) 和 B(32.2343,118.749) 之间浮动
    const aLat = 32.2322, aLng = 118.749;
    const bLat = 32.2343, bLng = 118.749;
    
    function randomBetween(min, max) {
        return min + Math.random() * (max - min);
    }
    
    function getMockPosition() {
        // 模拟无人机沿 AB 连线移动，并加入随机偏移
        let t = (Date.now() / 1000) % 1;  // 0-1 循环
        let lat = aLat + (bLat - aLat) * t + (Math.random() - 0.5) * 0.0005;
        let lng = aLng + (bLng - aLng) * t + (Math.random() - 0.5) * 0.0005;
        return { lat, lng };
    }
    
    function addHeartbeatLog() {
        const signal = Math.floor(randomBetween(40, 75));
        const battery = Math.floor(randomBetween(65, 100));
        const pos = getMockPosition();
        const time = new Date().toLocaleTimeString();
        const logDiv = document.getElementById('logArea');
        const entry = document.createElement('div');
        entry.className = 'log-entry';
        entry.innerHTML = `<span class="log-time">[${time}]</span> ❤️ 心跳包 #${seq++} | 信号: -${signal} dBm | 电量: ${battery}% | 无人机位置: ${pos.lat.toFixed(5)}°, ${pos.lng.toFixed(5)}° | 高度: 50m`;
        logDiv.insertBefore(entry, logDiv.firstChild);
        // 限制条目数量
        while(logDiv.children.length > 40) logDiv.removeChild(logDiv.lastChild);
        
        // 更新统计卡片
        document.getElementById('signalValue').innerText = `-${signal} dBm`;
        document.getElementById('batteryValue').innerText = `${battery}%`;
        document.getElementById('posValue').innerHTML = `${pos.lat.toFixed(5)}°,<br>${pos.lng.toFixed(5)}°`;
    }
    
    function startHeartbeat() {
        if(intervalId) clearInterval(intervalId);
        addHeartbeatLog(); // 立即一次
        intervalId = setInterval(addHeartbeatLog, 2200);
    }
    
    document.getElementById('clearLogBtn').addEventListener('click', () => {
        document.getElementById('logArea').innerHTML = '<div class="log-entry">✨ 日志已清空，继续接收心跳...</div>';
    });
    
    startHeartbeat();
</script>
</body>
</html>
"""

# ================== Streamlit 主界面 ==================
st.title("✈️ 无人机飞行规划与监控系统")
st.markdown("**南京科技职业学院** · 校园三维规划 & 遥测监控")

# 创建两个 Tab
tab1, tab2 = st.tabs(["🗺️ 航线规划（3D地图）", "📡 飞行监控（心跳包）"])

with tab1:
    st.components.v1.html(map_html, height=700, scrolling=False)

with tab2:
    st.components.v1.html(heartbeat_html, height=650, scrolling=False)
    st.caption("💡 心跳包数据由前端模拟，实时展示信号、电量及无人机位置（在A-B航线附近移动）")

# 可选：在侧边栏添加说明
with st.sidebar:
    st.markdown("### 🧭 系统说明")
    st.markdown("""
    - **航线规划**：使用3D地图，支持GCJ-02/WGS-84坐标转换，可设置A/B点，自动生成连线及障碍物（红色立方体）。
    - **障碍物圈选**：点击按钮后相机自动聚焦障碍物并高亮。
    - **飞行监控**：实时模拟无人机心跳包，显示信号强度、电量、位置及日志。
    - 默认A/B点位于南京科技职业学院校园内（约北纬32.2322~32.2343，东经118.749）。
    - 若Cesium地图无法加载（白屏），请替换`map_html`中的`Cesium.Ion.defaultAccessToken`为您自己的[免费Token](https://cesium.com/ion/signup)。
    """)
    st.info("📌 提示：地图中的A/B点坐标可通过右侧面板修改，坐标系转换自动生效。")
