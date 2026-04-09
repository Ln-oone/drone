import streamlit as st

st.set_page_config(page_title="无人机飞行规划与监控系统", page_icon="✈️", layout="wide")

# 高德地图 HTML 组件（3D视图，支持坐标转换，AB点，障碍物）
amap_html = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>高德3D地图</title>
    <style>
        body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; }
        #container { width: 100%; height: 100%; }
        .control-panel {
            position: absolute;
            top: 20px;
            right: 20px;
            width: 280px;
            background: rgba(0,0,0,0.75);
            backdrop-filter: blur(8px);
            border-radius: 12px;
            padding: 15px;
            color: white;
            z-index: 1000;
            font-family: sans-serif;
            font-size: 13px;
            border: 1px solid #3b82f6;
            pointer-events: auto;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
        }
        .control-panel h3 { margin: 0 0 10px 0; text-align: center; color: #90caf9; }
        .section { margin-bottom: 12px; border-bottom: 1px solid #555; padding-bottom: 8px; }
        .input-group { display: flex; gap: 8px; margin-bottom: 8px; }
        .input-group input { flex: 1; background: #1e1e2a; border: 1px solid #3b82f6; padding: 6px; border-radius: 6px; color: white; }
        button { background: #3b82f6; border: none; padding: 6px; border-radius: 20px; color: white; cursor: pointer; width: 100%; margin-top: 5px; }
        .coord-radio { display: flex; gap: 15px; margin: 8px 0; }
        .status { font-size: 12px; text-align: center; margin-top: 8px; color: #aaf; }
        .note { font-size: 11px; text-align: center; margin-top: 10px; color: #ccc; }
    </style>
    <!-- 引入高德地图 JS API -->
    <script src="https://webapi.amap.com/maps?v=2.0&key=YOUR_AMAP_KEY"></script>
    <!-- 引入高德地图 UI 组件库（可选） -->
    <script src="https://webapi.amap.com/ui/1.1/main.js"></script>
</head>
<body>
<div id="container"></div>
<div class="control-panel">
    <h3>🗺️ 高德3D地图 · 航线规划</h3>
    <div class="section">
        <div class="coord-radio" id="coordSysGroup">
            <label><input type="radio" name="coord" value="GCJ02" checked> GCJ-02 (高德/百度)</label>
            <label><input type="radio" name="coord" value="WGS84"> WGS-84</label>
        </div>
        <div class="note">注：高德地图使用 GCJ-02 坐标系，输入WGS84坐标会自动转换</div>
    </div>
    <div class="section">
        <div>📍 起点 A (校园内)</div>
        <div class="input-group">
            <input type="number" id="aLat" value="32.2322" step="0.0001" placeholder="纬度">
            <input type="number" id="aLng" value="118.7490" step="0.0001" placeholder="经度">
        </div>
        <button id="setABtn">设置 A 点</button>
    </div>
    <div class="section">
        <div>📍 终点 B (校园内)</div>
        <div class="input-group">
            <input type="number" id="bLat" value="32.2343" step="0.0001" placeholder="纬度">
            <input type="number" id="bLng" value="118.7490" step="0.0001" placeholder="经度">
        </div>
        <button id="setBBtn">设置 B 点</button>
    </div>
    <div class="section">
        <div>🧱 障碍物</div>
        <button id="highlightBtn" style="background:#a855f7;">🔍 圈选 / 高亮障碍物</button>
        <div class="status" id="statusMsg">⚪ 未设置 A/B 点</div>
    </div>
    <div class="note">💡 支持鼠标拖拽/右键旋转/滚轮缩放 | 红色立方体为障碍物</div>
</div>

<script>
    // 初始化地图（3D视图）
    let map = new AMap.Map('container', {
        center: [118.749, 32.2332],   // 南京科技职业学院
        zoom: 17,
        pitch: 65,                     // 倾斜角度（3D效果）
        viewMode: '3D',               // 3D视图
        building: true,               // 显示3D建筑物
        showIndoorMap: false
    });
    
    // 添加控件
    map.addControl(new AMap.Scale());
    map.addControl(new AMap.ToolBar({ position: 'RT' }));
    map.addControl(new AMap.ControlBar({ position: 'RB' }));  // 3D旋转控件
    
    // 坐标转换辅助（WGS84 -> GCJ02，因为高德地图使用GCJ02）
    function wgs84ToGcj02(lng, lat) {
        // 使用高德自带的坐标转换工具（如果可用）
        if (typeof AMap.GeometryUtil !== 'undefined' && AMap.GeometryUtil.gcj02ToWGS84) {
            // 注意：高德工具一般是 GCJ02 -> WGS84，我们需要逆转换。这里使用简单偏移法（中国境内近似）
            // 为了准确，我们直接调用高德转换服务？但需要 key。为了演示，使用开源转换函数（与之前一致）
        }
        // 开源转换函数（从 GCJ-02 转 WGS84 的逆过程，简化为加偏移）
        // 由于高德地图本身就是 GCJ-02，如果用户输入的是 WGS84，我们需要将 WGS84 转成 GCJ02 才能正确显示位置。
        // 这里使用一个简化的反向转换（实际应该用精确算法，但为了演示，直接用之前 GCJ02->WGS84 的逆）
        // 更可靠：使用 coordtransform 库？但我们不额外引入。下面提供精确转换（基于国测局算法逆推）。
        // 为了代码简洁，我们直接使用一个已有的在线转换函数（从 WGS84 转 GCJ02）。
        // 这里引入一个精简但准确的转换库（内置）
        function outOfChina(lng, lat) {
            return (lng < 72.004 || lng > 137.8347) || (lat < 0.8293 || lat > 55.8271);
        }
        function transformLat(x, y) {
            let ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x));
            ret += (20.0 * Math.sin(6.0 * x * Math.PI) + 20.0 * Math.sin(2.0 * x * Math.PI)) * 2.0 / 3.0;
            ret += (20.0 * Math.sin(y * Math.PI) + 40.0 * Math.sin(y / 3.0 * Math.PI)) * 2.0 / 3.0;
            ret += (160.0 * Math.sin(y / 12.0 * Math.PI) + 320 * Math.sin(y * Math.PI / 30.0)) * 2.0 / 3.0;
            return ret;
        }
        function transformLng(x, y) {
            let ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x));
            ret += (20.0 * Math.sin(6.0 * x * Math.PI) + 20.0 * Math.sin(2.0 * x * Math.PI)) * 2.0 / 3.0;
            ret += (20.0 * Math.sin(x * Math.PI) + 40.0 * Math.sin(x / 3.0 * Math.PI)) * 2.0 / 3.0;
            ret += (150.0 * Math.sin(x / 12.0 * Math.PI) + 300.0 * Math.sin(x / 30.0 * Math.PI)) * 2.0 / 3.0;
            return ret;
        }
        if (outOfChina(lng, lat)) return { lng, lat };
        let dLat = transformLat(lng - 105.0, lat - 35.0);
        let dLng = transformLng(lng - 105.0, lat - 35.0);
        let radLat = lat / 180.0 * Math.PI;
        let magic = Math.sin(radLat);
        magic = 1 - 0.006693421622965943 * magic * magic;
        let sqrtMagic = Math.sqrt(magic);
        dLat = (dLat * 180.0) / ((6378245.0 * (1 - 0.006693421622965943)) / (magic * sqrtMagic) * Math.PI);
        dLng = (dLng * 180.0) / (6378245.0 / sqrtMagic * Math.cos(radLat) * Math.PI);
        let mgLat = lat + dLat;
        let mgLng = lng + dLng;
        return { lng: mgLng, lat: mgLat };
    }
    
    // 获取用户输入的坐标（根据坐标系类型返回 GCJ02 坐标，因为高德地图使用 GCJ02）
    function getMapCoords() {
        let coordType = document.querySelector('input[name="coord"]:checked').value;
        let aLat = parseFloat(document.getElementById('aLat').value);
        let aLng = parseFloat(document.getElementById('aLng').value);
        let bLat = parseFloat(document.getElementById('bLat').value);
        let bLng = parseFloat(document.getElementById('bLng').value);
        if (coordType === 'WGS84') {
            let aGcj = wgs84ToGcj02(aLng, aLat);
            let bGcj = wgs84ToGcj02(bLng, bLat);
            return { a: [aGcj.lng, aGcj.lat], b: [bGcj.lng, bGcj.lat] };
        } else {
            return { a: [aLng, aLat], b: [bLng, bLat] };
        }
    }
    
    // 存储地图覆盖物
    let markerA = null, markerB = null, polyline = null, obstacleMarker = null;
    
    // 创建障碍物（红色立方体，使用自定义覆盖物，高德没有原生3D立方体，用 Marker 模拟+CSS或者用 Polygon 拉伸）
    // 为了效果，我们在中点添加一个红色3D柱状体（使用 AMap.Object3D 或简单添加一个圆形标记+图标）
    // 简单起见：添加一个圆形标记并显示红色方块，并添加一个 CSS 样式模拟立方体。
    // 更直观：使用海量点或者添加一个自定义图层。为了满足“障碍物圈选”，我们用一个醒目的红色标记+光圈。
    function createObstacle(lnglat) {
        if (obstacleMarker) map.remove(obstacleMarker);
        // 创建自定义内容 div 模拟立方体
        let content = '<div style="width: 30px; height: 30px; background-color: #ff0000; border: 2px solid yellow; border-radius: 4px; box-shadow: 0 0 15px red;"></div>';
        obstacleMarker = new AMap.Marker({
            position: lnglat,
            content: content,
            offset: new AMap.Pixel(-15, -15),
            title: "障碍物"
        });
        map.add(obstacleMarker);
        // 添加一个文本标签
        let labelMarker = new AMap.Marker({
            position: lnglat,
            content: '<div style="color: white; background: black; padding: 2px 6px; border-radius: 12px; font-size: 12px;">🚧 障碍物</div>',
            offset: new AMap.Pixel(-30, -40)
        });
        map.add(labelMarker);
        obstacleMarker.label = labelMarker;
    }
    
    function updateMap() {
        let coords = getMapCoords();
        let aPos = coords.a;
        let bPos = coords.b;
        
        if (markerA) map.remove(markerA);
        if (markerB) map.remove(markerB);
        if (polyline) map.remove(polyline);
        if (obstacleMarker) {
            map.remove(obstacleMarker);
            if (obstacleMarker.label) map.remove(obstacleMarker.label);
        }
        
        markerA = new AMap.Marker({ position: aPos, title: "A 起点", icon: 'https://webapi.amap.com/theme/v1.3/markers/n/mark_b.png', offset: new AMap.Pixel(-13, -30) });
        markerB = new AMap.Marker({ position: bPos, title: "B 终点", icon: 'https://webapi.amap.com/theme/v1.3/markers/n/mark_r.png', offset: new AMap.Pixel(-13, -30) });
        map.add([markerA, markerB]);
        
        // 连线
        polyline = new AMap.Polyline({ path: [aPos, bPos], strokeColor: "#ff3333", strokeWeight: 5, strokeOpacity: 0.8 });
        map.add(polyline);
        
        // 障碍物位于中点
        let midLng = (aPos[0] + bPos[0]) / 2;
        let midLat = (aPos[1] + bPos[1]) / 2;
        createObstacle([midLng, midLat]);
        
        document.getElementById('statusMsg').innerHTML = '✅ A/B 点已设，障碍物已生成';
        // 调整视野到合适范围
        map.setFitView([markerA, markerB, obstacleMarker]);
    }
    
    function highlightObstacle() {
        if (!obstacleMarker) { alert("请先设置A/B点生成障碍物"); return; }
        map.setZoomAndCenter(18, obstacleMarker.getPosition());
        // 闪烁效果
        let el = obstacleMarker.getContent();
        let originalBg = el.style.backgroundColor;
        el.style.backgroundColor = "#ffff00";
        setTimeout(() => { el.style.backgroundColor = originalBg; }, 800);
    }
    
    document.getElementById('setABtn').addEventListener('click', updateMap);
    document.getElementById('setBBtn').addEventListener('click', updateMap);
    document.getElementById('highlightBtn').addEventListener('click', highlightObstacle);
    document.querySelectorAll('input[name="coord"]').forEach(radio => radio.addEventListener('change', updateMap));
    
    setTimeout(updateMap, 1000);
</script>
</body>
</html>
""".replace("YOUR_AMAP_KEY", st.secrets.get("AMAP_KEY", "YOUR_AMAP_KEY"))  # 若在Streamlit Cloud可用secrets

# 心跳包监控（与之前相同，略作美化）
heartbeat_html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>心跳包监控</title>
    <style>
        body { background: #0b0f17; font-family: monospace; padding: 20px; color: #eef2ff; }
        .stats { display: flex; gap: 20px; margin-bottom: 25px; }
        .stat-card { background: #1e293b; border-radius: 20px; padding: 15px; flex:1; text-align: center; border-left: 4px solid #3b82f6; }
        .stat-value { font-size: 28px; font-weight: bold; color: #facc15; }
        .log-area { background: #0a0e16; border-radius: 20px; padding: 15px; height: 55vh; overflow-y: auto; }
        .log-entry { border-left: 3px solid #3b82f6; padding: 8px; margin: 8px 0; background: #111827; border-radius: 12px; }
        button { background: #2c3e66; border: none; padding: 8px 20px; border-radius: 40px; color: white; cursor: pointer; }
    </style>
</head>
<body>
<h2>📡 实时心跳数据流 (模拟无人机遥测)</h2>
<div class="stats">
    <div class="stat-card"><div>📶 信号强度</div><div class="stat-value" id="signalValue">-52 dBm</div></div>
    <div class="stat-card"><div>🔋 电池电量</div><div class="stat-value" id="batteryValue">98%</div></div>
    <div class="stat-card"><div>📍 无人机位置</div><div class="stat-value" id="posValue" style="font-size: 16px;">等待定位</div></div>
</div>
<button id="clearLogBtn">清空日志</button>
<div class="log-area" id="logArea"><div class="log-entry">✨ 心跳监控已启动...</div></div>
<script>
    let seq = 1, interval;
    const aLat=32.2322, aLng=118.749, bLat=32.2343, bLng=118.749;
    function getPos() {
        let t = (Date.now()/1000)%1;
        let lat = aLat + (bLat-aLat)*t + (Math.random()-0.5)*0.0005;
        let lng = aLng + (bLng-aLng)*t + (Math.random()-0.5)*0.0005;
        return {lat,lng};
    }
    function addLog() {
        let signal = Math.floor(40+Math.random()*35);
        let battery = Math.floor(65+Math.random()*35);
        let pos = getPos();
        let time = new Date().toLocaleTimeString();
        let logDiv = document.getElementById('logArea');
        let entry = document.createElement('div');
        entry.className = 'log-entry';
        entry.innerHTML = `<span style="color:#aaa;">[${time}]</span> ❤️ 心跳#${seq++} | 信号:-${signal}dBm | 电量:${battery}% | 位置:${pos.lat.toFixed(5)},${pos.lng.toFixed(5)} | 高度:50m`;
        logDiv.insertBefore(entry, logDiv.firstChild);
        while(logDiv.children.length>40) logDiv.removeChild(logDiv.lastChild);
        document.getElementById('signalValue').innerText = `-${signal} dBm`;
        document.getElementById('batteryValue').innerText = `${battery}%`;
        document.getElementById('posValue').innerHTML = `${pos.lat.toFixed(5)}°,<br>${pos.lng.toFixed(5)}°`;
    }
    function start() { if(interval) clearInterval(interval); addLog(); interval = setInterval(addLog, 2200); }
    document.getElementById('clearLogBtn').onclick = () => { document.getElementById('logArea').innerHTML = '<div class="log-entry">✨ 日志已清空...</div>'; };
    start();
</script>
</body>
</html>
"""

st.title("✈️ 无人机飞行规划与监控系统 (高德3D地图)")
st.markdown("**南京科技职业学院** · 真实卫星/矢量地图 | 支持3D倾斜、旋转")
tab1, tab2 = st.tabs(["🗺️ 航线规划（高德3D地图）", "📡 飞行监控（心跳包）"])

with tab1:
    st.components.v1.html(amap_html, height=700)
with tab2:
    st.components.v1.html(heartbeat_html, height=650)

with st.sidebar:
    st.markdown("### 📌 使用说明")
    st.markdown("""
    - **地图**：使用高德地图 JS API，真实卫星/矢量图，支持 3D 倾斜、旋转。
    - **坐标转换**：自动处理 WGS-84 与 GCJ-02 转换，确保标记位置准确。
    - **A/B 点**：输入经纬度后点击设置，地图上生成标记和连线。
    - **障碍物**：自动生成于连线中点，红色方块，点击“圈选”按钮聚焦高亮。
    - **心跳包**：在“飞行监控”标签页实时模拟。
    - **需要高德 Key**：请按第一步免费申请并替换代码中的 `YOUR_AMAP_KEY`。
    """)
    st.info("💡 提示：如果没有申请 Key，地图无法显示。申请后刷新页面即可。")
