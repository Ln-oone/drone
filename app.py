import streamlit as st

st.set_page_config(page_title="无人机飞行规划与监控系统", page_icon="✈️", layout="wide")

# 高德卫星地图 HTML（3D，支持倾斜/旋转，坐标转换，AB点，障碍物）
amap_satellite_html = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>高德卫星地图 - 3D航线规划</title>
    <style>
        body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; }
        #container { width: 100%; height: 100%; }
        .control-panel {
            position: absolute;
            top: 20px;
            right: 20px;
            width: 290px;
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
        button.obstacle-btn { background: #a855f7; }
        .coord-radio { display: flex; gap: 15px; margin: 8px 0; }
        .status { font-size: 12px; text-align: center; margin-top: 8px; color: #aaf; }
        .note { font-size: 11px; text-align: center; margin-top: 10px; color: #ccc; }
    </style>
    <!-- 高德地图安全密钥配置 -->
    <script type="text/javascript">
        window._AMapSecurityConfig = {
            securityJsCode: '7f6e2b9c8d4a1e5f3b7c9d2e8a4f6b1c'   // 请替换为您的安全密钥
        };
    </script>
    <!-- 引入高德地图 JS API -->
    <script src="https://webapi.amap.com/maps?v=2.0&key=17bf012d2daaa0963ed83efdcf079fa0"></script>
</head>
<body>
<div id="container"></div>
<div class="control-panel">
    <h3>🗺️ 高德卫星3D地图 · 航线规划</h3>
    <div class="section">
        <div class="coord-radio" id="coordSysGroup">
            <label><input type="radio" name="coord" value="GCJ02" checked> GCJ-02 (高德/百度)</label>
            <label><input type="radio" name="coord" value="WGS84"> WGS-84</label>
        </div>
        <div class="note">注：高德地图使用 GCJ-02，输入WGS84会自动转换</div>
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
        <button id="highlightBtn" class="obstacle-btn">🔍 圈选 / 高亮障碍物</button>
        <div class="status" id="statusMsg">⚪ 未设置 A/B 点</div>
    </div>
    <div class="note">💡 鼠标拖拽旋转 / 右键平移 / 滚轮缩放 | 红色方块为障碍物</div>
</div>

<script>
    // 初始化卫星地图（3D视图）
    var map = new AMap.Map('container', {
        center: [118.749, 32.2332],   // 南京科技职业学院
        zoom: 18,
        pitch: 65,                    // 倾斜角度（3D效果）
        viewMode: '3D',               // 3D视图
        layers: [new AMap.TileLayer.Satellite()],  // 使用卫星图层
        building: true,               // 显示3D建筑物
        showIndoorMap: false
    });
    
    // 添加控件
    map.addControl(new AMap.Scale());
    map.addControl(new AMap.ToolBar({ position: 'RT' }));
    map.addControl(new AMap.ControlBar({ position: 'RB' }));  // 3D旋转控件
    
    // ---------- 坐标转换：WGS84 -> GCJ02（高德地图使用GCJ02）----------
    function wgs84ToGcj02(lng, lat) {
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
        return { lng: lng + dLng, lat: lat + dLat };
    }
    
    // 获取用户输入坐标（转换为高德使用的GCJ02坐标）
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
    let markerA = null, markerB = null, polyline = null, obstacleMarker = null, obstacleLabel = null;
    
    // 创建自定义标记点（使用 DIV 样式，更可靠）
    function createCustomMarker(color, label, offset = [-15, -30]) {
        let content = document.createElement('div');
        content.innerHTML = `
            <div style="
                width: 30px;
                height: 30px;
                background-color: ${color};
                border: 3px solid white;
                border-radius: 50%;
                box-shadow: 0 0 10px rgba(0,0,0,0.5);
                display: flex;
                align-items: center;
                justify-content: center;
                font-weight: bold;
                font-size: 16px;
                color: white;
            ">${label}</div>
        `;
        return new AMap.Marker({
            position: [0, 0],
            content: content,
            offset: new AMap.Pixel(offset[0], offset[1]),
            anchor: 'bottom-center'
        });
    }
    
    // 创建障碍物（红色方块+标签）
    function createObstacle(lnglat) {
        if (obstacleMarker) map.remove(obstacleMarker);
        if (obstacleLabel) map.remove(obstacleLabel);
        // 自定义红色方块内容
        let content = document.createElement('div');
        content.innerHTML = '<div style="width: 30px; height: 30px; background-color: #ff0000; border: 2px solid yellow; border-radius: 4px; box-shadow: 0 0 15px red;"></div>';
        obstacleMarker = new AMap.Marker({
            position: lnglat,
            content: content,
            offset: new AMap.Pixel(-15, -15),
            title: "障碍物"
        });
        map.add(obstacleMarker);
        // 添加文字标签
        let labelContent = document.createElement('div');
        labelContent.innerHTML = '<div style="color: white; background: rgba(0,0,0,0.7); padding: 2px 6px; border-radius: 12px; font-size: 12px; white-space: nowrap;">🚧 障碍物</div>';
        obstacleLabel = new AMap.Marker({
            position: lnglat,
            content: labelContent,
            offset: new AMap.Pixel(-30, -40)
        });
        map.add(obstacleLabel);
    }
    
    function updateMap() {
        let coords = getMapCoords();
        let aPos = coords.a;
        let bPos = coords.b;
        
        // 移除旧覆盖物
        if (markerA) map.remove(markerA);
        if (markerB) map.remove(markerB);
        if (polyline) map.remove(polyline);
        if (obstacleMarker) {
            map.remove(obstacleMarker);
            if (obstacleLabel) map.remove(obstacleLabel);
        }
        
        // 创建 A 点标记（绿色，显示"A"）
        let aContent = document.createElement('div');
        aContent.innerHTML = '<div style="width: 32px; height: 32px; background-color: #00ff00; border: 3px solid white; border-radius: 50%; box-shadow: 0 0 10px rgba(0,0,0,0.5); display: flex; align-items: center; justify-content: center; font-weight: bold; font-size: 16px; color: black;">A</div>';
        markerA = new AMap.Marker({
            position: aPos,
            content: aContent,
            offset: new AMap.Pixel(-16, -16),
            title: "起点 A"
        });
        
        // 创建 B 点标记（红色，显示"B"）
        let bContent = document.createElement('div');
        bContent.innerHTML = '<div style="width: 32px; height: 32px; background-color: #ff4444; border: 3px solid white; border-radius: 50%; box-shadow: 0 0 10px rgba(0,0,0,0.5); display: flex; align-items: center; justify-content: center; font-weight: bold; font-size: 16px; color: white;">B</div>';
        markerB = new AMap.Marker({
            position: bPos,
            content: bContent,
            offset: new AMap.Pixel(-16, -16),
            title: "终点 B"
        });
        
        map.add([markerA, markerB]);
        
        // 添加文字标签（可选，增强显示）
        let aLabel = document.createElement('div');
        aLabel.innerHTML = '<div style="color: white; background: rgba(0,0,0,0.6); padding: 2px 6px; border-radius: 12px; font-size: 11px; margin-top: -25px; white-space: nowrap;">📍 起点</div>';
        let aLabelMarker = new AMap.Marker({
            position: [aPos[0], aPos[1] + 0.00008],
            content: aLabel,
            offset: new AMap.Pixel(-25, -10)
        });
        
        let bLabel = document.createElement('div');
        bLabel.innerHTML = '<div style="color: white; background: rgba(0,0,0,0.6); padding: 2px 6px; border-radius: 12px; font-size: 11px; margin-top: -25px; white-space: nowrap;">🏁 终点</div>';
        let bLabelMarker = new AMap.Marker({
            position: [bPos[0], bPos[1] + 0.00008],
            content: bLabel,
            offset: new AMap.Pixel(-25, -10)
        });
        
        map.add([aLabelMarker, bLabelMarker]);
        
        // 存储以便后续清理
        window.aLabelMarker = aLabelMarker;
        window.bLabelMarker = bLabelMarker;
        
        // 连线（红色虚线加粗）
        polyline = new AMap.Polyline({ 
            path: [aPos, bPos], 
            strokeColor: "#ff3333", 
            strokeWeight: 5, 
            strokeOpacity: 0.9,
            strokeStyle: "solid"
        });
        map.add(polyline);
        
        // 障碍物位于连线中点
        let midLng = (aPos[0] + bPos[0]) / 2;
        let midLat = (aPos[1] + bPos[1]) / 2;
        createObstacle([midLng, midLat]);
        
        document.getElementById('statusMsg').innerHTML = '✅ A/B 点已设，障碍物已生成';
        // 自动调整视野包含所有元素
        map.setFitView([markerA, markerB, obstacleMarker], false, [50, 50, 50, 50]);
    }
    
    function highlightObstacle() {
        if (!obstacleMarker) { alert("请先设置 A/B 点生成障碍物"); return; }
        map.setZoomAndCenter(20, obstacleMarker.getPosition());
        // 闪烁效果
        let contentDiv = obstacleMarker.getContent();
        if (contentDiv && contentDiv.firstChild) {
            let targetDiv = contentDiv.firstChild;
            let originalBg = targetDiv.style.backgroundColor;
            targetDiv.style.backgroundColor = "#ffff00";
            setTimeout(() => { 
                if(obstacleMarker && targetDiv) targetDiv.style.backgroundColor = originalBg; 
            }, 800);
        }
    }
    
    // 绑定事件
    document.getElementById('setABtn').addEventListener('click', updateMap);
    document.getElementById('setBBtn').addEventListener('click', updateMap);
    document.getElementById('highlightBtn').addEventListener('click', highlightObstacle);
    document.querySelectorAll('input[name="coord"]').forEach(radio => radio.addEventListener('change', updateMap));
    
    // 延迟确保地图完全加载后初始化点
    setTimeout(updateMap, 1500);
</script>
</body>
</html>
"""

# 飞行监控（心跳包模拟）
heartbeat_html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>心跳包监控</title>
    <style>
        body { background: #0b0f17; font-family: monospace; padding: 20px; color: #eef2ff; }
        .stats { display: flex; gap: 20px; margin-bottom: 25px; flex-wrap: wrap; }
        .stat-card { background: #1e293b; border-radius: 20px; padding: 15px; flex:1; text-align: center; border-left: 4px solid #3b82f6; }
        .stat-value { font-size: 28px; font-weight: bold; color: #facc15; }
        .log-area { background: #0a0e16; border-radius: 20px; padding: 15px; height: 55vh; overflow-y: auto; }
        .log-entry { border-left: 3px solid #3b82f6; padding: 8px; margin: 8px 0; background: #111827; border-radius: 12px; }
        button { background: #2c3e66; border: none; padding: 8px 20px; border-radius: 40px; color: white; cursor: pointer; margin-top: 15px; }
        h2 { margin-top: 0; }
    </style>
</head>
<body>
<div style="max-width: 1000px; margin: 0 auto;">
    <h2>📡 实时心跳数据流 (模拟无人机遥测)</h2>
    <div class="stats">
        <div class="stat-card"><div>📶 信号强度</div><div class="stat-value" id="signalValue">-52 dBm</div></div>
        <div class="stat-card"><div>🔋 电池电量</div><div class="stat-value" id="batteryValue">98%</div></div>
        <div class="stat-card"><div>📍 无人机位置</div><div class="stat-value" id="posValue" style="font-size: 16px;">等待定位</div></div>
    </div>
    <div style="text-align: right;"><button id="clearLogBtn">清空日志</button></div>
    <div class="log-area" id="logArea"><div class="log-entry">✨ 心跳监控已启动，等待数据包...</div></div>
</div>
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

st.title("✈️ 无人机飞行规划与监控系统 (高德卫星3D地图)")
st.markdown("**南京科技职业学院** · 真实卫星影像 + 3D地形 | 支持鼠标拖拽/右键旋转/滚轮缩放")

tab1, tab2 = st.tabs(["🗺️ 航线规划（高德卫星3D地图）", "📡 飞行监控（心跳包）"])

with tab1:
    st.components.v1.html(amap_satellite_html, height=700, scrolling=False)

with tab2:
    st.components.v1.html(heartbeat_html, height=650, scrolling=False)

with st.sidebar:
    st.markdown("### 🧭 使用说明")
    st.markdown("""
    - **地图切换**：自动处理 WGS-84 ↔ GCJ-02，确保标记准确。
    - **A/B点**：输入经纬度后点击设置，地图上生成彩色圆点标记和红色连线。
    - **障碍物**：自动生成于连线中点，红色方块，点击"圈选"按钮聚焦高亮。
    - **心跳包**：在"飞行监控"标签页实时模拟无人机遥测数据。
    - **卫星图加载稍慢**，请耐心等待几秒。
    """)
    st.info("📌 **提示**：A点为绿色圆点带'A'字母，B点为红色圆点带'B'字母")
    st.warning("⚠️ 请将高德地图API Key和安全密钥替换为您自己的")
