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
            background: rgba(0,0,0,0.85);
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
        button:hover { background: #2563eb; }
        button.obstacle-btn { background: #a855f7; }
        .coord-radio { display: flex; gap: 15px; margin: 8px 0; }
        .status { font-size: 12px; text-align: center; margin-top: 8px; color: #aaf; }
        .note { font-size: 11px; text-align: center; margin-top: 10px; color: #ccc; }
    </style>
    <script type="text/javascript">
        window._AMapSecurityConfig = {
            securityJsCode: 'YOUR_SECURITY_CODE'
        };
    </script>
    <script src="https://webapi.amap.com/maps?v=2.0&key=YOUR_AMAP_KEY"></script>
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
        <button id="setABtn">✈️ 设置 A 点</button>
    </div>
    <div class="section">
        <div>📍 终点 B (校园内)</div>
        <div class="input-group">
            <input type="number" id="bLat" value="32.2343" step="0.0001" placeholder="纬度">
            <input type="number" id="bLng" value="118.7490" step="0.0001" placeholder="经度">
        </div>
        <button id="setBBtn">🎯 设置 B 点</button>
    </div>
    <div class="section">
        <div>🧱 障碍物</div>
        <button id="highlightBtn" class="obstacle-btn">🔍 定位障碍物</button>
        <div class="status" id="statusMsg">⚪ 等待设置 A/B 点</div>
    </div>
    <div class="note">💡 鼠标拖拽旋转 / 右键平移 / 滚轮缩放 | 红色方块为障碍物</div>
</div>

<script>
    // 初始化地图
    var map = new AMap.Map('container', {
        center: [118.749, 32.2332],
        zoom: 18,
        pitch: 65,
        viewMode: '3D',
        layers: [new AMap.TileLayer.Satellite()],
        building: true,
        showIndoorMap: false
    });
    
    // 添加控件
    map.addControl(new AMap.Scale());
    map.addControl(new AMap.ToolBar({ position: 'RT' }));
    map.addControl(new AMap.ControlBar({ position: 'RB' }));
    
    // WGS84 转 GCJ02
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
    
    // 获取坐标
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
    
    // 存储覆盖物
    let markerA = null, markerB = null, polyline = null, obstacleMarker = null, textA = null, textB = null;
    
    // 创建A点标记
    function createMarkerA(position) {
        // 使用高德原生的Marker并自定义样式
        var markerContent = document.createElement('div');
        markerContent.style.backgroundColor = '#00ff00';
        markerContent.style.width = '40px';
        markerContent.style.height = '40px';
        markerContent.style.borderRadius = '50%';
        markerContent.style.border = '3px solid white';
        markerContent.style.boxShadow = '0 0 10px rgba(0,0,0,0.5)';
        markerContent.style.display = 'flex';
        markerContent.style.alignItems = 'center';
        markerContent.style.justifyContent = 'center';
        markerContent.style.fontSize = '20px';
        markerContent.style.fontWeight = 'bold';
        markerContent.style.color = 'black';
        markerContent.innerHTML = 'A';
        
        return new AMap.Marker({
            position: position,
            content: markerContent,
            offset: new AMap.Pixel(-20, -20),
            anchor: 'center',
            title: '起点 A'
        });
    }
    
    // 创建B点标记
    function createMarkerB(position) {
        var markerContent = document.createElement('div');
        markerContent.style.backgroundColor = '#ff4444';
        markerContent.style.width = '40px';
        markerContent.style.height = '40px';
        markerContent.style.borderRadius = '50%';
        markerContent.style.border = '3px solid white';
        markerContent.style.boxShadow = '0 0 10px rgba(0,0,0,0.5)';
        markerContent.style.display = 'flex';
        markerContent.style.alignItems = 'center';
        markerContent.style.justifyContent = 'center';
        markerContent.style.fontSize = '20px';
        markerContent.style.fontWeight = 'bold';
        markerContent.style.color = 'white';
        markerContent.innerHTML = 'B';
        
        return new AMap.Marker({
            position: position,
            content: markerContent,
            offset: new AMap.Pixel(-20, -20),
            anchor: 'center',
            title: '终点 B'
        });
    }
    
    // 创建文字标签
    function createTextLabel(position, text, offsetY) {
        var div = document.createElement('div');
        div.style.backgroundColor = 'rgba(0,0,0,0.7)';
        div.style.color = 'white';
        div.style.padding = '4px 10px';
        div.style.borderRadius = '20px';
        div.style.fontSize = '12px';
        div.style.whiteSpace = 'nowrap';
        div.style.fontWeight = 'bold';
        div.innerHTML = text;
        
        return new AMap.Marker({
            position: [position[0], position[1] + offsetY],
            content: div,
            offset: new AMap.Pixel(-30, -10),
            anchor: 'top-center'
        });
    }
    
    // 创建障碍物
    function createObstacle(position) {
        if (obstacleMarker) map.remove(obstacleMarker);
        
        var obstacleDiv = document.createElement('div');
        obstacleDiv.style.width = '40px';
        obstacleDiv.style.height = '40px';
        obstacleDiv.style.backgroundColor = '#ff0000';
        obstacleDiv.style.border = '3px solid #ffff00';
        obstacleDiv.style.borderRadius = '6px';
        obstacleDiv.style.boxShadow = '0 0 15px rgba(255,0,0,0.8)';
        obstacleDiv.style.display = 'flex';
        obstacleDiv.style.alignItems = 'center';
        obstacleDiv.style.justifyContent = 'center';
        obstacleDiv.innerHTML = '⚠️';
        obstacleDiv.style.fontSize = '20px';
        
        obstacleMarker = new AMap.Marker({
            position: position,
            content: obstacleDiv,
            offset: new AMap.Pixel(-20, -20),
            anchor: 'center',
            title: '障碍物'
        });
        map.add(obstacleMarker);
        
        // 添加障碍物文字标签
        var labelDiv = document.createElement('div');
        labelDiv.style.backgroundColor = 'rgba(0,0,0,0.7)';
        labelDiv.style.color = '#ff8888';
        labelDiv.style.padding = '2px 8px';
        labelDiv.style.borderRadius = '12px';
        labelDiv.style.fontSize = '11px';
        labelDiv.innerHTML = '🚧 障碍物';
        
        var obstacleLabel = new AMap.Marker({
            position: [position[0], position[1] + 0.0001],
            content: labelDiv,
            offset: new AMap.Pixel(-30, -5)
        });
        map.add(obstacleLabel);
        window.obstacleLabel = obstacleLabel;
    }
    
    // 更新地图
    function updateMap() {
        try {
            let coords = getMapCoords();
            let aPos = coords.a;
            let bPos = coords.b;
            
            console.log('A点坐标:', aPos);
            console.log('B点坐标:', bPos);
            
            // 清除旧的覆盖物
            if (markerA) map.remove(markerA);
            if (markerB) map.remove(markerB);
            if (polyline) map.remove(polyline);
            if (window.obstacleLabel) map.remove(window.obstacleLabel);
            if (textA) map.remove(textA);
            if (textB) map.remove(textB);
            
            // 创建新的标记
            markerA = createMarkerA(aPos);
            markerB = createMarkerB(bPos);
            textA = createTextLabel(aPos, '📍 起点 A', -0.00008);
            textB = createTextLabel(bPos, '🏁 终点 B', -0.00008);
            
            map.add([markerA, markerB, textA, textB]);
            
            // 绘制连线
            polyline = new AMap.Polyline({
                path: [aPos, bPos],
                strokeColor: '#ff3333',
                strokeWeight: 4,
                strokeOpacity: 0.8,
                strokeStyle: 'solid',
                lineJoin: 'round',
                lineCap: 'round'
            });
            map.add(polyline);
            
            // 计算中点并创建障碍物
            let midLng = (aPos[0] + bPos[0]) / 2;
            let midLat = (aPos[1] + bPos[1]) / 2;
            createObstacle([midLng, midLat]);
            
            // 更新状态
            document.getElementById('statusMsg').innerHTML = '✅ A/B 点已显示，障碍物已生成';
            document.getElementById('statusMsg').style.color = '#90ee90';
            
            // 调整视野
            setTimeout(() => {
                map.setFitView([markerA, markerB, obstacleMarker], false, [50, 50, 50, 50]);
            }, 100);
            
        } catch(e) {
            console.error('更新地图出错:', e);
            document.getElementById('statusMsg').innerHTML = '❌ 更新失败: ' + e.message;
        }
    }
    
    // 定位障碍物
    function locateObstacle() {
        if (!obstacleMarker) {
            alert('请先设置 A/B 点生成障碍物');
            return;
        }
        map.setZoomAndCenter(19, obstacleMarker.getPosition());
        // 闪烁效果
        var content = obstacleMarker.getContent();
        var originalBg = content.style.backgroundColor;
        content.style.backgroundColor = '#ffff00';
        setTimeout(() => {
            if (obstacleMarker) content.style.backgroundColor = originalBg;
        }, 500);
    }
    
    // 绑定事件
    document.getElementById('setABtn').addEventListener('click', updateMap);
    document.getElementById('setBBtn').addEventListener('click', updateMap);
    document.getElementById('highlightBtn').addEventListener('click', locateObstacle);
    document.querySelectorAll('input[name="coord"]').forEach(radio => {
        radio.addEventListener('change', updateMap);
    });
    
    // 页面加载完成后初始化
    window.onload = function() {
        setTimeout(updateMap, 1000);
    };
    
    // 确保地图完全加载后再执行一次
    map.on('complete', function() {
        console.log('地图加载完成');
        setTimeout(updateMap, 500);
    });
</script>
</body>
</html>
""".replace("YOUR_AMAP_KEY", "17bf012d2daaa0963ed83efdcf079fa0").replace("YOUR_SECURITY_CODE", "7f6e2b9c8d4a1e5f3b7c9d2e8a4f6b1c")

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
        let lat = aLat + (bLat-aLat)*t + (Math.random()-0.5)*0.0003;
        let lng = aLng + (bLng-aLng)*t + (Math.random()-0.5)*0.0003;
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
        entry.innerHTML = `<span style="color:#aaa;">[${time}]</span> ❤️ 心跳#${seq++} | 信号:-${signal}dBm | 电量:${battery}% | 位置:${pos.lat.toFixed(6)},${pos.lng.toFixed(6)} | 高度:50m | 速度:12m/s`;
        logDiv.insertBefore(entry, logDiv.firstChild);
        while(logDiv.children.length>40) logDiv.removeChild(logDiv.lastChild);
        document.getElementById('signalValue').innerText = `-${signal} dBm`;
        document.getElementById('batteryValue').innerText = `${battery}%`;
        document.getElementById('posValue').innerHTML = `${pos.lat.toFixed(6)}°<br>${pos.lng.toFixed(6)}°`;
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
    **🗺️ 地图操作**
    - 鼠标拖拽：旋转视角
    - 右键拖拽：平移地图
    - 滚轮：缩放地图
    
    **📍 航线规划**
    1. 输入A点（起点）经纬度
    2. 输入B点（终点）经纬度
    3. 点击"设置A点"或"设置B点"
    4. 地图自动显示标记、连线和障碍物
    
    **🎯 标记说明**
    - 🟢 **绿色圆点A**：起点
    - 🔴 **红色圆点B**：终点
    - 🟥 **红色方块**：障碍物（位于航线中点）
    
    **📡 飞行监控**
    - 实时显示心跳数据
    - 模拟信号强度、电量、位置
    - 自动更新无人机位置
    
    **⚠️ 重要提示**
    - 请确保网络连接正常
    - 卫星地图加载可能需要几秒钟
    - 如需使用自己的高德Key，请替换代码中的密钥
    """)
    
    st.success("✅ 系统已就绪，点击按钮开始规划航线")
    
    st.info("📌 **示例坐标**\n- A点: 32.2322, 118.7490\n- B点: 32.2343, 118.7490")
