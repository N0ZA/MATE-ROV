<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>ROV Mission Control v3</title>
    <script src="/socket.io/socket.io.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body { background: #0f172a; color: #f8fafc; font-family: 'Segoe UI', sans-serif; margin: 0; padding: 15px; height: 100vh; overflow: hidden; }
        .main-layout { display: grid; grid-template-columns: 300px 1fr 280px; gap: 15px; height: 95vh; }
        .card { background: #1e293b; padding: 12px; border-radius: 8px; border: 1px solid #334155; }
        .column { display: flex; flex-direction: column; gap: 15px; height: 100%; }
        
        /* LEFT COLUMN STYLES */
        select { width: 100%; padding: 10px; background: #0f172a; color: #38bdf8; border: 1px solid #38bdf8; border-radius: 4px; font-weight: bold; margin-bottom: 5px; }
        .thruster-graph-box { background: #020617; padding: 5px; border-radius: 4px; height: 50px; border: 1px solid #334155; margin-bottom: 4px; }
        .joy-val { font-family: monospace; color: #38bdf8; font-weight: bold; }

        /* CENTER COLUMN STYLES */
        .camera-grid { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: 1fr 1fr; gap: 10px; }
        .video-box { background: #000; border: 2px solid #475569; border-radius: 4px; display: flex; align-items: center; justify-content: center; position: relative; color: #475569; font-weight: bold; font-size: 12px; }
        .cam-tag { position: absolute; top: 8px; left: 8px; background: rgba(0,0,0,0.7); font-size: 10px; padding: 3px 8px; color: #38bdf8; border-radius: 2px; }

        /* RIGHT COLUMN STYLES */
        .compass { width: 100px; height: 100px; border: 3px solid #38bdf8; border-radius: 50%; position: relative; margin: 8px auto; background: #000; overflow: hidden; }
        .yaw-needle { width: 4px; height: 50px; background: #ef4444; position: absolute; left: 50%; bottom: 50%; transform-origin: bottom center; }
        .horizon-line { width: 100%; height: 2px; background: #22c55e; position: absolute; top: 50%; }

        h3 { font-size: 11px; margin: 0 0 10px 0; color: #38bdf8; text-transform: uppercase; letter-spacing: 1px; border-bottom: 1px solid #334155; padding-bottom: 4px; }
        .label { font-size: 10px; color: #94a3b8; text-transform: uppercase; }
        .val-num { font-family: monospace; font-size: 18px; color: #38bdf8; font-weight: bold; }
    </style>
</head>
<body>
    <div class="main-layout">
        
        <div class="column">
            <div class="card">
                <h3>DRIVE MODE</h3>
                <select>
                    <option>MANUAL</option>
                    <option>STABILIZE</option>
                    <option>YAW HOLD</option>
                    <option>DEPTH HOLD</option>
                    <option>POSITION HOLD</option>
                </select>
            </div>
            
            <div class="card">
                <h3>JOYSTICK MAPPING</h3>
                <div style="display: flex; justify-content: space-between;">
                    <div><span class="label">X:</span> <span id="jx" class="joy-val">0.00</span></div>
                    <div><span class="label">Y:</span> <span id="jy" class="joy-val">0.00</span></div>
                    <div><span class="label">Z:</span> <span id="jz" class="joy-val">0.00</span></div>
                </div>
            </div>

            <div class="card" style="flex-grow:1; overflow-y:auto;">
                <h3>THRUSTER PWM</h3>
                <div id="thruster-list"></div>
            </div>
        </div>

        <div class="camera-grid">
            <div class="video-box"><span class="cam-tag">CAM 1 - PRIMARY</span>NO SIGNAL</div>
            <div class="video-box"><span class="cam-tag">CAM 2 - REAR</span>NO SIGNAL</div>
            <div class="video-box"><span class="cam-tag">CAM 3 - TOOL</span>NO SIGNAL</div>
            <div class="video-box"><span class="cam-tag">CAM 4 - BELLY</span>NO SIGNAL</div>
        </div>

        <div class="column">
            <div class="card" style="text-align:center;">
                <h3>ORIENTATION</h3>
                <div class="label">YAW</div>
                <div class="compass"><div class="yaw-needle" id="yaw-needle"></div></div>
                
                <div class="label" style="margin-top:10px;">PITCH & ROLL</div>
                <div class="compass" style="border-color:#22c55e;"><div class="horizon-line" id="pr-line"></div></div>
                <div style="font-size:11px; margin-top:5px;">P: <span id="pv">0</span>° | R: <span id="rv">0</span>°</div>
            </div>

            <div class="card">
                <h3>BAR30 SENSOR</h3>
                <div class="label">Depth Reading</div>
                <div class="val-num" id="depth">0.00 m</div>
                <hr style="border:0; border-top:1px solid #334155; margin:10px 0;">
                <h3>NETWORK</h3>
                <div class="label">Latency</div>
                <div style="color:#22c55e; font-weight:bold;"><span id="ping">0</span> ms</div>
            </div>
        </div>
    </div>

    <script>
        const socket = io();
        const charts = [];
        const container = document.getElementById('thruster-list');

        // Create 7 Graphs
        for(let i=1; i<=7; i++) {
            const div = document.createElement('div');
            div.className = 'thruster-graph-box';
            div.innerHTML = `<span class="label" style="font-size:8px;">Thruster ${i}</span><canvas id="t${i}"></canvas>`;
            container.appendChild(div);
            charts.push(new Chart(document.getElementById(`t${i}`).getContext('2d'), {
                type: 'line',
                data: { labels: Array(20).fill(''), datasets: [{ data: Array(20).fill(0), borderColor: '#38bdf8', borderWidth: 1, pointRadius: 0, tension: 0.2 }] },
                options: { animation: false, responsive: true, maintainAspectRatio: false, scales: { y: { min: 0, max: 255, display: false }, x: { display: false } }, plugins: { legend: { display: false } } }
            }));
        }

        socket.on('robot-data', (data) => {
            // Update Visuals
            document.getElementById('yaw-needle').style.transform = `translateX(-50%) rotate(${data.yaw}deg)`;
            document.getElementById('pr-line').style.transform = `rotate(${data.roll}deg) translateY(${data.pitch * 2}px)`;
            
            // Update Text
            document.getElementById('pv').innerText = Math.round(data.pitch);
            document.getElementById('rv').innerText = Math.round(data.roll);
            document.getElementById('depth').innerText = data.depth.toFixed(2) + " m";
            document.getElementById('ping').innerText = data.ping;
            document.getElementById('jx').innerText = data.joy.x.toFixed(2);
            document.getElementById('jy').innerText = data.joy.y.toFixed(2);
            document.getElementById('jz').innerText = data.joy.z.toFixed(2);

            // Update Thruster Graphs
            data.thrusters.forEach((v, i) => {
                if(charts[i]) {
                    charts[i].data.datasets[0].data.push(v);
                    charts[i].data.datasets[0].data.shift();
                    charts[i].update();
                }
            });
        });
    </script>
</body>
</html>