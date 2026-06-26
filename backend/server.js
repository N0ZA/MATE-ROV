const express = require('express');
const { createServer } = require('http');
const { join } = require('path');
const { Server } = require('socket.io');
const dgram = require('dgram');
const { spawn } = require('child_process');
const fs = require('fs');
const { Worker } = require('worker_threads');

const app = express();
const server = createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

/* ================= TEENSY ================= */
const TEENSY_IP = process.env.TEENSY_IP || '192.168.2.177';
const TEENSY_PORT = 5000;

/* ================= STATE ================= */
let isArmed = true;
let cam2UndistortMode = false;
const CAM2_UNDISTORT_SCRIPT = join(__dirname, '../scripts/undistort_mjpeg.py');
const relayStates = { 1: false, 2: false, 3: false, 4: false };
let latestCamZoomState = { selectedSlot: -1, slotZooms: [1, 1, 1, 1] };

let latestRobotData = {
  roll: 0,
  pitch: 0,
  yaw: 0,
  depth: 0,
  thrusters: [1500,1500,1500,1500,1500,1500],
  imuOk: false,
  magOk: false
};

let latestJoystick = {
  x:0, y:0, yaw:0, vertical:0, roll:0, gain:1, verticalGain:1
};

let armData = {
  arm1_slew: 1500,
  arm1_shoulder: 1500,
  arm1_rotate: 1500,
  arm1_gripper: 1500, // PWM direct: 1300=open, 1700=close, 1500=neutral
  arm2_slew: 1500,
  arm2_shoulder: 1500,
  arm2_rotate: 1500,
  arm2_gripper: 1500, // PWM direct: 1300=open, 1700=close, 1500=neutral
};

const CALIBRATION_FILE = join(__dirname, 'calibration.json');
function loadCalibration() {
  try {
    const d = JSON.parse(fs.readFileSync(CALIBRATION_FILE, 'utf8'));
    return {
      thrusterDirs:     d.thrusterDirs     || {1:1,2:1,3:1,4:1,5:1,6:1},
      thrusterSelected: d.thrusterSelected || {1:false,2:false,3:false,4:false,5:false,6:false}
    };
  } catch {
    return {
      thrusterDirs:     {1:1,2:1,3:1,4:1,5:1,6:1},
      thrusterSelected: {1:false,2:false,3:false,4:false,5:false,6:false}
    };
  }
}
function saveCalibrationToFile() {
  fs.writeFileSync(CALIBRATION_FILE, JSON.stringify({ thrusterDirs, thrusterSelected }, null, 2));
}

const _cal = loadCalibration();
let thrusterDirs     = _cal.thrusterDirs;
let thrusterSelected = _cal.thrusterSelected;

/* ================= CONTROL WORKER ================= */
// jsState  Float32[7]: [x, y, yaw, vertical, roll, gain, verticalGain]
// ctrlState Int32[13]: [armed, dir1..6, sel1..6]
const jsBuffer   = new SharedArrayBuffer(7 * 4);
const jsState    = new Float32Array(jsBuffer);
const ctrlBuffer = new SharedArrayBuffer(13 * 4);
const ctrlState  = new Int32Array(ctrlBuffer);

jsState[5] = 0.5; // gain default
jsState[6] = 1.0; // verticalGain default

Atomics.store(ctrlState, 0, isArmed ? 1 : 0);
for (let i = 1; i <= 6; i++) Atomics.store(ctrlState, i,     thrusterDirs[i]);
for (let i = 1; i <= 6; i++) Atomics.store(ctrlState, 6 + i, thrusterSelected[i] ? 1 : 0);

const controlWorker = new Worker(join(__dirname, 'control-worker.js'), {
  workerData: { jsBuffer, ctrlBuffer, teensyIp: TEENSY_IP, teensyPort: TEENSY_PORT, initialArm: { ...armData } }
});
controlWorker.on('error', err => console.error('⚠️ Control worker error:', err));


/* ================= BINDINGS PERSISTENCE ================= */
const BINDINGS_FILE = join(__dirname, 'bindings.json');
// FIX: swapped axisX and axisYaw defaults to match physical controller layout
// axisX (sway) = axis 5 (was 0), axisYaw = axis 0 (was 5)
const DEFAULT_BINDINGS = {
  axisX: 5, axisY: 1, axisYaw: 0, axisThrottle: 6,
  btnTrigger: 0, btnCalib: 8,
  btnGainUp: 4, btnGainDown: 2, btnVGainUp: 5, btnVGainDown: 3,
  axisHatX: 4, axisHatY: 5,
  btnYawHoldGainUp: -1, btnYawHoldGainDown: -1,
  btnRollHoldGainUp: -1, btnRollHoldGainDown: -1
};

function loadBindings() {
  try {
    return JSON.parse(fs.readFileSync(BINDINGS_FILE, 'utf8'));
  } catch {
    return { ...DEFAULT_BINDINGS };
  }
}

function saveBindingsToFile(data) {
  fs.writeFileSync(BINDINGS_FILE, JSON.stringify(data, null, 2));
}

let savedBindings = loadBindings();

/* ================= ARM BINDINGS PERSISTENCE ================= */
const ARM_BINDINGS_FILE = join(__dirname, 'arm-bindings.json');

function loadArmBindings() {
  try {
    return JSON.parse(fs.readFileSync(ARM_BINDINGS_FILE, 'utf8'));
  } catch {
    return {};
  }
}

function saveArmBindingsToFile(data) {
  fs.writeFileSync(ARM_BINDINGS_FILE, JSON.stringify(data, null, 2));
}

let savedArmBindings = loadArmBindings();

/* ================= AXIS INVERT PERSISTENCE ================= */
const INVERT_FILE = join(__dirname, 'invert.json');
const DEFAULT_INVERT = {
  surge: false, sway: false, heave: false, yaw: false, pitch: false, roll: false,
  depthHoldDir: false, rollHoldDir: false, yawHoldDir: false,
  imuYaw: false, imuPitch: false, imuRoll: false,
  arm1Slew: false, arm1Shoulder: false, arm1Rotate: false, arm1Gripper: false,
  arm2Slew: false, arm2Shoulder: false, arm2Rotate: false, arm2Gripper: false
};

function loadInvert() {
  try {
    return { ...DEFAULT_INVERT, ...JSON.parse(fs.readFileSync(INVERT_FILE, 'utf8')) };
  } catch {
    return { ...DEFAULT_INVERT };
  }
}

function saveInvertToFile(data) {
  fs.writeFileSync(INVERT_FILE, JSON.stringify(data, null, 2));
}

let savedInvert = loadInvert();

/* ================= EXPRESS ================= */
app.use(express.json({ limit: '20mb' }));
app.get('/', (req, res) => res.sendFile(join(__dirname, '../frontend', 'index.html')));
app.use(express.static(join(__dirname, '../frontend')));
app.get('/api/bindings', (req, res) => {
  res.json(savedBindings);
});

app.post('/api/bindings', (req, res) => {
  savedBindings = { ...DEFAULT_BINDINGS, ...req.body };
  saveBindingsToFile(savedBindings);
  console.log('💾 Bindings saved:', JSON.stringify(savedBindings));
  res.json({ ok: true });
});

app.get('/api/arm-bindings', (req, res) => {
  res.json(savedArmBindings);
});

app.post('/api/arm-bindings', (req, res) => {
  savedArmBindings = req.body;
  saveArmBindingsToFile(savedArmBindings);
  console.log('💾 Arm bindings saved');
  res.json({ ok: true });
});

app.get('/api/invert', (req, res) => {
  res.json(savedInvert);
});

app.post('/api/invert', (req, res) => {
  savedInvert = { ...DEFAULT_INVERT, ...req.body };
  saveInvertToFile(savedInvert);
  console.log('💾 Axis invert saved:', JSON.stringify(savedInvert));
  res.json({ ok: true });
});

/* ================= RANGE PERSISTENCE ================= */
const RANGE_FILE = join(__dirname, 'range.json');

app.get('/api/range', (req, res) => {
  try {
    res.json(JSON.parse(fs.readFileSync(RANGE_FILE, 'utf8')));
  } catch {
    res.json({});
  }
});

app.post('/api/range', (req, res) => {
  fs.writeFileSync(RANGE_FILE, JSON.stringify(req.body, null, 2));
  console.log('💾 Range saved');
  res.json({ ok: true });
});

/* ================= KEEL KEY PERSISTENCE ================= */
const KEEL_SETTINGS_FILE = join(__dirname, 'keel-settings.json');

const KEEL_CORS = (res) => {
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
};

app.options('/api/keel-key',   (req, res) => { KEEL_CORS(res); res.sendStatus(204); });
app.options('/api/keel-depth', (req, res) => { KEEL_CORS(res); res.sendStatus(204); });

app.get('/api/keel-key', (req, res) => {
  KEEL_CORS(res);
  try {
    res.json(JSON.parse(fs.readFileSync(KEEL_SETTINGS_FILE, 'utf8')));
  } catch {
    res.json({ key: 'k' });
  }
});

app.post('/api/keel-key', (req, res) => {
  KEEL_CORS(res);
  const { key } = req.body;
  fs.writeFileSync(KEEL_SETTINGS_FILE, JSON.stringify({ key }, null, 2));
  console.log(`💾 Keel key saved: ${key}`);
  res.json({ ok: true });
});


/* ================= CAM MJPEG ================= */
const camClients2 = new Set();

app.get('/cam2', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Access-Control-Allow-Origin', '*');
  camClients2.add(res);
  req.on('close', () => camClients2.delete(res));
});

const camClients3 = new Set();

app.get('/cam3', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Access-Control-Allow-Origin', '*');
  camClients3.add(res);
  req.on('close', () => camClients3.delete(res));
});

const camClients4 = new Set();

app.get('/cam4', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Access-Control-Allow-Origin', '*');
  camClients4.add(res);
  req.on('close', () => camClients4.delete(res));
});

const camClients5 = new Set();

app.get('/cam5', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Access-Control-Allow-Origin', '*');
  camClients5.add(res);
  req.on('close', () => camClients5.delete(res));
});

/* ================= RTSP camera manager ================= */
// Two-phase watchdog:
//   CONNECT_MS — time allowed for GStreamer to establish RTSP and produce first frame.
//                Must be generous; killing too early causes an infinite restart loop.
//   STREAM_MS  — max gap between frames once streaming. Shorter so hangs recover fast.
const CONNECT_MS = 35000;
const STREAM_MS  = 12000;

function createRtspCam(label, url, clients, camNum) {
  const args = [
    'rtspsrc', `location=${url}`, 'latency=0', 'protocols=tcp',
    '!', 'rtph265depay',
    '!', 'nvv4l2decoder',
    '!', 'nvvidconv',
    '!', 'video/x-raw,format=I420',
    '!', 'jpegenc', 'quality=80',
    '!', 'multipartmux', 'boundary=frame',
    '!', 'fdsink', 'fd=1',
  ];

  let proc           = null;
  let watchdog       = null;
  let pendingRestart = false;
  let streaming      = false; // true once first frame received

  function setWatchdog(ms) {
    if (watchdog) clearTimeout(watchdog);
    watchdog = setTimeout(() => {
      const phase = streaming ? 'stream timeout' : 'connection timeout';
      console.warn(`⚠️ ${label} ${phase} — restarting`);
      if (proc) { proc.kill('SIGKILL'); proc = null; }
      // 2 s delay lets the camera release the RTSP session before we reconnect
      scheduleRestart(2000);
    }, ms);
  }

  function scheduleRestart(delay = 5000) {
    if (pendingRestart) return;
    pendingRestart = true;
    // Only notify the browser if the stream was live — startup retries should be invisible
    if (streaming) io.emit('cam-restart', { cam: camNum });
    setTimeout(() => { pendingRestart = false; start(); }, delay);
  }

  function start() {
    if (proc) return;
    streaming = false;
    proc = spawn('gst-launch-1.0', args);
    setWatchdog(CONNECT_MS); // generous timeout for initial RTSP handshake

    proc.on('error', err => {
      if (watchdog) { clearTimeout(watchdog); watchdog = null; }
      proc = null;
      if (err.code === 'ENOENT') { console.warn(`⚠️ GStreamer not found — ${label} disabled`); return; }
      scheduleRestart();
    });

    proc.stdout.on('data', chunk => {
      if (!streaming) {
        streaming = true;
        console.log(`✓ ${label} streaming`);
      }
      setWatchdog(STREAM_MS); // tighter watchdog now that stream is live
      for (const res of clients) {
        try {
          res.write(chunk);
          // Backpressure: if a client (throttled/hidden tab) falls >8MB behind,
          // drop it so it reconnects fresh instead of lagging minutes behind.
          if (res.writableLength > 8 * 1024 * 1024) {
            clients.delete(res);
            res.destroy();
          }
        } catch { clients.delete(res); }
      }
    });

    proc.stderr.on('data', () => {});

    proc.on('exit', () => {
      if (watchdog) { clearTimeout(watchdog); watchdog = null; }
      if (!proc) return; // already handled by watchdog
      proc = null;
      console.warn(`⚠️ ${label} exited — retrying in 5s`);
      scheduleRestart();
    });
  }

  function stop() {
    if (watchdog) { clearTimeout(watchdog); watchdog = null; }
    if (proc) { proc.kill(); proc = null; }
  }

  return { start, stop };
}

function createCam2(label, url, clients, camNum) {
  const args = [
    'rtspsrc', `location=${url}`, 'latency=0', 'protocols=tcp',
    '!', 'rtph265depay',
    '!', 'nvv4l2decoder',
    '!', 'nvvidconv',
    '!', 'video/x-raw,format=I420',
    '!', 'jpegenc', 'quality=80',
    '!', 'multipartmux', 'boundary=frame',
    '!', 'fdsink', 'fd=1',
  ];

  let proc           = null;
  let undistortProc  = null;
  let watchdog       = null;
  let pendingRestart = false;
  let streaming      = false;

  function setWatchdog(ms) {
    if (watchdog) clearTimeout(watchdog);
    watchdog = setTimeout(() => {
      const phase = streaming ? 'stream timeout' : 'connection timeout';
      console.warn(`⚠️ ${label} ${phase} — restarting`);
      if (undistortProc) { undistortProc.kill('SIGKILL'); undistortProc = null; }
      if (proc) { proc.kill('SIGKILL'); proc = null; }
      scheduleRestart(2000);
    }, ms);
  }

  function scheduleRestart(delay = 5000) {
    if (pendingRestart) return;
    pendingRestart = true;
    if (streaming) io.emit('cam-restart', { cam: camNum });
    setTimeout(() => { pendingRestart = false; start(); }, delay);
  }

  function start() {
    if (proc) return;
    streaming = false;
    proc = spawn('gst-launch-1.0', args);
    setWatchdog(CONNECT_MS);

    let dataSource = proc.stdout;

    if (cam2UndistortMode) {
      undistortProc = spawn('python3', [CAM2_UNDISTORT_SCRIPT]);
      proc.stdout.pipe(undistortProc.stdin);
      undistortProc.stderr.on('data', () => {});
      undistortProc.on('error', () => {});
      dataSource = undistortProc.stdout;
    }

    proc.on('error', err => {
      if (watchdog) { clearTimeout(watchdog); watchdog = null; }
      proc = null;
      if (undistortProc) { undistortProc.kill(); undistortProc = null; }
      if (err.code === 'ENOENT') { console.warn(`⚠️ GStreamer not found — ${label} disabled`); return; }
      scheduleRestart();
    });

    dataSource.on('data', chunk => {
      if (!streaming) {
        streaming = true;
        console.log(`✓ ${label} streaming${cam2UndistortMode ? ' (undistorted)' : ''}`);
      }
      setWatchdog(STREAM_MS);
      for (const res of clients) {
        try {
          res.write(chunk);
          if (res.writableLength > 8 * 1024 * 1024) {
            clients.delete(res);
            res.destroy();
          }
        } catch { clients.delete(res); }
      }
    });

    proc.stderr.on('data', () => {});

    proc.on('exit', () => {
      if (watchdog) { clearTimeout(watchdog); watchdog = null; }
      if (!proc) return;
      proc = null;
      if (undistortProc) { undistortProc.kill(); undistortProc = null; }
      console.warn(`⚠️ ${label} exited — retrying in 5s`);
      scheduleRestart();
    });
  }

  function stop() {
    if (watchdog) { clearTimeout(watchdog); watchdog = null; }
    if (undistortProc) { undistortProc.kill(); undistortProc = null; }
    if (proc) { proc.kill(); proc = null; }
  }

  return { start, stop };
}

const cam2 = createCam2('Cam2', 'rtsp://admin:Admin123@192.168.2.12:554/live/0/SUB', camClients2, 2);
const cam3 = createRtspCam('Cam3', 'rtsp://admin:Admin123@192.168.2.13:554/live/0/SUB', camClients3, 3);
const cam4 = createRtspCam('Cam4', 'rtsp://admin:Admin123@192.168.2.14:554/live/0/SUB', camClients4, 4);
const cam5 = createRtspCam('Cam5', 'rtsp://admin:Admin123@192.168.2.15:554/live/0/SUB', camClients5, 5);

cam2.start(); cam3.start(); cam4.start(); cam5.start();

function shutdown() {
  cam2.stop(); cam3.stop(); cam4.stop(); cam5.stop();
  process.exit(0);
}
process.on('SIGTERM', shutdown);
process.on('SIGINT', shutdown);

/* ================= MIXING ================= */
function clamp(v,min,max){return Math.max(min,Math.min(max,v));}

function mixThrusters(j){
  const g = clamp(j.gain||0.5,0,1);
  const vg = clamp(j.verticalGain||1,0,1);

  const x = clamp(j.x||0,-1,1);
  const y = clamp(j.y||0,-1,1);
  const yaw = clamp(j.yaw||0,-1,1);
  const vertical = clamp(j.vertical||0,-1,1);
  const roll = clamp(j.roll||0,-1,1);
  let fl = -y + yaw + x;
  let fr = -y - yaw - x;
  let rl = -y + yaw - x;
  let rr = -y - yaw + x;

  const maxH = Math.max(1,Math.abs(fl),Math.abs(fr),Math.abs(rl),Math.abs(rr));
  fl/=maxH; fr/=maxH; rl/=maxH; rr/=maxH;

  let vl = vertical + roll;
  let vr = vertical - roll;

  const maxV = Math.max(1,Math.abs(vl),Math.abs(vr));
  vl/=maxV; vr/=maxV;

  const pwmMinH = 1500 - g  * 300,  pwmMaxH = 1500 + g  * 300;
  const pwmMinV = 1500 - vg * 300,  pwmMaxV = 1500 + vg * 300;
  const toPwmH = (v,id) => Math.round(clamp(1500+v*thrusterDirs[id]*300, pwmMinH, pwmMaxH));
  const toPwmV = (v,id) => Math.round(clamp(1500+v*thrusterDirs[id]*300, pwmMinV, pwmMaxV));

  const anySelected = Object.values(thrusterSelected).some(v => v);
  const mask = id => (anySelected && !thrusterSelected[id]) ? 0 : 1;

  return [
    toPwmH(fl * mask(1), 1),
    toPwmH(fr * mask(2), 2),
    toPwmH(rl * mask(3), 3),
    toPwmH(rr * mask(4), 4),
    toPwmV(vl * mask(5), 5),
    toPwmV(vr * mask(6), 6)
  ];
}

/* ================= KEEL DEPTH ================= */
const KEEL_CAPTURE_PATH = join(__dirname, 'keel_capture.png');
const KEEL_SCRIPT       = join(__dirname, '../scripts/keelDepthDetection.py');
let keelProc = null;

app.post('/api/keel-depth', (req, res) => {
  KEEL_CORS(res);
  if (keelProc) return res.json({ error: 'already running' });
  const { image } = req.body;
  if (!image) return res.status(400).json({ error: 'no image' });
  const b64 = image.replace(/^data:image\/\w+;base64,/, '');
  fs.writeFileSync(KEEL_CAPTURE_PATH, Buffer.from(b64, 'base64'));
  keelProc = spawn('python3', [KEEL_SCRIPT, '--image', KEEL_CAPTURE_PATH], {
    env: { ...process.env, DISPLAY: process.env.DISPLAY || ':0' },
    stdio: 'inherit',
  });
  keelProc.on('error', (err) => {
    console.error('⚠️ Keel process error:', err.message);
    keelProc = null;
  });
  keelProc.on('exit', () => { keelProc = null; });
  console.log('📐 Keel depth tool opened');
  res.json({ ok: true });
});

/* ================= SOCKET.IO ================= */
io.on('connection', socket=>{
  console.log('🎮 Client connected');

  socket.emit('robot-data', latestRobotData);
  socket.emit('calibration-state', { dirs: thrusterDirs, selected: thrusterSelected });
  socket.emit('cam-zoom-state', latestCamZoomState);
  socket.emit('relay-states', relayStates);
  socket.emit('cam2-undistort-state', { active: cam2UndistortMode });

  socket.on('toggle-direction', (id) => {
    thrusterDirs[id] *= -1;
    Atomics.store(ctrlState, id, thrusterDirs[id]);
    io.emit('direction-update', { id, dir: thrusterDirs[id] });
  });

  socket.on('toggle-selection', (id) => {
    thrusterSelected[id] = !thrusterSelected[id];
    Atomics.store(ctrlState, 6 + id, thrusterSelected[id] ? 1 : 0);
    io.emit('selection-update', { id, selected: thrusterSelected[id] });
  });

  socket.on('save-calibration', () => {
    saveCalibrationToFile();
    console.log('💾 Calibration saved:', JSON.stringify({ thrusterDirs, thrusterSelected }));
    socket.emit('calibration-saved');
  });

  socket.on('set-armed', (state) => {
    isArmed = !!state;
    Atomics.store(ctrlState, 0, isArmed ? 1 : 0);
    if (!isArmed) {
      // Zero all joystick axes in shared memory so no stale signal can feed through,
      // regardless of mode (depth hold, yaw hold, etc.). Gains at [5] and [6] are kept.
      jsState[0] = 0; jsState[1] = 0; jsState[2] = 0;
      jsState[3] = 0; jsState[4] = 0;
      const neutral = [1500,1500,1500,1500,1500,1500];
      latestRobotData.thrusters = neutral;
      io.emit('thruster-pwm', neutral);
    }
    console.log(isArmed ? '🟢 ARMED' : '🔴 DISARMED');
  });

  socket.on('set-relay', ({ index, state }) => {
    relayStates[index] = !!state;
    controlWorker.postMessage({
      type: 'relay',
      relays: [1,2,3,4].map(i => relayStates[i] ? 1 : 0),
    });
    io.emit('relay-state', { index, state: !!state });
    console.log(`💡 Relay ${index} ${state ? 'ON' : 'OFF'}`);
  });

  socket.on('joystick-input', data=>{
    latestJoystick = data;
    if (!isArmed) return;

    // Write joystick state to shared memory — worker sends to Teensy at 100 Hz
    jsState[0] = data.x        || 0;
    jsState[1] = data.y        || 0;
    jsState[2] = data.yaw      || 0;
    jsState[3] = data.vertical || 0;
    jsState[4] = data.roll     || 0;
    jsState[5] = data.gain          !== undefined ? data.gain          : 0.5;
    jsState[6] = data.verticalGain  !== undefined ? data.verticalGain  : 1.0;

    const thrusters = mixThrusters(data);
    latestRobotData.thrusters = thrusters;
    io.emit('thruster-pwm', thrusters);
  });

  socket.on('arm-control', data=>{
    armData = {...data};
    controlWorker.postMessage({ type: 'arm', data: armData });
    io.emit('arm-update', armData);
  });

  socket.on('cam-zoom-state', (data) => {
    latestCamZoomState = data;
    socket.broadcast.emit('cam-zoom-state', data);
  });

  socket.on('promote-cam', (data) => {
    socket.broadcast.emit('promote-cam', data);
  });

  socket.on('key-forward', (data) => {
    socket.broadcast.emit('key-forward', data);
  });

  socket.on('cam2-undistort-toggle', () => {
    cam2UndistortMode = !cam2UndistortMode;
    io.emit('cam2-undistort-state', { active: cam2UndistortMode });
    io.emit('cam-restart', { cam: 2 });
    cam2.stop();
    setTimeout(() => cam2.start(), 1500);
    console.log(`🔭 Cam2 undistort: ${cam2UndistortMode ? 'ON' : 'OFF'}`);
  });
});

/* ================= CRAB DETECTION ================= */
const CAPTURE_PATH = join(__dirname, 'capture.png');
const INFER_OUT    = join(__dirname, 'infer_out.png');
const INFER_SCRIPT = join(__dirname, '../scripts/infer_crabs.py');

const CRAB_CORS = (res) => {
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
};

app.options('/api/capture-frame', (req, res) => { CRAB_CORS(res); res.sendStatus(204); });
app.options('/api/infer-crabs',   (req, res) => { CRAB_CORS(res); res.sendStatus(204); });

app.post('/api/capture-frame', (req, res) => {
  CRAB_CORS(res);
  const { image } = req.body;
  if (!image) return res.status(400).json({ ok: false, error: 'no image' });
  const base64 = image.replace(/^data:image\/\w+;base64,/, '');
  fs.writeFileSync(CAPTURE_PATH, Buffer.from(base64, 'base64'));
  res.json({ ok: true });
});

app.options('/api/save-snapshot', (req, res) => { CRAB_CORS(res); res.sendStatus(204); });

app.post('/api/save-snapshot', (req, res) => {
  CRAB_CORS(res);
  const { image, cam } = req.body;
  if (!image) return res.status(400).json({ ok: false, error: 'no image' });
  const now = new Date();
  const pad = n => String(n).padStart(2, '0');
  const ts  = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
  const datasetDir = join(__dirname, '../dataset');
  if (!fs.existsSync(datasetDir)) fs.mkdirSync(datasetDir, { recursive: true });
  const filename = `snapshot_${ts}${cam ? `_cam${cam}` : ''}.png`;
  const filepath = join(datasetDir, filename);
  const base64   = image.replace(/^data:image\/\w+;base64,/, '');
  fs.writeFileSync(filepath, Buffer.from(base64, 'base64'));
  console.log(`📸 Snapshot saved: ${filename}`);
  res.json({ ok: true, filename });
});

app.post('/api/infer-crabs', (req, res) => {
  CRAB_CORS(res);
  if (!fs.existsSync(CAPTURE_PATH)) return res.status(404).json({ error: 'no captured frame — press B first' });
  const proc = spawn('python3', [INFER_SCRIPT, CAPTURE_PATH, INFER_OUT]);
  let stdout = '', stderr = '';
  proc.stdout.on('data', d => { stdout += d; });
  proc.stderr.on('data', d => { stderr += d; });
  proc.on('close', code => {
    if (code !== 0) {
      console.error('infer_crabs error:', stderr.trim());
      return res.status(500).json({ error: stderr.slice(-200) || 'inference failed' });
    }
    try {
      const result = JSON.parse(stdout.trim());
      result.image = 'data:image/png;base64,' + fs.readFileSync(INFER_OUT).toString('base64');
      res.json(result);
    } catch (e) {
      res.status(500).json({ error: 'parse error: ' + stdout.slice(0, 100) });
    }
  });
});

/* ================= TEENSY TELEMETRY [ISM_ROLL,ISM_PITCH,RM3100_YAW,BAR30_DEPTH,imuOk,magOk] ================= */
const TEENSY_TELEMETRY_PORT = 5001;
const teensyTelemetry = dgram.createSocket('udp4');

teensyTelemetry.on('message', (raw) => {
  try {
    const telemetry = JSON.parse(raw.toString().trim());
    if (!Array.isArray(telemetry) || telemetry.length < 4) return;
    const [ismRoll, ismPitch, ismYaw, rm3100Yaw, bar30Depth, imuOkVal, magOkVal] = telemetry;
    if (!isNaN(ismRoll))    latestRobotData.roll  = Number(ismRoll.toFixed(2));
    if (!isNaN(ismPitch))   latestRobotData.pitch = Number(ismPitch.toFixed(2));
    if (!isNaN(ismYaw))  latestRobotData.yaw   = Number(ismYaw.toFixed(2));
    if (!isNaN(bar30Depth)) latestRobotData.depth = Number(Math.max(0, bar30Depth).toFixed(3));
    latestRobotData.imuOk = imuOkVal !== undefined ? !!imuOkVal : latestRobotData.imuOk;
    latestRobotData.magOk = magOkVal !== undefined ? !!magOkVal : latestRobotData.magOk;
    io.emit('robot-data', { ...latestRobotData, teensyLive: true });
  } catch (e) {}
});

teensyTelemetry.bind(TEENSY_TELEMETRY_PORT, '0.0.0.0', () => {
  console.log(`📡 Teensy telemetry listening on port ${TEENSY_TELEMETRY_PORT}`);
});

/* ================= START ================= */
server.listen(3000, ()=>{
  console.log('🌐 Controls → http://localhost:3000');
  console.log('🤖 Teensy:', TEENSY_IP+':'+TEENSY_PORT);
});

/* ================= CAMERA PAGE (port 3001) ================= */
const camApp = express();
camApp.get('/', (req, res) => res.sendFile(join(__dirname, '../frontend', 'camera.html')));

// Serve the /camN streams on 3001 as well. Browsers cap ~6 concurrent
// HTTP/1.1 connections per host:port, and MJPEG holds one open per feed.
// Pilot UI strip (4 feeds on :3000) + camera page (4 feeds) exceeded the
// cap when both pointed at :3000, starving feeds. Splitting them across
// two ports keeps each page within the per-port budget. The client Sets
// are shared, so the same GStreamer process fans out to both ports.
const camRouteSets = { 2: camClients2, 3: camClients3, 4: camClients4, 5: camClients5 };
for (const [n, clients] of Object.entries(camRouteSets)) {
  camApp.get(`/cam${n}`, (req, res) => {
    res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
    res.setHeader('Cache-Control', 'no-cache');
    res.setHeader('Access-Control-Allow-Origin', '*');
    clients.add(res);
    req.on('close', () => clients.delete(res));
  });
}

camApp.use(express.static(join(__dirname, '../frontend')));
const camServer = createServer(camApp);
camServer.listen(3001, () => console.log('📷 Cameras  → http://localhost:3001'));