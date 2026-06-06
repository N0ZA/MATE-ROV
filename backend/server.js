const express = require('express');
const { createServer } = require('http');
const { join } = require('path');
const { Server } = require('socket.io');
const dgram = require('dgram');
const { spawn } = require('child_process');
const fs = require('fs');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const { Worker } = require('worker_threads');

const app = express();
const server = createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

/* ================= TEENSY ================= */
const TEENSY_IP = process.env.TEENSY_IP || '192.168.2.177';
const TEENSY_PORT = 5000;

/* ================= STATE ================= */
let isArmed = true;

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
  arm1_gripper: 0, // 0 for stop, 1 for close, 2 for open
  arm2_slew: 1500,
  arm2_shoulder: 1500,
  arm2_rotate: 1500,
  arm2_gripper: 0 // 0 for stop, 1 for close, 2 for open
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

/* ================= SAFE ARDUINO SERIAL ================= */
const ARDUINO_PORT = process.env.ARDUINO_PORT || '/dev/ttyACM1';
const ARDUINO_BAUD = 115200;

function startArduino() {
  try {
    const arduinoSerial = new SerialPort({
      path: ARDUINO_PORT,
      baudRate: ARDUINO_BAUD,
      autoOpen: false
    });

    const parser = arduinoSerial.pipe(
      new ReadlineParser({ delimiter: '\n' })
    );

    arduinoSerial.open((err) => {
      if (err) {
        console.warn(`⚠️ Arduino NOT found on ${ARDUINO_PORT}`);
        return;
      }
      console.log(`✅ Arduino connected: ${ARDUINO_PORT}`);
    });

    parser.on('data', (line) => {
      line = line.trim();
      if (!line.startsWith('DATA:')) return;

      const parts = line.slice(5).split(',');
      if (parts.length !== 6) return;

      const angles = parts.map(Number);
      if (angles.some(isNaN)) return;

      const toPwm = (deg) => {
        const c = Math.max(0, Math.min(270, deg));
        if (Math.abs(c - 135) < 10) return 1500;
        return c < 135
          ? Math.round(1100 + (c/135)*400)
          : Math.round(1500 + ((c-135)/135)*400);
      };

      armData.arm1_shoulder = toPwm(angles[0]);
      armData.arm1_rotate   = toPwm(angles[1]);
      armData.arm2_shoulder = toPwm(angles[3]);
      armData.arm2_rotate   = toPwm(angles[4]);

      io.emit('arm-update', armData);
    });

    arduinoSerial.on('error', (err) => {
      console.warn('⚠️ Arduino error:', err.message);
    });

  } catch (e) {
    console.warn('⚠️ Arduino init failed:', e.message);
  }
}

startArduino();

/* ================= BINDINGS PERSISTENCE ================= */
const BINDINGS_FILE = join(__dirname, 'bindings.json');
// FIX: swapped axisX and axisYaw defaults to match physical controller layout
// axisX (sway) = axis 5 (was 0), axisYaw = axis 0 (was 5)
const DEFAULT_BINDINGS = {
  axisX: 5, axisY: 1, axisYaw: 0, axisThrottle: 6,
  btnTrigger: 0, btnCalib: 8,
  btnGainUp: 4, btnGainDown: 2, btnVGainUp: 5, btnVGainDown: 3,
  axisHatX: 4, axisHatY: 5
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

/* ================= AXIS INVERT PERSISTENCE ================= */
const INVERT_FILE = join(__dirname, 'invert.json');
const DEFAULT_INVERT = {
  surge: false, sway: false, heave: false, yaw: false, pitch: false, roll: false,
  depthHoldDir: false, rollHoldDir: false,
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
app.use(express.json());
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

/* ================= CAM 1 MJPEG ================= */
const camClients = new Set();

app.get('/cam1', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  camClients.add(res);
  req.on('close', () => camClients.delete(res));
});

const camClients2 = new Set();

app.get('/cam2', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  camClients2.add(res);
  req.on('close', () => camClients2.delete(res));
});

const camClients3 = new Set();

app.get('/cam3', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  camClients3.add(res);
  req.on('close', () => camClients3.delete(res));
});

const camClients4 = new Set();

app.get('/cam4', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  camClients4.add(res);
  req.on('close', () => camClients4.delete(res));
});

const camClients5 = new Set();

app.get('/cam5', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  camClients5.add(res);
  req.on('close', () => camClients5.delete(res));
});

const RTSP_URL1 = 'rtsp://192.168.2.2:8554/video_udp_stream_0';
const GST_ARGS = [
  'rtspsrc', `location=${RTSP_URL1}`, 'latency=0', 'protocols=tcp',
  '!', 'rtph264depay',
  '!', 'video/x-h264,stream-format=byte-stream,alignment=au',
  '!', 'nvv4l2decoder',
  '!', 'nvvidconv', 'flip-method=2',
  '!', 'video/x-raw,format=I420',
  '!', 'jpegenc', 'quality=80',
  '!', 'multipartmux', 'boundary=frame',
  '!', 'fdsink', 'fd=1'
];

let gstProc = null;

function startGStreamer() {
  gstProc = spawn('gst-launch-1.0', GST_ARGS);
  gstProc.on('error', (err) => {
    if (err.code === 'ENOENT') {
      console.warn('⚠️ GStreamer not found — camera feed disabled');
    } else {
      console.warn('⚠️ GStreamer error:', err.message, '— restarting in 3s');
      setTimeout(startGStreamer, 3000);
    }
    gstProc = null;
  });
  gstProc.stdout.on('data', chunk => {
    for (const res of camClients) res.write(chunk);
  });
  gstProc.stderr.on('data', d => console.log('🎥 GStreamer:', d.toString().trim()));
  gstProc.on('exit', (code, signal) => {
    if (!gstProc) return; // already handled by error event
    console.log('🎥 GStreamer exited:', code, '— restarting in 3s');
    gstProc = null;
    setTimeout(startGStreamer, 3000);
  });
}

startGStreamer();

/* ================= CAM 2 — RTSP ================= */
const RTSP_URL = 'rtsp://admin:admin@192.168.2.12:554/live/0/SUB';
const GST_RTSP_ARGS = [
  'rtspsrc', `location=${RTSP_URL}`, 'latency=0', 'protocols=tcp',
  '!', 'rtph265depay',
  '!', 'nvv4l2decoder',
  '!', 'nvvidconv',
  '!', 'video/x-raw,format=I420',
  '!', 'jpegenc', 'quality=80',
  '!', 'multipartmux', 'boundary=frame',
  '!', 'fdsink', 'fd=1'
];

let gstProc2 = null;

function startRtspCamera() {
  gstProc2 = spawn('gst-launch-1.0', GST_RTSP_ARGS);
  gstProc2.on('error', (err) => {
    if (err.code === 'ENOENT') {
      console.warn('⚠️ GStreamer not found — RTSP cam disabled');
    } else {
      console.warn('⚠️ RTSP cam error:', err.message, '— restarting in 5s');
      setTimeout(startRtspCamera, 5000);
    }
    gstProc2 = null;
  });
  gstProc2.stdout.on('data', chunk => {
    for (const res of camClients2) res.write(chunk);
  });
  gstProc2.stderr.on('data', d => console.log('📷 RTSP cam:', d.toString().trim()));
  gstProc2.on('exit', (code, signal) => {
    if (!gstProc2) return;
    console.log('📷 RTSP cam exited:', code, '— restarting in 5s');
    gstProc2 = null;
    setTimeout(startRtspCamera, 5000);
  });
}

startRtspCamera();

/* ================= CAM 3 — RTSP ================= */
const RTSP_URL3 = 'rtsp://admin:Admin123@192.168.2.13:554/live/0/SUB';
const GST_RTSP_ARGS3 = [
  'rtspsrc', `location=${RTSP_URL3}`, 'latency=0', 'protocols=tcp',
  '!', 'rtph265depay',
  '!', 'nvv4l2decoder',
  '!', 'nvvidconv',
  '!', 'video/x-raw,format=I420',
  '!', 'jpegenc', 'quality=80',
  '!', 'multipartmux', 'boundary=frame',
  '!', 'fdsink', 'fd=1'
];

let gstProc3 = null;

function startRtspCamera3() {
  gstProc3 = spawn('gst-launch-1.0', GST_RTSP_ARGS3);
  gstProc3.on('error', (err) => {
    if (err.code === 'ENOENT') {
      console.warn('⚠️ GStreamer not found — cam3 disabled');
    } else {
      console.warn('⚠️ Cam3 error:', err.message, '— restarting in 5s');
      setTimeout(startRtspCamera3, 5000);
    }
    gstProc3 = null;
  });
  gstProc3.stdout.on('data', chunk => {
    for (const res of camClients3) res.write(chunk);
  });
  gstProc3.stderr.on('data', d => console.log('📷 Cam3:', d.toString().trim()));
  gstProc3.on('exit', (code, signal) => {
    if (!gstProc3) return;
    console.log('📷 Cam3 exited:', code, '— restarting in 5s');
    gstProc3 = null;
    setTimeout(startRtspCamera3, 5000);
  });
}

startRtspCamera3();

/* ================= CAM 4 — RTSP ================= */
const RTSP_URL4 = 'rtsp://admin:Admin123@192.168.2.14:554/live/0/SUB';
const GST_RTSP_ARGS4 = [
  'rtspsrc', `location=${RTSP_URL4}`, 'latency=0', 'protocols=tcp',
  '!', 'rtph265depay',
  '!', 'nvv4l2decoder',
  '!', 'nvvidconv',
  '!', 'video/x-raw,format=I420',
  '!', 'jpegenc', 'quality=80',
  '!', 'multipartmux', 'boundary=frame',
  '!', 'fdsink', 'fd=1'
];

let gstProc4 = null;

function startRtspCamera4() {
  gstProc4 = spawn('gst-launch-1.0', GST_RTSP_ARGS4);
  gstProc4.on('error', (err) => {
    if (err.code === 'ENOENT') {
      console.warn('⚠️ GStreamer not found — cam4 disabled');
    } else {
      console.warn('⚠️ Cam4 error:', err.message, '— restarting in 5s');
      setTimeout(startRtspCamera4, 5000);
    }
    gstProc4 = null;
  });
  gstProc4.stdout.on('data', chunk => {
    for (const res of camClients4) res.write(chunk);
  });
  gstProc4.stderr.on('data', d => console.log('📷 Cam4:', d.toString().trim()));
  gstProc4.on('exit', (code, signal) => {
    if (!gstProc4) return;
    console.log('📷 Cam4 exited:', code, '— restarting in 5s');
    gstProc4 = null;
    setTimeout(startRtspCamera4, 5000);
  });
}

startRtspCamera4();

/* ================= CAM 5 — RTSP ================= */
const RTSP_URL5 = 'rtsp://admin:Admin123@192.168.2.15:554/live/0/SUB';
const GST_RTSP_ARGS5 = [
  'rtspsrc', `location=${RTSP_URL5}`, 'latency=0', 'protocols=tcp',
  '!', 'rtph265depay',
  '!', 'nvv4l2decoder',
  '!', 'nvvidconv',
  '!', 'video/x-raw,format=I420',
  '!', 'jpegenc', 'quality=80',
  '!', 'multipartmux', 'boundary=frame',
  '!', 'fdsink', 'fd=1'
];

let gstProc5 = null;

function startRtspCamera5() {
  gstProc5 = spawn('gst-launch-1.0', GST_RTSP_ARGS5);
  gstProc5.on('error', (err) => {
    if (err.code === 'ENOENT') {
      console.warn('⚠️ GStreamer not found — cam5 disabled');
    } else {
      console.warn('⚠️ Cam5 error:', err.message, '— restarting in 5s');
      setTimeout(startRtspCamera5, 5000);
    }
    gstProc5 = null;
  });
  gstProc5.stdout.on('data', chunk => {
    for (const res of camClients5) res.write(chunk);
  });
  gstProc5.stderr.on('data', d => console.log('📷 Cam5:', d.toString().trim()));
  gstProc5.on('exit', (code, signal) => {
    if (!gstProc5) return;
    console.log('📷 Cam5 exited:', code, '— restarting in 5s');
    gstProc5 = null;
    setTimeout(startRtspCamera5, 5000);
  });
}

startRtspCamera5();

function shutdown() {
  if (gstProc)  { gstProc.kill();  gstProc  = null; }
  if (gstProc2) { gstProc2.kill(); gstProc2 = null; }
  if (gstProc3) { gstProc3.kill(); gstProc3 = null; }
  if (gstProc4) { gstProc4.kill(); gstProc4 = null; }
  if (gstProc5) { gstProc5.kill(); gstProc5 = null; }
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

/* ================= SOCKET.IO ================= */
io.on('connection', socket=>{
  console.log('🎮 Client connected');

  socket.emit('robot-data', latestRobotData);
  socket.emit('calibration-state', { dirs: thrusterDirs, selected: thrusterSelected });

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
      const neutral = [1500,1500,1500,1500,1500,1500];
      latestRobotData.thrusters = neutral;
      io.emit('thruster-pwm', neutral);
    }
    console.log(isArmed ? '🟢 ARMED' : '🔴 DISARMED');
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
    if (!isNaN(rm3100Yaw))  latestRobotData.yaw   = Number(rm3100Yaw.toFixed(2));
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
camApp.use(express.static(join(__dirname, '../frontend')));
const camServer = createServer(camApp);
camServer.listen(3001, () => console.log('📷 Cameras  → http://localhost:3001'));
