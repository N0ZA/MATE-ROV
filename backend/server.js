const express = require('express');
const { createServer } = require('http');
const { join } = require('path');
const { Server } = require('socket.io');
const dgram = require('dgram');
const { spawn } = require('child_process');
const fs = require('fs');
const { MavLinkPacketSplitter, MavLinkPacketParser, common } = require('node-mavlink');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const server = createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

/* ================= TEENSY ================= */
const TEENSY_IP = process.env.TEENSY_IP || '192.168.2.177';
const TEENSY_PORT = 5000;
const teensyUdp = dgram.createSocket('udp4');

/* ================= PIXHAWK ================= */
const PIXHAWK_PORTS = [5600];

const splitter = new MavLinkPacketSplitter();
const parser = new MavLinkPacketParser();
const REGISTRY = { ...common.REGISTRY };

/* ================= STATE ================= */
let isArmed = true;

let latestRobotData = {
  roll: 0,
  pitch: 0,
  yaw: 0,
  depth: 0,
  thrusters: [1500,1500,1500,1500,1500,1500]
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
let pixhawkOffset    = null;
let surfacePressure  = null; // hPa — captured on first Bar30 packet

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

/* ================= CAM 1 MJPEG ================= */
const camClients = new Set();

app.get('/cam1', (req, res) => {
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  camClients.add(res);
  req.on('close', () => camClients.delete(res));
});

['cam2','cam3','cam4','cam5'].forEach(cam => {
  app.get(`/${cam}`, (req, res) => {
    res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
    res.setHeader('Cache-Control', 'no-cache');
    req.on('close', () => {});
  });
});

const GST_ARGS = [
  'udpsrc', 'port=5600',
  '!', 'application/x-rtp,media=video,clock-rate=90000,encoding-name=H264',
  '!', 'rtph264depay', '!', 'h264parse',
  '!', 'nvv4l2decoder',
  '!', 'nvvidconv',
  '!', 'video/x-raw,format=I420',
  '!', 'videoflip', 'method=rotate-180',
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

function shutdown() {
  if (gstProc) { gstProc.kill(); gstProc = null; }
  process.exit(0);
}
process.on('SIGTERM', shutdown);
process.on('SIGINT', shutdown);

/* ================= MIXING ================= */
function clamp(v,min,max){return Math.max(min,Math.min(max,v));}
function wrapAngle(deg){return ((deg%360)+540)%360-180;}

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

/* ================= TEENSY SEND ================= */
function sendToTeensy(thrusters, arm){
  const getGripperValues = (state) => {
    if (state === 1) return [0, 1]; // Close
    if (state === 2) return [1, 0]; // Open
    return [0, 0]; // Stop
  };

  const packet = [
    ...thrusters,
    arm.arm1_slew,
    arm.arm1_shoulder,
    arm.arm1_rotate,
    ...getGripperValues(arm.arm1_gripper),
    arm.arm2_slew,
    arm.arm2_shoulder,
    arm.arm2_rotate,
    ...getGripperValues(arm.arm2_gripper)
  ];

  const msg = Buffer.from(JSON.stringify({
    type:"all",
    pwms: packet,
    ts: Date.now()
  }));

  teensyUdp.send(msg, TEENSY_PORT, TEENSY_IP);
}

/* ================= PIXHAWK ================= */
PIXHAWK_PORTS.forEach(port=>{
  const sock = dgram.createSocket('udp4');
  sock.on('message', msg=>splitter.write(msg));
  sock.bind(port,'0.0.0.0',()=>console.log('🎯 Pixhawk',port));
});

splitter.pipe(parser);

parser.on('data', (packet) => {
  const msgid = packet.header.msgid;
  const clazz = REGISTRY[msgid];
  if (!clazz || !packet.protocol?.data) return;
  try {
    const data = packet.protocol.data(packet.payload, clazz);
    if (data.roll !== undefined) {
      const rawPitch = Number((data.pitch * 180 / Math.PI).toFixed(1));
      const rawYaw   = Number((data.yaw   * 180 / Math.PI).toFixed(1));

      if (!pixhawkOffset) {
        const rawRoll = Number((data.roll * 180 / Math.PI).toFixed(1));
        pixhawkOffset = { roll: rawRoll, pitch: rawPitch, yaw: rawYaw };
        console.log('🧭 Pixhawk offset set:', pixhawkOffset);
      }

      latestRobotData.pitch = Number((rawPitch - pixhawkOffset.pitch).toFixed(1));
      latestRobotData.yaw   = Number(wrapAngle(rawYaw - pixhawkOffset.yaw).toFixed(1));
      io.emit('robot-data', { ...latestRobotData, pixhawkLive: true });
    }
  } catch (e) {}
});

/* ================= SOCKET.IO ================= */
io.on('connection', socket=>{
  console.log('🎮 Client connected');

  socket.emit('robot-data', latestRobotData);
  socket.emit('calibration-state', { dirs: thrusterDirs, selected: thrusterSelected });

  socket.on('toggle-direction', (id) => {
    thrusterDirs[id] *= -1;
    io.emit('direction-update', { id, dir: thrusterDirs[id] });
  });

  socket.on('toggle-selection', (id) => {
    thrusterSelected[id] = !thrusterSelected[id];
    io.emit('selection-update', { id, selected: thrusterSelected[id] });
  });

  socket.on('save-calibration', () => {
    saveCalibrationToFile();
    console.log('💾 Calibration saved:', JSON.stringify({ thrusterDirs, thrusterSelected }));
    socket.emit('calibration-saved');
  });

  socket.on('set-armed', (state) => {
    isArmed = !!state;
    if (!isArmed) {
      const neutral = [1500,1500,1500,1500,1500,1500];
      latestRobotData.thrusters = neutral;
      sendToTeensy(neutral, armData);
      io.emit('thruster-pwm', neutral);
    }
    console.log(isArmed ? '🟢 ARMED' : '🔴 DISARMED');
  });

  socket.on('joystick-input', data=>{
    latestJoystick = data;
    if (!isArmed) return;

    const thrusters = mixThrusters(data);
    latestRobotData.thrusters = thrusters;

    sendToTeensy(thrusters, armData);

    io.emit('thruster-pwm', thrusters);
  });

  socket.on('arm-control', data=>{
    armData = {...data};

    const thrusters = isArmed ? latestRobotData.thrusters : [1500,1500,1500,1500,1500,1500];
    sendToTeensy(thrusters, armData);

    io.emit('arm-update', armData);
  });
});

/* ================= TEENSY TELEMETRY (roll + depth) ================= */
const TEENSY_TELEMETRY_PORT = 5000;
const teensyTelemetry = dgram.createSocket('udp4');

teensyTelemetry.on('message', (raw) => {
  try {
    const telemetry = JSON.parse(raw.toString().trim());
    if (!Array.isArray(telemetry) || telemetry.length < 2) return;
    const [roll, depth] = telemetry;
    if (!isNaN(roll))  latestRobotData.roll  = Number(roll.toFixed(1));
    if (!isNaN(depth)) latestRobotData.depth = Number(Math.max(0, depth).toFixed(2));
    io.emit('robot-data', { ...latestRobotData });
  } catch (e) {}
});

teensyTelemetry.bind(TEENSY_TELEMETRY_PORT, '0.0.0.0', () => {
  console.log(`📡 Teensy telemetry listening on port ${TEENSY_TELEMETRY_PORT}`);
});

/* ================= START ================= */
teensyUdp.bind(0);

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