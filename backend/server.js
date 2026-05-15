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
const PIXHAWK_PORTS = [14550, 14551, 14552, 14540];

const splitter = new MavLinkPacketSplitter();
const parser = new MavLinkPacketParser();
const REGISTRY = { ...common.REGISTRY };

/* ================= STATE ================= */
let latestRobotData = {
  roll: 0,
  pitch: 0,
  yaw: 0,
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

let thrusterDirs = {1:1,2:1,3:1,4:1,5:1,6:1};

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
      // armData.arm1_gripper  = toPwm(angles[2]); // Gripper is now digital
      armData.arm2_shoulder = toPwm(angles[3]);
      armData.arm2_rotate   = toPwm(angles[4]);
      // armData.arm2_gripper  = toPwm(angles[5]); // Gripper is now digital

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
const DEFAULT_BINDINGS = {
  axisX: 0, axisY: 1, axisYaw: 5, axisThrottle: 6,
  btnTrigger: 0, btnCalib: 8, btnGainUp: 12, btnGainDown: 13,
  btnVGainUp: 4, btnVGainDown: 5
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

/* ================= EXPRESS ================= */
app.use(express.json());
app.get('/', (req, res) => res.sendFile(join(__dirname, '../Frontend', 'index.html')));
app.use(express.static(join(__dirname, '../Frontend')));

app.get('/api/bindings', (req, res) => {
  res.json(savedBindings);
});

app.post('/api/bindings', (req, res) => {
  savedBindings = { ...DEFAULT_BINDINGS, ...req.body };
  saveBindingsToFile(savedBindings);
  console.log('💾 Bindings saved:', JSON.stringify(savedBindings));
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

const gst = spawn('gst-launch-1.0', [
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
]);

gst.stdout.on('data', chunk => {
  for (const res of camClients) res.write(chunk);
});
gst.stderr.on('data', d => console.log('🎥 GStreamer:', d.toString().trim()));
gst.on('exit', code => console.log('🎥 GStreamer exited:', code));

/* ================= MIXING ================= */
function clamp(v,min,max){return Math.max(min,Math.min(max,v));}

function mixThrusters(j){
  const g = clamp(j.gain||0.5,0,1);
  const vg = clamp(j.verticalGain||1,0,1);

  const x = clamp(j.x||0,-1,1);
  const y = clamp(j.y||0,-1,1);
  const yaw = clamp(j.yaw||0,-1,1);
  const vertical = clamp(j.vertical||0,-1,1) * vg;
  const roll = clamp(j.roll||0,-1,1);

  let fl = -y + x + yaw;
  let fr = -y - x - yaw;
  let rl = -y + x - yaw;
  let rr = -y - x + yaw;

  const maxH = Math.max(1,Math.abs(fl),Math.abs(fr),Math.abs(rl),Math.abs(rr));
  fl/=maxH; fr/=maxH; rl/=maxH; rr/=maxH;

  let vl = vertical + roll;
  let vr = vertical - roll;

  const maxV = Math.max(1,Math.abs(vl),Math.abs(vr));
  vl/=maxV; vr/=maxV;

  const pwmMin = 1500 - g * 400;
  const pwmMax = 1500 + g * 400;
  const toPwm = (v,id)=>
    Math.round(clamp(1500+v*thrusterDirs[id]*400, pwmMin, pwmMax));

  return [
    toPwm(fl,1),
    toPwm(fr,2),
    toPwm(rl,3),
    toPwm(rr,4),
    toPwm(vl,5),
    toPwm(vr,6)
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
      latestRobotData.roll  = Number((data.roll  * 180 / Math.PI).toFixed(1));
      latestRobotData.pitch = Number((data.pitch * 180 / Math.PI).toFixed(1));
      latestRobotData.yaw   = Number((data.yaw   * 180 / Math.PI).toFixed(1));
      io.emit('robot-data', latestRobotData);
    }
  } catch (e) {}
});

/* ================= SOCKET.IO ================= */
io.on('connection', socket=>{
  console.log('🎮 Client connected');

  socket.emit('robot-data', latestRobotData);

  socket.on('toggle-direction', (id) => {
    thrusterDirs[id] *= -1;
    io.emit('direction-update', { id, dir: thrusterDirs[id] });
  });

  socket.on('joystick-input', data=>{
    latestJoystick = data;

    const thrusters = mixThrusters(data);
    latestRobotData.thrusters = thrusters;

    sendToTeensy(thrusters, armData);

    io.emit('thruster-pwm', thrusters);
  });

  socket.on('arm-control', data=>{
    armData = {...data};

    sendToTeensy(latestRobotData.thrusters, armData);

    io.emit('arm-update', armData);
  });
});

/* ================= START ================= */
teensyUdp.bind(0);

server.listen(3000, ()=>{
  console.log('🌐 http://localhost:3000');
  console.log('🤖 Teensy:', TEENSY_IP+':'+TEENSY_PORT);
});


