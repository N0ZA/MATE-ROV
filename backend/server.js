const express = require('express');
const { createServer } = require('node:http');
const { join } = require('node:path');
const { Server } = require('socket.io');
const dgram = require('dgram');
const { MavLinkPacketSplitter, MavLinkPacketParser, common } = require('node-mavlink');

const app = express();
const server = createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

// --- PIXHAWK ETHERNET UDP CONFIGURATION ---
const PIXHAWK_UDP_PORT = 14550; // Standard MAVLink UDP port
const PIXHAWK_IP = process.env.PIXHAWK_IP || '192.168.2.10'; // Set your Pixhawk IP
const udp = dgram.createSocket('udp4');

// --- MAVLINK PARSER ---
const REGISTRY = { ...common.REGISTRY };
const splitter = new MavLinkPacketSplitter();
const parser = new MavLinkPacketParser();

// Pipe UDP -> MAVLink Parser
udp.on('message', (msg) => {
  splitter.write(Buffer.from(msg));
});

// --- DATA STATE (Pixhawk Ethernet Only) ---
let latestRobotData = {
  roll: 0,
  pitch: 0,
  yaw: 0,
  thrusters: [1500, 1500, 1500, 1500, 1500, 1500]
};

let latestJoystick = {
  x: 0,
  y: 0,
  yaw: 0,
  vertical: 0,
  pitch: 0,
  roll: 0,
  gain: 0.3,
  stability: false
};

let lastSentPwms = [1500, 1500, 1500, 1500, 1500, 1500];

// --- SERVE STATIC FILES ---
app.get('/', (req, res) => {
  res.sendFile(join(__dirname, 'index.html'));
});
app.use(express.static('.'));

// --- THRUSTER CONTROL ---
let thrusterDirs = {1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1};

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function mixThrusters(j) {
  const gain = clamp(Number(j.gain) || 0.7, 0, 1);
  const x = clamp(Number(j.x) || 0, -1, 1) * gain;
  const y = clamp(Number(j.y) || 0, -1, 1) * gain;
  const yaw = clamp(Number(j.yaw) || 0, -1, 1) * gain;
  const vertical = clamp(Number(j.vertical) || 0, -1, 1) * gain;
  const roll = clamp(Number(j.roll) || 0, -1, 1) * gain;

  let fl = -y + x + yaw;
  let fr = -y - x - yaw;
  let rl = -y + x - yaw;
  let rr = -y - x + yaw;

  const maxH = Math.max(1, Math.abs(fl), Math.abs(fr), Math.abs(rl), Math.abs(rr));
  fl /= maxH; fr /= maxH; rl /= maxH; rr /= maxH;

  let vl = vertical + roll;
  let vr = vertical - roll;
  const maxV = Math.max(1, Math.abs(vl), Math.abs(vr));
  vl /= maxV; vr /= maxV;

  const toPwm = (v, id) => Math.round(clamp(1500 + (v * thrusterDirs[id]) * 400, 1100, 1900));
  return [toPwm(fl, 1), toPwm(fr, 2), toPwm(rl, 3), toPwm(rr, 4), toPwm(vl, 5), toPwm(vr, 6)];
}

const TEENSY_IP = process.env.TEENSY_IP || '192.168.2.177';
const TEENSY_PORT = Number(process.env.TEENSY_PORT || 5000);
const teensyUdp = dgram.createSocket('udp4');

function sendPwmsToTeensy(pwms) {
  const msg = Buffer.from(JSON.stringify({ type: 'thrusters', pwms, ts: Date.now() }));
  teensyUdp.send(msg, TEENSY_PORT, TEENSY_IP, (err) => {
    if (err) console.error('Teensy UDP send error:', err.message);
  });
}

function broadcastRobotData() {
  io.emit('robot-data', { ...latestRobotData, joystick: latestJoystick });
}

// --- PIXHAWK ETHERNET MAVLINK HANDLER ---
parser.on('data', (packet) => {
  const msgid = packet.header.msgid;
  const clazz = REGISTRY[msgid];
  if (clazz) {
    try {
      const data = packet.protocol.data(packet.payload, clazz);
      if (data.roll !== undefined) { // ATTITUDE message
        const imuData = {
          roll: Number((data.roll * 180 / Math.PI).toFixed(1)),
          pitch: Number((data.pitch * 180 / Math.PI).toFixed(1)),
          yaw: Number((data.yaw * 180 / Math.PI).toFixed(1))
        };
        
        latestRobotData.roll = imuData.roll;
        latestRobotData.pitch = imuData.pitch;
        latestRobotData.yaw = imuData.yaw;
        
        io.emit('imu-update', imuData);
        console.log(`✅ PIXHAWK Ethernet: R:${imuData.roll}° P:${imuData.pitch}° Y:${imuData.yaw}°`);
        broadcastRobotData();
      }
    } catch (e) {
      // Ignore parse errors
    }
  }
});

// --- SOCKET.IO HANDLERS ---
io.on('connection', (socket) => {
  console.log('Socket.IO client connected:', socket.id);
  socket.emit('robot-data', { ...latestRobotData, joystick: latestJoystick });

  socket.on('toggle-direction', (id) => {
    thrusterDirs[id] = thrusterDirs[id] * -1;
    io.emit('direction-update', { id, dir: thrusterDirs[id] });
  });

  socket.on('joystick-input', (data) => {
    latestJoystick = {
      x: clamp(Number(data?.x) || 0, -1, 1),
      y: clamp(Number(data?.y) || 0, -1, 1),
      yaw: clamp(Number(data?.yaw) || 0, -1, 1),
      vertical: clamp(Number(data?.vertical) || 0, -1, 1),
      pitch: clamp(Number(data?.pitch) || 0, -1, 1),
      roll: clamp(Number(data?.roll) || 0, -1, 1),
      gain: clamp(Number(data?.gain) || 0.5, 0, 1),
      stability: Boolean(data?.stability)
    };

    const pwms = mixThrusters(latestJoystick);
    latestRobotData.thrusters = pwms;
    lastSentPwms = pwms;

    sendPwmsToTeensy(pwms);
    broadcastRobotData();
    io.emit('thruster-pwm', pwms);
  });
});

// --- START SERVERS ---
udp.bind(PIXHAWK_UDP_PORT, '0.0.0.0', () => {
  console.log(`🎯 Pixhawk UDP listener on 0.0.0.0:${PIXHAWK_UDP_PORT}`);
  console.log(`📡 Expecting Pixhawk from ${PIXHAWK_IP}`);
});

server.listen(3000, () => {
  console.log('🌐 PIXHAWK Ethernet Backend active at http://localhost:3000');
});
