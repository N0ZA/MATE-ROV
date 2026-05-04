const express = require('express');
const { createServer } = require('node:http');
const { join } = require('node:path');
const { Server } = require('socket.io');
const dgram = require('dgram');
const { MavLinkPacketSplitter, MavLinkPacketParser, common } = require('node-mavlink');

const app = express();
const server = createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

// PIXHAWK: 192.168.2.2
const PIXHAWK_IP = '192.168.2.2';
const PIXHAWK_PORTS = [14550, 14551, 14552, 14540]; // Try all common ports

// TEENSY
const TEENSY_IP = process.env.TEENSY_IP || '192.168.2.177';
const TEENSY_PORT = 5000;

// UDP Sockets + MAVLink
const pixhawkSockets = {};
const splitter = new MavLinkPacketSplitter();
const parser = new MavLinkPacketParser();
const REGISTRY = { ...common.REGISTRY };
const teensyUdp = dgram.createSocket('udp4');

let latestRobotData = { roll: 0, pitch: 0, yaw: 0, thrusters: [1500,1500,1500,1500,1500,1500], depth: 0, ping: 0 };
let latestJoystick = { x: 0, y: 0, yaw: 0, vertical: 0, pitch: 0, roll: 0, gain: 0.3, stability: false };
let thrusterDirs = {1:1,2:1,3:1,4:1,5:1,6:1};

// Serve GUI
app.get('/', (req, res) => res.sendFile(join(__dirname, 'index.html')));
app.use(express.static('.'));

// Multi-port Pixhawk listener
PIXHAWK_PORTS.forEach(port => {
  const sock = dgram.createSocket('udp4');
  pixhawkSockets[port] = sock;
  
  sock.on('message', (msg, rinfo) => {
    console.log(`📡 PIXHAWK ${rinfo.address}:${rinfo.port} → port ${port} (${msg.length}B)`);
    splitter.write(msg);
  });
  
  sock.bind(port, '0.0.0.0', () => {
    console.log(`🎯 Listening UDP ${port}`);
  });
});

// MAVLink → GUI
splitter.pipe(parser);
parser.on('data', (packet) => {
  const msgid = packet.header.msgid;
  const clazz = REGISTRY[msgid];
  if (clazz && packet.protocol?.data) {
    try {
      const data = packet.protocol.data(packet.payload, clazz);
      if (data.roll !== undefined) { // ATTITUDE (msg 30)
        latestRobotData.roll = Number((data.roll * 180 / Math.PI).toFixed(1));
        latestRobotData.pitch = Number((data.pitch * 180 / Math.PI).toFixed(1));
        latestRobotData.yaw = Number((data.yaw * 180 / Math.PI).toFixed(1));
        
        io.emit('imu-update', latestRobotData);
        io.emit('robot-data', { ...latestRobotData, joystick: latestJoystick });
        console.log(`✅ R:${latestRobotData.roll.toFixed(1)}° P:${latestRobotData.pitch.toFixed(1)}° Y:${latestRobotData.yaw.toFixed(1)}°`);
      }
    } catch(e) {}
  }
});

// Thruster mixing (your code)
function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

function mixThrusters(j) {
  const gain = clamp(j.gain || 0.7, 0, 1);
  const x = clamp(j.x || 0, -1, 1) * gain;
  const y = clamp(j.y || 0, -1, 1) * gain;
  const yaw = clamp(j.yaw || 0, -1, 1) * gain;
  const vertical = clamp(j.vertical || 0, -1, 1) * gain;
  const roll = clamp(j.roll || 0, -1, 1) * gain;

  let fl = -y + x + yaw, fr = -y - x - yaw, rl = -y + x - yaw, rr = -y - x + yaw;
  const maxH = Math.max(1, Math.abs(fl), Math.abs(fr), Math.abs(rl), Math.abs(rr));
  fl /= maxH; fr /= maxH; rl /= maxH; rr /= maxH;

  let vl = vertical + roll, vr = vertical - roll;
  const maxV = Math.max(1, Math.abs(vl), Math.abs(vr));
  vl /= maxV; vr /= maxV;

  const toPwm = (v, id) => Math.round(clamp(1500 + v * thrusterDirs[id] * 400, 1300, 1700));
  return [toPwm(fl,1), toPwm(fr,2), toPwm(rl,3), toPwm(rr,4), toPwm(vl,5), toPwm(vr,6)];
}

function sendPwmsToTeensy(pwms) {
  const msg = Buffer.from(JSON.stringify({ type: 'thrusters', pwms, ts: Date.now() }));
  teensyUdp.send(msg, TEENSY_PORT, TEENSY_IP, (err) => {
    if(err) console.error('❌ Teensy:', err.message);
  });
}

// Socket.IO
io.on('connection', (socket) => {
  console.log('🎮 Client:', socket.id);
  socket.emit('robot-data', { ...latestRobotData, joystick: latestJoystick });

  socket.on('toggle-direction', (id) => {
    thrusterDirs[id] *= -1;
    io.emit('direction-update', { id, dir: thrusterDirs[id] });
  });

  socket.on('joystick-input', (data) => {
    latestJoystick = data;
    const pwms = mixThrusters(data);
    latestRobotData.thrusters = pwms;
    sendPwmsToTeensy(pwms);
    io.emit('thruster-pwm', pwms);
    io.emit('robot-data', { ...latestRobotData, joystick: latestJoystick });
  });
});

teensyUdp.bind(0);
server.listen(3000, () => {
  console.log('🌐 http://localhost:3000');
  console.log('🎯 Ports:', PIXHAWK_PORTS.join(','));
  console.log('📡 Pixhawk:', PIXHAWK_IP);
  console.log('🔥 GUI + Pixhawk READY!');
});
