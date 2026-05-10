const express = require('express');
const { createServer } = require('node:http');
const { join } = require('node:path');
const { Server } = require('socket.io');
const dgram = require('dgram');
const { MavLinkPacketSplitter, MavLinkPacketParser, common } = require('node-mavlink');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const server = createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

const PIXHAWK_IP = '192.168.2.2';
const PIXHAWK_PORTS = [14550, 14551, 14552, 14540];

const TEENSY_IP = process.env.TEENSY_IP || '192.168.2.177';
const TEENSY_PORT = 5000;

const ARM_PORT = 6000;
const armUdp = dgram.createSocket('udp4');

const ARM_CONTROLLER_IP = process.env.ARM_CONTROLLER_IP || '192.168.2.200';
const ARM_CONTROLLER_PORT = process.env.ARM_CONTROLLER_PORT || 6001;

let armData = {
  arm1_slew: 1500, arm1_shoulder: 1500, arm1_rotate: 1500, arm1_gripper: 1500,
  arm2_slew: 1500, arm2_shoulder: 1500, arm2_rotate: 1500, arm2_gripper: 1500
};

// ---- ARDUINO SERIAL ----
const ARDUINO_PORT = process.env.ARDUINO_PORT || '/dev/tty.usbmodem1301';
const ARDUINO_BAUD = 115200; // Must match Serial.begin() in Arduino

const arduinoSerial = new SerialPort({ path: ARDUINO_PORT, baudRate: ARDUINO_BAUD });
const serialParser = arduinoSerial.pipe(new ReadlineParser({ delimiter: '\n' }));

// EXPECT Arduino lines like: DATA:a0,a1,a2,a3,a4,a5  (angles in degrees)
serialParser.on('data', (line) => {
  line = line.trim();
  if (!line.startsWith('DATA:')) return;

  const payload = line.slice(5); // remove "DATA:"
  const parts = payload.split(',');

  if (parts.length !== 6) {
    // wrong length, ignore
    return;
  }

  // Parse 6 angles
  const angles = parts.map((p) => parseFloat(p));
  if (angles.some((a) => Number.isNaN(a))) {
    return;
  }

  // Map angles [0..270] to PWM [1100..1900] with mid=135deg→1500us
  const toPwm = (deg) => {
    const clamped = Math.max(0, Math.min(270, deg));
    if (Math.abs(clamped - 135) < 10) {
      // ~20deg deadzone around center, tweak if you want
      return 1500;
    }
    if (clamped < 135) {
      // below mid: 0deg→1100, 135deg→1500
      const t = clamped / 135;
      return Math.round(1100 + t * (1500 - 1100));
    } else {
      // above mid: 135deg→1500, 270deg→1900
      const t = (clamped - 135) / 135;
      return Math.round(1500 + t * (1900 - 1500));
    }
  };

  // angles order from Arduino: [j0,j1,j2,j3,j4,j5]
  const pwmLeftShoulder  = toPwm(angles[0]);
  const pwmLeftElbow     = toPwm(angles[1]);
  const pwmLeftWrist     = toPwm(angles[2]);
  const pwmRightShoulder = toPwm(angles[3]);
  const pwmRightElbow    = toPwm(angles[4]);
  const pwmRightWrist    = toPwm(angles[5]);

  // Map to your existing armData structure
  armData = {
    ...armData,
    arm1_slew:     1500,                // no pot → neutral
    arm1_shoulder: pwmLeftShoulder,
    arm1_rotate:   pwmLeftElbow,
    arm1_gripper:  pwmLeftWrist,
    arm2_slew:     1500,
    arm2_shoulder: pwmRightShoulder,
    arm2_rotate:   pwmRightElbow,
    arm2_gripper:  pwmRightWrist,
  };

  // Send to GUI
  io.emit('arm-update', armData);

  console.log(
    '🎮 Pots →',
    angles.map(a => a.toFixed(1)).join(','),
    '→ PWMs',
    Object.values(armData).map(v => Math.round(v)).join(',')
  );
});

arduinoSerial.on('open', () => {
  console.log('✅ Arduino serial connected:', ARDUINO_PORT);
});

arduinoSerial.on('error', (err) => {
  console.error('❌ Arduino Serial error:', err.message);
  console.error('   → Close Arduino IDE Serial Monitor and retry');
});

// Serve GUI
app.get('/', (req, res) => res.sendFile(join(__dirname, 'index.html')));
app.use(express.static('.'));

const pixhawkSockets = {};
const splitter = new MavLinkPacketSplitter();
const parser = new MavLinkPacketParser();
const REGISTRY = { ...common.REGISTRY };
const teensyUdp = dgram.createSocket('udp4');

let latestRobotData = { roll: 0, pitch: 0, yaw: 0, thrusters: [1500,1500,1500,1500,1500,1500], depth: 0, ping: 0 };
let latestJoystick = { x: 0, y: 0, yaw: 0, vertical: 0, pitch: 0, roll: 0, gain: 0.3, stability: false };
let thrusterDirs = {1:1,2:1,3:1,4:1,5:1,6:1};

PIXHAWK_PORTS.forEach(port => {
  const sock = dgram.createSocket('udp4');
  pixhawkSockets[port] = sock;
  sock.on('message', (msg, rinfo) => {
    console.log(`📡 PIXHAWK ${rinfo.address}:${rinfo.port} → port ${port} (${msg.length}B)`);
    splitter.write(msg);
  });
  sock.bind(port, '0.0.0.0', () => console.log(`🎯 Listening UDP ${port}`));
});

armUdp.on('message', (msg) => {
  try {
    const data = JSON.parse(msg.toString());
    armData = { ...armData, ...data };
    io.emit('arm-update', armData);
    console.log('🤖 Arms (UDP hardware):', Object.values(armData).map(v => v.toFixed(0)).join(','));
  } catch (e) {
    console.log('Arm UDP parse error:', e);
  }
});
armUdp.bind(ARM_PORT);

splitter.pipe(parser);
parser.on('data', (packet) => {
  const msgid = packet.header.msgid;
  const clazz = REGISTRY[msgid];
  if (clazz && packet.protocol?.data) {
    try {
      const data = packet.protocol.data(packet.payload, clazz);
      if (data.roll !== undefined) {
        latestRobotData.roll  = Number((data.roll  * 180 / Math.PI).toFixed(1));
        latestRobotData.pitch = Number((data.pitch * 180 / Math.PI).toFixed(1));
        latestRobotData.yaw   = Number((data.yaw   * 180 / Math.PI).toFixed(1));
        io.emit('imu-update', latestRobotData);
        io.emit('robot-data', { ...latestRobotData, joystick: latestJoystick });
        console.log(`✅ R:${latestRobotData.roll}° P:${latestRobotData.pitch}° Y:${latestRobotData.yaw}°`);
      }
    } catch(e) {}
  }
});

function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

function mixThrusters(j) {
  const gain     = clamp(j.gain || 0.7, 0, 1);
  const x        = clamp(j.x || 0, -1, 1) * gain;
  const y        = clamp(j.y || 0, -1, 1) * gain;
  const yaw      = clamp(j.yaw || 0, -1, 1) * gain;
  const vertical = clamp(j.vertical || 0, -1, 1) * gain;
  const roll     = clamp(j.roll || 0, -1, 1) * gain;

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
    if (err) console.error('❌ Teensy:', err.message);
  });
}

function sendArmsToController(data) {
  const msg = Buffer.from(JSON.stringify({ type: 'arms', ...data, ts: Date.now() }));
  armUdp.send(msg, ARM_CONTROLLER_PORT, ARM_CONTROLLER_IP, (err) => {
    if (err) console.error('❌ Arm Controller:', err.message);
    else console.log('🤖 Arms → Controller:', ARM_CONTROLLER_IP + ':' + ARM_CONTROLLER_PORT);
  });
}

io.on('connection', (socket) => {
  console.log('🎮 Client:', socket.id);
  socket.emit('robot-data', { ...latestRobotData, joystick: latestJoystick });
  socket.emit('arm-update', armData); // send current pot state on connect

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

  socket.on('arm-control', (data) => {
    armData = {
      arm1_slew:     data.arm1_slew,
      arm1_shoulder: data.arm1_shoulder,
      arm1_rotate:   data.arm1_rotate,
      arm1_gripper:  data.arm1_gripper,
      arm2_slew:     data.arm2_slew,
      arm2_shoulder: data.arm2_shoulder,
      arm2_rotate:   data.arm2_rotate,
      arm2_gripper:  data.arm2_gripper
    };
    sendArmsToController(armData);
    io.emit('arm-update', armData);
    console.log('🤖 Arms (GUI):', Object.values(armData).map(v => v.toFixed(0)).join(','));
  });
});

teensyUdp.bind(0);
server.listen(3000, () => {
  console.log('🌐 http://localhost:3000');
  console.log('🎯 Ports:', PIXHAWK_PORTS.join(','));
  console.log('📡 Pixhawk:', PIXHAWK_IP);
  console.log('🤖 Arms UDP Listen:', ARM_PORT);
  console.log('🤖 Arms Controller:', ARM_CONTROLLER_IP + ':' + ARM_CONTROLLER_PORT);
  console.log('🎮 Arduino Serial:', ARDUINO_PORT, '@', ARDUINO_BAUD);
  console.log('🔥 Jalpari Mission Control v9 - Arms + GUI Ready!');
});
