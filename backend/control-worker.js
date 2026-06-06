const { workerData, parentPort } = require('worker_threads');
const dgram = require('dgram');

const { jsBuffer, ctrlBuffer, teensyIp, teensyPort, initialArm } = workerData;

// jsState  Float32[7]: [x, y, yaw, vertical, roll, gain, verticalGain]
// ctrlState Int32[13]: [armed, dir1..6, sel1..6]
const jsState   = new Float32Array(jsBuffer);
const ctrlState = new Int32Array(ctrlBuffer);

let armData = initialArm;

parentPort.on('message', msg => {
  if (msg.type === 'arm') armData = msg.data;
});

const sock = dgram.createSocket('udp4');
sock.bind(0);

function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

function mixAndSend() {
  const armed = Atomics.load(ctrlState, 0);

  let thrusters;
  if (!armed) {
    thrusters = [1500, 1500, 1500, 1500, 1500, 1500];
  } else {
    const g        = clamp(jsState[5], 0, 1);
    const vg       = clamp(jsState[6], 0, 1);
    const x        = clamp(jsState[0], -1, 1);
    const y        = clamp(jsState[1], -1, 1);
    const yaw      = clamp(jsState[2], -1, 1);
    const vertical = clamp(jsState[3], -1, 1);
    const roll     = clamp(jsState[4], -1, 1);

    let fl = -y + yaw + x;
    let fr = -y - yaw - x;
    let rl = -y + yaw - x;
    let rr = -y - yaw + x;

    const maxH = Math.max(1, Math.abs(fl), Math.abs(fr), Math.abs(rl), Math.abs(rr));
    fl/=maxH; fr/=maxH; rl/=maxH; rr/=maxH;

    let vl = vertical + roll;
    let vr = vertical - roll;
    const maxV = Math.max(1, Math.abs(vl), Math.abs(vr));
    vl/=maxV; vr/=maxV;

    const pwmMinH = 1500 - g  * 300, pwmMaxH = 1500 + g  * 300;
    const pwmMinV = 1500 - vg * 300, pwmMaxV = 1500 + vg * 300;

    const anySelected = [7, 8, 9, 10, 11, 12].some(i => Atomics.load(ctrlState, i));
    const mask = id => (anySelected && !Atomics.load(ctrlState, 6 + id)) ? 0 : 1;
    const dir  = id => Atomics.load(ctrlState, id);

    const toPwmH = (v, id) => Math.round(clamp(1500 + v * dir(id) * 300, pwmMinH, pwmMaxH));
    const toPwmV = (v, id) => Math.round(clamp(1500 + v * dir(id) * 300, pwmMinV, pwmMaxV));

    thrusters = [
      toPwmH(fl * mask(1), 1),
      toPwmH(fr * mask(2), 2),
      toPwmH(rl * mask(3), 3),
      toPwmH(rr * mask(4), 4),
      toPwmV(vl * mask(5), 5),
      toPwmV(vr * mask(6), 6),
    ];
  }

  const getGripperValues = state => {
    if (state === 1) return [0, 1];
    if (state === 2) return [1, 0];
    return [0, 0];
  };

  const packet = [
    ...thrusters,
    armData.arm1_slew,
    armData.arm1_shoulder,
    armData.arm1_rotate,
    ...getGripperValues(armData.arm1_gripper),
    armData.arm2_slew,
    armData.arm2_shoulder,
    armData.arm2_rotate,
    ...getGripperValues(armData.arm2_gripper),
  ];

  sock.send(
    Buffer.from(JSON.stringify({ type: 'all', pwms: packet, ts: Date.now() })),
    teensyPort,
    teensyIp
  );
}

setInterval(mixAndSend, 10);
