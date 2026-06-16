const { workerData, parentPort } = require('worker_threads');
const dgram = require('dgram');

const { jsBuffer, ctrlBuffer, teensyIp, teensyPort, initialArm } = workerData;

// jsState  Float32[7]: [x, y, yaw, vertical, roll, gain, verticalGain]
// ctrlState Int32[13]: [armed, dir1..6, sel1..6]
const jsState   = new Float32Array(jsBuffer);
const ctrlState = new Int32Array(ctrlBuffer);

let armData = initialArm;
let relayData = [0, 0, 0, 0];  // R1-R4, updated independently of arm

parentPort.on('message', msg => {
  if (msg.type === 'arm')   armData   = msg.data;
  if (msg.type === 'relay') relayData = msg.relays;
});

const sock = dgram.createSocket('udp4');
sock.bind(0);

function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

// =====================================================
// WRIST DIFFERENTIAL (worm-gear coupling)
// rotate + gripper sit on a shared worm-gear differential.
//   - Rotating the wrist back-drives the gripper jaws, so to ROTATE while
//     holding grip the gripper motor must spin complementarily => BOTH motors move.
//   - Pure OPEN/CLOSE spins ONLY the gripper motor (rotate motor stays at 1500).
// These are velocity commands (1500 = stop), so we mix in delta-from-1500 space:
//     rotateMotor  = rotateIntent
//     gripperMotor = gripperIntent + COUPLE * rotateIntent
// Tune COUPLE per arm:
//   magnitude = grip-shaft turns per wrist turn (your gear ratio, ~1.0 for 1:1)
//   sign      = flip if rotating makes the jaws creep the wrong way
//   0.0       = disable coupling for that arm (treat motors as independent)
// =====================================================
const ARM1_WRIST_COUPLE = 1.0;
const ARM2_WRIST_COUPLE = -1.0;

function mixWrist(rotatePwm, gripperPwm, couple) {
  const rotDelta  = rotatePwm  - 1500;
  const gripDelta = gripperPwm - 1500;
  const rotateMotor  = 1500 + rotDelta;
  const gripperMotor = 1500 + gripDelta + couple * rotDelta;
  return {
    rotate:  Math.round(clamp(rotateMotor,  1000, 2000)),
    gripper: Math.round(clamp(gripperMotor, 1000, 2000)),
  };
}

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

  // Differential mix: convert rotate/gripper INTENT -> the two coupled motor PWMs.
  // (slew + shoulder are independent single motors, passed through unchanged.)
  const w1 = mixWrist(armData.arm1_rotate, armData.arm1_gripper, ARM1_WRIST_COUPLE);
  const w2 = mixWrist(armData.arm2_rotate, armData.arm2_gripper, ARM2_WRIST_COUPLE);

  // Packet: [ESC1-6, MD1(arm1_slew), MD2(arm1_shoulder), MD3(arm1_rotate), MD4(arm1_gripper),
  //          MD5(arm2_slew), MD6(arm2_shoulder), MD7(arm2_rotate), MD8(arm2_gripper),
  //          R1, R2, R3, R4]
  const packet = [
    ...thrusters,
    armData.arm1_slew,
    armData.arm1_shoulder,
    w1.rotate,              // MD3: rotate motor (rotate intent)
    w1.gripper,             // MD4: gripper motor (grip intent + rotate compensation)
    armData.arm2_slew,
    armData.arm2_shoulder,
    w2.rotate,              // MD7: rotate motor (rotate intent)
    w2.gripper,             // MD8: gripper motor (grip intent + rotate compensation)
    relayData[0],           // R1 — light 1
    relayData[1],           // R2 — light 2
    relayData[2],           // R3 — light 3
    relayData[3],           // R4 — light 4
  ];
  
  sock.send(
    Buffer.from(JSON.stringify({ type: 'all', pwms: packet, ts: Date.now() })),
    teensyPort,
    teensyIp
  );
}

setInterval(mixAndSend, 10);