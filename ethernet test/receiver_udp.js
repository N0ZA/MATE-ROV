const dgram = require('dgram');

const LISTEN_PORT = 41234;
const LISTEN_ADDR = '0.0.0.0';

const server = dgram.createSocket('udp4');

let bytesThisSecond = 0;
let packetsThisSecond = 0;
let lastReportMs = Date.now();

server.on('error', (err) => {
  console.error('UDP server error:', err);
  server.close();
});

server.on('message', (msg, rinfo) => {
  bytesThisSecond += msg.length;
  packetsThisSecond += 1;

  const text = msg.toString('utf8').trimEnd();
  console.log(`[${rinfo.address}:${rinfo.port}] ${text}`);

  const now = Date.now();
  if (now - lastReportMs >= 1000) {
    const elapsedSec = (now - lastReportMs) / 1000;
    const bps = (bytesThisSecond * 8) / elapsedSec; // bits per second
    const kbps = bps / 1000;

    console.log(
      `--- bitrate: ${bps.toFixed(0)} bps (${kbps.toFixed(2)} kbps), ` +
      `packets: ${Math.round(packetsThisSecond / elapsedSec)}/s ---`
    );

    bytesThisSecond = 0;
    packetsThisSecond = 0;
    lastReportMs = now;
  }
});

server.on('listening', () => {
  const addr = server.address();
  console.log(`UDP receiver listening on ${addr.address}:${addr.port}`);
});

server.bind(LISTEN_PORT, LISTEN_ADDR);