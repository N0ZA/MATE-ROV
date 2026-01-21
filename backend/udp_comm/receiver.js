const dgram = require('dgram');
const server = dgram.createSocket('udp4');

const PORT = 41234;
const SCALE_FACTOR = 10000;

server.on('error', (err) => {
    console.log(`Server error:\n${err.stack}`);
    server.close();
});

server.on('message', (msg, rinfo) => {
    console.log(`\n--- Packet Received from ${rinfo.address}:${rinfo.port} ---`);
    console.log(`Buffer Hex: ${msg.toString('hex')}`);
    console.log(`Length: ${msg.length} bytes (${msg.length * 8} bits)`);

    if (msg.length !== 6) {
        console.log('WARNING: Unexpected packet length.');
        return;
    }

    // DECODE
    const raw1 = msg.readUInt16BE(0);
    const raw2 = msg.readUInt16BE(2);
    const raw3 = msg.readUInt16BE(4);

    const val1 = raw1 / SCALE_FACTOR;
    const val2 = raw2 / SCALE_FACTOR;
    const val3 = raw3 / SCALE_FACTOR;

    console.log(`Decoded Values:`);
    console.log(`  Value 1: ${val1}`);
    console.log(`  Value 2: ${val2}`);
    console.log(`  Value 3: ${val3}`);
});

server.on('listening', () => {
    const address = server.address();
    console.log(`Receiver listening on ${address.address}:${address.port}`);
});

server.bind(PORT);
