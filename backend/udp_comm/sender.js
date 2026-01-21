const dgram = require('dgram');
const client = dgram.createSocket('udp4');

// CONFIGURATION
const TARGET_IP = process.argv[2] || '127.0.0.1'; // Default to localhost if not provided
const TARGET_PORT = 41234;

// DATA TO SEND
const val1 = 3.57;
const val2 = 0.04;
const val3 = 0.04;

// ENCODING: Scaled Integers (x10000)
// 3.57 -> 35700
// 0.04 -> 400
// Max value for UInt16 is 65535, so max float is ~6.5535
const SCALE_FACTOR = 10000;

function encode(value) {
    return Math.round(value * SCALE_FACTOR);
}

// Prepare Buffer (6 bytes total)
const buffer = Buffer.alloc(6);
buffer.writeUInt16BE(encode(val1), 0);
buffer.writeUInt16BE(encode(val2), 2);
buffer.writeUInt16BE(encode(val3), 4);

console.log(`Sending packet to ${TARGET_IP}:${TARGET_PORT}`);
console.log(`Values: ${val1}, ${val2}, ${val3}`);
console.log(`Encoded: ${encode(val1)}, ${encode(val2)}, ${encode(val3)}`);
console.log(`Buffer Hex: ${buffer.toString('hex')} (Length: ${buffer.length} bytes / ${buffer.length * 8} bits)`);

client.send(buffer, TARGET_PORT, TARGET_IP, (err) => {
    if (err) {
        console.error('Error sending packet:', err);
    } else {
        console.log('Packet sent successfully.');
    }
    client.close();
});
