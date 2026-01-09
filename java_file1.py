const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// --- 1. SET YOUR PORT ---
// Run 'ls /dev/tty.usbmodem*' in terminal and paste the result here
const PROTOCOL_PORT = '/dev/tty.usbmodem179820401'; 
const BAUD_RATE = 115200;

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/index.html');
});

// --- 2. OPEN SERIAL PORT ---
const port = new SerialPort({ path: PROTOCOL_PORT, baudRate: BAUD_RATE }, (err) => {
    if (err) console.log('CRITICAL ERROR: ', err.message);
});

// Parser ensures we read full lines ending in Newline
const parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }));

parser.on('data', (line) => {
    try {
        // Line format: H:104.3,R:21.7,P:4.1,VX:-1.867,VY:-1.030,VZ:0.396
        const parts = line.split(',');
        const raw = {};
        
        parts.forEach(part => {
            const [key, val] = part.split(':');
            raw[key] = parseFloat(val);
        });

        // --- 3. SEND TO FRONTEND ---
        const payload = {
            yaw: raw.H || 0,
            roll: raw.R || 0,
            pitch: raw.P || 0,
            depth: raw.VZ || 0, // Using VZ as placeholder for depth
            joy: { x: raw.VX || 0, y: raw.VY || 0, z: raw.VZ || 0 },
            thrusters: [127, 127, 127, 127, 127, 127, 127], // Placeholder
            ping: 1
        };

        io.emit('robot-data', payload);
        
        // This confirms in your terminal that the backend is working
        console.log(`TEENSY DATA -> Yaw: ${payload.yaw} Pitch: ${payload.pitch} Roll: ${payload.roll}`);

    } catch (e) {
        console.log("Parsing error:", e.message);
    }
});

server.listen(3000, () => {
    console.log('Backend active at http://localhost:3000');
});