/**
 * IRIS Base Station WebSocket SERVER
 * Menerima koneksi dari Python Client (Robot) dan meneruskannya ke Frontend (Browser).
 */

const WebSocket = require('ws');
const PORT = 8080;


const wss = new WebSocket.Server({ port: PORT, host: '0.0.0.0' });


const clients = new Set();

console.log(`WebSocket Server running on ws://0.0.0.0:${PORT}`);

wss.on('connection', function connection(ws, req) {
    const ip = req.socket.remoteAddress;
    console.log(`[CONNECTION] New client connected from IP: ${ip}`);

    clients.add(ws);

   
    ws.on('message', function incoming(message) {
       
        clients.forEach(function (client) {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message.toString());
            }
        });
    });

    ws.on('close', function close() {
        console.log(`[DISCONNECT] Client from IP ${ip} disconnected.`);
        clients.delete(ws); 
    });

    ws.send(JSON.stringify({type: 'log', message: 'Connected to WebSocket Server'}));
});

wss.on('error', (error) => {
    console.error(`[SERVER ERROR] ${error.message}`);
});