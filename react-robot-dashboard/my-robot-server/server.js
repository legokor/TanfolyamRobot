const express = require('express');
const cors = require('cors');
const http = require('http');
const { Server } = require("socket.io");
const axios = require('axios');

const app = express();

// Enable CORS for all routes and all origins
app.use(cors({ origin: '*' }));

// For parsing application/json
app.use(express.json());

// Colors and their corresponding ports
const colors = {
  red: 14200,
  green: 14201,
  gray: 14202,
  pink: 14203,
  black: 14204,
};

// Map to store ESP IP addresses by color
const espIPs = {
  red: null,
  green: null,
  gray: null,
  pink: null,
  black: null,
};

// Create servers for each color
const servers = {};
const ioInstances = {};

Object.entries(colors).forEach(([color, port]) => {
  servers[color] = http.createServer(app);
  ioInstances[color] = new Server(servers[color], {
    cors: {
      origin: "*", // Allow all origins
      methods: ["GET", "POST"], // Allow only GET and POST requests
    },
  });

  ioInstances[color].on('connection', (socket) => {
    console.log(`${color} client connected`);

    // Handle message event from client
    socket.on('sendMessage', async (message) => {
      console.log(`${color} message received from client: `, message);

      const targetUrl = `http://${espIPs[color]}:80/receiveData`;
      try {
        // Construct JSON payload from the message
        const payload = { message: message };

        // Send POST request to the ESP with the specified URL with the payload
        const response = await axios.post(targetUrl, payload);
        console.log(`POST request sent to ${targetUrl} target URL:`, response.data);
      } catch (error) {
        console.error(`Error sending POST request to ${targetUrl} ESP:`, error);
      }
    });

    socket.on('disconnect', () => {
      console.log(`${color} client disconnected`);
    });
  });

  servers[color].listen(port, () => {
    console.log(`${color} server running on port ${port}`);
  });
});

// HTTP POST endpoint for receiving data
app.post('/publishData', (req, res) => {
  const data = req.body;
  const color = data.color; // Assuming the JSON contains a "color" field
  const espIp = req.socket.remoteAddress.split(":").pop(); // Get ESP IP address from request object

  if (colors[color]) {
    espIPs[color] = espIp;
    console.log(`Received data from ${color} ESP with IP: ${espIp}`);

    // Forward data to frontend on the corresponding port
    ioInstances[color].emit('Data received', data);
    console.log(`Forwarded data to ${color} frontend`);
  } else {
    console.log(`Unknown color: ${color}`);
  }

  res.status(200).send('Data received and broadcasted');
});

// Example REST endpoint
app.get('/status', (req, res) => {
  res.send('Server is up and running');
});

// Start a main server (if needed) on a separate port
const mainServer = http.createServer(app);
mainServer.listen(5000, () => {
  console.log(`Main server running on port 5000`);
});
