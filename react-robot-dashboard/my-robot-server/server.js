const express = require('express');
const cors = require('cors');
const http = require('http');
const { Server } = require("socket.io");
const axios = require('axios');

const app = express();
const port = 5000;
var targetUrl = 'http://127.0.0.1:80/receiveData';

// Enable CORS for all routes and all origins
app.use(cors({ origin: '*' }));

// For parsing application/json
app.use(express.json());

// Create server using http and attach socket.io to it
const server = http.createServer(app);
const io = new Server(server, {
  cors: {
    origin: "*", // Allow all origins
    methods: ["GET", "POST"] // Allow only GET and POST requests
  }
});

io.on('connection', (socket) => {
  console.log('a user connected');

  // Handle message event from client
  socket.on('sendMessage', async (message) => {
    console.log('Message received:', message);

    try {
      // Construct JSON payload from the message
      const payload = { message: message };

      // Send POST request to the specified URL with the payload
      const response = await axios.post(targetUrl, payload);
      console.log('POST request sent to target URL:', response.data);
    } catch (error) {
      console.error('Error sending POST request:', error);
    }
  });

  socket.on('disconnect', () => {
    console.log('user disconnected');
  });
});

// HTTP POST endpoint for receiving data
app.post('/publishData', (req, res) => {
  const data = req.body;
  const clientIp = req.socket.remoteAddress.split(":").pop();    // Get client IP address from request object
  targetUrl = `http://${clientIp}:80/receiveData`;
  console.log('Received data from IP:', clientIp);
  console.log('Send data to URL:', targetUrl);
  // Validate data here
  if (validateData(data)) {
    // If valid, emit to all WebSocket clients
    io.emit('Data recieved', data);
    console.log(data)
    res.status(200).send('Data received and broadcasted');
  } else {
    // If invalid, send back an error message
    res.status(400).send('Invalid data format');
  }
});

// Function to validate incoming data
function validateData(data) {
  // Implement actual validation logic according to your requirements
  // This is a basic example
  // if(data && data.id && typeof data.status === 'string') {
  //   return true;
  // }
  // return false;
  return true;
}

// Example REST endpoint
app.get('/status', (req, res) => {
  res.send('Server is up and running');
});

// Start the server
server.listen(port, () => {
  console.log(`Server running on port ${port}`);
});
