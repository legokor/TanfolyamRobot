import React, { useState, useEffect } from 'react';
import openSocket from 'socket.io-client';
import UltrasonicSensor from './UltrasonicSensor';
import HSVColorDisplay from './HSVColorDisplay';
import IMUSensorDisplay from './IMUSensorDisplay';
import MotorServoDisplay from './MotorServoDisplay';
import Terminal from './Terminal';
import Console from './Console';

const RobotPage = () => {
  const [data, setData] = useState(null);
  const [socket, setSocket] = useState(null);
  const [selectedColor, setSelectedColor] = useState(null);

  useEffect(() => {
    const port = selectedColor ? 14200 + ['red', 'green', 'gray', 'pink', 'black'].indexOf(selectedColor) : 5000;
    const windowUrl = window.location.hostname;
    const newSocket = openSocket(windowUrl + ":" + port);
    setSocket(newSocket);

    newSocket.on('Data received', (robotData) => {
      setData(robotData);
    });

    return () => newSocket.close();
  }, [selectedColor]);

  const sendMessage = (message) => {
    if (socket) {
      socket.emit('sendMessage', message); // Replace 'sendMessage' with your server's listening event
    }
  };

  const handleColorChange = (color) => {
    setSelectedColor(color);
    setData(null);
  };

  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'center', marginBottom: '20px' }}>
        <ColorSelector selectedColor={selectedColor} onColorChange={handleColorChange} />
      </div>
      <div style={{ display: 'flex', justifyContent: 'space-around' }}>
        {/* {data && <IMUSensorDisplay imuData={data.imu} />} */}
        <div style={sensorWrapperStyle}>
          {data && <UltrasonicSensor distance={data.usonic} />}
          {data && <HSVColorDisplay hsv={data.hsv} />}
        </div>
        <div style={sensorWrapperStyle}>
          {data && <IMUSensorDisplay imuData={data.imu} />}
        </div>
      </div>
      <div style={{ display: 'flex', justifyContent: 'space-around' }}>
        {data && (
          <MotorServoDisplay
            motora={data.motora}
            motorb={data.motorb}
            cpsa={data.cpsa}
            cpsb={data.cpsb}
            cnta={data.cnta}
            cntb={data.cntb}
            servo={data.servo}
          />
        )}
      </div>
      <div style={{ display: 'flex', justifyContent: 'space-around' }}>
        <Console data={data} />
      </div>
      <div style={{ justifyContent: 'space-around', bottom: 0, width: '100%' }}>
        <Terminal onSendMessage={sendMessage} />
      </div>
    </div>
  );
};

const ColorSelector = ({ selectedColor, onColorChange }) => {
  const colors = ['red', 'green', 'gray', 'pink', 'black'];

  return (
    <div>
      {colors.map((color) => (
        <button
          key={color}
          style={{
            backgroundColor: color,
            width: '50px',
            height: '50px',
            borderRadius: '50%',
            margin: '0 5px',
            border: selectedColor === color ? '2px solid white' : 'none',
          }}
          onClick={() => onColorChange(color)}
        />
      ))}
    </div>
  );
};

const sensorWrapperStyle = {
  backgroundColor: '#333', // Adjust the color to match your theme
  borderRadius: '10px',
  overflow: 'hidden', // This ensures the child components do not overflow the rounded corners
  marginBottom: '20px', // Space between this component and others, adjust as needed
};

export default RobotPage;
