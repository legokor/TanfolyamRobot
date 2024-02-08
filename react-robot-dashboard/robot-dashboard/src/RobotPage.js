import React, { useState, useEffect } from 'react';
import openSocket from 'socket.io-client';
import UltrasonicSensor from './UltrasonicSensor';
import HSVColorDisplay from './HSVColorDisplay';
import IMUSensorDisplay from './IMUSensorDisplay';
import MotorServoDisplay from './MotorServoDisplay';
import Terminal from './Terminal';

const RobotPage = () => {
  const [data, setData] = useState(null);
  const [socket, setSocket] = useState(null);

  useEffect(() => {
    const newSocket = openSocket('http://localhost:5000');
    setSocket(newSocket);

    newSocket.on('Data recieved', (robotData) => {
      setData(robotData);
    });

    return () => newSocket.close();
  }, [setData]);

  const sendMessage = (message) => {
    if (socket) {
      socket.emit('sendMessage', message); // Replace 'sendMessage' with your server's listening event
    }
  };

  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'space-around' }}>
        {data && <IMUSensorDisplay imuData={data.imu} />}
        <div style={sensorWrapperStyle}>
          {data && <UltrasonicSensor distance={data.usonic} />}
          {data && <HSVColorDisplay hsv={data.hsv} />}
        </div>
      </div>
      <div style={{ display: 'flex', justifyContent: 'space-around' }}>
        
        {data && (
          <MotorServoDisplay
            motora={data.motora}
            motorb={data.motorb}
            wheelaspeed={data.wheelaspeed}
            wheelbspeed={data.wheelbspeed}
            servo={data.servo}
          />
        )}
      </div>
      <div style={{ justifyContent: 'space-around',  bottom: 0, width: '100%' }}>
        <Terminal onSendMessage={sendMessage} />
      </div>
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
