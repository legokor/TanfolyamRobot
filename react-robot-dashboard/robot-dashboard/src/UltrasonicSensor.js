import React from 'react';
import './UltrasonicSensor.css'; // Ensure this CSS file is created

const UltrasonicSensor = ({ distance }) => {
  return (
    <div className="ultrasonic-sensor">
      <h2>Ultrasonic Sensor</h2>
      <p>Distance: {distance} cm</p>
    </div>
  );
};

export default UltrasonicSensor;
