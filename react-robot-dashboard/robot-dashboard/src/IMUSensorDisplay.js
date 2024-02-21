import React from 'react';
import './IMUSensorDisplay.css'; // Make sure to create this CSS file

const IMUSensorDisplay = ({ imuData }) => {
  if (!imuData) {
    return <div>Loading...</div>; // or some placeholder content
  }

  const { acc, gyro, temp } = imuData;

  return (
    <div className="imu-sensor-display">
      <h2>IMU Sensor</h2>
      <div className="imu-matrix">
        <div className="imu-column">
          <div className="imu-header">Acc <br />(m/s²)</div>
          <div className="imu-value">{acc.a}</div>
          <div className="imu-value">{acc.b}</div>
          <div className="imu-value">{acc.c}</div>
        </div>
        <div className="imu-column">
          <div className="imu-header">Gyroscope (°/s)</div>
          <div className="imu-value">{gyro.x}</div>
          <div className="imu-value">{gyro.y}</div>
          <div className="imu-value">{gyro.z}</div>
        </div>
        <div className="imu-column">
          <div className="imu-header">Temperature (°C)</div>
          <div className="imu-value">{temp}</div>
        </div>
      </div>
    </div>
  );
};

export default IMUSensorDisplay;
