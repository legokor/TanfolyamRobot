import React from 'react';
import './IMUSensorDisplay.css'; // Make sure to create this CSS file

const IMUSensorDisplay = ({ imuData }) => {
  if (!imuData) {
    return <div>Loading...</div>; // or some placeholder content
  }

  const { acc, gyro, mag, temp } = imuData;

  return (
    <div className="imu-sensor-display">
      <h2>IMU Sensor</h2>
      <div className="imu-matrix">
        <div className="imu-column">
          <div className="imu-header">Acc <br />(m/s²)</div>
          <div className="imu-value">{acc.x}</div>
          <div className="imu-value">{acc.y}</div>
          <div className="imu-value">{acc.z}</div>
        </div>
        <div className="imu-column">
          <div className="imu-header">Gyro (°/s)</div>
          <div className="imu-value">{gyro.x}</div>
          <div className="imu-value">{gyro.y}</div>
          <div className="imu-value">{gyro.z}</div>
        </div>
        <div className="imu-column">
          <div className="imu-header">Mag (uT)</div>
          <div className="imu-value">{mag.x}</div>
          <div className="imu-value">{mag.y}</div>
          <div className="imu-value">{mag.z}</div>
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
