import React from 'react';
import './IMUSensorDisplay.css'; // Make sure to create this CSS file

const IMUSensorDisplay = ({ imuData }) => {
  if (!imuData) {
    return <div>Loading...</div>; // or some placeholder content
  }

  const { acc, gyro, mag, angle, temp } = imuData;

  return (
    <div className="imu-sensor-display">
      <h2>IMU Sensor</h2>
      <div className="imu-matrix">
        <div className="imu-column">
          <div className="imu-header">Acc <br />(g)</div>
          <div className="imu-value">{acc.x.toFixed(3)}</div>
          <div className="imu-value">{acc.y.toFixed(3)}</div>
          <div className="imu-value">{acc.z.toFixed(3)}</div>
        </div>
        <div className="imu-column">
          <div className="imu-header">Gyro (°/s)</div>
          <div className="imu-value">{gyro.x.toFixed(1)}</div>
          <div className="imu-value">{gyro.y.toFixed(1)}</div>
          <div className="imu-value">{gyro.z.toFixed(1)}</div>
        </div>
        <div className="imu-column">
          <div className="imu-header">Mag (uT)</div>
          <div className="imu-value">{mag.x.toFixed(2)}</div>
          <div className="imu-value">{mag.y.toFixed(2)}</div>
          <div className="imu-value">{mag.z.toFixed(2)}</div>
        </div>
        <div className="imu-column">
          <div className="imu-header">Temperature (°C)</div>
          <div className="imu-value">{temp.toFixed(1)}</div>
          <div className="imu-header">Angle (°)</div>
          <div className="imu-value">{angle.pitch.toFixed(1)}</div>
          <div className="imu-value">{angle.roll.toFixed(1)}</div>
        </div>
      </div>
    </div>
  );
};

export default IMUSensorDisplay;
