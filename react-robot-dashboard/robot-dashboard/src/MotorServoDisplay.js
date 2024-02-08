import React from 'react';
import './MotorServoDisplay.css'; // Make sure to include this CSS file

const MotorServoDisplay = ({ motora, motorb, wheelaspeed, wheelbspeed, servo }) => {
  return (
    <div className="motor-servo-display">
      <h2>Motor and Servo Data</h2>
      <div className="data-matrix">
        <div className="data-column">
          <div className="data-header">Motors Power</div>
          <div className="data-value">Motor A: {motora}</div>
          <div className="data-value">Motor B: {motorb}</div>
        </div>
        <div className="data-column">
          <div className="data-header">Wheels Speed</div>
          <div className="data-value">Wheel A: {wheelaspeed} RPM</div>
          <div className="data-value">Wheel B: {wheelbspeed} RPM</div>
        </div>
        <div className="data-column">
          <div className="data-header">Servo Position</div>
          <div className="data-value">{servo}°</div>
        </div>
      </div>
    </div>
  );
};

export default MotorServoDisplay;
