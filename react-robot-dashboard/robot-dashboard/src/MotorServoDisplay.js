import React from 'react';
import './MotorServoDisplay.css'; // Make sure to include this CSS file

const MotorServoDisplay = ({ motora, motorb, cpsa, cpsb, cnta, cntb, servo }) => {
  return (
    <div className="motor-servo-display">
      <h2>Motor and Servo Data</h2>
      <div className="data-matrix">
        <div className="data-column">
          <div className="data-header">Motor Setpoint</div>
          <div className="data-value">Motor A: {motora}</div>
          <div className="data-value">Motor B: {motorb}</div>
        </div>
        <div className="data-column">
          <div className="data-header">Encoder Speed</div>
          <div className="data-value">Encoder A: {cpsa} CP/s</div>
          <div className="data-value">Encoder B: {cpsb} CP/s</div>
        </div>
        <div className="data-column">
          <div className="data-header">Encoder Position</div>
          <div className="data-value">Encoder A: {cnta}</div>
          <div className="data-value">Encoder B: {cntb}</div>
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
