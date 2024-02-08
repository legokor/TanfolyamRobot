import React from 'react';
import './HSVColorDisplay.css'; // Make sure to create this CSS file

const HSVColorDisplay = ({ hsv }) => {
  const hsvToCSS = (h, s, v) => {
    const f = (n, k = (n + h / 60) % 6) => v - v * s * Math.max(Math.min(k, 4 - k, 1), 0);
    const rgb = [f(5), f(3), f(1)].map(x => Math.round(x * 255));
    return `rgb(${rgb.join(',')})`;
  };

  const colorStyle = {
    backgroundColor: hsvToCSS(hsv.h, hsv.s / 100, hsv.v / 100),
  };

  return (
    <div className="hsv-color-display">
      <h2>HSV</h2>
      <div className="hsv-content">
        <div>
          <p>H: {hsv.h}</p>
          <p>S: {hsv.s}</p>
          <p>V: {hsv.v}</p>
        </div>
        <div className="color-preview" style={colorStyle}></div>
      </div>
    </div>
  );
};

export default HSVColorDisplay;
