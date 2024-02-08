import React from 'react';
import './App.css';
import RobotPage from './RobotPage'; // Import RobotPage component

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <h1 style={titleStyle}>LEGO Tanfolyam Robot Dashboard</h1>
        <RobotPage />
      </header>
    </div>
  );
}

// Inline style for the title
const titleStyle = {
  fontFamily: "'Montserrat', sans-serif",
  fontSize: '3rem', // Responsive font size
  color: '#ffcc00', // LEGO yellow color
  textTransform: 'uppercase',
  letterSpacing: '0.1em',
  margin: '0.5em 0',
  padding: '0.5em',
  borderRadius: '10px',
  backgroundColor: 'rgba(0, 0, 0, 0.7)',
  border: '2px solid #ffcc00',
};

export default App;
