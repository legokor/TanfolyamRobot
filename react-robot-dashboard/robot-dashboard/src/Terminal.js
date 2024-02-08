import React, { useState } from 'react';

const Terminal = ({ onSendMessage }) => {
    const [input, setInput] = useState('');
  
    const handleInputChange = (e) => {
      setInput(e.target.value);
    };
  
    const handleKeyDown = (e) => {
      if (e.key === 'Enter' && input.trim()) {
        onSendMessage(input);
        setInput('');
      }
    };
  
    return (
      <div className="terminal" style={terminalStyle}>
        <input
          type="text"
          value={input}
          onChange={handleInputChange}
          onKeyDown={handleKeyDown}
          style={inputStyle}
          placeholder="Type a command..."
        />
      </div>
    );
};

// Define your styles here to keep JSX clean
const terminalStyle = {
  backgroundColor: '#333',
  color: '#fff',
  padding: '20px',
  marginTop: '30px', // Adds space between this and other components
  borderRadius: '10px',
  boxShadow: '0 4px 8px rgba(0, 0, 0, 0.3)' // Adds a subtle shadow for depth
};

const inputStyle = {
  width: 'calc(100% - 10px)', // Accounts for padding
  padding: '10px',
  color: '#fff',
  backgroundColor: '#222',
  border: '1px solid #555',
  borderRadius: '5px',
  fontSize: '1.1em', // Makes text larger
  boxSizing: 'border-box' // Ensures padding doesn't add to width
};

export default Terminal;
