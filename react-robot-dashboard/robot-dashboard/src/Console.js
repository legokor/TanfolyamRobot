import React, { useState, useEffect } from 'react';

const Console = ({ data }) => {
  const [messages, setMessages] = useState([]);

  useEffect(() => {
    if (data && data.consolemessage && data.consolemessage !== "") {
      setMessages(prevMessages => [...prevMessages, data.consolemessage]);
    }
  }, [data]);

  return (
    <div style={{
      height: '300px',
      width: '100%',
      overflowY: 'scroll',
      backgroundColor: '#0e0e0e',
      color: '#FFFFFF', // Corrected color value
      fontFamily: 'monospace',
      fontSize: '50px',
      padding: '10px',
      boxSizing: 'border-box',
      textAlign: 'left', // Ensure text aligns to the left
      /* Custom Scrollbar styles */
      scrollbarWidth: 'thin',
      scrollbarColor: '#ffcc00 #0e0e0e',
      /* Webkit Scrollbar styles */
      WebkitOverflowScrolling: 'touch',
      '&::-webkit-scrollbar': {
        width: '12px',
      },
      '&::-webkit-scrollbar-track': {
        background: '#0e0e0e',
      },
      '&::-webkit-scrollbar-thumb': {
        background: '#ffcc00',
        borderRadius: '10px',
      }
    }}>
      {messages.map((message, index) => (
        <div key={index}>{message}</div>
      ))}
    </div>
  );
};

export default Console;
