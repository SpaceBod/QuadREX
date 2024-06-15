import React from 'react';
import './gamepad.css'; // Import CSS file for styling

const Gamepad = ({ onMove, robotName }) => {
  return (
    <div className="control-container">
      <h6>Controls</h6>
      <div className="o-pad-container">
        <div className="o-pad">
          <button className="up" onClick={() => onMove('up')} aria-label="Move Up">
            <span className="sr-only">Up</span>
          </button>
          <button className="left" onClick={() => onMove('left')} aria-label="Move Left">
            <span className="sr-only">Left</span>
          </button>
          <button className="right" onClick={() => onMove('right')} aria-label="Move Right">
            <span className="sr-only">Right</span>
          </button>
          <button className="down" onClick={() => onMove('down')} aria-label="Move Down">
            <span className="sr-only">Down</span>
          </button>
        </div>
      </div>
      <div className='title'>Live Video Feed</div>
      <div className="video-feed-container">
        <div className="video-feed">
          <span className="watermark">{robotName}</span>
          <span className="offline-watermark">OFFLINE</span>
        </div>
      </div>
    </div>
  );
};

export default Gamepad;
