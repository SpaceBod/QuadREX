import React from 'react';
import './gamepad.css';

const ToggleSwitch = ({ isOn, handleToggle }) => {
  return (
    <div className="toggle-switch">
      <input
        checked={isOn}
        onChange={handleToggle}
        className="toggle-switch-checkbox"
        id={`toggle-switch`}
        type="checkbox"
      />
      <label className="toggle-switch-label" htmlFor={`toggle-switch`}>
        <span className="toggle-switch-inner" />
        <span className="toggle-switch-switch" />
      </label>
      <span className="toggle-switch-text">{isOn ? 'Manual Mode' : 'Autonomous Mode'}</span>
    </div>
  );
};

export default ToggleSwitch;
