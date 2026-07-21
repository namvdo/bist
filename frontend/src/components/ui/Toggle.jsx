import React from 'react';

export const Toggle = ({ label, checked, onChange, colorLine, disabled }) => {
  return (
    <label className="t-row">
      <div className="t-label">
        {colorLine && <div className="t-swatch-line" style={{ background: colorLine }}></div>}
        {label}
      </div>
      <div className={`t-switch ${checked ? 'on' : ''} ${disabled ? 'disabled' : ''}`}>
        <input
          type="checkbox"
          checked={checked}
          disabled={disabled}
          onChange={event => onChange(event.target.checked)}
          aria-label={label}
        />
        <div className="t-track"></div>
        <div className="t-thumb"></div>
      </div>
    </label>
  );
};
