import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Toggle } from '../ui/Toggle';

export const PeriodicSearchPanel = ({
  dynamicSystem,
  periodicSearchSettings,
  updatePeriodicSearchSettings,
  periodicState,
  disabled
}) => {
  const supportsBoundarySearchSettings = dynamicSystem === 'henon' || dynamicSystem === 'custom';

  if (!supportsBoundarySearchSettings) {
    return null;
  }

  const updateGridSize = (e) => {
    updatePeriodicSearchSettings?.({ gridSize: parseInt(e.target.value, 10) });
  };

  const updateThetaGridSize = (e) => {
    updatePeriodicSearchSettings?.({ thetaGridSize: parseInt(e.target.value, 10) });
  };

  const updateResidualThreshold = (e) => {
    updatePeriodicSearchSettings?.({ residualThreshold: Number(e.target.value) });
  };

  const updateUseContinuation = (value) => {
    updatePeriodicSearchSettings?.({ useContinuation: value });
  };

  const showContinuationControls = dynamicSystem === 'henon';
  const computeMethod = periodicState?.computeMethod;
  const methodLabel = computeMethod === 'continuation'
    ? 'Last run used continuation'
    : computeMethod === 'grid'
      ? 'Last run used grid search'
      : 'Next run will use grid search';

  return (
    <Collapsible title="Periodic search" defaultOpen={true}>
      <div className="periodic-search-grid">
        <div className="start-field">
          <label htmlFor="periodic-grid-size">Grid size</label>
          <input
            id="periodic-grid-size"
            type="number"
            min="2"
            max="256"
            step="1"
            value={periodicSearchSettings?.gridSize ?? 10}
            onChange={updateGridSize}
            disabled={disabled}
          />
        </div>
        <div className="start-field">
          <label htmlFor="periodic-theta-grid-size">Theta grid</label>
          <input
            id="periodic-theta-grid-size"
            type="number"
            min="2"
            max="256"
            step="1"
            value={periodicSearchSettings?.thetaGridSize ?? 10}
            onChange={updateThetaGridSize}
            disabled={disabled}
          />
        </div>
        <div className="start-field periodic-search-threshold">
          <label htmlFor="periodic-residual-threshold">Residual threshold</label>
          <input
            id="periodic-residual-threshold"
            type="number"
            min="1e-14"
            max="1e-2"
            step="any"
            value={periodicSearchSettings?.residualThreshold ?? 1e-10}
            onChange={updateResidualThreshold}
            disabled={disabled}
          />
        </div>
      </div>
      {showContinuationControls && (
        <>
          <Toggle
            label="Use continuation"
            checked={Boolean(periodicSearchSettings?.useContinuation)}
            onChange={updateUseContinuation}
            disabled={disabled}
          />
          <div className="periodic-search-hint">
            {methodLabel}. Enable continuation to track existing Hénon orbits after parameter changes; leave it off to rediscover orbits by grid search.
          </div>
        </>
      )}
    </Collapsible>
  );
};
