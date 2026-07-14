import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Toggle } from '../ui/Toggle';
import { Slider } from '../ui/Slider';
import { HITTING_CONTOUR_LIMITS, getPresentLevels } from '../../utils/hittingContours';

const TOLERANCE_PRESETS = [1e-10, 1e-6, 1e-4, 1e-2];

const formatTolerance = (value) => Number(value).toExponential(0);

const isSameTolerance = (a, b) => (
  Number.isFinite(Number(a))
  && Number.isFinite(Number(b))
  && Math.abs(Math.log10(Number(a)) - Math.log10(Number(b))) < 1e-9
);

export const HittingContoursPanel = ({
  dynamicSystem,
  hittingContourState,
  setHittingContourState
}) => {
  if (dynamicSystem !== 'henon') {
    return null;
  }

  const levels = getPresentLevels(hittingContourState.result);
  const update = (patch) => {
    setHittingContourState(prev => ({ ...prev, ...patch }));
  };
  const updateNumber = (key, value) => {
    const parsed = Number(value);
    if (Number.isFinite(parsed)) {
      update({ [key]: parsed });
    }
  };
  const selectedLevels = Array.isArray(hittingContourState.selectedLevels)
    ? hittingContourState.selectedLevels
    : [];
  const toggleLevel = (level) => {
    const nextLevels = selectedLevels.includes(level)
      ? selectedLevels.filter(item => item !== level)
      : [...selectedLevels, level].sort((a, b) => a - b);
    update({
      selectedLevels: nextLevels,
      selectedLevel: nextLevels.length > 0 ? nextLevels[0] : 'all'
    });
  };

  return (
    <Collapsible title="Hitting levels" defaultOpen={false}>
      <Toggle
        label="Level overlay"
        checked={hittingContourState.showOverlay}
        onChange={v => update({ showOverlay: v })}
        disabled={hittingContourState.isComputing}
      />

      {hittingContourState.showOverlay && (
        <>
          <div className="small-label">Layout</div>
          <div className="level-filter">
            <button
              className={`level-btn ${hittingContourState.layoutMode !== 'spatial' ? 'on' : ''}`}
              onClick={() => update({ layoutMode: 'topographic' })}
            >
              topo
            </button>
            <button
              className={`level-btn ${hittingContourState.layoutMode === 'spatial' ? 'on' : ''}`}
              onClick={() => update({ layoutMode: 'spatial' })}
            >
              spatial
            </button>
          </div>
          <Toggle
            label="Circle lines"
            checked={hittingContourState.showLevelRings !== false}
            onChange={v => update({ showLevelRings: v })}
            disabled={hittingContourState.isComputing}
          />
          <Slider
            label="Grid"
            hint="samples"
            min={10}
            max={180}
            step={5}
            value={hittingContourState.sampleGridSize}
            onChange={v => update({ sampleGridSize: v })}
            disabled={hittingContourState.isComputing}
          />
          <Slider
            label="Normals"
            hint="directions"
            min={1}
            max={32}
            step={1}
            value={hittingContourState.thetaGridSize}
            onChange={v => update({ thetaGridSize: v })}
            disabled={hittingContourState.isComputing}
          />
          <Slider
            label="Max period"
            min={1}
            max={10}
            step={1}
            value={hittingContourState.maxPeriod}
            onChange={v => update({ maxPeriod: v })}
            disabled={hittingContourState.isComputing}
          />
          <Slider
            label="Max level"
            min={1}
            max={10}
            step={1}
            value={hittingContourState.maxLevel}
            onChange={v => update({ maxLevel: v })}
            disabled={hittingContourState.isComputing}
          />
          <Slider
            label="Ulam support"
            hint="mass kept"
            min={0.8}
            max={0.999}
            step={0.001}
            value={hittingContourState.supportMass}
            onChange={v => update({ supportMass: v })}
            disabled={hittingContourState.isComputing}
          />
          <div className="periodic-search-grid hitting-threshold-grid">
            <div className="start-field">
              <label htmlFor="hitting-hit-tolerance">Hit tolerance</label>
              <input
                id="hitting-hit-tolerance"
                type="number"
                min={HITTING_CONTOUR_LIMITS.hitToleranceMin}
                max={HITTING_CONTOUR_LIMITS.hitToleranceMax}
                step="any"
                value={hittingContourState.hitTolerance}
                onChange={e => updateNumber('hitTolerance', e.target.value)}
                disabled={hittingContourState.isComputing}
              />
            </div>
            <div className="start-field">
              <label htmlFor="hitting-orbit-residual">Orbit residual</label>
              <input
                id="hitting-orbit-residual"
                type="number"
                min={HITTING_CONTOUR_LIMITS.residualThresholdMin}
                max={HITTING_CONTOUR_LIMITS.residualThresholdMax}
                step="any"
                value={hittingContourState.residualThreshold}
                onChange={e => updateNumber('residualThreshold', e.target.value)}
                disabled={hittingContourState.isComputing}
              />
            </div>
          </div>
          <div className="level-filter tolerance-presets">
            {TOLERANCE_PRESETS.map(value => (
              <button
                key={value}
                className={`level-btn ${isSameTolerance(hittingContourState.hitTolerance, value) ? 'on' : ''}`}
                onClick={() => update({ hitTolerance: value })}
                disabled={hittingContourState.isComputing}
              >
                {formatTolerance(value)}
              </button>
            ))}
          </div>
          {hittingContourState.isComputing ? (
            <div className="contour-status computing">
              <span className="spinner"></span>
              Computing levels
            </div>
          ) : hittingContourState.error ? (
            <div className="contour-status error">{hittingContourState.error}</div>
          ) : hittingContourState.result ? (
            <div className="contour-status ready">
              {hittingContourState.result.summary?.hitCellCount || 0} cells, {hittingContourState.result.summary?.targetCount || 0} targets, {hittingContourState.result.summary?.activeBoxes || 0}/{hittingContourState.result.summary?.totalBoxes || 0} boxes
            </div>
          ) : (
            <div className="contour-status idle">Waiting</div>
          )}

          {levels.length > 0 && (
            <>
              <div className="small-label">Level filter</div>
              <div className="level-filter">
                <button
                  className={`level-btn ${selectedLevels.length === 0 ? 'on' : ''}`}
                  onClick={() => update({ selectedLevels: [], selectedLevel: 'all' })}
                >
                  all
                </button>
                {levels.map(level => (
                  <button
                    key={level}
                    className={`level-btn ${selectedLevels.includes(level) ? 'on' : ''}`}
                    onClick={() => toggleLevel(level)}
                  >
                    {level}
                  </button>
                ))}
              </div>
            </>
          )}
        </>
      )}
    </Collapsible>
  );
};
