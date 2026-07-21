import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Slider } from '../ui/Slider';
import { Toggle } from '../ui/Toggle';

export const GeometricOffsetsPanel = ({ state, setState, canCompute, compute }) => {
  const result = state.result;
  const latest = result?.levels?.[result.levels.length - 1];
  return (
    <Collapsible title="Geometric offsets" defaultOpen={true}>
      <Slider label="Levels" min={1} max={8} step={1} value={state.numLevels} disabled={state.isComputing}
        onChange={numLevels => setState(prev => ({ ...prev, numLevels }))} />
      <Slider label="Resolution" hint="grid" min={128} max={320} step={32} value={state.resolution} disabled={state.isComputing}
        onChange={resolution => setState(prev => ({ ...prev, resolution }))} />
      <Toggle label="Show geometric offset contours" checked={state.showContours}
        onChange={showContours => setState(prev => ({ ...prev, showContours }))} />
      <button className="param-apply-btn geometric-offset-compute" type="button" onClick={compute}
        disabled={!canCompute || state.isComputing}>
        {state.isComputing ? 'Computing geometric offsets…' : 'Compute exact ε-offset contours'}
      </button>
      {!canCompute && <div className="geometric-offset-note">Waiting for a closed unstable-manifold boundary.</div>}
      <div className="geometric-offset-note">Each contour is the signed-distance level kε from the MIS seed. Widen the current view if a requested contour reaches its edge.</div>
      {state.error && <div className="geometric-offset-status error" role="alert">{state.error}</div>}
      {result && !state.error && (
        <div aria-live="polite" className={`geometric-offset-status ${result.stop_reason === 'requested_levels_completed' ? 'ready' : 'warning'}`}>
          <div>{result.completed_levels} level{result.completed_levels === 1 ? '' : 's'} · {String(result.stop_reason).replaceAll('_', ' ')}</div>
          {latest && <div className="geometric-offset-metrics">
            <span>gap ε {result.epsilon.toFixed(4)}</span>
            <span>outer distance {latest.target_distance.toFixed(4)}</span>
            <span>area {latest.area.toFixed(4)}</span>
            <span>{latest.component_count} component{latest.component_count === 1 ? '' : 's'}</span>
            <span>gap residual {latest.gap_residual.toExponential(2)}</span>
            <span>target residual {latest.offset_residual.toExponential(2)}</span>
            <span>uncertainty ±{latest.uncertainty.toExponential(2)}</span>
          </div>}
          <div className="geometric-offset-levels" aria-label="Geometric offset target distances">
            {result.levels.map(level => (
              <span key={level.level}>G{level.level} {level.target_distance.toFixed(3)}</span>
            ))}
          </div>
        </div>
      )}
    </Collapsible>
  );
};
