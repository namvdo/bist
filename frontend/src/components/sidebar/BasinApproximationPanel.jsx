import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Slider } from '../ui/Slider';
import { Toggle } from '../ui/Toggle';
import { BASIN_LAYER_STYLES } from '../../utils/basinDisplay';

const formatInteger = value => Number(value || 0).toLocaleString();
const formatArea = value => Number.isFinite(value) ? value.toFixed(4) : 'n/a';

export const BasinApproximationPanel = ({ state, setState, canCompute, targetPointCount, compute }) => {
  const result = state.result;
  const totalGridCells = result
    ? Number(result.grid_x || 0) * Number(result.grid_y || 0) * Number(result.grid_theta || 0)
    : 0;
  const statusClass = result?.trapping_verified ? 'ready' : 'limited';

  return (
    <Collapsible title="Basin of attraction" defaultOpen={true}>
      <Slider label="Search depth" hint="map steps" min={1} max={60} step={1} value={state.maxLevels} disabled={state.isComputing}
        onChange={maxLevels => setState(previous => ({ ...previous, maxLevels, result: null }))} />
      <div className="basin-visibility-row">
        <Toggle label="Show basin" colorLine={BASIN_LAYER_STYLES.inner.color} checked={state.showBasin}
          onChange={showBasin => setState(previous => ({ ...previous, showBasin }))} />
      </div>
      <button className="param-apply-btn basin-compute" type="button" onClick={compute}
        disabled={!canCompute || state.isComputing}>
        {state.isComputing ? 'Computing basin…' : 'Compute basin'}
      </button>
      {!canCompute && <div className="basin-note">First compute a closed unstable-manifold MIS. Its lifted boundary samples seed the basin search.</div>}
      {canCompute && <div className="basin-note">Ready to search from {formatInteger(targetPointCount)} MIS boundary samples.</div>}

      {state.error && <div className="basin-status error" role="alert">{state.error}</div>}
      {result && !state.error && (
        <div aria-live="polite" className={`basin-status ${statusClass}`}>
          <div className="basin-status-title">Basin computation complete</div>
          <div className="basin-status-copy">
            {result.trapping_verified
              ? 'The dark-blue region is verified. The pale-blue edge records numerical uncertainty.'
              : 'This grid gives an outer estimate only because a trapping core was not verified.'}
          </div>
          <div className="basin-result-summary">
            <span>verified area {formatArea(result.inner_area)}</span>
            <span>covered area {formatArea(result.outer_area)}</span>
          </div>
          <details className="basin-details basin-diagnostics">
            <summary>Computation details</summary>
            <div className="basin-metrics">
              <span>status {String(result.stop_reason).replaceAll('_', ' ')}</span>
              <span>unresolved {formatArea(result.unresolved_area)}</span>
              <span>inner cells {formatInteger(result.inner_cell_count)}</span>
              <span>outer cells {formatInteger(result.outer_cell_count)}</span>
              <span>target core {formatInteger(result.target_cell_count)} / {formatInteger(result.candidate_target_cell_count)}</span>
              <span>levels in/out {result.completed_inner_levels} / {result.completed_outer_levels}</span>
              <span>forward rows {formatInteger(result.evaluated_cell_count)} / {formatInteger(totalGridCells)}</span>
              <span>inverse frontiers {formatInteger(result.inverse_frontier_cell_count)}</span>
              <span>graph edges {formatInteger(result.graph_edge_count)}</span>
              <span>stored runs {formatInteger(result.transition_run_count)}</span>
              <span>boundary contacts {formatInteger(result.boundary_contact_cell_count)}</span>
            </div>
          </details>
        </div>
      )}
    </Collapsible>
  );
};
