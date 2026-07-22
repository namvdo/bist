import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Toggle } from '../ui/Toggle';
import { BASIN_LAYER_STYLES } from '../../utils/basinDisplay';

const formatInteger = value => Number(value || 0).toLocaleString();
const formatArea = value => Number.isFinite(value) ? value.toFixed(4) : 'n/a';

const describeResult = result => {
  if (!result?.converged) {
    return `The internal safety limit of ${formatInteger(result?.expansion_limit)} expansion rounds was reached before convergence. The displayed basin is incomplete.`;
  }
  if (!result.trapping_verified) {
    return 'Backward expansion converged, but this grid gives an outer estimate only because a trapping core was not verified.';
  }
  if (result.stop_reason === 'domain_truncated') {
    return 'Backward expansion converged inside the selected domain, but the basin reaches its boundary and may continue outside it.';
  }
  if (result.stop_reason === 'resolution_limited') {
    return 'Backward expansion converged. The pale-yellow edge records uncertainty at the current grid resolution.';
  }
  return 'Backward expansion converged: no new predecessor boxes were found.';
};

export const BasinApproximationPanel = ({ state, setState, canCompute, targetPointCount, compute, cancel }) => {
  const result = state.result;
  const totalGridCells = result
    ? Number(result.grid_x || 0) * Number(result.grid_y || 0) * Number(result.grid_theta || 0)
    : 0;
  const statusClass = result?.converged && result?.trapping_verified ? 'ready' : 'limited';

  return (
    <Collapsible title="Basin of attraction" defaultOpen={true}>
      <div className="basin-visibility-row">
        <Toggle label="Show basin" colorLine={BASIN_LAYER_STYLES.inner.color} checked={state.showBasin}
          onChange={showBasin => setState(previous => ({ ...previous, showBasin }))} />
      </div>
      <button className="param-apply-btn basin-compute" type="button"
        onClick={state.isComputing ? cancel : compute}
        disabled={!state.isComputing && !canCompute}>
        {state.isComputing ? 'Cancel basin computation' : 'Compute basin'}
      </button>
      {state.isComputing
        ? <div className="basin-note" role="status">Expanding backward until no new boxes are found…</div>
        : canCompute && <div className="basin-note">Ready to search from {formatInteger(targetPointCount)} MIS boundary samples.</div>}

      {state.error && <div className="basin-status error" role="alert">{state.error}</div>}
      {state.notice && !state.error && <div className="basin-status limited" role="status">{state.notice}</div>}
      {result && !state.error && (
        <div aria-live="polite" className={`basin-status ${statusClass}`}>
          <div className="basin-status-title">Basin computation complete</div>
          <div className="basin-status-copy">{describeResult(result)}</div>
          <div className="basin-result-summary">
            <span>verified area {formatArea(result.inner_area)}</span>
            <span>covered area {formatArea(result.outer_area)}</span>
          </div>
          <details className="basin-details basin-diagnostics">
            <summary>Computation details</summary>
            <div className="basin-metrics">
              <span>status {String(result.stop_reason).replaceAll('_', ' ')}</span>
              <span>converged {result.converged ? 'yes' : 'no'}</span>
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
