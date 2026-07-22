import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Toggle } from '../ui/Toggle';
import {
  BASIN_ACCURACY_LIMITS,
  BASIN_ACCURACY_PRESETS,
  BASIN_COMPUTE_DEFAULTS,
  BASIN_LAYER_STYLES,
  validateBasinAccuracy
} from '../../utils/basinDisplay';

const formatInteger = value => Number(value || 0).toLocaleString();
const formatArea = value => Number.isFinite(value) ? value.toFixed(4) : 'n/a';
const formatBound = value => Number.isFinite(value) ? value.toFixed(3) : 'not available';

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
    return 'Backward expansion converged. Saturated yellow is verified finite capture; pale yellow remains possible at the refined resolution.';
  }
  if (!result.target_sampling_validated) {
    return 'The box computation converged, but the numerical MIS samples did not pass the closure and normal-continuity checks.';
  }
  if (!result.local_contraction_verified) {
    return 'Finite capture into the trapping core is verified, but contraction toward the MIS was not proved. This is not yet a complete attraction certificate.';
  }
  return 'Backward expansion converged with trapping, target-sampling, contraction, and domain checks verified.';
};

export const BasinApproximationPanel = ({ state, setState, canCompute, targetPointCount, compute, cancel }) => {
  const result = state.result;
  const settings = state.settings || BASIN_COMPUTE_DEFAULTS;
  const accuracy = validateBasinAccuracy(settings);
  const totalGridCells = result
    ? Number(result.grid_x || 0) * Number(result.grid_y || 0) * Number(result.grid_theta || 0)
    : 0;
  const statusClass = result?.end_to_end_verified ? 'ready' : 'limited';
  const updateSettings = changes => setState(previous => ({
    ...previous,
    settings: { ...(previous.settings || BASIN_COMPUTE_DEFAULTS), ...changes },
    result: null,
    error: null,
    notice: null
  }));
  const updateNumber = (key, integer = false) => event => {
    const numericValue = event.currentTarget.valueAsNumber;
    updateSettings({ [key]: integer && Number.isFinite(numericValue) ? Math.trunc(numericValue) : numericValue });
  };
  const selectedPreset = BASIN_ACCURACY_PRESETS.find(preset => Object.entries(preset.settings)
    .every(([key, value]) => settings[key] === value))?.id;
  const effective = accuracy.effective;
  const computeDisabled = !state.isComputing && (!canCompute || !accuracy.valid);

  return (
    <Collapsible title="Basin of attraction" defaultOpen={true}>
      <div className="basin-visibility-row">
        <Toggle label="Show basin" colorLine={BASIN_LAYER_STYLES.inner.color} checked={state.showBasin}
          onChange={showBasin => setState(previous => ({ ...previous, showBasin }))} />
      </div>
      <div className="basin-accuracy" aria-label="Basin accuracy controls">
        <div className="basin-control-heading">Accuracy</div>
        <div className="basin-presets" role="group" aria-label="Basin accuracy preset">
          {BASIN_ACCURACY_PRESETS.map(preset => (
            <button key={preset.id} className={`basin-preset${selectedPreset === preset.id ? ' active' : ''}`}
              type="button" aria-pressed={selectedPreset === preset.id} disabled={state.isComputing}
              onClick={() => updateSettings(preset.settings)}>
              {preset.label}
            </button>
          ))}
        </div>
        <details className="basin-details basin-accuracy-details">
          <summary>Numerical controls</summary>
          <div className="basin-control-grid">
            <label htmlFor="basin-grid-xy" title="Base number of spatial cells on each axis before refinement.">Position grid</label>
            <input id="basin-grid-xy" type="number" value={Number.isFinite(settings.gridXY) ? settings.gridXY : ''}
              min={BASIN_ACCURACY_LIMITS.minGridXY} max={BASIN_ACCURACY_LIMITS.maxGridXY} step="1"
              disabled={state.isComputing} onChange={updateNumber('gridXY', true)} />
            <label htmlFor="basin-grid-theta" title="Number of cells covering the full circle of normal directions.">Normal-angle grid</label>
            <input id="basin-grid-theta" type="number" value={Number.isFinite(settings.gridTheta) ? settings.gridTheta : ''}
              min={BASIN_ACCURACY_LIMITS.minGridTheta} max={BASIN_ACCURACY_LIMITS.maxGridTheta} step="1"
              disabled={state.isComputing} onChange={updateNumber('gridTheta', true)} />
            <label htmlFor="basin-refinements" title="Each pass doubles the position and normal-angle resolution. One or more also enables a nested-grid comparison.">Refinement passes</label>
            <input id="basin-refinements" type="number" value={Number.isFinite(settings.refinementRounds) ? settings.refinementRounds : ''}
              min={BASIN_ACCURACY_LIMITS.minRefinementRounds} max={BASIN_ACCURACY_LIMITS.maxRefinementRounds} step="1"
              disabled={state.isComputing} onChange={updateNumber('refinementRounds', true)} />
            <label htmlFor="basin-target-samples" title="Maximum number of resampled MIS boundary states supplied to the box computation.">Boundary samples</label>
            <input id="basin-target-samples" type="number" value={Number.isFinite(settings.targetSamples) ? settings.targetSamples : ''}
              min={BASIN_ACCURACY_LIMITS.minTargetSamples} max={BASIN_ACCURACY_LIMITS.maxTargetSamples} step="64"
              disabled={state.isComputing} onChange={updateNumber('targetSamples', true)} />
          </div>
          <div className="basin-control-heading basin-target-heading">Target enclosure</div>
          <div className="basin-control-grid">
            <label htmlFor="basin-position-radius" title="Position-space radius of the target tube. This changes the target set, not only numerical precision.">Position radius</label>
            <input id="basin-position-radius" type="number"
              value={Number.isFinite(settings.targetPositionRadius) ? settings.targetPositionRadius : ''}
              min="0.001" step="0.01" disabled={state.isComputing}
              onChange={updateNumber('targetPositionRadius')} />
            <label htmlFor="basin-angle-radius" title="Allowed circular difference from each supplied normal direction, in radians. This changes the target set.">Normal tolerance</label>
            <input id="basin-angle-radius" type="number"
              value={Number.isFinite(settings.targetAngleRadius) ? settings.targetAngleRadius : ''}
              min="0.001" max={Math.PI} step="0.05" disabled={state.isComputing}
              onChange={updateNumber('targetAngleRadius')} />
          </div>
          <div className="basin-note">Target radii define what is captured; changing them does not simply add precision.</div>
        </details>
        <div className="basin-grid-estimate">
          Effective grid {formatInteger(effective.x)} × {formatInteger(effective.y)} × {formatInteger(effective.theta)}
          {' = '}{formatInteger(effective.cells)} state cells
        </div>
        {settings.refinementRounds === 0 && accuracy.valid && (
          <div className="basin-control-warning">Use at least one refinement pass to run the nested-grid stability check.</div>
        )}
        {!accuracy.valid && <div className="basin-control-error" role="alert">{accuracy.error}</div>}
      </div>
      <button className="param-apply-btn basin-compute" type="button"
        onClick={state.isComputing ? cancel : compute}
        disabled={computeDisabled}>
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
            <span>finite-capture inner area {formatArea(result.inner_area)}</span>
            <span>possible outer area {formatArea(result.outer_area)}</span>
            <span>uncertainty gap {formatArea(result.unresolved_area)}</span>
          </div>
          <details className="basin-details basin-diagnostics">
            <summary>Computation details</summary>
            <div className="basin-metrics">
              <span>status {String(result.stop_reason).replaceAll('_', ' ')}</span>
              <span>converged {result.converged ? 'yes' : 'no'}</span>
              <span>end-to-end verified {result.end_to_end_verified ? 'yes' : 'no'}</span>
              <span>persistent grid {formatInteger(result.grid_x)} × {formatInteger(result.grid_y)} × {formatInteger(result.grid_theta)}</span>
              <span>automatic refinements {formatInteger(result.refinement_rounds)}</span>
              <span>nested-grid stable {result.refinement_stable == null ? 'not run' : result.refinement_stable ? 'yes' : 'no'}</span>
              <span>area change inner/outer {formatArea(result.refinement_inner_area_change)} / {formatArea(result.refinement_outer_area_change)}</span>
              <span>inner cells {formatInteger(result.inner_cell_count)}</span>
              <span>outer cells {formatInteger(result.outer_cell_count)}</span>
              <span>target core {formatInteger(result.target_cell_count)} / {formatInteger(result.candidate_target_cell_count)}</span>
              <span>target sampling {result.target_sampling_validated ? 'validated' : 'unverified'}</span>
              <span>contraction bound {formatBound(result.local_contraction_upper_bound)} ({result.local_contraction_verified ? 'verified' : 'unverified'})</span>
              <span>sample spacing median/max {formatBound(result.target_median_spacing)} / {formatBound(result.target_max_spacing)}</span>
              <span>closure gap {formatBound(result.target_closure_gap)}</span>
              <span>levels in/out {result.completed_inner_levels} / {result.completed_outer_levels}</span>
              <span>forward rows {formatInteger(result.evaluated_cell_count)} / {formatInteger(totalGridCells)}</span>
              <span>inverse frontiers {formatInteger(result.inverse_frontier_cell_count)}</span>
              <span>consistency rejections {formatInteger(result.forward_consistency_rejection_count)}</span>
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
