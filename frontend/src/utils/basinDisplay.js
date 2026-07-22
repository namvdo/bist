export const BASIN_LAYER_STYLES = Object.freeze({
  inner: Object.freeze({
    color: '#ffd400',
    opacity: 0.98,
    minimumOpacity: 0.82,
    z: 0.05,
    renderOrder: -20
  }),
  outer: Object.freeze({
    color: '#ffe066',
    opacity: 0.82,
    minimumOpacity: 0.55,
    z: 0.04,
    renderOrder: -30
  })
});

export const BASIN_COMPUTE_DEFAULTS = Object.freeze({
  gridXY: 24,
  gridTheta: 16,
  refinementRounds: 1,
  targetSamples: 2000,
  targetPositionRadius: 0.25,
  targetAngleRadius: 0.8
});

export const BASIN_ACCURACY_PRESETS = Object.freeze([
  Object.freeze({
    id: 'draft',
    label: 'Draft',
    settings: Object.freeze({ gridXY: 16, gridTheta: 12, refinementRounds: 1, targetSamples: 1000 })
  }),
  Object.freeze({
    id: 'standard',
    label: 'Standard',
    settings: Object.freeze({ gridXY: 24, gridTheta: 16, refinementRounds: 1, targetSamples: 2000 })
  }),
  Object.freeze({
    id: 'fine',
    label: 'Fine',
    settings: Object.freeze({ gridXY: 24, gridTheta: 16, refinementRounds: 2, targetSamples: 4000 })
  })
]);

export const BASIN_ACCURACY_LIMITS = Object.freeze({
  minGridXY: 4,
  maxGridXY: 512,
  minGridTheta: 8,
  maxGridTheta: 256,
  minRefinementRounds: 0,
  maxRefinementRounds: 2,
  minTargetSamples: 64,
  maxTargetSamples: 8000,
  maxEffectiveAxis: 512,
  maxEffectiveTheta: 256,
  maxStateCells: 2_000_000
});

/** Return the actual persistent grid after uniform refinement. */
export const basinEffectiveGrid = settings => {
  const factor = 2 ** Number(settings?.refinementRounds ?? 0);
  const x = Number(settings?.gridXY ?? 0) * factor;
  const theta = Number(settings?.gridTheta ?? 0) * factor;
  return { x, y: x, theta, cells: x * x * theta };
};

/**
 * Mirror the backend's hard grid guards so invalid or excessive work fails
 * before a worker and WebAssembly computation are started.
 */
export const validateBasinAccuracy = settings => {
  const limits = BASIN_ACCURACY_LIMITS;
  const integerFields = [
    ['Position grid', settings?.gridXY, limits.minGridXY, limits.maxGridXY],
    ['Normal-angle grid', settings?.gridTheta, limits.minGridTheta, limits.maxGridTheta],
    ['Refinement passes', settings?.refinementRounds, limits.minRefinementRounds, limits.maxRefinementRounds],
    ['Boundary samples', settings?.targetSamples, limits.minTargetSamples, limits.maxTargetSamples]
  ];
  for (const [label, value, minimum, maximum] of integerFields) {
    if (!Number.isInteger(value) || value < minimum || value > maximum) {
      return { valid: false, error: `${label} must be an integer from ${minimum} to ${maximum}.`, effective: basinEffectiveGrid(settings) };
    }
  }
  if (!Number.isFinite(settings?.targetPositionRadius) || settings.targetPositionRadius <= 0) {
    return { valid: false, error: 'Target position radius must be positive.', effective: basinEffectiveGrid(settings) };
  }
  if (!Number.isFinite(settings?.targetAngleRadius)
    || settings.targetAngleRadius <= 0
    || settings.targetAngleRadius > Math.PI) {
    return { valid: false, error: 'Target normal tolerance must lie in (0, π] radians.', effective: basinEffectiveGrid(settings) };
  }

  const effective = basinEffectiveGrid(settings);
  if (effective.x > limits.maxEffectiveAxis || effective.theta > limits.maxEffectiveTheta) {
    return { valid: false, error: 'The refined grid exceeds the supported per-axis limit.', effective };
  }
  if (!Number.isSafeInteger(effective.cells) || effective.cells > limits.maxStateCells) {
    return { valid: false, error: `The refined grid exceeds the ${limits.maxStateCells.toLocaleString()}-cell safety limit.`, effective };
  }
  return { valid: true, error: null, effective };
};

export const basinLayerOpacity = (style, coverage) => {
  const normalized = Math.max(0, Math.min(1, Number(coverage) || 0));
  return style.minimumOpacity + (style.opacity - style.minimumOpacity) * Math.sqrt(normalized);
};
