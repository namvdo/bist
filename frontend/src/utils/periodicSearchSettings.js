export const DEFAULT_PERIODIC_SEARCH_SETTINGS = {
  gridSize: 10,
  thetaGridSize: 10,
  residualThreshold: 1e-10,
  useContinuation: false
};

export const PERIODIC_SEARCH_LIMITS = {
  gridSizeMin: 2,
  gridSizeMax: 256,
  thetaGridSizeMin: 2,
  thetaGridSizeMax: 256,
  residualThresholdMin: 1e-14,
  residualThresholdMax: 1e-2
};

export const normalizePeriodicSearchSettings = (next, fallback = DEFAULT_PERIODIC_SEARCH_SETTINGS) => {
  const safeFallback = fallback || DEFAULT_PERIODIC_SEARCH_SETTINGS;
  const parsedGrid = Number.parseInt(`${next?.gridSize ?? safeFallback.gridSize}`, 10);
  const parsedTheta = Number.parseInt(`${next?.thetaGridSize ?? safeFallback.thetaGridSize}`, 10);
  const parsedThreshold = Number(next?.residualThreshold ?? safeFallback.residualThreshold);
  const useContinuation = Boolean(next?.useContinuation ?? safeFallback.useContinuation);

  const gridSize = Number.isFinite(parsedGrid)
    ? Math.min(PERIODIC_SEARCH_LIMITS.gridSizeMax, Math.max(PERIODIC_SEARCH_LIMITS.gridSizeMin, parsedGrid))
    : safeFallback.gridSize;

  const thetaGridSize = Number.isFinite(parsedTheta)
    ? Math.min(PERIODIC_SEARCH_LIMITS.thetaGridSizeMax, Math.max(PERIODIC_SEARCH_LIMITS.thetaGridSizeMin, parsedTheta))
    : safeFallback.thetaGridSize;

  const residualThreshold = Number.isFinite(parsedThreshold) && parsedThreshold > 0
    ? Math.min(PERIODIC_SEARCH_LIMITS.residualThresholdMax, Math.max(PERIODIC_SEARCH_LIMITS.residualThresholdMin, parsedThreshold))
    : safeFallback.residualThreshold;

  return {
    gridSize,
    thetaGridSize,
    residualThreshold,
    useContinuation
  };
};
