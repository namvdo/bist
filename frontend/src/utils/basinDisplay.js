export const BASIN_LAYER_STYLES = Object.freeze({
  inner: Object.freeze({
    color: '#2797ff',
    opacity: 0.78,
    z: 0.05,
    renderOrder: -20
  }),
  outer: Object.freeze({
    color: '#78d7ff',
    opacity: 0.3,
    z: 0.04,
    renderOrder: -30
  })
});

// These defaults give a useful first computation without exposing implementation
// parameters in the everyday UI. Search depth remains user-controlled because it
// changes how far the inverse frontier is expanded.
export const BASIN_COMPUTE_DEFAULTS = Object.freeze({
  gridXY: 40,
  gridTheta: 24,
  targetPositionRadius: 0.25,
  targetAngleRadius: 0.8
});
