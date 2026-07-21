import { describe, expect, it } from 'vitest';
import { BASIN_COMPUTE_DEFAULTS, BASIN_LAYER_STYLES } from './basinDisplay';

describe('basin display styling', () => {
  it('uses two blue tones and draws the verified core above the uncertainty band', () => {
    expect(BASIN_LAYER_STYLES.inner.color).toBe('#2797ff');
    expect(BASIN_LAYER_STYLES.outer.color).toBe('#78d7ff');
    expect(BASIN_LAYER_STYLES.inner.z).toBeGreaterThan(BASIN_LAYER_STYLES.outer.z);
    expect(BASIN_LAYER_STYLES.inner.opacity).toBeGreaterThan(BASIN_LAYER_STYLES.outer.opacity);
  });

  it('keeps stable numerical defaults outside the everyday UI', () => {
    expect(BASIN_COMPUTE_DEFAULTS).toEqual({
      gridXY: 40,
      gridTheta: 24,
      targetPositionRadius: 0.25,
      targetAngleRadius: 0.8
    });
  });
});
