import { describe, expect, it } from 'vitest';
import { BASIN_COMPUTE_DEFAULTS, BASIN_LAYER_STYLES } from './basinDisplay';

describe('basin display styling', () => {
  it('uses two yellow tones and draws the verified core above the uncertainty band', () => {
    expect(BASIN_LAYER_STYLES.inner.color).toBe('#ffd400');
    expect(BASIN_LAYER_STYLES.outer.color).toBe('#ffe56b');
    expect(BASIN_LAYER_STYLES.inner.opacity).toBeGreaterThanOrEqual(0.9);
    expect(BASIN_LAYER_STYLES.outer.opacity).toBeGreaterThanOrEqual(0.45);
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
