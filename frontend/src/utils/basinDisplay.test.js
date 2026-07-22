import { describe, expect, it } from 'vitest';
import {
  basinEffectiveGrid,
  basinLayerOpacity,
  BASIN_ACCURACY_PRESETS,
  BASIN_COMPUTE_DEFAULTS,
  BASIN_LAYER_STYLES,
  validateBasinAccuracy
} from './basinDisplay';

describe('basin display styling', () => {
  it('uses two yellow tones and draws the verified core above the uncertainty band', () => {
    expect(BASIN_LAYER_STYLES.inner.color).toBe('#ffd400');
    expect(BASIN_LAYER_STYLES.outer.color).toBe('#ffe066');
    expect(BASIN_LAYER_STYLES.inner.opacity).toBeGreaterThanOrEqual(0.9);
    expect(BASIN_LAYER_STYLES.outer.opacity).toBeGreaterThanOrEqual(0.8);
    expect(BASIN_LAYER_STYLES.inner.z).toBeGreaterThan(BASIN_LAYER_STYLES.outer.z);
    expect(BASIN_LAYER_STYLES.inner.opacity).toBeGreaterThan(BASIN_LAYER_STYLES.outer.opacity);
  });

  it('keeps stable numerical defaults outside the everyday UI', () => {
    expect(BASIN_COMPUTE_DEFAULTS).toEqual({
      gridXY: 24,
      gridTheta: 16,
      refinementRounds: 1,
      targetSamples: 2000,
      targetPositionRadius: 0.25,
      targetAngleRadius: 0.8
    });
  });

  it('provides bounded presets and reports their effective persistent grids', () => {
    expect(BASIN_ACCURACY_PRESETS.map(preset => preset.id)).toEqual(['draft', 'standard', 'fine']);
    expect(basinEffectiveGrid(BASIN_COMPUTE_DEFAULTS)).toEqual({
      x: 48,
      y: 48,
      theta: 32,
      cells: 73728
    });
    const fine = { ...BASIN_COMPUTE_DEFAULTS, ...BASIN_ACCURACY_PRESETS[2].settings };
    expect(validateBasinAccuracy(fine)).toMatchObject({
      valid: true,
      effective: { x: 96, y: 96, theta: 64, cells: 589824 }
    });
  });

  it('fails fast when custom settings exceed resource or target limits', () => {
    expect(validateBasinAccuracy({ ...BASIN_COMPUTE_DEFAULTS, gridXY: 128, refinementRounds: 2 }))
      .toMatchObject({ valid: false, error: expect.stringContaining('safety limit') });
    expect(validateBasinAccuracy({ ...BASIN_COMPUTE_DEFAULTS, targetAngleRadius: Math.PI + 0.01 }))
      .toMatchObject({ valid: false, error: expect.stringContaining('(0, π]') });
    expect(validateBasinAccuracy({ ...BASIN_COMPUTE_DEFAULTS, targetSamples: 12 }))
      .toMatchObject({ valid: false, error: expect.stringContaining('Boundary samples') });
  });

  it('keeps low angular coverage visible while strengthening dense cells', () => {
    expect(basinLayerOpacity(BASIN_LAYER_STYLES.outer, 0)).toBe(BASIN_LAYER_STYLES.outer.minimumOpacity);
    expect(basinLayerOpacity(BASIN_LAYER_STYLES.outer, 1)).toBe(BASIN_LAYER_STYLES.outer.opacity);
    expect(basinLayerOpacity(BASIN_LAYER_STYLES.inner, 0.75)).toBeGreaterThan(
      basinLayerOpacity(BASIN_LAYER_STYLES.inner, 0.25)
    );
  });
});
