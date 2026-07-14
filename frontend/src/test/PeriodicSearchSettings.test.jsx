import { describe, expect, it } from 'vitest';
import { normalizePeriodicSearchSettings } from '../utils/periodicSearchSettings';

describe('normalizePeriodicSearchSettings', () => {
  it('clamps grid and theta sizes to supported bounds', () => {
    const normalized = normalizePeriodicSearchSettings({
      gridSize: 1,
      thetaGridSize: 9999,
      residualThreshold: 1e-10
    });

    expect(normalized.gridSize).toBe(2);
    expect(normalized.thetaGridSize).toBe(256);
  });

  it('falls back to defaults for invalid threshold values', () => {
    const normalized = normalizePeriodicSearchSettings({
      gridSize: 10,
      thetaGridSize: 10,
      residualThreshold: NaN
    });

    expect(normalized.residualThreshold).toBe(1e-10);
  });

  it('normalizes the optional continuation flag', () => {
    expect(normalizePeriodicSearchSettings({ useContinuation: true }).useContinuation).toBe(true);
    expect(normalizePeriodicSearchSettings({ useContinuation: false }, { useContinuation: true }).useContinuation).toBe(false);
  });
});
