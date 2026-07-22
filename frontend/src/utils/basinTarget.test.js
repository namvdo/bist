import { describe, expect, it } from 'vitest';
import {
  buildBasinTarget,
  deriveExtendedBoundary,
  resampleClosedBoundary,
  resampleExtendedBoundary,
  targetNeedsGeometricRepair
} from './basinTarget';

describe('basinTarget', () => {
  it('preserves and normalizes MIS boundary normal directions', () => {
    const points = [
      [0, 0, 2, 0],
      [1, 0, 0, -3],
      [1, 1, -4, 0],
      [0, 1, 0, 5]
    ];
    const target = buildBasinTarget([{ plus: { extended_points: points, points: points.map(p => p.slice(0, 2)) } }]);
    expect(target).toHaveLength(4);
    target.forEach(point => expect(Math.hypot(point.nx, point.ny)).toBeCloseTo(1, 12));
  });

  it('derives outward unit normals when only position samples exist', () => {
    const target = buildBasinTarget([{ plus: { points: [[0, 0], [1, 0], [1, 1], [0, 1]] } }]);
    expect(target).toHaveLength(4);
    target.forEach(point => expect(Math.hypot(point.nx, point.ny)).toBeCloseTo(1, 12));
  });

  it('completes a negative-multiplier boundary-state phase through the deterministic map', () => {
    const params = { a: 0.4, b: 0.3, epsilon: 0.1 };
    const extended = [
      [1, 0, 1, 0],
      [1.2, 0.2, 0.8, 0.6],
      [1.1, 0.5, 0, 1]
    ];
    const target = buildBasinTarget([{
      eigenvalue: -1.1,
      plus: { extended_points: extended, points: extended.map(p => p.slice(0, 2)) }
    }], 100, params);
    expect(target.length).toBeGreaterThanOrEqual(3);
    expect(target.every(point => [point.x, point.y, point.nx, point.ny].every(Number.isFinite))).toBe(true);
  });

  it('rejects degenerate geometric boundaries', () => {
    expect(deriveExtendedBoundary([[0, 0], [1, 0], [2, 0]])).toHaveLength(3);
    expect(buildBasinTarget([{ plus: { points: [[0, 0], [1, 0], [2, 0]] } }])).toEqual([]);
  });

  it('limits transfer size', () => {
    const points = Array.from({ length: 120 }, (_, index) => {
      const theta = 2 * Math.PI * index / 120;
      return [Math.cos(theta), Math.sin(theta), Math.cos(theta), Math.sin(theta)];
    });
    expect(buildBasinTarget([{ plus: { extended_points: points, points: points.map(p => p.slice(0, 2)) } }], 24)).toHaveLength(24);
  });

  it('repairs a discontinuous branch seam with uniform position and normal interpolation', () => {
    const discontinuous = [
      [0, 0, 1, 0],
      [1, 0, 1, 0],
      [1, 1, 1, 0],
      [0, 1, -1, 0]
    ];
    expect(targetNeedsGeometricRepair(discontinuous)).toBe(true);
    const target = buildBasinTarget([{
      plus: { extended_points: discontinuous, points: discontinuous.map(point => point.slice(0, 2)) }
    }], 20);
    expect(target).toHaveLength(20);
    const repaired = target.map(({ x, y, nx, ny }) => [x, y, nx, ny]);
    expect(targetNeedsGeometricRepair(repaired)).toBe(false);
    repaired.forEach(point => expect(Math.hypot(point[2], point[3])).toBeCloseTo(1, 12));
  });

  it('interpolates normal directions along the shortest circular arc', () => {
    const boundary = [
      [0, 0, 1, 0],
      [1, 0, 0, 1],
      [1, 1, -1, 0],
      [0, 1, 0, -1]
    ];
    const sampled = resampleExtendedBoundary(boundary, 8);
    expect(sampled[1][2]).toBeCloseTo(Math.SQRT1_2, 12);
    expect(sampled[1][3]).toBeCloseTo(Math.SQRT1_2, 12);
    sampled.forEach(point => expect(Math.hypot(point[2], point[3])).toBeCloseTo(1, 12));
  });

  it('resamples a closed boundary without duplicating the endpoint', () => {
    const sampled = resampleClosedBoundary([[0, 0], [1, 0], [1, 1], [0, 1]], 8);
    expect(sampled).toHaveLength(8);
    expect(sampled[0]).toEqual([0, 0]);
    expect(sampled.at(-1)).not.toEqual(sampled[0]);
    const lengths = sampled.map((point, index) => {
      const next = sampled[(index + 1) % sampled.length];
      return Math.hypot(next[0] - point[0], next[1] - point[1]);
    });
    expect(Math.max(...lengths) - Math.min(...lengths)).toBeLessThan(1e-12);
  });
});
