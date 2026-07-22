import { describe, expect, it } from 'vitest';
import { buildBasinTarget, deriveExtendedBoundary } from './basinTarget';

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
});
