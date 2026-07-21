import { describe, expect, it } from 'vitest';
import {
  buildGeometricOffsetSeed,
  completeNegativeMultiplierPhase,
  forwardBoundaryPoint,
  isClosedCandidate,
  joinClosedBranches,
  signedArea
} from './geometricOffsetSeed';

describe('geometricOffsetSeed', () => {
  it('selects the largest closed manifold candidate', () => {
    const manifolds = [{
      plus: { points: [[0, 0], [2, 0], [2, 2], [0, 2]] },
      minus: { points: [[0, 0], [0.2, 0], [0, 0.2]] },
    }];
    const seed = buildGeometricOffsetSeed(manifolds);
    expect(seed).toHaveLength(4);
    expect(Math.abs(signedArea(seed))).toBe(4);
  });

  it('rejects missing and degenerate seeds', () => {
    expect(buildGeometricOffsetSeed([])).toEqual([]);
    expect(buildGeometricOffsetSeed([{ plus: { points: [[0, 0], [1, 0], [2, 0]] } }])).toEqual([]);
  });

  it('rejects an open trajectory whose artificial closing edge is too long', () => {
    const openArc = Array.from({ length: 40 }, (_, i) => [i / 39, Math.sin(i / 39)]);
    expect(isClosedCandidate(openArc)).toBe(false);
    expect(buildGeometricOffsetSeed([{ plus: { points: openArc } }])).toEqual([]);
  });

  it('accepts two closed seams even when adaptive sampling has long internal edges', () => {
    const plus = [[0, 0], [0.01, 0.02], [0.02, 0.04], [1, 1], [4, 0]];
    const minus = [[0, 0.001], [0.01, -0.02], [0.02, -0.04], [1, -1], [4, 0.001]];
    const joined = joinClosedBranches(plus, minus);
    expect(joined).not.toBeNull();
    expect(buildGeometricOffsetSeed([{ plus: { points: plus }, minus: { points: minus } }])).toHaveLength(10);
  });

  it('completes the missing phase for a negative multiplier', () => {
    const params = { a: 0.4, b: 0.3, epsilon: 0.1 };
    expect(forwardBoundaryPoint([1, 0, 1, 0], params)).toEqual([0.6, 0.4, 0, 1]);
    const trajectory = {
      extended_points: [
        [1, 0, 1, 0],
        [1.2, 0.2, 0.8, 0.6],
        [1.1, 0.5, 0, 1]
      ]
    };
    expect(completeNegativeMultiplierPhase(trajectory, -1.05, params)).toHaveLength(6);
    expect(completeNegativeMultiplierPhase(trajectory, 1.05, params)).toBeNull();
  });

  it('bounds the seed size for worker transfer', () => {
    const points = Array.from({ length: 100 }, (_, i) => {
      const angle = Math.PI * 2 * i / 100;
      return [Math.cos(angle), Math.sin(angle)];
    });
    expect(buildGeometricOffsetSeed([{ plus: { points } }], 20).length).toBeLessThanOrEqual(20);
  });
});
