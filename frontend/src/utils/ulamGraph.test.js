import { describe, expect, it } from 'vitest';
import {
  buildReverseTransitions,
  computeAbsorptionProbabilities,
  computeReachabilityLayers,
  findReachabilityBoundary,
  getGridNeighbors
} from './ulamGraph';

describe('buildReverseTransitions', () => {
  it('reverses directed Ulam transitions above the probability threshold', () => {
    const transitionsByBox = [
      [{ index: 1, probability: 0.9 }, { index: 2, probability: 0.05 }],
      [{ index: 2, probability: 0.8 }],
      []
    ];

    expect(buildReverseTransitions({
      transitionsByBox,
      boxCount: 3,
      probabilityThreshold: 0.1
    })).toEqual([
      [],
      [0],
      [1]
    ]);
  });
});

describe('computeReachabilityLayers', () => {
  it('computes minimum candidate depth to a target box', () => {
    const transitionsByBox = [
      [{ index: 1, probability: 1 }],
      [{ index: 2, probability: 1 }],
      [{ index: 3, probability: 1 }],
      []
    ];

    const result = computeReachabilityLayers({
      transitionsByBox,
      targetIndices: [3],
      boxCount: 4,
      maxDepth: 3
    });

    expect(result.depthByBox).toEqual([3, 2, 1, 0]);
    expect(result.layers).toEqual([[3], [2], [1], [0]]);
  });

  it('uses the shortest depth when multiple equilibrium target boxes exist', () => {
    const transitionsByBox = [
      [{ index: 1, probability: 1 }],
      [{ index: 2, probability: 1 }],
      [{ index: 3, probability: 1 }],
      [],
      [{ index: 5, probability: 1 }],
      []
    ];

    const result = computeReachabilityLayers({
      transitionsByBox,
      targetIndices: [3, 5],
      boxCount: 6,
      maxDepth: 3
    });

    expect(result.depthByBox).toEqual([3, 2, 1, 0, 1, 0]);
  });
});

describe('grid boundary helpers', () => {
  it('returns four-neighborhood grid neighbors by default', () => {
    expect(getGridNeighbors(4, 3)).toEqual([1, 3, 5, 7]);
    expect(getGridNeighbors(0, 3)).toEqual([1, 3]);
  });

  it('finds boundary boxes between reachable and unreachable graph regions', () => {
    const depthByBox = [
      -1, 1, -1,
      -1, 0, -1,
      -1, -1, -1
    ];

    expect(findReachabilityBoundary({ depthByBox, subdivisions: 3 })).toEqual({
      insideBoundary: [1, 4],
      outsideBoundary: [0, 2, 3, 5, 7]
    });
  });
});

describe('computeAbsorptionProbabilities', () => {
  it('propagates target-reaching probability through the transition matrix', () => {
    const transitionsByBox = [
      [{ index: 1, probability: 0.25 }, { index: 2, probability: 0.75 }],
      [{ index: 3, probability: 1 }],
      [],
      []
    ];

    expect(computeAbsorptionProbabilities({
      transitionsByBox,
      targetIndices: [3],
      boxCount: 4,
      iterations: 2
    })).toEqual([0.25, 1, 0, 1]);
  });
});
