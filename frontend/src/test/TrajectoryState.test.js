import { describe, expect, it } from 'vitest';
import {
  appendTrajectoryHistoryPoint,
  shouldRecordTrajectoryHistoryPoint
} from '../utils/trajectoryState';

describe('trajectoryState', () => {
  it('does not record the discrete boundary-map seed as an iterate', () => {
    expect(shouldRecordTrajectoryHistoryPoint({ isContinuous: false, iteration: 0 })).toBe(false);

    const points = appendTrajectoryHistoryPoint({
      points: [],
      point: { x: 0.1, y: 0.1, nx: 1, ny: 0 },
      iteration: 0,
      isContinuous: false
    });

    expect(points).toEqual([]);
  });

  it('records completed discrete boundary-map images after the seed', () => {
    const point = { x: 1.096, y: 0.13, nx: 0, ny: 1 };
    const points = appendTrajectoryHistoryPoint({
      points: [],
      point,
      iteration: 1,
      isContinuous: false
    });

    expect(points).toEqual([point]);
  });

  it('keeps recording continuous trajectories from iteration zero', () => {
    const point = { x: 0.1, y: 0.1, nx: 1, ny: 0 };
    const points = appendTrajectoryHistoryPoint({
      points: [],
      point,
      iteration: 0,
      isContinuous: true
    });

    expect(points).toEqual([point]);
  });

  it('bounds continuous trajectory history when requested', () => {
    const points = appendTrajectoryHistoryPoint({
      points: [{ x: 0 }, { x: 1 }],
      point: { x: 2 },
      iteration: 3,
      isContinuous: true,
      maxHistory: 2
    });

    expect(points).toEqual([{ x: 1 }, { x: 2 }]);
  });
});
