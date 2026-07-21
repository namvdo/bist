import { describe, expect, it } from 'vitest';
import {
  MIN_VIEW_SPAN,
  normalizeViewRange,
  RANGE_LIMIT,
  zoomViewRange
} from './viewRange';

describe('normalizeViewRange', () => {
  it('orders and clamps values', () => {
    const result = normalizeViewRange({ xMin: 12, xMax: -5, yMin: -20, yMax: 3 });
    expect(result.xMin).toBe(-5);
    expect(result.xMax).toBe(RANGE_LIMIT);
    expect(result.yMin).toBe(-RANGE_LIMIT);
    expect(result.yMax).toBe(3);
  });

  it('expands zero-width ranges', () => {
    const result = normalizeViewRange({ xMin: 1, xMax: 1, yMin: 2, yMax: 2 });
    expect(result.xMax).toBeGreaterThan(result.xMin);
    expect(result.yMax).toBeGreaterThan(result.yMin);
  });
});

describe('zoomViewRange', () => {
  it('zooms around the current center', () => {
    const result = zoomViewRange({ xMin: -2, xMax: 2, yMin: -1.5, yMax: 1.5 }, 0.8);

    expect(result.xMin).toBeCloseTo(-1.6);
    expect(result.xMax).toBeCloseTo(1.6);
    expect(result.yMin).toBeCloseTo(-1.2);
    expect(result.yMax).toBeCloseTo(1.2);
  });

  it('keeps zoom-out ranges inside the global bounds without shrinking their span', () => {
    const result = zoomViewRange({ xMin: 8, xMax: 10, yMin: -10, yMax: -8 }, 2);

    expect(result).toEqual({ xMin: 6, xMax: 10, yMin: -10, yMax: -6 });
  });

  it('stops zoom-in before the range becomes numerically unusable', () => {
    const result = zoomViewRange({ xMin: -0.01, xMax: 0.01, yMin: -0.01, yMax: 0.01 }, 0.1);

    expect(result.xMax - result.xMin).toBeCloseTo(MIN_VIEW_SPAN);
    expect(result.yMax - result.yMin).toBeCloseTo(MIN_VIEW_SPAN);
  });
});
