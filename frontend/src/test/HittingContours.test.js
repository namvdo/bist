import { describe, expect, it } from 'vitest';
import {
  getHittingTargetContours,
  getPresentLevels,
  getVisibleHittingCells,
  normalizeHittingContourSettings,
  normalizeHittingContourState,
  pickHitForLevel
} from '../utils/hittingContours';

describe('hitting contour utilities', () => {
  it('normalizes user-facing settings into safe computation limits', () => {
    const normalized = normalizeHittingContourState({
      sampleGridSize: 999,
      thetaGridSize: 0,
      maxPeriod: 99,
      maxLevel: -3,
      supportMass: 2,
      hitTolerance: 0,
      residualThreshold: 5
    });

    expect(normalized.sampleGridSize).toBe(180);
    expect(normalized.thetaGridSize).toBe(1);
    expect(normalized.maxPeriod).toBe(10);
    expect(normalized.maxLevel).toBe(1);
    expect(normalized.supportMass).toBe(0.9999);
    expect(normalized.hitTolerance).toBe(1e-12);
    expect(normalized.residualThreshold).toBe(1e-2);
    expect(normalized.layoutMode).toBe('topographic');
    expect(normalizeHittingContourState({ layoutMode: 'spatial' }).layoutMode).toBe('spatial');
    expect(normalizeHittingContourState({}).hitTolerance).toBe(1e-2);
    expect(normalizeHittingContourState({ maxLevel: 99 }).maxLevel).toBe(10);
    expect(normalizeHittingContourState({ selectedLevels: [3, 1, 99, 3] }).selectedLevels).toEqual([1, 3]);
  });

  it('keeps computation settings separate from render state', () => {
    const settings = normalizeHittingContourSettings({
      showOverlay: true,
      result: { cells: [] },
      selectedLevel: 3,
      sampleGridSize: 64
    });

    expect(settings.sampleGridSize).toBe(64);
    expect(settings.showOverlay).toBeUndefined();
    expect(settings.result).toBeUndefined();
    expect(settings.selectedLevel).toBeUndefined();
    expect(settings.selectedLevels).toBeUndefined();
  });

  it('returns only positive levels present in the computation result', () => {
    expect(getPresentLevels({ levelsPresent: [0, 1, 3, -1, 2] })).toEqual([1, 3, 2]);
    expect(getPresentLevels(null)).toEqual([]);
  });

  it('picks earliest hit for all-level rendering and exact hit for a selected level', () => {
    const cell = {
      hits: [
        { level: 5, stability: 'stable' },
        { level: 2, stability: 'saddle' }
      ]
    };

    expect(pickHitForLevel(cell, 'all')).toEqual({ level: 2, stability: 'saddle' });
    expect(pickHitForLevel(cell, 5)).toEqual({ level: 5, stability: 'stable' });
    expect(pickHitForLevel(cell, [5, 2])).toEqual({ level: 2, stability: 'saddle' });
    expect(pickHitForLevel(cell, 4)).toBeNull();
  });

  it('filters visible cells by selected level', () => {
    const result = {
      cells: [
        { index: 0, hits: [{ level: 1 }] },
        { index: 1, hits: [{ level: 2 }] },
        { index: 2, hits: [] }
      ]
    };

    expect(getVisibleHittingCells(result, 2)).toEqual([
      { cell: result.cells[1], hit: { level: 2 } }
    ]);
    expect(getVisibleHittingCells(result, 'all')).toHaveLength(2);
  });

  it('groups hits into target-centered contour rings', () => {
    const result = {
      targets: [
        { targetIndex: 0, orbitIndex: 0, pointIndex: 0, stability: 'stable' },
        { targetIndex: 1, orbitIndex: 0, pointIndex: 1, stability: 'saddle' }
      ],
      cells: [
        { index: 0, x: 0.4, y: 0.2, hits: [{ targetIndex: 0, level: 1, distance: 0.02 }, { targetIndex: 1, level: 2, distance: 0.04 }] },
        { index: 1, x: 0.1, y: 0.2, hits: [{ targetIndex: 0, level: 3, distance: 0.03 }, { targetIndex: 0, level: 1, distance: 0.01 }] }
      ]
    };

    const contours = getHittingTargetContours(result, 'all');

    expect(contours).toHaveLength(2);
    expect(contours[0].target.targetIndex).toBe(0);
    expect(contours[0].bestLevel).toBe(1);
    expect(contours[0].totalHits).toBe(3);
    expect(contours[0].rings.map(ring => [ring.level, ring.count])).toEqual([[1, 2], [3, 1]]);
    expect(contours[0].rings[0].points).toEqual([
      expect.objectContaining({ level: 1, targetIndex: 0, distance: 0.01, orderIndex: 0 }),
      expect.objectContaining({ level: 1, targetIndex: 0, distance: 0.02, orderIndex: 1 })
    ]);
    expect(getHittingTargetContours(result, [1, 3])[0].rings.map(ring => ring.level)).toEqual([1, 3]);
    expect(getHittingTargetContours(result, 2)).toEqual([
      expect.objectContaining({
        target: result.targets[1],
        rings: [expect.objectContaining({ level: 2, count: 1 })]
      })
    ]);
  });
});
