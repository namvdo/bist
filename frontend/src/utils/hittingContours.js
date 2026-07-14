export const DEFAULT_HITTING_CONTOUR_STATE = {
  showOverlay: false,
  isComputing: false,
  result: null,
  selectedLevel: 'all',
  selectedLevels: [],
  layoutMode: 'topographic',
  showLevelRings: true,
  sampleGridSize: 60,
  thetaGridSize: 8,
  maxPeriod: 10,
  maxLevel: 10,
  ulamSubdivisions: 40,
  ulamPointsPerBox: 64,
  ulamIterations: 20,
  supportMass: 0.995,
  hitTolerance: 1e-2,
  residualThreshold: 1e-10,
  error: null
};

export const HITTING_CONTOUR_LIMITS = {
  sampleGridMin: 10,
  sampleGridMax: 180,
  thetaGridMin: 1,
  thetaGridMax: 64,
  maxPeriodMin: 1,
  maxPeriodMax: 10,
  maxLevelMin: 1,
  maxLevelMax: 10,
  ulamSubdivisionsMin: 10,
  ulamSubdivisionsMax: 180,
  ulamPointsPerBoxMin: 4,
  ulamPointsPerBoxMax: 256,
  supportMassMin: 0.8,
  supportMassMax: 0.9999,
  hitToleranceMin: 1e-12,
  hitToleranceMax: 1e-1,
  residualThresholdMin: 1e-14,
  residualThresholdMax: 1e-2
};

export const HITTING_CONTOUR_SETTING_KEYS = [
  'sampleGridSize',
  'thetaGridSize',
  'maxPeriod',
  'maxLevel',
  'ulamSubdivisions',
  'ulamPointsPerBox',
  'ulamIterations',
  'supportMass',
  'hitTolerance',
  'residualThreshold'
];

const clampInteger = (value, min, max, fallback) => {
  const parsed = Number.parseInt(`${value}`, 10);
  if (!Number.isFinite(parsed)) return fallback;
  return Math.min(max, Math.max(min, parsed));
};

const clampNumber = (value, min, max, fallback) => {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) return fallback;
  return Math.min(max, Math.max(min, parsed));
};

const normalizeSelectedLevels = (value) => {
  const source = Array.isArray(value) ? value : [];
  const unique = new Set();
  source.forEach((level) => {
    const parsed = Number.parseInt(`${level}`, 10);
    if (Number.isInteger(parsed) && parsed >= HITTING_CONTOUR_LIMITS.maxLevelMin && parsed <= HITTING_CONTOUR_LIMITS.maxLevelMax) {
      unique.add(parsed);
    }
  });
  return [...unique].sort((a, b) => a - b);
};

const selectionToSet = (selectedLevel = 'all') => {
  if (Array.isArray(selectedLevel)) {
    const levels = normalizeSelectedLevels(selectedLevel);
    return levels.length > 0 ? new Set(levels) : null;
  }
  if (selectedLevel === 'all' || selectedLevel == null) return null;
  const parsed = Number.parseInt(`${selectedLevel}`, 10);
  return Number.isInteger(parsed) ? new Set([parsed]) : null;
};

const orderPointsByProjection = (points) => {
  if (!Array.isArray(points) || points.length <= 1) {
    return (points || []).map((point, orderIndex) => ({ ...point, orderIndex, projection: 0 }));
  }

  const mean = points.reduce(
    (acc, point) => ({ x: acc.x + point.x, y: acc.y + point.y }),
    { x: 0, y: 0 }
  );
  mean.x /= points.length;
  mean.y /= points.length;

  const covariance = points.reduce((acc, point) => {
    const dx = point.x - mean.x;
    const dy = point.y - mean.y;
    return {
      xx: acc.xx + dx * dx,
      xy: acc.xy + dx * dy,
      yy: acc.yy + dy * dy
    };
  }, { xx: 0, xy: 0, yy: 0 });

  const angle = 0.5 * Math.atan2(2 * covariance.xy, covariance.xx - covariance.yy);
  const ux = Math.cos(angle);
  const uy = Math.sin(angle);

  return points
    .map((point) => ({
      ...point,
      projection: (point.x - mean.x) * ux + (point.y - mean.y) * uy
    }))
    .sort((a, b) => (
      a.projection - b.projection
      || a.y - b.y
      || a.x - b.x
      || (a.cellIndex ?? 0) - (b.cellIndex ?? 0)
    ))
    .map((point, orderIndex) => ({ ...point, orderIndex }));
};

export const normalizeHittingContourState = (next, fallback = DEFAULT_HITTING_CONTOUR_STATE) => {
  const safe = fallback || DEFAULT_HITTING_CONTOUR_STATE;
  const selectedLevelsSource = next?.selectedLevels
    ?? (Number.isInteger(next?.selectedLevel) ? [next.selectedLevel] : safe.selectedLevels);
  const selectedLevels = normalizeSelectedLevels(selectedLevelsSource);
  return {
    ...safe,
    ...next,
    selectedLevel: selectedLevels.length > 0 ? selectedLevels[0] : 'all',
    selectedLevels,
    layoutMode: next?.layoutMode === 'spatial' ? 'spatial' : 'topographic',
    showLevelRings: next?.showLevelRings ?? safe.showLevelRings ?? true,
    sampleGridSize: clampInteger(next?.sampleGridSize ?? safe.sampleGridSize, HITTING_CONTOUR_LIMITS.sampleGridMin, HITTING_CONTOUR_LIMITS.sampleGridMax, safe.sampleGridSize),
    thetaGridSize: clampInteger(next?.thetaGridSize ?? safe.thetaGridSize, HITTING_CONTOUR_LIMITS.thetaGridMin, HITTING_CONTOUR_LIMITS.thetaGridMax, safe.thetaGridSize),
    maxPeriod: clampInteger(next?.maxPeriod ?? safe.maxPeriod, HITTING_CONTOUR_LIMITS.maxPeriodMin, HITTING_CONTOUR_LIMITS.maxPeriodMax, safe.maxPeriod),
    maxLevel: clampInteger(next?.maxLevel ?? safe.maxLevel, HITTING_CONTOUR_LIMITS.maxLevelMin, HITTING_CONTOUR_LIMITS.maxLevelMax, safe.maxLevel),
    ulamSubdivisions: clampInteger(next?.ulamSubdivisions ?? safe.ulamSubdivisions, HITTING_CONTOUR_LIMITS.ulamSubdivisionsMin, HITTING_CONTOUR_LIMITS.ulamSubdivisionsMax, safe.ulamSubdivisions),
    ulamPointsPerBox: clampInteger(next?.ulamPointsPerBox ?? safe.ulamPointsPerBox, HITTING_CONTOUR_LIMITS.ulamPointsPerBoxMin, HITTING_CONTOUR_LIMITS.ulamPointsPerBoxMax, safe.ulamPointsPerBox),
    ulamIterations: clampInteger(next?.ulamIterations ?? safe.ulamIterations, 1, 100, safe.ulamIterations),
    supportMass: clampNumber(next?.supportMass ?? safe.supportMass, HITTING_CONTOUR_LIMITS.supportMassMin, HITTING_CONTOUR_LIMITS.supportMassMax, safe.supportMass),
    hitTolerance: clampNumber(next?.hitTolerance ?? safe.hitTolerance, HITTING_CONTOUR_LIMITS.hitToleranceMin, HITTING_CONTOUR_LIMITS.hitToleranceMax, safe.hitTolerance),
    residualThreshold: clampNumber(next?.residualThreshold ?? safe.residualThreshold, HITTING_CONTOUR_LIMITS.residualThresholdMin, HITTING_CONTOUR_LIMITS.residualThresholdMax, safe.residualThreshold)
  };
};

export const normalizeHittingContourSettings = (next, fallback = DEFAULT_HITTING_CONTOUR_STATE) => {
  const normalized = normalizeHittingContourState(next, fallback);
  return HITTING_CONTOUR_SETTING_KEYS.reduce((settings, key) => {
    settings[key] = normalized[key];
    return settings;
  }, {});
};

export const getPresentLevels = (result) => (
  Array.isArray(result?.levelsPresent) ? result.levelsPresent.filter((level) => Number.isInteger(level) && level > 0) : []
);

export const pickHitForLevel = (cell, selectedLevel = 'all') => {
  const hits = Array.isArray(cell?.hits) ? cell.hits : [];
  if (hits.length === 0) return null;
  const selected = selectionToSet(selectedLevel);
  const candidates = selected ? hits.filter((hit) => selected.has(hit.level)) : hits;
  return candidates.reduce((best, hit) => {
    if (!best) return hit;
    if ((hit.level ?? Infinity) < (best.level ?? Infinity)) return hit;
    return best;
  }, null);
};

export const getVisibleHittingCells = (result, selectedLevel = 'all') => {
  const cells = Array.isArray(result?.cells) ? result.cells : [];
  return cells
    .map((cell) => ({ cell, hit: pickHitForLevel(cell, selectedLevel) }))
    .filter((entry) => entry.hit);
};

export const getHittingTargetContours = (result, selectedLevel = 'all') => {
  const targets = Array.isArray(result?.targets) ? result.targets : [];
  const cells = Array.isArray(result?.cells) ? result.cells : [];
  const selected = selectionToSet(selectedLevel);
  const byTarget = new Map();

  targets.forEach((target) => {
    byTarget.set(target.targetIndex, {
      target,
      levelMap: new Map(),
      totalHits: 0,
      bestLevel: Infinity
    });
  });

  cells.forEach((cell) => {
    const hits = Array.isArray(cell?.hits) ? cell.hits : [];
    hits.forEach((hit) => {
      if (selected && !selected.has(hit.level)) return;
      const entry = byTarget.get(hit.targetIndex);
      if (!entry) return;
      const existing = entry.levelMap.get(hit.level) || {
        level: hit.level,
        count: 0,
        minDistance: Infinity,
        maxDistance: 0,
        points: []
      };
      existing.count += 1;
      existing.minDistance = Math.min(existing.minDistance, hit.distance ?? Infinity);
      existing.maxDistance = Math.max(existing.maxDistance, hit.distance ?? 0);
      existing.points.push({
        cellIndex: cell.index,
        x: cell.x,
        y: cell.y,
        level: hit.level,
        targetIndex: hit.targetIndex,
        orbitIndex: hit.orbitIndex,
        pointIndex: hit.pointIndex,
        period: hit.period,
        stability: hit.stability,
        distance: hit.distance
      });
      entry.levelMap.set(hit.level, existing);
      entry.totalHits += 1;
      entry.bestLevel = Math.min(entry.bestLevel, hit.level);
    });
  });

  return [...byTarget.values()]
    .map((entry) => ({
      target: entry.target,
      totalHits: entry.totalHits,
      bestLevel: Number.isFinite(entry.bestLevel) ? entry.bestLevel : null,
      rings: [...entry.levelMap.values()]
        .map((ring) => ({ ...ring, points: orderPointsByProjection(ring.points) }))
        .sort((a, b) => a.level - b.level)
    }))
    .filter((entry) => entry.totalHits > 0)
    .sort((a, b) => (
      (a.target.orbitIndex ?? 0) - (b.target.orbitIndex ?? 0)
      || (a.target.pointIndex ?? 0) - (b.target.pointIndex ?? 0)
      || (a.target.targetIndex ?? 0) - (b.target.targetIndex ?? 0)
    ));
};

export const getStabilityColorHex = (stability) => {
  const key = `${stability || ''}`.toLowerCase();
  if (key === 'stable') return 0x5a9668;
  if (key === 'unstable') return 0xa85252;
  if (key === 'saddle') return 0xb8904a;
  return 0x5b88b5;
};
