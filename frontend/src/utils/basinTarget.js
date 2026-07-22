import {
  buildGeometricOffsetSeed,
  forwardBoundaryPoint,
  isClosedCandidate,
  joinClosedBranches,
  signedArea
} from './geometricOffsetSeed.js';

const cleanExtendedPoints = (points) => (points || [])
  .filter(point => Array.isArray(point) && point.length >= 4 && point.slice(0, 4).every(Number.isFinite))
  .map(([x, y, nx, ny]) => {
    const length = Math.hypot(nx, ny);
    return length > 1e-12 ? [x, y, nx / length, ny / length] : null;
  })
  .filter(Boolean);

const positions = points => points.map(([x, y]) => [x, y]);

const circularDistance = (left, right) => {
  const period = 2 * Math.PI;
  const difference = Math.abs(((left - right) % period + period) % period);
  return Math.min(difference, period - difference);
};

const median = values => {
  if (values.length === 0) return Infinity;
  const ordered = [...values].sort((left, right) => left - right);
  const middle = Math.floor(ordered.length / 2);
  return ordered.length % 2 === 0
    ? 0.5 * (ordered[middle - 1] + ordered[middle])
    : ordered[middle];
};

const targetNeedsGeometricRepair = points => {
  if (points.length < 3) return true;
  const lengths = points.map((point, index) => {
    const next = points[(index + 1) % points.length];
    return Math.hypot(next[0] - point[0], next[1] - point[1]);
  });
  const typicalSpacing = median(lengths.filter(length => length > 1e-12));
  const hasSpacingGap = !Number.isFinite(typicalSpacing)
    || Math.max(...lengths) > Math.max(8 * typicalSpacing, 1e-10);
  const hasNormalJump = points.some((point, index) => {
    const next = points[(index + 1) % points.length];
    return circularDistance(Math.atan2(point[3], point[2]), Math.atan2(next[3], next[2])) > Math.PI / 2;
  });
  return hasSpacingGap || hasNormalJump;
};

const resampleClosedBoundary = (boundary, pointCount) => {
  if (boundary.length < 3 || pointCount < 3) return boundary;
  const segments = boundary.map((point, index) => {
    const next = boundary[(index + 1) % boundary.length];
    return { point, next, length: Math.hypot(next[0] - point[0], next[1] - point[1]) };
  }).filter(segment => segment.length > 1e-12);
  const perimeter = segments.reduce((sum, segment) => sum + segment.length, 0);
  if (!Number.isFinite(perimeter) || perimeter <= 1e-12) return boundary;

  const result = [];
  let segmentIndex = 0;
  let segmentStart = 0;
  for (let index = 0; index < pointCount; index += 1) {
    const targetDistance = perimeter * index / pointCount;
    while (segmentIndex + 1 < segments.length
      && segmentStart + segments[segmentIndex].length < targetDistance) {
      segmentStart += segments[segmentIndex].length;
      segmentIndex += 1;
    }
    const segment = segments[segmentIndex];
    const fraction = Math.max(0, Math.min(1, (targetDistance - segmentStart) / segment.length));
    result.push([
      segment.point[0] + fraction * (segment.next[0] - segment.point[0]),
      segment.point[1] + fraction * (segment.next[1] - segment.point[1])
    ]);
  }
  return result;
};

const resampleExtendedBoundary = (boundary, pointCount) => {
  if (boundary.length < 3 || pointCount < 3) return boundary;
  const positionsOnly = positions(boundary);
  const resampledPositions = resampleClosedBoundary(positionsOnly, pointCount);
  const segments = boundary.map((point, index) => {
    const next = boundary[(index + 1) % boundary.length];
    return { point, next, length: Math.hypot(next[0] - point[0], next[1] - point[1]) };
  }).filter(segment => segment.length > 1e-12);
  const perimeter = segments.reduce((sum, segment) => sum + segment.length, 0);
  if (!Number.isFinite(perimeter) || perimeter <= 1e-12) return boundary;

  const result = [];
  let segmentIndex = 0;
  let segmentStart = 0;
  for (let index = 0; index < pointCount; index += 1) {
    const targetDistance = perimeter * index / pointCount;
    while (segmentIndex + 1 < segments.length
      && segmentStart + segments[segmentIndex].length < targetDistance) {
      segmentStart += segments[segmentIndex].length;
      segmentIndex += 1;
    }
    const segment = segments[segmentIndex];
    const fraction = Math.max(0, Math.min(1, (targetDistance - segmentStart) / segment.length));
    const theta = Math.atan2(segment.point[3], segment.point[2]);
    const nextTheta = Math.atan2(segment.next[3], segment.next[2]);
    const angularIncrement = Math.atan2(Math.sin(nextTheta - theta), Math.cos(nextTheta - theta));
    const interpolatedTheta = theta + fraction * angularIncrement;
    result.push([
      resampledPositions[index][0],
      resampledPositions[index][1],
      Math.cos(interpolatedTheta),
      Math.sin(interpolatedTheta)
    ]);
  }
  return result;
};

const completeNegativePhase = (points, eigenvalue, params) => {
  if (!(eigenvalue < 0) || points.length < 3) return null;
  const image = points.map(point => forwardBoundaryPoint(point, params));
  if (image.some(point => point === null)) return null;
  return [...points, ...image.reverse()];
};

const deriveExtendedBoundary = (boundary) => {
  if (!Array.isArray(boundary) || boundary.length < 3) return [];
  const area = signedArea(boundary);
  return boundary.map(([x, y], index) => {
    const previous = boundary[(index + boundary.length - 1) % boundary.length];
    const next = boundary[(index + 1) % boundary.length];
    const tx = next[0] - previous[0];
    const ty = next[1] - previous[1];
    const length = Math.hypot(tx, ty);
    if (length < 1e-12) return null;
    const nx = area >= 0 ? ty / length : -ty / length;
    const ny = area >= 0 ? -tx / length : tx / length;
    return [x, y, nx, ny];
  }).filter(Boolean);
};

/**
 * Select a closed MIS unit-normal-bundle curve for the deterministic extended map.
 * Actual manifold normals are preferred; geometric curve normals are a
 * deterministic fallback when an older manifold result has positions only.
 */
export const buildBasinTarget = (manifolds, maxPoints = 2000, params = {}) => {
  const candidates = [];
  for (const manifold of manifolds || []) {
    const plus = cleanExtendedPoints(manifold?.plus?.extended_points);
    const minus = cleanExtendedPoints(manifold?.minus?.extended_points);
    if (plus.length >= 3 && isClosedCandidate(positions(plus))) candidates.push(plus);
    if (minus.length >= 3 && isClosedCandidate(positions(minus))) candidates.push(minus);

    if (plus.length >= 2 && minus.length >= 2 && joinClosedBranches(positions(plus), positions(minus))) {
      candidates.push([...plus, ...minus.slice().reverse()]);
    }
    const plusPhase = completeNegativePhase(plus, manifold?.eigenvalue, params);
    const minusPhase = completeNegativePhase(minus, manifold?.eigenvalue, params);
    if (plusPhase && isClosedCandidate(positions(plusPhase))) candidates.push(plusPhase);
    if (minusPhase && isClosedCandidate(positions(minusPhase))) candidates.push(minusPhase);
  }

  let selected = candidates
    .filter(candidate => Math.abs(signedArea(positions(candidate))) > 1e-10)
    .sort((left, right) => Math.abs(signedArea(positions(right))) - Math.abs(signedArea(positions(left))))[0];

  if (!selected) {
    selected = deriveExtendedBoundary(buildGeometricOffsetSeed(manifolds, maxPoints, params));
  }
  if (!selected || selected.length < 3) return [];

  if (targetNeedsGeometricRepair(selected)) {
    selected = resampleExtendedBoundary(selected, Math.min(maxPoints, Math.max(64, selected.length)));
  }

  const stride = Math.max(1, Math.ceil(selected.length / maxPoints));
  return selected
    .filter((_, index) => index % stride === 0)
    .map(([x, y, nx, ny]) => ({ x, y, nx, ny }));
};

export {
  cleanExtendedPoints,
  completeNegativePhase,
  deriveExtendedBoundary,
  resampleClosedBoundary,
  resampleExtendedBoundary,
  targetNeedsGeometricRepair
};
