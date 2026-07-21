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
 * Select a closed lifted MIS curve for the deterministic extended map.
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

  const stride = Math.max(1, Math.ceil(selected.length / maxPoints));
  return selected
    .filter((_, index) => index % stride === 0)
    .map(([x, y, nx, ny]) => ({ x, y, nx, ny }));
};

export { cleanExtendedPoints, completeNegativePhase, deriveExtendedBoundary };
