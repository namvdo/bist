const signedArea = (points) => {
  if (!Array.isArray(points) || points.length < 3) return 0;
  let area = 0;
  for (let i = 0; i < points.length; i += 1) {
    const [x1, y1] = points[i];
    const [x2, y2] = points[(i + 1) % points.length];
    area += x1 * y2 - x2 * y1;
  }
  return area * 0.5;
};

const cleanPoints = (points) => (points || []).filter(
  point => Array.isArray(point) && point.length >= 2 && Number.isFinite(point[0]) && Number.isFinite(point[1])
);

const distance = (left, right) => Math.hypot(left[0] - right[0], left[1] - right[1]);

const closureTolerance = (points) => {
  if (points.length < 3) return 0;
  const openSegmentLengths = points.slice(1)
    .map((point, index) => distance(points[index], point))
    .filter(length => length > 1e-12)
    .sort((left, right) => left - right);
  if (openSegmentLengths.length < 2) return 0;
  const median = openSegmentLengths[Math.floor(openSegmentLengths.length / 2)];
  let xMin = points[0][0];
  let xMax = points[0][0];
  let yMin = points[0][1];
  let yMax = points[0][1];
  for (const [x, y] of points) {
    xMin = Math.min(xMin, x);
    xMax = Math.max(xMax, x);
    yMin = Math.min(yMin, y);
    yMax = Math.max(yMax, y);
  }
  const diameter = Math.hypot(xMax - xMin, yMax - yMin);
  return Math.max(8 * median, 0.08 * diameter, 1e-10);
};

const isClosedCandidate = (points) => {
  if (points.length < 3) return false;
  return distance(points[0], points[points.length - 1]) <= closureTolerance(points);
};

const joinClosedBranches = (plus, minus) => {
  if (plus.length < 2 || minus.length < 2) return null;
  const joined = [...plus, ...minus.slice().reverse()];
  const tolerance = closureTolerance(joined);
  const initialSeam = distance(plus[0], minus[0]);
  const terminalSeam = distance(plus[plus.length - 1], minus[minus.length - 1]);
  return initialSeam <= tolerance && terminalSeam <= tolerance ? joined : null;
};

const cleanExtendedPoints = (points) => (points || []).filter(
  point => Array.isArray(point) && point.length >= 4 && point.slice(0, 4).every(Number.isFinite)
);

const forwardBoundaryPoint = ([x, y, nx, ny], { a, b, epsilon }) => {
  if (![a, b, epsilon].every(Number.isFinite) || Math.abs(b) < 1e-12) return null;
  let nextNx = ny;
  let nextNy = nx / b + (2 * a * x * ny) / b;
  const normalLength = Math.hypot(nextNx, nextNy);
  if (!Number.isFinite(normalLength) || normalLength < 1e-12) return null;
  nextNx /= normalLength;
  nextNy /= normalLength;
  return [
    1 - a * x * x + y + epsilon * nextNx,
    b * x + epsilon * nextNy,
    nextNx,
    nextNy
  ];
};

const completeNegativeMultiplierPhase = (trajectory, eigenvalue, params) => {
  if (!(eigenvalue < 0)) return null;
  const extended = cleanExtendedPoints(trajectory?.extended_points);
  if (extended.length < 3) return null;
  const image = extended.map(point => forwardBoundaryPoint(point, params));
  if (image.some(point => point === null)) return null;
  return [
    ...extended.map(([x, y]) => [x, y]),
    ...image.reverse().map(([x, y]) => [x, y])
  ];
};

export const buildGeometricOffsetSeed = (manifolds, maxPoints = 4000, params = {}) => {
  const candidates = [];
  for (const manifold of manifolds || []) {
    const plus = cleanPoints(manifold?.plus?.points);
    const minus = cleanPoints(manifold?.minus?.points);
    if (plus.length >= 3) candidates.push(plus);
    if (minus.length >= 3) candidates.push(minus);
    const joined = joinClosedBranches(plus, minus);
    if (joined) candidates.push(joined);
    const plusPhaseBoundary = completeNegativeMultiplierPhase(manifold?.plus, manifold?.eigenvalue, params);
    const minusPhaseBoundary = completeNegativeMultiplierPhase(manifold?.minus, manifold?.eigenvalue, params);
    if (plusPhaseBoundary) candidates.push(plusPhaseBoundary);
    if (minusPhaseBoundary) candidates.push(minusPhaseBoundary);
  }

  const seed = candidates
    .filter(points => points.length >= 3 && isClosedCandidate(points))
    .sort((left, right) => Math.abs(signedArea(right)) - Math.abs(signedArea(left)))[0];

  if (!seed || Math.abs(signedArea(seed)) < 1e-10) return [];
  const stride = Math.max(1, Math.ceil(seed.length / maxPoints));
  return seed.filter((_, index) => index % stride === 0);
};

export { completeNegativeMultiplierPhase, forwardBoundaryPoint, isClosedCandidate, joinClosedBranches, signedArea };
