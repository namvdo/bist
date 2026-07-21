export const RANGE_LIMIT = 10;
export const MIN_VIEW_SPAN = 0.05;
export const ZOOM_IN_FACTOR = 0.8;
export const ZOOM_OUT_FACTOR = 1.25;

export const DEFAULT_VIEW_RANGE = {
  xMin: -2,
  xMax: 2,
  yMin: -1.5,
  yMax: 1.5
};

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

export const normalizeViewRange = (range, limit = RANGE_LIMIT) => {
  let xMin = Number.isFinite(range.xMin) ? range.xMin : DEFAULT_VIEW_RANGE.xMin;
  let xMax = Number.isFinite(range.xMax) ? range.xMax : DEFAULT_VIEW_RANGE.xMax;
  let yMin = Number.isFinite(range.yMin) ? range.yMin : DEFAULT_VIEW_RANGE.yMin;
  let yMax = Number.isFinite(range.yMax) ? range.yMax : DEFAULT_VIEW_RANGE.yMax;

  let loX = Math.min(xMin, xMax);
  let hiX = Math.max(xMin, xMax);
  let loY = Math.min(yMin, yMax);
  let hiY = Math.max(yMin, yMax);

  loX = clamp(loX, -limit, limit);
  hiX = clamp(hiX, -limit, limit);
  loY = clamp(loY, -limit, limit);
  hiY = clamp(hiY, -limit, limit);

  if (Math.abs(hiX - loX) < 1e-6) {
    const center = (hiX + loX) / 2;
    loX = clamp(center - 1, -limit, limit);
    hiX = clamp(center + 1, -limit, limit);
  }

  if (Math.abs(hiY - loY) < 1e-6) {
    const center = (hiY + loY) / 2;
    loY = clamp(center - 1, -limit, limit);
    hiY = clamp(center + 1, -limit, limit);
  }

  return { xMin: loX, xMax: hiX, yMin: loY, yMax: hiY };
};

const zoomAxis = (min, max, factor, limit, minSpan) => {
  const center = (min + max) / 2;
  const span = clamp((max - min) * factor, minSpan, limit * 2);
  let low = center - span / 2;
  let high = center + span / 2;

  if (low < -limit) {
    high += -limit - low;
    low = -limit;
  }
  if (high > limit) {
    low -= high - limit;
    high = limit;
  }

  return [clamp(low, -limit, limit), clamp(high, -limit, limit)];
};

export const zoomViewRange = (
  range,
  factor,
  limit = RANGE_LIMIT,
  minSpan = MIN_VIEW_SPAN
) => {
  if (!Number.isFinite(factor) || factor <= 0) {
    return normalizeViewRange(range, limit);
  }

  const normalized = normalizeViewRange(range, limit);
  const [xMin, xMax] = zoomAxis(normalized.xMin, normalized.xMax, factor, limit, minSpan);
  const [yMin, yMax] = zoomAxis(normalized.yMin, normalized.yMax, factor, limit, minSpan);
  return { xMin, xMax, yMin, yMax };
};
