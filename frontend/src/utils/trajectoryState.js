export const shouldRecordTrajectoryHistoryPoint = ({ isContinuous = false, iteration = 0 } = {}) => {
  if (isContinuous) return true;
  return Number.isFinite(iteration) && iteration > 0;
};

export const appendTrajectoryHistoryPoint = ({
  points = [],
  point,
  iteration = 0,
  isContinuous = false,
  maxHistory = null
} = {}) => {
  const nextPoints = Array.isArray(points) ? [...points] : [];
  if (!point || !shouldRecordTrajectoryHistoryPoint({ isContinuous, iteration })) {
    return nextPoints;
  }

  nextPoints.push(point);
  if (isContinuous && Number.isFinite(maxHistory) && maxHistory > 0 && nextPoints.length > maxHistory) {
    return nextPoints.slice(nextPoints.length - maxHistory);
  }
  return nextPoints;
};
