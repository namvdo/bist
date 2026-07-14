const MIS_SUPPORT_THRESHOLD = 1e-10;
const MIS_FILTER_SUBDIVISIONS = 64;
const MIS_FILTER_POINTS_PER_BOX = 64;

let wasmPromise = null;
let cachedUlamComputer = null;
let cachedPeriodicComputation = null;

const ensureWasm = async () => {
  if (!wasmPromise) {
    wasmPromise = import('../pkg/henon_periodic_orbits.js').then(async (mod) => {
      await mod.default();
      return mod;
    });
  }
  return wasmPromise;
};

const cleanupCachedUlamComputer = () => {
  if (cachedUlamComputer && typeof cachedUlamComputer.free === 'function') {
    cachedUlamComputer.free();
  }
  cachedUlamComputer = null;
};

const getSupportIndex = (x, y, support) => {
  if (!support) return -1;
  const { xMin, xMax, yMin, yMax, subdivisions } = support;
  if (x < xMin || x > xMax || y < yMin || y > yMax) return -1;

  const dx = (xMax - xMin) / subdivisions;
  const dy = (yMax - yMin) / subdivisions;
  if (!Number.isFinite(dx) || !Number.isFinite(dy) || dx <= 0 || dy <= 0) {
    return -1;
  }

  let ix = Math.floor((x - xMin) / dx);
  let iy = Math.floor((y - yMin) / dy);
  if (ix >= subdivisions) ix -= 1;
  if (iy >= subdivisions) iy -= 1;
  if (ix < 0 || iy < 0) return -1;
  return iy * subdivisions + ix;
};

const isSupportedPoint = (x, y, support) => {
  if (!support) return true;
  const idx = getSupportIndex(x, y, support);
  if (idx < 0) return false;
  return (support.invariantMeasure?.[idx] ?? 0) > support.threshold;
};

const filterOrbitsBySupport = (orbits, support) => {
  return (orbits || []).filter((orbit) =>
    (orbit.points || []).every(([x, y]) => isSupportedPoint(x, y, support))
  );
};

const sameNumber = (a, b) => Math.abs((a ?? 0) - (b ?? 0)) < 1e-12;

const sameViewRange = (a, b) => (
  sameNumber(a?.xMin, b?.xMin)
  && sameNumber(a?.xMax, b?.xMax)
  && sameNumber(a?.yMin, b?.yMin)
  && sameNumber(a?.yMax, b?.yMax)
);

const samePeriodicSearchSettings = (a, b) => (
  sameNumber(a?.gridSize, b?.gridSize)
  && sameNumber(a?.thetaGridSize, b?.thetaGridSize)
  && sameNumber(a?.residualThreshold, b?.residualThreshold)
);

const canContinueHenonPeriodic = (wasm, payload) => {
  const previous = cachedPeriodicComputation;
  return (
    payload.dynamicSystem === 'henon'
    && payload.periodicSearchSettings?.useContinuation === true
    && previous?.dynamicSystem === 'henon'
    && typeof wasm.continueBoundaryHenonOrbits === 'function'
    && (previous.allOrbits || []).length > 0
    && previous.params?.maxPeriod === payload.params?.maxPeriod
    && sameViewRange(previous.viewRange, payload.viewRange)
    && samePeriodicSearchSettings(previous.periodicSearchSettings, payload.periodicSearchSettings)
  );
};

const describePeriodicContinuationSkip = (wasm, payload) => {
  const previous = cachedPeriodicComputation;
  if (payload.dynamicSystem !== 'henon') return 'system is not Hénon';
  if (payload.periodicSearchSettings?.useContinuation !== true) return 'continuation is disabled';
  if (!previous) return 'no previous Hénon orbit cache';
  if (previous.dynamicSystem !== 'henon') return 'previous cache is not Hénon';
  if (typeof wasm.continueBoundaryHenonOrbits !== 'function') return 'WASM continuation export is unavailable';
  if ((previous.allOrbits || []).length === 0) return 'previous orbit cache is empty';
  if (previous.params?.maxPeriod !== payload.params?.maxPeriod) return 'max period changed';
  if (!sameViewRange(previous.viewRange, payload.viewRange)) return 'view range changed';
  if (!samePeriodicSearchSettings(previous.periodicSearchSettings, payload.periodicSearchSettings)) {
    return 'periodic search settings changed';
  }
  return null;
};

const computePeriodic = async (payload) => {
  const wasm = await ensureWasm();
  const { dynamicSystem, params, viewRange, periodicSearchSettings } = payload;

  if (dynamicSystem === 'custom' || dynamicSystem === 'custom_ode') {
    return { orbits: [], support: null };
  }

  let system = null;
  let supportComputer = null;

  try {
    let allOrbits = null;
    let usedContinuation = false;

    if (canContinueHenonPeriodic(wasm, payload)) {
      try {
        const previous = cachedPeriodicComputation;
        const continued = wasm.continueBoundaryHenonOrbits(
          previous.allOrbits,
          previous.params.a,
          previous.params.b,
          previous.params.epsilon,
          params.a,
          params.b,
          params.epsilon,
          params.maxPeriod,
          periodicSearchSettings.residualThreshold
        );
        if ((continued || []).length > 0) {
          allOrbits = continued;
          usedContinuation = true;
          console.log(
            `Periodic orbits: used continuation from a=${previous.params.a}, b=${previous.params.b}, ε=${previous.params.epsilon} to a=${params.a}, b=${params.b}, ε=${params.epsilon}`
          );
        }
      } catch (err) {
        console.warn('Periodic orbit continuation failed; falling back to full search.', err);
      }
    } else if (dynamicSystem === 'henon') {
      console.log(`Periodic orbits: running full grid search (${describePeriodicContinuationSkip(wasm, payload)})`);
    }

    if (!allOrbits && dynamicSystem === 'duffing') {
      system = new wasm.DuffingSystemWasm(params.a, params.b, params.maxPeriod);
    } else if (dynamicSystem === 'duffing_ode') {
      system = new wasm.EulerMapSystemWasm(
        params.delta,
        params.h,
        params.epsilon,
        params.maxPeriod
      );
    } else if (!allOrbits) {
      system = new wasm.BoundaryHenonSystemWasm(
        params.a,
        params.b,
        params.epsilon,
        params.maxPeriod,
        viewRange.xMin,
        viewRange.xMax,
        viewRange.yMin,
        viewRange.yMax,
        periodicSearchSettings.gridSize,
        periodicSearchSettings.thetaGridSize,
        periodicSearchSettings.residualThreshold
      );
    }

    if (!allOrbits) {
      allOrbits = system.getPeriodicOrbits();
    }

    let orbits = allOrbits;
    let support = null;

    if (dynamicSystem === 'henon') {
      supportComputer = new wasm.UlamComputer(
        params.a,
        params.b,
        MIS_FILTER_SUBDIVISIONS,
        MIS_FILTER_POINTS_PER_BOX,
        params.epsilon,
        viewRange.xMin,
        viewRange.xMax,
        viewRange.yMin,
        viewRange.yMax
      );

      support = {
        invariantMeasure: supportComputer.get_invariant_measure(),
        subdivisions: MIS_FILTER_SUBDIVISIONS,
        xMin: viewRange.xMin,
        xMax: viewRange.xMax,
        yMin: viewRange.yMin,
        yMax: viewRange.yMax,
        threshold: MIS_SUPPORT_THRESHOLD
      };

      orbits = filterOrbitsBySupport(orbits, support);
    }

    if (dynamicSystem === 'henon') {
      cachedPeriodicComputation = {
        dynamicSystem,
        params: {
          a: params.a,
          b: params.b,
          epsilon: params.epsilon,
          maxPeriod: params.maxPeriod
        },
        viewRange: { ...viewRange },
        periodicSearchSettings: {
          gridSize: periodicSearchSettings.gridSize,
          thetaGridSize: periodicSearchSettings.thetaGridSize,
          residualThreshold: periodicSearchSettings.residualThreshold
        },
        allOrbits
      };
    } else {
      cachedPeriodicComputation = null;
    }

    return { orbits, support, usedContinuation };
  } finally {
    if (supportComputer && typeof supportComputer.free === 'function') {
      supportComputer.free();
    }
    if (system && typeof system.free === 'function') {
      system.free();
    }
  }
};

const computeManifolds = async (payload) => {
  const wasm = await ensureWasm();
  const {
    dynamicSystem,
    params,
    viewRange,
    periodicOrbits,
    customEquations,
    customParams,
    showStableManifold,
    showUnstableManifold,
    intersectionThreshold
  } = payload;

  if (dynamicSystem === 'duffing') {
    const result = wasm.compute_duffing_manifold_simple(
      params.a,
      params.b,
      params.epsilon,
      viewRange.xMin,
      viewRange.xMax,
      viewRange.yMin,
      viewRange.yMax
    );
    return {
      manifolds: result.manifolds || [],
      stableManifolds: [],
      fixedPoints: result.fixed_points || [],
      intersections: []
    };
  }

  if (dynamicSystem === 'custom') {
    if ((periodicOrbits || []).length > 0) {
      if (showStableManifold || showUnstableManifold) {
        const result = wasm.compute_stable_and_unstable_manifolds_user_defined(
          customEquations.xEq,
          customEquations.yEq,
          customParams,
          params.epsilon,
          viewRange.xMin,
          viewRange.xMax,
          viewRange.yMin,
          viewRange.yMax,
          periodicOrbits,
          intersectionThreshold
        );
        return {
          manifolds: result.unstable_manifolds || [],
          stableManifolds: result.stable_manifolds || [],
          fixedPoints: result.fixed_points || [],
          intersections: result.intersections || []
        };
      }
      return {
        manifolds: [],
        stableManifolds: [],
        fixedPoints: [],
        intersections: []
      };
    }

    const result = wasm.compute_user_defined_manifold(
      customEquations.xEq,
      customEquations.yEq,
      customParams,
      params.epsilon,
      viewRange.xMin,
      viewRange.xMax,
      viewRange.yMin,
      viewRange.yMax
    );

    return {
      manifolds: result.manifolds || [],
      stableManifolds: [],
      fixedPoints: result.fixed_points || [],
      intersections: []
    };
  }

  if ((periodicOrbits || []).length > 0) {
    if (showStableManifold || showUnstableManifold) {
      const result = wasm.compute_stable_and_unstable_manifolds(
        params.a,
        params.b,
        params.epsilon,
        viewRange.xMin,
        viewRange.xMax,
        viewRange.yMin,
        viewRange.yMax,
        periodicOrbits,
        intersectionThreshold
      );
      return {
        manifolds: result.unstable_manifolds || [],
        stableManifolds: result.stable_manifolds || [],
        fixedPoints: result.fixed_points || [],
        intersections: result.intersections || []
      };
    }
    return {
      manifolds: [],
      stableManifolds: [],
      fixedPoints: [],
      intersections: []
    };
  }

  const result = wasm.compute_manifold_simple(
    params.a,
    params.b,
    params.epsilon,
    viewRange.xMin,
    viewRange.xMax,
    viewRange.yMin,
    viewRange.yMax
  );

  return {
    manifolds: result.manifolds || [],
    stableManifolds: [],
    fixedPoints: result.fixed_points || [],
    intersections: []
  };
};

const buildUlamComputer = (wasm, payload) => {
  const {
    dynamicSystem,
    params,
    viewRange,
    ulam,
    customEquations,
    customParams
  } = payload;

  if (dynamicSystem === 'custom') {
    return new wasm.UlamComputerUserDefined(
      customEquations.xEq,
      customEquations.yEq,
      customParams,
      ulam.subdivisions,
      ulam.pointsPerBox,
      ulam.epsilon,
      viewRange.xMin,
      viewRange.xMax,
      viewRange.yMin,
      viewRange.yMax
    );
  }

  if (dynamicSystem === 'custom_ode') {
    const capitalT = Math.max(params.h * 10, 0.5);
    return new wasm.UlamComputerContinuousUserDefined(
      customEquations.xEq,
      customEquations.yEq,
      customParams,
      capitalT,
      ulam.subdivisions,
      ulam.pointsPerBox,
      ulam.epsilon,
      viewRange.xMin,
      viewRange.xMax,
      viewRange.yMin,
      viewRange.yMax
    );
  }

  if (dynamicSystem === 'duffing_ode') {
    const capitalT = Math.max(params.h * 10, 0.5);
    return new wasm.UlamComputerContinuous(
      params.delta,
      capitalT,
      ulam.subdivisions,
      ulam.pointsPerBox,
      ulam.epsilon,
      viewRange.xMin,
      viewRange.xMax,
      viewRange.yMin,
      viewRange.yMax
    );
  }

  return new wasm.UlamComputer(
    params.a,
    params.b,
    ulam.subdivisions,
    ulam.pointsPerBox,
    ulam.epsilon,
    viewRange.xMin,
    viewRange.xMax,
    viewRange.yMin,
    viewRange.yMax
  );
};

const computeUlam = async (payload) => {
  const wasm = await ensureWasm();
  cleanupCachedUlamComputer();
  cachedUlamComputer = buildUlamComputer(wasm, payload);

  const boxes = cachedUlamComputer.get_grid_boxes();
  const invariantMeasure = cachedUlamComputer.get_invariant_measure();
  const leftEigenvector = cachedUlamComputer.get_left_eigenvector();

  let currentBoxIndex = -1;
  if (payload.currentPoint) {
    currentBoxIndex = cachedUlamComputer.get_box_index(
      payload.currentPoint.x,
      payload.currentPoint.y
    );
  }

  return {
    boxes,
    invariantMeasure,
    leftEigenvector,
    currentBoxIndex
  };
};

const getUlamTransitions = async (payload) => {
  if (!cachedUlamComputer) {
    return [];
  }
  return cachedUlamComputer.get_transitions(payload.index) || [];
};

const computeHittingContours = async (payload) => {
  const wasm = await ensureWasm();
  const { params, viewRange, settings } = payload;

  if (typeof wasm.computeHenonHittingLevelSets !== 'function') {
    throw new Error('Hénon hitting-level contour export is unavailable');
  }

  return wasm.computeHenonHittingLevelSets(
    params.a,
    params.b,
    params.epsilon,
    viewRange.xMin,
    viewRange.xMax,
    viewRange.yMin,
    viewRange.yMax,
    settings.maxPeriod,
    settings.ulamSubdivisions,
    settings.ulamPointsPerBox,
    settings.ulamIterations,
    settings.supportMass,
    settings.thetaGridSize,
    settings.sampleGridSize,
    settings.maxLevel,
    settings.hitTolerance,
    settings.residualThreshold
  );
};

self.onmessage = async (event) => {
  const { id, kind, payload } = event.data || {};
  if (!kind) return;

  try {
    let result = null;
    if (kind === 'computePeriodic') {
      result = await computePeriodic(payload);
    } else if (kind === 'computeManifolds') {
      result = await computeManifolds(payload);
    } else if (kind === 'computeUlam') {
      result = await computeUlam(payload);
    } else if (kind === 'computeHittingContours') {
      result = await computeHittingContours(payload);
    } else if (kind === 'getUlamTransitions') {
      result = await getUlamTransitions(payload);
    } else {
      throw new Error(`Unknown worker task: ${kind}`);
    }

    self.postMessage({ id, ok: true, kind, result });
  } catch (err) {
    self.postMessage({
      id,
      ok: false,
      kind,
      error: err instanceof Error ? err.message : String(err)
    });
  }
};
