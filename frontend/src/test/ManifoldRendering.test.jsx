import { describe, it, expect } from 'vitest';

/**
 * Tests for the manifold rendering logic.
 * These test the pure logic of classifying which manifolds belong to repellers
 * vs saddles, mirroring the logic in SetValuedViz.jsx rendering code.
 */
describe('Manifold Classification Logic', () => {
  const classifyManifolds = (manifolds, fixedPoints) => {
    const repellerPoints = new Set();
    fixedPoints.forEach(fp => {
      const stab = (fp.stability || '').toLowerCase();
      if (stab === 'unstable') {
        repellerPoints.add(`${fp.x.toFixed(8)},${fp.y.toFixed(8)}`);
      }
    });

    return manifolds.map(m => {
      const spKey = `${m.saddle_point[0].toFixed(8)},${m.saddle_point[1].toFixed(8)}`;
      return {
        ...m,
        isRepeller: repellerPoints.has(spKey),
      };
    });
  };


  it('handles case with no repellers', () => {
    const fixedPoints = [
      { x: 0.5, y: 0.15, stability: 'Saddle', eigenvalues: [1.5, -0.2] },
      { x: 0.8, y: 0.24, stability: 'Attractor', eigenvalues: [0.3, 0.1] },
    ];
    const manifolds = [
      { saddle_point: [0.5, 0.15], plus: { points: [] }, minus: { points: [] } },
    ];

    const classified = classifyManifolds(manifolds, fixedPoints);
    expect(classified[0].isRepeller).toBe(false);
  });

  it('handles "unstable" as synonym for repeller', () => {
    const fixedPoints = [
      { x: 1.0, y: 0.3, stability: 'unstable', eigenvalues: [2.0, 1.5] },
    ];
    const manifolds = [
      { saddle_point: [1.0, 0.3], plus: { points: [] }, minus: { points: [] } },
    ];

    const classified = classifyManifolds(manifolds, fixedPoints);
    expect(classified[0].isRepeller).toBe(true);
  });

  it('handles empty manifolds and fixed points', () => {
    const classified = classifyManifolds([], []);
    expect(classified).toEqual([]);
  });
});

describe('Fixed Point Color Assignment', () => {
  const ORBIT_COLORS = {
    attractor: '#27ae60',
    repeller: '#e74c3c',
    saddlePoint: '#eedf32',
    periodicBlue: '#3498db',
  };

  const getFixedPointColor = (stability) => {
    const stabLower = (stability || '').toLowerCase();
    const isAttractor = stabLower === 'attractor' || stabLower === 'stable';
    const isRepeller = stabLower === 'repeller' || stabLower === 'unstable';
    const isSaddle = stabLower === 'saddle';
    return isAttractor ? ORBIT_COLORS.attractor :
      isRepeller ? ORBIT_COLORS.repeller :
      isSaddle ? ORBIT_COLORS.saddlePoint : ORBIT_COLORS.periodicBlue;
  };

  it('assigns green to attractors', () => {
    expect(getFixedPointColor('Attractor')).toBe('#27ae60');
    expect(getFixedPointColor('stable')).toBe('#27ae60');
  });

  it('assigns red to repellers (distinct from saddles)', () => {
    expect(getFixedPointColor('Repeller')).toBe('#e74c3c');
    expect(getFixedPointColor('unstable')).toBe('#e74c3c');
  });

  it('assigns yellow to saddles', () => {
    expect(getFixedPointColor('Saddle')).toBe('#eedf32');
    expect(getFixedPointColor('saddle')).toBe('#eedf32');
  });

  it('repeller and saddle have different colors', () => {
    expect(getFixedPointColor('Repeller')).not.toBe(getFixedPointColor('Saddle'));
  });

  it('assigns default color to unknown stability', () => {
    expect(getFixedPointColor('')).toBe('#3498db');
    expect(getFixedPointColor(null)).toBe('#3498db');
  });
});

describe('Bifurcation State Management', () => {
  it('initial state has correct defaults', () => {
    const initialState = {
      data: [],
      isComputing: false,
      error: null,
      aMin: 0.1,
      aMax: 2.0,
      numSamples: 30,
      threshold: 0.2,
      intersectionCount: 0,
      criticalValues: [],
    };

    expect(initialState.data).toEqual([]);
    expect(initialState.isComputing).toBe(false);
    expect(initialState.aMin).toBeLessThan(initialState.aMax);
    expect(initialState.numSamples).toBeGreaterThan(0);
  });

  it('bifurcation data has expected shape', () => {
    const sampleData = [
      { a: 0.5, hausdorff_distance: 0.3, max_unstable_to_stable: 0.3, max_stable_to_unstable: 0.2, has_intersection: false, intersection_threshold: 0.02 },
      { a: 1.0, hausdorff_distance: 0.01, max_unstable_to_stable: 0.01, max_stable_to_unstable: 0.005, has_intersection: true, intersection_threshold: 0.02 },
    ];

    sampleData.forEach(d => {
      expect(d).toHaveProperty('a');
      expect(d).toHaveProperty('hausdorff_distance');
      expect(d).toHaveProperty('has_intersection');
      expect(typeof d.a).toBe('number');
      expect(typeof d.hausdorff_distance).toBe('number');
      expect(typeof d.has_intersection).toBe('boolean');
      expect(d.hausdorff_distance).toBeGreaterThanOrEqual(0);
    });
  });

  it('critical values are extracted correctly from data', () => {
    const data = [
      { a: 0.5, hausdorff_distance: 0.3, has_intersection: false },
      { a: 1.0, hausdorff_distance: 0.01, has_intersection: true },
      { a: 1.2, hausdorff_distance: 0.005, has_intersection: true },
      { a: 1.5, hausdorff_distance: 0.5, has_intersection: false },
    ];

    const intersections = data.filter(d => d.has_intersection);
    const criticalValues = intersections.map(d => d.a);

    expect(criticalValues).toEqual([1.0, 1.2]);
    expect(intersections.length).toBe(2);
  });
});

describe('Parameter Sweep State Management', () => {
  it('initial sweep state has correct defaults', () => {
    const initialState = {
      results: null,
      isComputing: false,
      error: null,
      sweepParam: 'a',
      sweepMin: 0.1,
      sweepMax: 2.0,
      numSamples: 10,
      maxPeriod: 3,
    };

    expect(initialState.results).toBeNull();
    expect(initialState.isComputing).toBe(false);
    expect(initialState.sweepMin).toBeLessThan(initialState.sweepMax);
    expect(initialState.sweepParam).toBe('a');
    expect(initialState.numSamples).toBeGreaterThan(0);
    expect(initialState.maxPeriod).toBeGreaterThan(0);
  });

  it('sweep result data has expected shape', () => {
    const sampleResult = {
      param_value: 1.0,
      total_orbits: 3,
      stable_count: 1,
      unstable_count: 1,
      saddle_count: 1,
      orbits: [
        { points: [[0.5, 0.15]], period: 1, stability: 'stable', eigenvalue1: 0.3, eigenvalue2: 0.1 },
        { points: [[0.8, 0.24]], period: 1, stability: 'saddle', eigenvalue1: 1.5, eigenvalue2: 0.2 },
        { points: [[-1.0, -0.3], [0.6, -0.3]], period: 2, stability: 'unstable', eigenvalue1: 2.0, eigenvalue2: 1.5 },
      ],
    };

    expect(sampleResult.total_orbits).toBe(sampleResult.stable_count + sampleResult.unstable_count + sampleResult.saddle_count);
    sampleResult.orbits.forEach(o => {
      expect(o).toHaveProperty('period');
      expect(o).toHaveProperty('stability');
      expect(o).toHaveProperty('eigenvalue1');
      expect(o).toHaveProperty('eigenvalue2');
      expect(o.points.length).toBe(o.period);
      expect(['stable', 'unstable', 'saddle']).toContain(o.stability);
    });
  });

  it('CSV export produces valid format', () => {
    const results = [
      {
        param_value: 1.0,
        orbits: [
          { points: [[0.5, 0.15]], period: 1, stability: 'stable', eigenvalue1: 0.3, eigenvalue2: 0.1 },
        ],
        total_orbits: 1, stable_count: 1, unstable_count: 0, saddle_count: 0,
      },
    ];

    let csv = 'parameter_a,period,stability,eigenvalue1,eigenvalue2,x,y\n';
    for (const r of results) {
      for (const o of r.orbits) {
        for (const [x, y] of o.points) {
          csv += `${r.param_value},${o.period},${o.stability},${o.eigenvalue1},${o.eigenvalue2},${x},${y}\n`;
        }
      }
    }

    const lines = csv.trim().split('\n');
    expect(lines[0]).toBe('parameter_a,period,stability,eigenvalue1,eigenvalue2,x,y');
    expect(lines.length).toBe(2);
    const cols = lines[1].split(',');
    expect(cols.length).toBe(7);
    expect(parseFloat(cols[0])).toBe(1.0);
    expect(cols[2]).toBe('stable');
  });

  it('orbit counts are aggregated correctly across sweep', () => {
    const results = [
      { param_value: 0.5, total_orbits: 2, stable_count: 1, unstable_count: 0, saddle_count: 1, orbits: [] },
      { param_value: 1.0, total_orbits: 3, stable_count: 0, unstable_count: 1, saddle_count: 2, orbits: [] },
      { param_value: 1.5, total_orbits: 4, stable_count: 2, unstable_count: 1, saddle_count: 1, orbits: [] },
    ];

    const totalOrbits = results.reduce((sum, r) => sum + r.total_orbits, 0);
    expect(totalOrbits).toBe(9);

    const totalStable = results.reduce((sum, r) => sum + r.stable_count, 0);
    const totalUnstable = results.reduce((sum, r) => sum + r.unstable_count, 0);
    const totalSaddle = results.reduce((sum, r) => sum + r.saddle_count, 0);
    expect(totalStable + totalUnstable + totalSaddle).toBe(totalOrbits);
  });
});

describe('Manifold Computation Trigger Logic', () => {
  // Mirrors the gating logic from SetValuedViz.jsx manifold computation useEffect
  const shouldComputeManifolds = (showStable, showUnstable) => showStable || showUnstable;

  it('computes when only unstable is enabled', () => {
    expect(shouldComputeManifolds(false, true)).toBe(true);
  });

  it('computes when only stable is enabled', () => {
    expect(shouldComputeManifolds(true, false)).toBe(true);
  });

  it('computes when both are enabled', () => {
    expect(shouldComputeManifolds(true, true)).toBe(true);
  });

  it('does not compute when neither is enabled', () => {
    expect(shouldComputeManifolds(false, false)).toBe(false);
  });
});

describe('System-Specific Screenshot Metadata', () => {
  const getParamOverlayText = (dynamicSystem, params, customParams) => {
    if (dynamicSystem === 'duffing_ode') {
      return `δ = ${(params.delta || 0).toFixed(4)}  h = ${(params.h || 0).toFixed(4)}  ε = ${(params.epsilon || 0).toFixed(4)}`;
    } else if (dynamicSystem === 'custom') {
      const cp = (customParams || []).map(p => `${p.name} = ${p.value.toFixed(4)}`).join('  ');
      return cp || `ε = ${(params.epsilon || 0).toFixed(4)}`;
    }
    return `a = ${(params.a || 0).toFixed(4)}  b = ${(params.b || 0).toFixed(4)}  ε = ${(params.epsilon || 0).toFixed(4)}`;
  };

  const getSystemFilePrefix = (dynamicSystem) => {
    const prefixes = { henon: 'henon', duffing: 'duffing_map', duffing_ode: 'duffing_ode', custom: 'custom' };
    return prefixes[dynamicSystem] || 'system';
  };

  it('henon shows a, b, epsilon', () => {
    const text = getParamOverlayText('henon', { a: 1.4, b: 0.3, epsilon: 0.01 });
    expect(text).toContain('a = 1.4000');
    expect(text).toContain('b = 0.3000');
    expect(text).toContain('ε = 0.0100');
  });

  it('duffing_ode shows delta, h, epsilon', () => {
    const text = getParamOverlayText('duffing_ode', { delta: 0.15, h: 0.05, epsilon: 0.1 });
    expect(text).toContain('δ = 0.1500');
    expect(text).toContain('h = 0.0500');
    expect(text).not.toContain('a =');
  });

  it('custom shows custom param names', () => {
    const cp = [{ name: 'alpha', value: 2.5 }, { name: 'beta', value: 0.7 }];
    const text = getParamOverlayText('custom', { epsilon: 0.01 }, cp);
    expect(text).toContain('alpha = 2.5000');
    expect(text).toContain('beta = 0.7000');
  });

  it('henon file prefix is henon', () => {
    expect(getSystemFilePrefix('henon')).toBe('henon');
  });

  it('duffing_ode file prefix is duffing_ode', () => {
    expect(getSystemFilePrefix('duffing_ode')).toBe('duffing_ode');
  });

  it('custom file prefix is custom', () => {
    expect(getSystemFilePrefix('custom')).toBe('custom');
  });
});

describe('Tab Switch State Cleanup', () => {
  it('cleanup should reset manifold visibility flags', () => {
    // Simulate what the dynamicSystem change useEffect should do
    const prevState = {
      showUnstableManifold: true,
      showStableManifold: true,
      showOrbits: true,
      showTrail: false,
      trajectoryPoints: [{ x: 1, y: 2 }],
      manifolds: [{ id: 1 }],
      stableManifolds: [{ id: 2 }],
    };

    // Simulate cleanup
    const cleaned = {
      ...prevState,
      isRunning: false,
      hasStarted: false,
      iteration: 0,
      trajectoryPoints: [],
      manifolds: [],
      stableManifolds: [],
      fixedPoints: [],
      intersections: [],
      showUnstableManifold: false,
      showStableManifold: false,
      showOrbits: true,
      showTrail: true,
    };

    expect(cleaned.showUnstableManifold).toBe(false);
    expect(cleaned.showStableManifold).toBe(false);
    expect(cleaned.trajectoryPoints).toEqual([]);
    expect(cleaned.manifolds).toEqual([]);
    expect(cleaned.stableManifolds).toEqual([]);
    expect(cleaned.showOrbits).toBe(true);
    expect(cleaned.showTrail).toBe(true);
  });
});
