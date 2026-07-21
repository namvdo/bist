import React from 'react';
import { render, screen } from '@testing-library/react';
import { beforeEach, describe, expect, it, vi } from 'vitest';

const controlsBarMock = vi.fn();

vi.mock('../sidebar/SystemPicker', () => ({ SystemPicker: () => <div data-testid="system-picker" /> }));
vi.mock('../sidebar/EquationDisplay', () => ({ EquationDisplay: () => <div data-testid="equation-display" /> }));
vi.mock('../sidebar/ParametersPanel', () => ({ ParametersPanel: () => <div data-testid="parameters-panel" /> }));
vi.mock('../sidebar/ManifoldsPanel', () => ({ ManifoldsPanel: () => <div data-testid="manifolds-panel" /> }));
vi.mock('../sidebar/GeometricOffsetsPanel', () => ({ GeometricOffsetsPanel: () => <div data-testid="geometric-offsets-panel" /> }));
vi.mock('../sidebar/BasinApproximationPanel', () => ({ BasinApproximationPanel: () => <div data-testid="basin-panel" /> }));
vi.mock('../sidebar/VisualizationPanel', () => ({ VisualizationPanel: () => <div data-testid="visualization-panel" /> }));
vi.mock('../sidebar/StartingPoint', () => ({ StartingPoint: () => <div data-testid="starting-point" /> }));
vi.mock('../sidebar/PeriodicOrbitsPanel', () => ({ PeriodicOrbitsPanel: () => <div data-testid="periodic-orbits" /> }));
vi.mock('../sidebar/PeriodicSearchPanel', () => ({ PeriodicSearchPanel: () => <div data-testid="periodic-search-panel" /> }));
vi.mock('../sidebar/UlamPanel', () => ({ UlamPanel: () => <div data-testid="ulam-panel" /> }));
vi.mock('../sidebar/AnimationPanel', () => ({ AnimationPanel: () => <div data-testid="animation-panel" /> }));
vi.mock('../sidebar/ParameterSweepPanel', () => ({ ParameterSweepPanel: () => <div data-testid="sweep-panel" /> }));
vi.mock('./InfoStrip', () => ({ InfoStrip: () => <div data-testid="info-strip" /> }));
vi.mock('./ControlsBar', () => ({
  ControlsBar: (props) => {
    controlsBarMock(props);
    return <div data-testid="controls-bar" />;
  }
}));

import { Sidebar } from './Sidebar';

const baseProps = {
  type: 'continuous',
  setType: vi.fn(),
  dynamicSystem: 'duffing_ode',
  setDynamicSystem: vi.fn(),
  SYSTEMS: { continuous: [], discrete: [] },
  customEquations: {},
  setCustomEquations: vi.fn(),
  equationError: null,
  params: {},
  setParams: vi.fn(),
  applyPreset: vi.fn(),
  customParams: [],
  setCustomParams: vi.fn(),
  paramErrors: [],
  hasPendingInputChanges: false,
  applyInputsAndRecompute: vi.fn(),
  appliedParams: {},
  viewRange: { xMin: -2, xMax: 2, yMin: -1.5, yMax: 1.5 },
  setViewRange: vi.fn(),
  rangeLimit: 10,
  resetViewRange: vi.fn(),
  manifoldState: {},
  setManifoldState: vi.fn(),
  geometricOffsetState: {},
  setGeometricOffsetState: vi.fn(),
  canComputeGeometricOffsets: false,
  computeGeometricOffsets: vi.fn(),
  basinState: {},
  setBasinState: vi.fn(),
  canComputeBasin: false,
  basinTargetPointCount: 0,
  computeBasin: vi.fn(),
  ORBIT_COLORS: {},
  filters: {},
  setFilters: vi.fn(),
  periodicState: {},
  periodicSearchSettings: { gridSize: 10, thetaGridSize: 10, residualThreshold: 1e-10 },
  updatePeriodicSearchSettings: vi.fn(),
  updateStartPoint: vi.fn(),
  animationState: {},
  setAnimationState: vi.fn(),
  recordingState: {},
  startAnimation: vi.fn(),
  stopAnimation: vi.fn(),
  toggleRecording: vi.fn(),
  ulamState: {},
  setUlamState: vi.fn(),
  sweepState: { results: null, isComputing: false, error: null, sweepParam: 'a', sweepMin: 0.1, sweepMax: 2.0, numSamples: 10, maxPeriod: 3 },
  setSweepState: vi.fn(),
  wasmModule: null,
  bdeState: {},
  stepForwardManifold: vi.fn(),
  runToConvergenceManifold: vi.fn(),
  resetManifold: vi.fn(),
  toggleBdeFlow: vi.fn(),
  resetBdeFlow: vi.fn()
};

describe('Sidebar', () => {
  beforeEach(() => {
    controlsBarMock.mockClear();
  });

  it('shows the starting point panel for continuous systems', () => {
    render(<Sidebar {...baseProps} type="continuous" dynamicSystem="duffing_ode" />);
    expect(screen.getByTestId('starting-point')).toBeInTheDocument();
  });

  it('hides the starting point panel for discrete systems', () => {
    render(<Sidebar {...baseProps} type="discrete" dynamicSystem="henon" />);
    expect(screen.queryByTestId('starting-point')).toBeNull();
  });

  it('shows periodic search section for discrete boundary systems', () => {
    render(<Sidebar {...baseProps} type="discrete" dynamicSystem="henon" />);
    expect(screen.getByTestId('periodic-search-panel')).toBeInTheDocument();
  });

  it('shows geometric offset controls only for the Hénon boundary map', () => {
    const { rerender } = render(<Sidebar {...baseProps} type="discrete" dynamicSystem="henon" />);
    expect(screen.getByTestId('geometric-offsets-panel')).toBeInTheDocument();

    rerender(<Sidebar {...baseProps} type="discrete" dynamicSystem="standard" />);
    expect(screen.queryByTestId('geometric-offsets-panel')).toBeNull();
  });

  it('shows deterministic basin controls only for the Hénon boundary map', () => {
    const { rerender } = render(<Sidebar {...baseProps} type="discrete" dynamicSystem="henon" />);
    expect(screen.getByTestId('basin-panel')).toBeInTheDocument();

    rerender(<Sidebar {...baseProps} type="discrete" dynamicSystem="standard" />);
    expect(screen.queryByTestId('basin-panel')).toBeNull();
  });

  it('passes recompute controls into the bottom controls bar', () => {
    const onRecompute = vi.fn();
    render(<Sidebar {...baseProps} applyInputsAndRecompute={onRecompute} hasPendingInputChanges={true} />);
    const lastCall = controlsBarMock.mock.calls[controlsBarMock.mock.calls.length - 1];
    expect(lastCall[0].applyInputsAndRecompute).toBe(onRecompute);
    expect(lastCall[0].hasPendingInputChanges).toBe(true);
  });
});
