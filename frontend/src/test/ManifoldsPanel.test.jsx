import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import { vi } from 'vitest';
import { ManifoldsPanel } from '../components/sidebar/ManifoldsPanel';

const ORBIT_COLORS = {
  manifold: '#1e90ff',
  stableManifold: '#ffa500',
  repellerManifold: '#ff4444',
  attractor: '#27ae60',
  repeller: '#e74c3c',
  saddlePoint: '#eedf32',
};

describe('ManifoldsPanel', () => {
  const defaultManifoldState = {
    showUnstableManifold: false,
    showStableManifold: false,
    showNormalFan: false,
    normalFanCount: 8,
    showRepellerManifold: false,
    intersectionThreshold: 0.05,
    intersections: [],
  };

  it('renders all three manifold toggles', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={defaultManifoldState}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    expect(screen.getByText('Unstable manifold')).toBeInTheDocument();
    expect(screen.getByText('Stable manifold')).toBeInTheDocument();
    expect(screen.getByText('Normal directions')).toBeInTheDocument();
  });

  it('shows normal fan controls when enabled', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={{ ...defaultManifoldState, showNormalFan: true }}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    expect(screen.getByText('Directions')).toBeInTheDocument();
    expect(screen.getByText('Click the viewport to move the base point.')).toBeInTheDocument();
  });

  it('updates the normal fan direction count', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={{ ...defaultManifoldState, showNormalFan: true }}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    fireEvent.change(screen.getAllByDisplayValue('8')[0], { target: { value: '12' } });
    expect(setManifoldState.mock.calls[0][0](defaultManifoldState).normalFanCount).toBe(12);
  });

  it('renders fixed point classification legend', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={defaultManifoldState}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    expect(screen.getByText(/Attractor \(\|λ/)).toBeInTheDocument();
    expect(screen.getByText(/Saddle \(\|λ/)).toBeInTheDocument();
    expect(screen.getByText(/Repeller \(\|λ/)).toBeInTheDocument();
  });

  it('shows intersection detection panel when stable manifold enabled', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={{ ...defaultManifoldState, showStableManifold: true }}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    expect(screen.getByText(/Detection threshold/)).toBeInTheDocument();
  });

  it('hides intersection panel when stable manifold disabled', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={{ ...defaultManifoldState, showStableManifold: false }}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    expect(screen.queryByText(/Detection threshold/)).toBeNull();
  });

  it('shows heteroclinic warning when intersections found', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={{
          ...defaultManifoldState,
          showStableManifold: true,
          intersections: [
            { has_intersection: true, min_distance: 0.02 },
            { has_intersection: false, min_distance: 0.5 },
          ],
        }}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    expect(screen.getByText(/Heteroclinic connection/)).toBeInTheDocument();
    expect(screen.getByText(/1 connection found/)).toBeInTheDocument();
  });

  it('shows no connections message when intersections checked but none found', () => {
    const setManifoldState = vi.fn();
    render(
      <ManifoldsPanel
        manifoldState={{
          ...defaultManifoldState,
          showStableManifold: true,
          intersections: [
            { has_intersection: false, min_distance: 0.3 },
          ],
        }}
        setManifoldState={setManifoldState}
        ORBIT_COLORS={ORBIT_COLORS}
      />
    );

    expect(screen.getByText(/No heteroclinic connections/)).toBeInTheDocument();
  });
});
