import { render, screen } from '@testing-library/react';
import { vi } from 'vitest';
import { PeriodicOrbitsPanel } from './PeriodicOrbitsPanel';

const baseProps = {
  manifoldState: {
    showOrbits: true,
    showTrail: true,
    showAttractingRegions: true,
    fixedPoints: []
  },
  setManifoldState: vi.fn(),
  filters: {
    period1: true,
    period2: true,
    period3: true,
    period4: true,
    period5: true,
    period6plus: false
  },
  setFilters: vi.fn(),
  periodicState: {
    isReady: true,
    orbits: []
  }
};

describe('PeriodicOrbitsPanel', () => {
  it('keeps orbit markers and trajectory controls without the redundant orbit-lines toggle', () => {
    render(<PeriodicOrbitsPanel {...baseProps} />);

    expect(screen.getByLabelText('Orbit markers')).toBeInTheDocument();
    expect(screen.getByLabelText('Trajectory trail')).toBeInTheDocument();
    expect(screen.queryByLabelText('Orbit lines')).toBeNull();
  });

  it('shows period counts from periodic orbit data', () => {
    const periodicState = {
      isReady: true,
      orbits: [
        {
          period: 1,
          stability: 'saddle',
          points: [[1.219, 0.789]],
          eigenvalues: [0.43]
        },
        {
          period: 2,
          stability: 'stable',
          points: [[1.522, -0.184], [-1.331, 1.154]],
          eigenvalues: [0.22, 0.31]
        }
      ]
    };

    render(<PeriodicOrbitsPanel {...baseProps} periodicState={periodicState} />);

    expect(screen.getByText('Period filter')).toBeInTheDocument();
    expect(screen.getAllByRole('button').length).toBe(6);
    expect(screen.getByRole('button', { name: /6\+/ })).toBeInTheDocument();
  });

  it('shows fixed point rows when manifold fixed points are available', () => {
    const manifoldState = {
      ...baseProps.manifoldState,
      fixedPoints: [
        { x: 0.5, y: 0.25, stability: 'saddle', eigenvalues: [1.2] }
      ]
    };

    render(
      <PeriodicOrbitsPanel
        {...baseProps}
        manifoldState={manifoldState}
        periodicState={{ orbits: [] }}
      />
    );

    expect(screen.getByText('Fixed points (1)')).toBeInTheDocument();
    expect(screen.getByText('(0.500, 0.250)')).toBeInTheDocument();
  });

  it('does not render periodic search controls in this panel', () => {
    render(<PeriodicOrbitsPanel {...baseProps} />);
    expect(screen.queryByLabelText('Grid size')).toBeNull();
    expect(screen.queryByLabelText('Theta grid')).toBeNull();
    expect(screen.queryByLabelText('Residual threshold')).toBeNull();
  });
});
