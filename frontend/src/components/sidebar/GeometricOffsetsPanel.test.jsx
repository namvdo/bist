import React from 'react';
import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { GeometricOffsetsPanel } from './GeometricOffsetsPanel';

const state = {
  numLevels: 5,
  resolution: 256,
  showContours: true,
  isComputing: false,
  result: null,
  error: null
};

const result = {
  completed_levels: 2,
  epsilon: 0.1,
  stop_reason: 'requested_levels_completed',
  levels: [
    { level: 1, target_distance: 0.1, area: 1.2, component_count: 1, offset_residual: 0.001, gap_residual: 0.0015, uncertainty: 0.01 },
    { level: 2, target_distance: 0.2, area: 1.8, component_count: 1, offset_residual: 0.002, gap_residual: 0.0025, uncertainty: 0.01 }
  ]
};

describe('GeometricOffsetsPanel', () => {
  it('disables computation without showing prerequisite instructions', () => {
    const { container } = render(<GeometricOffsetsPanel state={state} setState={vi.fn()} canCompute={false} compute={vi.fn()} />);
    expect(screen.getByRole('button', { name: /Compute exact ε-offset/ })).toBeDisabled();
    expect(container.querySelector('.geometric-offset-note')).toBeNull();
  });

  it('runs geometric offset computation', () => {
    const compute = vi.fn();
    render(<GeometricOffsetsPanel state={state} setState={vi.fn()} canCompute compute={compute} />);
    fireEvent.click(screen.getByRole('button', { name: /Compute exact ε-offset/ }));
    expect(compute).toHaveBeenCalledOnce();
  });

  it('reports geometric target distances and residuals', () => {
    render(<GeometricOffsetsPanel state={{ ...state, result }} setState={vi.fn()} canCompute compute={vi.fn()} />);
    expect(screen.getByText(/2 levels/)).toBeInTheDocument();
    expect(screen.getByText(/gap ε 0.1000/)).toBeInTheDocument();
    expect(screen.getByLabelText(/Geometric offset target distances/)).toHaveTextContent('G2 0.200');
    expect(screen.getByText(/gap residual 2.50e-3/)).toBeInTheDocument();
    expect(screen.getByText(/target residual 2.00e-3/)).toBeInTheDocument();
  });

  it('marks a contour leaving the view as a warning', () => {
    const escaped = { ...result, completed_levels: 1, stop_reason: 'escaped_domain', levels: result.levels.slice(0, 1) };
    const { container } = render(<GeometricOffsetsPanel state={{ ...state, result: escaped }} setState={vi.fn()} canCompute compute={vi.fn()} />);
    expect(container.querySelector('.geometric-offset-status')).toHaveClass('warning');
  });
});
