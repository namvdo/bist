import React from 'react';
import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { BasinApproximationPanel } from './BasinApproximationPanel';

const state = {
  showBasin: true,
  isComputing: false,
  result: null,
  error: null,
  notice: null
};

describe('BasinApproximationPanel', () => {
  it('disables computation without adding a long prerequisite description', () => {
    render(<BasinApproximationPanel state={state} setState={vi.fn()} canCompute={false} targetPointCount={0} compute={vi.fn()} />);
    expect(screen.getByRole('button', { name: /Compute basin/ })).toBeDisabled();
    expect(screen.queryByText(/closed unstable-manifold MIS/)).toBeNull();
  });

  it('keeps the common workflow compact and runs the computation', () => {
    const compute = vi.fn();
    const { container } = render(<BasinApproximationPanel state={state} setState={vi.fn()} canCompute targetPointCount={320} compute={compute} />);
    fireEvent.click(screen.getByRole('button', { name: /Compute basin/ }));
    expect(compute).toHaveBeenCalledOnce();
    expect(screen.getByText(/320 MIS boundary samples/)).toBeInTheDocument();
    expect(screen.getByLabelText('Show basin')).toBeChecked();
    expect(screen.queryByText('Quality')).not.toBeInTheDocument();
    expect(screen.queryByText('Advanced numerical settings')).not.toBeInTheDocument();
    expect(screen.queryByLabelText(/Certified/)).not.toBeInTheDocument();
    expect(screen.queryByLabelText(/Possible/)).not.toBeInTheDocument();
    expect(screen.queryByRole('slider')).toBeNull();
    expect(container.querySelector('.t-swatch-line')).toHaveStyle({ background: '#ffd400' });
  });

  it('offers immediate cancellation while automatic expansion is running', () => {
    const cancel = vi.fn();
    render(<BasinApproximationPanel state={{ ...state, isComputing: true }} setState={vi.fn()} canCompute={false} targetPointCount={320} compute={vi.fn()} cancel={cancel} />);
    fireEvent.click(screen.getByRole('button', { name: /Cancel basin computation/ }));
    expect(cancel).toHaveBeenCalledOnce();
    expect(screen.getByText(/until no new boxes are found/)).toBeInTheDocument();
  });

  it('reports graph and certification diagnostics', () => {
    const result = {
      stop_reason: 'resolution_limited',
      trapping_verified: true,
      converged: true,
      expansion_limit: 512,
      inner_area: 1.2,
      outer_area: 1.8,
      unresolved_area: 0.6,
      inner_cell_count: 120,
      outer_cell_count: 180,
      target_cell_count: 20,
      candidate_target_cell_count: 30,
      completed_inner_levels: 4,
      completed_outer_levels: 7,
      grid_x: 40,
      grid_y: 40,
      grid_theta: 24,
      evaluated_cell_count: 850,
      inverse_frontier_cell_count: 175,
      graph_edge_count: 2300,
      transition_run_count: 420,
      boundary_contact_cell_count: 0
    };
    const { container } = render(<BasinApproximationPanel state={{ ...state, result }} setState={vi.fn()} canCompute targetPointCount={50} compute={vi.fn()} />);
    fireEvent.click(screen.getByText('Computation details'));
    expect(screen.getByText(/target core 20 \/ 30/)).toBeInTheDocument();
    expect(screen.getByText(/forward rows 850 \/ 38,400/)).toBeInTheDocument();
    expect(screen.getByText(/inverse frontiers 175/)).toBeInTheDocument();
    expect(screen.getByText(/graph edges 2,300/)).toBeInTheDocument();
    expect(screen.getByText(/stored runs 420/)).toBeInTheDocument();
    expect(screen.getByText(/Backward expansion converged/)).toBeInTheDocument();
    expect(screen.getByText(/converged yes/)).toBeInTheDocument();
    expect(screen.queryByText(/blue region/)).not.toBeInTheDocument();
    expect(container.querySelector('.basin-status')).toHaveClass('ready');
  });

  it('labels an outer-only result without claiming certification', () => {
    const result = {
      stop_reason: 'no_trapping_core',
      trapping_verified: false,
      converged: true,
      expansion_limit: 512,
      inner_area: 0,
      outer_area: 2.1,
      unresolved_area: 2.1,
      inner_cell_count: 0,
      outer_cell_count: 210,
      target_cell_count: 0,
      candidate_target_cell_count: 40,
      completed_inner_levels: 0,
      completed_outer_levels: 8,
      graph_edge_count: 3200,
      boundary_contact_cell_count: 0
    };
    render(<BasinApproximationPanel state={{ ...state, result }} setState={vi.fn()} canCompute targetPointCount={50} compute={vi.fn()} />);
    expect(screen.getByText(/outer estimate only/)).toBeInTheDocument();
    expect(screen.getByText(/verified area 0.0000/)).toBeInTheDocument();
  });

  it('clearly marks a resource-limited result as incomplete', () => {
    const result = {
      stop_reason: 'resource_limit',
      trapping_verified: true,
      converged: false,
      expansion_limit: 512,
      inner_area: 1.2,
      outer_area: 2.1,
      unresolved_area: 0.9,
      inner_cell_count: 120,
      outer_cell_count: 210,
      target_cell_count: 20,
      candidate_target_cell_count: 30,
      completed_inner_levels: 511,
      completed_outer_levels: 512,
      grid_x: 40,
      grid_y: 40,
      grid_theta: 24,
      evaluated_cell_count: 38000,
      inverse_frontier_cell_count: 37000,
      graph_edge_count: 100000,
      transition_run_count: 20000,
      boundary_contact_cell_count: 40
    };
    const { container } = render(<BasinApproximationPanel state={{ ...state, result }} setState={vi.fn()} canCompute targetPointCount={50} compute={vi.fn()} />);
    expect(screen.getByText(/reached before convergence/)).toBeInTheDocument();
    expect(screen.getByText(/displayed basin is incomplete/)).toBeInTheDocument();
    expect(container.querySelector('.basin-status')).toHaveClass('limited');
  });
});
