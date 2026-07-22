import React from 'react';
import { fireEvent, render, screen } from '@testing-library/react';
import { vi } from 'vitest';
import { PeriodicSearchPanel } from './PeriodicSearchPanel';

const baseProps = {
  dynamicSystem: 'henon',
  periodicSearchSettings: {
    gridSize: 10,
    thetaGridSize: 10,
    residualThreshold: 1e-10,
    useContinuation: false
  },
  periodicState: { computeMethod: null },
  updatePeriodicSearchSettings: vi.fn(),
  disabled: false
};

describe('PeriodicSearchPanel', () => {
  it('renders search controls for boundary-map systems', () => {
    render(<PeriodicSearchPanel {...baseProps} />);
    expect(screen.getByLabelText('Grid size')).toBeInTheDocument();
    expect(screen.getByLabelText('Theta grid')).toBeInTheDocument();
    expect(screen.getByLabelText('Residual threshold')).toBeInTheDocument();
    expect(screen.getByText('Use continuation')).toBeInTheDocument();
  });

  it('emits updates when search settings change', () => {
    const onUpdate = vi.fn();
    render(<PeriodicSearchPanel {...baseProps} updatePeriodicSearchSettings={onUpdate} />);

    fireEvent.change(screen.getByLabelText('Grid size'), { target: { value: '24' } });
    fireEvent.change(screen.getByLabelText('Theta grid'), { target: { value: '32' } });
    fireEvent.change(screen.getByLabelText('Residual threshold'), { target: { value: '1e-8' } });
    fireEvent.click(screen.getByText('Use continuation'));

    expect(onUpdate).toHaveBeenCalledWith({ gridSize: 24 });
    expect(onUpdate).toHaveBeenCalledWith({ thetaGridSize: 32 });
    expect(onUpdate).toHaveBeenCalledWith({ residualThreshold: 1e-8 });
    expect(onUpdate).toHaveBeenCalledWith({ useContinuation: true });
  });

  it('keeps continuation compact without the explanatory paragraph', () => {
    render(<PeriodicSearchPanel {...baseProps} periodicState={{ computeMethod: 'continuation' }} />);
    expect(screen.getByText('Use continuation')).toBeInTheDocument();
    expect(screen.queryByText(/Last run used continuation/)).toBeNull();
    expect(screen.queryByText(/Enable continuation/)).toBeNull();
  });

  it('is hidden for systems without boundary periodic search', () => {
    const { container } = render(<PeriodicSearchPanel {...baseProps} dynamicSystem="duffing" />);
    expect(container).toBeEmptyDOMElement();
  });
});
