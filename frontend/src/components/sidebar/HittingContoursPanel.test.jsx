import React from 'react';
import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { HittingContoursPanel } from './HittingContoursPanel';
import { DEFAULT_HITTING_CONTOUR_STATE } from '../../utils/hittingContours';

describe('HittingContoursPanel', () => {
  it('is hidden outside the Hénon boundary map', () => {
    const { container } = render(
      <HittingContoursPanel
        dynamicSystem="duffing"
        hittingContourState={DEFAULT_HITTING_CONTOUR_STATE}
        setHittingContourState={vi.fn()}
      />
    );

    expect(container).toBeEmptyDOMElement();
  });

  it('shows only levels returned by the computation result', () => {
    render(
      <HittingContoursPanel
        dynamicSystem="henon"
        hittingContourState={{
          ...DEFAULT_HITTING_CONTOUR_STATE,
          showOverlay: true,
          result: {
            levelsPresent: [1, 3],
            summary: { hitCellCount: 4, targetCount: 2 }
          }
        }}
        setHittingContourState={vi.fn()}
      />
    );

    expect(screen.getByText('Hitting levels')).toBeInTheDocument();
    expect(screen.getByText('topo')).toBeInTheDocument();
    expect(screen.getByText('spatial')).toBeInTheDocument();
    expect(screen.getByText('Circle lines')).toBeInTheDocument();
    expect(screen.getByText('Ulam support')).toBeInTheDocument();
    expect(screen.getByLabelText('Hit tolerance')).toBeInTheDocument();
    expect(screen.getByLabelText('Orbit residual')).toBeInTheDocument();
    expect(screen.getByText('1e-10')).toBeInTheDocument();
    expect(screen.getByText('1e-2')).toBeInTheDocument();
    expect(screen.queryByText('Radius')).toBeNull();
    expect(screen.getByText('1')).toBeInTheDocument();
    expect(screen.getByText('3')).toBeInTheDocument();
    expect(screen.queryByText('2')).toBeNull();
  });

  it('emits live threshold setting updates', () => {
    const onSetState = vi.fn();
    render(
      <HittingContoursPanel
        dynamicSystem="henon"
        hittingContourState={{
          ...DEFAULT_HITTING_CONTOUR_STATE,
          showOverlay: true
        }}
        setHittingContourState={onSetState}
      />
    );

    fireEvent.change(screen.getByLabelText('Hit tolerance'), { target: { value: '1e-4' } });
    fireEvent.change(screen.getByLabelText('Orbit residual'), { target: { value: '1e-8' } });
    fireEvent.click(screen.getByText('1e-6'));

    expect(onSetState.mock.calls[0][0](DEFAULT_HITTING_CONTOUR_STATE).hitTolerance).toBe(1e-4);
    expect(onSetState.mock.calls[1][0](DEFAULT_HITTING_CONTOUR_STATE).residualThreshold).toBe(1e-8);
    expect(onSetState.mock.calls[2][0](DEFAULT_HITTING_CONTOUR_STATE).hitTolerance).toBe(1e-6);
  });
});
