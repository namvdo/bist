import React from 'react';
import { render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { Viewport } from './Viewport';

const baseProps = {
  type: 'continuous',
  canvasRef: { current: null },
  tooltip: { visible: false },
  manifoldState: {
    showUnstableManifold: false,
    showStableManifold: false,
    showOrbits: false
  },
  geometricOffsetState: { showContours: false },
  basinState: { showBasin: false },
  ulamState: { showUlamOverlay: false },
  handleZoomIn: vi.fn(),
  handleZoomOut: vi.fn(),
  handleResetView: vi.fn(),
  handlePanMode: vi.fn(),
  savePNG: vi.fn()
};

describe('Viewport', () => {
  it('shows the start point tool for continuous systems', () => {
    render(<Viewport {...baseProps} type="continuous" />);
    expect(screen.getByTitle('Place start point')).toBeInTheDocument();
  });

  it('hides the start point tool for discrete systems', () => {
    render(<Viewport {...baseProps} type="discrete" />);
    expect(screen.queryByTitle('Place start point')).toBeNull();
  });
});
