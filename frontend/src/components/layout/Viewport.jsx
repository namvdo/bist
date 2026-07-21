import React from 'react';
import { BASIN_LAYER_STYLES } from '../../utils/basinDisplay';

export const Viewport = ({ type, canvasRef, tooltip, manifoldState, geometricOffsetState, basinState, ulamState, displayRange, handleZoomIn, handleZoomOut, handleResetView, handlePanMode, savePNG }) => {
  const serializedDisplayRange = displayRange
    ? `${displayRange.xMin},${displayRange.xMax},${displayRange.yMin},${displayRange.yMax}`
    : undefined;

  return (
    <div className="viewport" data-view-range={serializedDisplayRange}>
      <div className="vp-tools">
        <button type="button" className="vp-btn" title="Zoom in" aria-label="Zoom in" onClick={handleZoomIn}>+</button>
        <button type="button" className="vp-btn" title="Zoom out" aria-label="Zoom out" onClick={handleZoomOut}>−</button>
        <button type="button" className="vp-btn" title="Reset view" aria-label="Reset view" onClick={handleResetView}>⌂</button>
        <div className="vp-sep"></div>
        {type === 'continuous' && (
          <button className="vp-btn active" title="Place start point">📍</button>
        )}
        <button className="vp-btn" title="Pan" onClick={handlePanMode}>⊹</button>
        <div className="vp-sep"></div>
        <button className="vp-btn" title="Save PNG" onClick={savePNG}>↓</button>
      </div>



      <div className="vp-legend">
        <div className="vp-legend-title">Legend</div>
        {manifoldState.showUnstableManifold && <div className="lg-item"><div className="lg-line" style={{ background: '#5b88b5' }}></div>Unstable manifold</div>}
        {manifoldState.showStableManifold && <div className="lg-item"><div className="lg-line" style={{ background: '#b8904a' }}></div>Stable manifold</div>}
        {geometricOffsetState?.showContours && geometricOffsetState?.result && <div className="lg-item"><div className="lg-line" style={{ background: '#3f9186' }}></div>Geometric ε-offsets</div>}
        {basinState?.showBasin && basinState?.result && <div className="lg-item"><div className="lg-box" style={{ background: BASIN_LAYER_STYLES.inner.color }}></div>Basin approximation</div>}
        {manifoldState.showOrbits && (
          <>
            <div className="lg-item"><div className="lg-dot" style={{ background: '#b8904a' }}></div>Saddle</div>
            <div className="lg-item"><div className="lg-dot" style={{ background: '#5a9668' }}></div>Stable</div>
            <div className="lg-item"><div className="lg-dot" style={{ background: '#a85252' }}></div>Unstable</div>
          </>
        )}
        <div className="lg-item"><div className="lg-dot" style={{ background: '#8a5faa' }}></div>Trajectory</div>
      </div>

      {tooltip.visible && !ulamState.showUlamOverlay && tooltip.data?.type !== 'Ulam Box' && (
        <div className="vp-tooltip" style={{ top: Math.min(tooltip.y, window.innerHeight - 150), left: Math.min(tooltip.x + 15, window.innerWidth - 200) }}>
          <div className="vp-tt-head">
            <div className="t-swatch" style={{ background: tooltip.data.stability === 'stable' ? '#5a9668' : tooltip.data.stability === 'saddle' ? '#b8904a' : '#a85252', width: '8px', height: '8px', borderRadius: '50%', flexShrink: 0 }}></div>
            {tooltip.data.type === 'Fixed Point' ? 'Fixed point' : `Period-${tooltip.data.period} orbit`}
          </div>
          <div className="vp-tt-grid">
            <span className="tt-k">Position</span><span className="tt-v em">({tooltip.data.pos.x.toFixed(4)}, {tooltip.data.pos.y.toFixed(4)})</span>
            <span className="tt-k">Stability</span>
            <span className="tt-v" style={{ color: tooltip.data.stability === 'stable' ? 'var(--green)' : tooltip.data.stability === 'saddle' ? 'var(--amber)' : 'var(--red)' }}>
              {tooltip.data.stability?.charAt(0).toUpperCase() + tooltip.data.stability?.slice(1)}
            </span>
            {tooltip.data.eigenvalues && (
              <>
                <span className="tt-k">Eigenvalues</span>
                <span className="tt-v">{tooltip.data.eigenvalues.map(v => v.toFixed(3)).join(', ')}</span>
              </>
            )}
            {tooltip.data.jacobian && (
              <>
                <span className="tt-k">det(J)</span><span className="tt-v">{tooltip.data.jacobian.det.toFixed(3)}</span>
                <span className="tt-k">tr(J)</span><span className="tt-v">{tooltip.data.jacobian.trace.toFixed(3)}</span>
              </>
            )}
          </div>
        </div>
      )}

      {tooltip.visible && tooltip.data && tooltip.data.type === 'Ulam Box' && (
        <div className="vp-tooltip" style={{ top: Math.min(tooltip.y, window.innerHeight - 150), left: Math.min(tooltip.x + 15, window.innerWidth - 200) }}>
          <div className="vp-tt-head">
            <div className="t-swatch" style={{ background: '#5b88b5', width: '8px', height: '8px', borderRadius: '50%', flexShrink: 0 }}></div>
            Ulam Box #{tooltip.data.boxIndex}
          </div>
          <div className="vp-tt-grid">
            <span className="tt-k">Center</span><span className="tt-v em">({tooltip.data.pos.x.toFixed(3)}, {tooltip.data.pos.y.toFixed(3)})</span>
            <span className="tt-k">Measure</span><span className="tt-v" style={{ color: 'var(--amber)' }}>{tooltip.data.measurePercent.toFixed(1)}% of max</span>
            <span className="tt-k">Transitions</span><span className="tt-v">{tooltip.data.numTransitions} paths</span>
            {tooltip.data.topTransitions && tooltip.data.topTransitions.length > 0 && (
              <>
                <span className="tt-k" style={{ gridColumn: '1 / -1', marginTop: '4px' }}>Top targets:</span>
                {tooltip.data.topTransitions.map((t, idx) => (
                  <React.Fragment key={idx}>
                    <span className="tt-k" style={{ paddingLeft: '10px' }}>Box #{t.index}</span>
                    <span className="tt-v">{(t.probability * 100).toFixed(1)}%</span>
                  </React.Fragment>
                ))}
              </>
            )}
          </div>
        </div>
      )}

      <canvas
        ref={canvasRef}
        style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%' }}
      />
    </div>
  );
}
