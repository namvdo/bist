import React from 'react';

const formatResidual = (value) => {
  if (!Number.isFinite(value)) return 'n/a';
  if (value === 0) return '0';
  if (value < 1e-3) return value.toExponential(2);
  return value.toFixed(5);
};

export const Viewport = ({ type, canvasRef, tooltip, manifoldState, ulamState, hittingContourState, handleZoomIn, handleZoomOut, handleResetView, handlePanMode, savePNG }) => {
  return (
    <div className="viewport">
      <div className="vp-tools">
        <button className="vp-btn" title="Zoom in" onClick={handleZoomIn}>+</button>
        <button className="vp-btn" title="Zoom out" onClick={handleZoomOut}>−</button>
        <button className="vp-btn" title="Reset view" onClick={handleResetView}>⌂</button>
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
        {manifoldState.showNormalFan && <div className="lg-item"><div className="lg-line" style={{ background: '#ddd5c8' }}></div>Normal directions</div>}
        {manifoldState.showOrbits && (
          <>
            <div className="lg-item"><div className="lg-dot" style={{ background: '#b8904a' }}></div>Saddle</div>
            <div className="lg-item"><div className="lg-dot" style={{ background: '#5a9668' }}></div>Stable</div>
            <div className="lg-item"><div className="lg-dot" style={{ background: '#a85252' }}></div>Unstable</div>
          </>
        )}
        {hittingContourState?.showOverlay && (
          <div className="lg-item"><div className="lg-dot" style={{ background: '#b8904a' }}></div>{hittingContourState.layoutMode === 'spatial' ? 'Hitting level' : 'Hitting contours'}</div>
        )}
        <div className="lg-item"><div className="lg-dot" style={{ background: '#8a5faa' }}></div>Trajectory</div>
      </div>

      {tooltip.visible && tooltip.data?.type === 'Hitting Level' && (
        <div className="vp-tooltip" style={{ top: Math.min(tooltip.y, window.innerHeight - 160), left: Math.min(tooltip.x + 15, window.innerWidth - 230) }}>
          <div className="vp-tt-head">
            <div className="t-swatch" style={{ background: tooltip.data.hit.stability === 'stable' ? '#5a9668' : tooltip.data.hit.stability === 'unstable' ? '#a85252' : '#b8904a', width: '8px', height: '8px', borderRadius: '50%', flexShrink: 0 }}></div>
            Level {tooltip.data.hit.level} hit
          </div>
          <div className="vp-tt-grid">
            <span className="tt-k">Sample</span><span className="tt-v em">({tooltip.data.cell.x.toFixed(4)}, {tooltip.data.cell.y.toFixed(4)})</span>
            <span className="tt-k">Target</span><span className="tt-v">#{tooltip.data.hit.targetIndex}</span>
            <span className="tt-k">Period</span><span className="tt-v">{tooltip.data.hit.period}</span>
            <span className="tt-k">Stability</span><span className="tt-v">{tooltip.data.hit.stability}</span>
            <span className="tt-k">Distance</span><span className="tt-v">{formatResidual(tooltip.data.hit.distance)}</span>
            <span className="tt-k">Tolerance</span><span className="tt-v">{formatResidual(tooltip.data.radius)}</span>
            <span className="tt-k">Hits here</span><span className="tt-v">{tooltip.data.hitCount}</span>
          </div>
        </div>
      )}

      {tooltip.visible && tooltip.data?.type === 'Hitting Target' && (
        <div className="vp-tooltip" style={{ top: Math.min(tooltip.y, window.innerHeight - 150), left: Math.min(tooltip.x + 15, window.innerWidth - 220) }}>
          <div className="vp-tt-head">
            <div className="t-swatch" style={{ background: tooltip.data.target.stability === 'stable' ? '#5a9668' : tooltip.data.target.stability === 'unstable' ? '#a85252' : '#b8904a', width: '8px', height: '8px', borderRadius: '50%', flexShrink: 0 }}></div>
            Periodic target #{tooltip.data.target.targetIndex}
          </div>
          <div className="vp-tt-grid">
            <span className="tt-k">Position</span><span className="tt-v em">({tooltip.data.target.x.toFixed(4)}, {tooltip.data.target.y.toFixed(4)})</span>
            <span className="tt-k">Normal</span><span className="tt-v">({tooltip.data.target.nx.toFixed(3)}, {tooltip.data.target.ny.toFixed(3)})</span>
            <span className="tt-k">Period</span><span className="tt-v">{tooltip.data.target.period}</span>
            <span className="tt-k">Stability</span><span className="tt-v">{tooltip.data.target.stability}</span>
            {tooltip.data.topographic && (
              <>
                <span className="tt-k">Best level</span><span className="tt-v">{tooltip.data.topographic.bestLevel ?? 'n/a'}</span>
                <span className="tt-k">Rings</span><span className="tt-v">{tooltip.data.topographic.ringCount}</span>
                <span className="tt-k">Hits</span><span className="tt-v">{tooltip.data.topographic.totalHits}</span>
              </>
            )}
          </div>
        </div>
      )}

      {tooltip.visible && tooltip.data?.type === 'Hitting Contour Ring' && (
        <div className="vp-tooltip" style={{ top: Math.min(tooltip.y, window.innerHeight - 190), left: Math.min(tooltip.x + 15, window.innerWidth - 260) }}>
          <div className="vp-tt-head">
            <div className="t-swatch" style={{ background: tooltip.data.target.stability === 'stable' ? '#5a9668' : tooltip.data.target.stability === 'unstable' ? '#a85252' : '#b8904a', width: '8px', height: '8px', borderRadius: '50%', flexShrink: 0 }}></div>
            Level {tooltip.data.level} contour
          </div>
          <div className="vp-tt-grid">
            <span className="tt-k">Target</span><span className="tt-v">#{tooltip.data.target.targetIndex}</span>
            <span className="tt-k">Period</span><span className="tt-v">{tooltip.data.target.period}</span>
            <span className="tt-k">Points</span><span className="tt-v">{tooltip.data.count}</span>
            <span className="tt-k">Samples</span>
            <span className="tt-v">
              {(tooltip.data.points || []).slice(0, 5).map(point => `(${point.x.toFixed(3)}, ${point.y.toFixed(3)})`).join(', ')}
              {(tooltip.data.points || []).length > 5 ? ' ...' : ''}
            </span>
          </div>
        </div>
      )}

      {tooltip.visible && tooltip.data?.type === 'Hitting Level Point' && (
        <div className="vp-tooltip" style={{ top: Math.min(tooltip.y, window.innerHeight - 170), left: Math.min(tooltip.x + 15, window.innerWidth - 240) }}>
          <div className="vp-tt-head">
            <div className="t-swatch" style={{ background: tooltip.data.point.stability === 'stable' ? '#5a9668' : tooltip.data.point.stability === 'unstable' ? '#a85252' : '#b8904a', width: '8px', height: '8px', borderRadius: '50%', flexShrink: 0 }}></div>
            Level {tooltip.data.level} sample
          </div>
          <div className="vp-tt-grid">
            <span className="tt-k">Sample</span><span className="tt-v em">({tooltip.data.point.x.toFixed(4)}, {tooltip.data.point.y.toFixed(4)})</span>
            <span className="tt-k">Target</span><span className="tt-v">#{tooltip.data.target.targetIndex}</span>
            <span className="tt-k">Period</span><span className="tt-v">{tooltip.data.target.period}</span>
            <span className="tt-k">Distance</span><span className="tt-v">{formatResidual(tooltip.data.point.distance)}</span>
            <span className="tt-k">Cell</span><span className="tt-v">#{tooltip.data.point.cellIndex}</span>
          </div>
        </div>
      )}

      {tooltip.visible && !ulamState.showUlamOverlay && tooltip.data?.type !== 'Ulam Box' && tooltip.data?.type !== 'Hitting Level' && tooltip.data?.type !== 'Hitting Target' && tooltip.data?.type !== 'Hitting Contour Ring' && tooltip.data?.type !== 'Hitting Level Point' && (
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
