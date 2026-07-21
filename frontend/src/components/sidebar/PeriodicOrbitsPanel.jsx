import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Toggle } from '../ui/Toggle';

export const PeriodicOrbitsPanel = ({
  manifoldState,
  setManifoldState,
  filters,
  setFilters,
  periodicState
}) => {

  const toggleFilter = (period) => {
    setFilters(prev => ({ ...prev, [period]: !prev[period] }));
  };

  const periodCounts = [1, 2, 3, 4, 5, '6+'].map(period => {
    const key = period === '6+' ? 'period6plus' : `period${period}`;

    // Count how many orbits match this period
    let count = 0;
    if (periodicState.orbits) {
      if (period === '6+') {
        count = periodicState.orbits.filter(o => o.period >= 6).length;
      } else {
        count = periodicState.orbits.filter(o => o.period === period).length;
      }
    }

    return {
      label: period,
      key: key,
      count,
      active: filters[key]
    };
  });

  return (
    <Collapsible title="Periodic orbits" defaultOpen={true}>
      <Toggle
        label="Orbit markers"
        checked={manifoldState.showOrbits}
        onChange={v => setManifoldState(prev => ({ ...prev, showOrbits: v }))}
      />
      <Toggle
        label="Trajectory trail"
        checked={manifoldState.showTrail}
        onChange={v => setManifoldState(prev => ({ ...prev, showTrail: v }))}
      />

      <div className="small-label">Period filter</div>
      <div className="period-filter">
        {periodCounts.map(p => (
          <button
            key={p.label}
            className={`per-btn ${p.active ? 'on' : ''}`}
            onClick={() => toggleFilter(p.key)}
          >
            {p.label}<span className="per-count">{p.count}</span>
          </button>
        ))}
      </div>

      <div style={{ display: 'flex', gap: '10px', marginTop: '7px', fontSize: '10px', color: 'var(--text-3)' }}>
        <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
          <span style={{ width: '7px', height: '7px', borderRadius: '50%', background: '#5a9668', display: 'inline-block' }}></span>Stable
        </span>
        <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
          <span style={{ width: '7px', height: '7px', borderRadius: '50%', background: '#b8904a', display: 'inline-block' }}></span>Saddle
        </span>
        <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
          <span style={{ width: '7px', height: '7px', borderRadius: '50%', background: '#a85252', display: 'inline-block' }}></span>Unstable
        </span>
      </div>

      {(manifoldState.fixedPoints && manifoldState.fixedPoints.length > 0) && (
        <>
          <div className="small-label" style={{ marginTop: '10px' }}>Fixed points ({manifoldState.fixedPoints.length})</div>
          <div className="fp-list">
            {manifoldState.fixedPoints.map((fp, i) => {
              const bg = fp.stability === 'stable' ? '#5a9668' : fp.stability === 'saddle' ? '#b8904a' : '#a85252';
              return (
                <div key={i} className="fp-row">
                  <div className="fp-dot" style={{ background: bg }}></div>
                  ({fp.x.toFixed(3)}, {fp.y.toFixed(3)})
                  <span className="fp-stab">
                    {fp.stability.charAt(0).toUpperCase() + fp.stability.slice(1)}
                    {fp.eigenvalues && fp.eigenvalues.length > 0 && ` · λ=${Math.max(...fp.eigenvalues).toFixed(2)}`}
                  </span>
                </div>
              );
            })}
          </div>
        </>
      )}

    </Collapsible>
  );
};
