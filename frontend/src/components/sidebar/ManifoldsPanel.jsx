import React from 'react';
import { Collapsible } from '../ui/Collapsible';
import { Toggle } from '../ui/Toggle';
import { Slider } from '../ui/Slider';

export const ManifoldsPanel = ({ manifoldState, setManifoldState, ORBIT_COLORS }) => {
  return (
    <Collapsible title="Manifolds" defaultOpen={true}>
      <Toggle
        label="Unstable manifold"
        colorLine={ORBIT_COLORS.manifold}
        checked={manifoldState.showUnstableManifold}
        onChange={(v) => setManifoldState(prev => ({ ...prev, showUnstableManifold: v }))}
      />

      <Toggle
        label="Stable manifold"
        colorLine={ORBIT_COLORS.stableManifold}
        checked={manifoldState.showStableManifold}
        onChange={(v) => setManifoldState(prev => ({ ...prev, showStableManifold: v }))}
      />

      {manifoldState.showStableManifold && (
        <div id="intersect-panel" style={{ marginTop: '8px' }}>
          <Slider
            label="Detection threshold ε"
            min={0.001} max={0.2} step={0.001}
            value={manifoldState.intersectionThreshold}
            onChange={v => setManifoldState(prev => ({ ...prev, intersectionThreshold: v }))}
          />
          {(() => {
            const heteroClinic = manifoldState.intersections.filter(i => i.has_intersection);
            if (heteroClinic.length > 0) {
              const minDist = Math.min(...heteroClinic.map(i => i.min_distance));
              return (
                <div className="intersect-warn">
                  <div>⚠ Heteroclinic connection!</div>
                  <div style={{ fontSize: '9px', opacity: 0.8, marginTop: '2px' }}>
                    {heteroClinic.length} connection{heteroClinic.length > 1 ? 's' : ''} found (min d = {minDist.toFixed(4)})
                  </div>
                </div>
              );
            } else if (manifoldState.intersections.length > 0) {
              return <div className="intersect-ok">✓ No heteroclinic connections</div>;
            } else {
              return <div style={{ fontSize: '10px', color: 'var(--text-3)' }}>Need ≥2 saddles for detection</div>;
            }
          })()}
        </div>
      )}

      {/* Fixed point legend */}
      <div style={{ marginTop: '8px', fontSize: '10px', color: 'var(--text-2)' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px', marginBottom: '2px' }}>
          <span style={{ width: '8px', height: '8px', borderRadius: '50%', background: ORBIT_COLORS.attractor, display: 'inline-block' }} />
          <span>Attractor (|λ₁|,|λ₂| &lt; 1)</span>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px', marginBottom: '2px' }}>
          <span style={{ width: '8px', height: '8px', borderRadius: '50%', background: ORBIT_COLORS.saddlePoint, display: 'inline-block' }} />
          <span>Saddle (|λ₁| &lt; 1 &lt; |λ₂|)</span>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
          <span style={{ width: '8px', height: '8px', borderRadius: '50%', background: ORBIT_COLORS.repeller, display: 'inline-block' }} />
          <span>Repeller (|λ₁|,|λ₂| &gt; 1)</span>
        </div>
      </div>
    </Collapsible>
  );
};
