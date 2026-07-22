import React from 'react';
import { SystemPicker } from '../sidebar/SystemPicker';
import { EquationDisplay } from '../sidebar/EquationDisplay';
import { ParametersPanel } from '../sidebar/ParametersPanel';
import { ManifoldsPanel } from '../sidebar/ManifoldsPanel';
import { VisualizationPanel } from '../sidebar/VisualizationPanel';
import { StartingPoint } from '../sidebar/StartingPoint';
import { PeriodicOrbitsPanel } from '../sidebar/PeriodicOrbitsPanel';
import { PeriodicSearchPanel } from '../sidebar/PeriodicSearchPanel';
import { UlamPanel } from '../sidebar/UlamPanel';
import { AnimationPanel } from '../sidebar/AnimationPanel';
import { ParameterSweepPanel } from '../sidebar/ParameterSweepPanel';
import { GeometricOffsetsPanel } from '../sidebar/GeometricOffsetsPanel';
import { BasinApproximationPanel } from '../sidebar/BasinApproximationPanel';
import { InfoStrip } from './InfoStrip';
import { ControlsBar } from './ControlsBar';

export const Sidebar = (props) => {
  const { ORBIT_COLORS } = props;
  const logoSrc = `${import.meta.env.BASE_URL}bist_logo.png`;

  return (
    <div className="sidebar">
      <div className="app-name">
        <img src={logoSrc} alt="BIST Logo" style={{ height: '18px', width: 'auto' }} />
        BIST
      </div>

      <div className="sidebar-scroll">
        <SystemPicker
          type={props.type}
          setType={props.setType}
          systemId={props.dynamicSystem}
          setSystemId={props.setDynamicSystem}
          systems={props.SYSTEMS}
        />

        <EquationDisplay
          systemId={props.dynamicSystem}
          customEquations={props.customEquations}
          setCustomEquations={props.setCustomEquations}
          equationError={props.equationError}
          disabled={props.manifoldState.isRunning}
        />

        <ParametersPanel
          systemId={props.dynamicSystem}
          params={props.params}
          setParams={props.setParams}
          disabled={props.manifoldState.isRunning}
          systems={props.SYSTEMS}
          applyPreset={props.applyPreset}
          customParams={props.customParams}
          setCustomParams={props.setCustomParams}
          paramErrors={props.paramErrors}
        />

        {props.type === 'discrete' && (
          <PeriodicSearchPanel
            dynamicSystem={props.dynamicSystem}
            periodicSearchSettings={props.periodicSearchSettings}
            updatePeriodicSearchSettings={props.updatePeriodicSearchSettings}
            periodicState={props.periodicState}
            disabled={props.manifoldState.isRunning}
          />
        )}

        <VisualizationPanel
          manifoldState={props.manifoldState}
          setManifoldState={props.setManifoldState}
          viewRange={props.viewRange}
          setViewRange={props.setViewRange}
          rangeLimit={props.rangeLimit}
          resetViewRange={props.resetViewRange}
        />

        {props.type === 'discrete' && (
          <>
            <ManifoldsPanel
              manifoldState={props.manifoldState}
              setManifoldState={props.setManifoldState}
              ORBIT_COLORS={ORBIT_COLORS}
            />
            {props.dynamicSystem === 'henon' && (
              <>
                <GeometricOffsetsPanel
                  state={props.geometricOffsetState}
                  setState={props.setGeometricOffsetState}
                  canCompute={props.canComputeGeometricOffsets}
                  compute={props.computeGeometricOffsets}
                />
                <BasinApproximationPanel
                  state={props.basinState}
                  setState={props.setBasinState}
                  canCompute={props.canComputeBasin}
                  targetPointCount={props.basinTargetPointCount}
                  compute={props.computeBasin}
                  cancel={props.cancelBasinComputation}
                />
              </>
            )}
            <PeriodicOrbitsPanel
              manifoldState={props.manifoldState}
              setManifoldState={props.setManifoldState}
              filters={props.filters}
              setFilters={props.setFilters}
              periodicState={props.periodicState}
            />
          </>
        )}

        {props.type === 'continuous' && (
          <StartingPoint
            type={props.type}
            startPoint={props.manifoldState.startPoint}
            updateStartPoint={props.updateStartPoint}
          />
        )}

        {props.type === 'discrete' && props.dynamicSystem !== 'custom' && (
          <AnimationPanel
            animationState={props.animationState}
            setAnimationState={props.setAnimationState}
            manifoldState={props.manifoldState}
            recordingState={props.recordingState}
            startAnimation={props.startAnimation}
            stopAnimation={props.stopAnimation}
            toggleRecording={props.toggleRecording}
          />
        )}

        {props.type === 'discrete' && (
          <ParameterSweepPanel
            wasmModule={props.wasmModule}
            params={props.params}
            viewRange={props.viewRange}
            sweepState={props.sweepState}
            setSweepState={props.setSweepState}
            dynamicSystem={props.dynamicSystem}
            customEquations={props.customEquations}
            customParams={props.customParams}
          />
        )}

        <UlamPanel
          ulamState={props.ulamState}
          setUlamState={props.setUlamState}
        />

      </div>

      <InfoStrip
        type={props.type}
        manifoldState={props.manifoldState}
        ulamState={props.ulamState}
        params={props.appliedParams || props.params}
        periodicState={props.periodicState}
      />

      <ControlsBar
        dynamicSystem={props.dynamicSystem}
        manifoldState={props.manifoldState}
        bdeState={props.bdeState}
        stepForwardManifold={props.stepForwardManifold}
        runToConvergenceManifold={props.runToConvergenceManifold}
        resetManifold={props.resetManifold}
        toggleBdeFlow={props.toggleBdeFlow}
        resetBdeFlow={props.resetBdeFlow}
        applyInputsAndRecompute={props.applyInputsAndRecompute}
        hasPendingInputChanges={props.hasPendingInputChanges}
      />
    </div>
  );
};
