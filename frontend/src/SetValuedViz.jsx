import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import * as THREE from 'three';
import { Shell } from './components/layout/Shell';
import { Sidebar } from './components/layout/Sidebar';
import { Viewport } from './components/layout/Viewport';
import { normalizeParams } from './utils/paramUtils';
import { DEFAULT_VIEW_RANGE, RANGE_LIMIT, normalizeViewRange } from './utils/viewRange';
import {
    DEFAULT_PERIODIC_SEARCH_SETTINGS,
    normalizePeriodicSearchSettings
} from './utils/periodicSearchSettings';
import { applyStartPointUpdate } from './utils/startPointState';
import {
    DEFAULT_HITTING_CONTOUR_STATE,
    getHittingTargetContours,
    getStabilityColorHex,
    getVisibleHittingCells,
    normalizeHittingContourSettings,
    normalizeHittingContourState
} from './utils/hittingContours';

const GRID_STYLE = {
    gridDivisions: 16,
    axisColor: 0x888888,
    gridColor: 0x333333
};

const EMPTY_ARRAY = [];
const DEFAULT_VALIDATION = { normalized: EMPTY_ARRAY, errors: EMPTY_ARRAY, valid: true };
const PARAM_ABS_LIMIT = 10;

const buildCircleRingPoints = (centerX, centerY, radius, segments = 128) => {
    const points = [];
    for (let i = 0; i <= segments; i++) {
        const t = (i / segments) * Math.PI * 2;
        points.push(new THREE.Vector3(
            centerX + Math.cos(t) * radius,
            centerY + Math.sin(t) * radius,
            0.34
        ));
    }
    return points;
};

const buildAbstractGrid = (viewRange, divisions = 12) => {
    const points = [];
    const width = viewRange.xMax - viewRange.xMin;
    const height = viewRange.yMax - viewRange.yMin;
    if (width <= 0 || height <= 0) return points;

    for (let i = 0; i <= divisions; i++) {
        const tx = i / divisions;
        const x = viewRange.xMin + tx * width;
        const y = viewRange.yMin + tx * height;
        points.push(new THREE.Vector3(x, viewRange.yMin, 0.285));
        points.push(new THREE.Vector3(x, viewRange.yMax, 0.285));
        points.push(new THREE.Vector3(viewRange.xMin, y, 0.285));
        points.push(new THREE.Vector3(viewRange.xMax, y, 0.285));
    }

    return points;
};
const INITIAL_CUSTOM_EQUATIONS = {
    custom: {
        xEq: '1 - a * x^2 + y',
        yEq: 'b * x'
    },
    custom_ode: {
        xEq: 'y',
        yEq: 'x - x^3 - delta * y'
    }
};
const INITIAL_CUSTOM_PARAMS = {
    custom: [
        { name: 'a', value: 1.4 },
        { name: 'b', value: 0.3 }
    ],
    custom_ode: [
        { name: 'delta', value: 0.15 }
    ]
};
const INITIAL_PARAMS = {
    a: 0.4,
    b: 0.3,
    delta: 0.15,
    h: 0.05,
    epsilon: 0.1,
    startX: 0.1,
    startY: 0.1,
    maxIterations: 1000,
    maxPeriod: 5
};


const getSupportIndex = (x, y, support) => {
    if (!support) return -1;
    const { xMin, xMax, yMin, yMax, subdivisions } = support;
    if (x < xMin || x > xMax || y < yMin || y > yMax) return -1;

    const dx = (xMax - xMin) / subdivisions;
    const dy = (yMax - yMin) / subdivisions;

    if (!Number.isFinite(dx) || !Number.isFinite(dy) || dx <= 0 || dy <= 0) {
        return -1;
    }

    let ix = Math.floor((x - xMin) / dx);
    let iy = Math.floor((y - yMin) / dy);
    if (ix >= subdivisions) ix -= 1;
    if (iy >= subdivisions) iy -= 1;
    if (ix < 0 || iy < 0) return -1;
    return iy * subdivisions + ix;
};

const getGridBoxIndex = (x, y, range, subdivisions) => {
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(subdivisions) || subdivisions <= 0) {
        return -1;
    }
    if (x < range.xMin || x > range.xMax || y < range.yMin || y > range.yMax) return -1;

    const dx = (range.xMax - range.xMin) / subdivisions;
    const dy = (range.yMax - range.yMin) / subdivisions;
    if (!Number.isFinite(dx) || !Number.isFinite(dy) || dx <= 0 || dy <= 0) return -1;

    let ix = Math.floor((x - range.xMin) / dx);
    let iy = Math.floor((y - range.yMin) / dy);
    if (ix >= subdivisions) ix -= 1;
    if (iy >= subdivisions) iy -= 1;
    if (ix < 0 || iy < 0) return -1;
    return iy * subdivisions + ix;
};

const isSupportedPoint = (x, y, support) => {
    if (!support) return true;
    const idx = getSupportIndex(x, y, support);
    if (idx < 0) return false;
    return (support.invariantMeasure?.[idx] ?? 0) > support.threshold;
};

const clipTrajectoryBySupport = (traj, support) => {
    if (!traj?.points || !support) return traj;
    return {
        ...traj,
        points: traj.points.filter(([x, y]) => isSupportedPoint(x, y, support))
    };
};

const clipManifoldsBySupport = (manifolds, support) => {
    if (!support) return manifolds || [];
    return (manifolds || [])
        .map(m => ({
            ...m,
            plus: clipTrajectoryBySupport(m.plus, support),
            minus: clipTrajectoryBySupport(m.minus, support)
        }))
        .filter(m => ((m.plus?.points?.length || 0) + (m.minus?.points?.length || 0)) > 0);
};

const clampToRange = (value, minValue, maxValue, fallbackValue) => {
    if (!Number.isFinite(value)) {
        return fallbackValue;
    }
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
};


const createCoordinateSystem = (scene, range) => {
    const { gridDivisions, axisColor, gridColor } = GRID_STYLE;
    const { xMin, xMax, yMin, yMax } = range;
    const xStep = (xMax - xMin) / gridDivisions;
    const yStep = (yMax - yMin) / gridDivisions;

    const gridGroup = new THREE.Group();
    gridGroup.name = 'coordinate-system';

    for (let i = 0; i <= gridDivisions; i++) {
        const x = xMin + i * xStep;
        const isAxis = Math.abs(x) < 0.01;
        const points = [
            new THREE.Vector3(x, yMin, -0.01),
            new THREE.Vector3(x, yMax, -0.01)
        ];
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
            color: isAxis ? axisColor : gridColor,
            transparent: true,
            opacity: isAxis ? 1.0 : 0.4
        });
        const line = new THREE.Line(geometry, material);
        line.userData.isGrid = true;
        gridGroup.add(line);
    }

    for (let i = 0; i <= gridDivisions; i++) {
        const y = yMin + i * yStep;
        const isAxis = Math.abs(y) < 0.01;
        const points = [
            new THREE.Vector3(xMin, y, -0.01),
            new THREE.Vector3(xMax, y, -0.01)
        ];
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
            color: isAxis ? axisColor : gridColor,
            transparent: true,
            opacity: isAxis ? 1.0 : 0.4
        });
        const line = new THREE.Line(geometry, material);
        line.userData.isGrid = true;
        gridGroup.add(line);
    }

    const createTextSprite = (text, position, fontSize = 0.15) => {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 128;
        canvas.height = 64;
        context.fillStyle = 'transparent';
        context.fillRect(0, 0, canvas.width, canvas.height);
        context.font = 'Bold 32px Arial';
        context.fillStyle = '#aaaaaa';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(text, canvas.width / 2, canvas.height / 2);

        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter;
        const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
        const sprite = new THREE.Sprite(material);
        sprite.position.copy(position);
        sprite.scale.set(fontSize * 2, fontSize, 1);
        sprite.userData.isGrid = true;
        return sprite;
    };

    for (let i = 0; i <= gridDivisions; i++) {
        const x = xMin + i * xStep;
        if (Math.abs(x) > 0.01) {
            gridGroup.add(createTextSprite(x.toFixed(1), new THREE.Vector3(x, yMin - 0.15, 0), 0.12));
        }
    }
    for (let i = 0; i <= gridDivisions; i++) {
        const y = yMin + i * yStep;
        if (Math.abs(y) > 0.01) {
            gridGroup.add(createTextSprite(y.toFixed(1), new THREE.Vector3(xMin - 0.2, y, 0), 0.12));
        }
    }

    gridGroup.add(createTextSprite('x', new THREE.Vector3(xMax + 0.2, 0, 0), 0.18));
    gridGroup.add(createTextSprite('y', new THREE.Vector3(0, yMax + 0.15, 0), 0.18));
    gridGroup.add(createTextSprite('0', new THREE.Vector3(-0.12, -0.12, 0), 0.1));

    scene.add(gridGroup);
    return gridGroup;
};

const ORBIT_COLORS = {
    period1: { stable: '#27ae60', unstable: '#e74c3c', saddle: '#eedf32' },
    period2: { stable: '#27ae60', unstable: '#e74c3c', saddle: '#eedf32' },
    period3: { stable: '#27ae60', unstable: '#e74c3c', saddle: '#eedf32' },
    period4: { stable: '#27ae60', unstable: '#e74c3c', saddle: '#eedf32' },
    period5: { stable: '#27ae60', unstable: '#e74c3c', saddle: '#eedf32' },
    period6plus: { stable: '#27ae60', unstable: '#e74c3c', saddle: '#eedf32' },
    trajectory: '#ff00ff',  // Bright magenta for high visibility
    manifold: '#1e90ff',  // Blue for unstable manifold
    stableManifold: '#ffa500', // Orange for stable manifold
    attractor: '#27ae60',
    repeller: '#e74c3c',
    saddlePoint: '#eedf32',
    periodicBlue: '#3498db'
};

const SetValuedViz = () => {
    const canvasRef = useRef(null);
    const rendererRef = useRef(null);
    const sceneRef = useRef(null);
    const cameraRef = useRef(null);
    const animationFrameRef = useRef(null);
    const batchAnimationRef = useRef(null);
    const manifoldDebounceRef = useRef(null);

    const [dynamicSystem, setDynamicSystem] = useState('duffing_ode'); // 'henon', 'duffing', or 'custom'
    const [customEquations, setCustomEquations] = useState(INITIAL_CUSTOM_EQUATIONS);
    const [draftCustomEquations, setDraftCustomEquations] = useState(INITIAL_CUSTOM_EQUATIONS);
    const [customParams, setCustomParams] = useState(INITIAL_CUSTOM_PARAMS);
    const [draftCustomParams, setDraftCustomParams] = useState(INITIAL_CUSTOM_PARAMS);
    const [equationError, setEquationError] = useState(null);
    const [wasmModule, setWasmModule] = useState(null);
    const [computeRequestId, setComputeRequestId] = useState(0);

    const [params, setParams] = useState(INITIAL_PARAMS);
    const [draftParams, setDraftParams] = useState(INITIAL_PARAMS);

    const [periodicState, setPeriodicState] = useState({
        orbits: [],
        isReady: false,
        showOrbits: false,
        computeMethod: null
    });
    const [hittingContourState, setHittingContourState] = useState(DEFAULT_HITTING_CONTOUR_STATE);
    const [periodicSearchSettings, setPeriodicSearchSettings] = useState(DEFAULT_PERIODIC_SEARCH_SETTINGS);
    const [draftPeriodicSearchSettings, setDraftPeriodicSearchSettings] = useState(DEFAULT_PERIODIC_SEARCH_SETTINGS);

    const [manifoldState, setManifoldState] = useState({
        manifolds: [],
        stableManifolds: [],
        fixedPoints: [],
        intersections: [],
        isComputing: false,
        isReady: false,
        showOrbits: true,
        showOrbitLines: false,
        showUnstableManifold: false,
        showStableManifold: false,
        showNormalFan: false,
        normalFanCount: 8,
        intersectionThreshold: 0.05,
        highlightedOrbitId: null,
        currentPoint: { x: 0.1, y: 0.1, nx: 1.0, ny: 0.0 }, // 4D point for boundary map
        trajectoryPoints: [],
        iteration: 0,
        isRunning: false,
        hasStarted: false,
        showTrail: true,
        startPoint: { x: 0.1, y: 0.1, nx: 1.0, ny: 0.0 }
    });

    const [filters, setFilters] = useState({
        period1: true, period2: true, period3: true,
        period4: true, period5: true, period6plus: false
    });

    const isCustomDiscrete = dynamicSystem === 'custom';
    const isCustomContinuous = dynamicSystem === 'custom_ode';
    const isCustomSystem = isCustomDiscrete || isCustomContinuous;
    const type = (dynamicSystem === 'duffing_ode' || dynamicSystem === 'custom_ode') ? 'continuous' : 'discrete';
    const supportsPeriodicSearchSettings = dynamicSystem === 'henon' || dynamicSystem === 'custom';
    const activeCustomKey = isCustomSystem ? dynamicSystem : 'custom';
    const activeAppliedCustomEquations = isCustomSystem ? customEquations[activeCustomKey] : customEquations.custom;
    const activeDraftCustomEquations = isCustomSystem ? draftCustomEquations[activeCustomKey] : draftCustomEquations.custom;
    const activeAppliedCustomParams = isCustomSystem ? customParams[activeCustomKey] : EMPTY_ARRAY;
    const activeDraftCustomParams = isCustomSystem ? draftCustomParams[activeCustomKey] : EMPTY_ARRAY;
    const appliedParamValidation = useMemo(() => {
        if (!isCustomSystem) {
            return DEFAULT_VALIDATION;
        }
        return normalizeParams(activeAppliedCustomParams);
    }, [isCustomSystem, activeAppliedCustomParams]);
    const draftParamValidation = useMemo(() => {
        if (!isCustomSystem) {
            return DEFAULT_VALIDATION;
        }
        return normalizeParams(activeDraftCustomParams);
    }, [isCustomSystem, activeDraftCustomParams]);
    const hasPendingInputChanges = useMemo(() => {
        const paramsDirty = JSON.stringify(draftParams) !== JSON.stringify(params);
        const periodicSearchDirty = supportsPeriodicSearchSettings && (
            draftPeriodicSearchSettings.gridSize !== periodicSearchSettings.gridSize
            || draftPeriodicSearchSettings.thetaGridSize !== periodicSearchSettings.thetaGridSize
            || draftPeriodicSearchSettings.residualThreshold !== periodicSearchSettings.residualThreshold
            || draftPeriodicSearchSettings.useContinuation !== periodicSearchSettings.useContinuation
        );
        if (!isCustomSystem) {
            return paramsDirty || periodicSearchDirty;
        }
        const equationsDirty = JSON.stringify(activeDraftCustomEquations) !== JSON.stringify(activeAppliedCustomEquations);
        const customParamsDirty = JSON.stringify(activeDraftCustomParams) !== JSON.stringify(activeAppliedCustomParams);
        return paramsDirty || equationsDirty || customParamsDirty || periodicSearchDirty;
    }, [
        draftParams,
        params,
        isCustomSystem,
        supportsPeriodicSearchSettings,
        draftPeriodicSearchSettings,
        periodicSearchSettings,
        activeDraftCustomEquations,
        activeAppliedCustomEquations,
        activeDraftCustomParams,
        activeAppliedCustomParams
    ]);

    // Parameter sweep state
    const [sweepState, setSweepState] = useState({
        results: null,
        isComputing: false,
        error: null,
        sweepParam: 'a',
        sweepMin: 0.1,
        sweepMax: 2.0,
        numSamples: 10,
        maxPeriod: 3,
    });

    // BDE Simulator
    const [bdeState, setBdeState] = useState({
        points: [],
        isRunning: false
    });
    const bdeSimRef = useRef(null);
    const bdeAnimRef = useRef(null);
    const postApplyActionRef = useRef(null);

    // Ulam method state
    const [ulamState, setUlamState] = useState({
        gridBoxes: [],
        invariantMeasure: null,
        leftEigenvector: null, // backward invariant measure
        transitions: null, // array of {index, probability}
        selectedBoxIndex: null,
        currentBoxIndex: -1,
        isComputing: false,
        subdivisions: 20,
        pointsPerBox: 64, // 8x8 grid per box
        epsilon: 0.05, // epsilon ball radius for set-valued transitions
        showUlamOverlay: false,
        showTransitions: true,
        showCurrentBox: true, // highlight box containing current trajectory point
        needsRecompute: false // flag for auto-recompute
    });

    // Parameter animation state for manifold mode
    const [animationState, setAnimationState] = useState({
        isAnimating: false,
        parameter: 'a', // 'a', 'b', or 'epsilon'
        rangeValue: 0.1, // the range amount (e.g., 0.1 means go from current to current+0.1 or current-0.1)
        direction: 1, // +1 for positive direction, -1 for negative direction
        steps: 10, // number of steps to divide the range
        currentStep: 0, // current step in the animation
        baseValue: null, // the original value when animation started
        targetValue: null // the target value at the end of animation
    });

    // Video recording state
    const [recordingState, setRecordingState] = useState({
        isRecording: false,
        isEncoding: false,
        frameCount: 0,
        recordingEnabled: false, // toggle for recording with animation
        encodingProgress: 0,
        error: null
    });

    const recordedFramesRef = useRef([]);
    const encoderWorkerRef = useRef(null);

    const computeWorkerRef = useRef(null);
    const computeWorkerRequestIdRef = useRef(0);
    const computeWorkerPendingRef = useRef(new Map());
    const ulamDebounceRef = useRef(null);
    const hittingContourDebounceRef = useRef(null);
    const hittingContourRequestRef = useRef(0);
    const ulamSupportRef = useRef(null);
    const ulamTransitionsRequestRef = useRef(0);

    const [tooltip, setTooltip] = useState({
        visible: false,
        x: 0,
        y: 0,
        data: null
    });
    const [viewRange, setViewRange] = useState(DEFAULT_VIEW_RANGE);
    const viewRangeRef = useRef(viewRange);
    const gridGroupRef = useRef(null);

    const raycasterRef = useRef(new THREE.Raycaster());
    const mouseRef = useRef(new THREE.Vector2());

    const initComputeWorker = useCallback(() => {
        if (computeWorkerRef.current) {
            return computeWorkerRef.current;
        }

        const worker = new Worker(
            new URL('./compute.worker.js', import.meta.url),
            { type: 'module' }
        );

        worker.onmessage = (event) => {
            const { id, ok, result, error } = event.data || {};
            if (typeof id !== 'number') return;
            const pending = computeWorkerPendingRef.current.get(id);
            if (!pending) return;
            computeWorkerPendingRef.current.delete(id);

            if (ok) {
                pending.resolve(result);
            } else {
                pending.reject(new Error(error || 'Worker task failed'));
            }
        };

        worker.onerror = (event) => {
            const err = new Error(event?.message || 'Compute worker error');
            computeWorkerPendingRef.current.forEach(({ reject }) => reject(err));
            computeWorkerPendingRef.current.clear();
        };

        computeWorkerRef.current = worker;
        return worker;
    }, []);

    const runComputeTask = useCallback((kind, payload) => {
        return new Promise((resolve, reject) => {
            const worker = initComputeWorker();
            const requestId = ++computeWorkerRequestIdRef.current;
            computeWorkerPendingRef.current.set(requestId, { resolve, reject });
            worker.postMessage({ id: requestId, kind, payload });
        });
    }, [initComputeWorker]);

    const updatePeriodicSearchSettings = useCallback((patch) => {
        setDraftPeriodicSearchSettings(prev => normalizePeriodicSearchSettings({ ...prev, ...patch }, prev));
    }, []);


    const updateViewRange = useCallback((patch) => {
        setViewRange(prev => {
            const next = { ...prev };
            Object.entries(patch).forEach(([key, value]) => {
                if (Number.isFinite(value)) {
                    next[key] = value;
                }
            });
            return normalizeViewRange(next);
        });
    }, []);

    const resetViewRange = useCallback(() => {
        setViewRange(DEFAULT_VIEW_RANGE);
    }, []);

    const applyViewRangeToCamera = useCallback((range) => {
        const camera = cameraRef.current;
        if (!camera) return;

        const gridHeight = range.yMax - range.yMin;
        const padding = 0.5;
        const viewWidth = window.innerWidth - 268;
        const aspect = viewWidth / window.innerHeight;
        const frustumHeight = gridHeight + padding * 2;
        const frustumWidth = frustumHeight * aspect;

        camera.left = -frustumWidth / 2;
        camera.right = frustumWidth / 2;
        camera.top = frustumHeight / 2;
        camera.bottom = -frustumHeight / 2;

        const centerX = (range.xMin + range.xMax) / 2;
        const centerY = (range.yMin + range.yMax) / 2;
        camera.position.set(centerX, centerY, 5);
        camera.lookAt(centerX, centerY, 0);
        camera.updateProjectionMatrix();
    }, []);

    useEffect(() => {
        if (!canvasRef.current) return;

        const scene = new THREE.Scene();
        sceneRef.current = scene;

        const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 1000);
        camera.position.z = 5;
        cameraRef.current = camera;

        const renderer = new THREE.WebGLRenderer({
            canvas: canvasRef.current,
            antialias: true,
            alpha: true,
            preserveDrawingBuffer: true
        });
        renderer.setSize(window.innerWidth - 268, window.innerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        rendererRef.current = renderer;

        viewRangeRef.current = viewRange;
        applyViewRangeToCamera(viewRange);
        gridGroupRef.current = createCoordinateSystem(scene, viewRange);

        const handleResize = () => {
            const range = viewRangeRef.current;
            applyViewRangeToCamera(range);
            renderer.setSize(window.innerWidth - 268, window.innerHeight);
        };
        window.addEventListener('resize', handleResize);

        const animate = () => {
            animationFrameRef.current = requestAnimationFrame(animate);
            renderer.render(scene, camera);
        };
        animate();

        return () => {
            window.removeEventListener('resize', handleResize);
            if (animationFrameRef.current) cancelAnimationFrame(animationFrameRef.current);
            if (batchAnimationRef.current) cancelAnimationFrame(batchAnimationRef.current);
            renderer.dispose();
        };
    }, [applyViewRangeToCamera]);

    useEffect(() => {
        viewRangeRef.current = viewRange;
        const scene = sceneRef.current;
        if (!scene) return;
        if (gridGroupRef.current) {
            scene.remove(gridGroupRef.current);
        }
        gridGroupRef.current = createCoordinateSystem(scene, viewRange);
        applyViewRangeToCamera(viewRange);
    }, [viewRange, applyViewRangeToCamera]);

    useEffect(() => {
        const loadWasm = async () => {
            try {
                const wasm = await import('../pkg/henon_periodic_orbits.js');
                await wasm.default();
                setWasmModule(wasm);
                console.log('WASM module loaded successfully');
            } catch (err) {
                console.error('Failed to load WASM module:', err);
            }
        };
        loadWasm();
    }, []);

    useEffect(() => {
        return () => {
            computeWorkerPendingRef.current.forEach(({ reject }) => {
                reject(new Error('Compute worker terminated'));
            });
            computeWorkerPendingRef.current.clear();
            if (computeWorkerRef.current) {
                computeWorkerRef.current.terminate();
                computeWorkerRef.current = null;
            }
        };
    }, []);

    const validateCustomDraft = useCallback((candidateEquations, candidateParams, candidateEpsilon) => {
        if (!isCustomSystem) {
            return { valid: true };
        }
        if (!draftParamValidation.valid) {
            const firstError = draftParamValidation.errors.find(Boolean);
            return { valid: false, error: firstError ? `Parameter error: ${firstError}` : 'Invalid parameters' };
        }
        if (!wasmModule) {
            return { valid: true };
        }
        try {
            if (dynamicSystem === 'custom') {
                const result = wasmModule.evaluate_user_defined_map(
                    0.5, 0.5,
                    candidateEquations.xEq, candidateEquations.yEq,
                    candidateParams,
                    candidateEpsilon
                );
                if (result && isFinite(result.x) && isFinite(result.y)) {
                    return { valid: true };
                }
            } else {
                const result = wasmModule.evaluate_user_defined_ode(
                    0.5, 0.5,
                    candidateEquations.xEq, candidateEquations.yEq,
                    candidateParams
                );
                if (result && isFinite(result.x) && isFinite(result.y)) {
                    return { valid: true };
                }
            }
            return { valid: false, error: 'Equations produce non-finite values' };
        } catch (err) {
            return { valid: false, error: String(err).replace('Error: ', '') };
        }
    }, [draftParamValidation, dynamicSystem, isCustomSystem, wasmModule]);

    const applyInputsAndRecompute = useCallback(() => {
        const nextDraftParams = {
            ...draftParams,
            a: clampToRange(draftParams.a, -PARAM_ABS_LIMIT, PARAM_ABS_LIMIT, params.a),
            b: clampToRange(draftParams.b, -PARAM_ABS_LIMIT, PARAM_ABS_LIMIT, params.b)
        };
        if (nextDraftParams.a !== draftParams.a || nextDraftParams.b !== draftParams.b) {
            setDraftParams(prev => ({
                ...prev,
                a: nextDraftParams.a,
                b: nextDraftParams.b
            }));
        }
        if (isCustomSystem) {
            const nextDraftEquations = draftCustomEquations[activeCustomKey];
            const nextDraftCustomParams = draftParamValidation.normalized;
            const validation = validateCustomDraft(nextDraftEquations, nextDraftCustomParams, nextDraftParams.epsilon);
            if (!validation.valid) {
                setEquationError(validation.error);
                return;
            }
            setCustomEquations(prev => ({
                ...prev,
                [activeCustomKey]: {
                    xEq: nextDraftEquations.xEq,
                    yEq: nextDraftEquations.yEq
                }
            }));
            setCustomParams(prev => ({
                ...prev,
                [activeCustomKey]: nextDraftCustomParams
            }));
        }
        setEquationError(null);
        setParams(nextDraftParams);
        setPeriodicSearchSettings(draftPeriodicSearchSettings);
        setComputeRequestId(prev => prev + 1);
        return true;
    }, [draftParams, params.a, params.b, isCustomSystem, draftCustomEquations, activeCustomKey, draftParamValidation, validateCustomDraft, draftPeriodicSearchSettings]);

    const applyIfNeededBeforeAction = useCallback((action) => {
        if (!hasPendingInputChanges) {
            return false;
        }
        postApplyActionRef.current = action;
        const applied = applyInputsAndRecompute();
        if (!applied) {
            postApplyActionRef.current = null;
        }
        return applied;
    }, [hasPendingInputChanges, applyInputsAndRecompute]);

    useEffect(() => {
        if (!equationError) return;
        setEquationError(null);
    }, [draftCustomEquations, draftCustomParams, draftParams.epsilon, dynamicSystem]);

    const systemLabel = useMemo(() => {
        const labels = { henon: 'Hénon', duffing: 'Duffing Map', duffing_ode: 'Duffing ODE', custom: 'Custom', custom_ode: 'Custom ODE' };
        return labels[dynamicSystem] || dynamicSystem;
    }, [dynamicSystem]);

    const paramOverlayText = useMemo(() => {
        if (dynamicSystem === 'duffing_ode') {
            return `δ = ${(params.delta || 0).toFixed(4)}  h = ${(params.h || 0).toFixed(4)}  ε = ${(params.epsilon || 0).toFixed(4)}`;
        } else if (dynamicSystem === 'custom_ode') {
            const cp = (customParams.custom_ode || []).map(p => `${p.name} = ${p.value.toFixed(4)}`).join('  ');
            return cp || `h = ${(params.h || 0).toFixed(4)}  ε = ${(params.epsilon || 0).toFixed(4)}`;
        } else if (dynamicSystem === 'custom') {
            const cp = (customParams.custom || []).map(p => `${p.name} = ${p.value.toFixed(4)}`).join('  ');
            return cp || `ε = ${(params.epsilon || 0).toFixed(4)}`;
        }
        return `a = ${(params.a || 0).toFixed(4)}  b = ${(params.b || 0).toFixed(4)}  ε = ${(params.epsilon || 0).toFixed(4)}`;
    }, [dynamicSystem, params, customParams]);

    const systemFilePrefix = useMemo(() => {
        const prefixes = { henon: 'henon', duffing: 'duffing_map', duffing_ode: 'duffing_ode', custom: 'custom', custom_ode: 'custom_ode' };
        return prefixes[dynamicSystem] || 'system';
    }, [dynamicSystem]);

    const paramFileString = useMemo(() => {
        if (dynamicSystem === 'duffing_ode') {
            const dStr = (params.delta || 0).toFixed(3).replace('.', 'p').replace('-', 'm');
            const hStr = (params.h || 0).toFixed(3).replace('.', 'p').replace('-', 'm');
            const epsStr = (params.epsilon || 0).toFixed(4).replace('.', 'p').replace('-', 'm');
            return `d${dStr}_h${hStr}_eps${epsStr}`;
        }
        const aStr = (params.a || 0).toFixed(3).replace('.', 'p').replace('-', 'm');
        const bStr = (params.b || 0).toFixed(3).replace('.', 'p').replace('-', 'm');
        const epsStr = (params.epsilon || 0).toFixed(4).replace('.', 'p').replace('-', 'm');
        return `a${aStr}_b${bStr}_eps${epsStr}`;
    }, [dynamicSystem, params]);

    useEffect(() => {
        let paramPatch = null;
        if (dynamicSystem === 'duffing') {
            paramPatch = { a: 2.75, b: 0.2 };
        } else if (dynamicSystem === 'duffing_ode') {
            paramPatch = { delta: 0.15, h: 0.05, epsilon: 0.1 };
        } else if (dynamicSystem === 'custom_ode') {
            paramPatch = { h: 0.05, epsilon: 0.1 };
        } else if (dynamicSystem === 'custom') {
            paramPatch = { maxPeriod: 3 };
        } else {
            paramPatch = { a: 0.4, b: 0.3 };
        }
        if (paramPatch) {
            setParams(prev => ({ ...prev, ...paramPatch }));
            setDraftParams(prev => ({ ...prev, ...paramPatch }));
        }

        setManifoldState(prev => ({
            ...prev,
            isRunning: false,
            hasStarted: false,
            iteration: 0,
            trajectoryPoints: [],
            manifolds: [],
            stableManifolds: [],
            fixedPoints: [],
            intersections: [],
            showUnstableManifold: false,
            showStableManifold: false,
            showOrbits: true,
            showOrbitLines: false,
            showTrail: true,
        }));
        setBdeState(prev => ({
            ...prev,
            isRunning: false,
            points: []
        }));
        setPeriodicState(prev => ({
            ...prev,
            orbits: [],
            showOrbits: false,
        }));
        setUlamState(prev => ({
            ...prev,
            showUlamOverlay: false,
            gridBoxes: [],
            invariantMeasure: null,
            transitions: null,
            currentBoxIndex: -1,
            selectedBoxIndex: null,
        }));
        setSweepState(prev => ({
            ...prev,
            results: null,
            error: null,
        }));
    }, [dynamicSystem]);

    const computeJacobian = useCallback((x, y) => {
        if (dynamicSystem === 'duffing_ode') {
            const h = params.h;
            const delta = params.delta;
            // Df = [[0, 1], [1 - 3*x^2, -delta]]
            // DF_h = I + h * Df
            const j11 = 1.0;
            const j12 = h;
            const j21 = h * (1.0 - 3.0 * x * x);
            const j22 = 1.0 - h * delta;
            const trace = j11 + j22;
            const det = j11 * j22 - j12 * j21;
            return { j11, j12, j21, j22, trace, det };
        }

        if (dynamicSystem === 'custom' && wasmModule && appliedParamValidation.valid) {
            const h = 1e-5;
            const evalMap = (xv, yv) => wasmModule.evaluate_user_defined_map(
                xv, yv,
                activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                appliedParamValidation.normalized,
                params.epsilon
            );
            const f1 = evalMap(x + h, y);
            const f2 = evalMap(x - h, y);
            const f3 = evalMap(x, y + h);
            const f4 = evalMap(x, y - h);
            if (!f1 || !f2 || !f3 || !f4) {
                return { j11: 0, j12: 0, j21: 0, j22: 0, trace: 0, det: 0 };
            }
            const j11 = (f1.x - f2.x) / (2 * h);
            const j12 = (f3.x - f4.x) / (2 * h);
            const j21 = (f1.y - f2.y) / (2 * h);
            const j22 = (f3.y - f4.y) / (2 * h);
            const trace = j11 + j22;
            const det = j11 * j22 - j12 * j21;
            return { j11, j12, j21, j22, trace, det };
        }

        if (dynamicSystem === 'custom_ode' && wasmModule && appliedParamValidation.valid) {
            const h = 1e-5;
            const evalVF = (xv, yv) => wasmModule.evaluate_user_defined_ode(
                xv, yv,
                activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                appliedParamValidation.normalized
            );
            const f1 = evalVF(x + h, y);
            const f2 = evalVF(x - h, y);
            const f3 = evalVF(x, y + h);
            const f4 = evalVF(x, y - h);
            if (!f1 || !f2 || !f3 || !f4) {
                return { j11: 0, j12: 0, j21: 0, j22: 0, trace: 0, det: 0 };
            }
            const dfx_dx = (f1.x - f2.x) / (2 * h);
            const dfx_dy = (f3.x - f4.x) / (2 * h);
            const dfy_dx = (f1.y - f2.y) / (2 * h);
            const dfy_dy = (f3.y - f4.y) / (2 * h);
            const step = params.h;
            const j11 = 1.0 + step * dfx_dx;
            const j12 = step * dfx_dy;
            const j21 = step * dfy_dx;
            const j22 = 1.0 + step * dfy_dy;
            const trace = j11 + j22;
            const det = j11 * j22 - j12 * j21;
            return { j11, j12, j21, j22, trace, det };
        }

        const a = params.a;
        const b = params.b;
        const j11 = -2 * a * x;
        const j12 = 1;
        const j21 = b;
        const j22 = 0;
        const trace = j11 + j22;
        const det = j11 * j22 - j12 * j21;
        return { j11, j12, j21, j22, trace, det };
    }, [params.a, params.b, params.delta, params.h, params.epsilon, dynamicSystem, activeAppliedCustomEquations, appliedParamValidation, wasmModule]);

    const handleMouseMove = useCallback((event) => {
        if (!canvasRef.current || !sceneRef.current || !cameraRef.current) return;

        const rect = canvasRef.current.getBoundingClientRect();
        mouseRef.current.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        mouseRef.current.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

        raycasterRef.current.setFromCamera(mouseRef.current, cameraRef.current);

        if (hittingContourState.showOverlay && hittingContourState.result) {
            const levelMesh = sceneRef.current.getObjectByName('hitting-level-cells');
            if (levelMesh) {
                const intersects = raycasterRef.current.intersectObject(levelMesh);
                if (intersects.length > 0 && intersects[0].instanceId !== undefined) {
                    const entry = levelMesh.userData.entries?.[intersects[0].instanceId];
                    if (entry) {
                        setTooltip({
                            visible: true,
                            x: event.clientX,
                            y: event.clientY,
                            data: {
                                type: 'Hitting Level',
                                cell: entry.cell,
                                hit: entry.hit,
                                hitCount: entry.cell.hits?.length || 0,
                                radius: hittingContourState.result.settings?.hitRadius
                            }
                        });
                        return;
                    }
                }
            }
        }

        if (ulamState.showUlamOverlay && ulamState.gridBoxes.length > 0) {
            const ulamMesh = sceneRef.current.getObjectByName('ulam-grid');
            if (ulamMesh) {
                const intersects = raycasterRef.current.intersectObject(ulamMesh);
                if (intersects.length > 0 && intersects[0].instanceId !== undefined) {
                    const boxIndex = intersects[0].instanceId;
                    const box = ulamState.gridBoxes[boxIndex];
                    const measure = ulamState.invariantMeasure ? ulamState.invariantMeasure[boxIndex] : 0;
                    const maxMeasure = ulamState.invariantMeasure ? Math.max(...ulamState.invariantMeasure) : 1;

                    let numTransitions = 0;
                    let topTransitions = [];
                    if (ulamState.selectedBoxIndex === boxIndex && Array.isArray(ulamState.transitions)) {
                        numTransitions = ulamState.transitions.length;
                        topTransitions = [...ulamState.transitions]
                            .sort((a, b) => (b.probability || 0) - (a.probability || 0))
                            .slice(0, 3);
                    }

                    setTooltip({
                        visible: true,
                        x: event.clientX,
                        y: event.clientY,
                        data: {
                            type: 'Ulam Box',
                            boxIndex: boxIndex,
                            pos: { x: box.center[0], y: box.center[1] },
                            measure: measure,
                            measurePercent: maxMeasure > 0 ? (measure / maxMeasure * 100) : 0,
                            numTransitions: numTransitions,
                            topTransitions: topTransitions,
                            isCurrentBox: boxIndex === ulamState.currentBoxIndex
                        }
                    });
                    return;
                }
            }
        }

        const meshes = [];
        sceneRef.current.traverse((obj) => {
            if (obj.isMesh && (
                obj.userData.type === 'orbit'
                || obj.userData.type === 'fixedPoint'
                || obj.userData.type === 'hittingTarget'
                || obj.userData.type === 'hittingContourRing'
                || obj.userData.type === 'hittingLevelPoint'
            )) {
                meshes.push(obj);
            }
        });

        const intersects = raycasterRef.current.intersectObjects(meshes, false);

        if (intersects.length > 0) {
            const hit = intersects[0].object;
            const data = hit.userData;
            if (data.type === 'hittingTarget') {
                setTooltip({
                    visible: true,
                    x: event.clientX,
                    y: event.clientY,
                    data: {
                        type: 'Hitting Target',
                        target: data.target,
                        topographic: data.topographic
                    }
                });
                return;
            }

            if (data.type === 'hittingContourRing') {
                setTooltip({
                    visible: true,
                    x: event.clientX,
                    y: event.clientY,
                    data: {
                        type: 'Hitting Contour Ring',
                        target: data.target,
                        level: data.level,
                        count: data.count,
                        points: data.points || []
                    }
                });
                return;
            }

            if (data.type === 'hittingLevelPoint') {
                const point = data.points?.[intersects[0].instanceId];
                if (point) {
                    setTooltip({
                        visible: true,
                        x: event.clientX,
                        y: event.clientY,
                        data: {
                            type: 'Hitting Level Point',
                            target: data.target,
                            level: data.level,
                            point
                        }
                    });
                    return;
                }
            }

            const jac = computeJacobian(data.pos.x, data.pos.y);

            if (data.type === 'orbit' && data.orbitId && manifoldState.showOrbitLines) {
                setManifoldState(prev => prev.highlightedOrbitId !== data.orbitId
                    ? { ...prev, highlightedOrbitId: data.orbitId }
                    : prev);
            }

            setTooltip({
                visible: true,
                x: event.clientX,
                y: event.clientY,
                data: {
                    type: data.type === 'fixedPoint' ? 'Fixed Point' : 'Periodic Point',
                    period: data.period,
                    stability: data.stability,
                    pos: data.pos,
                    eigenvalues: data.eigenvalues,
                    jacobian: jac,
                    orbitSize: data.orbitPoints?.length || 1
                }
            });
        } else {
            setTooltip(prev => prev.visible ? { ...prev, visible: false } : prev);
            if (manifoldState.highlightedOrbitId !== null) {
                setManifoldState(prev => prev.highlightedOrbitId !== null
                    ? { ...prev, highlightedOrbitId: null }
                    : prev);
            }
        }
    }, [computeJacobian, hittingContourState.showOverlay, hittingContourState.result, ulamState.showUlamOverlay, ulamState.gridBoxes, ulamState.invariantMeasure, ulamState.currentBoxIndex, ulamState.selectedBoxIndex, ulamState.transitions, manifoldState.showOrbitLines, manifoldState.highlightedOrbitId]);

    const requestUlamTransitions = useCallback((index, mode = 'selected') => {
        if (index < 0) return;
        const requestId = ++ulamTransitionsRequestRef.current;
        runComputeTask('getUlamTransitions', { index }).then((transitions) => {
            if (requestId !== ulamTransitionsRequestRef.current) return;
            setUlamState(prev => {
                if (mode === 'selected' && prev.selectedBoxIndex !== index) return prev;
                if (mode === 'current' && (prev.currentBoxIndex !== index || !prev.showCurrentBox)) return prev;
                return { ...prev, transitions: transitions || [] };
            });
        }).catch((err) => {
            console.warn('Failed to fetch Ulam transitions:', err);
        });
    }, [runComputeTask]);

    const handleUlamClick = useCallback((index) => {
        setUlamState(prev => ({
            ...prev,
            selectedBoxIndex: index,
            transitions: null
        }));
        requestUlamTransitions(index, 'selected');
    }, [requestUlamTransitions]);

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const handleClick = (event) => {
            const rect = canvas.getBoundingClientRect();
            const x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
            const y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

            if (ulamState.showUlamOverlay && ulamState.gridBoxes.length) {
                raycasterRef.current.setFromCamera(new THREE.Vector2(x, y), cameraRef.current);
                const scene = sceneRef.current;
                const ulamMesh = scene.getObjectByName('ulam-grid');

                if (ulamMesh) {
                    const intersects = raycasterRef.current.intersectObject(ulamMesh);
                    if (intersects.length > 0) {
                        const instanceId = intersects[0].instanceId;
                        if (instanceId !== undefined) {
                            handleUlamClick(instanceId);
                            return;
                        }
                    } else {
                        setUlamState(prev => ({ ...prev, selectedBoxIndex: null, transitions: null }));
                        return;
                    }
                }
            }

            raycasterRef.current.setFromCamera(new THREE.Vector2(x, y), cameraRef.current);
            const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
            const target = new THREE.Vector3();
            raycasterRef.current.ray.intersectPlane(plane, target);
            if (target) {
                setManifoldState(prev => {
                    const newStart = { ...prev.startPoint, x: target.x, y: target.y };
                    return applyStartPointUpdate(prev, newStart);
                });
            }
        };

        canvas.addEventListener('mousemove', handleMouseMove);
        canvas.addEventListener('click', handleClick);

        return () => {
            canvas.removeEventListener('mousemove', handleMouseMove);
            canvas.removeEventListener('click', handleClick);
        };
    }, [handleMouseMove, ulamState.showUlamOverlay, ulamState.gridBoxes.length, handleUlamClick]);

    useEffect(() => {
        if (!wasmModule) return;

        let cancelled = false;
        if (dynamicSystem === 'custom' || dynamicSystem === 'custom_ode') {
            ulamSupportRef.current = null;
            setPeriodicState(prev => ({ ...prev, isReady: true, orbits: [], computeMethod: null }));
            return;
        }
        setPeriodicState(prev => ({ ...prev, isReady: false, orbits: [], showOrbits: false, computeMethod: null }));

        const initSystem = async () => {
            try {
                if (cancelled) return;
                const result = await runComputeTask('computePeriodic', {
                    dynamicSystem,
                    params: {
                        a: params.a,
                        b: params.b,
                        delta: params.delta,
                        h: params.h,
                        epsilon: params.epsilon,
                        maxPeriod: params.maxPeriod
                    },
                    viewRange: {
                        xMin: viewRange.xMin,
                        xMax: viewRange.xMax,
                        yMin: viewRange.yMin,
                        yMax: viewRange.yMax
                    },
                    periodicSearchSettings: {
                        gridSize: periodicSearchSettings.gridSize,
                        thetaGridSize: periodicSearchSettings.thetaGridSize,
                        residualThreshold: periodicSearchSettings.residualThreshold,
                        useContinuation: periodicSearchSettings.useContinuation
                    }
                });
                if (cancelled) return;

                ulamSupportRef.current = dynamicSystem === 'henon' ? (result?.support || null) : null;
                setPeriodicState(prev => ({
                    ...prev,
                    orbits: result?.orbits || [],
                    isReady: true,
                    computeMethod: result?.usedContinuation ? 'continuation' : 'grid'
                }));
            } catch (err) {
                if (cancelled) return;
                console.error('Failed to compute periodic orbits:', err);
                ulamSupportRef.current = null;
                setPeriodicState(prev => ({ ...prev, isReady: true, orbits: [], computeMethod: null }));
            }
        };

        initSystem();
        return () => { cancelled = true; };
    }, [wasmModule, dynamicSystem, params.a, params.b, params.delta, params.h, params.epsilon, params.maxPeriod, params.startX, params.startY, viewRange, periodicSearchSettings.gridSize, periodicSearchSettings.thetaGridSize, periodicSearchSettings.residualThreshold, periodicSearchSettings.useContinuation, computeRequestId, runComputeTask]);

    useEffect(() => {
        if (manifoldDebounceRef.current) {
            clearTimeout(manifoldDebounceRef.current);
        }
        let cancelled = false;

        setManifoldState(prev => ({ ...prev, isComputing: true }));

        manifoldDebounceRef.current = setTimeout(() => {
            if (!wasmModule) return;
            const support = dynamicSystem === 'henon' ? ulamSupportRef.current : null;

            const manifoldsEnabled = manifoldState.showUnstableManifold || manifoldState.showStableManifold;

            if (!manifoldsEnabled && (dynamicSystem === 'henon' || dynamicSystem === 'duffing' || dynamicSystem === 'custom')) {
                const orbits = periodicState.orbits || [];
                const fixedPoints = orbits
                    .filter(o => o.period === 1)
                    .map(o => ({
                        x: o.points[0][0],
                        y: o.points[0][1],
                        stability: o.stability.toLowerCase(),
                        eigenvalues: o.eigenvalues || [0, 0]
                    }));

                setManifoldState(prev => ({
                    ...prev,
                    manifolds: [],
                    stableManifolds: [],
                    fixedPoints: fixedPoints,
                    intersections: [],
                    isComputing: false,
                    isReady: true
                }));
                return;
            }
            if ((dynamicSystem === 'custom' || dynamicSystem === 'custom_ode') && !appliedParamValidation.valid) {
                setManifoldState(prev => ({
                    ...prev,
                    isComputing: false,
                    isReady: true,
                    manifolds: [],
                    stableManifolds: [],
                    fixedPoints: [],
                    intersections: []
                }));
                return;
            }
            if (dynamicSystem === 'custom_ode') {
                console.log('Initializing user-defined ODE BDE flow simulation');
                if (bdeSimRef.current) {
                    bdeSimRef.current.free();
                }
                bdeSimRef.current = new wasmModule.BdeSimulatorUserDefinedWasm(
                    activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                    appliedParamValidation.normalized,
                    params.epsilon,
                    manifoldState.startPoint.x, manifoldState.startPoint.y, 0.05, 1000
                );
                setBdeState(prev => ({ ...prev, points: bdeSimRef.current.get_points(), isRunning: false }));
                if (bdeAnimRef.current) cancelAnimationFrame(bdeAnimRef.current);

                setManifoldState(prev => ({
                    ...prev,
                    manifolds: [],
                    stableManifolds: [],
                    fixedPoints: [],
                    isComputing: false,
                    isReady: true
                }));
                return;
            }
            if (dynamicSystem === 'duffing_ode') {
                try {
                    console.log('Initializing Duffing ODE BDE flow simulation');
                    if (bdeSimRef.current) {
                        bdeSimRef.current.free();
                    }
                    bdeSimRef.current = new wasmModule.BdeSimulatorWasm(
                        params.delta,
                        params.epsilon,
                        manifoldState.startPoint.x, manifoldState.startPoint.y, 0.05, 1000
                    );
                    setBdeState(prev => ({ ...prev, points: bdeSimRef.current.get_points(), isRunning: false }));
                    if (bdeAnimRef.current) cancelAnimationFrame(bdeAnimRef.current);

                    setManifoldState(prev => ({
                        ...prev,
                        manifolds: [],
                        stableManifolds: [],
                        fixedPoints: [],
                        isComputing: false,
                        isReady: true
                    }));
                } catch (err) {
                    console.error('Manifold computation error:', err);
                    setManifoldState(prev => ({ ...prev, isComputing: false }));
                }
                return;
            }

            runComputeTask('computeManifolds', {
                dynamicSystem,
                params: {
                    a: params.a,
                    b: params.b,
                    epsilon: params.epsilon
                },
                viewRange: {
                    xMin: viewRange.xMin,
                    xMax: viewRange.xMax,
                    yMin: viewRange.yMin,
                    yMax: viewRange.yMax
                },
                periodicOrbits: periodicState.orbits || [],
                customEquations: activeAppliedCustomEquations,
                customParams: appliedParamValidation.normalized,
                showStableManifold: manifoldState.showStableManifold,
                showUnstableManifold: manifoldState.showUnstableManifold,
                intersectionThreshold: manifoldState.intersectionThreshold
            }).then((result) => {
                if (cancelled) return;
                setManifoldState(prev => ({
                    ...prev,
                    manifolds: clipManifoldsBySupport(result?.manifolds || [], support),
                    stableManifolds: clipManifoldsBySupport(result?.stableManifolds || [], support),
                    fixedPoints: result?.fixedPoints || [],
                    intersections: result?.intersections || [],
                    isComputing: false,
                    isReady: true
                }));
            }).catch((err) => {
                if (cancelled) return;
                console.error('Manifold computation error:', err);
                setManifoldState(prev => ({ ...prev, isComputing: false }));
            });
        }, 500);

        return () => {
            cancelled = true;
            if (manifoldDebounceRef.current) {
                clearTimeout(manifoldDebounceRef.current);
            }
        };
    }, [dynamicSystem, params.a, params.b, params.delta, params.h, params.epsilon, periodicState.orbits, wasmModule, manifoldState.showStableManifold, manifoldState.showUnstableManifold, manifoldState.intersectionThreshold, activeAppliedCustomEquations, appliedParamValidation, manifoldState.startPoint.x, manifoldState.startPoint.y, viewRange, computeRequestId, runComputeTask]);

    useEffect(() => {
        if (!animationState.isAnimating) {
            return;
        }

        if (manifoldState.isComputing) {
            return;
        }

        const { parameter, rangeValue, direction, steps, currentStep, baseValue } = animationState;

        if (currentStep >= steps) {
            setAnimationState(prev => ({
                ...prev,
                isAnimating: false
            }));
            return;
        }

        const stepSize = rangeValue / steps;
        const nextStep = currentStep + 1;
        const nextValue = baseValue + (direction * stepSize * nextStep);

        setParams(p => ({ ...p, [parameter]: parseFloat(nextValue.toFixed(4)) }));
        setAnimationState(prev => ({
            ...prev,
            currentStep: nextStep
        }));

    }, [animationState.isAnimating, animationState.currentStep, manifoldState.isComputing]);

    const startAnimation = useCallback(async () => {
        const baseVal = params[animationState.parameter];
        const targetVal = baseVal + (animationState.direction * animationState.rangeValue);

        if (recordingState.recordingEnabled && canvasRef.current) {
            try {
                const canvas = canvasRef.current;
                const width = 1280;
                const height = 720;
                const offscreen = new OffscreenCanvas(width, height);
                const ctx = offscreen.getContext('2d');
                ctx.drawImage(canvas, 0, 0, width, height);

                ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
                ctx.fillRect(10, height - 40, 500, 30);
                ctx.font = 'bold 16px monospace';
                ctx.fillStyle = '#4CAF50';
                ctx.fillText(`${systemLabel} | ${paramOverlayText}`, 20, height - 18);

                const bitmap = await createImageBitmap(offscreen);
                recordedFramesRef.current.push(bitmap);
                setRecordingState(prev => ({ ...prev, frameCount: 1 }));
                console.log('Initial frame captured');
            } catch (err) {
                console.error('Initial frame capture error:', err);
            }
        }

        setAnimationState(prev => ({
            ...prev,
            isAnimating: true,
            baseValue: baseVal,
            targetValue: targetVal,
            currentStep: 0
        }));
    }, [params, animationState.parameter, animationState.direction, animationState.rangeValue, recordingState.recordingEnabled, systemLabel, paramOverlayText]);

    const stopAnimation = useCallback(() => {
        setAnimationState(prev => ({
            ...prev,
            isAnimating: false,
            currentStep: 0
        }));
    }, []);

    const captureFrame = useCallback(async () => {
        if (!canvasRef.current || !recordingState.recordingEnabled) return null;

        const canvas = canvasRef.current;
        const width = 1280;
        const height = 720;

        const offscreen = new OffscreenCanvas(width, height);
        const ctx = offscreen.getContext('2d');

        ctx.drawImage(canvas, 0, 0, width, height);

        ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
        ctx.fillRect(10, height - 40, 500, 30);
        ctx.font = 'bold 16px monospace';
        ctx.fillStyle = '#4CAF50';
        ctx.fillText(`${systemLabel} | ${paramOverlayText}`, 20, height - 18);

        const bitmap = await createImageBitmap(offscreen);
        return bitmap;
    }, [recordingState.recordingEnabled, systemLabel, paramOverlayText]);

    const initEncoderWorker = useCallback(() => {
        if (encoderWorkerRef.current) {
            encoderWorkerRef.current.terminate();
        }

        const worker = new Worker(
            new URL('./videoEncoder.worker.js', import.meta.url),
            { type: 'classic' }
        );

        worker.onmessage = (e) => {
            const { type, blob, frameCount, error } = e.data;

            switch (type) {
                case 'ready':
                    console.log('Encoder ready');
                    break;
                case 'progress':
                    setRecordingState(prev => ({ ...prev, frameCount: frameCount }));
                    break;
                case 'complete': {
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    const filename = generateFilename();
                    a.href = url;
                    a.download = filename;
                    document.body.appendChild(a);
                    a.click();
                    document.body.removeChild(a);
                    URL.revokeObjectURL(url);

                    setRecordingState(prev => ({
                        ...prev,
                        isEncoding: false,
                        isRecording: false,
                        recordingEnabled: false
                    }));
                    recordedFramesRef.current = [];
                    break;
                }
                case 'error':
                    console.error('Encoder error:', error);
                    setRecordingState(prev => ({ ...prev, error, isEncoding: false }));
                    break;
            }
        };

        encoderWorkerRef.current = worker;
        return worker;
    }, []);

    const generateFilename = useCallback(() => {
        const paramName = animationState.parameter;
        const startStr = (animationState.baseValue || 0).toFixed(3).replace('.', 'p').replace('-', 'm');
        const endStr = (animationState.targetValue || 0).toFixed(3).replace('.', 'p').replace('-', 'm');

        return `${systemFilePrefix}_${paramName}_${paramFileString}_${startStr}_to_${endStr}.mp4`;
    }, [systemFilePrefix, paramFileString, animationState.parameter, animationState.baseValue, animationState.targetValue]);

    const savePNG = useCallback(async () => {
        if (!canvasRef.current || !rendererRef.current || !sceneRef.current || !cameraRef.current) return;

        // Force a render to ensure canvas has the current frame
        rendererRef.current.render(sceneRef.current, cameraRef.current);

        const canvas = canvasRef.current;
        const width = 1920;
        const height = 1080;

        const offscreen = new OffscreenCanvas(width, height);
        const ctx = offscreen.getContext('2d');

        ctx.drawImage(canvas, 0, 0, width, height);

        ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
        ctx.fillRect(10, height - 80, 500, 70);

        ctx.font = 'bold 18px monospace';
        ctx.fillStyle = '#4CAF50';

        ctx.fillText(`${systemLabel} | Iteration: ${manifoldState.iteration}`, 20, height - 55);
        ctx.font = '14px monospace';
        ctx.fillStyle = '#aaa';
        const unstablePts = manifoldState.manifolds.reduce((sum, m) => sum + (m.points_positive?.length || 0) + (m.points_negative?.length || 0), 0);
        const orbitsInfo = periodicState.orbits.length > 0 ? `${periodicState.orbits.length} orbits, ` : '';
        ctx.fillText(`${orbitsInfo}${manifoldState.fixedPoints.length} fixed pts, ${unstablePts} manifold pts`, 20, height - 32);

        ctx.font = 'bold 14px monospace';
        ctx.fillStyle = '#4CAF50';
        ctx.fillText(paramOverlayText, 20, height - 12);

        const blob = await offscreen.convertToBlob({ type: 'image/png', quality: 1.0 });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');

        const iterStr = manifoldState.hasStarted ? `_iter${manifoldState.iteration}` : '';

        a.href = url;
        a.download = `${systemFilePrefix}_${paramFileString}${iterStr}.png`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }, [params, periodicState.orbits.length, manifoldState.iteration, manifoldState.hasStarted, manifoldState.manifolds, manifoldState.fixedPoints, systemLabel, paramOverlayText, systemFilePrefix, paramFileString]);

    const startEncoding = useCallback(async () => {
        if (recordedFramesRef.current.length === 0) {
            console.warn('No frames to encode');
            return;
        }

        setRecordingState(prev => ({ ...prev, isEncoding: true, encodingProgress: 0 }));

        const worker = initEncoderWorker();

        worker.postMessage({
            type: 'init',
            data: {
                width: 1280,
                height: 720,
                fps: 2,
                filename: generateFilename()
            }
        });

        await new Promise(resolve => setTimeout(resolve, 100));

        const fps = 2;
        const frameDuration = 1000000 / fps; // microseconds (500ms per frame)

        for (let i = 0; i < recordedFramesRef.current.length; i++) {
            const frame = recordedFramesRef.current[i];
            worker.postMessage({
                type: 'frame',
                data: {
                    imageData: frame,
                    timestamp: i * frameDuration,
                    duration: frameDuration
                }
            });

            if (i % 5 === 0) {
                await new Promise(resolve => setTimeout(resolve, 10));
            }
        }

        worker.postMessage({ type: 'finish' });
    }, [initEncoderWorker, generateFilename]);

    const wasComputingRef = useRef(false);

    useEffect(() => {
        const wasComputing = wasComputingRef.current;
        const isComputing = manifoldState.isComputing;
        wasComputingRef.current = isComputing;

        if (!recordingState.recordingEnabled || !animationState.isAnimating) {
            return;
        }

        if (wasComputing && !isComputing) {
            console.log(`[Recording] Manifold finished, capturing frame for step ${animationState.currentStep}...`);

            requestAnimationFrame(async () => {
                try {
                    const frame = await captureFrame();
                    if (frame) {
                        recordedFramesRef.current.push(frame);
                        setRecordingState(prev => ({ ...prev, frameCount: recordedFramesRef.current.length }));
                        console.log(`[Recording] Frame ${recordedFramesRef.current.length} captured`);
                    } else {
                        console.log('[Recording] captureFrame returned null');
                    }
                } catch (err) {
                    console.error('[Recording] Frame capture error:', err);
                }
            });
        }
    }, [manifoldState.isComputing, animationState.isAnimating, recordingState.recordingEnabled, animationState.currentStep, captureFrame]);

    useEffect(() => {
        if (!animationState.isAnimating && recordingState.recordingEnabled && recordedFramesRef.current.length > 0 && !recordingState.isEncoding) {
            console.log(`[Recording] Animation finished with ${recordedFramesRef.current.length} frames, starting encoding...`);
            startEncoding();
        }
    }, [animationState.isAnimating, recordingState.recordingEnabled, recordingState.isEncoding, startEncoding]);

    // Toggle recording mode
    const toggleRecording = useCallback(() => {
        if (recordingState.recordingEnabled) {
            // Disable recording
            setRecordingState(prev => ({ ...prev, recordingEnabled: false }));
            recordedFramesRef.current = [];
        } else {
            // Enable recording
            setRecordingState(prev => ({ ...prev, recordingEnabled: true, frameCount: 0, error: null }));
            recordedFramesRef.current = [];
        }
    }, [recordingState.recordingEnabled]);

    useEffect(() => {
        if (!sceneRef.current) return;
        const scene = sceneRef.current;

        const toRemove = [];
        scene.traverse(child => {
            if (child.userData.type === 'trajectory' || child.userData.type === 'orbit' || child.userData.type === 'orbitLine' || child.userData.type === 'manifold' || child.userData.type === 'fixedPoint' || child.userData.type === 'bde' || child.userData.type === 'normalFan') {
                toRemove.push(child);
            }
        });
        toRemove.forEach(obj => {
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) obj.material.dispose();
            scene.remove(obj);
        });

        if (manifoldState.showUnstableManifold && manifoldState.manifolds.length > 0) {
            manifoldState.manifolds.forEach(m => {
                const color = ORBIT_COLORS.manifold;

                [m.plus, m.minus].forEach(traj => {
                    if (traj && traj.points && traj.points.length > 1) {
                        traj.points.forEach(([x, y]) => {
                            const geom = new THREE.SphereGeometry(0.008, 6, 6);
                            const mat = new THREE.MeshBasicMaterial({
                                color: new THREE.Color(color)
                            });
                            const sphere = new THREE.Mesh(geom, mat);
                            sphere.position.set(x, y, 0.1);
                            sphere.userData.type = 'manifold';
                            scene.add(sphere);
                        });
                    }
                });
            });
        }

        if (manifoldState.showStableManifold && manifoldState.stableManifolds.length > 0) {
            manifoldState.stableManifolds.forEach(m => {
                [m.plus, m.minus].forEach(traj => {
                    if (traj && traj.points && traj.points.length > 0) {
                        traj.points.forEach(([x, y]) => {
                            const geom = new THREE.SphereGeometry(0.008, 6, 6);
                            const mat = new THREE.MeshBasicMaterial({
                                color: new THREE.Color(ORBIT_COLORS.stableManifold)
                            });
                            const sphere = new THREE.Mesh(geom, mat);
                            sphere.position.set(x, y, 0.08);
                            sphere.userData.type = 'manifold';
                            scene.add(sphere);
                        });
                    }
                });
            });
        }

        if (dynamicSystem === 'duffing_ode' && bdeState.points && bdeState.points.length > 1) {
            const pts = bdeState.points.map(p => new THREE.Vector3(p.x, p.y, 0.15));
            pts.push(pts[0].clone());
            const geom = new THREE.BufferGeometry().setFromPoints(pts);
            const mat = new THREE.LineBasicMaterial({
                color: new THREE.Color('#3d5afe'),
                linewidth: 2,
                transparent: true,
                opacity: 0.85
            });
            const line = new THREE.Line(geom, mat);
            line.userData.type = 'bde';
            scene.add(line);
        }

        manifoldState.fixedPoints.forEach(fp => {
            const stabLower = (fp.stability || '').toLowerCase();
            const isAttractor = stabLower === 'attractor' || stabLower === 'stable';
            const isRepeller = stabLower === 'repeller' || stabLower === 'unstable';
            const isSaddle = stabLower === 'saddle';
            const color = isAttractor ? ORBIT_COLORS.attractor :
                isRepeller ? ORBIT_COLORS.repeller :
                isSaddle ? ORBIT_COLORS.saddlePoint : ORBIT_COLORS.periodicBlue;
            const radius = isAttractor ? 0.03 : isRepeller ? 0.028 : 0.025;
            const geom = new THREE.SphereGeometry(radius, 12, 12);
            const mat = new THREE.MeshBasicMaterial({ color: new THREE.Color(color) });
            const sphere = new THREE.Mesh(geom, mat);
            sphere.position.set(fp.x, fp.y, 0.2);
            sphere.userData = {
                type: 'fixedPoint',
                period: 1,
                stability: fp.stability,
                pos: { x: fp.x, y: fp.y },
                eigenvalues: fp.eigenvalues || null
            };
            scene.add(sphere);
        });

        if (type === 'discrete' && manifoldState.showNormalFan) {
            const basePoint = manifoldState.hasStarted && manifoldState.currentPoint
                ? manifoldState.currentPoint
                : manifoldState.startPoint;
            const count = Math.max(4, Math.min(32, Math.floor(manifoldState.normalFanCount || 8)));
            const viewScale = Math.max(
                0.08,
                Math.min(viewRange.xMax - viewRange.xMin, viewRange.yMax - viewRange.yMin) * 0.055
            );
            const tangentHalfLength = viewScale * 0.92;
            const normalLength = viewScale * 0.78;
            const tangentPoints = [];
            const normalPoints = [];

            for (let k = 0; k < count; k += 1) {
                const theta = Math.PI * 2 * (k + 0.5) / count;
                const nx = Math.cos(theta);
                const ny = Math.sin(theta);
                const tx = -ny;
                const ty = nx;

                tangentPoints.push(
                    new THREE.Vector3(basePoint.x - tx * tangentHalfLength, basePoint.y - ty * tangentHalfLength, 0.31),
                    new THREE.Vector3(basePoint.x + tx * tangentHalfLength, basePoint.y + ty * tangentHalfLength, 0.31)
                );
                normalPoints.push(
                    new THREE.Vector3(basePoint.x, basePoint.y, 0.33),
                    new THREE.Vector3(basePoint.x + nx * normalLength, basePoint.y + ny * normalLength, 0.33)
                );
            }

            const tangentGeom = new THREE.BufferGeometry().setFromPoints(tangentPoints);
            const tangentMat = new THREE.LineBasicMaterial({
                color: new THREE.Color('#7d7168'),
                transparent: true,
                opacity: 0.42,
                depthWrite: false
            });
            const tangentLines = new THREE.LineSegments(tangentGeom, tangentMat);
            tangentLines.userData.type = 'normalFan';
            scene.add(tangentLines);

            const normalGeom = new THREE.BufferGeometry().setFromPoints(normalPoints);
            const normalMat = new THREE.LineBasicMaterial({
                color: new THREE.Color('#ddd5c8'),
                transparent: true,
                opacity: 0.86,
                depthWrite: false
            });
            const normalLines = new THREE.LineSegments(normalGeom, normalMat);
            normalLines.userData.type = 'normalFan';
            scene.add(normalLines);

            const centerGeom = new THREE.RingGeometry(viewScale * 0.08, viewScale * 0.12, 24);
            const centerMat = new THREE.MeshBasicMaterial({
                color: new THREE.Color('#ddd5c8'),
                transparent: true,
                opacity: 0.9,
                side: THREE.DoubleSide,
                depthWrite: false
            });
            const center = new THREE.Mesh(centerGeom, centerMat);
            center.position.set(basePoint.x, basePoint.y, 0.35);
            center.userData.type = 'normalFan';
            scene.add(center);

            const currentNorm = Math.hypot(basePoint.nx || 0, basePoint.ny || 0);
            if (currentNorm > 1e-12) {
                const nx = basePoint.nx / currentNorm;
                const ny = basePoint.ny / currentNorm;
                const selectedGeom = new THREE.BufferGeometry().setFromPoints([
                    new THREE.Vector3(basePoint.x, basePoint.y, 0.36),
                    new THREE.Vector3(basePoint.x + nx * normalLength * 1.15, basePoint.y + ny * normalLength * 1.15, 0.36)
                ]);
                const selectedMat = new THREE.LineBasicMaterial({
                    color: new THREE.Color(ORBIT_COLORS.trajectory),
                    transparent: true,
                    opacity: 0.95,
                    depthWrite: false
                });
                const selectedLine = new THREE.LineSegments(selectedGeom, selectedMat);
                selectedLine.userData.type = 'normalFan';
                scene.add(selectedLine);
            }
        }

        if (manifoldState.showTrail && manifoldState.trajectoryPoints.length > 0) {
            if (dynamicSystem === 'duffing_ode') {
                const points = manifoldState.trajectoryPoints.map(p => new THREE.Vector3(p.x, p.y, 0.25));
                const geom = new THREE.BufferGeometry().setFromPoints(points);
                const mat = new THREE.LineBasicMaterial({ color: new THREE.Color(ORBIT_COLORS.manifold), linewidth: 2, transparent: true, opacity: 0.8 });
                const line = new THREE.Line(geom, mat);
                line.userData.type = 'trajectory';
                scene.add(line);
            } else {
                manifoldState.trajectoryPoints.forEach((point, idx) => {
                    const normalizedIdx = idx / manifoldState.trajectoryPoints.length;
                    const size = 0.022 * (0.4 + 0.6 * normalizedIdx);
                    const geom = new THREE.SphereGeometry(size, 8, 8);
                    const mat = new THREE.MeshBasicMaterial({ color: new THREE.Color(ORBIT_COLORS.trajectory), opacity: 0.4 + 0.6 * normalizedIdx, transparent: true });
                    const sphere = new THREE.Mesh(geom, mat);
                    sphere.position.set(point.x, point.y, 0.25);
                    sphere.userData.type = 'trajectory';
                    scene.add(sphere);
                });
            }
        }

        if (manifoldState.hasStarted && manifoldState.currentPoint) {
            const glowGeom = new THREE.RingGeometry(0.05, 0.05, 20);
            const currentColor = dynamicSystem === 'duffing_ode' ? ORBIT_COLORS.manifold : ORBIT_COLORS.trajectory;
            const glowMat = new THREE.MeshBasicMaterial({ color: new THREE.Color(currentColor), opacity: 0.6, transparent: true, side: THREE.DoubleSide });
            const glowRing = new THREE.Mesh(glowGeom, glowMat);
            glowRing.position.set(manifoldState.currentPoint.x, manifoldState.currentPoint.y, 0.3);
            glowRing.userData.type = 'trajectory';
            scene.add(glowRing);

            const geom = new THREE.SphereGeometry(0.02, 16, 16);
            const mat = new THREE.MeshBasicMaterial({ color: new THREE.Color('#ffffff') });
            const sphere = new THREE.Mesh(geom, mat);
            sphere.position.set(manifoldState.currentPoint.x, manifoldState.currentPoint.y, 0.3);
            sphere.userData.type = 'trajectory';
            scene.add(sphere);
        }

        if (manifoldState.showOrbits && periodicState.orbits.length > 0) {
            const visibleOrbits = periodicState.orbits.filter(o => isOrbitVisible(o));
            const HIGHLIGHT_COLOR = '#ff00ff';

            visibleOrbits.forEach((orbit, orbitIdx) => {
                const orbitId = `orbit-${orbit.period}-${orbitIdx}`;
                const isHighlighted = manifoldState.highlightedOrbitId === orbitId;
                const pointColor = isHighlighted ? HIGHLIGHT_COLOR : getOrbitColor(orbit);

                orbit.points.forEach((pt, ptIdx) => {
                    const geom = new THREE.SphereGeometry(isHighlighted ? 0.04 : 0.02, 10, 10);
                    const mat = new THREE.MeshBasicMaterial({ color: new THREE.Color(pointColor) });
                    const sphere = new THREE.Mesh(geom, mat);
                    sphere.position.set(pt[0], pt[1], isHighlighted ? 0.15 : 0.05);
                    sphere.userData = {
                        type: 'orbit',
                        orbitId: orbitId,
                        period: orbit.period,
                        stability: orbit.stability,
                        pointIndex: ptIdx,
                        pos: { x: pt[0], y: pt[1] },
                        orbitPoints: orbit.points,
                        eigenvalues: orbit.eigenvalues || null
                    };
                    scene.add(sphere);
                });

                if (manifoldState.showOrbitLines && orbit.points.length > 1) {
                    const pointColor = isHighlighted ? HIGHLIGHT_COLOR : getOrbitColor(orbit);
                    const opacity = isHighlighted ? 1.0 : 0.6;

                    orbit.points.forEach((pt) => {
                        const geom = new THREE.SphereGeometry(isHighlighted ? 0.035 : 0.018, 8, 8);
                        const mat = new THREE.MeshBasicMaterial({
                            color: new THREE.Color(pointColor),
                            transparent: true,
                            opacity: opacity
                        });
                        const sphere = new THREE.Mesh(geom, mat);
                        sphere.position.set(pt[0], pt[1], isHighlighted ? 0.14 : 0.04);
                        sphere.userData = { type: 'orbitLine', orbitId: orbitId };
                        scene.add(sphere);
                    });
                }
            });
        }
    }, [periodicState, manifoldState, filters, bdeState, dynamicSystem, type, viewRange]);

    const getOrbitColor = (orbit) => {
        const { stability } = orbit;
        if (stability.toLowerCase() === 'stable') return ORBIT_COLORS.attractor;
        if (stability.toLowerCase() === 'saddle') return ORBIT_COLORS.saddlePoint;
        if (stability.toLowerCase() === 'unstable') return ORBIT_COLORS.repeller;
        return ORBIT_COLORS.periodicBlue;
    };

    const isOrbitVisible = (orbit) => {
        if (orbit.period === 1) return filters.period1;
        if (orbit.period === 2) return filters.period2;
        if (orbit.period === 3) return filters.period3;
        if (orbit.period === 4) return filters.period4;
        if (orbit.period === 5) return filters.period5;
        return filters.period6plus;
    };

    const stepForwardManifold = useCallback(() => {
        if (manifoldState.isRunning || !wasmModule) return;
        const { x, y, nx, ny } = manifoldState.currentPoint;

        if (!isFinite(x) || !isFinite(y) || Math.abs(x) > 10 || Math.abs(y) > 10) {
            setManifoldState(prev => ({
                ...prev,
                currentPoint: { ...prev.startPoint },
                trajectoryPoints: [],
                iteration: 0,
                hasStarted: false
            }));
            return;
        }

        let nextPoint;
        if (dynamicSystem === 'custom') {
            if (!appliedParamValidation.valid) return;
            const result = wasmModule.boundary_map_user_defined(
                x, y, nx, ny,
                activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                appliedParamValidation.normalized,
                params.epsilon
            );
            nextPoint = result;
        } else if (dynamicSystem === 'custom_ode') {
            if (!appliedParamValidation.valid) return;
            const result = wasmModule.boundary_map_user_defined_ode(
                x, y, nx, ny,
                activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                appliedParamValidation.normalized,
                params.h,
                params.epsilon
            );
            nextPoint = result;
        } else if (dynamicSystem === 'duffing_ode') {
            const { boundary_map_duffing_ode } = wasmModule;
            if (!boundary_map_duffing_ode) {
                console.error('boundary_map_duffing_ode not found in WASM module');
                return;
            }
            nextPoint = boundary_map_duffing_ode(x, y, nx, ny, params.delta, params.h, params.epsilon);
        } else {
            const { boundary_map } = wasmModule;
            if (!boundary_map) {
                console.error('boundary_map not found in WASM module');
                return;
            }
            nextPoint = boundary_map(x, y, nx, ny, params.a, params.b, params.epsilon);
        }

        setManifoldState(prev => ({
            ...prev,
            currentPoint: { x: nextPoint.x, y: nextPoint.y, nx: nextPoint.nx, ny: nextPoint.ny },
            trajectoryPoints: [...prev.trajectoryPoints, { x, y, nx, ny }],
            iteration: prev.iteration + 1,
            hasStarted: true
        }));
    }, [manifoldState.isReady, manifoldState.isRunning, manifoldState.currentPoint, wasmModule, params, dynamicSystem, activeAppliedCustomEquations, appliedParamValidation]);

    const runToConvergenceManifold = useCallback(() => {
        if (!wasmModule) return;

        if (manifoldState.isRunning) {
            cancelAnimationFrame(batchAnimationRef.current);
            setManifoldState(prev => ({ ...prev, isRunning: false }));
            return;
        }

        setManifoldState(prev => ({ ...prev, isRunning: true }));

        const stepFn = (cx, cy, cnx, cny) => {
            if (dynamicSystem === 'custom') {
                if (!appliedParamValidation.valid) return null;
                return wasmModule.boundary_map_user_defined(
                    cx, cy, cnx, cny,
                    activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                    appliedParamValidation.normalized,
                    params.epsilon
                );
            } else if (dynamicSystem === 'custom_ode') {
                if (!appliedParamValidation.valid) return null;
                return wasmModule.boundary_map_user_defined_ode(
                    cx, cy, cnx, cny,
                    activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                    appliedParamValidation.normalized,
                    params.h,
                    params.epsilon
                );
            } else if (dynamicSystem === 'duffing_ode') {
                const { boundary_map_duffing_ode } = wasmModule;
                if (!boundary_map_duffing_ode) return null;
                return boundary_map_duffing_ode(cx, cy, cnx, cny, params.delta, params.h, params.epsilon);
            } else {
                const { boundary_map } = wasmModule;
                if (!boundary_map) return null;
                return boundary_map(cx, cy, cnx, cny, params.a, params.b, params.epsilon);
            }
        };

        let currentX = manifoldState.currentPoint.x;
        let currentY = manifoldState.currentPoint.y;
        let currentNx = manifoldState.currentPoint.nx;
        let currentNy = manifoldState.currentPoint.ny;
        let iteration = manifoldState.iteration;
        const newPoints = [...manifoldState.trajectoryPoints];

        const isContinuous = dynamicSystem === 'duffing_ode' || dynamicSystem === 'custom_ode';
        const limitIterations = !isContinuous;
        const currentBatchSize = isContinuous ? 15 : 5;

        const animateStep = () => {
            for (let i = 0; i < currentBatchSize; i++) {
                if (limitIterations && iteration >= params.maxIterations) break;

                if (!isFinite(currentX) || !isFinite(currentY) || Math.abs(currentX) > 10 || Math.abs(currentY) > 10) {
                    setManifoldState(prev => ({
                        ...prev,
                        isRunning: false,
                        hasStarted: true,
                        trajectoryPoints: newPoints,
                        currentPoint: { x: currentX, y: currentY, nx: currentNx, ny: currentNy },
                        iteration
                    }));
                    return;
                }

                if (isContinuous && newPoints.length >= 1000) {
                    newPoints.shift();
                }
                newPoints.push({ x: currentX, y: currentY, nx: currentNx, ny: currentNy });

                const next = stepFn(currentX, currentY, currentNx, currentNy);
                if (!next) {
                    setManifoldState(prev => ({ ...prev, isRunning: false }));
                    return;
                }
                currentX = next.x;
                currentY = next.y;
                currentNx = next.nx;
                currentNy = next.ny;
                iteration++;
            }

            setManifoldState(prev => ({
                ...prev,
                currentPoint: { x: currentX, y: currentY, nx: currentNx, ny: currentNy },
                trajectoryPoints: [...newPoints],
                iteration,
                hasStarted: true
            }));

            if (!limitIterations || iteration < params.maxIterations) {
                batchAnimationRef.current = requestAnimationFrame(animateStep);
            } else {
                setManifoldState(prev => ({ ...prev, isRunning: false }));
            }
        };
        batchAnimationRef.current = requestAnimationFrame(animateStep);
    }, [manifoldState, params, wasmModule, dynamicSystem, activeAppliedCustomEquations, appliedParamValidation]);

    const handleStepForwardManifold = useCallback(() => {
        if (applyIfNeededBeforeAction('step')) return;
        stepForwardManifold();
    }, [applyIfNeededBeforeAction, stepForwardManifold]);

    const handleRunToConvergenceManifold = useCallback(() => {
        if (manifoldState.isRunning) {
            runToConvergenceManifold();
            return;
        }
        if (applyIfNeededBeforeAction('play')) return;
        runToConvergenceManifold();
    }, [manifoldState.isRunning, applyIfNeededBeforeAction, runToConvergenceManifold]);

    useEffect(() => {
        const pendingAction = postApplyActionRef.current;
        if (!pendingAction) return;
        if (hasPendingInputChanges) return;
        if (manifoldState.isComputing) return;
        if (!periodicState.isReady) return;

        postApplyActionRef.current = null;
        if (pendingAction === 'play') {
            runToConvergenceManifold();
        } else if (pendingAction === 'step') {
            stepForwardManifold();
        }
    }, [hasPendingInputChanges, manifoldState.isComputing, periodicState.isReady, runToConvergenceManifold, stepForwardManifold]);

    const toggleBdeFlow = useCallback(() => {
        if (bdeState.isRunning) {
            cancelAnimationFrame(bdeAnimRef.current);
            setBdeState(prev => ({ ...prev, isRunning: false }));
        } else {
            setBdeState(prev => ({ ...prev, isRunning: true }));
            let frameCount = 0;
            const stepBde = () => {
                if (!bdeSimRef.current) return;
                for (let i = 0; i < 3; i++) {
                    bdeSimRef.current.step(params.h);
                }
                frameCount++;
                if (frameCount % 20 === 0) {
                    bdeSimRef.current.reparameterize();
                }
                setBdeState(prev => ({ ...prev, points: bdeSimRef.current.get_points() }));
                bdeAnimRef.current = requestAnimationFrame(stepBde);
            };
            bdeAnimRef.current = requestAnimationFrame(stepBde);
        }
    }, [bdeState.isRunning, params.h]);

    const resetBdeFlow = useCallback(() => {
        if (bdeAnimRef.current) cancelAnimationFrame(bdeAnimRef.current);
        if (!wasmModule) return;
        if (bdeSimRef.current) {
            bdeSimRef.current.free();
        }
        if (dynamicSystem === 'custom_ode') {
            if (!appliedParamValidation.valid) return;
            bdeSimRef.current = new wasmModule.BdeSimulatorUserDefinedWasm(
                activeAppliedCustomEquations.xEq, activeAppliedCustomEquations.yEq,
                appliedParamValidation.normalized,
                params.epsilon,
                manifoldState.startPoint.x,
                manifoldState.startPoint.y,
                0.05, 1000
            );
        } else {
            bdeSimRef.current = new wasmModule.BdeSimulatorWasm(
                params.delta,
                params.epsilon,
                manifoldState.startPoint.x,
                manifoldState.startPoint.y,
                0.05, 1000
            );
        }
        setBdeState({ points: bdeSimRef.current.get_points(), isRunning: false });
    }, [wasmModule, params.delta, params.epsilon, manifoldState.startPoint.x, manifoldState.startPoint.y, dynamicSystem, activeAppliedCustomEquations, appliedParamValidation]);

    const resetManifold = useCallback(() => {
        if (batchAnimationRef.current) cancelAnimationFrame(batchAnimationRef.current);
        setManifoldState(prev => ({
            ...prev,
            currentPoint: { ...prev.startPoint },
            trajectoryPoints: [],
            iteration: 0,
            isRunning: false,
            hasStarted: false
        }));
    }, []);

    useEffect(() => {
        resetManifold();
    }, [params.a, params.b, params.delta, params.h, params.epsilon, activeAppliedCustomEquations, appliedParamValidation, dynamicSystem, resetManifold]);

    useEffect(() => {
        if (dynamicSystem === 'duffing_ode' || dynamicSystem === 'custom_ode') {
            resetBdeFlow();
        }
    }, [dynamicSystem, resetBdeFlow]);
    const computeUlam = useCallback(async () => {
        if (!wasmModule) return;
        if ((dynamicSystem === 'custom' || dynamicSystem === 'custom_ode') && !appliedParamValidation.valid) {
            setUlamState(prev => ({ ...prev, isComputing: false }));
            return;
        }
        setUlamState(prev => ({ ...prev, isComputing: true, needsRecompute: false }));
        try {
            const result = await runComputeTask('computeUlam', {
                dynamicSystem,
                params: {
                    a: params.a,
                    b: params.b,
                    delta: params.delta,
                    h: params.h
                },
                viewRange: {
                    xMin: viewRange.xMin,
                    xMax: viewRange.xMax,
                    yMin: viewRange.yMin,
                    yMax: viewRange.yMax
                },
                ulam: {
                    subdivisions: ulamState.subdivisions,
                    pointsPerBox: ulamState.pointsPerBox,
                    epsilon: ulamState.epsilon
                },
                customEquations: activeAppliedCustomEquations,
                customParams: appliedParamValidation.normalized,
                currentPoint: manifoldState.hasStarted && manifoldState.currentPoint
                    ? {
                        x: manifoldState.currentPoint.x,
                        y: manifoldState.currentPoint.y
                    }
                    : null
            });

            setUlamState(prev => ({
                ...prev,
                isComputing: false,
                gridBoxes: result?.boxes || [],
                invariantMeasure: result?.invariantMeasure || null,
                leftEigenvector: result?.leftEigenvector || null,
                currentBoxIndex: result?.currentBoxIndex ?? -1,
                selectedBoxIndex: null,
                transitions: null
            }));

        } catch (err) {
            console.error("Ulam computation failed:", err);
            setUlamState(prev => ({ ...prev, isComputing: false }));
        }
    }, [wasmModule, params.a, params.b, params.delta, params.h, ulamState.subdivisions, ulamState.pointsPerBox, ulamState.epsilon, manifoldState.hasStarted, manifoldState.currentPoint, dynamicSystem, activeAppliedCustomEquations, appliedParamValidation, viewRange, runComputeTask]);

    useEffect(() => {
        setUlamState(prev => ({ ...prev, epsilon: params.epsilon }));
    }, [params.epsilon]);

    useEffect(() => {
        if (!ulamState.showUlamOverlay || !wasmModule) return;

        if (ulamDebounceRef.current) {
            clearTimeout(ulamDebounceRef.current);
        }


        ulamDebounceRef.current = setTimeout(() => {
            computeUlam();
        }, 200);

        return () => {
            if (ulamDebounceRef.current) {
                clearTimeout(ulamDebounceRef.current);
            }
        };
    }, [
        ulamState.showUlamOverlay,
        wasmModule,
        params.a,
        params.b,
        params.delta,
        params.h,
        ulamState.epsilon,
        ulamState.subdivisions,
        ulamState.pointsPerBox,
        computeRequestId,
        computeUlam
    ]);

    const computeHittingContours = useCallback(async () => {
        if (!wasmModule || dynamicSystem !== 'henon') return;
        const settings = normalizeHittingContourSettings({
            sampleGridSize: hittingContourState.sampleGridSize,
            thetaGridSize: hittingContourState.thetaGridSize,
            maxPeriod: hittingContourState.maxPeriod,
            maxLevel: hittingContourState.maxLevel,
            ulamSubdivisions: hittingContourState.ulamSubdivisions,
            ulamPointsPerBox: hittingContourState.ulamPointsPerBox,
            ulamIterations: hittingContourState.ulamIterations,
            supportMass: hittingContourState.supportMass,
            hitTolerance: hittingContourState.hitTolerance,
            residualThreshold: hittingContourState.residualThreshold
        });
        const requestId = ++hittingContourRequestRef.current;
        setHittingContourState(prev => ({
            ...prev,
            ...settings,
            isComputing: true,
            error: null
        }));

        try {
            const result = await runComputeTask('computeHittingContours', {
                params: {
                    a: params.a,
                    b: params.b,
                    epsilon: params.epsilon
                },
                viewRange: {
                    xMin: viewRange.xMin,
                    xMax: viewRange.xMax,
                    yMin: viewRange.yMin,
                    yMax: viewRange.yMax
                },
                settings
            });

            if (requestId !== hittingContourRequestRef.current) return;
            setHittingContourState(prev => {
                const levels = Array.isArray(result?.levelsPresent) ? result.levelsPresent : [];
                const selectedLevels = Array.isArray(prev.selectedLevels)
                    ? prev.selectedLevels.filter(level => levels.includes(level))
                    : [];
                return {
                    ...prev,
                    isComputing: false,
                    result,
                    selectedLevels,
                    selectedLevel: selectedLevels.length > 0 ? selectedLevels[0] : 'all',
                    error: null
                };
            });
        } catch (err) {
            if (requestId !== hittingContourRequestRef.current) return;
            console.error('Hitting contour computation failed:', err);
            setHittingContourState(prev => ({
                ...prev,
                isComputing: false,
                error: err?.message || 'Computation failed'
            }));
        }
    }, [
        wasmModule,
        dynamicSystem,
        hittingContourState.sampleGridSize,
        hittingContourState.thetaGridSize,
        hittingContourState.maxPeriod,
        hittingContourState.maxLevel,
        hittingContourState.ulamSubdivisions,
        hittingContourState.ulamPointsPerBox,
        hittingContourState.ulamIterations,
        hittingContourState.supportMass,
        hittingContourState.hitTolerance,
        hittingContourState.residualThreshold,
        params.a,
        params.b,
        params.epsilon,
        viewRange,
        runComputeTask
    ]);

    useEffect(() => {
        if (dynamicSystem !== 'henon') {
            setHittingContourState(prev => prev.result || prev.error
                ? { ...prev, isComputing: false, result: null, error: null }
                : prev);
            return;
        }
        if (!hittingContourState.showOverlay || !wasmModule) return;

        if (hittingContourDebounceRef.current) {
            clearTimeout(hittingContourDebounceRef.current);
        }

        hittingContourDebounceRef.current = setTimeout(() => {
            computeHittingContours();
        }, 300);

        return () => {
            if (hittingContourDebounceRef.current) {
                clearTimeout(hittingContourDebounceRef.current);
            }
        };
    }, [
        dynamicSystem,
        hittingContourState.showOverlay,
        hittingContourState.sampleGridSize,
        hittingContourState.thetaGridSize,
        hittingContourState.maxPeriod,
        hittingContourState.maxLevel,
        hittingContourState.ulamSubdivisions,
        hittingContourState.ulamPointsPerBox,
        hittingContourState.ulamIterations,
        hittingContourState.supportMass,
        hittingContourState.hitTolerance,
        hittingContourState.residualThreshold,
        params.a,
        params.b,
        params.epsilon,
        viewRange,
        wasmModule,
        computeRequestId,
        computeHittingContours
    ]);

    useEffect(() => {
        if (!ulamState.showUlamOverlay || !ulamState.gridBoxes.length) return;

        if (manifoldState.hasStarted && manifoldState.currentPoint) {
            const boxIdx = getGridBoxIndex(
                manifoldState.currentPoint.x,
                manifoldState.currentPoint.y,
                viewRange,
                ulamState.subdivisions
            );

            if (boxIdx !== ulamState.currentBoxIndex) {
                setUlamState(prev => ({
                    ...prev,
                    currentBoxIndex: boxIdx,
                    transitions: prev.showCurrentBox ? null : prev.transitions,
                    selectedBoxIndex: prev.showCurrentBox ? boxIdx : prev.selectedBoxIndex
                }));
                if (ulamState.showCurrentBox && boxIdx >= 0) {
                    requestUlamTransitions(boxIdx, 'current');
                }
            }
        }
    }, [manifoldState.currentPoint, manifoldState.hasStarted, ulamState.showUlamOverlay, ulamState.gridBoxes.length, ulamState.subdivisions, ulamState.currentBoxIndex, ulamState.showCurrentBox, viewRange, requestUlamTransitions]);


    useEffect(() => {
        const scene = sceneRef.current;
        if (!scene) return;

        const cleanup = () => {
            const oldMesh = scene.getObjectByName('ulam-grid');
            if (oldMesh) {
                oldMesh.geometry.dispose();
                oldMesh.material.dispose();
                scene.remove(oldMesh);
            }
        };

        if (!ulamState.showUlamOverlay || !ulamState.gridBoxes.length) {
            cleanup();
            return;
        }

        cleanup();

        const boxes = ulamState.gridBoxes;
        const count = boxes.length;
        const geometry = new THREE.PlaneGeometry(1, 1);
        const material = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            transparent: true,
            opacity: 0.5,
            side: THREE.DoubleSide,
            depthWrite: false
        });

        const mesh = new THREE.InstancedMesh(geometry, material, count);
        mesh.name = 'ulam-grid';
        mesh.userData.type = 'ulamGrid';

        const dummy = new THREE.Object3D();
        const color = new THREE.Color();

        const transitionMap = new Map();
        if (ulamState.selectedBoxIndex !== null && ulamState.transitions) {
            ulamState.transitions.forEach(t => {
                transitionMap.set(t.index, t.probability);
            });
        }

        let maxMeasure = 0;
        if (ulamState.invariantMeasure) {
            maxMeasure = Math.max(...ulamState.invariantMeasure);
        }

        boxes.forEach((box, i) => {
            const cx = box.center[0];
            const cy = box.center[1];
            const rx = box.radius[0];
            const ry = box.radius[1];

            dummy.position.set(cx, cy, -0.05);
            dummy.scale.set(rx * 2 * 0.95, ry * 2 * 0.95, 1);
            dummy.updateMatrix();
            mesh.setMatrixAt(i, dummy.matrix);

            const isCurrentBox = ulamState.showCurrentBox && i === ulamState.currentBoxIndex;
            const isSelectedBox = ulamState.selectedBoxIndex !== null && i === ulamState.selectedBoxIndex;

            if (isCurrentBox && !isSelectedBox) {
                color.setHex(0xff00ff);
                mesh.setColorAt(i, color);
            } else if (ulamState.selectedBoxIndex !== null) {
                if (i === ulamState.selectedBoxIndex) {
                    color.setHex(0x00ffff);
                    mesh.setColorAt(i, color);
                } else if (transitionMap.has(i)) {
                    const prob = transitionMap.get(i);
                    color.setHSL(0.7 - prob * 0.7, 1.0, 0.5);
                    mesh.setColorAt(i, color);
                } else {
                    color.setHex(0x222222);
                    mesh.setColorAt(i, color);
                }
            } else if (ulamState.invariantMeasure && ulamState.invariantMeasure.length === count) {
                const measure = ulamState.invariantMeasure[i];
                if (measure > 0) {
                    const intensity = measure / maxMeasure;
                    const h = 0.66 - (intensity * 0.5);
                    color.setHSL(h, 1.0, 0.5);
                    mesh.setColorAt(i, color);
                } else {
                    color.setHex(0x111111);
                    mesh.setColorAt(i, color);
                }
            } else {
                color.setHex(0x333333);
                mesh.setColorAt(i, color);
            }
        });

        mesh.instanceMatrix.needsUpdate = true;
        if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;

        scene.add(mesh);

        return cleanup;
    }, [ulamState.showUlamOverlay, ulamState.gridBoxes, ulamState.selectedBoxIndex, ulamState.transitions, ulamState.invariantMeasure, ulamState.currentBoxIndex, ulamState.showCurrentBox]);

    useEffect(() => {
        const scene = sceneRef.current;
        if (!scene) return;

        const cleanup = () => {
            const oldGroup = scene.getObjectByName('hitting-level-overlay');
            if (!oldGroup) return;
            oldGroup.traverse((child) => {
                if (child.geometry) child.geometry.dispose();
                if (child.material) child.material.dispose();
            });
            scene.remove(oldGroup);
        };

        cleanup();

        if (!hittingContourState.showOverlay || !hittingContourState.result) {
            return cleanup;
        }

        const result = hittingContourState.result;
        const settings = result.settings || {};
        const gridSize = settings.sampleGridSize || hittingContourState.sampleGridSize;
        const dx = (viewRange.xMax - viewRange.xMin) / gridSize;
        const dy = (viewRange.yMax - viewRange.yMin) / gridSize;
        const layoutMode = hittingContourState.layoutMode || 'topographic';
        const selectedLevels = Array.isArray(hittingContourState.selectedLevels)
            ? hittingContourState.selectedLevels
            : [];
        const levelSelection = selectedLevels.length > 0 ? selectedLevels : 'all';
        const group = new THREE.Group();
        group.name = 'hitting-level-overlay';

        if (layoutMode === 'spatial') {
            const visibleEntries = getVisibleHittingCells(result, levelSelection);
            if (visibleEntries.length > 0 && dx > 0 && dy > 0) {
                const geometry = new THREE.PlaneGeometry(1, 1);
                const material = new THREE.MeshBasicMaterial({
                    color: 0xffffff,
                    transparent: true,
                    opacity: 0.58,
                    side: THREE.DoubleSide,
                    depthWrite: false
                });
                const mesh = new THREE.InstancedMesh(geometry, material, visibleEntries.length);
                mesh.name = 'hitting-level-cells';
                mesh.userData.type = 'hittingLevelCells';
                mesh.userData.entries = visibleEntries;

                const dummy = new THREE.Object3D();
                const color = new THREE.Color();
                const maxLevel = Math.max(1, settings.maxLevel || hittingContourState.maxLevel);

                visibleEntries.forEach(({ cell, hit }, instanceId) => {
                    dummy.position.set(cell.x, cell.y, -0.025);
                    dummy.scale.set(dx * 0.82, dy * 0.82, 1);
                    dummy.updateMatrix();
                    mesh.setMatrixAt(instanceId, dummy.matrix);

                    color.setHex(getStabilityColorHex(hit.stability));
                    const levelFactor = 1 - Math.min(maxLevel, hit.level || maxLevel) / (maxLevel + 1);
                    color.lerp(new THREE.Color(0xddd5c8), 0.18 + 0.28 * levelFactor);
                    mesh.setColorAt(instanceId, color);
                });

                mesh.instanceMatrix.needsUpdate = true;
                if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
                group.add(mesh);
            }

            (result.targets || []).forEach((target) => {
                const geometry = new THREE.SphereGeometry(0.022, 12, 12);
                const material = new THREE.MeshBasicMaterial({
                    color: new THREE.Color(getStabilityColorHex(target.stability)),
                    transparent: true,
                    opacity: 0.95,
                    depthWrite: false
                });
                const marker = new THREE.Mesh(geometry, material);
                marker.position.set(target.x, target.y, 0.36);
                marker.userData = {
                    type: 'hittingTarget',
                    target
                };
                group.add(marker);
            });
        } else {
            const targetContours = getHittingTargetContours(result, levelSelection);
            const viewWidth = viewRange.xMax - viewRange.xMin;
            const viewHeight = viewRange.yMax - viewRange.yMin;
            const count = targetContours.length;
            const aspect = viewWidth > 0 && viewHeight > 0 ? viewWidth / viewHeight : 1;
            const cols = Math.max(1, Math.ceil(Math.sqrt(count * aspect)));
            const rows = Math.max(1, Math.ceil(count / cols));
            const padX = viewWidth * 0.10;
            const padY = viewHeight * 0.10;
            const cellW = count > 0 ? (viewWidth - 2 * padX) / cols : 0;
            const cellH = count > 0 ? (viewHeight - 2 * padY) / rows : 0;
            const minCell = Math.max(0.01, Math.min(cellW, cellH));
            const maxLevel = Math.max(1, settings.maxLevel || hittingContourState.maxLevel);
            const contourHighlight = new THREE.Color(0xddd5c8);

            const gridPoints = buildAbstractGrid(viewRange, 12);
            if (gridPoints.length > 0) {
                const gridGeometry = new THREE.BufferGeometry().setFromPoints(gridPoints);
                const gridMaterial = new THREE.LineBasicMaterial({
                    color: 0x4a4540,
                    transparent: true,
                    opacity: 0.22,
                    depthWrite: false
                });
                const abstractGrid = new THREE.LineSegments(gridGeometry, gridMaterial);
                abstractGrid.name = 'hitting-topographic-grid';
                abstractGrid.userData.type = 'hittingTopographicGrid';
                group.add(abstractGrid);
            }

            targetContours.forEach((entry, index) => {
                const col = index % cols;
                const row = Math.floor(index / cols);
                const centerX = viewRange.xMin + padX + cellW * (col + 0.5);
                const centerY = viewRange.yMax - padY - cellH * (row + 0.5);
                const rings = entry.rings;
                const baseColor = new THREE.Color(getStabilityColorHex(entry.target.stability));
                const outerRadius = minCell * 0.38;
                const innerRadius = minCell * 0.09;

                rings.forEach((ring) => {
                    const levelRatio = Math.min(maxLevel, Math.max(1, ring.level)) / Math.max(1, maxLevel);
                    const radius = innerRadius + (outerRadius - innerRadius) * levelRatio;
                    const color = baseColor.clone().lerp(contourHighlight, 0.18 + 0.26 * levelRatio);
                    if (hittingContourState.showLevelRings !== false) {
                        const geometry = new THREE.BufferGeometry().setFromPoints(
                            buildCircleRingPoints(centerX, centerY, radius)
                        );
                        const material = new THREE.LineBasicMaterial({
                            color,
                            transparent: true,
                            opacity: selectedLevels.length === 0 ? 0.48 : 0.68,
                            depthWrite: false
                        });
                        const line = new THREE.LineLoop(geometry, material);
                        line.userData = {
                            type: 'hittingContourRing',
                            target: entry.target,
                            level: ring.level,
                            count: ring.count,
                            points: ring.points
                        };
                        group.add(line);
                    }

                    if (hittingContourState.showLevelRings !== false) {
                        const hitAreaGeometry = new THREE.TorusGeometry(radius, minCell * 0.009, 8, 128);
                        const hitAreaMaterial = new THREE.MeshBasicMaterial({
                            color,
                            transparent: true,
                            opacity: 0.001,
                            depthWrite: false,
                            side: THREE.DoubleSide
                        });
                        const hitArea = new THREE.Mesh(hitAreaGeometry, hitAreaMaterial);
                        hitArea.position.set(centerX, centerY, 0.345);
                        hitArea.userData = {
                            type: 'hittingContourRing',
                            target: entry.target,
                            level: ring.level,
                            count: ring.count,
                            points: ring.points
                        };
                        group.add(hitArea);
                    }

                    if (ring.points.length > 0) {
                        const pointGeometry = new THREE.SphereGeometry(minCell * 0.012, 10, 10);
                        const pointMaterial = new THREE.MeshBasicMaterial({
                            color,
                            transparent: true,
                            opacity: 0.98,
                            depthWrite: false
                        });
                        const pointMesh = new THREE.InstancedMesh(pointGeometry, pointMaterial, ring.points.length);
                        pointMesh.name = 'hitting-level-points';
                        pointMesh.userData = {
                            type: 'hittingLevelPoint',
                            target: entry.target,
                            level: ring.level,
                            points: ring.points
                        };
                        const dummy = new THREE.Object3D();
                        const angleOffset = ((entry.target.targetIndex || 0) * 0.73 + ring.level * 0.19) % (Math.PI * 2);
                        ring.points.forEach((point, pointIndex) => {
                            const angle = angleOffset + (pointIndex / ring.points.length) * Math.PI * 2;
                            dummy.position.set(
                                centerX + Math.cos(angle) * radius,
                                centerY + Math.sin(angle) * radius,
                                0.39
                            );
                            dummy.updateMatrix();
                            pointMesh.setMatrixAt(pointIndex, dummy.matrix);
                        });
                        pointMesh.instanceMatrix.needsUpdate = true;
                        group.add(pointMesh);
                    }
                });

                const markerGeometry = new THREE.SphereGeometry(minCell * 0.035, 16, 16);
                const markerMaterial = new THREE.MeshBasicMaterial({
                    color: baseColor,
                    transparent: true,
                    opacity: 0.98,
                    depthWrite: false
                });
                const marker = new THREE.Mesh(markerGeometry, markerMaterial);
                marker.position.set(centerX, centerY, 0.38);
                marker.userData = {
                    type: 'hittingTarget',
                    target: entry.target,
                    topographic: {
                        totalHits: entry.totalHits,
                        bestLevel: entry.bestLevel,
                        ringCount: rings.length
                    }
                };
                group.add(marker);
            });
        }

        scene.add(group);
        return cleanup;
    }, [
        hittingContourState.showOverlay,
        hittingContourState.result,
        hittingContourState.selectedLevel,
        hittingContourState.selectedLevels,
        hittingContourState.layoutMode,
        hittingContourState.showLevelRings,
        hittingContourState.sampleGridSize,
        hittingContourState.maxLevel,
        viewRange
    ]);

    const SYSTEMS = {
        discrete: [
            { id: 'henon', name: 'Hénon Map', presets: [{ name: 'Standard', vals: { a: 1.4, b: 0.3 } }, { name: 'Lozi-like', vals: { a: 1.7, b: 0.5 } }] },
            { id: 'duffing', name: 'Duffing Map', presets: [{ name: 'Standard', vals: { a: 2.75, b: 0.2 } }] },
            { id: 'custom', name: 'Custom Equations', presets: [] }
        ],
        continuous: [
            { id: 'duffing_ode', name: 'Duffing Oscillator', presets: [{ name: 'Chaos', vals: { delta: 0.25 } }] },
            { id: 'custom_ode', name: 'Custom ODE', presets: [] }
        ]
    };

    const setType = (newType) => {
        if (newType === 'continuous') setDynamicSystem('duffing_ode');
        if (newType === 'discrete') setDynamicSystem('henon');
    };

    const updateStartPoint = (key, value) => {
        setManifoldState(prev => {
            const newStart = { ...prev.startPoint, [key]: value };
            return applyStartPointUpdate(prev, newStart);
        });
        if (typeof window.update_start_point === 'function') {
            window.update_start_point(
                key === 'x' ? value : manifoldState.startPoint.x,
                key === 'y' ? value : manifoldState.startPoint.y,
            );
        }
    };

    const applyPreset = (presetVals) => {
        setDraftParams(prev => ({ ...prev, ...presetVals }));
    };

    return (
        <Shell>
            <Sidebar
                type={type}
                setType={setType}
                dynamicSystem={dynamicSystem}
                setDynamicSystem={setDynamicSystem}
                SYSTEMS={SYSTEMS}
                customEquations={draftCustomEquations}
                setCustomEquations={setDraftCustomEquations}
                equationError={equationError}
                params={draftParams}
                setParams={setDraftParams}
                applyPreset={applyPreset}
                customParams={activeDraftCustomParams}
                setCustomParams={(next) => {
                    setDraftCustomParams(prev => ({
                        ...prev,
                        [activeCustomKey]: typeof next === 'function' ? next(prev[activeCustomKey]) : next
                    }));
                }}
                paramErrors={draftParamValidation.errors}
                hasPendingInputChanges={hasPendingInputChanges}
                applyInputsAndRecompute={applyInputsAndRecompute}
                appliedParams={params}
                viewRange={viewRange}
                setViewRange={updateViewRange}
                rangeLimit={RANGE_LIMIT}
                resetViewRange={resetViewRange}
                manifoldState={manifoldState}
                setManifoldState={setManifoldState}
                ORBIT_COLORS={ORBIT_COLORS}
                filters={filters}
                setFilters={setFilters}
                periodicState={periodicState}
                hittingContourState={hittingContourState}
                setHittingContourState={setHittingContourState}
                periodicSearchSettings={draftPeriodicSearchSettings}
                updatePeriodicSearchSettings={updatePeriodicSearchSettings}
                updateStartPoint={updateStartPoint}
                animationState={animationState}
                setAnimationState={setAnimationState}
                recordingState={recordingState}
                startAnimation={startAnimation}
                stopAnimation={stopAnimation}
                toggleRecording={toggleRecording}
                ulamState={ulamState}
                setUlamState={setUlamState}
                wasmModule={wasmModule}
                sweepState={sweepState}
                setSweepState={setSweepState}
                bdeState={bdeState}
                stepForwardManifold={handleStepForwardManifold}
                runToConvergenceManifold={handleRunToConvergenceManifold}
                resetManifold={resetManifold}
                toggleBdeFlow={toggleBdeFlow}
                resetBdeFlow={resetBdeFlow}
            />
            <Viewport
                type={type}
                canvasRef={canvasRef}
                tooltip={tooltip}
                manifoldState={manifoldState}
                ulamState={ulamState}
                hittingContourState={hittingContourState}
                savePNG={savePNG}
                handleZoomIn={() => { }}
                handleZoomOut={() => { }}
                handleResetView={() => { }}
                handlePanMode={() => { }}
            />
        </Shell>
    );
}

export default SetValuedViz;
