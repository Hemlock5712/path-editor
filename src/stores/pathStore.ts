import { create } from 'zustand';
import {
  Point,
  HeadingWaypoint,
  VelocityConstraints,
  DEFAULT_CONSTRAINTS,
} from '../types';

interface Snapshot {
  controlPoints: Point[];
  headingWaypoints: HeadingWaypoint[];
  constraints: VelocityConstraints;
}

const MAX_UNDO = 50;

interface PathState {
  controlPoints: Point[];
  headingWaypoints: HeadingWaypoint[];
  constraints: VelocityConstraints;
  selectedPointIndex: number | null;

  undoStack: Snapshot[];
  redoStack: Snapshot[];

  // Point mutations
  addPoint: (point: Point) => void;
  movePoint: (index: number, point: Point) => void;
  deletePoint: (index: number) => void;
  insertPointAfter: (index: number, point: Point) => void;
  selectPoint: (index: number | null) => void;

  // Heading mutations
  setHeading: (waypointIndex: number, degrees: number | null) => void;

  // Constraint mutations
  setConstraints: (constraints: VelocityConstraints) => void;

  // Undo / redo
  undo: () => void;
  redo: () => void;

  // Lifecycle
  clear: () => void;
  loadPath: (
    controlPoints: Point[],
    headingWaypoints: HeadingWaypoint[],
    constraints: VelocityConstraints,
  ) => void;
}

function takeSnapshot(state: PathState): Snapshot {
  return {
    controlPoints: state.controlPoints.map((p) => ({ ...p })),
    headingWaypoints: state.headingWaypoints.map((h) => ({ ...h })),
    constraints: { ...state.constraints },
  };
}

function pushUndo(state: PathState): { undoStack: Snapshot[]; redoStack: Snapshot[] } {
  const snap = takeSnapshot(state);
  const stack = [...state.undoStack, snap];
  if (stack.length > MAX_UNDO) stack.shift();
  return { undoStack: stack, redoStack: [] };
}

export const usePathStore = create<PathState>()((set) => ({
  controlPoints: [],
  headingWaypoints: [],
  constraints: { ...DEFAULT_CONSTRAINTS },
  selectedPointIndex: null,

  undoStack: [],
  redoStack: [],

  // ---- Point mutations ----

  addPoint: (point) =>
    set((state) => ({
      ...pushUndo(state),
      controlPoints: [...state.controlPoints, { ...point }],
    })),

  movePoint: (index, point) =>
    set((state) => ({
      controlPoints: state.controlPoints.map((p, i) =>
        i === index ? { ...point } : p,
      ),
    })),

  deletePoint: (index) =>
    set((state) => ({
      ...pushUndo(state),
      controlPoints: state.controlPoints.filter((_, i) => i !== index),
      headingWaypoints: state.headingWaypoints.filter(
        (hw) => Math.round(hw.waypointIndex) !== index,
      ),
      selectedPointIndex: null,
    })),

  insertPointAfter: (index, point) =>
    set((state) => {
      const controlPoints = [...state.controlPoints];
      controlPoints.splice(index + 1, 0, { ...point });

      // Shift heading waypoints that reference indices after the insertion
      const headingWaypoints = state.headingWaypoints.map((hw) => ({
        ...hw,
        waypointIndex:
          hw.waypointIndex > index ? hw.waypointIndex + 1 : hw.waypointIndex,
      }));

      return {
        ...pushUndo(state),
        controlPoints,
        headingWaypoints,
      };
    }),

  selectPoint: (index) => set({ selectedPointIndex: index }),

  // ---- Heading mutations ----

  setHeading: (waypointIndex, degrees) =>
    set((state) => {
      let headingWaypoints: HeadingWaypoint[];
      if (degrees === null) {
        headingWaypoints = state.headingWaypoints.filter(
          (hw) => Math.round(hw.waypointIndex) !== waypointIndex,
        );
      } else {
        const existing = state.headingWaypoints.findIndex(
          (hw) => Math.round(hw.waypointIndex) === waypointIndex,
        );
        if (existing >= 0) {
          headingWaypoints = state.headingWaypoints.map((hw, i) =>
            i === existing ? { waypointIndex, degrees } : hw,
          );
        } else {
          headingWaypoints = [...state.headingWaypoints, { waypointIndex, degrees }];
        }
      }
      return { ...pushUndo(state), headingWaypoints };
    }),

  // ---- Constraint mutations ----

  setConstraints: (constraints) =>
    set(() => ({ constraints: { ...constraints } })),

  // ---- Undo / redo ----

  undo: () =>
    set((state) => {
      if (state.undoStack.length === 0) return state;
      const snap = state.undoStack[state.undoStack.length - 1];
      const currentSnap = takeSnapshot(state);
      const redoStack = [...state.redoStack, currentSnap];
      if (redoStack.length > MAX_UNDO) redoStack.shift();
      return {
        undoStack: state.undoStack.slice(0, -1),
        redoStack,
        controlPoints: snap.controlPoints,
        headingWaypoints: snap.headingWaypoints,
        constraints: snap.constraints,
        selectedPointIndex: null,
      };
    }),

  redo: () =>
    set((state) => {
      if (state.redoStack.length === 0) return state;
      const snap = state.redoStack[state.redoStack.length - 1];
      const currentSnap = takeSnapshot(state);
      const undoStack = [...state.undoStack, currentSnap];
      if (undoStack.length > MAX_UNDO) undoStack.shift();
      return {
        redoStack: state.redoStack.slice(0, -1),
        undoStack,
        controlPoints: snap.controlPoints,
        headingWaypoints: snap.headingWaypoints,
        constraints: snap.constraints,
        selectedPointIndex: null,
      };
    }),

  // ---- Lifecycle ----

  clear: () =>
    set((state) => ({
      ...pushUndo(state),
      controlPoints: [] as Point[],
      headingWaypoints: [] as HeadingWaypoint[],
      constraints: { ...DEFAULT_CONSTRAINTS },
      selectedPointIndex: null,
    })),

  loadPath: (controlPoints, headingWaypoints, constraints) =>
    set((state) => ({
      ...pushUndo(state),
      controlPoints: controlPoints.map((p) => ({ ...p })),
      headingWaypoints: headingWaypoints.map((h) => ({ ...h })),
      constraints: { ...constraints },
      selectedPointIndex: null,
    })),
}));
