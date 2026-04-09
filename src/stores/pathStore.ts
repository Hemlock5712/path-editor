import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import {
  Point,
  NamedPoint,
  HeadingWaypoint,
  VelocityConstraints,
  ConstraintZone,
  RotationZone,
  NamedPath,
  DEFAULT_CONSTRAINTS,
  FIELD_HEIGHT,
} from '../types';
import { useSelectionStore } from './selectionStore';

interface Snapshot {
  controlPoints: Point[];
  controlPointRefs: (string | null)[];
  headingWaypoints: HeadingWaypoint[];
  constraints: VelocityConstraints;
  constraintZones: ConstraintZone[];
  rotationZones: RotationZone[];
  namedPoints: Record<string, NamedPoint>;
}

const MAX_UNDO = 50;

interface PathState {
  // Multi-path data
  paths: Record<string, NamedPath>;
  pathOrder: string[];
  activePathName: string | null;

  // Named points registry (global, not per-path)
  namedPoints: Record<string, NamedPoint>;

  // Active path flat fields (convenience selectors for existing consumers)
  controlPoints: Point[];
  controlPointRefs: (string | null)[];
  headingWaypoints: HeadingWaypoint[];
  constraints: VelocityConstraints;
  constraintZones: ConstraintZone[];
  rotationZones: RotationZone[];
  undoStack: Snapshot[];
  redoStack: Snapshot[];

  // Multi-path actions
  loadAllPaths: (paths: NamedPath[]) => void;
  setActivePath: (name: string) => void;
  addPath: (name: string) => void;
  renamePath: (oldName: string, newName: string) => void;
  deletePath: (name: string) => void;
  reorderPath: (fromIndex: number, toIndex: number) => void;
  getAllPaths: () => NamedPath[];

  // Point mutations (operate on active path)
  addPoint: (point: Point) => void;
  movePoint: (index: number, point: Point) => void;
  deletePoint: (index: number) => void;
  insertPointAfter: (index: number, point: Point) => void;
  // Named point actions
  addNamedPoint: (
    name: string,
    point: Point,
    headingDegrees?: number | null
  ) => void;
  deleteNamedPoint: (name: string) => void;
  renameNamedPoint: (oldName: string, newName: string) => void;
  updateNamedPointPosition: (name: string, point: Point) => void;
  placeNamedPoint: (name: string) => void;
  placeNamedPointAt: (name: string, afterIndex: number) => void;
  savePointAsNamed: (index: number, name: string) => void;
  linkPointToNamed: (index: number, name: string) => void;
  unlinkPoint: (index: number) => void;

  // Heading mutations
  setHeading: (waypointIndex: number, degrees: number | null) => void;

  // Constraint mutations
  setConstraints: (constraints: VelocityConstraints) => void;

  // Constraint zone mutations
  addConstraintZone: (zone: ConstraintZone) => void;
  updateConstraintZone: (index: number, zone: ConstraintZone) => void;
  deleteConstraintZone: (index: number) => void;

  // Rotation zone mutations
  addRotationZone: (zone: RotationZone) => void;
  updateRotationZone: (id: string, updates: Partial<RotationZone>) => void;
  moveRotationZoneHandle: (id: string, updates: Partial<RotationZone>) => void;
  deleteRotationZone: (id: string) => void;
  // Path transforms
  flipPathY: () => void;
  duplicatePath: (name?: string) => void;

  // Undo / redo
  pushUndoSnapshot: () => void;
  undo: () => void;
  redo: () => void;

  // Lifecycle
  clear: () => void;
}

function takeSnapshot(state: PathState): Snapshot {
  return {
    controlPoints: state.controlPoints.map((p) => ({ ...p })),
    controlPointRefs: [...state.controlPointRefs],
    headingWaypoints: state.headingWaypoints.map((h) => ({ ...h })),
    constraints: { ...state.constraints },
    constraintZones: state.constraintZones.map((z) => ({ ...z })),
    rotationZones: state.rotationZones.map((z) => ({
      ...z,
      targetPoint: { ...z.targetPoint },
    })),
    namedPoints: Object.fromEntries(
      Object.entries(state.namedPoints).map(([k, v]) => [k, { ...v }])
    ),
  };
}

function pushUndo(state: PathState): {
  undoStack: Snapshot[];
  redoStack: Snapshot[];
} {
  const snap = takeSnapshot(state);
  const stack = [...state.undoStack, snap];
  if (stack.length > MAX_UNDO) stack.shift();
  return { undoStack: stack, redoStack: [] };
}

/** Save the current flat fields back into the paths map for the active path. */
function syncActiveToMap(state: PathState): Record<string, NamedPath> {
  if (!state.activePathName || !state.paths[state.activePathName])
    return state.paths;
  return {
    ...state.paths,
    [state.activePathName]: {
      ...state.paths[state.activePathName],
      controlPoints: state.controlPoints.map((p) => ({ ...p })),
      controlPointRefs: [...state.controlPointRefs],
      headingWaypoints: state.headingWaypoints.map((h) => ({ ...h })),
      constraints: { ...state.constraints },
      constraintZones: state.constraintZones.map((z) => ({ ...z })),
      rotationZones: state.rotationZones.map((z) => ({
        ...z,
        targetPoint: { ...z.targetPoint },
      })),
    },
  };
}

/**
 * Propagate named point positions to all paths. Returns updated paths map
 * and (if the active path was affected) updated flat controlPoints.
 */
function propagateNamedPoints(
  namedPoints: Record<string, NamedPoint>,
  paths: Record<string, NamedPath>,
  activePathName: string | null
): { paths: Record<string, NamedPath>; controlPoints?: Point[] } {
  // Early out: check if any path references any named point at all
  const namedPointNames = new Set(Object.keys(namedPoints));
  let anyRef = false;
  for (const path of Object.values(paths)) {
    if (path.controlPointRefs?.some((ref) => ref && namedPointNames.has(ref))) {
      anyRef = true;
      break;
    }
  }
  if (!anyRef) return { paths };

  const updatedPaths: Record<string, NamedPath> = {};
  let activeControlPoints: Point[] | undefined;

  for (const [pathName, path] of Object.entries(paths)) {
    const refs = path.controlPointRefs || [];
    let changed = false;
    const newPoints = path.controlPoints.map((pt, i) => {
      const ref = refs[i];
      if (ref && namedPoints[ref]) {
        const np = namedPoints[ref];
        if (pt.x !== np.x || pt.y !== np.y) {
          changed = true;
          return { x: np.x, y: np.y };
        }
      }
      return pt;
    });
    updatedPaths[pathName] = changed
      ? { ...path, controlPoints: newPoints }
      : path;
    if (pathName === activePathName && changed) {
      activeControlPoints = newPoints.map((p) => ({ ...p }));
    }
  }

  return { paths: updatedPaths, controlPoints: activeControlPoints };
}

function mirrorHeading(degrees: number | null): number | null {
  if (degrees === null) return null;
  return ((-degrees % 360) + 360) % 360;
}

/** Load named points from localStorage. */
function loadNamedPointsFromStorage(): Record<string, NamedPoint> {
  try {
    const raw = localStorage.getItem('pathEditor_namedPoints');
    if (raw) {
      const parsed = JSON.parse(raw);
      for (const key of Object.keys(parsed)) {
        if (parsed[key].headingDegrees === undefined) {
          parsed[key].headingDegrees = null;
        }
      }
      return parsed;
    }
  } catch {
    /* ignore */
  }
  return {};
}


export const usePathStore = create<PathState>()(
  persist(
  (set, get) => ({
  paths: {
    'Path 1': {
      name: 'Path 1',
      controlPoints: [],
      controlPointRefs: [],
      headingWaypoints: [],
      constraints: { ...DEFAULT_CONSTRAINTS },
      constraintZones: [],
      rotationZones: [],
    },
  },
  pathOrder: ['Path 1'],
  activePathName: 'Path 1',

  namedPoints: loadNamedPointsFromStorage(),

  controlPoints: [],
  controlPointRefs: [],
  headingWaypoints: [],
  constraints: { ...DEFAULT_CONSTRAINTS },
  constraintZones: [],
  rotationZones: [],

  undoStack: [],
  redoStack: [],

  // ---- Multi-path actions ----

  loadAllPaths: (namedPaths) =>
    set(() => {
      useSelectionStore.getState().clearSelection();
      const paths: Record<string, NamedPath> = {};
      for (const p of namedPaths) {
        paths[p.name] = {
          ...p,
          controlPointRefs:
            p.controlPointRefs || Array(p.controlPoints.length).fill(null),
        };
      }
      const pathOrder = namedPaths.map((p) => p.name);
      const firstName = namedPaths.length > 0 ? namedPaths[0].name : null;
      const active = firstName ? paths[firstName] : null;
      return {
        paths,
        pathOrder,
        activePathName: firstName,
        controlPoints: active
          ? active.controlPoints.map((p) => ({ ...p }))
          : [],
        controlPointRefs: active ? [...active.controlPointRefs] : [],
        headingWaypoints: active
          ? active.headingWaypoints.map((h) => ({ ...h }))
          : [],
        constraints: active
          ? { ...active.constraints }
          : { ...DEFAULT_CONSTRAINTS },
        constraintZones: active
          ? active.constraintZones.map((z) => ({ ...z }))
          : [],
        rotationZones: active
          ? active.rotationZones.map((z) => ({
              ...z,
              targetPoint: { ...z.targetPoint },
            }))
          : [],
        undoStack: [],
        redoStack: [],
      };
    }),

  setActivePath: (name) =>
    set((state) => {
      if (name === state.activePathName) return state;
      useSelectionStore.getState().clearSelection();
      const updatedPaths = syncActiveToMap(state);
      const target = updatedPaths[name];
      if (!target) return state;
      return {
        paths: updatedPaths,
        activePathName: name,
        controlPoints: target.controlPoints.map((p) => ({ ...p })),
        controlPointRefs: [...(target.controlPointRefs || [])],
        headingWaypoints: target.headingWaypoints.map((h) => ({ ...h })),
        constraints: { ...target.constraints },
        constraintZones: target.constraintZones.map((z) => ({ ...z })),
        rotationZones: target.rotationZones.map((z) => ({
          ...z,
          targetPoint: { ...z.targetPoint },
        })),
        undoStack: [],
        redoStack: [],
      };
    }),

  deletePath: (name) =>
    set((state) => {
      useSelectionStore.getState().clearSelection();
      const updatedPaths = syncActiveToMap(state);
      const { [name]: _, ...remaining } = updatedPaths;
      const newOrder = state.pathOrder.filter((n) => n !== name);
      const newActive = newOrder.length > 0 ? newOrder[0] : null;
      const target = newActive ? remaining[newActive] : null;
      return {
        paths: remaining,
        pathOrder: newOrder,
        activePathName: newActive,
        controlPoints: target
          ? target.controlPoints.map((p) => ({ ...p }))
          : [],
        controlPointRefs: target ? [...(target.controlPointRefs || [])] : [],
        headingWaypoints: target
          ? target.headingWaypoints.map((h) => ({ ...h }))
          : [],
        constraints: target
          ? { ...target.constraints }
          : { ...DEFAULT_CONSTRAINTS },
        constraintZones: target
          ? target.constraintZones.map((z) => ({ ...z }))
          : [],
        rotationZones: target
          ? target.rotationZones.map((z) => ({
              ...z,
              targetPoint: { ...z.targetPoint },
            }))
          : [],
        undoStack: [],
        redoStack: [],
      };
    }),

  reorderPath: (fromIndex, toIndex) =>
    set((state) => {
      const order = [...state.pathOrder];
      const [moved] = order.splice(fromIndex, 1);
      order.splice(toIndex, 0, moved);
      return { pathOrder: order };
    }),

  getAllPaths: () => {
    const state = get();
    const updatedPaths = syncActiveToMap(state);
    return Object.values(updatedPaths);
  },

  addPath: (name) =>
    set((state) => {
      useSelectionStore.getState().clearSelection();
      const updatedPaths = syncActiveToMap(state);
      // Auto-increment name if collision
      let finalName = name;
      let counter = 2;
      while (updatedPaths[finalName]) {
        finalName = `${name} ${counter}`;
        counter++;
      }
      const newPath: NamedPath = {
        name: finalName,
        controlPoints: [],
        controlPointRefs: [],
        headingWaypoints: [],
        constraints: { ...DEFAULT_CONSTRAINTS },
        constraintZones: [],
        rotationZones: [],
      };
      return {
        paths: { ...updatedPaths, [finalName]: newPath },
        pathOrder: [...state.pathOrder, finalName],
        activePathName: finalName,
        controlPoints: [],
        controlPointRefs: [],
        headingWaypoints: [],
        constraints: { ...DEFAULT_CONSTRAINTS },
        constraintZones: [],
        rotationZones: [],
        undoStack: [],
        redoStack: [],
      };
    }),

  renamePath: (oldName, newName) =>
    set((state) => {
      const trimmed = newName.trim();
      if (!trimmed || trimmed === oldName || state.paths[trimmed]) return state;
      const updatedPaths = syncActiveToMap(state);
      const pathData = updatedPaths[oldName];
      if (!pathData) return state;
      const { [oldName]: _, ...rest } = updatedPaths;
      return {
        paths: { ...rest, [trimmed]: { ...pathData, name: trimmed } },
        pathOrder: state.pathOrder.map((n) => (n === oldName ? trimmed : n)),
        activePathName:
          state.activePathName === oldName ? trimmed : state.activePathName,
      };
    }),

  // ---- Point mutations ----

  addPoint: (point) =>
    set((state) => ({
      ...pushUndo(state),
      controlPoints: [...state.controlPoints, { ...point }],
      controlPointRefs: [...state.controlPointRefs, null],
    })),

  movePoint: (index, point) =>
    set((state) => {
      const ref = state.controlPointRefs[index];
      if (ref && state.namedPoints[ref]) {
        // Moving a linked point: update the named point registry and propagate
        const newNamedPoints: Record<string, NamedPoint> = {
          ...state.namedPoints,
          [ref]: { ...state.namedPoints[ref], x: point.x, y: point.y },
        };
        const mirror = state.namedPoints[ref].mirrorName;
        if (mirror && newNamedPoints[mirror]) {
          newNamedPoints[mirror] = {
            ...newNamedPoints[mirror],
            x: point.x,
            y: FIELD_HEIGHT - point.y,
          };
        }


        // Sync active path into map first so propagation sees latest state
        const synced = syncActiveToMap(state);
        // Update the active path's point in the synced map
        const activePath = synced[state.activePathName!];
        const updatedActivePoints = activePath.controlPoints.map((p, i) =>
          i === index ? { ...point } : p
        );
        const syncedWithMove = {
          ...synced,
          [state.activePathName!]: {
            ...activePath,
            controlPoints: updatedActivePoints,
          },
        };

        const propagated = propagateNamedPoints(
          newNamedPoints,
          syncedWithMove,
          state.activePathName
        );

        // Extract the active path's control points from propagated result
        const activeFromPropagated = propagated.paths[state.activePathName!];
        return {
          namedPoints: newNamedPoints,
          paths: propagated.paths,
          controlPoints: activeFromPropagated.controlPoints.map((p) => ({
            ...p,
          })),
        };
      }

      // Normal unlinked point
      return {
        controlPoints: state.controlPoints.map((p, i) =>
          i === index ? { ...point } : p
        ),
      };
    }),

  deletePoint: (index) =>
    set((state) => {
      useSelectionStore.getState().clearSelection();
      return {
        ...pushUndo(state),
        controlPoints: state.controlPoints.filter((_, i) => i !== index),
        controlPointRefs: state.controlPointRefs.filter((_, i) => i !== index),
        headingWaypoints: state.headingWaypoints.filter(
          (hw) => Math.round(hw.waypointIndex) !== index
        ),
        constraintZones: state.constraintZones
          .filter(
            (z) =>
              z.startWaypointIndex !== index && z.endWaypointIndex !== index
          )
          .map((z) => ({
            ...z,
            startWaypointIndex:
              z.startWaypointIndex > index
                ? z.startWaypointIndex - 1
                : z.startWaypointIndex,
            endWaypointIndex:
              z.endWaypointIndex > index
                ? z.endWaypointIndex - 1
                : z.endWaypointIndex,
          })),
        rotationZones: state.rotationZones
          .filter(
            (z) =>
              !(
                Math.ceil(z.startWaypointIndex) === index &&
                Math.floor(z.endWaypointIndex) === index
              )
          )
          .map((z) => ({
            ...z,
            startWaypointIndex:
              z.startWaypointIndex > index
                ? z.startWaypointIndex - 1
                : z.startWaypointIndex,
            endWaypointIndex:
              z.endWaypointIndex > index
                ? z.endWaypointIndex - 1
                : z.endWaypointIndex,
          })),
      };
    }),

  insertPointAfter: (index, point) =>
    set((state) => {
      const controlPoints = [...state.controlPoints];
      controlPoints.splice(index + 1, 0, { ...point });

      const controlPointRefs = [...state.controlPointRefs];
      controlPointRefs.splice(index + 1, 0, null);

      const headingWaypoints = state.headingWaypoints.map((hw) => ({
        ...hw,
        waypointIndex:
          hw.waypointIndex > index ? hw.waypointIndex + 1 : hw.waypointIndex,
      }));

      const constraintZones = state.constraintZones.map((z) => ({
        ...z,
        startWaypointIndex:
          z.startWaypointIndex > index
            ? z.startWaypointIndex + 1
            : z.startWaypointIndex,
        endWaypointIndex:
          z.endWaypointIndex > index
            ? z.endWaypointIndex + 1
            : z.endWaypointIndex,
      }));

      const rotationZones = state.rotationZones.map((z) => ({
        ...z,
        startWaypointIndex:
          z.startWaypointIndex > index
            ? z.startWaypointIndex + 1
            : z.startWaypointIndex,
        endWaypointIndex:
          z.endWaypointIndex > index
            ? z.endWaypointIndex + 1
            : z.endWaypointIndex,
      }));

      return {
        ...pushUndo(state),
        controlPoints,
        controlPointRefs,
        headingWaypoints,
        constraintZones,
        rotationZones,
      };
    }),

  // ---- Named point actions ----

  addNamedPoint: (name, point, headingDegrees) =>
    set((state) => {
      const heading = headingDegrees ?? null;
      const mirrorName = `${name} (Mirror)`;
      const newNamedPoints: Record<string, NamedPoint> = {
        ...state.namedPoints,
        [name]: {
          name,
          x: point.x,
          y: point.y,
          headingDegrees: heading,
          mirrorName,
        },
        [mirrorName]: {
          name: mirrorName,
          x: point.x,
          y: FIELD_HEIGHT - point.y,
          headingDegrees: mirrorHeading(heading),
          mirrorName: name,
        },
      };

      return { namedPoints: newNamedPoints };
    }),

  deleteNamedPoint: (name) =>
    set((state) => {
      const np = state.namedPoints[name];
      if (!np) return state;

      const { [name]: _, ...rest } = state.namedPoints;
      // Also remove the mirror if it exists
      let newNamedPoints = rest;
      if (np.mirrorName && newNamedPoints[np.mirrorName]) {
        const { [np.mirrorName]: __, ...withoutMirror } = newNamedPoints;
        newNamedPoints = withoutMirror;
      }

      // Unlink all refs in all paths
      const synced = syncActiveToMap(state);
      const updatedPaths: Record<string, NamedPath> = {};
      for (const [pName, path] of Object.entries(synced)) {
        const refs = (path.controlPointRefs || []).map((r) =>
          r === name || r === np.mirrorName ? null : r
        );
        updatedPaths[pName] = { ...path, controlPointRefs: refs };
      }

      const activeRefs = state.controlPointRefs.map((r) =>
        r === name || r === np.mirrorName ? null : r
      );


      return {
        namedPoints: newNamedPoints,
        paths: updatedPaths,
        controlPointRefs: activeRefs,
      };
    }),

  renameNamedPoint: (oldName, newName) =>
    set((state) => {
      const trimmed = newName.trim();
      if (!trimmed || trimmed === oldName || state.namedPoints[trimmed])
        return state;
      const np = state.namedPoints[oldName];
      if (!np) return state;

      const { [oldName]: _, ...rest } = state.namedPoints;
      const newNamedPoints: Record<string, NamedPoint> = {
        ...rest,
        [trimmed]: { ...np, name: trimmed },
      };

      // Update mirror's pointer
      if (np.mirrorName && newNamedPoints[np.mirrorName]) {
        newNamedPoints[np.mirrorName] = {
          ...newNamedPoints[np.mirrorName],
          mirrorName: trimmed,
        };
      }

      // Update all refs across all paths
      const synced = syncActiveToMap(state);
      const updatedPaths: Record<string, NamedPath> = {};
      for (const [pName, path] of Object.entries(synced)) {
        const refs = (path.controlPointRefs || []).map((r) =>
          r === oldName ? trimmed : r
        );
        updatedPaths[pName] = { ...path, controlPointRefs: refs };
      }

      const activeRefs = state.controlPointRefs.map((r) =>
        r === oldName ? trimmed : r
      );


      return {
        namedPoints: newNamedPoints,
        paths: updatedPaths,
        controlPointRefs: activeRefs,
      };
    }),

  updateNamedPointPosition: (name, point) =>
    set((state) => {
      const np = state.namedPoints[name];
      if (!np) return state;

      const newNamedPoints: Record<string, NamedPoint> = {
        ...state.namedPoints,
        [name]: { ...np, x: point.x, y: point.y },
      };
      if (np.mirrorName && newNamedPoints[np.mirrorName]) {
        newNamedPoints[np.mirrorName] = {
          ...newNamedPoints[np.mirrorName],
          x: point.x,
          y: FIELD_HEIGHT - point.y,
        };
      }


      const synced = syncActiveToMap(state);
      const propagated = propagateNamedPoints(
        newNamedPoints,
        synced,
        state.activePathName
      );
      return {
        ...pushUndo(state),
        namedPoints: newNamedPoints,
        paths: propagated.paths,
        ...(propagated.controlPoints
          ? { controlPoints: propagated.controlPoints }
          : {}),
      };
    }),

  placeNamedPoint: (name) =>
    set((state) => {
      const np = state.namedPoints[name];
      if (!np) return state;
      const newIndex = state.controlPoints.length;
      const headingWaypoints =
        np.headingDegrees !== null
          ? [
              ...state.headingWaypoints,
              { waypointIndex: newIndex, degrees: np.headingDegrees },
            ]
          : state.headingWaypoints;
      return {
        ...pushUndo(state),
        controlPoints: [...state.controlPoints, { x: np.x, y: np.y }],
        controlPointRefs: [...state.controlPointRefs, name],
        headingWaypoints,
      };
    }),

  placeNamedPointAt: (name, afterIndex) =>
    set((state) => {
      const np = state.namedPoints[name];
      if (!np) return state;

      const controlPoints = [...state.controlPoints];
      controlPoints.splice(afterIndex + 1, 0, { x: np.x, y: np.y });

      const controlPointRefs = [...state.controlPointRefs];
      controlPointRefs.splice(afterIndex + 1, 0, name);

      const headingWaypoints = state.headingWaypoints.map((hw) => ({
        ...hw,
        waypointIndex:
          hw.waypointIndex > afterIndex
            ? hw.waypointIndex + 1
            : hw.waypointIndex,
      }));
      if (np.headingDegrees !== null) {
        headingWaypoints.push({
          waypointIndex: afterIndex + 1,
          degrees: np.headingDegrees,
        });
      }

      const constraintZones = state.constraintZones.map((z) => ({
        ...z,
        startWaypointIndex:
          z.startWaypointIndex > afterIndex
            ? z.startWaypointIndex + 1
            : z.startWaypointIndex,
        endWaypointIndex:
          z.endWaypointIndex > afterIndex
            ? z.endWaypointIndex + 1
            : z.endWaypointIndex,
      }));

      const rotationZones = state.rotationZones.map((z) => ({
        ...z,
        startWaypointIndex:
          z.startWaypointIndex > afterIndex
            ? z.startWaypointIndex + 1
            : z.startWaypointIndex,
        endWaypointIndex:
          z.endWaypointIndex > afterIndex
            ? z.endWaypointIndex + 1
            : z.endWaypointIndex,
      }));

      return {
        ...pushUndo(state),
        controlPoints,
        controlPointRefs,
        headingWaypoints,
        constraintZones,
        rotationZones,
      };
    }),

  savePointAsNamed: (index, name) =>
    set((state) => {
      const pt = state.controlPoints[index];
      if (!pt) return state;

      const headingWp = state.headingWaypoints.find(
        (hw) => Math.round(hw.waypointIndex) === index
      );
      const heading = headingWp ? headingWp.degrees : null;

      const mirrorName = `${name} (Mirror)`;
      const newNamedPoints: Record<string, NamedPoint> = {
        ...state.namedPoints,
        [name]: { name, x: pt.x, y: pt.y, headingDegrees: heading, mirrorName },
        [mirrorName]: {
          name: mirrorName,
          x: pt.x,
          y: FIELD_HEIGHT - pt.y,
          headingDegrees: mirrorHeading(heading),
          mirrorName: name,
        },
      };

      const controlPointRefs = [...state.controlPointRefs];
      controlPointRefs[index] = name;


      return { namedPoints: newNamedPoints, controlPointRefs };
    }),

  linkPointToNamed: (index, name) =>
    set((state) => {
      const np = state.namedPoints[name];
      if (!np) return state;

      const controlPoints = state.controlPoints.map((p, i) =>
        i === index ? { x: np.x, y: np.y } : p
      );
      const controlPointRefs = [...state.controlPointRefs];
      controlPointRefs[index] = name;

      let headingWaypoints = state.headingWaypoints;
      if (np.headingDegrees !== null) {
        const existingIdx = headingWaypoints.findIndex(
          (hw) => Math.round(hw.waypointIndex) === index
        );
        if (existingIdx >= 0) {
          headingWaypoints = headingWaypoints.map((hw, i) =>
            i === existingIdx
              ? { waypointIndex: index, degrees: np.headingDegrees! }
              : hw
          );
        } else {
          headingWaypoints = [
            ...headingWaypoints,
            { waypointIndex: index, degrees: np.headingDegrees },
          ];
        }
      }

      return {
        ...pushUndo(state),
        controlPoints,
        controlPointRefs,
        headingWaypoints,
      };
    }),

  unlinkPoint: (index) =>
    set((state) => {
      const controlPointRefs = [...state.controlPointRefs];
      controlPointRefs[index] = null;
      return { controlPointRefs };
    }),

  // ---- Heading mutations ----

  setHeading: (waypointIndex, degrees) =>
    set((state) => {
      let headingWaypoints: HeadingWaypoint[];
      if (degrees === null) {
        headingWaypoints = state.headingWaypoints.filter(
          (hw) => Math.round(hw.waypointIndex) !== waypointIndex
        );
      } else {
        const existing = state.headingWaypoints.findIndex(
          (hw) => Math.round(hw.waypointIndex) === waypointIndex
        );
        if (existing >= 0) {
          headingWaypoints = state.headingWaypoints.map((hw, i) =>
            i === existing ? { waypointIndex, degrees } : hw
          );
        } else {
          headingWaypoints = [
            ...state.headingWaypoints,
            { waypointIndex, degrees },
          ];
        }
      }
      // Sync heading back to linked named point and its mirror
      const refName = state.controlPointRefs[waypointIndex];
      let namedPoints = state.namedPoints;
      if (refName && namedPoints[refName]) {
        const np = namedPoints[refName];
        namedPoints = { ...namedPoints, [refName]: { ...np, headingDegrees: degrees } };
        if (np.mirrorName && namedPoints[np.mirrorName]) {
          const mirror = namedPoints[np.mirrorName];
          namedPoints = { ...namedPoints, [np.mirrorName]: { ...mirror, headingDegrees: mirrorHeading(degrees) } };
        }
      }

      return { ...pushUndo(state), headingWaypoints, namedPoints };
    }),

  // ---- Constraint mutations ----

  setConstraints: (constraints) =>
    set(() => ({ constraints: { ...constraints } })),

  // ---- Constraint zone mutations ----

  addConstraintZone: (zone) =>
    set((state) => ({
      ...pushUndo(state),
      constraintZones: [
        ...state.constraintZones,
        { ...zone, id: zone.id || crypto.randomUUID() },
      ],
    })),

  updateConstraintZone: (index, zone) =>
    set((state) => ({
      ...pushUndo(state),
      constraintZones: state.constraintZones.map((z, i) =>
        i === index ? { ...zone } : z
      ),
    })),

  deleteConstraintZone: (index) =>
    set((state) => ({
      ...pushUndo(state),
      constraintZones: state.constraintZones.filter((_, i) => i !== index),
    })),

  // ---- Rotation zone mutations ----

  addRotationZone: (zone) =>
    set((state) => {
      useSelectionStore.getState().selectZone(zone.id);
      return {
        ...pushUndo(state),
        rotationZones: [
          ...state.rotationZones,
          { ...zone, targetPoint: { ...zone.targetPoint } },
        ],
      };
    }),

  updateRotationZone: (id, updates) =>
    set((state) => ({
      ...pushUndo(state),
      rotationZones: state.rotationZones.map((z) =>
        z.id === id
          ? {
              ...z,
              ...updates,
              targetPoint: updates.targetPoint
                ? { ...updates.targetPoint }
                : z.targetPoint,
            }
          : z
      ),
    })),

  moveRotationZoneHandle: (id, updates) =>
    set((state) => ({
      rotationZones: state.rotationZones.map((z) =>
        z.id === id
          ? {
              ...z,
              ...updates,
              targetPoint: updates.targetPoint
                ? { ...updates.targetPoint }
                : z.targetPoint,
            }
          : z
      ),
    })),

  deleteRotationZone: (id) =>
    set((state) => {
      if (useSelectionStore.getState().selectedZoneId === id) {
        useSelectionStore.getState().selectZone(null);
      }
      return {
        ...pushUndo(state),
        rotationZones: state.rotationZones.filter((z) => z.id !== id),
      };
    }),

  // ---- Path transforms ----

  flipPathY: () =>
    set((state) => {
      // For linked points, swap refs to their mirror counterpart
      const controlPointRefs = state.controlPointRefs.map((ref) => {
        if (!ref) return null;
        const np = state.namedPoints[ref];
        if (np && np.mirrorName && state.namedPoints[np.mirrorName]) {
          return np.mirrorName;
        }
        return null; // Unlink if no mirror exists
      });

      // Flip coordinates: for linked points use the mirror's coords, for unlinked just flip
      const controlPoints = state.controlPoints.map((p, i) => {
        const newRef = controlPointRefs[i];
        if (newRef && state.namedPoints[newRef]) {
          const mirror = state.namedPoints[newRef];
          return { x: mirror.x, y: mirror.y };
        }
        return { x: p.x, y: FIELD_HEIGHT - p.y };
      });

      return {
        ...pushUndo(state),
        controlPoints,
        controlPointRefs,
        headingWaypoints: state.headingWaypoints.map((hw) => ({
          ...hw,
          degrees: ((-hw.degrees % 360) + 360) % 360,
        })),
        rotationZones: state.rotationZones.map((z) => ({
          ...z,
          targetPoint: {
            x: z.targetPoint.x,
            y: FIELD_HEIGHT - z.targetPoint.y,
          },
        })),
      };
    }),

  duplicatePath: (name?: string) =>
    set((state) => {
      const sourceName = name || state.activePathName;
      if (!sourceName) return state;
      useSelectionStore.getState().clearSelection();
      const updatedPaths = syncActiveToMap(state);
      if (!updatedPaths[sourceName]) return state;
      const baseName = sourceName + ' (Copy)';
      let finalName = baseName;
      let counter = 2;
      while (updatedPaths[finalName]) {
        finalName = `${baseName} ${counter}`;
        counter++;
      }
      const src = updatedPaths[sourceName];
      const copy: NamedPath = {
        name: finalName,
        controlPoints: src.controlPoints.map((p) => ({ ...p })),
        controlPointRefs: [...(src.controlPointRefs || [])],
        headingWaypoints: src.headingWaypoints.map((h) => ({ ...h })),
        constraints: { ...src.constraints },
        constraintZones: src.constraintZones.map((z) => ({ ...z })),
        rotationZones: src.rotationZones.map((z) => ({
          ...z,
          targetPoint: { ...z.targetPoint },
        })),
      };
      return {
        paths: { ...updatedPaths, [finalName]: copy },
        pathOrder: [...state.pathOrder, finalName],
        activePathName: finalName,
        controlPoints: copy.controlPoints.map((p) => ({ ...p })),
        controlPointRefs: [...copy.controlPointRefs],
        headingWaypoints: copy.headingWaypoints.map((h) => ({ ...h })),
        constraints: { ...copy.constraints },
        constraintZones: copy.constraintZones.map((z) => ({ ...z })),
        rotationZones: copy.rotationZones.map((z) => ({
          ...z,
          targetPoint: { ...z.targetPoint },
        })),
        undoStack: [],
        redoStack: [],
      };
    }),

  // ---- Undo / redo ----

  pushUndoSnapshot: () => set((state) => pushUndo(state)),

  undo: () =>
    set((state) => {
      if (state.undoStack.length === 0) return state;
      useSelectionStore.getState().clearSelection();
      const snap = state.undoStack[state.undoStack.length - 1];
      const currentSnap = takeSnapshot(state);
      const redoStack = [...state.redoStack, currentSnap];
      if (redoStack.length > MAX_UNDO) redoStack.shift();

      // Propagate restored named points to all paths
      const synced = syncActiveToMap(state);
      const propagated = propagateNamedPoints(
        snap.namedPoints,
        synced,
        state.activePathName
      );


      return {
        undoStack: state.undoStack.slice(0, -1),
        redoStack,
        controlPoints: snap.controlPoints,
        controlPointRefs: snap.controlPointRefs,
        headingWaypoints: snap.headingWaypoints,
        constraints: snap.constraints,
        constraintZones: snap.constraintZones,
        rotationZones: snap.rotationZones,
        namedPoints: snap.namedPoints,
        paths: propagated.paths,
      };
    }),

  redo: () =>
    set((state) => {
      if (state.redoStack.length === 0) return state;
      useSelectionStore.getState().clearSelection();
      const snap = state.redoStack[state.redoStack.length - 1];
      const currentSnap = takeSnapshot(state);
      const undoStack = [...state.undoStack, currentSnap];
      if (undoStack.length > MAX_UNDO) undoStack.shift();

      // Propagate restored named points to all paths
      const synced = syncActiveToMap(state);
      const propagated = propagateNamedPoints(
        snap.namedPoints,
        synced,
        state.activePathName
      );


      return {
        redoStack: state.redoStack.slice(0, -1),
        undoStack,
        controlPoints: snap.controlPoints,
        controlPointRefs: snap.controlPointRefs,
        headingWaypoints: snap.headingWaypoints,
        constraints: snap.constraints,
        constraintZones: snap.constraintZones,
        rotationZones: snap.rotationZones,
        namedPoints: snap.namedPoints,
        paths: propagated.paths,
      };
    }),

  // ---- Lifecycle ----

  clear: () =>
    set((state) => {
      useSelectionStore.getState().clearSelection();
      return {
        ...pushUndo(state),
        controlPoints: [] as Point[],
        controlPointRefs: [] as (string | null)[],
        headingWaypoints: [] as HeadingWaypoint[],
        constraints: { ...DEFAULT_CONSTRAINTS },
        constraintZones: [] as ConstraintZone[],
        rotationZones: [] as RotationZone[],
      };
    }),
}),
  {
    name: 'frc-path-editor-paths',
    partialize: (state) => ({
      paths: syncActiveToMap(state),
      pathOrder: state.pathOrder,
      activePathName: state.activePathName,
      namedPoints: state.namedPoints,
      controlPoints: state.controlPoints,
      controlPointRefs: state.controlPointRefs,
      headingWaypoints: state.headingWaypoints,
      constraints: state.constraints,
      constraintZones: state.constraintZones,
      rotationZones: state.rotationZones,
    }),
    onRehydrateStorage: () => () => {
      // Clean up old namedPoints localStorage key after migration
      try {
        localStorage.removeItem('pathEditor_namedPoints');
      } catch {
        /* ignore */
      }
    },
  }
  )
);
