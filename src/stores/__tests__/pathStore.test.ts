import { describe, it, expect, beforeEach } from 'vitest';
import { usePathStore } from '../pathStore';
import { useSelectionStore } from '../selectionStore';
import { DEFAULT_CONSTRAINTS } from '../../types';

function resetStores() {
  usePathStore.setState({
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
    namedPoints: {},
    controlPoints: [],
    controlPointRefs: [],
    headingWaypoints: [],
    constraints: { ...DEFAULT_CONSTRAINTS },
    constraintZones: [],
    rotationZones: [],
    undoStack: [],
    redoStack: [],
  });
  useSelectionStore.setState({
    selectedPointIndex: null,
    selectedZoneId: null,
  });
}

describe('pathStore', () => {
  beforeEach(resetStores);

  describe('addPoint / movePoint / deletePoint', () => {
    it('adds a point', () => {
      usePathStore.getState().addPoint({ x: 1, y: 2 });
      expect(usePathStore.getState().controlPoints).toHaveLength(1);
      expect(usePathStore.getState().controlPoints[0]).toEqual({ x: 1, y: 2 });
    });

    it('moves a point', () => {
      usePathStore.getState().addPoint({ x: 0, y: 0 });
      usePathStore.getState().addPoint({ x: 5, y: 5 });
      usePathStore.getState().movePoint(0, { x: 2, y: 3 });
      expect(usePathStore.getState().controlPoints[0]).toEqual({ x: 2, y: 3 });
    });

    it('deletes a point and clears selection', () => {
      usePathStore.getState().addPoint({ x: 0, y: 0 });
      usePathStore.getState().addPoint({ x: 1, y: 1 });
      usePathStore.getState().addPoint({ x: 2, y: 2 });
      useSelectionStore.getState().selectPoint(1);

      usePathStore.getState().deletePoint(1);

      expect(usePathStore.getState().controlPoints).toHaveLength(2);
      expect(useSelectionStore.getState().selectedPointIndex).toBeNull();
    });
  });

  describe('undo / redo', () => {
    it('undoes addPoint', () => {
      usePathStore.getState().addPoint({ x: 1, y: 1 });
      expect(usePathStore.getState().controlPoints).toHaveLength(1);

      usePathStore.getState().undo();
      expect(usePathStore.getState().controlPoints).toHaveLength(0);
    });

    it('redoes after undo', () => {
      usePathStore.getState().addPoint({ x: 1, y: 1 });
      usePathStore.getState().undo();
      usePathStore.getState().redo();
      expect(usePathStore.getState().controlPoints).toHaveLength(1);
    });

    it('redo stack is cleared on new mutation', () => {
      usePathStore.getState().addPoint({ x: 1, y: 1 });
      usePathStore.getState().undo();
      usePathStore.getState().addPoint({ x: 2, y: 2 });
      expect(usePathStore.getState().redoStack).toHaveLength(0);
    });
  });

  describe('multi-path', () => {
    it('adds a new path and switches to it', () => {
      usePathStore.getState().addPath('Path 2');
      expect(usePathStore.getState().activePathName).toBe('Path 2');
      expect(usePathStore.getState().pathOrder).toEqual(['Path 1', 'Path 2']);
    });

    it('preserves points when switching paths', () => {
      usePathStore.getState().addPoint({ x: 1, y: 2 });
      usePathStore.getState().addPath('Path 2');
      usePathStore.getState().addPoint({ x: 3, y: 4 });

      // Switch back
      usePathStore.getState().setActivePath('Path 1');
      expect(usePathStore.getState().controlPoints).toEqual([{ x: 1, y: 2 }]);

      // Switch forward
      usePathStore.getState().setActivePath('Path 2');
      expect(usePathStore.getState().controlPoints).toEqual([{ x: 3, y: 4 }]);
    });

    it('deletes a path and activates another', () => {
      usePathStore.getState().addPath('Path 2');
      usePathStore.getState().deletePath('Path 2');
      expect(usePathStore.getState().activePathName).toBe('Path 1');
      expect(usePathStore.getState().pathOrder).toEqual(['Path 1']);
    });

    it('reorders paths', () => {
      usePathStore.getState().addPath('Path 2');
      usePathStore.getState().addPath('Path 3');
      usePathStore.getState().reorderPath(2, 0);
      expect(usePathStore.getState().pathOrder).toEqual([
        'Path 3',
        'Path 1',
        'Path 2',
      ]);
    });
  });

  describe('pushUndoSnapshot / moveRotationZoneHandle', () => {
    it('moveRotationZoneHandle does not push undo', () => {
      usePathStore.getState().addPoint({ x: 0, y: 0 });
      usePathStore.getState().addPoint({ x: 5, y: 5 });
      usePathStore.getState().addRotationZone({
        id: 'rz1',
        startWaypointIndex: 0,
        endWaypointIndex: 1,
        targetPoint: { x: 2, y: 2 },
      });
      const stackBefore = usePathStore.getState().undoStack.length;

      usePathStore.getState().moveRotationZoneHandle('rz1', {
        targetPoint: { x: 3, y: 3 },
      });

      expect(usePathStore.getState().undoStack.length).toBe(stackBefore);
      const zone = usePathStore
        .getState()
        .rotationZones.find((z) => z.id === 'rz1');
      expect(zone?.targetPoint).toEqual({ x: 3, y: 3 });
    });

    it('pushUndoSnapshot pushes to undo stack', () => {
      usePathStore.getState().addPoint({ x: 0, y: 0 });
      const stackBefore = usePathStore.getState().undoStack.length;
      usePathStore.getState().pushUndoSnapshot();
      expect(usePathStore.getState().undoStack.length).toBe(stackBefore + 1);
    });
  });
});
