import { useRef, useCallback, useState } from 'react';
import { usePathStore } from '../stores/pathStore';
import { useSelectionStore } from '../stores/selectionStore';
import { useEditorStore } from '../stores/editorStore';
import {
  canvasToField,
  clampToField,
  type CanvasTransform,
} from '../utils/canvasTransform';
import { snapToGrid } from '../utils/snapping';
import { hitTestControlPoint, hitTestRotationZone } from '../utils/hitTesting';
import { useTooltip } from './useTooltip';
import { SplinePath } from '../math/SplinePath';
import type { Point } from '../types';
import type { ContextMenuState } from './useContextMenu';

interface InactivePath {
  name: string;
  controlPoints: Point[];
  spline: SplinePath | null;
}

interface UseCanvasInteractionArgs {
  canvasRef: React.RefObject<HTMLCanvasElement | null>;
  containerRef: React.RefObject<HTMLDivElement | null>;
  cw: number;
  ch: number;
  transform: CanvasTransform;
  splinePath: SplinePath | null;
  inactivePaths: InactivePath[];
  menu: ContextMenuState;
  hideContextMenu: () => void;
  showContextMenu: (
    x: number,
    y: number,
    pointIndex: number | null,
    fieldX: number,
    fieldY: number
  ) => void;
  handleWheel: (e: WheelEvent, cw: number, ch: number) => void;
  handleMiddleMouseDrag: (dx: number, dy: number, cw: number) => void;
}

export function useCanvasInteraction({
  canvasRef,
  containerRef,
  cw,
  ch,
  transform,
  splinePath,
  inactivePaths,
  menu,
  hideContextMenu,
  showContextMenu,
  handleWheel,
  handleMiddleMouseDrag,
}: UseCanvasInteractionArgs) {
  // Store selectors
  const controlPoints = usePathStore((s) => s.controlPoints);
  const rotationZones = usePathStore((s) => s.rotationZones);
  const addPoint = usePathStore((s) => s.addPoint);
  const movePoint = usePathStore((s) => s.movePoint);
  const moveRotationZoneHandle = usePathStore((s) => s.moveRotationZoneHandle);
  const pushUndoSnapshot = usePathStore((s) => s.pushUndoSnapshot);
  const setActivePath = usePathStore((s) => s.setActivePath);
  const selectPoint = useSelectionStore((s) => s.selectPoint);
  const selectZone = useSelectionStore((s) => s.selectZone);
  const snapEnabled = useEditorStore((s) => s.snapToGrid);
  const gridSize = useEditorStore((s) => s.gridSize);
  const setHoveredPointIndex = useEditorStore((s) => s.setHoveredPointIndex);

  // Drag state
  const [dragging, setDragging] = useState<number | null>(null);
  const [zoneDrag, setZoneDrag] = useState<{
    zoneId: string;
    handle: 'start' | 'end' | 'target';
  } | null>(null);
  const [middleDragging, setMiddleDragging] = useState(false);
  const lastMiddlePos = useRef({ x: 0, y: 0 });

  // Tooltip
  const { tooltip, show: showTooltip, hide: hideTooltip } = useTooltip();

  // Convert mouse event to canvas coordinates
  const getCanvasPos = useCallback(
    (e: React.MouseEvent): { cx: number; cy: number } => {
      const canvas = canvasRef.current;
      if (!canvas) return { cx: 0, cy: 0 };
      const rect = canvas.getBoundingClientRect();
      return {
        cx: ((e.clientX - rect.left) / rect.width) * cw,
        cy: ((e.clientY - rect.top) / rect.height) * ch,
      };
    },
    [canvasRef, cw, ch]
  );

  // Apply snap-to-grid if enabled
  const maybeSnap = useCallback(
    (pt: { x: number; y: number }): { x: number; y: number } => {
      if (snapEnabled) return snapToGrid(pt, gridSize);
      return pt;
    },
    [snapEnabled, gridSize]
  );

  // ─── Mouse Handlers ─────────────────────────────────────────────────────

  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      if (menu.visible) {
        hideContextMenu();
        return;
      }

      // Middle mouse button - start panning
      if (e.button === 1) {
        e.preventDefault();
        setMiddleDragging(true);
        lastMiddlePos.current = { x: e.clientX, y: e.clientY };
        return;
      }

      if (e.button === 2) return;

      const { cx, cy } = getCanvasPos(e);

      // Check zone handles/targets first
      const zoneHit = hitTestRotationZone(
        cx,
        cy,
        rotationZones,
        splinePath,
        controlPoints.length,
        cw,
        ch,
        transform
      );
      if (zoneHit) {
        pushUndoSnapshot();
        setZoneDrag(zoneHit);
        selectZone(zoneHit.zoneId);
        selectPoint(null);
        return;
      }

      const hit = hitTestControlPoint(cx, cy, controlPoints, cw, ch, transform);

      if (hit !== null) {
        setDragging(hit);
        selectPoint(hit);
        selectZone(null);
      } else {
        // Check if clicking near an inactive path to switch to it
        const fieldPt = canvasToField(cx, cy, cw, ch, transform);
        let switched = false;
        for (const ip of inactivePaths) {
          if (ip.spline) {
            const s = ip.spline.getClosestPointS(fieldPt);
            const closest = ip.spline.getPoint(s);
            const dist = Math.sqrt(
              (fieldPt.x - closest.x) ** 2 + (fieldPt.y - closest.y) ** 2
            );
            if (dist < 0.3) {
              setActivePath(ip.name);
              switched = true;
              break;
            }
          }
        }
        if (!switched) {
          let pt = canvasToField(cx, cy, cw, ch, transform);
          pt = clampToField(pt);
          pt = maybeSnap(pt);
          addPoint(pt);
          selectPoint(controlPoints.length);
          selectZone(null);
        }
      }
    },
    [
      menu.visible,
      hideContextMenu,
      getCanvasPos,
      rotationZones,
      splinePath,
      controlPoints,
      cw,
      ch,
      transform,
      pushUndoSnapshot,
      selectZone,
      selectPoint,
      addPoint,
      maybeSnap,
      inactivePaths,
      setActivePath,
    ]
  );

  const handleMouseMove = useCallback(
    (e: React.MouseEvent) => {
      // Middle-drag panning
      if (middleDragging) {
        const dx = e.clientX - lastMiddlePos.current.x;
        const dy = e.clientY - lastMiddlePos.current.y;
        lastMiddlePos.current = { x: e.clientX, y: e.clientY };
        const canvas = canvasRef.current;
        if (canvas) {
          const rect = canvas.getBoundingClientRect();
          const ratio = cw / rect.width;
          handleMiddleMouseDrag(dx * ratio, dy * ratio, cw);
        }
        return;
      }

      const { cx, cy } = getCanvasPos(e);

      // Dragging a zone handle/target
      if (zoneDrag !== null && splinePath) {
        let fieldPt = canvasToField(cx, cy, cw, ch, transform);
        fieldPt = clampToField(fieldPt);
        const numCPs = controlPoints.length;

        if (zoneDrag.handle === 'target') {
          moveRotationZoneHandle(zoneDrag.zoneId, { targetPoint: fieldPt });
        } else {
          const s = splinePath.getClosestPointS(fieldPt);
          const waypointIndex = Math.max(
            0,
            Math.min(numCPs - 1, (s / splinePath.totalLength) * (numCPs - 1))
          );
          if (zoneDrag.handle === 'start') {
            moveRotationZoneHandle(zoneDrag.zoneId, {
              startWaypointIndex: waypointIndex,
            });
          } else {
            moveRotationZoneHandle(zoneDrag.zoneId, {
              endWaypointIndex: waypointIndex,
            });
          }
        }
        return;
      }

      // Dragging a point
      if (dragging !== null) {
        let fieldPt = canvasToField(cx, cy, cw, ch, transform);
        fieldPt = clampToField(fieldPt);
        fieldPt = maybeSnap(fieldPt);
        movePoint(dragging, fieldPt);
        const canvas = canvasRef.current;
        if (canvas) {
          const rect = canvas.getBoundingClientRect();
          showTooltip(
            e.clientX - rect.left + 16,
            e.clientY - rect.top - 28,
            fieldPt.x,
            fieldPt.y
          );
        }
        return;
      }

      // Hover detection
      const hit = hitTestControlPoint(cx, cy, controlPoints, cw, ch, transform);
      setHoveredPointIndex(hit);

      // Coordinate tooltip
      let fieldPt = canvasToField(cx, cy, cw, ch, transform);
      fieldPt = clampToField(fieldPt);
      const snapped = maybeSnap(fieldPt);
      const canvas = canvasRef.current;
      if (canvas) {
        const rect = canvas.getBoundingClientRect();
        showTooltip(
          e.clientX - rect.left + 16,
          e.clientY - rect.top - 28,
          snapped.x,
          snapped.y
        );
      }
    },
    [
      middleDragging,
      handleMiddleMouseDrag,
      canvasRef,
      cw,
      ch,
      getCanvasPos,
      dragging,
      zoneDrag,
      splinePath,
      controlPoints,
      moveRotationZoneHandle,
      transform,
      maybeSnap,
      movePoint,
      setHoveredPointIndex,
      showTooltip,
    ]
  );

  const handleMouseUp = useCallback((e: React.MouseEvent) => {
    if (e.button === 1) {
      setMiddleDragging(false);
      return;
    }
    setDragging(null);
    setZoneDrag(null);
  }, []);

  const handleMouseLeave = useCallback(() => {
    setDragging(null);
    setZoneDrag(null);
    setMiddleDragging(false);
    hideTooltip();
    setHoveredPointIndex(null);
  }, [setHoveredPointIndex, hideTooltip]);

  const handleWheelEvent = useCallback(
    (e: React.WheelEvent) => {
      handleWheel(e.nativeEvent, cw, ch);
    },
    [handleWheel, cw, ch]
  );

  const handleContextMenu = useCallback(
    (e: React.MouseEvent) => {
      e.preventDefault();
      const { cx, cy } = getCanvasPos(e);
      const hit = hitTestControlPoint(cx, cy, controlPoints, cw, ch, transform);
      let fieldPt = canvasToField(cx, cy, cw, ch, transform);
      fieldPt = clampToField(fieldPt);
      fieldPt = maybeSnap(fieldPt);

      const container = containerRef.current;
      if (container) {
        const rect = container.getBoundingClientRect();
        showContextMenu(
          e.clientX - rect.left,
          e.clientY - rect.top,
          hit,
          fieldPt.x,
          fieldPt.y
        );
      }
    },
    [
      getCanvasPos,
      controlPoints,
      cw,
      ch,
      transform,
      maybeSnap,
      showContextMenu,
      containerRef,
    ]
  );

  // Cursor style
  let cursor = 'crosshair';
  if (dragging !== null || zoneDrag !== null) cursor = 'grabbing';
  else if (middleDragging) cursor = 'grabbing';

  return {
    tooltip,
    cursor,
    dragging,
    handleMouseDown,
    handleMouseMove,
    handleMouseUp,
    handleMouseLeave,
    handleWheelEvent,
    handleContextMenu,
  } as const;
}
