import { useRef, useEffect, useCallback, useState, useMemo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useEditorStore } from '../../stores/editorStore';
import { useCanvasTransform } from '../../hooks/useCanvasTransform';
import { useContextMenu, type ContextMenuState } from '../../hooks/useContextMenu';
import {
  canvasToField,
  fieldToCanvas,
  getScale,
  clampToField,
  type CanvasTransform,
} from '../../utils/canvasTransform';
import { snapToGrid } from '../../utils/snapping';
import { FIELD_WIDTH, FIELD_HEIGHT } from '../../types';
import { Maximize } from 'lucide-react';
import { useSettingsStore } from '../../stores/settingsStore';
import { SplinePath } from '../../math/SplinePath';
import * as Painters from './FieldPainters';
import { FieldContextMenu } from './FieldContextMenu';

// AdvantageScope field image crop coordinates
const IMG_TOP_LEFT_X = 524;
const IMG_TOP_LEFT_Y = 95;
const IMG_FIELD_W = 3378 - 524;
const IMG_FIELD_H = 1489 - 95;

interface FieldCanvasProps {
  splinePath: SplinePath | null;
  scrubberHeading: number | null;
}

export function FieldCanvas({ splinePath, scrubberHeading }: FieldCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const fieldImageRef = useRef<HTMLImageElement | null>(null);
  const [imageLoaded, setImageLoaded] = useState(false);

  // Canvas dimensions tracking
  const [canvasDims, setCanvasDims] = useState({ width: 800, height: 400 });

  // Stores
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const namedPoints = usePathStore((s) => s.namedPoints);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const constraintZones = usePathStore((s) => s.constraintZones);
  const rotationZones = usePathStore((s) => s.rotationZones);
  const selectedPointIndex = usePathStore((s) => s.selectedPointIndex);
  const selectedZoneId = usePathStore((s) => s.selectedZoneId);
  const paths = usePathStore((s) => s.paths);
  const activePathName = usePathStore((s) => s.activePathName);
  const setActivePath = usePathStore((s) => s.setActivePath);
  const addPoint = usePathStore((s) => s.addPoint);
  const movePoint = usePathStore((s) => s.movePoint);
  const selectPoint = usePathStore((s) => s.selectPoint);
  const selectZone = usePathStore((s) => s.selectZone);
  const updateRotationZone = usePathStore((s) => s.updateRotationZone);

  // Compute splines for inactive paths
  const inactivePaths = useMemo(() => {
    return Object.entries(paths)
      .filter(([name]) => name !== activePathName)
      .map(([name, path]) => ({
        name,
        controlPoints: path.controlPoints,
        spline: path.controlPoints.length >= 2 ? new SplinePath(path.controlPoints) : null,
      }));
  }, [paths, activePathName]);

  const showGrid = useEditorStore((s) => s.showGrid);
  const showMinimap = useEditorStore((s) => s.showMinimap);
  const snapEnabled = useEditorStore((s) => s.snapToGrid);
  const gridSize = useEditorStore((s) => s.gridSize);
  const hoveredPointIndex = useEditorStore((s) => s.hoveredPointIndex);
  const setHoveredPointIndex = useEditorStore((s) => s.setHoveredPointIndex);
  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);
  const playbackState = useEditorStore((s) => s.playbackState);
  const showWaypointGhosts = useEditorStore((s) => s.showWaypointGhosts);
  const zoom = useEditorStore((s) => s.zoom);
  const panOffset = useEditorStore((s) => s.panOffset);

  // Robot dimensions from settings
  const robotLength = useSettingsStore((s) => s.robotLength);
  const robotWidth = useSettingsStore((s) => s.robotWidth);

  // Zoom/pan
  const { transform, handleWheel, handleMiddleMouseDrag, resetView } = useCanvasTransform();

  // Context menu
  const { menu, show: showContextMenu, hide: hideContextMenu } = useContextMenu();

  // Drag state
  const [dragging, setDragging] = useState<number | null>(null);
  const [zoneDrag, setZoneDrag] = useState<{ zoneId: string; handle: 'start' | 'end' | 'target' } | null>(null);
  const [middleDragging, setMiddleDragging] = useState(false);
  const lastMiddlePos = useRef({ x: 0, y: 0 });

  // Coordinate tooltip
  const [tooltip, setTooltip] = useState<{
    visible: boolean;
    x: number;
    y: number;
    fieldX: number;
    fieldY: number;
  }>({ visible: false, x: 0, y: 0, fieldX: 0, fieldY: 0 });

  // Load field image on mount
  useEffect(() => {
    const img = new Image();
    img.src = '/field-2026.png';
    img.onload = () => {
      fieldImageRef.current = img;
      setImageLoaded(true);
    };
    img.onerror = () => {
      fieldImageRef.current = null;
      setImageLoaded(true);
    };
  }, []);

  // ResizeObserver to fill container, maintaining aspect ratio
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const observer = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width: containerW, height: containerH } = entry.contentRect;
        if (containerW <= 0 || containerH <= 0) continue;

        const fieldAspect = FIELD_WIDTH / FIELD_HEIGHT;
        let drawW = containerW;
        let drawH = containerW / fieldAspect;
        if (drawH > containerH) {
          drawH = containerH;
          drawW = containerH * fieldAspect;
        }

        // Use devicePixelRatio for crisp rendering
        const dpr = window.devicePixelRatio || 1;
        setCanvasDims({
          width: Math.round(drawW * dpr),
          height: Math.round(drawH * dpr),
        });
      }
    });

    observer.observe(container);
    return () => observer.disconnect();
  }, []);

  // Compute internal canvas resolution
  const cw = canvasDims.width;
  const ch = canvasDims.height;

  // Hit test: find which control point is at canvas position
  const hitTest = useCallback(
    (canvasX: number, canvasY: number): number | null => {
      const scale = getScale(cw, transform);
      const hitRadius = Math.max(10, Math.min(18, 14 / Math.sqrt(transform.zoom)));

      for (let i = controlPoints.length - 1; i >= 0; i--) {
        const c = fieldToCanvas(controlPoints[i], cw, ch, transform);
        const dx = canvasX - c.cx;
        const dy = canvasY - c.cy;
        if (dx * dx + dy * dy <= hitRadius * hitRadius) {
          return i;
        }
      }
      return null;
    },
    [controlPoints, cw, ch, transform],
  );

  // Hit test rotation zone handles/targets
  const hitTestZone = useCallback(
    (canvasX: number, canvasY: number): { zoneId: string; handle: 'start' | 'end' | 'target' } | null => {
      if (!splinePath || splinePath.totalLength <= 0) return null;
      const hitRadius = Math.max(12, Math.min(20, 16 / Math.sqrt(transform.zoom)));
      const numCPs = controlPoints.length;

      for (const zone of rotationZones) {
        // Check target point first (highest priority)
        const tc = fieldToCanvas(zone.targetPoint, cw, ch, transform);
        if ((canvasX - tc.cx) ** 2 + (canvasY - tc.cy) ** 2 <= hitRadius * hitRadius) {
          return { zoneId: zone.id, handle: 'target' };
        }

        // Check boundary handles
        for (const handle of ['start', 'end'] as const) {
          const idx = handle === 'start' ? zone.startWaypointIndex : zone.endWaypointIndex;
          const frac = idx / (numCPs - 1);
          const s = frac * splinePath.totalLength;
          const pt = splinePath.getPoint(s);
          const pc = fieldToCanvas(pt, cw, ch, transform);
          if ((canvasX - pc.cx) ** 2 + (canvasY - pc.cy) ** 2 <= hitRadius * hitRadius) {
            return { zoneId: zone.id, handle };
          }
        }
      }
      return null;
    },
    [rotationZones, splinePath, controlPoints.length, cw, ch, transform],
  );

  // Convert mouse event to canvas coordinates (accounting for CSS vs canvas resolution)
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
    [cw, ch],
  );

  // Apply snap-to-grid if enabled
  const maybeSnap = useCallback(
    (pt: { x: number; y: number }): { x: number; y: number } => {
      if (snapEnabled) return snapToGrid(pt, gridSize);
      return pt;
    },
    [snapEnabled, gridSize],
  );

  // ─── Drawing ────────────────────────────────────────────────────────────

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, cw, ch);

    // Fill canvas background so padding area matches the dark theme
    ctx.fillStyle = '#0a0a0f';
    ctx.fillRect(0, 0, cw, ch);

    // Layer 0: Field background (rotated 180° to match correct orientation)
    if (fieldImageRef.current) {
      // Compute where the full field maps to on the canvas with the current transform
      const topLeft = fieldToCanvas({ x: 0, y: FIELD_HEIGHT }, cw, ch, transform);
      const bottomRight = fieldToCanvas({ x: FIELD_WIDTH, y: 0 }, cw, ch, transform);
      const drawW = bottomRight.cx - topLeft.cx;
      const drawH = bottomRight.cy - topLeft.cy;

      const centerX = topLeft.cx + drawW / 2;
      const centerY = topLeft.cy + drawH / 2;
      ctx.save();
      ctx.translate(centerX, centerY);
      ctx.rotate(Math.PI);
      ctx.drawImage(
        fieldImageRef.current,
        IMG_TOP_LEFT_X,
        IMG_TOP_LEFT_Y,
        IMG_FIELD_W,
        IMG_FIELD_H,
        -drawW / 2,
        -drawH / 2,
        drawW,
        drawH,
      );
      ctx.restore();
    } else {
      ctx.fillStyle = '#2a2a2a';
      ctx.fillRect(0, 0, cw, ch);
    }

    // Layer 1: Grid
    Painters.drawGrid(ctx, cw, ch, transform, showGrid);

    // Layer 1.5: Named point markers
    Painters.drawNamedPoints(ctx, cw, ch, transform, namedPoints, controlPointRefs);

    // Layer 1.6: Inactive paths
    inactivePaths.forEach((ip, idx) => {
      if (ip.spline) {
        Painters.drawInactivePath(ctx, cw, ch, transform, ip.spline, ip.controlPoints, ip.name, idx);
      }
    });

    // Layer 2: Connection lines
    Painters.drawConnectionLines(ctx, cw, ch, transform, controlPoints);

    // Layer 2.5: Constraint zones (behind the path line)
    if (splinePath && constraintZones.length > 0) {
      Painters.drawConstraintZones(ctx, cw, ch, transform, splinePath, constraintZones);
    }

    // Layer 3: Spline path
    if (splinePath) {
      Painters.drawPath(ctx, cw, ch, transform, splinePath);
    }

    // Layer 3.5: Rotation zones
    if (splinePath && rotationZones.length > 0) {
      Painters.drawRotationZones(ctx, cw, ch, transform, splinePath, rotationZones, selectedZoneId);
    }

    // Layer 4: Scrubber ghost (visible whenever scrubber is active, not just during playback)
    if (splinePath && scrubberDistance > 0) {
      Painters.drawScrubberGhost(
        ctx,
        cw,
        ch,
        transform,
        splinePath,
        scrubberDistance,
        scrubberHeading,
        robotLength,
        robotWidth,
      );
    }

    // Layer 4.5: Waypoint robot ghosts
    if (splinePath && showWaypointGhosts && controlPoints.length >= 2) {
      Painters.drawWaypointGhosts(
        ctx, cw, ch, transform, splinePath, controlPoints, headingWaypoints,
        robotLength, robotWidth,
      );
    }

    // Layer 5: Heading arrows
    Painters.drawHeadingArrows(ctx, cw, ch, transform, controlPoints, headingWaypoints);

    // Layer 6: Control points (drawn last so they're on top)
    Painters.drawControlPoints(
      ctx,
      cw,
      ch,
      transform,
      controlPoints,
      controlPointRefs,
      selectedPointIndex,
      hoveredPointIndex,
    );

    // Layer 8: Minimap
    if (showMinimap) {
      Painters.drawMinimap(ctx, cw, ch, transform, splinePath, controlPoints);
    }
  }, [
    cw,
    ch,
    transform,
    showGrid,
    showMinimap,
    showWaypointGhosts,
    controlPoints,
    controlPointRefs,
    namedPoints,
    headingWaypoints,
    constraintZones,
    rotationZones,
    selectedZoneId,
    selectedPointIndex,
    hoveredPointIndex,
    splinePath,
    scrubberDistance,
    scrubberHeading,
    playbackState,
    inactivePaths,
    dragging,
    imageLoaded,
    robotLength,
    robotWidth,
  ]);

  // Redraw on state changes
  useEffect(() => {
    draw();
  }, [draw]);

  // ─── Mouse Handlers ─────────────────────────────────────────────────────

  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      // Close context menu on any click
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

      // Right click - handled by context menu
      if (e.button === 2) return;

      // Left click
      const { cx, cy } = getCanvasPos(e);

      // Check zone handles/targets first
      const zoneHit = hitTestZone(cx, cy);
      if (zoneHit) {
        setZoneDrag(zoneHit);
        selectZone(zoneHit.zoneId);
        selectPoint(null);
        return;
      }

      const hit = hitTest(cx, cy);

      if (hit !== null) {
        setDragging(hit);
        selectPoint(hit);
        selectZone(null);
      } else {
        // Check if clicking near an inactive path to switch to it
        let fieldPt = canvasToField(cx, cy, cw, ch, transform);
        let switched = false;
        for (const ip of inactivePaths) {
          if (ip.spline) {
            const s = ip.spline.getClosestPointS(fieldPt);
            const closest = ip.spline.getPoint(s);
            const dist = Math.sqrt((fieldPt.x - closest.x) ** 2 + (fieldPt.y - closest.y) ** 2);
            if (dist < 0.3) {
              setActivePath(ip.name);
              switched = true;
              break;
            }
          }
        }
        if (!switched) {
          let fieldPt = canvasToField(cx, cy, cw, ch, transform);
          fieldPt = clampToField(fieldPt);
          fieldPt = maybeSnap(fieldPt);
          addPoint(fieldPt);
          selectPoint(controlPoints.length);
          selectZone(null);
        }
      }
    },
    [
      menu.visible,
      hideContextMenu,
      getCanvasPos,
      hitTest,
      hitTestZone,
      addPoint,
      controlPoints.length,
      selectPoint,
      selectZone,
      maybeSnap,
      cw,
      ch,
      transform,
      inactivePaths,
      setActivePath,
    ],
  );

  const handleMouseMove = useCallback(
    (e: React.MouseEvent) => {
      // Middle-drag panning
      if (middleDragging) {
        const dx = e.clientX - lastMiddlePos.current.x;
        const dy = e.clientY - lastMiddlePos.current.y;
        lastMiddlePos.current = { x: e.clientX, y: e.clientY };
        // Convert CSS pixels to canvas pixels using the display ratio
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
          updateRotationZone(zoneDrag.zoneId, { targetPoint: fieldPt });
        } else {
          // Snap to path and convert to waypoint index
          const s = splinePath.getClosestPointS(fieldPt);
          const waypointIndex = Math.max(0, Math.min(numCPs - 1, (s / splinePath.totalLength) * (numCPs - 1)));
          if (zoneDrag.handle === 'start') {
            updateRotationZone(zoneDrag.zoneId, { startWaypointIndex: waypointIndex });
          } else {
            updateRotationZone(zoneDrag.zoneId, { endWaypointIndex: waypointIndex });
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
          setTooltip({
            visible: true,
            x: e.clientX - rect.left + 16,
            y: e.clientY - rect.top - 28,
            fieldX: fieldPt.x,
            fieldY: fieldPt.y,
          });
        }
        return;
      }

      // Hover detection
      const hit = hitTest(cx, cy);
      setHoveredPointIndex(hit);

      // Coordinate tooltip
      let fieldPt = canvasToField(cx, cy, cw, ch, transform);
      fieldPt = clampToField(fieldPt);
      const snapped = maybeSnap(fieldPt);
      const canvas = canvasRef.current;
      if (canvas) {
        const rect = canvas.getBoundingClientRect();
        setTooltip({
          visible: true,
          x: e.clientX - rect.left + 16,
          y: e.clientY - rect.top - 28,
          fieldX: snapped.x,
          fieldY: snapped.y,
        });
      }
    },
    [
      middleDragging,
      handleMiddleMouseDrag,
      cw,
      ch,
      getCanvasPos,
      dragging,
      zoneDrag,
      splinePath,
      controlPoints.length,
      updateRotationZone,
      transform,
      maybeSnap,
      movePoint,
      hitTest,
      setHoveredPointIndex,
    ],
  );

  const handleMouseUp = useCallback(
    (e: React.MouseEvent) => {
      if (e.button === 1) {
        setMiddleDragging(false);
        return;
      }
      setDragging(null);
      setZoneDrag(null);
    },
    [],
  );

  const handleMouseLeave = useCallback(() => {
    setDragging(null);
    setZoneDrag(null);
    setMiddleDragging(false);
    setTooltip((prev) => ({ ...prev, visible: false }));
    setHoveredPointIndex(null);
  }, [setHoveredPointIndex]);

  // Wheel zoom
  const handleWheelEvent = useCallback(
    (e: React.WheelEvent) => {
      handleWheel(e.nativeEvent, cw, ch);
    },
    [handleWheel, cw, ch],
  );

  // Context menu
  const handleContextMenu = useCallback(
    (e: React.MouseEvent) => {
      e.preventDefault();
      const { cx, cy } = getCanvasPos(e);
      const hit = hitTest(cx, cy);
      let fieldPt = canvasToField(cx, cy, cw, ch, transform);
      fieldPt = clampToField(fieldPt);
      fieldPt = maybeSnap(fieldPt);

      // Position relative to the container
      const container = containerRef.current;
      if (container) {
        const rect = container.getBoundingClientRect();
        showContextMenu(
          e.clientX - rect.left,
          e.clientY - rect.top,
          hit,
          fieldPt.x,
          fieldPt.y,
        );
      }
    },
    [getCanvasPos, hitTest, cw, ch, transform, maybeSnap, showContextMenu],
  );

  // Prevent default middle-click scroll behavior
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const preventMiddleScroll = (e: MouseEvent) => {
      if (e.button === 1) e.preventDefault();
    };
    canvas.addEventListener('mousedown', preventMiddleScroll);
    return () => canvas.removeEventListener('mousedown', preventMiddleScroll);
  }, []);

  // Compute CSS canvas size from internal resolution
  const dpr = typeof window !== 'undefined' ? window.devicePixelRatio || 1 : 1;
  const cssWidth = cw / dpr;
  const cssHeight = ch / dpr;

  // Cursor style
  let cursor = 'crosshair';
  if (dragging !== null || zoneDrag !== null) cursor = 'grabbing';
  else if (middleDragging) cursor = 'grabbing';
  else if (hoveredPointIndex !== null) cursor = 'grab';

  const viewMoved = zoom !== 1.0 || panOffset.x !== 0 || panOffset.y !== 0;

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full overflow-hidden flex items-center justify-center"
      style={{ minHeight: '200px' }}
    >
      <canvas
        ref={canvasRef}
        width={cw}
        height={ch}
        style={{
          width: `${cssWidth}px`,
          height: `${cssHeight}px`,
          cursor,
          display: 'block',
        }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseLeave}
        onWheel={handleWheelEvent}
        onContextMenu={handleContextMenu}
      />

      {/* Coordinate tooltip — neon styled */}
      {tooltip.visible && (
        <div
          className="pointer-events-none absolute px-2 py-1 rounded text-xs font-mono"
          style={{
            left: tooltip.x,
            top: tooltip.y,
            background: 'rgba(5, 5, 5, 0.9)',
            color: '#00FFaa',
            border: '1px solid rgba(0, 255, 170, 0.15)',
            boxShadow: '0 0 12px rgba(0, 255, 170, 0.08)',
            whiteSpace: 'nowrap',
            zIndex: 10,
          }}
        >
          ({tooltip.fieldX.toFixed(2)}, {tooltip.fieldY.toFixed(2)})
        </div>
      )}

      {/* Reset view button — only visible when zoomed/panned */}
      {viewMoved && (
        <button
          onClick={resetView}
          className="absolute top-2 right-2 flex items-center gap-1.5 px-2 py-1 rounded text-xs font-mono transition-opacity"
          style={{
            background: 'rgba(5, 5, 5, 0.85)',
            color: '#00FFaa',
            border: '1px solid rgba(0, 255, 170, 0.2)',
            boxShadow: '0 0 12px rgba(0, 255, 170, 0.08)',
            zIndex: 10,
          }}
          title="Reset view (0)"
        >
          <Maximize size={12} />
          Reset View
        </button>
      )}

      {/* Context menu */}
      {menu.visible && <FieldContextMenu menu={menu} onClose={hideContextMenu} />}
    </div>
  );
}
