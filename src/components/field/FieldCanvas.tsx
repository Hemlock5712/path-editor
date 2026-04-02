import { useRef, useEffect, useCallback, useState } from 'react';
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
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const selectedPointIndex = usePathStore((s) => s.selectedPointIndex);
  const addPoint = usePathStore((s) => s.addPoint);
  const movePoint = usePathStore((s) => s.movePoint);
  const selectPoint = usePathStore((s) => s.selectPoint);

  const showGrid = useEditorStore((s) => s.showGrid);
  const showMinimap = useEditorStore((s) => s.showMinimap);
  const snapEnabled = useEditorStore((s) => s.snapToGrid);
  const gridSize = useEditorStore((s) => s.gridSize);
  const hoveredPointIndex = useEditorStore((s) => s.hoveredPointIndex);
  const setHoveredPointIndex = useEditorStore((s) => s.setHoveredPointIndex);
  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);
  const playbackState = useEditorStore((s) => s.playbackState);
  const showWaypointGhosts = useEditorStore((s) => s.showWaypointGhosts);

  // Robot dimensions from settings
  const robotLength = useSettingsStore((s) => s.robotLength);
  const robotWidth = useSettingsStore((s) => s.robotWidth);

  // Zoom/pan
  const { transform, handleWheel, handleMiddleMouseDrag, resetView } = useCanvasTransform();

  // Context menu
  const { menu, show: showContextMenu, hide: hideContextMenu } = useContextMenu();

  // Drag state
  const [dragging, setDragging] = useState<number | null>(null);
  const [middleDragging, setMiddleDragging] = useState(false);
  const lastMiddlePos = useRef({ x: 0, y: 0 });

  // Ghost preview point (where click would add a new point)
  const [ghostPoint, setGhostPoint] = useState<{ x: number; y: number } | null>(null);

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

    // Layer 0: Field background
    if (fieldImageRef.current) {
      // Compute where the full field maps to on the canvas with the current transform
      const topLeft = fieldToCanvas({ x: 0, y: FIELD_HEIGHT }, cw, ch, transform);
      const bottomRight = fieldToCanvas({ x: FIELD_WIDTH, y: 0 }, cw, ch, transform);
      const drawW = bottomRight.cx - topLeft.cx;
      const drawH = bottomRight.cy - topLeft.cy;

      ctx.drawImage(
        fieldImageRef.current,
        IMG_TOP_LEFT_X,
        IMG_TOP_LEFT_Y,
        IMG_FIELD_W,
        IMG_FIELD_H,
        topLeft.cx,
        topLeft.cy,
        drawW,
        drawH,
      );
    } else {
      ctx.fillStyle = '#2a2a2a';
      ctx.fillRect(0, 0, cw, ch);
    }

    // Layer 1: Grid
    Painters.drawGrid(ctx, cw, ch, transform, showGrid);

    // Layer 2: Connection lines
    Painters.drawConnectionLines(ctx, cw, ch, transform, controlPoints);

    // Layer 3: Spline path
    if (splinePath) {
      Painters.drawPath(ctx, cw, ch, transform, splinePath);
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

    // Layer 6: Ghost preview point — neon green glow
    if (ghostPoint && dragging === null) {
      const { cx: gx, cy: gy } = fieldToCanvas(ghostPoint, cw, ch, transform);
      ctx.save();
      ctx.shadowColor = '#00FFaa';
      ctx.shadowBlur = 8;
      ctx.beginPath();
      ctx.arc(gx, gy, 5, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(0, 255, 170, 0.2)';
      ctx.fill();
      ctx.strokeStyle = 'rgba(0, 255, 170, 0.5)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.restore();
    }

    // Layer 7: Control points (drawn last so they're on top)
    Painters.drawControlPoints(
      ctx,
      cw,
      ch,
      transform,
      controlPoints,
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
    headingWaypoints,
    selectedPointIndex,
    hoveredPointIndex,
    splinePath,
    scrubberDistance,
    scrubberHeading,
    playbackState,
    ghostPoint,
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
      const hit = hitTest(cx, cy);

      if (hit !== null) {
        setDragging(hit);
        selectPoint(hit);
      } else {
        let fieldPt = canvasToField(cx, cy, cw, ch, transform);
        fieldPt = clampToField(fieldPt);
        fieldPt = maybeSnap(fieldPt);
        addPoint(fieldPt);
        selectPoint(controlPoints.length); // Select newly added point
      }
    },
    [
      menu.visible,
      hideContextMenu,
      getCanvasPos,
      hitTest,
      selectPoint,
      cw,
      ch,
      transform,
      maybeSnap,
      addPoint,
      controlPoints.length,
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

      // Dragging a point
      if (dragging !== null) {
        let fieldPt = canvasToField(cx, cy, cw, ch, transform);
        fieldPt = clampToField(fieldPt);
        fieldPt = maybeSnap(fieldPt);
        movePoint(dragging, fieldPt);
        setGhostPoint(null);
        return;
      }

      // Hover detection
      const hit = hitTest(cx, cy);
      setHoveredPointIndex(hit);

      // Update ghost preview and tooltip
      let fieldPt = canvasToField(cx, cy, cw, ch, transform);
      fieldPt = clampToField(fieldPt);
      const snapped = maybeSnap(fieldPt);

      if (hit === null) {
        setGhostPoint(snapped);
      } else {
        setGhostPoint(null);
      }

      // Coordinate tooltip
      const canvas = canvasRef.current;
      if (canvas) {
        const rect = canvas.getBoundingClientRect();
        setTooltip({
          visible: true,
          x: e.clientX - rect.left + 16,
          y: e.clientY - rect.top - 8,
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
    },
    [],
  );

  const handleMouseLeave = useCallback(() => {
    setDragging(null);
    setMiddleDragging(false);
    setGhostPoint(null);
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
  if (dragging !== null) cursor = 'grabbing';
  else if (middleDragging) cursor = 'grabbing';
  else if (hoveredPointIndex !== null) cursor = 'grab';

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

      {/* Context menu */}
      {menu.visible && <FieldContextMenu menu={menu} onClose={hideContextMenu} />}
    </div>
  );
}
