import { useRef, useEffect, useState, useMemo, useCallback } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';
import { useEditorStore } from '../../stores/editorStore';
import { useCanvasTransform } from '../../hooks/useCanvasTransform';
import { useContextMenu } from '../../hooks/useContextMenu';
import { useCanvasInteraction } from '../../hooks/useCanvasInteraction';
import { FIELD_WIDTH, FIELD_HEIGHT } from '../../types';
import { Maximize } from 'lucide-react';
import { useSettingsStore } from '../../stores/settingsStore';
import { SplinePath } from '../../math/SplinePath';
import { renderFieldCanvas, type DrawContext } from './FieldRenderer';
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

  // Canvas dimensions tracking (dpr stored alongside to avoid mismatch)
  const [canvasDims, setCanvasDims] = useState({ width: 800, height: 400, dpr: typeof window !== 'undefined' ? window.devicePixelRatio || 1 : 1 });

  // Stores
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const namedPoints = usePathStore((s) => s.namedPoints);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const constraintZones = usePathStore((s) => s.constraintZones);
  const rotationZones = usePathStore((s) => s.rotationZones);
  const selectedPointIndex = useSelectionStore((s) => s.selectedPointIndex);
  const selectedZoneId = useSelectionStore((s) => s.selectedZoneId);
  const paths = usePathStore((s) => s.paths);
  const activePathName = usePathStore((s) => s.activePathName);

  const showGrid = useEditorStore((s) => s.showGrid);
  const showMinimap = useEditorStore((s) => s.showMinimap);
  const hoveredPointIndex = useEditorStore((s) => s.hoveredPointIndex);
  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);
  const showWaypointGhosts = useEditorStore((s) => s.showWaypointGhosts);
  const zoom = useEditorStore((s) => s.zoom);
  const panOffset = useEditorStore((s) => s.panOffset);

  const robotLength = useSettingsStore((s) => s.robotLength);
  const robotWidth = useSettingsStore((s) => s.robotWidth);

  // Compute splines for inactive paths
  const inactivePaths = useMemo(() => {
    return Object.entries(paths)
      .filter(([name]) => name !== activePathName)
      .map(([name, path]) => ({
        name,
        controlPoints: path.controlPoints,
        spline:
          path.controlPoints.length >= 2
            ? new SplinePath(path.controlPoints)
            : null,
      }));
  }, [paths, activePathName]);

  // Zoom/pan
  const { transform, handleWheel, handleMiddleMouseDrag, resetView } =
    useCanvasTransform();

  // Context menu
  const {
    menu,
    show: showContextMenu,
    hide: hideContextMenu,
  } = useContextMenu();

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

  // Track DPR changes (e.g. window moved between monitors)
  const [currentDpr, setCurrentDpr] = useState(typeof window !== 'undefined' ? window.devicePixelRatio || 1 : 1);
  useEffect(() => {
    const mql = window.matchMedia(`(resolution: ${currentDpr}dppx)`);
    const handler = () => setCurrentDpr(window.devicePixelRatio || 1);
    mql.addEventListener('change', handler);
    return () => mql.removeEventListener('change', handler);
  }, [currentDpr]);

  // ResizeObserver to fill container, maintaining aspect ratio
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const recalc = (containerW: number, containerH: number) => {
      if (containerW <= 0 || containerH <= 0) return;

      const fieldAspect = FIELD_WIDTH / FIELD_HEIGHT;
      let drawW = containerW;
      let drawH = containerW / fieldAspect;
      if (drawH > containerH) {
        drawH = containerH;
        drawW = containerH * fieldAspect;
      }

      const dpr = window.devicePixelRatio || 1;
      setCanvasDims({
        width: Math.round(drawW * dpr),
        height: Math.round(drawH * dpr),
        dpr,
      });
    };

    const observer = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width: containerW, height: containerH } = entry.contentRect;
        recalc(containerW, containerH);
      }
    });

    observer.observe(container);

    // Recalculate immediately when DPR changes
    const rect = container.getBoundingClientRect();
    recalc(rect.width, rect.height);

    return () => observer.disconnect();
  }, [currentDpr]);

  const cw = canvasDims.width;
  const ch = canvasDims.height;

  // Canvas interaction (mouse handlers, drag, tooltip)
  const {
    tooltip,
    cursor,
    handleMouseDown,
    handleMouseMove,
    handleMouseUp,
    handleMouseLeave,
    handleWheelEvent,
    handleContextMenu,
  } = useCanvasInteraction({
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
  });

  // Hover cursor adjustment (hoveredPointIndex lives in editorStore, read here for cursor)
  const effectiveCursor =
    cursor === 'crosshair' && hoveredPointIndex !== null ? 'grab' : cursor;

  // ─── Drawing ────────────────────────────────────────────────────────────

  const drawCtx: DrawContext = useMemo(
    () => ({
      cw,
      ch,
      transform,
      fieldImage: fieldImageRef.current,
      imgTopLeftX: IMG_TOP_LEFT_X,
      imgTopLeftY: IMG_TOP_LEFT_Y,
      imgFieldW: IMG_FIELD_W,
      imgFieldH: IMG_FIELD_H,
      controlPoints,
      controlPointRefs,
      namedPoints,
      headingWaypoints,
      constraintZones,
      rotationZones,
      splinePath,
      inactivePaths,
      selectedPointIndex,
      selectedZoneId,
      hoveredPointIndex,
      showGrid,
      showMinimap,
      showWaypointGhosts,
      scrubberDistance,
      scrubberHeading,
      robotLength,
      robotWidth,
    }),
    [
      cw,
      ch,
      transform,
      imageLoaded,
      controlPoints,
      controlPointRefs,
      namedPoints,
      headingWaypoints,
      constraintZones,
      rotationZones,
      splinePath,
      inactivePaths,
      selectedPointIndex,
      selectedZoneId,
      hoveredPointIndex,
      showGrid,
      showMinimap,
      showWaypointGhosts,
      scrubberDistance,
      scrubberHeading,
      robotLength,
      robotWidth,
    ]
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    renderFieldCanvas(ctx, drawCtx);
  }, [drawCtx]);

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

  // Keyboard handler for canvas accessibility
  const gridSize = useEditorStore((s) => s.gridSize);
  const movePoint = usePathStore((s) => s.movePoint);
  const deletePoint = usePathStore((s) => s.deletePoint);
  const selectPoint = useSelectionStore((s) => s.selectPoint);
  const clearSelection = useSelectionStore((s) => s.clearSelection);

  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      if (e.key === 'Escape') {
        clearSelection();
        return;
      }
      if (e.key === 'Tab') {
        e.preventDefault();
        if (controlPoints.length === 0) return;
        const dir = e.shiftKey ? -1 : 1;
        const next =
          selectedPointIndex === null
            ? 0
            : (selectedPointIndex + dir + controlPoints.length) %
              controlPoints.length;
        selectPoint(next);
        return;
      }
      if (
        selectedPointIndex !== null &&
        selectedPointIndex < controlPoints.length
      ) {
        const step = gridSize || 0.1;
        const pt = controlPoints[selectedPointIndex];
        if (e.key === 'ArrowLeft') {
          e.preventDefault();
          movePoint(selectedPointIndex, { x: pt.x - step, y: pt.y });
        } else if (e.key === 'ArrowRight') {
          e.preventDefault();
          movePoint(selectedPointIndex, { x: pt.x + step, y: pt.y });
        } else if (e.key === 'ArrowUp') {
          e.preventDefault();
          movePoint(selectedPointIndex, { x: pt.x, y: pt.y + step });
        } else if (e.key === 'ArrowDown') {
          e.preventDefault();
          movePoint(selectedPointIndex, { x: pt.x, y: pt.y - step });
        } else if (e.key === 'Delete' || e.key === 'Backspace') {
          if (controlPoints.length > 2) {
            deletePoint(selectedPointIndex);
          }
        }
      }
    },
    [
      controlPoints,
      selectedPointIndex,
      gridSize,
      movePoint,
      deletePoint,
      selectPoint,
      clearSelection,
    ]
  );

  const cssWidth = cw / canvasDims.dpr;
  const cssHeight = ch / canvasDims.dpr;

  const viewMoved = zoom !== 1.0 || panOffset.x !== 0 || panOffset.y !== 0;

  return (
    <div
      ref={containerRef}
      className="relative flex h-full w-full items-center justify-center overflow-hidden"
      style={{ minHeight: '200px' }}
    >
      <canvas
        ref={canvasRef}
        width={cw}
        height={ch}
        role="application"
        aria-label="Field editor canvas"
        tabIndex={0}
        style={{
          width: `${cssWidth}px`,
          height: `${cssHeight}px`,
          cursor: effectiveCursor,
          display: 'block',
        }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseLeave}
        onWheel={handleWheelEvent}
        onContextMenu={handleContextMenu}
        onKeyDown={handleKeyDown}
      />

      {tooltip.visible && (
        <div
          className="pointer-events-none absolute rounded px-2 py-1 font-mono text-xs"
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

      {viewMoved && (
        <button
          onClick={resetView}
          className="absolute top-2 right-2 flex items-center gap-1.5 rounded px-2 py-1 font-mono text-xs transition-opacity"
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

      {menu.visible && (
        <FieldContextMenu menu={menu} onClose={hideContextMenu} />
      )}
    </div>
  );
}
