import { useCallback } from 'react';
import { useEditorStore } from '../stores/editorStore';
import {
  zoomAtPoint,
  FIELD_PADDING,
  type CanvasTransform,
} from '../utils/canvasTransform';
import { FIELD_WIDTH } from '../types';

export function useCanvasTransform() {
  const zoom = useEditorStore((s) => s.zoom);
  const panOffset = useEditorStore((s) => s.panOffset);
  const setZoom = useEditorStore((s) => s.setZoom);
  const setPanOffset = useEditorStore((s) => s.setPanOffset);
  const resetView = useEditorStore((s) => s.resetView);

  const transform: CanvasTransform = {
    zoom,
    panX: panOffset.x,
    panY: panOffset.y,
  };

  const handleWheel = useCallback(
    (e: WheelEvent, canvasWidth: number, canvasHeight: number) => {
      e.preventDefault();
      const rect = (e.target as HTMLElement).getBoundingClientRect();
      const cx = ((e.clientX - rect.left) / rect.width) * canvasWidth;
      const cy = ((e.clientY - rect.top) / rect.height) * canvasHeight;

      const zoomDelta = -e.deltaY * 0.001;
      const newTransform = zoomAtPoint(
        transform,
        cx,
        cy,
        canvasWidth,
        canvasHeight,
        zoomDelta
      );

      setZoom(newTransform.zoom);
      setPanOffset({ x: newTransform.panX, y: newTransform.panY });
    },
    [transform, setZoom, setPanOffset]
  );

  const handleMiddleMouseDrag = useCallback(
    (dx: number, dy: number, canvasWidth: number) => {
      const pad = Math.min(FIELD_PADDING, canvasWidth * 0.05);
      const scale = ((canvasWidth - 2 * pad) / FIELD_WIDTH) * zoom;
      setPanOffset({
        x: panOffset.x - dx / scale,
        y: panOffset.y + dy / scale, // Y is inverted on canvas
      });
    },
    [zoom, panOffset, setPanOffset]
  );

  return { transform, handleWheel, handleMiddleMouseDrag, resetView };
}
