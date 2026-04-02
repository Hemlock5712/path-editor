import { useRef, useEffect, useCallback, useState } from 'react';
import { useEditorStore } from '../../stores/editorStore';
import { velocityToColor } from '../../utils/colors';
import type { AnalyticsArrays } from '../../math/ProfileAnalytics';
import type { VelocityProfile } from '../../math/VelocityProfile';
import type { TimeEstimator } from '../../math/TimeEstimator';

interface VelocityDistanceChartProps {
  analytics: AnalyticsArrays | null;
  profile: VelocityProfile | null;
  timeEstimator: TimeEstimator | null;
  maxVelocity: number;
}

const PADDING = { top: 20, right: 16, bottom: 32, left: 48 };

export function VelocityDistanceChart({
  analytics,
  profile,
  timeEstimator,
  maxVelocity,
}: VelocityDistanceChartProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [tooltip, setTooltip] = useState<{ x: number; y: number; text: string } | null>(null);

  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);
  const setScrubberDistance = useEditorStore((s) => s.setScrubberDistance);
  const playbackState = useEditorStore((s) => s.playbackState);

  // ResizeObserver
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const observer = new ResizeObserver((entries) => {
      const entry = entries[0];
      if (entry) {
        const { width, height } = entry.contentRect;
        setSize({ width: Math.floor(width), height: Math.floor(height) });
      }
    });
    observer.observe(container);
    return () => observer.disconnect();
  }, []);

  // Convert distance to canvas X
  const distToX = useCallback(
    (d: number): number => {
      if (!analytics || analytics.distances.length < 2) return PADDING.left;
      const maxDist = analytics.distances[analytics.distances.length - 1];
      if (maxDist <= 0) return PADDING.left;
      const plotW = size.width - PADDING.left - PADDING.right;
      return PADDING.left + (d / maxDist) * plotW;
    },
    [analytics, size.width],
  );

  // Convert velocity to canvas Y
  const velToY = useCallback(
    (v: number): number => {
      const plotH = size.height - PADDING.top - PADDING.bottom;
      const yMax = maxVelocity * 1.1;
      return PADDING.top + plotH * (1 - v / yMax);
    },
    [maxVelocity, size.height],
  );

  // Convert canvas X to distance
  const xToDist = useCallback(
    (x: number): number => {
      if (!analytics || analytics.distances.length < 2) return 0;
      const maxDist = analytics.distances[analytics.distances.length - 1];
      const plotW = size.width - PADDING.left - PADDING.right;
      const d = ((x - PADDING.left) / plotW) * maxDist;
      return Math.max(0, Math.min(d, maxDist));
    },
    [analytics, size.width],
  );

  // Draw chart
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || size.width === 0 || size.height === 0) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    canvas.width = size.width * dpr;
    canvas.height = size.height * dpr;
    ctx.scale(dpr, dpr);

    // Clear
    ctx.clearRect(0, 0, size.width, size.height);

    const plotW = size.width - PADDING.left - PADDING.right;
    const plotH = size.height - PADDING.top - PADDING.bottom;
    const yMax = maxVelocity * 1.1;

    if (!analytics || analytics.distances.length < 2 || plotW <= 0 || plotH <= 0) {
      ctx.fillStyle = '#6b6b7a';
      ctx.font = '12px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('No path data', size.width / 2, size.height / 2);
      return;
    }

    const maxDist = analytics.distances[analytics.distances.length - 1];
    const n = analytics.distances.length;

    // Grid lines
    ctx.strokeStyle = '#252530';
    ctx.lineWidth = 1;

    // Y grid
    const yStep = computeNiceStep(yMax, plotH, 30);
    for (let v = 0; v <= yMax; v += yStep) {
      const y = velToY(v);
      ctx.beginPath();
      ctx.moveTo(PADDING.left, y);
      ctx.lineTo(PADDING.left + plotW, y);
      ctx.stroke();
    }

    // X grid
    const xStep = computeNiceStep(maxDist, plotW, 50);
    for (let d = 0; d <= maxDist; d += xStep) {
      const x = distToX(d);
      ctx.beginPath();
      ctx.moveTo(x, PADDING.top);
      ctx.lineTo(x, PADDING.top + plotH);
      ctx.stroke();
    }

    // Draw filled area chart with velocity-to-color gradient
    const baseY = velToY(0);
    for (let i = 0; i < n - 1; i++) {
      const x1 = distToX(analytics.distances[i]);
      const x2 = distToX(analytics.distances[i + 1]);
      const y1 = velToY(analytics.velocities[i]);
      const y2 = velToY(analytics.velocities[i + 1]);
      const avgV = (analytics.velocities[i] + analytics.velocities[i + 1]) / 2;

      // Parse the hsl color and make it translucent for fill
      const color = velocityToColor(avgV, maxVelocity);

      // Fill
      ctx.fillStyle = color.replace('rgb(', 'rgba(').replace(')', ', 0.3)');
      ctx.beginPath();
      ctx.moveTo(x1, baseY);
      ctx.lineTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.lineTo(x2, baseY);
      ctx.closePath();
      ctx.fill();

      // Line on top
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }

    // Y-axis labels
    ctx.fillStyle = '#6b6b7a';
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let v = 0; v <= yMax; v += yStep) {
      const y = velToY(v);
      ctx.fillText(v.toFixed(1), PADDING.left - 6, y);
    }

    // X-axis labels
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let d = 0; d <= maxDist; d += xStep) {
      const x = distToX(d);
      ctx.fillText(d.toFixed(1), x, PADDING.top + plotH + 6);
    }

    // Axis labels
    ctx.fillStyle = '#4a4a58';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('distance (m)', PADDING.left + plotW / 2, size.height - 4);

    ctx.save();
    ctx.translate(10, PADDING.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText('velocity (m/s)', 0, 0);
    ctx.restore();

    // Plot border
    ctx.strokeStyle = '#32323e';
    ctx.lineWidth = 1;
    ctx.strokeRect(PADDING.left, PADDING.top, plotW, plotH);

    // Scrubber line
    if (scrubberDistance > 0 && maxDist > 0) {
      const sx = distToX(scrubberDistance);

      // Vertical line
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(sx, PADDING.top);
      ctx.lineTo(sx, PADDING.top + plotH);
      ctx.stroke();

      // Handle circle at the top
      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(sx, PADDING.top, 4, 0, Math.PI * 2);
      ctx.fill();

      // Value dot on the curve
      if (profile) {
        const v = profile.getVelocity(scrubberDistance);
        const dotY = velToY(v);
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(sx, dotY, 3, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 1;
        ctx.stroke();
      }
    }
  }, [
    analytics,
    profile,
    maxVelocity,
    size,
    scrubberDistance,
    playbackState,
    distToX,
    velToY,
  ]);

  // Mouse event handlers
  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      if (!analytics || analytics.distances.length < 2) return;
      const rect = canvasRef.current?.getBoundingClientRect();
      if (!rect) return;
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;

      // Check if within plot area
      if (
        x >= PADDING.left &&
        x <= size.width - PADDING.right &&
        y >= PADDING.top &&
        y <= size.height - PADDING.bottom
      ) {
        const d = xToDist(x);
        setScrubberDistance(d);
        setIsDragging(true);
      }
    },
    [analytics, size, xToDist, setScrubberDistance],
  );

  const handleMouseMove = useCallback(
    (e: React.MouseEvent) => {
      if (!analytics || analytics.distances.length < 2) return;
      const rect = canvasRef.current?.getBoundingClientRect();
      if (!rect) return;
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;

      if (isDragging) {
        const d = xToDist(x);
        setScrubberDistance(d);
      }

      // Tooltip
      if (
        x >= PADDING.left &&
        x <= size.width - PADDING.right &&
        y >= PADDING.top &&
        y <= size.height - PADDING.bottom
      ) {
        const d = xToDist(x);
        const vel = profile?.getVelocity(d) ?? 0;
        const time = timeEstimator?.getTime(d) ?? 0;
        setTooltip({
          x: e.clientX - rect.left,
          y: e.clientY - rect.top,
          text: `d: ${d.toFixed(2)}m  v: ${vel.toFixed(2)}m/s  t: ${time.toFixed(2)}s`,
        });
      } else {
        setTooltip(null);
      }
    },
    [analytics, isDragging, size, xToDist, profile, timeEstimator, setScrubberDistance],
  );

  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
  }, []);

  const handleMouseLeave = useCallback(() => {
    setIsDragging(false);
    setTooltip(null);
  }, []);

  return (
    <div ref={containerRef} className="relative w-full h-full">
      <canvas
        ref={canvasRef}
        style={{ width: size.width, height: size.height }}
        className="cursor-crosshair"
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseLeave}
      />
      {tooltip && (
        <div
          className="tooltip"
          style={{
            left: Math.min(tooltip.x + 12, size.width - 200),
            top: tooltip.y - 28,
          }}
        >
          {tooltip.text}
        </div>
      )}
    </div>
  );
}

function computeNiceStep(range: number, pixels: number, minPixelGap: number): number {
  if (range <= 0 || pixels <= 0) return 1;
  const rawStep = (range * minPixelGap) / pixels;
  const magnitude = Math.pow(10, Math.floor(Math.log10(rawStep)));
  const normalized = rawStep / magnitude;
  let nice: number;
  if (normalized <= 1) nice = 1;
  else if (normalized <= 2) nice = 2;
  else if (normalized <= 5) nice = 5;
  else nice = 10;
  return nice * magnitude;
}
