import { useRef, useEffect, useCallback, useState } from 'react';
import { useEditorStore } from '../../stores/editorStore';
import { usePathStore } from '../../stores/pathStore';
import { HEADING_COLOR } from '../../utils/colors';
import type { AnalyticsArrays } from '../../math/ProfileAnalytics';
import type { SplinePath } from '../../math/SplinePath';

interface HeadingChartProps {
  analytics: AnalyticsArrays | null;
  splinePath: SplinePath | null;
}

const PADDING = { top: 20, right: 16, bottom: 32, left: 48 };

export function HeadingChart({ analytics, splinePath }: HeadingChartProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [tooltip, setTooltip] = useState<{ x: number; y: number; text: string } | null>(null);

  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);
  const setScrubberDistance = useEditorStore((s) => s.setScrubberDistance);
  const playbackState = useEditorStore((s) => s.playbackState);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);

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

  // Compute heading range for Y axis
  const getHeadingRange = useCallback((): { minDeg: number; maxDeg: number } => {
    if (!analytics || analytics.headings.length < 2) return { minDeg: -180, maxDeg: 180 };
    let minH = Infinity;
    let maxH = -Infinity;
    let anyValid = false;
    for (const h of analytics.headings) {
      if (isNaN(h)) continue;
      const deg = (h * 180) / Math.PI;
      anyValid = true;
      if (deg < minH) minH = deg;
      if (deg > maxH) maxH = deg;
    }
    if (!anyValid) return { minDeg: -180, maxDeg: 180 };
    const pad = Math.max((maxH - minH) * 0.15, 15);
    return { minDeg: minH - pad, maxDeg: maxH + pad };
  }, [analytics]);

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

  const degToY = useCallback(
    (deg: number): number => {
      const plotH = size.height - PADDING.top - PADDING.bottom;
      const { minDeg, maxDeg } = getHeadingRange();
      const range = maxDeg - minDeg;
      if (range <= 0) return PADDING.top + plotH / 2;
      return PADDING.top + plotH * (1 - (deg - minDeg) / range);
    },
    [getHeadingRange, size.height],
  );

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

  const waypointIndexToDistance = useCallback(
    (wpIndex: number): number => {
      if (!splinePath || controlPoints.length < 2) return 0;
      const frac = wpIndex / (controlPoints.length - 1);
      return frac * splinePath.totalLength;
    },
    [splinePath, controlPoints.length],
  );

  // Interpolate heading at distance from analytics arrays (linear between samples)
  const getHeadingAtDistance = useCallback(
    (d: number): number => {
      if (!analytics || analytics.distances.length < 2) return NaN;
      const dists = analytics.distances;
      const headings = analytics.headings;
      if (d <= dists[0]) return headings[0];
      if (d >= dists[dists.length - 1]) return headings[headings.length - 1];

      let lo = 0;
      let hi = dists.length - 1;
      while (lo + 1 < hi) {
        const mid = (lo + hi) >>> 1;
        if (dists[mid] <= d) lo = mid;
        else hi = mid;
      }
      const frac = (d - dists[lo]) / (dists[hi] - dists[lo]);
      const hLo = headings[lo];
      const hHi = headings[hi];
      if (isNaN(hLo) || isNaN(hHi)) return NaN;
      // Shortest-arc interpolation
      let diff = ((hHi - hLo + Math.PI) % (2 * Math.PI)) - Math.PI;
      if (diff < -Math.PI) diff += 2 * Math.PI;
      return hLo + diff * frac;
    },
    [analytics],
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

    ctx.clearRect(0, 0, size.width, size.height);

    const plotW = size.width - PADDING.left - PADDING.right;
    const plotH = size.height - PADDING.top - PADDING.bottom;
    const { minDeg, maxDeg } = getHeadingRange();

    if (!analytics || analytics.distances.length < 2 || plotW <= 0 || plotH <= 0) {
      ctx.fillStyle = '#6b6b7a';
      ctx.font = '12px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('No heading data', size.width / 2, size.height / 2);
      return;
    }

    const maxDist = analytics.distances[analytics.distances.length - 1];
    const n = analytics.distances.length;

    // Check if there are any valid heading values
    let hasValidHeading = false;
    for (const h of analytics.headings) {
      if (!isNaN(h)) {
        hasValidHeading = true;
        break;
      }
    }

    if (!hasValidHeading) {
      ctx.fillStyle = '#6b6b7a';
      ctx.font = '12px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('No heading waypoints set', size.width / 2, size.height / 2);
      return;
    }

    // Grid lines
    ctx.strokeStyle = '#252530';
    ctx.lineWidth = 1;

    const degRange = maxDeg - minDeg;
    const yStep = computeNiceStep(degRange, plotH, 30);
    const yStart = Math.ceil(minDeg / yStep) * yStep;
    for (let deg = yStart; deg <= maxDeg; deg += yStep) {
      const y = degToY(deg);
      ctx.beginPath();
      ctx.moveTo(PADDING.left, y);
      ctx.lineTo(PADDING.left + plotW, y);
      ctx.stroke();
    }

    const xStep = computeNiceStep(maxDist, plotW, 50);
    for (let d = 0; d <= maxDist; d += xStep) {
      const x = distToX(d);
      ctx.beginPath();
      ctx.moveTo(x, PADDING.top);
      ctx.lineTo(x, PADDING.top + plotH);
      ctx.stroke();
    }

    // Draw heading line
    ctx.strokeStyle = HEADING_COLOR;
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < n; i++) {
      const h = analytics.headings[i];
      if (isNaN(h)) continue;
      const x = distToX(analytics.distances[i]);
      const deg = (h * 180) / Math.PI;
      const y = degToY(deg);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();

    // Draw heading waypoint dots as circles on the line
    for (const hw of headingWaypoints) {
      if (controlPoints.length < 2) continue;
      const wpDist = waypointIndexToDistance(hw.waypointIndex);
      const x = distToX(wpDist);
      const y = degToY(hw.degrees);

      // Outer circle
      ctx.fillStyle = HEADING_COLOR;
      ctx.beginPath();
      ctx.arc(x, y, 5, 0, Math.PI * 2);
      ctx.fill();

      // Inner white circle
      ctx.fillStyle = '#0a0a0f';
      ctx.beginPath();
      ctx.arc(x, y, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }

    // Y-axis labels
    ctx.fillStyle = '#6b6b7a';
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let deg = yStart; deg <= maxDeg; deg += yStep) {
      const y = degToY(deg);
      ctx.fillText(`${deg.toFixed(0)}`, PADDING.left - 6, y);
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
    ctx.fillText('heading (deg)', 0, 0);
    ctx.restore();

    // Plot border
    ctx.strokeStyle = '#32323e';
    ctx.lineWidth = 1;
    ctx.strokeRect(PADDING.left, PADDING.top, plotW, plotH);

    // Scrubber line
    if (scrubberDistance > 0 && maxDist > 0) {
      const sx = distToX(scrubberDistance);

      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(sx, PADDING.top);
      ctx.lineTo(sx, PADDING.top + plotH);
      ctx.stroke();

      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(sx, PADDING.top, 4, 0, Math.PI * 2);
      ctx.fill();

      // Value dot
      const hRad = getHeadingAtDistance(scrubberDistance);
      if (!isNaN(hRad)) {
        const deg = (hRad * 180) / Math.PI;
        const dotY = degToY(deg);
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
    splinePath,
    size,
    scrubberDistance,
    controlPoints.length,
    headingWaypoints,
    playbackState,
    getHeadingRange,
    distToX,
    degToY,
    waypointIndexToDistance,
    getHeadingAtDistance,
  ]);

  // Mouse handlers
  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      if (!analytics || analytics.distances.length < 2) return;
      const rect = canvasRef.current?.getBoundingClientRect();
      if (!rect) return;
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;

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

      if (
        x >= PADDING.left &&
        x <= size.width - PADDING.right &&
        y >= PADDING.top &&
        y <= size.height - PADDING.bottom
      ) {
        const d = xToDist(x);
        const hRad = getHeadingAtDistance(d);
        const hDeg = isNaN(hRad) ? 'N/A' : `${((hRad * 180) / Math.PI).toFixed(1)} deg`;
        setTooltip({
          x: e.clientX - rect.left,
          y: e.clientY - rect.top,
          text: `d: ${d.toFixed(2)}m  heading: ${hDeg}`,
        });
      } else {
        setTooltip(null);
      }
    },
    [analytics, isDragging, size, xToDist, getHeadingAtDistance, setScrubberDistance],
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
