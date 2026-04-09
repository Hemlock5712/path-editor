import {
  Point,
  NamedPoint,
  HeadingWaypoint,
  ConstraintZone,
  RotationZone,
  FIELD_WIDTH,
  FIELD_HEIGHT,
} from '../../types';
import {
  fieldToCanvas,
  canvasToField,
  getScale,
  type CanvasTransform,
} from '../../utils/canvasTransform';
import { curvatureToColor, drawOutlinedText } from '../../utils/colors';
import { SplinePath } from '../../math/SplinePath';
import {
  buildArcLengthHeadings,
  interpolateHeadingSorted,
} from '../../math/ProfileAnalytics';
import { shortestArcDiff } from '../../math/angleUtils';

// ─── Inactive Path ──────────────────────────────────────────────────────────

const INACTIVE_COLORS = [
  'rgba(100, 140, 255, 0.25)',
  'rgba(180, 100, 255, 0.25)',
  'rgba(255, 140, 60, 0.25)',
  'rgba(255, 80, 120, 0.25)',
  'rgba(80, 220, 200, 0.25)',
];

export function drawInactivePath(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath,
  controlPoints: Point[],
  label: string,
  colorIndex: number
): void {
  if (splinePath.totalLength <= 0) return;

  const color = INACTIVE_COLORS[colorIndex % INACTIVE_COLORS.length];
  const numSamples = Math.max(150, Math.floor(transform.zoom * 150));
  const ds = splinePath.totalLength / numSamples;
  const lineWidth = Math.max(1.5, Math.min(3, 2.5 / Math.sqrt(transform.zoom)));

  // Draw spline
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  const start = fieldToCanvas(splinePath.getPoint(0), cw, ch, transform);
  ctx.moveTo(start.cx, start.cy);
  for (let i = 1; i <= numSamples; i++) {
    const pt = fieldToCanvas(splinePath.getPoint(i * ds), cw, ch, transform);
    ctx.lineTo(pt.cx, pt.cy);
  }
  ctx.stroke();
  ctx.restore();

  // Draw small dots for control points
  const dotRadius = Math.max(3, Math.min(5, 4 / Math.sqrt(transform.zoom)));
  for (const pt of controlPoints) {
    const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);
    ctx.beginPath();
    ctx.arc(cx, cy, dotRadius, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
  }

  // Label near start point
  if (controlPoints.length > 0) {
    const { cx, cy } = fieldToCanvas(controlPoints[0], cw, ch, transform);
    const fontSize = Math.max(9, Math.min(13, 11 / Math.sqrt(transform.zoom)));
    ctx.save();
    ctx.font = `bold ${fontSize}px monospace`;
    ctx.fillStyle = color.replace(/[\d.]+\)$/, '0.6)');
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText(label, cx + dotRadius + 4, cy - dotRadius - 2);
    ctx.restore();
  }
}

// ─── Grid ───────────────────────────────────────────────────────────────────

export function drawGrid(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  showGrid: boolean
): void {
  if (!showGrid) return;

  const zoom = transform.zoom;
  let step: number;
  if (zoom >= 4) step = 0.25;
  else if (zoom >= 2) step = 0.5;
  else step = 1;

  // Subtler grid for neon minimal
  const alpha = zoom >= 4 ? 0.03 : zoom >= 2 ? 0.04 : 0.05;
  ctx.strokeStyle = `rgba(0, 255, 170, ${alpha})`;
  ctx.lineWidth = 1;

  // Compute visible field region for culling
  const topLeft = canvasToField(0, 0, cw, ch, transform);
  const bottomRight = canvasToField(cw, ch, cw, ch, transform);
  const minX = Math.floor(Math.min(topLeft.x, bottomRight.x) / step) * step;
  const maxX = Math.ceil(Math.max(topLeft.x, bottomRight.x) / step) * step;
  const minY = Math.floor(Math.min(topLeft.y, bottomRight.y) / step) * step;
  const maxY = Math.ceil(Math.max(topLeft.y, bottomRight.y) / step) * step;

  ctx.beginPath();
  for (let x = minX; x <= maxX; x += step) {
    if (x < 0 || x > FIELD_WIDTH) continue;
    const { cx: cx0 } = fieldToCanvas({ x, y: 0 }, cw, ch, transform);
    const { cy: cy0 } = fieldToCanvas(
      { x, y: FIELD_HEIGHT },
      cw,
      ch,
      transform
    );
    const { cy: cy1 } = fieldToCanvas({ x, y: 0 }, cw, ch, transform);
    ctx.moveTo(cx0, cy0);
    ctx.lineTo(cx0, cy1);
  }
  for (let y = minY; y <= maxY; y += step) {
    if (y < 0 || y > FIELD_HEIGHT) continue;
    const { cy: cy0 } = fieldToCanvas({ x: 0, y }, cw, ch, transform);
    const { cx: cx0 } = fieldToCanvas({ x: 0, y }, cw, ch, transform);
    const { cx: cx1 } = fieldToCanvas({ x: FIELD_WIDTH, y }, cw, ch, transform);
    ctx.moveTo(cx0, cy0);
    ctx.lineTo(cx1, cy0);
  }
  ctx.stroke();
}

// ─── Constraint Zones ────────────────────────────────────────────────────────

export function drawConstraintZones(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath,
  constraintZones: ConstraintZone[]
): void {
  if (constraintZones.length === 0 || splinePath.totalLength <= 0) return;

  const lineWidth = Math.max(6, Math.min(16, 12 / Math.sqrt(transform.zoom)));
  const numSamples = Math.max(200, Math.floor(transform.zoom * 200));
  const ds = splinePath.totalLength / numSamples;

  for (const zone of constraintZones) {
    const startS = splinePath.getArcLengthAtWaypointIndex(
      zone.startWaypointIndex
    );
    const endS = splinePath.getArcLengthAtWaypointIndex(zone.endWaypointIndex);

    // Draw wider amber overlay along the zone
    ctx.save();
    ctx.strokeStyle = 'rgba(255, 180, 0, 0.2)';
    ctx.lineWidth = lineWidth;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.shadowColor = 'rgba(255, 180, 0, 0.15)';
    ctx.shadowBlur = 8;

    ctx.beginPath();
    let started = false;
    for (let i = 0; i <= numSamples; i++) {
      const s = i * ds;
      if (s < startS || s > endS) {
        if (started) break;
        continue;
      }
      const pt = splinePath.getPoint(s);
      const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);
      if (!started) {
        ctx.moveTo(cx, cy);
        started = true;
      } else {
        ctx.lineTo(cx, cy);
      }
    }
    ctx.stroke();
    ctx.restore();
  }
}

// ─── Rotation Zones ─────────────────────────────────────────────────────────

export function drawRotationZones(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath,
  rotationZones: RotationZone[],
  selectedZoneId: string | null
): void {
  if (rotationZones.length === 0 || splinePath.totalLength <= 0) return;

  const numCPs = splinePath.controlPoints.length;
  const lineWidth = Math.max(6, Math.min(16, 12 / Math.sqrt(transform.zoom)));
  const numSamples = Math.max(200, Math.floor(transform.zoom * 200));
  const ds = splinePath.totalLength / numSamples;
  const handleSize = Math.max(5, Math.min(10, 8 / Math.sqrt(transform.zoom)));

  for (const zone of rotationZones) {
    const isSelected = zone.id === selectedZoneId;
    const startFrac = zone.startWaypointIndex / (numCPs - 1);
    const endFrac = zone.endWaypointIndex / (numCPs - 1);
    const startS = startFrac * splinePath.totalLength;
    const endS = endFrac * splinePath.totalLength;

    const color = isSelected
      ? 'rgba(255, 153, 51, 0.4)'
      : 'rgba(255, 153, 51, 0.2)';
    const glowAlpha = isSelected ? 0.3 : 0.15;

    // Draw orange overlay along the zone segment
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.shadowColor = `rgba(255, 153, 51, ${glowAlpha})`;
    ctx.shadowBlur = isSelected ? 12 : 8;

    ctx.beginPath();
    let started = false;
    for (let i = 0; i <= numSamples; i++) {
      const s = i * ds;
      if (s < startS || s > endS) {
        if (started) break;
        continue;
      }
      const pt = splinePath.getPoint(s);
      const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);
      if (!started) {
        ctx.moveTo(cx, cy);
        started = true;
      } else {
        ctx.lineTo(cx, cy);
      }
    }
    ctx.stroke();
    ctx.restore();

    // Boundary tick marks (drag handles)
    for (const boundaryS of [startS, endS]) {
      const pt = splinePath.getPoint(boundaryS);
      const tan = splinePath.getTangent(boundaryS);
      const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);
      // Perpendicular to path tangent (canvas Y is inverted)
      const nx = tan.y;
      const ny = tan.x;

      ctx.save();
      ctx.strokeStyle = isSelected ? '#FF9933' : 'rgba(255, 153, 51, 0.6)';
      ctx.lineWidth = 2.5;
      ctx.shadowColor = '#FF9933';
      ctx.shadowBlur = isSelected ? 8 : 4;
      ctx.beginPath();
      ctx.moveTo(cx - nx * handleSize * 1.5, cy - ny * handleSize * 1.5);
      ctx.lineTo(cx + nx * handleSize * 1.5, cy + ny * handleSize * 1.5);
      ctx.stroke();
      ctx.restore();
    }

    // Aim line: dashed line from zone midpoint on path to target
    const midS = (startS + endS) / 2;
    const midPt = splinePath.getPoint(midS);
    const midC = fieldToCanvas(midPt, cw, ch, transform);
    const targetC = fieldToCanvas(zone.targetPoint, cw, ch, transform);

    ctx.save();
    ctx.strokeStyle = isSelected
      ? 'rgba(255, 153, 51, 0.5)'
      : 'rgba(255, 153, 51, 0.25)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(midC.cx, midC.cy);
    ctx.lineTo(targetC.cx, targetC.cy);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();

    // Target point: bullseye
    const targetAlpha = isSelected ? 1.0 : 0.6;
    ctx.save();
    ctx.strokeStyle = `rgba(255, 153, 51, ${targetAlpha})`;
    ctx.fillStyle = `rgba(255, 153, 51, ${targetAlpha * 0.3})`;
    ctx.lineWidth = 2;
    ctx.shadowColor = '#FF9933';
    ctx.shadowBlur = isSelected ? 10 : 4;

    // Outer ring
    ctx.beginPath();
    ctx.arc(targetC.cx, targetC.cy, handleSize * 1.2, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();

    // Inner dot
    ctx.beginPath();
    ctx.arc(targetC.cx, targetC.cy, handleSize * 0.4, 0, 2 * Math.PI);
    ctx.fillStyle = `rgba(255, 153, 51, ${targetAlpha})`;
    ctx.fill();

    // Crosshair lines
    ctx.beginPath();
    ctx.moveTo(targetC.cx - handleSize * 1.8, targetC.cy);
    ctx.lineTo(targetC.cx - handleSize * 0.6, targetC.cy);
    ctx.moveTo(targetC.cx + handleSize * 0.6, targetC.cy);
    ctx.lineTo(targetC.cx + handleSize * 1.8, targetC.cy);
    ctx.moveTo(targetC.cx, targetC.cy - handleSize * 1.8);
    ctx.lineTo(targetC.cx, targetC.cy - handleSize * 0.6);
    ctx.moveTo(targetC.cx, targetC.cy + handleSize * 0.6);
    ctx.lineTo(targetC.cx, targetC.cy + handleSize * 1.8);
    ctx.stroke();
    ctx.restore();
  }
}

// ─── Path ───────────────────────────────────────────────────────────────────

export function drawPath(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath
): void {
  if (splinePath.totalLength <= 0) return;

  // More samples at higher zoom for smoother curves
  const numSamples = Math.max(300, Math.floor(transform.zoom * 300));
  const ds = splinePath.totalLength / numSamples;
  // Scale line width inversely so it doesn't get too thick when zoomed or too thin when zoomed out
  const lineWidth = Math.max(1.5, Math.min(4, 3 / Math.sqrt(transform.zoom)));

  // Build a shared Path2D for shadow + bloom (avoids duplicating polyline loop)
  const pathLine = new Path2D();
  const start = fieldToCanvas(splinePath.getPoint(0), cw, ch, transform);
  pathLine.moveTo(start.cx, start.cy);
  for (let i = 1; i <= numSamples; i++) {
    const pt = fieldToCanvas(splinePath.getPoint(i * ds), cw, ch, transform);
    pathLine.lineTo(pt.cx, pt.cy);
  }

  // ── SHADOW PASS: soft dark aura for contrast on light backgrounds ──
  ctx.save();
  ctx.strokeStyle = 'rgba(0, 0, 0, 0.01)';
  ctx.lineWidth = lineWidth * 2;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.shadowColor = 'rgba(0, 0, 0, 0.7)';
  ctx.shadowBlur = 10;
  ctx.stroke(pathLine);
  ctx.restore();

  // ── BLOOM PASS: wider, dimmer neon green glow behind the path ──
  ctx.save();
  ctx.globalAlpha = 0.15;
  ctx.strokeStyle = '#00FFaa';
  ctx.lineWidth = lineWidth * 4;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.stroke(pathLine);
  ctx.restore();

  // ── MAIN PASS: curvature-colored segments ──
  for (let i = 0; i < numSamples; i++) {
    const s0 = i * ds;
    const s1 = (i + 1) * ds;
    const p0 = splinePath.getPoint(s0);
    const p1 = splinePath.getPoint(s1);
    const k = splinePath.getCurvature((s0 + s1) / 2);

    const c0 = fieldToCanvas(p0, cw, ch, transform);
    const c1 = fieldToCanvas(p1, cw, ch, transform);

    ctx.strokeStyle = curvatureToColor(k);
    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    ctx.moveTo(c0.cx, c0.cy);
    ctx.lineTo(c1.cx, c1.cy);
    ctx.stroke();
  }

  // Draw direction arrowheads along the path
  const arrowSpacing = 0.5; // meters between arrows
  const numArrows = Math.floor(splinePath.totalLength / arrowSpacing);
  const arrowSize = Math.max(4, Math.min(8, 6 / Math.sqrt(transform.zoom)));

  ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
  for (let i = 1; i <= numArrows; i++) {
    const s = i * arrowSpacing;
    if (s >= splinePath.totalLength) break;
    const pt = splinePath.getPoint(s);
    const tan = splinePath.getTangent(s);
    const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);

    // Tangent angle in canvas space (Y is inverted on canvas)
    const angle = Math.atan2(-tan.y, tan.x);

    ctx.beginPath();
    ctx.moveTo(
      cx + Math.cos(angle) * arrowSize,
      cy + Math.sin(angle) * arrowSize
    );
    ctx.lineTo(
      cx + Math.cos(angle + 2.5) * arrowSize * 0.7,
      cy + Math.sin(angle + 2.5) * arrowSize * 0.7
    );
    ctx.lineTo(
      cx + Math.cos(angle - 2.5) * arrowSize * 0.7,
      cy + Math.sin(angle - 2.5) * arrowSize * 0.7
    );
    ctx.closePath();
    ctx.fill();
  }
}

// ─── Named Point Markers ─────────────────────────────────────────────────────

export function drawNamedPoints(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  namedPoints: Record<string, NamedPoint>,
  controlPointRefs: (string | null)[]
): void {
  const points = Object.values(namedPoints);
  if (points.length === 0) return;

  // Names already shown by drawControlPoints — skip to avoid duplicates
  const linkedNames = new Set(
    controlPointRefs.filter((r): r is string => r !== null)
  );

  const size = Math.max(4, Math.min(7, 6 / Math.sqrt(transform.zoom)));
  const fontSize = Math.max(14, Math.min(20, 18 / Math.sqrt(transform.zoom)));

  for (const np of points) {
    const { cx, cy } = fieldToCanvas(np, cw, ch, transform);
    const isMirror = np.name.endsWith(' (Mirror)');
    const alpha = isMirror ? 0.25 : 0.5;

    // Diamond marker with soft dark shadow
    ctx.save();
    ctx.fillStyle = `rgba(255, 179, 0, ${alpha * 0.4})`;
    ctx.strokeStyle = `rgba(255, 179, 0, ${alpha})`;
    ctx.lineWidth = 1.5;
    ctx.shadowColor = 'rgba(0, 0, 0, 0.6)';
    ctx.shadowBlur = 6;
    ctx.beginPath();
    ctx.moveTo(cx, cy - size);
    ctx.lineTo(cx + size, cy);
    ctx.lineTo(cx, cy + size);
    ctx.lineTo(cx - size, cy);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Name label (only for primary points that aren't currently linked
    // on the active path — linked points already show the name via
    // drawControlPoints, so skip to avoid duplicate text)
    if (!isMirror && !linkedNames.has(np.name)) {
      ctx.save();
      ctx.font = `${fontSize}px monospace`;
      ctx.textAlign = 'left';
      ctx.textBaseline = 'bottom';
      drawOutlinedText(
        ctx,
        np.name,
        cx + size + 3,
        cy - size,
        `rgba(255, 179, 0, ${alpha * 0.8})`
      );
      ctx.restore();
    }
  }
}

// ─── Control Points ─────────────────────────────────────────────────────────

export function drawControlPoints(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  controlPoints: Point[],
  controlPointRefs: (string | null)[],
  selectedIndex: number | null,
  hoveredIndex: number | null
): void {
  if (controlPoints.length === 0) return;

  // Base radius in pixels, scaled inversely with zoom so points stay manageable
  const baseRadius = Math.max(5, Math.min(10, 8 / Math.sqrt(transform.zoom)));
  const selectedRadius = baseRadius * 1.35;
  const hoveredRadius = baseRadius * 1.15;
  const fontSize = Math.max(8, Math.min(13, 11 / Math.sqrt(transform.zoom)));

  controlPoints.forEach((pt, i) => {
    const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);
    const isSelected = i === selectedIndex;
    const isHovered = i === hoveredIndex;
    const isFirst = i === 0;
    const isLast = i === controlPoints.length - 1;
    const ref = controlPointRefs[i] || null;

    let radius = baseRadius;
    if (isSelected) radius = selectedRadius;
    else if (isHovered) radius = hoveredRadius;

    // Neon glow ring for selected/hovered
    if (isSelected || isHovered) {
      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, radius + 6, 0, 2 * Math.PI);
      ctx.strokeStyle = isSelected
        ? 'rgba(0, 255, 170, 0.4)'
        : 'rgba(0, 255, 170, 0.2)';
      ctx.lineWidth = 2;
      ctx.shadowColor = '#00FFaa';
      ctx.shadowBlur = isSelected ? 12 : 6;
      ctx.stroke();
      ctx.restore();
    }

    // Amber ring for linked points
    if (ref) {
      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, radius + 3, 0, 2 * Math.PI);
      ctx.strokeStyle = 'rgba(255, 179, 0, 0.6)';
      ctx.lineWidth = 2;
      ctx.shadowColor = '#FFB300';
      ctx.shadowBlur = 6;
      ctx.stroke();
      ctx.restore();
    }

    ctx.save();
    // Soft dark shadow for contrast on light backgrounds
    ctx.shadowColor = 'rgba(0, 0, 0, 0.6)';
    ctx.shadowBlur = 8;
    if (isFirst) {
      // Neon green diamond for first point
      ctx.fillStyle = ref ? '#FFB300' : '#00FFaa';
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(cx, cy - radius);
      ctx.lineTo(cx + radius, cy);
      ctx.lineTo(cx, cy + radius);
      ctx.lineTo(cx - radius, cy);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    } else if (isLast) {
      // Neon pink-red square for last point (amber if linked)
      ctx.fillStyle = ref ? '#FFB300' : '#FF3366';
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.fillRect(cx - radius, cy - radius, radius * 2, radius * 2);
      ctx.strokeRect(cx - radius, cy - radius, radius * 2, radius * 2);
    } else {
      // Circle for interior points (amber-tinted if linked)
      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
      if (ref) {
        ctx.fillStyle = isHovered
          ? 'rgba(255, 199, 50, 0.95)'
          : 'rgba(255, 179, 0, 0.8)';
      } else {
        ctx.fillStyle = isHovered
          ? 'rgba(255, 255, 255, 0.9)'
          : 'rgba(255, 255, 255, 0.7)';
      }
      ctx.fill();
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.4)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    ctx.restore();

    // Label: show name for linked points, index for regular
    ctx.fillStyle = '#000';
    ctx.font = `bold ${fontSize}px monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(i), cx, cy);

    // Show named point name above the point if linked
    if (ref) {
      const nameFontSize = Math.max(
        14,
        Math.min(20, 18 / Math.sqrt(transform.zoom))
      );
      ctx.save();
      ctx.font = `${nameFontSize}px monospace`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'bottom';
      drawOutlinedText(ctx, ref, cx, cy - radius - 4, 'rgba(255, 179, 0, 0.8)');
      ctx.restore();
    }
  });
}

// ─── Heading Arrows ─────────────────────────────────────────────────────────

export function drawHeadingArrows(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  controlPoints: Point[],
  headingWaypoints: HeadingWaypoint[]
): void {
  if (headingWaypoints.length === 0 || controlPoints.length === 0) return;

  const arrowLen = Math.max(15, Math.min(30, 25 / Math.sqrt(transform.zoom)));
  const headLen = arrowLen * 0.32;

  headingWaypoints.forEach((hw) => {
    const idx = Math.round(hw.waypointIndex);
    if (idx < 0 || idx >= controlPoints.length) return;

    const pt = controlPoints[idx];
    const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);
    const rad = (hw.degrees * Math.PI) / 180;

    // Arrow line — neon cyan
    const tipX = cx + Math.cos(rad) * arrowLen;
    // Canvas Y is flipped relative to field Y
    const tipY = cy - Math.sin(rad) * arrowLen;

    // Arrow line — neon cyan with dark shadow + glow
    const angle = Math.atan2(tipY - cy, tipX - cx);

    ctx.save();
    ctx.strokeStyle = '#00DDFF';
    ctx.lineWidth = 2.5;
    ctx.lineCap = 'round';
    ctx.shadowColor = 'rgba(0, 0, 0, 0.6)';
    ctx.shadowBlur = 8;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();

    // Arrowhead
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(
      tipX - headLen * Math.cos(angle - 0.4),
      tipY - headLen * Math.sin(angle - 0.4)
    );
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(
      tipX - headLen * Math.cos(angle + 0.4),
      tipY - headLen * Math.sin(angle + 0.4)
    );
    ctx.stroke();
    ctx.restore();

    // Degree label with outline
    ctx.font = 'bold 10px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    drawOutlinedText(
      ctx,
      `${hw.degrees.toFixed(0)}`,
      tipX,
      tipY - 4,
      '#00DDFF'
    );
  });
}

// ─── Connection Lines ───────────────────────────────────────────────────────

export function drawConnectionLines(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  controlPoints: Point[]
): void {
  if (controlPoints.length < 2) return;

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 6]);
  ctx.beginPath();
  const first = fieldToCanvas(controlPoints[0], cw, ch, transform);
  ctx.moveTo(first.cx, first.cy);
  for (let i = 1; i < controlPoints.length; i++) {
    const c = fieldToCanvas(controlPoints[i], cw, ch, transform);
    ctx.lineTo(c.cx, c.cy);
  }
  ctx.stroke();
  ctx.setLineDash([]);
}

// ─── Scrubber Ghost ─────────────────────────────────────────────────────────

export function drawScrubberGhost(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath,
  scrubberDistance: number,
  heading: number | null,
  robotLength: number,
  robotWidth: number
): void {
  if (splinePath.totalLength <= 0 || scrubberDistance <= 0) return;

  const pt = splinePath.getPoint(scrubberDistance);
  const tan = splinePath.getTangent(scrubberDistance);
  const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);
  const scale = getScale(cw, transform);

  const rot = heading ?? Math.atan2(tan.y, tan.x);

  const rw = robotLength * scale;
  const rh = robotWidth * scale;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-rot); // Canvas rotation is CW, field rotation is CCW

  // Neon green robot outline with dark shadow for contrast
  ctx.fillStyle = 'rgba(0, 255, 170, 0.08)';
  ctx.shadowColor = 'rgba(0, 0, 0, 0.5)';
  ctx.shadowBlur = 8;
  ctx.fillRect(-rw / 2, -rh / 2, rw, rh);

  ctx.strokeStyle = 'rgba(0, 255, 170, 0.35)';
  ctx.lineWidth = 2;
  ctx.strokeRect(-rw / 2, -rh / 2, rw, rh);

  // Heading triangle at the front
  ctx.fillStyle = 'rgba(0, 255, 170, 0.5)';
  ctx.beginPath();
  ctx.moveTo(rw / 2, 0);
  ctx.lineTo(rw / 2 - 8, -6);
  ctx.lineTo(rw / 2 - 8, 6);
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}

// ─── Waypoint Ghosts ─────────────────────────────────────────────────────────

export function drawWaypointGhosts(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath,
  controlPoints: Point[],
  headingWaypoints: HeadingWaypoint[],
  robotLength: number,
  robotWidth: number
): void {
  if (splinePath.totalLength <= 0 || controlPoints.length < 2) return;

  const scale = getScale(cw, transform);
  const rw = robotLength * scale;
  const rh = robotWidth * scale;
  const numCPs = controlPoints.length;

  // Build lookup from waypoint index -> heading in radians
  const headingMap = new Map<number, number>();
  for (const hw of headingWaypoints) {
    headingMap.set(Math.round(hw.waypointIndex), (hw.degrees * Math.PI) / 180);
  }

  for (let i = 0; i < numCPs; i++) {
    // Determine heading: assigned heading if exists, otherwise path tangent
    let rot: number;
    if (headingMap.has(i)) {
      rot = headingMap.get(i)!;
    } else {
      const frac = i / (numCPs - 1);
      const distance = frac * splinePath.totalLength;
      const tan = splinePath.getTangent(distance);
      rot = Math.atan2(tan.y, tan.x);
    }

    const pt = controlPoints[i];
    const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-rot); // Canvas CW, field CCW

    // Semi-transparent dashed robot outline (dimmer than scrubber ghost)
    ctx.fillStyle = 'rgba(0, 255, 170, 0.04)';
    ctx.fillRect(-rw / 2, -rh / 2, rw, rh);

    ctx.strokeStyle = 'rgba(0, 255, 170, 0.18)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    ctx.strokeRect(-rw / 2, -rh / 2, rw, rh);
    ctx.setLineDash([]);

    // Small heading indicator at the front
    ctx.fillStyle = 'rgba(0, 255, 170, 0.25)';
    ctx.beginPath();
    ctx.moveTo(rw / 2, 0);
    ctx.lineTo(rw / 2 - 6, -4);
    ctx.lineTo(rw / 2 - 6, 4);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }
}

// ─── Interpolated Heading Ghosts ─────────────────────────────────────────────

export function drawInterpolatedHeadingGhosts(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath,
  controlPoints: Point[],
  headingWaypoints: HeadingWaypoint[],
  robotLength: number,
  robotWidth: number
): void {
  if (
    headingWaypoints.length < 2 ||
    controlPoints.length < 2 ||
    splinePath.totalLength <= 0
  )
    return;

  const sorted = buildArcLengthHeadings(headingWaypoints, splinePath);
  if (sorted.length < 2) return;

  const scale = getScale(cw, transform);
  const rw = robotLength * scale;
  const rh = robotWidth * scale;

  for (let i = 0; i < sorted.length - 1; i++) {
    const angularDiff = Math.abs(shortestArcDiff(sorted[i].rad, sorted[i + 1].rad));
    if (angularDiff < 0.01) continue;

    const startDist = sorted[i].frac * splinePath.totalLength;
    const endDist = sorted[i + 1].frac * splinePath.totalLength;
    const segLength = endDist - startDist;

    if (segLength < robotLength) continue;

    const numGhosts = Math.max(2, Math.min(5, Math.ceil(segLength / 0.5)));

    for (let j = 1; j <= numGhosts; j++) {
      const t = j / (numGhosts + 1);
      const dist = startDist + t * segLength;
      const progress = dist / splinePath.totalLength;

      const pt = splinePath.getPoint(dist);
      const rot = interpolateHeadingSorted(sorted, progress);
      const { cx, cy } = fieldToCanvas(pt, cw, ch, transform);

      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-rot);

      ctx.fillStyle = 'rgba(0, 255, 170, 0.02)';
      ctx.fillRect(-rw / 2, -rh / 2, rw, rh);

      ctx.strokeStyle = 'rgba(0, 255, 170, 0.10)';
      ctx.lineWidth = 1.0;
      ctx.setLineDash([2, 4]);
      ctx.strokeRect(-rw / 2, -rh / 2, rw, rh);
      ctx.setLineDash([]);

      ctx.fillStyle = 'rgba(0, 255, 170, 0.14)';
      ctx.beginPath();
      ctx.moveTo(rw / 2, 0);
      ctx.lineTo(rw / 2 - 6, -4);
      ctx.lineTo(rw / 2 - 6, 4);
      ctx.closePath();
      ctx.fill();

      ctx.restore();
    }
  }
}

// ─── Minimap ────────────────────────────────────────────────────────────────

export function drawMinimap(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath | null,
  controlPoints: Point[]
): void {
  if (transform.zoom <= 1.5) return;

  const mapW = 200;
  const mapH = Math.round(mapW * (FIELD_HEIGHT / FIELD_WIDTH));
  const margin = 10;
  const mapX = cw - mapW - margin;
  const mapY = ch - mapH - margin;
  const mapScale = mapW / FIELD_WIDTH;

  // Background — darker, neon-bordered
  ctx.save();
  ctx.fillStyle = 'rgba(5, 5, 5, 0.8)';
  ctx.strokeStyle = 'rgba(0, 255, 170, 0.15)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(mapX - 2, mapY - 2, mapW + 4, mapH + 4, 4);
  ctx.fill();
  ctx.stroke();

  // Clip to minimap region
  ctx.beginPath();
  ctx.rect(mapX, mapY, mapW, mapH);
  ctx.clip();

  // Field outline
  ctx.strokeStyle = 'rgba(0, 255, 170, 0.08)';
  ctx.lineWidth = 1;
  ctx.strokeRect(mapX, mapY, mapW, mapH);

  // Helper: field coords to minimap pixel coords
  const toMini = (p: Point): { mx: number; my: number } => ({
    mx: mapX + p.x * mapScale,
    my: mapY + (FIELD_HEIGHT - p.y) * mapScale,
  });

  // Draw simplified path — neon green
  if (splinePath && splinePath.totalLength > 0) {
    const numSamples = 80;
    const ds = splinePath.totalLength / numSamples;

    ctx.strokeStyle = 'rgba(0, 255, 170, 0.5)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const first = toMini(splinePath.getPoint(0));
    ctx.moveTo(first.mx, first.my);
    for (let i = 1; i <= numSamples; i++) {
      const pt = toMini(splinePath.getPoint(i * ds));
      ctx.lineTo(pt.mx, pt.my);
    }
    ctx.stroke();
  }

  // Draw control points as small dots
  controlPoints.forEach((pt, i) => {
    const { mx, my } = toMini(pt);
    ctx.beginPath();
    ctx.arc(mx, my, 2, 0, 2 * Math.PI);
    ctx.fillStyle =
      i === 0
        ? '#00FFaa'
        : i === controlPoints.length - 1
          ? '#FF3366'
          : 'rgba(255, 255, 255, 0.6)';
    ctx.fill();
  });

  // Viewport rectangle
  const viewTopLeft = canvasToField(0, 0, cw, ch, transform);
  const viewBottomRight = canvasToField(cw, ch, cw, ch, transform);
  const vtl = toMini({
    x: Math.max(0, viewTopLeft.x),
    y: Math.min(FIELD_HEIGHT, viewTopLeft.y),
  });
  const vbr = toMini({
    x: Math.min(FIELD_WIDTH, viewBottomRight.x),
    y: Math.max(0, viewBottomRight.y),
  });

  ctx.strokeStyle = 'rgba(0, 255, 170, 0.4)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([]);
  ctx.strokeRect(vtl.mx, vtl.my, vbr.mx - vtl.mx, vbr.my - vtl.my);

  ctx.restore();
}
