import {
  Point,
  HeadingWaypoint,
  FIELD_WIDTH,
  FIELD_HEIGHT,
} from '../../types';
import {
  fieldToCanvas,
  canvasToField,
  getScale,
  type CanvasTransform,
} from '../../utils/canvasTransform';
import { curvatureToColor } from '../../utils/colors';
import { SplinePath } from '../../math/SplinePath';

// ─── Grid ───────────────────────────────────────────────────────────────────

export function drawGrid(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  showGrid: boolean,
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
    const { cy: cy0 } = fieldToCanvas({ x, y: FIELD_HEIGHT }, cw, ch, transform);
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

// ─── Path ───────────────────────────────────────────────────────────────────

export function drawPath(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath,
): void {
  if (splinePath.totalLength <= 0) return;

  const scale = getScale(cw, transform);
  // More samples at higher zoom for smoother curves
  const numSamples = Math.max(300, Math.floor(transform.zoom * 300));
  const ds = splinePath.totalLength / numSamples;
  // Scale line width inversely so it doesn't get too thick when zoomed or too thin when zoomed out
  const lineWidth = Math.max(1.5, Math.min(4, 3 / Math.sqrt(transform.zoom)));

  // ── BLOOM PASS: wider, dimmer neon green glow behind the path ──
  ctx.save();
  ctx.globalAlpha = 0.15;
  ctx.strokeStyle = '#00FFaa';
  ctx.lineWidth = lineWidth * 4;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  const bloomStart = fieldToCanvas(splinePath.getPoint(0), cw, ch, transform);
  ctx.moveTo(bloomStart.cx, bloomStart.cy);
  for (let i = 1; i <= numSamples; i++) {
    const pt = fieldToCanvas(splinePath.getPoint(i * ds), cw, ch, transform);
    ctx.lineTo(pt.cx, pt.cy);
  }
  ctx.stroke();
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
    ctx.moveTo(cx + Math.cos(angle) * arrowSize, cy + Math.sin(angle) * arrowSize);
    ctx.lineTo(
      cx + Math.cos(angle + 2.5) * arrowSize * 0.7,
      cy + Math.sin(angle + 2.5) * arrowSize * 0.7,
    );
    ctx.lineTo(
      cx + Math.cos(angle - 2.5) * arrowSize * 0.7,
      cy + Math.sin(angle - 2.5) * arrowSize * 0.7,
    );
    ctx.closePath();
    ctx.fill();
  }
}

// ─── Control Points ─────────────────────────────────────────────────────────

export function drawControlPoints(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  controlPoints: Point[],
  selectedIndex: number | null,
  hoveredIndex: number | null,
): void {
  if (controlPoints.length === 0) return;

  const scale = getScale(cw, transform);
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

    ctx.save();
    if (isFirst) {
      // Neon green diamond for first point
      ctx.fillStyle = '#00FFaa';
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
      // Neon pink-red square for last point
      ctx.fillStyle = '#FF3366';
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.fillRect(cx - radius, cy - radius, radius * 2, radius * 2);
      ctx.strokeRect(cx - radius, cy - radius, radius * 2, radius * 2);
    } else {
      // Subtle white circle for interior points
      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
      ctx.fillStyle = isHovered ? 'rgba(255, 255, 255, 0.9)' : 'rgba(255, 255, 255, 0.7)';
      ctx.fill();
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.4)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    ctx.restore();

    // Index label
    ctx.fillStyle = isFirst || isLast ? '#000' : '#000';
    ctx.font = `bold ${fontSize}px monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(i), cx, cy);
  });
}

// ─── Heading Arrows ─────────────────────────────────────────────────────────

export function drawHeadingArrows(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  controlPoints: Point[],
  headingWaypoints: HeadingWaypoint[],
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

    ctx.save();
    ctx.strokeStyle = '#00DDFF';
    ctx.lineWidth = 2.5;
    ctx.shadowColor = '#00DDFF';
    ctx.shadowBlur = 6;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();

    // Arrowhead
    const angle = Math.atan2(tipY - cy, tipX - cx);
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(tipX - headLen * Math.cos(angle - 0.4), tipY - headLen * Math.sin(angle - 0.4));
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(tipX - headLen * Math.cos(angle + 0.4), tipY - headLen * Math.sin(angle + 0.4));
    ctx.stroke();
    ctx.restore();

    // Degree label
    ctx.fillStyle = '#00DDFF';
    ctx.font = 'bold 10px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`${hw.degrees.toFixed(0)}`, tipX, tipY - 4);
  });
}

// ─── Connection Lines ───────────────────────────────────────────────────────

export function drawConnectionLines(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  controlPoints: Point[],
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
  robotWidth: number,
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

  // Neon green robot outline
  ctx.fillStyle = 'rgba(0, 255, 170, 0.08)';
  ctx.fillRect(-rw / 2, -rh / 2, rw, rh);

  ctx.strokeStyle = 'rgba(0, 255, 170, 0.35)';
  ctx.lineWidth = 2;
  ctx.shadowColor = '#00FFaa';
  ctx.shadowBlur = 8;
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
  robotWidth: number,
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

// ─── Minimap ────────────────────────────────────────────────────────────────

export function drawMinimap(
  ctx: CanvasRenderingContext2D,
  cw: number,
  ch: number,
  transform: CanvasTransform,
  splinePath: SplinePath | null,
  controlPoints: Point[],
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
    ctx.fillStyle = i === 0 ? '#00FFaa' : i === controlPoints.length - 1 ? '#FF3366' : 'rgba(255, 255, 255, 0.6)';
    ctx.fill();
  });

  // Viewport rectangle
  const viewTopLeft = canvasToField(0, 0, cw, ch, transform);
  const viewBottomRight = canvasToField(cw, ch, cw, ch, transform);
  const vtl = toMini({ x: Math.max(0, viewTopLeft.x), y: Math.min(FIELD_HEIGHT, viewTopLeft.y) });
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
