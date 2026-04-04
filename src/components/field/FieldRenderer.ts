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
  type CanvasTransform,
} from '../../utils/canvasTransform';
import { SplinePath } from '../../math/SplinePath';
import * as Painters from './FieldPainters';

/** Context passed to renderFieldCanvas to avoid a 20+ argument function. */
export interface DrawContext {
  cw: number;
  ch: number;
  transform: CanvasTransform;
  fieldImage: HTMLImageElement | null;
  // Field image crop coordinates
  imgTopLeftX: number;
  imgTopLeftY: number;
  imgFieldW: number;
  imgFieldH: number;
  // Path data
  controlPoints: Point[];
  controlPointRefs: (string | null)[];
  namedPoints: Record<string, NamedPoint>;
  headingWaypoints: HeadingWaypoint[];
  constraintZones: ConstraintZone[];
  rotationZones: RotationZone[];
  splinePath: SplinePath | null;
  inactivePaths: {
    name: string;
    controlPoints: Point[];
    spline: SplinePath | null;
  }[];
  // Selection / interaction
  selectedPointIndex: number | null;
  selectedZoneId: string | null;
  hoveredPointIndex: number | null;
  // Editor settings
  showGrid: boolean;
  showMinimap: boolean;
  showWaypointGhosts: boolean;
  // Scrubber
  scrubberDistance: number;
  scrubberHeading: number | null;
  // Robot dimensions
  robotLength: number;
  robotWidth: number;
}

export function renderFieldCanvas(
  ctx: CanvasRenderingContext2D,
  dc: DrawContext
) {
  const { cw, ch, transform } = dc;

  ctx.clearRect(0, 0, cw, ch);

  // Background
  ctx.fillStyle = '#0a0a0f';
  ctx.fillRect(0, 0, cw, ch);

  // Layer 0: Field image
  if (dc.fieldImage) {
    const topLeft = fieldToCanvas({ x: 0, y: FIELD_HEIGHT }, cw, ch, transform);
    const bottomRight = fieldToCanvas(
      { x: FIELD_WIDTH, y: 0 },
      cw,
      ch,
      transform
    );
    const drawW = bottomRight.cx - topLeft.cx;
    const drawH = bottomRight.cy - topLeft.cy;

    const centerX = topLeft.cx + drawW / 2;
    const centerY = topLeft.cy + drawH / 2;
    ctx.save();
    ctx.translate(centerX, centerY);
    ctx.rotate(Math.PI);
    ctx.drawImage(
      dc.fieldImage,
      dc.imgTopLeftX,
      dc.imgTopLeftY,
      dc.imgFieldW,
      dc.imgFieldH,
      -drawW / 2,
      -drawH / 2,
      drawW,
      drawH
    );
    ctx.restore();
  } else {
    ctx.fillStyle = '#2a2a2a';
    ctx.fillRect(0, 0, cw, ch);
  }

  // Layer 1: Grid
  Painters.drawGrid(ctx, cw, ch, transform, dc.showGrid);

  // Layer 1.5: Named points
  Painters.drawNamedPoints(
    ctx,
    cw,
    ch,
    transform,
    dc.namedPoints,
    dc.controlPointRefs
  );

  // Layer 1.6: Inactive paths
  dc.inactivePaths.forEach((ip, idx) => {
    if (ip.spline) {
      Painters.drawInactivePath(
        ctx,
        cw,
        ch,
        transform,
        ip.spline,
        ip.controlPoints,
        ip.name,
        idx
      );
    }
  });

  // Layer 2: Connection lines
  Painters.drawConnectionLines(ctx, cw, ch, transform, dc.controlPoints);

  // Layer 2.5: Constraint zones
  if (dc.splinePath && dc.constraintZones.length > 0) {
    Painters.drawConstraintZones(
      ctx,
      cw,
      ch,
      transform,
      dc.splinePath,
      dc.constraintZones
    );
  }

  // Layer 3: Spline path
  if (dc.splinePath) {
    Painters.drawPath(ctx, cw, ch, transform, dc.splinePath);
  }

  // Layer 3.5: Rotation zones
  if (dc.splinePath && dc.rotationZones.length > 0) {
    Painters.drawRotationZones(
      ctx,
      cw,
      ch,
      transform,
      dc.splinePath,
      dc.rotationZones,
      dc.selectedZoneId
    );
  }

  // Layer 4: Scrubber ghost
  if (dc.splinePath && dc.scrubberDistance > 0) {
    Painters.drawScrubberGhost(
      ctx,
      cw,
      ch,
      transform,
      dc.splinePath,
      dc.scrubberDistance,
      dc.scrubberHeading,
      dc.robotLength,
      dc.robotWidth
    );
  }

  // Layer 4.5: Waypoint ghosts
  if (dc.splinePath && dc.showWaypointGhosts && dc.controlPoints.length >= 2) {
    Painters.drawWaypointGhosts(
      ctx,
      cw,
      ch,
      transform,
      dc.splinePath,
      dc.controlPoints,
      dc.headingWaypoints,
      dc.robotLength,
      dc.robotWidth
    );
  }

  // Layer 5: Heading arrows
  Painters.drawHeadingArrows(
    ctx,
    cw,
    ch,
    transform,
    dc.controlPoints,
    dc.headingWaypoints
  );

  // Layer 6: Control points
  Painters.drawControlPoints(
    ctx,
    cw,
    ch,
    transform,
    dc.controlPoints,
    dc.controlPointRefs,
    dc.selectedPointIndex,
    dc.hoveredPointIndex
  );

  // Layer 8: Minimap
  if (dc.showMinimap) {
    Painters.drawMinimap(
      ctx,
      cw,
      ch,
      transform,
      dc.splinePath,
      dc.controlPoints
    );
  }
}
