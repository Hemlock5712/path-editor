import {
  Route,
  SlidersHorizontal,
  RotateCw,
  FlipHorizontal2,
  TrendingUp,
  Spline,
  Monitor,
} from 'lucide-react';
import { Titlebar } from '../layout/Titlebar';
import { DocsSection, CodeBlock, Callout, Prose, InlineCode } from './DocsSection';

const sections = [
  { id: 'follow-path', label: 'FollowPath Command' },
  { id: 'overrides', label: 'Per-Axis Overrides' },
  { id: 'rotation', label: 'Rotation Suppliers' },
  { id: 'alliance', label: 'Alliance Mirroring' },
  { id: 'velocity-profile', label: 'Velocity Profile' },
  { id: 'spline-path', label: 'SplinePath' },
  { id: 'logging', label: 'AdvantageScope Logging' },
];

function BuilderRow({ method, defaults, desc }: { method: string; defaults: string; desc: string }) {
  return (
    <div className="flex gap-3 py-1.5 border-b border-white/[0.03] last:border-0">
      <code className="text-accent-green text-[10px] font-mono w-56 shrink-0">{method}</code>
      <span className="text-zinc-600 text-[10px] font-mono w-24 shrink-0">{defaults}</span>
      <span className="text-zinc-500 text-[10px] flex-1">{desc}</span>
    </div>
  );
}

function LogRow({ key_, desc }: { key_: string; desc: string }) {
  return (
    <div className="flex gap-3 py-1 border-b border-white/[0.03] last:border-0">
      <code className="text-accent-green text-[10px] font-mono w-56 shrink-0">{key_}</code>
      <span className="text-zinc-500 text-[10px] flex-1">{desc}</span>
    </div>
  );
}

export function RobotIntegrationPage() {
  return (
    <div className="h-screen flex flex-col bg-surface-950">
      <Titlebar />
      <div className="flex-1 overflow-y-auto">
        <div className="max-w-2xl mx-auto px-6 py-8 space-y-5 animate-fadeIn">
          <div className="mb-6">
            <h2 className="text-sm font-light tracking-[0.15em] uppercase text-accent-green/60">
              Robot Integration
            </h2>
            <p className="text-xs text-zinc-500 mt-1">
              Complete reference for the distance-based path following system.
            </p>
          </div>

          {/* Anchor nav */}
          <div className="flex flex-wrap gap-1.5">
            {sections.map((s) => (
              <a
                key={s.id}
                href={`#${s.id}`}
                className="px-2 py-0.5 text-[10px] text-zinc-600 hover:text-accent-green border border-white/[0.04] rounded transition-colors"
              >
                {s.label}
              </a>
            ))}
          </div>

          {/* FollowPath Command */}
          <DocsSection icon={<Route size={13} />} title="FollowPath Command" id="follow-path">
            <Prose>
              <p>
                A distance-based path following command for swerve drive. The path parameter tracks the robot's actual projected position on the spline — if the robot gets hit or stalls, the path "waits" rather than running ahead on a timer.
              </p>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Constructors</div>
              <CodeBlock>
{`// Basic (default constraints)
new FollowPath(drivetrain, spline)

// With constraints
new FollowPath(drivetrain, spline, constraints)

// With constraints and constraint zones
new FollowPath(drivetrain, spline, constraints, constraintZones)`}
              </CodeBlock>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Builder Methods</div>
              <div className="neon-panel p-3 overflow-x-auto">
                <BuilderRow method=".withLookahead(k, min, max)" defaults="0.15, 0.15, 1.0" desc="Adaptive lookahead: distance = k * speed + min, clamped to [min, max]" />
                <BuilderRow method=".withLookaheadMaxArcAngle(rad)" defaults="PI / 6" desc="Caps lookahead on sharp curves so the chord stays close to the arc" />
                <BuilderRow method=".withCrossTrackGains(kp, kd)" defaults="3.0, 0.5" desc="Cross-track PD correction: kp = m/s per meter error, kd = damping" />
                <BuilderRow method=".withCurvatureFeedforward(gain)" defaults="0.1" desc="Proactive push toward center of curvature: gain * v^2 * kappa" />
                <BuilderRow method=".withCompletionTolerance(m)" defaults="0.05" desc="Distance from path end to trigger isFinished()" />
                <BuilderRow method=".withRotationSupplier(supplier)" defaults="hold heading" desc="Rotation strategy (see Rotation Suppliers section)" />
                <BuilderRow method=".withMaxRotationBudget(frac)" defaults="0.30" desc="Max fraction of friction budget reserved for rotation (0 to 1)" />
              </div>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Algorithm Overview</div>
              <ol className="list-decimal list-inside space-y-1 text-zinc-500 text-[11px]">
                <li>Project robot onto path — find closest arc-length position</li>
                <li>Adaptive lookahead — further ahead when moving faster, capped on sharp curves</li>
                <li>Query profiled speed from VelocityProfile at projected arc-length</li>
                <li>Velocity direction toward the lookahead point</li>
                <li>Cross-track PD correction + curvature feedforward</li>
                <li>Apply per-axis overrides (if any)</li>
                <li>Rotation budget allocation — partition friction circle between translation and rotation</li>
                <li>Feed through AccelerationLimiter (motor torque + friction circle enforcement)</li>
              </ol>
              <p className="text-zinc-600 text-[11px]">
                If no rotation supplier is set, the command defaults to holding the robot's current heading at the time <InlineCode>initialize()</InlineCode> runs.
              </p>
            </Prose>
          </DocsSection>

          {/* Per-Axis Speed Overrides */}
          <DocsSection icon={<SlidersHorizontal size={13} />} title="Per-Axis Speed Overrides" id="overrides">
            <Prose>
              <p>
                Override any axis (X, Y, rotation) independently during path following. Each axis has two modes:
              </p>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">With Limits (Recommended)</div>
              <p className="text-zinc-600 text-[11px]">
                The override value still passes through AccelerationLimiter — friction circle, motor torque, and jerk limits all apply.
              </p>
              <CodeBlock>
{`.overrideXSpeedWithLimits(() -> joystick.getLeftX() * 2.0)
.overrideYSpeedWithLimits(() -> joystick.getLeftY() * 2.0)
.overrideRotSpeedWithLimits(() -> visionAlignOmega)`}
              </CodeBlock>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Unlimited</div>
              <p className="text-zinc-600 text-[11px]">
                Bypasses all acceleration limits on the overridden axis. The unlimited axis does not consume friction budget from the remaining limited axes.
              </p>
              <CodeBlock>
{`.overrideXSpeed(() -> rawDriverX)
.overrideYSpeed(() -> rawDriverY)
.overrideRotSpeed(() -> rawDriverOmega)`}
              </CodeBlock>

              <Callout color="amber">
                Unlimited overrides bypass all safety limits. The caller is responsible for not exceeding hardware limits.
              </Callout>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Use Cases</div>
              <ul className="list-disc list-inside space-y-1 text-zinc-500 text-[11px]">
                <li><span className="text-zinc-400">Driver strafe:</span> Override Y axis while path controls X and rotation</li>
                <li><span className="text-zinc-400">Vision alignment:</span> Override rotation with a vision PID supplier while path controls translation</li>
                <li><span className="text-zinc-400">Manual adjustment:</span> Override X with joystick input while approaching a game piece</li>
              </ul>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Friction Budget</div>
              <p className="text-zinc-600 text-[11px]">
                When limited overrides and path-controlled axes coexist, the friction circle is shared between them. Unlimited axes are excluded entirely — they don't reduce the budget available to other axes. This means you can override one axis without slowing down the path-controlled axes.
              </p>
            </Prose>
          </DocsSection>

          {/* Rotation Suppliers */}
          <DocsSection icon={<RotateCw size={13} />} title="Rotation Suppliers" id="rotation">
            <Prose>
              <p>
                The <InlineCode>RotationSupplier</InlineCode> interface receives <InlineCode>(Pose2d robotPose, double pathS, Translation2d pathTangent)</InlineCode> and returns desired angular velocity in rad/s. All built-in strategies use stopping-omega for smooth deceleration.
              </p>

              <div className="space-y-3 mt-3">
                <div className="neon-panel p-3">
                  <div className="text-[11px] text-accent-green/60 mb-1">faceForward()</div>
                  <p className="text-zinc-500 text-[11px] mb-2">Align heading with the path tangent direction.</p>
                  <CodeBlock>{`.withRotationSupplier(RotationSuppliers.faceForward())`}</CodeBlock>
                </div>

                <div className="neon-panel p-3">
                  <div className="text-[11px] text-accent-green/60 mb-1">facePoint(target)</div>
                  <p className="text-zinc-500 text-[11px] mb-2">Aim at a fixed field point while driving (e.g., the speaker).</p>
                  <CodeBlock>
{`.withRotationSupplier(RotationSuppliers.facePoint(
    new Translation2d(4.5, 5.5)))`}
                  </CodeBlock>
                </div>

                <div className="neon-panel p-3">
                  <div className="text-[11px] text-accent-green/60 mb-1">holdHeading(heading)</div>
                  <p className="text-zinc-500 text-[11px] mb-2">Maintain a constant heading throughout the path.</p>
                  <CodeBlock>
{`.withRotationSupplier(RotationSuppliers.holdHeading(
    Rotation2d.fromDegrees(180)))`}
                  </CodeBlock>
                </div>

                <div className="neon-panel p-3">
                  <div className="text-[11px] text-accent-green/60 mb-1">interpolateAlongPath(path, headingWaypoints)</div>
                  <p className="text-zinc-500 text-[11px] mb-2">Smoothly interpolate heading between heading waypoints set in the editor. Uses shortest-angle wrapping. Before the first waypoint, holds the first heading; after the last, holds the last.</p>
                  <CodeBlock>
{`.withRotationSupplier(RotationSuppliers.interpolateAlongPath(
    spline, data.headingWaypoints()))`}
                  </CodeBlock>
                </div>

                <div className="neon-panel p-3">
                  <div className="text-[11px] text-accent-green/60 mb-1">fromZones(path, headingWaypoints, rotationZones)</div>
                  <p className="text-zinc-500 text-[11px] mb-2">
                    The recommended strategy when using the editor. Inside rotation zones, dispatches to <InlineCode>facePoint</InlineCode>. Outside zones, falls back to heading interpolation (or <InlineCode>faceForward</InlineCode> if no heading waypoints are set). This matches the editor's visual heading behavior.
                  </p>
                  <CodeBlock>
{`.withRotationSupplier(RotationSuppliers.fromZones(
    spline, data.headingWaypoints(),
    data.rotationZones()))`}
                  </CodeBlock>
                </div>

                <div className="neon-panel p-3">
                  <div className="text-[11px] text-accent-green/60 mb-1">Custom (DoubleSupplier)</div>
                  <p className="text-zinc-500 text-[11px] mb-2">For custom rotation logic. Wraps a simple lambda into the RotationSupplier interface.</p>
                  <CodeBlock>{`.withRotationSupplier(() -> myPidController.calculate(...))`}</CodeBlock>
                </div>
              </div>
            </Prose>
          </DocsSection>

          {/* Alliance Mirroring */}
          <DocsSection icon={<FlipHorizontal2 size={13} />} title="Alliance Mirroring" id="alliance">
            <Prose>
              <p>
                Paths are authored for the <span className="text-accent-blue">blue alliance</span> in the editor. For rotationally symmetric fields, mirroring is a 180-degree rotation about the field center.
              </p>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">What Gets Mirrored</div>
              <ul className="list-disc list-inside space-y-1 text-zinc-500 text-[11px]">
                <li>Control points: <InlineCode>(FIELD_W - x, FIELD_H - y)</InlineCode></li>
                <li>Heading waypoints: rotated by 180 degrees</li>
                <li>Rotation zone target points: also rotated 180 degrees</li>
                <li>Constraint zones: unchanged (they reference waypoint indices, not field coordinates)</li>
              </ul>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Usage</div>
              <CodeBlock>
{`// Call during autonomousInit() when alliance is known
PathData data = Paths.forAlliance(Paths.MY_PATH);`}
              </CodeBlock>

              <Callout color="amber">
                Alliance may not be known before <InlineCode>autonomousInit()</InlineCode>. Call <InlineCode>forAlliance()</InlineCode> at that point or later.
              </Callout>
            </Prose>
          </DocsSection>

          {/* Velocity Profile */}
          <DocsSection icon={<TrendingUp size={13} />} title="Velocity Profile" id="velocity-profile">
            <Prose>
              <p>
                A forward-backward pass algorithm computes the maximum achievable speed at every point along the path. The profile respects four constraint types simultaneously:
              </p>
              <ol className="list-decimal list-inside space-y-1.5 text-zinc-500 text-[11px] mt-2">
                <li>
                  <span className="text-zinc-400">Curvature limits:</span>{' '}
                  <InlineCode>v_max = sqrt(friction / kappa)</InlineCode> — centripetal acceleration constraint
                </li>
                <li>
                  <span className="text-zinc-400">Motor torque curves:</span> Acceleration decreases with speed due to back-EMF. Uses real dyno data from your motor configuration in Settings.
                </li>
                <li>
                  <span className="text-zinc-400">Friction circle:</span> Tangential + centripetal must fit within the friction budget
                </li>
                <li>
                  <span className="text-zinc-400">Constraint zones:</span> Per-segment velocity and acceleration ceilings (restrictive-only — zones can only lower limits, never raise them)
                </li>
              </ol>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Constraint Zones</div>
              <CodeBlock>
{`new FollowPath(drivetrain, spline, constraints,
    List.of(
        new PathData.ConstraintZone(1, 3, 2.0, 4.0),  // waypoints 1-3: slow
        new PathData.ConstraintZone(5, 6, 1.0, 3.0)   // waypoints 5-6: very slow
    ))`}
              </CodeBlock>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Curvature Smoothing</div>
              <p className="text-zinc-600 text-[11px]">
                A reaction-time lookahead window (0.1s of distance in each direction) takes the maximum curvature, preventing the profile from exploiting curvature drops at Catmull-Rom knots faster than the control loop can react.
              </p>
            </Prose>
          </DocsSection>

          {/* SplinePath */}
          <DocsSection icon={<Spline size={13} />} title="SplinePath" id="spline-path">
            <Prose>
              <p>
                Catmull-Rom cubic spline through all control points, arc-length parameterized via 5-point Gauss-Legendre quadrature with a dense lookup table (~1000 samples/meter).
              </p>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Key Methods</div>
              <div className="neon-panel p-3 overflow-x-auto">
                <BuilderRow method="getPoint(s)" defaults="" desc="Position at arc-length s" />
                <BuilderRow method="getTangent(s)" defaults="" desc="Unit tangent vector at arc-length s" />
                <BuilderRow method="getCurvature(s)" defaults="" desc="Signed curvature (positive = turning left)" />
                <BuilderRow method="getClosestPoint(pos)" defaults="" desc="Projects a field position onto the path — returns arc-length, distance, tangent" />
                <BuilderRow method="getTotalLength()" defaults="" desc="Total arc-length of the spline in meters" />
              </div>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Closest-Point Projection</div>
              <p className="text-zinc-600 text-[11px]">
                Uses a coarse search in a cached window around the last projection, then Newton-Raphson refinement (up to 8 iterations). Amortized O(1) when tracking the path during normal following.
              </p>
            </Prose>
          </DocsSection>

          {/* AdvantageScope Logging */}
          <DocsSection icon={<Monitor size={13} />} title="AdvantageScope Logging" id="logging">
            <Prose>
              <p>
                FollowPath logs tracking data via AdvantageKit Logger for real-time debugging:
              </p>
              <div className="neon-panel p-3 mt-2 overflow-x-auto">
                <LogRow key_="FollowPath/CrossTrackError" desc="Signed perpendicular distance from path (meters)" />
                <LogRow key_="FollowPath/CrossTrackRate" desc="Rate of change of cross-track error" />
                <LogRow key_="FollowPath/ArcLengthS" desc="Current position along path (meters)" />
                <LogRow key_="FollowPath/Progress" desc="0.0 to 1.0 progress fraction" />
                <LogRow key_="FollowPath/ProfiledSpeed" desc="Target speed from velocity profile" />
                <LogRow key_="FollowPath/ActualSpeed" desc="Measured robot speed" />
                <LogRow key_="FollowPath/SpeedError" desc="Actual - profiled speed (tracking error)" />
                <LogRow key_="FollowPath/LookaheadDist" desc="Current adaptive lookahead distance" />
                <LogRow key_="FollowPath/Curvature" desc="Path curvature at projection point" />
                <LogRow key_="FollowPath/CurvatureFeedforward" desc="Curvature feedforward contribution" />
                <LogRow key_="FollowPath/Omega" desc="Commanded angular velocity" />
                <LogRow key_="FollowPath/ReferencePath" desc="Full path as Pose2d[] (trajectory in AdvantageScope)" />
                <LogRow key_="FollowPath/TargetPoint" desc="Lookahead target point on path" />
                <LogRow key_="FollowPath/ClosestPoint" desc="Robot's closest projection on path" />
                <LogRow key_="FollowPath/RobotPose" desc="Current robot pose" />
              </div>
            </Prose>
          </DocsSection>

          <div className="pb-8" />
        </div>
      </div>
    </div>
  );
}
