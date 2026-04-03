import {
  MousePointer2,
  Gauge,
  Crosshair,
  Compass,
  BarChart3,
  Play,
  Save,
  Keyboard,
} from 'lucide-react';
import { Titlebar } from '../layout/Titlebar';
import { DocsSection, CodeBlock, Prose, InlineCode, Callout } from './DocsSection';
import { Link } from 'react-router-dom';

function ShortcutRow({ keys, desc }: { keys: string; desc: string }) {
  return (
    <div className="flex gap-3 py-1 border-b border-white/[0.03] last:border-0">
      <code className="text-accent-green text-[10px] font-mono w-40 shrink-0">{keys}</code>
      <span className="text-zinc-500 text-[10px] flex-1">{desc}</span>
    </div>
  );
}

export function EditorGuidePage() {
  return (
    <div className="h-screen flex flex-col bg-surface-950">
      <Titlebar />
      <div className="flex-1 overflow-y-auto">
        <div className="max-w-2xl mx-auto px-6 py-8 space-y-5 animate-fadeIn">
          <div className="mb-6">
            <h2 className="text-sm font-light tracking-[0.15em] uppercase text-accent-green/60">
              Editor Guide
            </h2>
            <p className="text-xs text-zinc-500 mt-1">
              How to design, visualize, and export paths using the web editor.
            </p>
          </div>

          {/* Canvas Interaction */}
          <DocsSection icon={<MousePointer2 size={13} />} title="Canvas Interaction">
            <Prose>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px]">
                <li><span className="text-zinc-400">Click</span> on the field to add a control point</li>
                <li><span className="text-zinc-400">Click and drag</span> an existing point to move it</li>
                <li><span className="text-zinc-400">Right-click</span> on a path segment to insert a point between two existing points</li>
                <li><span className="text-zinc-400">Arrow keys</span> nudge the selected point by the grid size (default 0.05m)</li>
                <li><span className="text-zinc-400">Shift + Arrow keys</span> fine-nudge by 0.01m</li>
                <li><span className="text-zinc-400">Delete / Backspace</span> removes the selected point (minimum 2 points)</li>
                <li><span className="text-zinc-400">Scroll</span> to zoom in/out</li>
                <li><span className="text-zinc-400">Middle-click drag</span> or drag on empty space to pan</li>
                <li>Press <InlineCode>0</InlineCode> to reset the view</li>
                <li>Press <InlineCode>G</InlineCode> to toggle snap-to-grid</li>
                <li>Press <InlineCode>M</InlineCode> to toggle the minimap (appears when zoomed in)</li>
              </ul>
            </Prose>
          </DocsSection>

          {/* Multi-Path Editing */}
          <DocsSection icon={<Save size={13} />} title="Multi-Path Editing">
            <Prose>
              <p>
                The editor supports multiple paths in a single <InlineCode>Paths.java</InlineCode> file. Each path appears as a tab above the canvas.
              </p>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px] mt-2">
                <li>Click the <InlineCode>+</InlineCode> button to add a new path</li>
                <li>Click a tab to switch between paths</li>
                <li>Double-click a tab to rename a path</li>
                <li>Right-click a tab for rename/delete options</li>
                <li>Each path has its own control points, constraints, heading waypoints, and zones</li>
              </ul>
              <p className="text-zinc-600 text-[11px] mt-2">
                When you save, all paths are exported into a single <InlineCode>Paths.java</InlineCode> file with one <InlineCode>PathData</InlineCode> constant per path. Path names become UPPER_SNAKE_CASE constant names.
              </p>
            </Prose>
          </DocsSection>

          {/* Constraint Zones */}
          <DocsSection icon={<Gauge size={13} />} title="Constraint Zones">
            <Prose>
              <p>
                Regional velocity and acceleration limits applied to segments of the path. Useful for slowing down through tight sections or near game pieces.
              </p>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px] mt-2">
                <li>In the sidebar under "Constraint Zones", click <span className="text-zinc-400">Add Zone</span></li>
                <li>Set the start and end waypoint indices</li>
                <li>Set max velocity (m/s) and max acceleration (m/s²)</li>
                <li>The velocity profile recomputes in real time — watch the velocity chart to see the effect</li>
                <li>Zones appear as amber overlays on the path canvas</li>
              </ul>
              <Callout color="amber">
                Constraint zones are restrictive-only. They can lower velocity and acceleration below the global limits, but never raise them.
              </Callout>
            </Prose>
          </DocsSection>

          {/* Rotation Zones */}
          <DocsSection icon={<Crosshair size={13} />} title="Rotation Zones">
            <Prose>
              <p>
                Regions where the robot faces a specific field point instead of following heading interpolation. Used for face-point targeting (e.g., aiming at the speaker during approach).
              </p>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px] mt-2">
                <li>In the sidebar under "Rotation Zones", click <span className="text-zinc-400">Add Face-Point Zone</span></li>
                <li>Set start/end waypoint indices (fractional allowed, e.g., 1.5 = halfway between waypoint 1 and 2)</li>
                <li>Set the target point (x, y) the robot should face</li>
                <li>Zones appear as orange overlays with a crosshair at the target point</li>
                <li>Click a zone to select and highlight it</li>
              </ul>
              <p className="text-zinc-600 text-[11px] mt-2">
                On the robot side, rotation zones map to{' '}
                <InlineCode>RotationSuppliers.fromZones()</InlineCode>. See the{' '}
                <Link to="/docs/robot-integration#rotation" className="text-accent-green hover:underline">
                  Rotation Suppliers
                </Link>{' '}
                section for details.
              </p>
            </Prose>
          </DocsSection>

          {/* Heading Waypoints */}
          <DocsSection icon={<Compass size={13} />} title="Heading Waypoints">
            <Prose>
              <p>
                Per-waypoint heading targets that control the robot's orientation along the path.
              </p>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px] mt-2">
                <li>Select a control point, then set a heading (in degrees) in the sidebar heading editor</li>
                <li>The compass preview shows the heading needle</li>
                <li>Clear the heading field to remove a heading from a waypoint</li>
                <li>Between waypoints, headings are smoothly interpolated using shortest-angle wrapping</li>
                <li>Heading arrows appear as cyan indicators on the canvas</li>
              </ul>
              <p className="text-zinc-600 text-[11px] mt-2">
                On the robot side, heading waypoints map to <InlineCode>PathData.HeadingWaypoint</InlineCode> and{' '}
                <InlineCode>RotationSuppliers.interpolateAlongPath()</InlineCode>.
              </p>
            </Prose>
          </DocsSection>

          {/* Charts & Visualization */}
          <DocsSection icon={<BarChart3 size={13} />} title="Charts & Visualization">
            <Prose>
              <p>The bottom panel shows real-time charts computed from the path and velocity profile:</p>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px] mt-2">
                <li><span className="text-zinc-400">Velocity vs Distance</span> — the velocity profile curve with constraint zone shading</li>
                <li><span className="text-zinc-400">Velocity vs Time</span> — same data mapped to the time domain</li>
                <li><span className="text-zinc-400">Acceleration vs Distance</span> — tangential acceleration along the path</li>
                <li><span className="text-zinc-400">Curvature vs Distance</span> — signed curvature showing sharp turns</li>
                <li><span className="text-zinc-400">Heading vs Distance</span> — heading interpolation curve</li>
              </ul>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Path Coloring</div>
              <p className="text-zinc-500 text-[11px]">
                The path on the canvas is colored by curvature magnitude:{' '}
                <span className="text-accent-blue">blue</span> (low) to{' '}
                <span className="text-accent-cyan">cyan</span> to{' '}
                <span className="text-accent-green">green</span> to{' '}
                <span className="text-accent-amber">yellow</span> to{' '}
                <span className="text-accent-red">red</span> (high curvature).
              </p>

              <div className="text-[11px] text-zinc-300 font-medium mt-3 mb-1">Robot Ghosts</div>
              <p className="text-zinc-500 text-[11px]">
                Toggle the box icon in the toolbar to show/hide robot outlines at each waypoint. Robot dimensions come from the{' '}
                <Link to="/settings" className="text-accent-green hover:underline">Settings</Link> page.
              </p>
            </Prose>
          </DocsSection>

          {/* Playback */}
          <DocsSection icon={<Play size={13} />} title="Playback">
            <Prose>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px]">
                <li>Use the Play/Pause/Stop controls in the toolbar</li>
                <li>During playback, a green robot ghost shows the simulated position along the path</li>
                <li>The titlebar shows the current scrubber position and total path length</li>
                <li>Press <InlineCode>Space</InlineCode> to play/stop</li>
                <li>Press <InlineCode>.</InlineCode> (period) to step forward</li>
                <li>Press <InlineCode>,</InlineCode> (comma) to step backward</li>
              </ul>
            </Prose>
          </DocsSection>

          {/* File Operations */}
          <DocsSection icon={<Save size={13} />} title="File Operations">
            <Prose>
              <ul className="list-disc list-inside space-y-1.5 text-zinc-500 text-[11px]">
                <li><span className="text-zinc-400">Save Java</span> (<InlineCode>Ctrl+S</InlineCode>) — Exports all paths as a <InlineCode>Paths.java</InlineCode> file</li>
                <li><span className="text-zinc-400">Load Java</span> (<InlineCode>Ctrl+O</InlineCode>) — Loads a previously saved <InlineCode>Paths.java</InlineCode> file back into the editor</li>
              </ul>
              <Callout color="green">
                Java export is the recommended workflow for competition. The generated <InlineCode>Paths.java</InlineCode> is a complete, compilable source file with all your paths as constants. See{' '}
                <Link to="/docs/getting-started" className="text-accent-green hover:underline">Getting Started</Link> for details.
              </Callout>
            </Prose>
          </DocsSection>

          {/* Keyboard Shortcuts */}
          <DocsSection icon={<Keyboard size={13} />} title="Keyboard Shortcuts">
            <div className="neon-panel p-3 overflow-x-auto">
              <ShortcutRow keys="Ctrl+S" desc="Save Paths.java" />
              <ShortcutRow keys="Ctrl+O" desc="Load Paths.java" />
              <ShortcutRow keys="Ctrl+Z" desc="Undo" />
              <ShortcutRow keys="Ctrl+Shift+Z / Ctrl+Y" desc="Redo" />
              <ShortcutRow keys="Delete / Backspace" desc="Delete selected point" />
              <ShortcutRow keys="Escape" desc="Deselect point" />
              <ShortcutRow keys="Space" desc="Play / Stop playback" />
              <ShortcutRow keys=". (period)" desc="Step forward" />
              <ShortcutRow keys=", (comma)" desc="Step backward" />
              <ShortcutRow keys="Arrow keys" desc="Nudge selected point (grid size or 0.05m)" />
              <ShortcutRow keys="Shift + Arrow keys" desc="Fine nudge (0.01m)" />
              <ShortcutRow keys="G" desc="Toggle snap to grid" />
              <ShortcutRow keys="M" desc="Toggle minimap" />
              <ShortcutRow keys="+ / -" desc="Zoom in / out" />
              <ShortcutRow keys="0" desc="Reset view" />
            </div>
          </DocsSection>

          <div className="pb-8" />
        </div>
      </div>
    </div>
  );
}
