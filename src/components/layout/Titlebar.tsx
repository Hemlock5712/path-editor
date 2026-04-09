import { useState, useRef, useEffect } from 'react';
import {
  Timer,
  PanelRightClose,
  PanelRightOpen,
  ChevronDown,
} from 'lucide-react';
import { NavLink, useLocation } from 'react-router-dom';
import { useEditorStore } from '../../stores/editorStore';
import type { PathStats } from '../../math/ProfileAnalytics';

interface TitlebarProps {
  stats?: PathStats | null;
  showSidebar?: boolean;
}

const navLinks = [
  { to: '/', label: 'Editor' },
  { to: '/settings', label: 'Settings' },
  { to: '/downloads', label: 'Downloads' },
];

const docsLinks = [
  { to: '/docs/getting-started', label: 'Getting Started' },
  { to: '/docs/robot-integration', label: 'Robot Integration' },
  { to: '/docs/editor-guide', label: 'Editor Guide' },
];

export function Titlebar({ stats, showSidebar = false }: TitlebarProps) {
  const sidebarCollapsed = useEditorStore((s) => s.sidebarCollapsed);
  const toggleSidebar = useEditorStore((s) => s.toggleSidebar);
  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);
  const location = useLocation();
  const isEditor = location.pathname === '/';
  const isDocsActive = location.pathname.startsWith('/docs');

  const [docsOpen, setDocsOpen] = useState(false);
  const docsRef = useRef<HTMLDivElement>(null);

  // Close dropdown on click outside
  useEffect(() => {
    if (!docsOpen) return;
    const handleClick = (e: MouseEvent) => {
      if (docsRef.current && !docsRef.current.contains(e.target as Node)) {
        setDocsOpen(false);
      }
    };
    document.addEventListener('mousedown', handleClick);
    return () => document.removeEventListener('mousedown', handleClick);
  }, [docsOpen]);

  return (
    <div className="relative flex h-16 flex-shrink-0 items-center justify-between border-b border-white/[0.05] bg-[#050505]/95 px-5 backdrop-blur-sm">
      {/* Left: App title + Nav */}
      <div className="flex min-w-0 items-center gap-5">
        <div className="brand-rail min-w-0 pr-2">
          <div className="flex items-center gap-3">
            <div className="flex h-9 w-9 shrink-0 items-center justify-center rounded-xl border border-white/8 bg-white/[0.02]">
              <div className="grid h-[18px] w-[18px] grid-cols-2 gap-0.5">
                <span className="rounded-[2px] bg-accent-green/80" />
                <span className="rounded-[2px] bg-white/15" />
                <span className="rounded-[2px] bg-white/15" />
                <span className="rounded-[2px] bg-accent-blue/70" />
              </div>
            </div>
            <div className="min-w-0">
              <div className="brand-wordmark truncate text-[13px] font-semibold leading-none">
                FRC Path Editor
              </div>
              <div className="brand-kicker mt-1 text-[11px] leading-none">
                Trajectory Workspace
              </div>
            </div>
          </div>
        </div>

        <nav className="flex items-center gap-1.5">
          {navLinks.map((link) => (
            <NavLink
              key={link.to}
              to={link.to}
              className={({ isActive }) =>
                `rounded-full px-3 py-1.5 text-xs font-medium tracking-[0.05em] transition-all ${
                  isActive
                    ? 'bg-accent-green/[0.08] text-zinc-100 ring-1 ring-accent-green/18'
                    : 'text-zinc-300 hover:bg-white/[0.05] hover:text-zinc-100'
                }`
              }
            >
              {link.label}
            </NavLink>
          ))}

          {/* Docs dropdown */}
          <div ref={docsRef} className="relative">
            <button
              onClick={() => setDocsOpen((prev) => !prev)}
              className={`flex items-center gap-1 rounded-full px-3 py-1.5 text-xs font-medium tracking-[0.05em] transition-all ${
                isDocsActive
                  ? 'bg-accent-green/[0.08] text-zinc-100 ring-1 ring-accent-green/18'
                  : 'text-zinc-300 hover:bg-white/[0.05] hover:text-zinc-100'
              }`}
            >
              Docs
              <ChevronDown
                size={10}
                className={`transition-transform ${docsOpen ? 'rotate-180' : ''}`}
              />
            </button>
            {docsOpen && (
              <div className="animate-fadeIn absolute top-full left-0 z-50 mt-2 min-w-[180px] rounded-xl border border-white/8 bg-zinc-950/95 py-1.5 shadow-2xl backdrop-blur-sm">
                {docsLinks.map((link) => (
                  <NavLink
                    key={link.to}
                    to={link.to}
                    onClick={() => setDocsOpen(false)}
                    className={({ isActive }) =>
                      `block px-3 py-2 text-xs tracking-[0.03em] transition-colors ${
                        isActive
                          ? 'bg-accent-green/[0.06] text-zinc-100'
                          : 'text-zinc-200 hover:bg-white/[0.04] hover:text-accent-green'
                      }`
                    }
                  >
                    {link.label}
                  </NavLink>
                ))}
              </div>
            )}
          </div>
        </nav>

        {/* Stats (editor only) */}
        {isEditor && stats && (
          <div className="flex items-center gap-2 font-mono text-xs">
            <span className="precision-chip text-zinc-300">
              <span className="text-[11px] uppercase tracking-[0.1em] text-zinc-300">
                Distance
              </span>
              <span className="text-accent-green neon-glow">
                {stats.totalLength.toFixed(2)}m
              </span>
            </span>
            <span className="precision-chip text-zinc-300">
              <span className="text-[11px] uppercase tracking-[0.1em] text-zinc-300">
                Time
              </span>
              <span className="text-accent-amber flex items-center gap-1.5">
                <Timer size={11} />
                {stats.estimatedTime.toFixed(2)}s
              </span>
            </span>
          </div>
        )}
      </div>

      {/* Center: Scrubber info (editor only) */}
      {isEditor && stats && scrubberDistance > 0 && (
        <div className="hidden text-sm font-mono tracking-[0.08em] text-zinc-300 uppercase xl:block">
          <span className="mr-2 text-zinc-200">Cursor</span>
          <span className="text-accent-green/70">
            {scrubberDistance.toFixed(2)}m
          </span>
          <span className="mx-2 text-zinc-500">/</span>
          <span className="text-zinc-300">{stats.totalLength.toFixed(2)}m</span>
        </div>
      )}

      {/* Right: Sidebar toggle (editor only) */}
      <div className="flex items-center gap-3">
        {isEditor && showSidebar && (
          <button
            onClick={toggleSidebar}
            className="btn-ghost rounded-full border border-white/[0.05] p-2 transition-colors"
            title={sidebarCollapsed ? 'Show sidebar' : 'Hide sidebar'}
          >
            {sidebarCollapsed ? (
              <PanelRightOpen size={15} />
            ) : (
              <PanelRightClose size={15} />
            )}
          </button>
        )}
      </div>
    </div>
  );
}
