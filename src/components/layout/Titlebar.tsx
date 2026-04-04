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
    <div className="flex h-10 flex-shrink-0 items-center justify-between border-b border-white/[0.04] bg-[#050505] px-5">
      {/* Left: App title + Nav */}
      <div className="flex items-center gap-5">
        <h1 className="text-xs font-light tracking-[0.2em] text-zinc-400 uppercase">
          FRC Path Editor
        </h1>
        <nav className="flex items-center gap-1">
          {navLinks.map((link) => (
            <NavLink
              key={link.to}
              to={link.to}
              className={({ isActive }) =>
                `rounded px-2.5 py-1 text-[11px] font-medium tracking-wide transition-all ${
                  isActive
                    ? 'text-accent-green bg-accent-green/[0.06]'
                    : 'text-zinc-500 hover:text-zinc-300'
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
              className={`flex items-center gap-1 rounded px-2.5 py-1 text-[11px] font-medium tracking-wide transition-all ${
                isDocsActive
                  ? 'text-accent-green bg-accent-green/[0.06]'
                  : 'text-zinc-500 hover:text-zinc-300'
              }`}
            >
              Docs
              <ChevronDown
                size={10}
                className={`transition-transform ${docsOpen ? 'rotate-180' : ''}`}
              />
            </button>
            {docsOpen && (
              <div className="border-accent-green/10 shadow-accent-green/5 animate-fadeIn absolute top-full left-0 z-50 mt-1 min-w-[160px] rounded-lg border bg-zinc-950/95 py-1 shadow-lg backdrop-blur-sm">
                {docsLinks.map((link) => (
                  <NavLink
                    key={link.to}
                    to={link.to}
                    onClick={() => setDocsOpen(false)}
                    className={({ isActive }) =>
                      `block px-3 py-1.5 text-[11px] transition-colors ${
                        isActive
                          ? 'text-accent-green bg-accent-green/[0.06]'
                          : 'hover:text-accent-green hover:bg-accent-green/[0.04] text-zinc-400'
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
          <div className="flex items-center gap-4 font-mono text-xs">
            <span className="text-accent-green neon-glow">
              {stats.totalLength.toFixed(2)}m
            </span>
            <span className="text-accent-amber flex items-center gap-1.5">
              <Timer size={11} />
              {stats.estimatedTime.toFixed(2)}s
            </span>
          </div>
        )}
      </div>

      {/* Center: Scrubber info (editor only) */}
      {isEditor && stats && scrubberDistance > 0 && (
        <div className="text-accent-green/50 font-mono text-xs">
          {scrubberDistance.toFixed(2)}m / {stats.totalLength.toFixed(2)}m
        </div>
      )}

      {/* Right: Sidebar toggle (editor only) */}
      <div className="flex items-center gap-3">
        {isEditor && showSidebar && (
          <button
            onClick={toggleSidebar}
            className="btn-ghost hover:text-accent-green p-1.5 transition-colors"
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
