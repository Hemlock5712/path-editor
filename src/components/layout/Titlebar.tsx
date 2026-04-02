import { Timer, PanelRightClose, PanelRightOpen } from 'lucide-react';
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

export function Titlebar({ stats, showSidebar = false }: TitlebarProps) {
  const sidebarCollapsed = useEditorStore((s) => s.sidebarCollapsed);
  const toggleSidebar = useEditorStore((s) => s.toggleSidebar);
  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);
  const location = useLocation();
  const isEditor = location.pathname === '/';

  return (
    <div className="h-10 flex-shrink-0 flex items-center justify-between px-5 bg-[#050505] border-b border-white/[0.04]">
      {/* Left: App title + Nav */}
      <div className="flex items-center gap-5">
        <h1 className="text-xs font-light tracking-[0.2em] uppercase text-zinc-400">
          FRC Path Editor
        </h1>
        <nav className="flex items-center gap-1">
          {navLinks.map((link) => (
            <NavLink
              key={link.to}
              to={link.to}
              className={({ isActive }) =>
                `px-2.5 py-1 text-[11px] font-medium tracking-wide rounded transition-all ${
                  isActive
                    ? 'text-accent-green bg-accent-green/[0.06]'
                    : 'text-zinc-500 hover:text-zinc-300'
                }`
              }
            >
              {link.label}
            </NavLink>
          ))}
        </nav>

        {/* Stats (editor only) */}
        {isEditor && stats && (
          <div className="flex items-center gap-4 text-xs font-mono">
            <span className="text-accent-green neon-glow">{stats.totalLength.toFixed(2)}m</span>
            <span className="flex items-center gap-1.5 text-accent-amber">
              <Timer size={11} />
              {stats.estimatedTime.toFixed(2)}s
            </span>
          </div>
        )}
      </div>

      {/* Center: Scrubber info (editor only) */}
      {isEditor && stats && scrubberDistance > 0 && (
        <div className="text-xs font-mono text-accent-green/50">
          {scrubberDistance.toFixed(2)}m / {stats.totalLength.toFixed(2)}m
        </div>
      )}

      {/* Right: Sidebar toggle (editor only) */}
      <div className="flex items-center gap-3">
        {isEditor && showSidebar && (
          <button onClick={toggleSidebar} className="btn-ghost p-1.5 hover:text-accent-green transition-colors">
            {sidebarCollapsed ? <PanelRightOpen size={15} /> : <PanelRightClose size={15} />}
          </button>
        )}
      </div>
    </div>
  );
}
