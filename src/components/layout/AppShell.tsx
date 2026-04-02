import { useRef, useState, useCallback, type ReactNode } from 'react';
import { useEditorStore } from '../../stores/editorStore';

interface AppShellProps {
  titlebar: ReactNode;
  field: ReactNode;
  sidebar: ReactNode;
  bottomPanel: ReactNode;
}

export function AppShell({ titlebar, field, sidebar, bottomPanel }: AppShellProps) {
  const sidebarCollapsed = useEditorStore((s) => s.sidebarCollapsed);
  const bottomPanelHeight = useEditorStore((s) => s.bottomPanelHeight);
  const setBottomPanelHeight = useEditorStore((s) => s.setBottomPanelHeight);
  const [resizing, setResizing] = useState(false);

  const handleResizeStart = useCallback(
    (e: React.MouseEvent) => {
      e.preventDefault();
      setResizing(true);
      const startY = e.clientY;
      const startHeight = bottomPanelHeight;

      const handleMove = (moveE: MouseEvent) => {
        const delta = startY - moveE.clientY;
        setBottomPanelHeight(Math.max(120, Math.min(400, startHeight + delta)));
      };

      const handleUp = () => {
        setResizing(false);
        document.removeEventListener('mousemove', handleMove);
        document.removeEventListener('mouseup', handleUp);
      };

      document.addEventListener('mousemove', handleMove);
      document.addEventListener('mouseup', handleUp);
    },
    [bottomPanelHeight, setBottomPanelHeight],
  );

  return (
    <div className="flex flex-col h-screen bg-[#050505] text-zinc-100 overflow-hidden">
      {/* Titlebar */}
      {titlebar}

      {/* Main content area */}
      <div className="flex flex-1 min-h-0 overflow-hidden">
        {/* Field + bottom panel */}
        <div className="flex flex-col flex-1 min-w-0">
          {/* Field canvas area */}
          <div className="flex-1 min-h-0 relative">{field}</div>

          {/* Resize handle — neon glow line */}
          <div
            className={`h-1 cursor-row-resize flex items-center justify-center group transition-all duration-300 ${resizing ? 'bg-accent-green/10' : 'bg-transparent'}`}
            onMouseDown={handleResizeStart}
          >
            <div className={`w-12 h-px rounded-full transition-all duration-300 ${resizing ? 'bg-accent-green/80 shadow-[0_0_8px_rgba(0,255,170,0.4)]' : 'bg-white/[0.06] group-hover:bg-accent-green/40 group-hover:shadow-[0_0_6px_rgba(0,255,170,0.2)]'}`} />
          </div>

          {/* Bottom panel (charts) */}
          <div style={{ height: bottomPanelHeight }} className="flex-shrink-0 overflow-hidden">
            {bottomPanel}
          </div>
        </div>

        {/* Sidebar */}
        <div
          className={`flex-shrink-0 border-l border-white/[0.04] bg-surface-950 transition-all duration-300 overflow-hidden ${sidebarCollapsed ? 'w-0' : 'w-80'}`}
        >
          <div className="w-80 h-full overflow-y-auto p-3 space-y-3">{sidebar}</div>
        </div>
      </div>
    </div>
  );
}
