import { useState, useCallback, type ReactNode } from 'react';
import { useEditorStore } from '../../stores/editorStore';

interface AppShellProps {
  titlebar: ReactNode;
  field: ReactNode;
  sidebar: ReactNode;
  bottomPanel: ReactNode;
}

export function AppShell({
  titlebar,
  field,
  sidebar,
  bottomPanel,
}: AppShellProps) {
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
    [bottomPanelHeight, setBottomPanelHeight]
  );

  return (
    <div className="flex h-screen flex-col overflow-hidden bg-[#050505] text-zinc-100">
      {/* Titlebar */}
      {titlebar}

      {/* Main content area */}
      <div className="flex min-h-0 flex-1 overflow-hidden">
        {/* Field + bottom panel */}
        <div className="flex min-w-0 flex-1 flex-col">
          {/* Field canvas area */}
          <div className="relative min-h-0 flex-1">{field}</div>

          {/* Resize handle — neon glow line */}
          <div
            className={`group flex h-1 cursor-row-resize items-center justify-center transition-all duration-300 ${resizing ? 'bg-accent-green/10' : 'bg-transparent'}`}
            onMouseDown={handleResizeStart}
          >
            <div
              className={`h-px w-12 rounded-full transition-all duration-300 ${resizing ? 'bg-accent-green/80 shadow-[0_0_8px_rgba(0,255,170,0.4)]' : 'group-hover:bg-accent-green/40 bg-white/[0.06] group-hover:shadow-[0_0_6px_rgba(0,255,170,0.2)]'}`}
            />
          </div>

          {/* Bottom panel (charts) */}
          <div
            style={{ height: bottomPanelHeight }}
            className="flex-shrink-0 overflow-hidden"
          >
            {bottomPanel}
          </div>
        </div>

        {/* Sidebar */}
        <div
          className={`bg-surface-950 flex-shrink-0 overflow-hidden border-l border-white/[0.04] transition-all duration-300 ${sidebarCollapsed ? 'w-0' : 'w-80'}`}
        >
          <div className="h-full w-80 space-y-3 overflow-y-auto p-3">
            {sidebar}
          </div>
        </div>
      </div>
    </div>
  );
}
