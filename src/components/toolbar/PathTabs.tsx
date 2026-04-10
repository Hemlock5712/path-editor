import { useState, useRef, useEffect, useCallback } from 'react';
import { Plus, Eye, EyeOff } from 'lucide-react';
import { usePathStore } from '../../stores/pathStore';
import { useEditorStore } from '../../stores/editorStore';

export function PathTabs() {
  const paths = usePathStore((s) => s.paths);
  const pathOrder = usePathStore((s) => s.pathOrder);
  const activePathName = usePathStore((s) => s.activePathName);
  const setActivePath = usePathStore((s) => s.setActivePath);
  const addPath = usePathStore((s) => s.addPath);
  const renamePath = usePathStore((s) => s.renamePath);
  const deletePath = usePathStore((s) => s.deletePath);
  const duplicatePath = usePathStore((s) => s.duplicatePath);
  const reorderPath = usePathStore((s) => s.reorderPath);

  const hiddenPaths = useEditorStore((s) => s.hiddenPaths);
  const togglePathVisibility = useEditorStore((s) => s.togglePathVisibility);
  const showAllPaths = useEditorStore((s) => s.showAllPaths);
  const hideAllInactivePaths = useEditorStore((s) => s.hideAllInactivePaths);

  const [editingName, setEditingName] = useState<string | null>(null);
  const [editValue, setEditValue] = useState('');
  const [contextMenu, setContextMenu] = useState<{
    name: string;
    x: number;
    y: number;
  } | null>(null);
  const [dragIndex, setDragIndex] = useState<number | null>(null);
  const [dropIndex, setDropIndex] = useState<number | null>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (editingName && inputRef.current) {
      inputRef.current.focus();
      inputRef.current.select();
    }
  }, [editingName]);

  // Close context menu on click outside or Escape
  useEffect(() => {
    if (!contextMenu) return;
    const handleClose = (e: MouseEvent | KeyboardEvent) => {
      if (e instanceof KeyboardEvent && e.key !== 'Escape') return;
      setContextMenu(null);
    };
    document.addEventListener('mousedown', handleClose);
    document.addEventListener('keydown', handleClose);
    return () => {
      document.removeEventListener('mousedown', handleClose);
      document.removeEventListener('keydown', handleClose);
    };
  }, [contextMenu]);

  const startRename = useCallback((name: string) => {
    setEditingName(name);
    setEditValue(name);
    setContextMenu(null);
  }, []);

  const commitRename = useCallback(() => {
    if (editingName && editValue.trim() && editValue.trim() !== editingName) {
      renamePath(editingName, editValue.trim());
    }
    setEditingName(null);
  }, [editingName, editValue, renamePath]);

  const handleAdd = useCallback(() => {
    const base = 'Path';
    let n = pathOrder.length + 1;
    let name = `${base} ${n}`;
    while (paths[name]) {
      n++;
      name = `${base} ${n}`;
    }
    addPath(name);
  }, [pathOrder.length, paths, addPath]);

  const handleDragStart = useCallback((e: React.DragEvent, index: number) => {
    setDragIndex(index);
    e.dataTransfer.effectAllowed = 'move';
    e.dataTransfer.setData('text/plain', String(index));
  }, []);

  const handleDragOver = useCallback(
    (e: React.DragEvent, index: number) => {
      e.preventDefault();
      e.dataTransfer.dropEffect = 'move';
      if (dragIndex !== null && index !== dragIndex) {
        setDropIndex(index);
      }
    },
    [dragIndex]
  );

  const handleDrop = useCallback(
    (e: React.DragEvent, toIndex: number) => {
      e.preventDefault();
      setDragIndex((current) => {
        if (current !== null && current !== toIndex) {
          reorderPath(current, toIndex);
        }
        return null;
      });
      setDropIndex(null);
    },
    [reorderPath]
  );

  const handleDragEnd = useCallback(() => {
    setDragIndex(null);
    setDropIndex(null);
  }, []);

  const handleActivate = useCallback(
    (name: string) => {
      if (hiddenPaths.includes(name)) {
        togglePathVisibility(name);
      }
      setActivePath(name);
    },
    [setActivePath, hiddenPaths, togglePathVisibility]
  );

  return (
    <div className="flex flex-shrink-0 items-center gap-1 overflow-x-auto border-b border-white/[0.04] px-3 pt-2 pb-1.5">
      {pathOrder.map((name, index) => (
        <div
          key={name}
          className={`relative flex-shrink-0 ${
            dropIndex === index && dragIndex !== null && dragIndex !== index
              ? dragIndex > index
                ? 'before:bg-accent-green before:absolute before:top-1 before:bottom-1 before:left-0 before:w-0.5 before:rounded-full'
                : 'after:bg-accent-green after:absolute after:top-1 after:right-0 after:bottom-1 after:w-0.5 after:rounded-full'
              : ''
          }`}
          draggable={editingName !== name}
          onDragStart={(e) => handleDragStart(e, index)}
          onDragOver={(e) => handleDragOver(e, index)}
          onDrop={(e) => handleDrop(e, index)}
          onDragEnd={handleDragEnd}
        >
          {editingName === name ? (
            <input
              ref={inputRef}
              value={editValue}
              onChange={(e) => setEditValue(e.target.value)}
              onBlur={commitRename}
              onKeyDown={(e) => {
                if (e.key === 'Enter') commitRename();
                if (e.key === 'Escape') setEditingName(null);
              }}
              className="text-accent-green border-accent-green/40 w-28 rounded border bg-zinc-900 px-2.5 py-1 text-xs font-medium outline-none"
            />
          ) : (
            <div
              className={`flex items-center rounded-full border transition-all duration-200 ${
                activePathName === name
                  ? 'border-accent-green/20 bg-accent-green/[0.08] text-zinc-100 shadow-[0_1px_8px_rgba(0,255,170,0.14)]'
                  : hiddenPaths.includes(name)
                    ? 'border-white/[0.04] bg-white/[0.01] text-zinc-500'
                    : 'border-white/[0.07] bg-white/[0.02] text-zinc-300 hover:bg-white/[0.05] hover:text-zinc-100'
              } ${dragIndex === index ? 'opacity-40' : ''}`}
            >
              <button
                onClick={() => handleActivate(name)}
                onDoubleClick={() => startRename(name)}
                onContextMenu={(e) => {
                  e.preventDefault();
                  setContextMenu({ name, x: e.clientX, y: e.clientY });
                }}
                className="cursor-grab px-3 py-1.5 text-sm font-medium tracking-[0.04em] active:cursor-grabbing"
              >
                <span className="mr-2 font-mono text-[11px] text-zinc-400">
                  {(index + 1).toString().padStart(2, '0')}
                </span>
                {name}
              </button>
              {activePathName !== name && (
                <button
                  onClick={() => togglePathVisibility(name)}
                  className="mr-1.5 rounded-full p-1 opacity-40 hover:opacity-100 transition-opacity"
                  title={hiddenPaths.includes(name) ? 'Show path' : 'Hide path'}
                >
                  {hiddenPaths.includes(name) ? <EyeOff size={11} /> : <Eye size={11} />}
                </button>
              )}
            </div>
          )}
        </div>
      ))}

      {/* Add path button */}
      <button
        onClick={handleAdd}
        className="btn-ghost ml-1 rounded-full border border-dashed border-white/[0.08] p-2 text-zinc-300 transition-colors hover:border-accent-green/20 hover:text-accent-green"
        title="New path"
      >
        <Plus size={14} />
      </button>

      {/* Bulk visibility controls */}
      {pathOrder.length >= 2 && (
        <div className="ml-1 flex items-center gap-0.5">
          <button
            onClick={showAllPaths}
            className="rounded-full p-1.5 text-zinc-400 transition-colors hover:text-zinc-100 disabled:opacity-30"
            title="Show all paths"
            disabled={hiddenPaths.length === 0}
          >
            <Eye size={13} />
          </button>
          <button
            onClick={hideAllInactivePaths}
            className="rounded-full p-1.5 text-zinc-400 transition-colors hover:text-zinc-100"
            title="Hide all other paths"
          >
            <EyeOff size={13} />
          </button>
        </div>
      )}

      {/* Context menu */}
      {contextMenu && (
        <div
          className="fixed z-50 min-w-[140px] rounded-xl border border-white/[0.08] bg-zinc-950/95 py-1.5 shadow-2xl backdrop-blur-sm"
          style={{ left: contextMenu.x, top: contextMenu.y }}
          onMouseDown={(e) => e.stopPropagation()}
        >
          <button
            className="hover:bg-accent-green/10 hover:text-accent-green w-full px-3 py-2 text-left text-xs tracking-[0.05em] text-zinc-300 transition-colors"
            onClick={() => startRename(contextMenu.name)}
          >
            Rename
          </button>
          {contextMenu.name !== activePathName && (
            <button
              className="hover:bg-accent-green/10 hover:text-accent-green w-full px-3 py-2 text-left text-xs tracking-[0.05em] text-zinc-300 transition-colors"
              onClick={() => {
                togglePathVisibility(contextMenu.name);
                setContextMenu(null);
              }}
            >
              {hiddenPaths.includes(contextMenu.name) ? 'Show' : 'Hide'}
            </button>
          )}
          <button
            className="hover:bg-accent-green/10 hover:text-accent-green w-full px-3 py-2 text-left text-xs tracking-[0.05em] text-zinc-300 transition-colors"
            onClick={() => {
              duplicatePath(contextMenu.name);
              setContextMenu(null);
            }}
          >
            Duplicate
          </button>
          <button
            className="w-full px-3 py-2 text-left text-xs tracking-[0.05em] text-red-400 transition-colors hover:bg-red-500/10 disabled:cursor-not-allowed disabled:opacity-30"
            disabled={pathOrder.length <= 1}
            onClick={() => {
              deletePath(contextMenu.name);
              setContextMenu(null);
            }}
          >
            Delete
          </button>
        </div>
      )}
    </div>
  );
}
