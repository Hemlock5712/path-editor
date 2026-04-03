import { useState, useRef, useEffect } from 'react';
import { Plus } from 'lucide-react';
import { usePathStore } from '../../stores/pathStore';

export function PathTabs() {
  const paths = usePathStore((s) => s.paths);
  const pathOrder = usePathStore((s) => s.pathOrder);
  const activePathName = usePathStore((s) => s.activePathName);
  const setActivePath = usePathStore((s) => s.setActivePath);
  const addPath = usePathStore((s) => s.addPath);
  const renamePath = usePathStore((s) => s.renamePath);
  const deletePath = usePathStore((s) => s.deletePath);
  const reorderPath = usePathStore((s) => s.reorderPath);

  const [editingName, setEditingName] = useState<string | null>(null);
  const [editValue, setEditValue] = useState('');
  const [contextMenu, setContextMenu] = useState<{ name: string; x: number; y: number } | null>(null);
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

  const startRename = (name: string) => {
    setEditingName(name);
    setEditValue(name);
    setContextMenu(null);
  };

  const commitRename = () => {
    if (editingName && editValue.trim() && editValue.trim() !== editingName) {
      renamePath(editingName, editValue.trim());
    }
    setEditingName(null);
  };

  const handleAdd = () => {
    let base = 'Path';
    let n = pathOrder.length + 1;
    let name = `${base} ${n}`;
    while (paths[name]) {
      n++;
      name = `${base} ${n}`;
    }
    addPath(name);
  };

  const handleDragStart = (e: React.DragEvent, index: number) => {
    setDragIndex(index);
    e.dataTransfer.effectAllowed = 'move';
    e.dataTransfer.setData('text/plain', String(index));
  };

  const handleDragOver = (e: React.DragEvent, index: number) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'move';
    if (dragIndex !== null && index !== dragIndex) {
      setDropIndex(index);
    }
  };

  const handleDrop = (e: React.DragEvent, toIndex: number) => {
    e.preventDefault();
    if (dragIndex !== null && dragIndex !== toIndex) {
      reorderPath(dragIndex, toIndex);
    }
    setDragIndex(null);
    setDropIndex(null);
  };

  const handleDragEnd = () => {
    setDragIndex(null);
    setDropIndex(null);
  };

  return (
    <div className="flex items-center gap-0.5 px-3 pt-1.5 pb-0 flex-shrink-0 overflow-x-auto">
      {pathOrder.map((name, index) => (
        <div
          key={name}
          className={`flex-shrink-0 relative ${
            dropIndex === index && dragIndex !== null && dragIndex !== index
              ? dragIndex > index
                ? 'before:absolute before:left-0 before:top-1 before:bottom-1 before:w-0.5 before:bg-accent-green before:rounded-full'
                : 'after:absolute after:right-0 after:top-1 after:bottom-1 after:w-0.5 after:bg-accent-green after:rounded-full'
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
              className="px-2.5 py-1 text-xs font-medium bg-zinc-900 text-accent-green border border-accent-green/40 rounded outline-none w-28"
            />
          ) : (
            <button
              onClick={() => setActivePath(name)}
              onDoubleClick={() => startRename(name)}
              onContextMenu={(e) => {
                e.preventDefault();
                setContextMenu({ name, x: e.clientX, y: e.clientY });
              }}
              className={`px-2.5 py-1 text-xs font-medium transition-all duration-200 border-b cursor-grab active:cursor-grabbing ${
                activePathName === name
                  ? 'text-accent-green border-accent-green shadow-[0_1px_6px_rgba(0,255,170,0.3)]'
                  : 'text-zinc-600 border-transparent hover:text-zinc-400'
              } ${dragIndex === index ? 'opacity-40' : ''}`}
            >
              {name}
            </button>
          )}
        </div>
      ))}

      {/* Add path button */}
      <button
        onClick={handleAdd}
        className="btn-ghost p-1 ml-1 text-zinc-600 hover:text-accent-green transition-colors"
        title="New path"
      >
        <Plus size={14} />
      </button>

      {/* Context menu */}
      {contextMenu && (
        <div
          className="fixed z-50 min-w-[120px] rounded-lg border border-accent-green/20 bg-zinc-950/95 backdrop-blur-sm shadow-lg shadow-accent-green/5 py-1"
          style={{ left: contextMenu.x, top: contextMenu.y }}
          onMouseDown={(e) => e.stopPropagation()}
        >
          <button
            className="w-full text-left px-3 py-1.5 text-xs text-zinc-300 hover:bg-accent-green/10 hover:text-accent-green transition-colors"
            onClick={() => startRename(contextMenu.name)}
          >
            Rename
          </button>
          <button
            className="w-full text-left px-3 py-1.5 text-xs text-red-400 hover:bg-red-500/10 transition-colors disabled:opacity-30 disabled:cursor-not-allowed"
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
