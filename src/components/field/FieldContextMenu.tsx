import { useEffect, useRef, useCallback } from 'react';
import { usePathStore } from '../../stores/pathStore';
import type { ContextMenuState } from '../../hooks/useContextMenu';
import { Trash2, Plus, Navigation, Flag } from 'lucide-react';

interface FieldContextMenuProps {
  menu: ContextMenuState;
  onClose: () => void;
}

export function FieldContextMenu({ menu, onClose }: FieldContextMenuProps) {
  const menuRef = useRef<HTMLDivElement>(null);

  const deletePoint = usePathStore((s) => s.deletePoint);
  const insertPointAfter = usePathStore((s) => s.insertPointAfter);
  const addPoint = usePathStore((s) => s.addPoint);
  const setHeading = usePathStore((s) => s.setHeading);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const selectPoint = usePathStore((s) => s.selectPoint);

  // Close on Escape key
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [onClose]);

  // Close on click outside
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(e.target as Node)) {
        onClose();
      }
    };
    // Use setTimeout to avoid closing immediately from the right-click that opened it
    const timer = setTimeout(() => {
      document.addEventListener('mousedown', handleClickOutside);
    }, 0);
    return () => {
      clearTimeout(timer);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [onClose]);

  const handleDelete = useCallback(() => {
    if (menu.pointIndex !== null) {
      deletePoint(menu.pointIndex);
    }
    onClose();
  }, [menu.pointIndex, deletePoint, onClose]);

  const handleInsertBefore = useCallback(() => {
    if (menu.pointIndex !== null && menu.pointIndex > 0) {
      // Insert before: insert after the previous point, at the midpoint location
      const prev = controlPoints[menu.pointIndex - 1];
      const curr = controlPoints[menu.pointIndex];
      const mid = { x: (prev.x + curr.x) / 2, y: (prev.y + curr.y) / 2 };
      insertPointAfter(menu.pointIndex - 1, mid);
      selectPoint(menu.pointIndex);
    }
    onClose();
  }, [menu.pointIndex, controlPoints, insertPointAfter, selectPoint, onClose]);

  const handleInsertAfter = useCallback(() => {
    if (menu.pointIndex !== null) {
      if (menu.pointIndex < controlPoints.length - 1) {
        const curr = controlPoints[menu.pointIndex];
        const next = controlPoints[menu.pointIndex + 1];
        const mid = { x: (curr.x + next.x) / 2, y: (curr.y + next.y) / 2 };
        insertPointAfter(menu.pointIndex, mid);
        selectPoint(menu.pointIndex + 1);
      } else {
        // Last point: insert at a small offset
        const curr = controlPoints[menu.pointIndex];
        const newPt = { x: curr.x + 0.5, y: curr.y };
        insertPointAfter(menu.pointIndex, newPt);
        selectPoint(menu.pointIndex + 1);
      }
    }
    onClose();
  }, [menu.pointIndex, controlPoints, insertPointAfter, selectPoint, onClose]);

  const handleSetHeading = useCallback(() => {
    if (menu.pointIndex !== null) {
      // Check if heading already exists
      const existing = headingWaypoints.find(
        (hw) => Math.round(hw.waypointIndex) === menu.pointIndex,
      );
      if (existing) {
        // Remove existing heading
        setHeading(menu.pointIndex, null);
      } else {
        // Set default heading of 0 degrees
        setHeading(menu.pointIndex, 0);
      }
    }
    onClose();
  }, [menu.pointIndex, headingWaypoints, setHeading, onClose]);

  const handleAddPointHere = useCallback(() => {
    addPoint({ x: menu.fieldX, y: menu.fieldY });
    selectPoint(controlPoints.length);
    onClose();
  }, [menu.fieldX, menu.fieldY, addPoint, selectPoint, controlPoints.length, onClose]);

  if (!menu.visible) return null;

  // Check if heading exists for point
  const hasHeading =
    menu.pointIndex !== null &&
    headingWaypoints.some((hw) => Math.round(hw.waypointIndex) === menu.pointIndex);

  // Check if we can insert before (not first point)
  const canInsertBefore = menu.pointIndex !== null && menu.pointIndex > 0;

  return (
    <div
      ref={menuRef}
      className="absolute z-50"
      style={{
        left: menu.x,
        top: menu.y,
      }}
    >
      <div
        className="min-w-[180px] rounded-lg overflow-hidden shadow-xl border"
        style={{
          background: 'rgba(5, 5, 5, 0.95)',
          borderColor: 'rgba(0, 255, 170, 0.1)',
          boxShadow: '0 4px 24px rgba(0, 0, 0, 0.6), 0 0 16px rgba(0, 255, 170, 0.05)',
        }}
      >
        {menu.pointIndex !== null ? (
          <>
            {/* Header */}
            <div
              className="px-3 py-1.5 text-xs font-mono"
              style={{
                color: 'rgba(255, 255, 255, 0.4)',
                borderBottom: '1px solid rgba(255, 255, 255, 0.08)',
              }}
            >
              Point {menu.pointIndex}
            </div>

            {/* Delete */}
            <MenuItem
              icon={<Trash2 size={14} />}
              label="Delete Point"
              onClick={handleDelete}
              disabled={controlPoints.length <= 2}
              destructive
            />

            {/* Insert Before */}
            <MenuItem
              icon={<Plus size={14} />}
              label="Insert Before"
              onClick={handleInsertBefore}
              disabled={!canInsertBefore}
            />

            {/* Insert After */}
            <MenuItem
              icon={<Plus size={14} />}
              label="Insert After"
              onClick={handleInsertAfter}
            />

            {/* Separator */}
            <div
              style={{
                height: '1px',
                background: 'rgba(255, 255, 255, 0.08)',
                margin: '2px 0',
              }}
            />

            {/* Set/Remove Heading */}
            <MenuItem
              icon={<Navigation size={14} />}
              label={hasHeading ? 'Remove Heading' : 'Set Heading'}
              onClick={handleSetHeading}
            />
          </>
        ) : (
          <>
            {/* Add point at click location */}
            <MenuItem
              icon={<Plus size={14} />}
              label={`Add Point (${menu.fieldX.toFixed(2)}, ${menu.fieldY.toFixed(2)})`}
              onClick={handleAddPointHere}
            />
          </>
        )}
      </div>
    </div>
  );
}

// ─── Menu Item Component ──────────────────────────────────────────────────

interface MenuItemProps {
  icon: React.ReactNode;
  label: string;
  onClick: () => void;
  disabled?: boolean;
  destructive?: boolean;
}

function MenuItem({ icon, label, onClick, disabled = false, destructive = false }: MenuItemProps) {
  const baseColor = destructive ? 'rgba(255, 51, 102, 0.9)' : 'rgba(228, 228, 231, 0.85)';
  const disabledColor = 'rgba(255, 255, 255, 0.25)';

  return (
    <button
      className="w-full flex items-center gap-2 px-3 py-1.5 text-left text-sm font-mono transition-colors"
      style={{
        color: disabled ? disabledColor : baseColor,
        cursor: disabled ? 'default' : 'pointer',
        background: 'transparent',
        border: 'none',
        outline: 'none',
      }}
      onClick={disabled ? undefined : onClick}
      onMouseEnter={(e) => {
        if (!disabled) {
          (e.currentTarget as HTMLElement).style.background = 'rgba(0, 255, 170, 0.05)';
        }
      }}
      onMouseLeave={(e) => {
        (e.currentTarget as HTMLElement).style.background = 'transparent';
      }}
      disabled={disabled}
    >
      <span className="flex-shrink-0 opacity-70">{icon}</span>
      <span>{label}</span>
    </button>
  );
}
