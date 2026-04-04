import { useEffect, useRef } from 'react';
import type { ContextMenuState } from '../../hooks/useContextMenu';
import { PointContextMenu } from './PointContextMenu';
import { EmptySpaceContextMenu } from './EmptySpaceContextMenu';

interface FieldContextMenuProps {
  menu: ContextMenuState;
  onClose: () => void;
}

export function FieldContextMenu({ menu, onClose }: FieldContextMenuProps) {
  const menuRef = useRef<HTMLDivElement>(null);

  // Close on Escape
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
    const timer = setTimeout(() => {
      document.addEventListener('mousedown', handleClickOutside);
    }, 0);
    return () => {
      clearTimeout(timer);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [onClose]);

  if (!menu.visible) return null;

  return (
    <div
      ref={menuRef}
      className="absolute z-50"
      style={{ left: menu.x, top: menu.y }}
    >
      <div
        className="min-w-[180px] overflow-hidden rounded-lg border shadow-xl"
        style={{
          background: 'rgba(5, 5, 5, 0.95)',
          borderColor: 'rgba(0, 255, 170, 0.1)',
          boxShadow:
            '0 4px 24px rgba(0, 0, 0, 0.6), 0 0 16px rgba(0, 255, 170, 0.05)',
        }}
      >
        {menu.pointIndex !== null ? (
          <PointContextMenu pointIndex={menu.pointIndex} onClose={onClose} />
        ) : (
          <EmptySpaceContextMenu
            fieldX={menu.fieldX}
            fieldY={menu.fieldY}
            onClose={onClose}
          />
        )}
      </div>
    </div>
  );
}
