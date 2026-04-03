import { useEffect, useRef, useCallback, useState } from 'react';
import { usePathStore } from '../../stores/pathStore';
import type { ContextMenuState } from '../../hooks/useContextMenu';
import { MenuItem, MenuSeparator, SubMenu } from '../ui/ContextMenuPrimitives';
import { Trash2, Plus, Navigation, Crosshair, FlipVertical2, Copy, MapPin, Link, Unlink } from 'lucide-react';

interface FieldContextMenuProps {
  menu: ContextMenuState;
  onClose: () => void;
}

export function FieldContextMenu({ menu, onClose }: FieldContextMenuProps) {
  const menuRef = useRef<HTMLDivElement>(null);

  const addPoint = usePathStore((s) => s.addPoint);
  const deletePoint = usePathStore((s) => s.deletePoint);
  const insertPointAfter = usePathStore((s) => s.insertPointAfter);
  const setHeading = usePathStore((s) => s.setHeading);
  const addRotationZone = usePathStore((s) => s.addRotationZone);
  const flipPathY = usePathStore((s) => s.flipPathY);
  const duplicatePath = usePathStore((s) => s.duplicatePath);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const selectPoint = usePathStore((s) => s.selectPoint);
  const namedPoints = usePathStore((s) => s.namedPoints);
  const savePointAsNamed = usePathStore((s) => s.savePointAsNamed);
  const linkPointToNamed = usePathStore((s) => s.linkPointToNamed);
  const unlinkPoint = usePathStore((s) => s.unlinkPoint);
  const placeNamedPoint = usePathStore((s) => s.placeNamedPoint);

  const [namingPoint, setNamingPoint] = useState(false);
  const [nameValue, setNameValue] = useState('');
  const nameInputRef = useRef<HTMLInputElement>(null);

  // Close on Escape key
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        if (namingPoint) {
          setNamingPoint(false);
        } else {
          onClose();
        }
      }
    };
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [onClose, namingPoint]);

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

  // Focus name input when naming mode opens
  useEffect(() => {
    if (namingPoint && nameInputRef.current) nameInputRef.current.focus();
  }, [namingPoint]);

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

  const handleAddPoint = useCallback(() => {
    addPoint({ x: menu.fieldX, y: menu.fieldY });
    selectPoint(controlPoints.length);
    onClose();
  }, [menu.fieldX, menu.fieldY, addPoint, selectPoint, controlPoints.length, onClose]);

  const handleFlipPathY = useCallback(() => {
    flipPathY();
    onClose();
  }, [flipPathY, onClose]);

  const handleDuplicatePath = useCallback(() => {
    duplicatePath();
    onClose();
  }, [duplicatePath, onClose]);

  const handleAddFacePointZone = useCallback(() => {
    if (controlPoints.length < 2) return;
    const numCPs = controlPoints.length;
    // Estimate a waypoint index near the click point
    // Simple heuristic: use the clicked field coordinate's position along the path
    const midIdx = (numCPs - 1) / 2;
    const halfSpan = Math.max(0.5, (numCPs - 1) * 0.15);
    const startIdx = Math.max(0, midIdx - halfSpan);
    const endIdx = Math.min(numCPs - 1, midIdx + halfSpan);
    // Target: offset from the click point
    const target = { x: menu.fieldX, y: menu.fieldY + 2 };

    addRotationZone({
      id: crypto.randomUUID(),
      startWaypointIndex: Math.round(startIdx * 10) / 10,
      endWaypointIndex: Math.round(endIdx * 10) / 10,
      targetPoint: target,
    });
    onClose();
  }, [controlPoints.length, menu.fieldX, menu.fieldY, addRotationZone, onClose]);

  const handleSaveAsNamed = useCallback(() => {
    setNamingPoint(true);
    setNameValue('');
  }, []);

  const handleConfirmName = useCallback(() => {
    if (menu.pointIndex !== null && nameValue.trim() && !namedPoints[nameValue.trim()]) {
      savePointAsNamed(menu.pointIndex, nameValue.trim());
    }
    setNamingPoint(false);
    onClose();
  }, [menu.pointIndex, nameValue, namedPoints, savePointAsNamed, onClose]);

  const handleUnlink = useCallback(() => {
    if (menu.pointIndex !== null) {
      unlinkPoint(menu.pointIndex);
    }
    onClose();
  }, [menu.pointIndex, unlinkPoint, onClose]);

  if (!menu.visible) return null;

  // Check if heading exists for point
  const hasHeading =
    menu.pointIndex !== null &&
    headingWaypoints.some((hw) => Math.round(hw.waypointIndex) === menu.pointIndex);

  // Check if we can insert before (not first point)
  const canInsertBefore = menu.pointIndex !== null && menu.pointIndex > 0;

  // Check if this point is linked to a named point
  const pointRef = menu.pointIndex !== null ? controlPointRefs[menu.pointIndex] : null;

  // Named points list (exclude mirrors for cleaner submenu)
  const namedPointList = Object.values(namedPoints).filter(
    (np) => !np.name.endsWith(' (Mirror)'),
  );

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
              {pointRef ? (
                <span style={{ color: 'rgba(255, 179, 0, 0.7)' }}>
                  <MapPin size={10} style={{ display: 'inline', marginRight: 4 }} />
                  {pointRef}
                </span>
              ) : (
                <>Point {menu.pointIndex}</>
              )}
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
            <MenuSeparator />

            {/* Set/Remove Heading */}
            <MenuItem
              icon={<Navigation size={14} />}
              label={hasHeading ? 'Remove Heading' : 'Set Heading'}
              onClick={handleSetHeading}
            />

            {/* Separator */}
            <MenuSeparator />

            {/* Named point actions */}
            {namingPoint ? (
              <div className="px-3 py-1.5">
                <input
                  ref={nameInputRef}
                  className="w-full text-xs font-mono bg-transparent border-b outline-none px-0.5 py-0.5"
                  style={{
                    borderColor: 'rgba(255, 179, 0, 0.4)',
                    color: 'rgba(255, 179, 0, 0.9)',
                  }}
                  placeholder="Point name..."
                  value={nameValue}
                  onChange={(e) => setNameValue(e.target.value)}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter') handleConfirmName();
                    if (e.key === 'Escape') {
                      setNamingPoint(false);
                      e.stopPropagation();
                    }
                  }}
                />
              </div>
            ) : (
              <>
                {!pointRef && (
                  <MenuItem
                    icon={<MapPin size={14} />}
                    label="Save as Named Point"
                    onClick={handleSaveAsNamed}
                  />
                )}

                {pointRef && (
                  <MenuItem
                    icon={<Unlink size={14} />}
                    label="Unlink Point"
                    onClick={handleUnlink}
                  />
                )}

                {namedPointList.length > 0 && (
                  <SubMenu
                    icon={<Link size={14} />}
                    label="Link to Named Point"
                  >
                    {namedPointList.map((np) => (
                      <MenuItem
                        key={np.name}
                        icon={<MapPin size={12} />}
                        label={np.name}
                        onClick={() => {
                          if (menu.pointIndex !== null) {
                            linkPointToNamed(menu.pointIndex, np.name);
                          }
                          onClose();
                        }}
                      />
                    ))}
                  </SubMenu>
                )}
              </>
            )}
          </>
        ) : (
          <>
            {/* Add point here */}
            <MenuItem
              icon={<Plus size={14} />}
              label="Add Point Here"
              onClick={handleAddPoint}
            />

            {/* Place named point submenu */}
            {namedPointList.length > 0 && (
              <SubMenu
                icon={<MapPin size={14} />}
                label="Place Named Point"
              >
                {namedPointList.map((np) => (
                  <MenuItem
                    key={np.name}
                    icon={<MapPin size={12} />}
                    label={np.name}
                    onClick={() => {
                      placeNamedPoint(np.name);
                      selectPoint(controlPoints.length);
                      onClose();
                    }}
                  />
                ))}
              </SubMenu>
            )}

            {/* Add face-point zone */}
            <MenuItem
              icon={<Crosshair size={14} />}
              label="Add Face-Point Zone"
              onClick={handleAddFacePointZone}
              disabled={controlPoints.length < 2}
            />

            {/* Separator */}
            <MenuSeparator />

            {/* Flip path */}
            <MenuItem
              icon={<FlipVertical2 size={14} />}
              label="Flip Path (Left/Right)"
              onClick={handleFlipPathY}
              disabled={controlPoints.length < 2}
            />

            {/* Duplicate path */}
            <MenuItem
              icon={<Copy size={14} />}
              label="Duplicate Path"
              onClick={handleDuplicatePath}
              disabled={controlPoints.length === 0}
            />
          </>
        )}
      </div>
    </div>
  );
}

