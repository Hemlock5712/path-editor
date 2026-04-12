import { useCallback, useState, useEffect, useRef } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';
import { MenuItem, MenuSeparator, SubMenu } from '../ui/ContextMenuPrimitives';
import { Trash2, Plus, Navigation, MapPin, Link, Unlink } from 'lucide-react';
import { getPrimaryNamedPoints } from '../../types';

interface PointContextMenuProps {
  pointIndex: number;
  onClose: () => void;
}

export function PointContextMenu({
  pointIndex,
  onClose,
}: PointContextMenuProps) {
  const deletePoint = usePathStore((s) => s.deletePoint);
  const insertPointAfter = usePathStore((s) => s.insertPointAfter);
  const setHeading = usePathStore((s) => s.setHeading);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const namedPoints = usePathStore((s) => s.namedPoints);
  const savePointAsNamed = usePathStore((s) => s.savePointAsNamed);
  const linkPointToNamed = usePathStore((s) => s.linkPointToNamed);
  const unlinkPoint = usePathStore((s) => s.unlinkPoint);
  const selectPoint = useSelectionStore((s) => s.selectPoint);

  const [namingPoint, setNamingPoint] = useState(false);
  const [nameValue, setNameValue] = useState('');
  const nameInputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (namingPoint && nameInputRef.current) nameInputRef.current.focus();
  }, [namingPoint]);

  const handleDelete = useCallback(() => {
    deletePoint(pointIndex);
    onClose();
  }, [pointIndex, deletePoint, onClose]);

  const handleInsertBefore = useCallback(() => {
    if (pointIndex > 0) {
      const prev = controlPoints[pointIndex - 1];
      const curr = controlPoints[pointIndex];
      const mid = { x: (prev.x + curr.x) / 2, y: (prev.y + curr.y) / 2 };
      insertPointAfter(pointIndex - 1, mid);
      selectPoint(pointIndex);
    } else {
      const curr = controlPoints[0];
      insertPointAfter(-1, { x: curr.x - 0.5, y: curr.y });
      selectPoint(0);
    }
    onClose();
  }, [pointIndex, controlPoints, insertPointAfter, selectPoint, onClose]);

  const handleInsertAfter = useCallback(() => {
    if (pointIndex < controlPoints.length - 1) {
      const curr = controlPoints[pointIndex];
      const next = controlPoints[pointIndex + 1];
      const mid = { x: (curr.x + next.x) / 2, y: (curr.y + next.y) / 2 };
      insertPointAfter(pointIndex, mid);
      selectPoint(pointIndex + 1);
    } else {
      const curr = controlPoints[pointIndex];
      insertPointAfter(pointIndex, { x: curr.x + 0.5, y: curr.y });
      selectPoint(pointIndex + 1);
    }
    onClose();
  }, [pointIndex, controlPoints, insertPointAfter, selectPoint, onClose]);

  const handleSetHeading = useCallback(() => {
    const existing = headingWaypoints.find(
      (hw) => Math.round(hw.waypointIndex) === pointIndex
    );
    setHeading(pointIndex, existing ? null : 0);
    onClose();
  }, [pointIndex, headingWaypoints, setHeading, onClose]);

  const handleSaveAsNamed = useCallback(() => {
    setNamingPoint(true);
    setNameValue('');
  }, []);

  const handleConfirmName = useCallback(() => {
    if (nameValue.trim() && !namedPoints[nameValue.trim()]) {
      savePointAsNamed(pointIndex, nameValue.trim());
    }
    setNamingPoint(false);
    onClose();
  }, [pointIndex, nameValue, namedPoints, savePointAsNamed, onClose]);

  const handleUnlink = useCallback(() => {
    unlinkPoint(pointIndex);
    onClose();
  }, [pointIndex, unlinkPoint, onClose]);

  const hasHeading = headingWaypoints.some(
    (hw) => Math.round(hw.waypointIndex) === pointIndex
  );
  const pointRef = controlPointRefs[pointIndex];
  const namedPointList = getPrimaryNamedPoints(namedPoints);

  return (
    <>
      {/* Header */}
      <div
        className="px-3 py-1.5 font-mono text-xs"
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
          <>Point {pointIndex}</>
        )}
      </div>

      <MenuItem
        icon={<Trash2 size={14} />}
        label="Delete Point"
        onClick={handleDelete}
        disabled={controlPoints.length <= 2}
        destructive
      />
      <MenuItem
        icon={<Plus size={14} />}
        label="Insert Before"
        onClick={handleInsertBefore}
      />
      <MenuItem
        icon={<Plus size={14} />}
        label="Insert After"
        onClick={handleInsertAfter}
      />

      <MenuSeparator />

      <MenuItem
        icon={<Navigation size={14} />}
        label={hasHeading ? 'Remove Heading' : 'Set Heading'}
        onClick={handleSetHeading}
      />

      <MenuSeparator />

      {namingPoint ? (
        <div className="px-3 py-1.5">
          <input
            ref={nameInputRef}
            className="w-full border-b bg-transparent px-0.5 py-0.5 font-mono text-xs outline-none"
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
            <SubMenu icon={<Link size={14} />} label="Link to Named Point">
              {namedPointList.flatMap((np) => {
                const items = [
                  <MenuItem
                    key={np.name}
                    icon={<MapPin size={12} />}
                    label={np.name}
                    onClick={() => {
                      linkPointToNamed(pointIndex, np.name);
                      onClose();
                    }}
                  />,
                ];
                if (np.mirrorName && namedPoints[np.mirrorName]) {
                  items.push(
                    <MenuItem
                      key={np.mirrorName}
                      icon={<MapPin size={12} />}
                      label={`${np.name} (Flipped)`}
                      onClick={() => {
                        linkPointToNamed(pointIndex, np.mirrorName!);
                        onClose();
                      }}
                    />
                  );
                }
                return items;
              })}
            </SubMenu>
          )}
        </>
      )}
    </>
  );
}
