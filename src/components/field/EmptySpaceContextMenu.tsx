import { useCallback } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';
import { MenuItem, MenuSeparator, SubMenu } from '../ui/ContextMenuPrimitives';
import { Plus, Crosshair, FlipVertical2, Copy, MapPin } from 'lucide-react';

interface EmptySpaceContextMenuProps {
  fieldX: number;
  fieldY: number;
  onClose: () => void;
}

export function EmptySpaceContextMenu({
  fieldX,
  fieldY,
  onClose,
}: EmptySpaceContextMenuProps) {
  const addPoint = usePathStore((s) => s.addPoint);
  const addRotationZone = usePathStore((s) => s.addRotationZone);
  const flipPathY = usePathStore((s) => s.flipPathY);
  const duplicatePath = usePathStore((s) => s.duplicatePath);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const namedPoints = usePathStore((s) => s.namedPoints);
  const placeNamedPoint = usePathStore((s) => s.placeNamedPoint);
  const selectPoint = useSelectionStore((s) => s.selectPoint);

  const handleAddPoint = useCallback(() => {
    addPoint({ x: fieldX, y: fieldY });
    selectPoint(controlPoints.length);
    onClose();
  }, [fieldX, fieldY, addPoint, selectPoint, controlPoints.length, onClose]);

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
    const midIdx = (numCPs - 1) / 2;
    const halfSpan = Math.max(0.5, (numCPs - 1) * 0.15);
    const startIdx = Math.max(0, midIdx - halfSpan);
    const endIdx = Math.min(numCPs - 1, midIdx + halfSpan);
    const target = { x: fieldX, y: fieldY + 2 };

    addRotationZone({
      id: crypto.randomUUID(),
      startWaypointIndex: Math.round(startIdx * 10) / 10,
      endWaypointIndex: Math.round(endIdx * 10) / 10,
      targetPoint: target,
    });
    onClose();
  }, [controlPoints.length, fieldX, fieldY, addRotationZone, onClose]);

  const namedPointList = Object.values(namedPoints).filter(
    (np) => !np.name.endsWith(' (Mirror)')
  );

  return (
    <>
      <MenuItem
        icon={<Plus size={14} />}
        label="Add Point Here"
        onClick={handleAddPoint}
      />

      {namedPointList.length > 0 && (
        <SubMenu icon={<MapPin size={14} />} label="Place Named Point">
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

      <MenuItem
        icon={<Crosshair size={14} />}
        label="Add Face-Point Zone"
        onClick={handleAddFacePointZone}
        disabled={controlPoints.length < 2}
      />

      <MenuSeparator />

      <MenuItem
        icon={<FlipVertical2 size={14} />}
        label="Flip Path (Left/Right)"
        onClick={handleFlipPathY}
        disabled={controlPoints.length < 2}
      />

      <MenuItem
        icon={<Copy size={14} />}
        label="Duplicate Path"
        onClick={handleDuplicatePath}
        disabled={controlPoints.length === 0}
      />
    </>
  );
}
