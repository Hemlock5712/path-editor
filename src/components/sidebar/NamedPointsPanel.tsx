import { useState, useRef, useEffect, useMemo, memo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';
import { MapPin, X, Plus, Link, FlipVertical2 } from 'lucide-react';
import { Combobox } from '../ui/Combobox';
import { getPrimaryNamedPoints } from '../../types';

export const NamedPointsPanel = memo(function NamedPointsPanel() {
  const namedPoints = usePathStore((s) => s.namedPoints);
  const selectedPointIndex = useSelectionStore((s) => s.selectedPointIndex);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const deleteNamedPoint = usePathStore((s) => s.deleteNamedPoint);
  const renameNamedPoint = usePathStore((s) => s.renameNamedPoint);
  const placeNamedPoint = usePathStore((s) => s.placeNamedPoint);
  const savePointAsNamed = usePathStore((s) => s.savePointAsNamed);

  const [renamingKey, setRenamingKey] = useState<string | null>(null);
  const [renameValue, setRenameValue] = useState('');
  const [savingName, setSavingName] = useState(false);
  const [saveNameValue, setSaveNameValue] = useState('');
  const renameInputRef = useRef<HTMLInputElement>(null);
  const saveInputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (renamingKey && renameInputRef.current) renameInputRef.current.focus();
  }, [renamingKey]);

  useEffect(() => {
    if (savingName && saveInputRef.current) saveInputRef.current.focus();
  }, [savingName]);

  const primaryPoints = getPrimaryNamedPoints(namedPoints);

  const handleStartRename = (name: string) => {
    setRenamingKey(name);
    setRenameValue(name);
  };

  const handleConfirmRename = () => {
    if (
      renamingKey &&
      renameValue.trim() &&
      renameValue.trim() !== renamingKey
    ) {
      renameNamedPoint(renamingKey, renameValue.trim());
    }
    setRenamingKey(null);
  };

  const handleStartSave = () => {
    setSavingName(true);
    setSaveNameValue('');
  };

  const handleConfirmSave = () => {
    if (selectedPointIndex !== null && saveNameValue.trim()) {
      const name = saveNameValue.trim();
      if (!namedPoints[name]) {
        savePointAsNamed(selectedPointIndex, name);
      }
    }
    setSavingName(false);
  };

  const [pickerValue, setPickerValue] = useState('');

  const pointOptions = useMemo(
    () =>
      primaryPoints.flatMap((np) => {
        const mirror = np.mirrorName ? namedPoints[np.mirrorName] : null;
        const items = [
          {
            value: np.name,
            label: np.name,
            detail: `(${np.x.toFixed(2)}, ${np.y.toFixed(2)})${np.headingDegrees !== null ? ` hdg: ${np.headingDegrees.toFixed(0)}\u00B0` : ''}`,
          },
        ];
        if (mirror) {
          items.push({
            value: mirror.name,
            label: `${np.name} (Flipped)`,
            detail: `(${mirror.x.toFixed(2)}, ${mirror.y.toFixed(2)})${mirror.headingDegrees !== null ? ` hdg: ${mirror.headingDegrees.toFixed(0)}\u00B0` : ''}`,
          });
        }
        return items;
      }),
    [primaryPoints, namedPoints]
  );

  const canSave = selectedPointIndex !== null && controlPoints.length > 0;
  const selectedRef =
    selectedPointIndex !== null ? controlPointRefs[selectedPointIndex] : null;

  return (
    <div className="space-y-2.5">
      {/* Quick-place named point picker */}
      {pointOptions.length > 0 && (
        <Combobox
          options={pointOptions}
          value={pickerValue}
          onChange={setPickerValue}
          onCommit={(name) => {
            placeNamedPoint(name);
            setPickerValue('');
          }}
          placeholder="Place a named point..."
        />
      )}

      {primaryPoints.map((np) => {
        const mirror = np.mirrorName ? namedPoints[np.mirrorName] : null;
        const isRenaming = renamingKey === np.name;

        return (
          <div
            key={np.name}
            className="space-y-2 rounded-lg border border-amber-400/20 bg-amber-500/[0.06] p-3"
          >
            <div className="flex items-center justify-between">
              {isRenaming ? (
                <input
                  ref={renameInputRef}
                  className="mr-2 w-full border-b border-amber-300/60 bg-transparent font-mono text-[12px] text-amber-100 outline-none"
                  value={renameValue}
                  onChange={(e) => setRenameValue(e.target.value)}
                  onBlur={handleConfirmRename}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter') handleConfirmRename();
                    if (e.key === 'Escape') setRenamingKey(null);
                  }}
                />
              ) : (
                <span
                  className="flex cursor-pointer items-center gap-1.5 font-mono text-[12px] text-amber-100"
                  onDoubleClick={() => handleStartRename(np.name)}
                  title="Double-click to rename"
                >
                  <MapPin size={11} />
                  {np.name}
                </span>
              )}
              <div className="flex items-center gap-1">
                <button
                  onClick={() => placeNamedPoint(np.name)}
                  className="btn-ghost p-0.5 text-zinc-300 hover:text-accent-green"
                  title="Place on active path"
                >
                  <Plus size={12} />
                </button>
                {np.mirrorName && namedPoints[np.mirrorName] && (
                  <button
                    onClick={() => placeNamedPoint(np.mirrorName!)}
                    className="btn-ghost p-0.5 text-zinc-300 hover:text-accent-green"
                    title="Place flipped on active path"
                  >
                    <FlipVertical2 size={12} />
                  </button>
                )}
                <button
                  onClick={() => deleteNamedPoint(np.name)}
                  className="btn-ghost p-0.5 text-zinc-300 hover:text-red-400"
                  title="Delete named point"
                >
                  <X size={12} />
                </button>
              </div>
            </div>

            <div className="flex items-center gap-3 font-mono text-[12px] text-zinc-200">
              <span>x: {np.x.toFixed(2)}</span>
              <span>y: {np.y.toFixed(2)}</span>
              {np.headingDegrees !== null && (
                <span>hdg: {np.headingDegrees.toFixed(0)}&deg;</span>
              )}
            </div>

            {mirror && (
              <div className="flex items-center gap-3 font-mono text-[12px] text-zinc-500">
                <span className="text-amber-200">mirror</span>
                <span>x: {mirror.x.toFixed(2)}</span>
                <span>y: {mirror.y.toFixed(2)}</span>
                {mirror.headingDegrees !== null && (
                  <span>hdg: {mirror.headingDegrees.toFixed(0)}&deg;</span>
                )}
              </div>
            )}
          </div>
        );
      })}

      {/* Save selected point form */}
      {savingName ? (
        <div className="flex items-center gap-1.5">
          <input
            ref={saveInputRef}
            className="border-accent-green/30 flex-1 border-b bg-transparent px-1 py-0.5 text-xs text-zinc-300 outline-none"
            placeholder="Point name..."
            value={saveNameValue}
            onChange={(e) => setSaveNameValue(e.target.value)}
            onBlur={handleConfirmSave}
            onKeyDown={(e) => {
              if (e.key === 'Enter') handleConfirmSave();
              if (e.key === 'Escape') setSavingName(false);
            }}
          />
        </div>
      ) : (
        <div className="flex items-center gap-1.5">
          <button
            onClick={handleStartSave}
            disabled={!canSave}
            className="btn-ghost flex w-full items-center justify-center gap-1.5 py-1.5 text-xs disabled:opacity-30"
          >
            <MapPin size={13} />
            Save Selected Point
          </button>
        </div>
      )}

      {/* Link/unlink indicator for selected point */}
      {selectedPointIndex !== null && selectedRef && (
        <div className="flex items-center gap-1.5 font-mono text-[12px] text-amber-200">
          <Link size={11} />
          Linked to: {selectedRef}
        </div>
      )}
    </div>
  );
});
