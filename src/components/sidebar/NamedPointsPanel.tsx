import { useState, useRef, useEffect } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { MapPin, X, Plus, Link, Unlink } from 'lucide-react';

export function NamedPointsPanel() {
  const namedPoints = usePathStore((s) => s.namedPoints);
  const selectedPointIndex = usePathStore((s) => s.selectedPointIndex);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const addNamedPoint = usePathStore((s) => s.addNamedPoint);
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

  // Only show primary points (exclude mirrors to reduce clutter)
  const primaryPoints = Object.values(namedPoints).filter((np) => {
    // A point is "primary" if its mirror's mirrorName points back to it
    // and its name doesn't end with " (Mirror)"
    return !np.name.endsWith(' (Mirror)');
  });

  const handleStartRename = (name: string) => {
    setRenamingKey(name);
    setRenameValue(name);
  };

  const handleConfirmRename = () => {
    if (renamingKey && renameValue.trim() && renameValue.trim() !== renamingKey) {
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

  const canSave = selectedPointIndex !== null && controlPoints.length > 0;
  const selectedRef = selectedPointIndex !== null ? controlPointRefs[selectedPointIndex] : null;

  return (
    <div className="space-y-2.5">
      {primaryPoints.map((np) => {
        const mirror = np.mirrorName ? namedPoints[np.mirrorName] : null;
        const isRenaming = renamingKey === np.name;

        return (
          <div
            key={np.name}
            className="rounded border border-amber-500/15 bg-amber-500/[0.04] p-2.5 space-y-1.5"
          >
            <div className="flex items-center justify-between">
              {isRenaming ? (
                <input
                  ref={renameInputRef}
                  className="text-[11px] font-mono bg-transparent border-b border-amber-400/40 text-amber-300 outline-none w-full mr-2"
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
                  className="text-[11px] font-mono text-amber-400/80 flex items-center gap-1.5 cursor-pointer"
                  onDoubleClick={() => handleStartRename(np.name)}
                  title="Double-click to rename"
                >
                  <MapPin size={10} />
                  {np.name}
                </span>
              )}
              <div className="flex items-center gap-1">
                <button
                  onClick={() => placeNamedPoint(np.name)}
                  className="btn-ghost p-0.5 text-zinc-500 hover:text-accent-green"
                  title="Place on active path"
                >
                  <Plus size={12} />
                </button>
                <button
                  onClick={() => deleteNamedPoint(np.name)}
                  className="btn-ghost p-0.5 text-zinc-500 hover:text-red-400"
                  title="Delete named point"
                >
                  <X size={12} />
                </button>
              </div>
            </div>

            <div className="flex items-center gap-3 text-[10px] text-zinc-500 font-mono">
              <span>x: {np.x.toFixed(2)}</span>
              <span>y: {np.y.toFixed(2)}</span>
              {np.headingDegrees !== null && (
                <span>hdg: {np.headingDegrees.toFixed(0)}&deg;</span>
              )}
            </div>

            {mirror && (
              <div className="flex items-center gap-3 text-[10px] text-zinc-600 font-mono">
                <span className="text-amber-500/40">mirror</span>
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
            className="flex-1 text-xs bg-transparent border-b border-accent-green/30 text-zinc-300 outline-none px-1 py-0.5"
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
            className="btn-ghost flex items-center gap-1.5 text-xs w-full justify-center py-1.5 disabled:opacity-30"
          >
            <MapPin size={13} />
            Save Selected Point
          </button>
        </div>
      )}

      {/* Link/unlink indicator for selected point */}
      {selectedPointIndex !== null && selectedRef && (
        <div className="flex items-center gap-1.5 text-[10px] text-amber-400/60 font-mono">
          <Link size={10} />
          Linked to: {selectedRef}
        </div>
      )}
    </div>
  );
}
