import { create } from 'zustand';

interface SelectionState {
  selectedPointIndex: number | null;
  selectedZoneId: string | null;
  /** Increments on every selectPoint call so effects re-fire even for the same index. */
  selectionVersion: number;
  selectPoint: (index: number | null) => void;
  selectZone: (id: string | null) => void;
  clearSelection: () => void;
}

export const useSelectionStore = create<SelectionState>()((set) => ({
  selectedPointIndex: null,
  selectedZoneId: null,
  selectionVersion: 0,
  selectPoint: (index) =>
    set((s) => ({ selectedPointIndex: index, selectionVersion: s.selectionVersion + 1 })),
  selectZone: (id) => set({ selectedZoneId: id }),
  clearSelection: () => set({ selectedPointIndex: null, selectedZoneId: null }),
}));
