import { create } from 'zustand';

interface SelectionState {
  selectedPointIndex: number | null;
  selectedZoneId: string | null;
  selectPoint: (index: number | null) => void;
  selectZone: (id: string | null) => void;
  clearSelection: () => void;
}

export const useSelectionStore = create<SelectionState>()((set) => ({
  selectedPointIndex: null,
  selectedZoneId: null,
  selectPoint: (index) => set({ selectedPointIndex: index }),
  selectZone: (id) => set({ selectedZoneId: id }),
  clearSelection: () => set({ selectedPointIndex: null, selectedZoneId: null }),
}));
