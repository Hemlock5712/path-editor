import { create } from 'zustand';

interface EditorState {
  zoom: number;
  panOffset: { x: number; y: number };
  playbackState: 'stopped' | 'playing' | 'paused';
  playbackSpeed: number;
  scrubberDistance: number;
  activeChart: string;
  showGrid: boolean;
  showMinimap: boolean;
  snapToGrid: boolean;
  gridSize: number;
  sidebarCollapsed: boolean;
  bottomPanelHeight: number;
  hoveredPointIndex: number | null;
  showWaypointGhosts: boolean;

  setZoom: (zoom: number) => void;
  setPanOffset: (offset: { x: number; y: number }) => void;
  setPlaybackState: (state: 'stopped' | 'playing' | 'paused') => void;
  setPlaybackSpeed: (speed: number) => void;
  setScrubberDistance: (distance: number) => void;
  setActiveChart: (chart: string) => void;
  toggleGrid: () => void;
  toggleMinimap: () => void;
  toggleSnapToGrid: () => void;
  setGridSize: (size: number) => void;
  toggleSidebar: () => void;
  setBottomPanelHeight: (height: number) => void;
  setHoveredPointIndex: (index: number | null) => void;
  toggleWaypointGhosts: () => void;
  resetView: () => void;
}

export const useEditorStore = create<EditorState>()((set) => ({
  zoom: 1.0,
  panOffset: { x: 0, y: 0 },
  playbackState: 'stopped',
  playbackSpeed: 1.0,
  scrubberDistance: 0,
  activeChart: 'velocity-distance',
  showGrid: true,
  showMinimap: true,
  snapToGrid: false,
  gridSize: 0.5,
  sidebarCollapsed: false,
  bottomPanelHeight: 200,
  hoveredPointIndex: null,
  showWaypointGhosts: true,

  setZoom: (zoom) => set({ zoom }),
  setPanOffset: (panOffset) => set({ panOffset }),
  setPlaybackState: (playbackState) => set({ playbackState }),
  setPlaybackSpeed: (playbackSpeed) => set({ playbackSpeed }),
  setScrubberDistance: (scrubberDistance) => set({ scrubberDistance }),
  setActiveChart: (activeChart) => set({ activeChart }),
  toggleGrid: () => set((state) => ({ showGrid: !state.showGrid })),
  toggleMinimap: () => set((state) => ({ showMinimap: !state.showMinimap })),
  toggleSnapToGrid: () => set((state) => ({ snapToGrid: !state.snapToGrid })),
  setGridSize: (gridSize) => set({ gridSize }),
  toggleSidebar: () =>
    set((state) => ({ sidebarCollapsed: !state.sidebarCollapsed })),
  setBottomPanelHeight: (bottomPanelHeight) => set({ bottomPanelHeight }),
  setHoveredPointIndex: (hoveredPointIndex) => set({ hoveredPointIndex }),
  toggleWaypointGhosts: () =>
    set((state) => ({ showWaypointGhosts: !state.showWaypointGhosts })),
  resetView: () => set({ zoom: 1.0, panOffset: { x: 0, y: 0 } }),
}));
