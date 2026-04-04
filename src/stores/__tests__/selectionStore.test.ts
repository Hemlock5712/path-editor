import { describe, it, expect, beforeEach } from 'vitest';
import { useSelectionStore } from '../selectionStore';

describe('useSelectionStore', () => {
  beforeEach(() => {
    useSelectionStore.setState({
      selectedPointIndex: null,
      selectedZoneId: null,
    });
  });

  it('starts with null selections', () => {
    const state = useSelectionStore.getState();
    expect(state.selectedPointIndex).toBeNull();
    expect(state.selectedZoneId).toBeNull();
  });

  it('selectPoint updates selectedPointIndex', () => {
    useSelectionStore.getState().selectPoint(3);
    expect(useSelectionStore.getState().selectedPointIndex).toBe(3);
  });

  it('selectZone updates selectedZoneId', () => {
    useSelectionStore.getState().selectZone('zone-1');
    expect(useSelectionStore.getState().selectedZoneId).toBe('zone-1');
  });

  it('clearSelection resets both', () => {
    useSelectionStore.getState().selectPoint(5);
    useSelectionStore.getState().selectZone('zone-2');
    useSelectionStore.getState().clearSelection();
    const state = useSelectionStore.getState();
    expect(state.selectedPointIndex).toBeNull();
    expect(state.selectedZoneId).toBeNull();
  });

  it('selectPoint(null) clears point selection', () => {
    useSelectionStore.getState().selectPoint(2);
    useSelectionStore.getState().selectPoint(null);
    expect(useSelectionStore.getState().selectedPointIndex).toBeNull();
  });

  it('selectZone(null) clears zone selection', () => {
    useSelectionStore.getState().selectZone('zone-1');
    useSelectionStore.getState().selectZone(null);
    expect(useSelectionStore.getState().selectedZoneId).toBeNull();
  });
});
