import { useState, useCallback } from 'react';

export interface ContextMenuState {
  visible: boolean;
  x: number;
  y: number;
  pointIndex: number | null; // which point was right-clicked (null = empty space)
  fieldX: number; // field coordinates of click
  fieldY: number;
}

const INITIAL_STATE: ContextMenuState = {
  visible: false,
  x: 0,
  y: 0,
  pointIndex: null,
  fieldX: 0,
  fieldY: 0,
};

export function useContextMenu() {
  const [menu, setMenu] = useState<ContextMenuState>(INITIAL_STATE);

  const show = useCallback(
    (x: number, y: number, pointIndex: number | null, fieldX: number, fieldY: number) => {
      setMenu({ visible: true, x, y, pointIndex, fieldX, fieldY });
    },
    [],
  );

  const hide = useCallback(() => {
    setMenu(INITIAL_STATE);
  }, []);

  return { menu, show, hide };
}
