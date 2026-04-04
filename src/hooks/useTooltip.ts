import { useState, useCallback } from 'react';

export interface TooltipState {
  visible: boolean;
  x: number;
  y: number;
  fieldX: number;
  fieldY: number;
}

const HIDDEN: TooltipState = {
  visible: false,
  x: 0,
  y: 0,
  fieldX: 0,
  fieldY: 0,
};

export function useTooltip() {
  const [tooltip, setTooltip] = useState<TooltipState>(HIDDEN);

  const show = useCallback(
    (x: number, y: number, fieldX: number, fieldY: number) => {
      setTooltip({ visible: true, x, y, fieldX, fieldY });
    },
    []
  );

  const hide = useCallback(() => {
    setTooltip((prev) => (prev.visible ? { ...prev, visible: false } : prev));
  }, []);

  return { tooltip, show, hide } as const;
}
