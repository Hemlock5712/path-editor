// A minimal external store for React (useSyncExternalStore), so the stores stay plain TS that tests
// can drive without a DOM. Selectors must return stable references: a state slice, not a new object.

import { useSyncExternalStore } from "react";

export interface Store<S> {
  get(): S;
  set(update: Partial<S> | ((s: S) => Partial<S>)): void;
  subscribe(listener: () => void): () => void;
}

export function createStore<S extends object>(initial: S): Store<S> {
  let state = initial;
  const listeners = new Set<() => void>();
  return {
    get: () => state,
    set(update) {
      const partial = typeof update === "function" ? update(state) : update;
      state = { ...state, ...partial };
      for (const l of listeners) l();
    },
    subscribe(listener) {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
  };
}

export function useStore<S, T>(store: Store<S>, selector: (s: S) => T): T {
  return useSyncExternalStore(store.subscribe, () => selector(store.get()));
}
