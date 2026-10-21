// A yes/no question in the app's own dialog (never the browser's confirm()).

import { createStore } from "./store.ts";

export interface Question {
  readonly text: string;
  readonly ok: string;
  readonly answer: (yes: boolean) => void;
}

export const question = createStore<{ readonly current: Question | null }>({ current: null });

export function askConfirm(text: string, ok = "OK"): Promise<boolean> {
  return new Promise((resolve) => {
    question.get().current?.answer(false);
    question.set({
      current: {
        text,
        ok,
        answer: (yes) => {
          question.set({ current: null });
          resolve(yes);
        },
      },
    });
  });
}
