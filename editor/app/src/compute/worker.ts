// The analysis worker: builds and checks one path per message, off the UI thread.

import { analyse, type AnalysisInput } from "./analysis.ts";

export interface AnalysisRequest {
  readonly id: number;
  readonly input: AnalysisInput;
}

export interface AnalysisResponse {
  readonly id: number;
  readonly result: ReturnType<typeof analyse>;
}

// The app's types use the DOM lib; inside a worker, self is the worker scope
const scope = self as unknown as {
  onmessage: ((event: MessageEvent<AnalysisRequest>) => void) | null;
  postMessage(message: AnalysisResponse): void;
};

scope.onmessage = (event) => {
  const { id, input } = event.data;
  scope.postMessage({ id, result: analyse(input) });
};
