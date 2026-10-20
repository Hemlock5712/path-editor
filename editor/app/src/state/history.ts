// Command-pattern undo and redo. A command knows how to apply itself to a document and how to revert
// it; the history keeps the two stacks. Documents are immutable, so a command holds the before and
// after values of just the part it changes.

export interface Command<T> {
  readonly label: string;
  apply(doc: T): T;
  revert(doc: T): T;
}

export class History<T> {
  private undoStack: Command<T>[] = [];
  private redoStack: Command<T>[] = [];

  constructor(private readonly limit = 500) {}

  /** Applies a command and records it. */
  execute(doc: T, command: Command<T>): T {
    const next = command.apply(doc);
    this.record(command);
    return next;
  }

  /** Records a command whose effect is already in the document (the end of a drag). */
  record(command: Command<T>): void {
    this.undoStack.push(command);
    if (this.undoStack.length > this.limit) this.undoStack.shift();
    this.redoStack = [];
  }

  undo(doc: T): T {
    const command = this.undoStack.pop();
    if (!command) return doc;
    this.redoStack.push(command);
    return command.revert(doc);
  }

  redo(doc: T): T {
    const command = this.redoStack.pop();
    if (!command) return doc;
    this.undoStack.push(command);
    return command.apply(doc);
  }

  clear(): void {
    this.undoStack = [];
    this.redoStack = [];
  }

  get canUndo(): boolean {
    return this.undoStack.length > 0;
  }

  get canRedo(): boolean {
    return this.redoStack.length > 0;
  }

  /** What undo would undo, for the button's tooltip. */
  get undoLabel(): string | null {
    return this.undoStack.at(-1)?.label ?? null;
  }

  get redoLabel(): string | null {
    return this.redoStack.at(-1)?.label ?? null;
  }
}

/** Several commands as one step: applied in order, reverted in reverse. */
export function composite<T>(label: string, commands: readonly Command<T>[]): Command<T> {
  return {
    label,
    apply: (doc) => commands.reduce((d, c) => c.apply(d), doc),
    revert: (doc) => commands.reduceRight((d, c) => c.revert(d), doc),
  };
}
