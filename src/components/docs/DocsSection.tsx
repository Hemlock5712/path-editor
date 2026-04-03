import type { ReactNode } from 'react';

export function DocsSection({
  icon,
  title,
  id,
  children,
}: {
  icon: ReactNode;
  title: string;
  id?: string;
  children: ReactNode;
}) {
  return (
    <div id={id} className="neon-panel p-4 scroll-mt-6">
      <h3 className="flex items-center gap-2 text-[11px] font-light tracking-wide text-accent-green/40 mb-3">
        {icon}
        {title}
      </h3>
      {children}
    </div>
  );
}

export function CodeBlock({ children }: { children: string }) {
  return (
    <pre className="bg-surface-900 rounded p-3 text-[11px] overflow-x-auto text-zinc-300 font-mono leading-relaxed">
      {children}
    </pre>
  );
}

export function Callout({
  children,
  color = 'green',
}: {
  children: ReactNode;
  color?: 'green' | 'amber' | 'red';
}) {
  const colors = {
    green: 'border-accent-green/20 text-accent-green/80',
    amber: 'border-accent-amber/20 text-accent-amber/80',
    red: 'border-accent-red/20 text-accent-red/80',
  };
  return (
    <div className={`border-l-2 pl-3 py-1 text-[11px] ${colors[color]}`}>
      {children}
    </div>
  );
}

export function Prose({ children }: { children: ReactNode }) {
  return <div className="text-xs text-zinc-400 space-y-2">{children}</div>;
}

export function InlineCode({ children }: { children: ReactNode }) {
  return (
    <code className="text-accent-green text-[11px] font-mono bg-surface-900 px-1 py-0.5 rounded">
      {children}
    </code>
  );
}
