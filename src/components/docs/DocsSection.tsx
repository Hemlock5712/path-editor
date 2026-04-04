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
    <div id={id} className="neon-panel scroll-mt-6 p-4">
      <h3 className="text-accent-green/40 mb-3 flex items-center gap-2 text-[11px] font-light tracking-wide">
        {icon}
        {title}
      </h3>
      {children}
    </div>
  );
}

export function CodeBlock({ children }: { children: string }) {
  return (
    <pre className="bg-surface-900 overflow-x-auto rounded p-3 font-mono text-[11px] leading-relaxed text-zinc-300">
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
    <div className={`border-l-2 py-1 pl-3 text-[11px] ${colors[color]}`}>
      {children}
    </div>
  );
}

export function Prose({ children }: { children: ReactNode }) {
  return <div className="space-y-2 text-xs text-zinc-400">{children}</div>;
}

export function InlineCode({ children }: { children: ReactNode }) {
  return (
    <code className="text-accent-green bg-surface-900 rounded px-1 py-0.5 font-mono text-[11px]">
      {children}
    </code>
  );
}
