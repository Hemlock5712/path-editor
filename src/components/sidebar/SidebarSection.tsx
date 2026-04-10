export function SidebarSection({
  title,
  children,
  delay = 0,
}: {
  title: string;
  children: React.ReactNode;
  delay?: number;
}) {
  return (
    <div
      className="neon-panel animate-fadeIn relative overflow-visible p-4"
      style={{ animationDelay: `${delay * 50}ms` }}
    >
      <div className="absolute top-0 left-0 h-8 w-8 border-t border-l border-accent-green/14" />
      <div className="mb-3">
        <div className="mb-2 flex items-center justify-between gap-3">
          <h3 className="text-xs font-semibold tracking-[0.08em] text-zinc-100 uppercase">
            {title}
          </h3>
          <span className="font-mono text-[11px] text-zinc-400">
            {(delay + 1).toString().padStart(2, '0')}
          </span>
        </div>
        <div className="section-rule" />
      </div>
      {children}
    </div>
  );
}
