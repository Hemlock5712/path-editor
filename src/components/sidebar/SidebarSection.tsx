export function SidebarSection({ title, children, delay = 0 }: { title: string; children: React.ReactNode; delay?: number }) {
  return (
    <div
      className="neon-panel p-3.5 animate-fadeIn"
      style={{ animationDelay: `${delay * 50}ms` }}
    >
      <h3 className="text-[11px] font-light tracking-wide text-accent-green/40 mb-2.5">{title}</h3>
      {children}
    </div>
  );
}
