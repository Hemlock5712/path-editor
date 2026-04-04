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
      className="neon-panel animate-fadeIn p-3.5"
      style={{ animationDelay: `${delay * 50}ms` }}
    >
      <h3 className="text-accent-green/40 mb-2.5 text-[11px] font-light tracking-wide">
        {title}
      </h3>
      {children}
    </div>
  );
}
