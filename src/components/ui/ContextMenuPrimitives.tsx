import { useState, useRef } from 'react';

// ─── Menu Separator ────────────────────────────────────────────────────────

export function MenuSeparator() {
  return (
    <div
      style={{
        height: '1px',
        background: 'rgba(255, 255, 255, 0.08)',
        margin: '2px 0',
      }}
    />
  );
}

// ─── SubMenu Component ────────────────────────────────────────────────────

interface SubMenuProps {
  icon: React.ReactNode;
  label: string;
  children: React.ReactNode;
}

export function SubMenu({ icon, label, children }: SubMenuProps) {
  const [open, setOpen] = useState(false);
  const triggerRef = useRef<HTMLButtonElement>(null);
  const submenuRef = useRef<HTMLDivElement>(null);
  const baseColor = 'rgba(228, 228, 231, 0.85)';

  return (
    <div
      className="relative"
      onMouseEnter={() => setOpen(true)}
      onMouseLeave={() => setOpen(false)}
    >
      <button
        ref={triggerRef}
        className="flex w-full items-center gap-2 px-3 py-1.5 text-left font-mono text-sm transition-colors"
        role="menuitem"
        aria-haspopup="true"
        aria-expanded={open}
        style={{
          color: baseColor,
          cursor: 'pointer',
          background: 'transparent',
          border: 'none',
          outline: 'none',
        }}
        onMouseEnter={(e) => {
          (e.currentTarget as HTMLElement).style.background =
            'rgba(0, 255, 170, 0.05)';
        }}
        onMouseLeave={(e) => {
          (e.currentTarget as HTMLElement).style.background = 'transparent';
        }}
        onKeyDown={(e) => {
          if (e.key === 'ArrowRight') {
            e.preventDefault();
            setOpen(true);
            requestAnimationFrame(() => {
              submenuRef.current
                ?.querySelector<HTMLElement>('[role="menuitem"]')
                ?.focus();
            });
          } else if (e.key === 'ArrowLeft' || e.key === 'Escape') {
            e.preventDefault();
            setOpen(false);
            triggerRef.current?.focus();
          }
        }}
      >
        <span className="flex-shrink-0 opacity-70">{icon}</span>
        <span className="flex-1">{label}</span>
        <span className="text-xs opacity-40">&#9656;</span>
      </button>

      {open && (
        <div
          ref={submenuRef}
          className="absolute top-0 left-full ml-0.5 min-w-[160px] overflow-hidden rounded-lg border shadow-xl"
          role="menu"
          style={{
            background: 'rgba(5, 5, 5, 0.95)',
            borderColor: 'rgba(0, 255, 170, 0.1)',
            boxShadow:
              '0 4px 24px rgba(0, 0, 0, 0.6), 0 0 16px rgba(0, 255, 170, 0.05)',
          }}
        >
          {children}
        </div>
      )}
    </div>
  );
}

// ─── Menu Item Component ──────────────────────────────────────────────────

interface MenuItemProps {
  icon: React.ReactNode;
  label: string;
  onClick: () => void;
  disabled?: boolean;
  destructive?: boolean;
}

export function MenuItem({
  icon,
  label,
  onClick,
  disabled = false,
  destructive = false,
}: MenuItemProps) {
  const baseColor = destructive
    ? 'rgba(255, 51, 102, 0.9)'
    : 'rgba(228, 228, 231, 0.85)';
  const disabledColor = 'rgba(255, 255, 255, 0.25)';

  return (
    <button
      className="flex w-full items-center gap-2 px-3 py-1.5 text-left font-mono text-sm transition-colors"
      role="menuitem"
      style={{
        color: disabled ? disabledColor : baseColor,
        cursor: disabled ? 'default' : 'pointer',
        background: 'transparent',
        border: 'none',
        outline: 'none',
      }}
      onClick={disabled ? undefined : onClick}
      onMouseEnter={(e) => {
        if (!disabled) {
          (e.currentTarget as HTMLElement).style.background =
            'rgba(0, 255, 170, 0.05)';
        }
      }}
      onMouseLeave={(e) => {
        (e.currentTarget as HTMLElement).style.background = 'transparent';
      }}
      disabled={disabled}
    >
      <span className="flex-shrink-0 opacity-70">{icon}</span>
      <span>{label}</span>
    </button>
  );
}
