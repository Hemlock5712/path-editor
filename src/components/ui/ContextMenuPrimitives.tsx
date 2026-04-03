import { useState } from 'react';

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
  const baseColor = 'rgba(228, 228, 231, 0.85)';

  return (
    <div
      className="relative"
      onMouseEnter={() => setOpen(true)}
      onMouseLeave={() => setOpen(false)}
    >
      <button
        className="w-full flex items-center gap-2 px-3 py-1.5 text-left text-sm font-mono transition-colors"
        style={{
          color: baseColor,
          cursor: 'pointer',
          background: 'transparent',
          border: 'none',
          outline: 'none',
        }}
        onMouseEnter={(e) => {
          (e.currentTarget as HTMLElement).style.background = 'rgba(0, 255, 170, 0.05)';
        }}
        onMouseLeave={(e) => {
          (e.currentTarget as HTMLElement).style.background = 'transparent';
        }}
      >
        <span className="flex-shrink-0 opacity-70">{icon}</span>
        <span className="flex-1">{label}</span>
        <span className="opacity-40 text-xs">&#9656;</span>
      </button>

      {open && (
        <div
          className="absolute left-full top-0 ml-0.5 min-w-[160px] rounded-lg overflow-hidden shadow-xl border"
          style={{
            background: 'rgba(5, 5, 5, 0.95)',
            borderColor: 'rgba(0, 255, 170, 0.1)',
            boxShadow: '0 4px 24px rgba(0, 0, 0, 0.6), 0 0 16px rgba(0, 255, 170, 0.05)',
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

export function MenuItem({ icon, label, onClick, disabled = false, destructive = false }: MenuItemProps) {
  const baseColor = destructive ? 'rgba(255, 51, 102, 0.9)' : 'rgba(228, 228, 231, 0.85)';
  const disabledColor = 'rgba(255, 255, 255, 0.25)';

  return (
    <button
      className="w-full flex items-center gap-2 px-3 py-1.5 text-left text-sm font-mono transition-colors"
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
          (e.currentTarget as HTMLElement).style.background = 'rgba(0, 255, 170, 0.05)';
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
