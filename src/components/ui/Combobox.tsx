import { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import { ChevronDown } from 'lucide-react';

export interface ComboboxOption {
  value: string;
  label: string;
  detail?: string;
}

interface ComboboxProps {
  options: ComboboxOption[];
  value: string;
  onChange: (value: string) => void;
  onCommit?: (value: string) => void;
  placeholder?: string;
  allowCustom?: boolean;
  className?: string;
}

export function Combobox({
  options,
  value,
  onChange,
  onCommit,
  placeholder,
  allowCustom = false,
  className,
}: ComboboxProps) {
  const [open, setOpen] = useState(false);
  const [highlightIndex, setHighlightIndex] = useState(-1);
  const containerRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  const listRef = useRef<HTMLDivElement>(null);

  const filtered = useMemo(() => {
    if (!value.trim()) return options;
    const lower = value.toLowerCase();
    return options.filter(
      (o) =>
        o.label.toLowerCase().includes(lower) ||
        (o.detail && o.detail.toLowerCase().includes(lower))
    );
  }, [options, value]);

  // Reset highlight when filtered list changes
  useEffect(() => {
    setHighlightIndex(-1);
  }, [filtered.length]);

  // Scroll highlighted item into view
  useEffect(() => {
    if (highlightIndex >= 0 && listRef.current) {
      const items = listRef.current.querySelectorAll('[role="option"]');
      items[highlightIndex]?.scrollIntoView({ block: 'nearest' });
    }
  }, [highlightIndex]);

  // Click outside to close
  useEffect(() => {
    function handleMouseDown(e: MouseEvent) {
      if (
        containerRef.current &&
        !containerRef.current.contains(e.target as Node)
      ) {
        setOpen(false);
      }
    }
    document.addEventListener('mousedown', handleMouseDown);
    return () => document.removeEventListener('mousedown', handleMouseDown);
  }, []);

  const commit = useCallback(
    (val: string) => {
      const trimmed = val.trim();
      if (!trimmed) return;
      if (!allowCustom) {
        const match = options.find(
          (o) => o.value.toLowerCase() === trimmed.toLowerCase()
        );
        if (!match) return;
        onCommit?.(match.value);
      } else {
        onCommit?.(trimmed);
      }
      onChange('');
      setOpen(false);
      setHighlightIndex(-1);
    },
    [allowCustom, onChange, onCommit, options]
  );

  const selectOption = useCallback(
    (opt: ComboboxOption) => {
      if (onCommit) {
        onChange('');
        onCommit(opt.value);
      } else {
        onChange(opt.value);
      }
      setOpen(false);
      setHighlightIndex(-1);
      inputRef.current?.focus();
    },
    [onChange, onCommit]
  );

  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      if (e.key === 'ArrowDown') {
        e.preventDefault();
        if (!open) {
          setOpen(true);
          setHighlightIndex(0);
        } else {
          setHighlightIndex((i) => Math.min(i + 1, filtered.length - 1));
        }
      } else if (e.key === 'ArrowUp') {
        e.preventDefault();
        setHighlightIndex((i) => Math.max(i - 1, 0));
      } else if (e.key === 'Enter') {
        e.preventDefault();
        if (open && highlightIndex >= 0 && highlightIndex < filtered.length) {
          selectOption(filtered[highlightIndex]);
        } else if (onCommit) {
          commit(value);
        }
      } else if (e.key === 'Escape') {
        e.preventDefault();
        setOpen(false);
        setHighlightIndex(-1);
      }
    },
    [open, highlightIndex, filtered, selectOption, onCommit, commit, value]
  );

  return (
    <div ref={containerRef} className={`relative ${className ?? ''}`}>
      <div className="flex items-center">
        <input
          ref={inputRef}
          type="text"
          role="combobox"
          aria-expanded={open}
          aria-haspopup="listbox"
          aria-autocomplete="list"
          value={value}
          onChange={(e) => {
            onChange(e.target.value);
            setOpen(true);
          }}
          onFocus={() => setOpen(true)}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          className="flex-1"
        />
        <button
          type="button"
          tabIndex={-1}
          className="btn-ghost -ml-6 px-1 py-1 text-zinc-400 hover:text-zinc-200"
          onClick={() => {
            setOpen((o) => !o);
            inputRef.current?.focus();
          }}
          aria-label="Toggle options"
        >
          <ChevronDown
            size={12}
            className={`transition-transform ${open ? 'rotate-180' : ''}`}
          />
        </button>
      </div>

      {open && filtered.length > 0 && (
        <div
          ref={listRef}
          role="listbox"
          className="absolute z-50 mt-1 max-h-48 w-full overflow-y-auto rounded-lg border shadow-xl"
          style={{
            background: 'rgba(5, 5, 5, 0.95)',
            borderColor: 'rgba(0, 255, 170, 0.1)',
            boxShadow:
              '0 4px 24px rgba(0, 0, 0, 0.6), 0 0 16px rgba(0, 255, 170, 0.05)',
          }}
        >
          {filtered.map((opt, i) => (
            <button
              key={opt.value}
              role="option"
              aria-selected={i === highlightIndex}
              className="flex w-full cursor-pointer flex-col px-3 py-1.5 text-left font-mono text-[12px] transition-colors"
              style={{
                background:
                  i === highlightIndex
                    ? 'rgba(0, 255, 170, 0.08)'
                    : 'transparent',
                color: 'rgba(228, 228, 231, 0.85)',
                border: 'none',
                outline: 'none',
              }}
              onMouseEnter={() => setHighlightIndex(i)}
              onMouseDown={(e) => {
                e.preventDefault(); // Prevent input blur
                selectOption(opt);
              }}
            >
              <span>{opt.label}</span>
              {opt.detail && (
                <span className="text-[11px] text-zinc-500">{opt.detail}</span>
              )}
            </button>
          ))}
        </div>
      )}
    </div>
  );
}
