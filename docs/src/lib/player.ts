// Plays the scenes built in scenes.ts: a field drawn on a canvas, a robot moving along its recorded run,
// an optional strip chart, and a play bar. One script for every figure on the page.

import type { Chart, Line, Mark, Scene, Shape, Track, Variant } from './scenes.ts';

type Palette = Record<'bg' | 'bg2' | 'bg3' | 'tx' | 'tx2' | 'tx3' | 'rule' | 'ruleSoft' | 'accent' | 'lift' | 'ok', string>;

const REVEAL_SECONDS = 0.4;
const HOLD_SECONDS = 1.4; // the pause on the last frame before the scene starts over
const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');

function palette(el: Element): Palette {
  const css = getComputedStyle(el);
  const v = (name: string) => css.getPropertyValue(name).trim();
  return {
    bg: v('--bg'),
    bg2: v('--bg2'),
    bg3: v('--bg3'),
    tx: v('--tx'),
    tx2: v('--tx2'),
    tx3: v('--tx3'),
    rule: v('--rule'),
    ruleSoft: v('--rule-soft'),
    accent: v('--accent'),
    lift: v('--lift'),
    ok: v('--ok'),
  };
}

const clamp01 = (x: number) => Math.max(0, Math.min(1, x));

/** A value from samples every `dt`, at time `t` (held at both ends). */
function at(samples: number[], dt: number, t: number): number {
  if (samples.length === 0) return 0;
  const f = Math.max(0, Math.min(samples.length - 1, t / dt));
  const i = Math.floor(f);
  const j = Math.min(samples.length - 1, i + 1);
  return samples[i]! + (samples[j]! - samples[i]!) * (f - i);
}

/** The same for an angle, turning the short way between samples. */
function angleAt(samples: number[], dt: number, t: number): number {
  if (samples.length === 0) return 0;
  const f = Math.max(0, Math.min(samples.length - 1, t / dt));
  const i = Math.floor(f);
  const j = Math.min(samples.length - 1, i + 1);
  let d = samples[j]! - samples[i]!;
  d = Math.atan2(Math.sin(d), Math.cos(d));
  return samples[i]! + d * (f - i);
}

class Player {
  private readonly scene: Scene;
  private readonly stage: HTMLCanvasElement;
  private readonly chart: HTMLCanvasElement | null;
  private readonly range: HTMLInputElement;
  private readonly button: HTMLButtonElement;
  private readonly clock: HTMLElement;
  private readonly step: HTMLElement | null;
  private readonly facts: HTMLElement | null;
  private readonly tabs: HTMLButtonElement[];
  private colours: Palette;
  private variant = 0;
  private t = 0;
  private playing = false;
  private visible = false;
  private userPaused = false;
  private last = 0;
  private held = 0;
  private frame = 0;

  constructor(private readonly root: HTMLElement) {
    this.scene = JSON.parse(root.querySelector('script[type="application/json"]')!.textContent!) as Scene;
    this.stage = root.querySelector<HTMLCanvasElement>('canvas.scene-stage')!;
    this.chart = root.querySelector<HTMLCanvasElement>('canvas.scene-chart');
    this.range = root.querySelector<HTMLInputElement>('input[type="range"]')!;
    this.button = root.querySelector<HTMLButtonElement>('button.scene-play')!;
    this.clock = root.querySelector<HTMLElement>('.scene-clock')!;
    this.step = root.querySelector<HTMLElement>('.scene-step');
    this.facts = root.querySelector<HTMLElement>('.scene-facts');
    this.tabs = [...root.querySelectorAll<HTMLButtonElement>('.scene-tabs button')];
    this.colours = palette(root);

    this.tabs.forEach((tab, i) => tab.addEventListener('click', () => this.choose(i)));
    this.button.addEventListener('click', () => {
      if (this.playing) {
        this.userPaused = true;
        this.pause();
      } else {
        this.userPaused = false;
        if (this.t >= this.current.seconds) this.t = 0;
        this.play();
      }
    });
    this.range.addEventListener('input', () => {
      this.userPaused = true;
      this.pause();
      this.t = Number(this.range.value);
      this.draw();
    });

    new ResizeObserver(() => this.draw()).observe(this.stage.parentElement!);
    new MutationObserver(() => {
      this.colours = palette(root);
      this.draw();
    }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    new IntersectionObserver(
      ([entry]) => {
        this.visible = entry!.isIntersecting;
        if (this.visible && !this.userPaused && !reducedMotion.matches) this.play();
        else if (!this.visible) this.pause();
      },
      { threshold: 0.35 },
    ).observe(root);

    this.choose(0);
    // with reduced motion the scene opens on its last frame and plays only when asked
    if (reducedMotion.matches) this.t = this.current.seconds;
    this.draw();
  }

  private get current(): Variant {
    return this.scene.variants[this.variant]!;
  }

  private choose(i: number): void {
    this.variant = i;
    this.t = 0;
    this.held = 0;
    this.tabs.forEach((tab, k) => tab.setAttribute('aria-pressed', String(k === i)));
    this.range.max = String(this.current.seconds);
    if (this.facts) {
      this.facts.replaceChildren(
        ...(this.current.facts ?? []).map((f) => {
          const item = document.createElement('span');
          const label = document.createElement('span');
          label.textContent = f.label;
          const value = document.createElement('b');
          value.textContent = f.value;
          item.append(label, value);
          return item;
        }),
      );
    }
    this.draw();
    if (this.visible && !this.userPaused && !reducedMotion.matches) this.play();
  }

  private play(): void {
    if (this.playing) return;
    this.playing = true;
    this.root.dataset.playing = 'true';
    this.button.setAttribute('aria-label', 'Pause');
    this.last = performance.now();
    this.frame = requestAnimationFrame(this.tick);
  }

  private pause(): void {
    this.playing = false;
    this.root.dataset.playing = 'false';
    this.button.setAttribute('aria-label', 'Play');
    cancelAnimationFrame(this.frame);
  }

  private readonly tick = (now: number): void => {
    if (!this.playing) return;
    const dt = Math.min(0.1, (now - this.last) / 1000);
    this.last = now;
    if (this.t >= this.current.seconds) {
      this.held += dt;
      if (this.held >= HOLD_SECONDS) {
        this.held = 0;
        this.t = 0;
      }
    } else {
      this.t = Math.min(this.current.seconds, this.t + dt);
    }
    this.draw();
    this.frame = requestAnimationFrame(this.tick);
  };

  // ------------------------------------------------------------------------------------------ drawing

  private draw(): void {
    const v = this.current;
    this.range.value = String(this.t);
    this.clock.textContent = `${this.t.toFixed(1)} / ${v.seconds.toFixed(1)} s`;
    if (this.step) {
      const steps = v.steps ?? [];
      let text = '';
      for (const s of steps) if (s.at <= this.t + 1e-9) text = s.text;
      if (this.step.textContent !== text) this.step.textContent = text;
    }
    this.drawStage(v);
    if (this.chart) this.drawChart(v.chart);
  }

  /** Sizes a canvas to its box at the screen's pixel density; returns the context in CSS pixels. */
  private fit(canvas: HTMLCanvasElement, height: number): [CanvasRenderingContext2D, number, number] {
    const width = canvas.parentElement!.clientWidth;
    const ratio = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
      canvas.width = Math.round(width * ratio);
      canvas.height = Math.round(height * ratio);
      canvas.style.height = `${height}px`;
    }
    const ctx = canvas.getContext('2d')!;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    return [ctx, width, height];
  }

  private drawStage(v: Variant): void {
    const [x0, y0, x1, y1] = this.scene.view;
    const width = this.stage.parentElement!.clientWidth;
    if (width === 0) return;
    const scale = width / (x1 - x0);
    const [ctx, , height] = this.fit(this.stage, Math.round((y1 - y0) * scale));
    const c = this.colours;
    const px = (x: number) => (x - x0) * scale;
    const py = (y: number) => height - (y - y0) * scale;
    const shown = (when: number | undefined) => clamp01((this.t - (when ?? 0)) / REVEAL_SECONDS + (when ? 0 : 1));
    const small = width < 520;
    const font = `${small ? 9 : 10.5}px ${getComputedStyle(this.root).getPropertyValue('--sl-font-mono') || 'monospace'}, monospace`;

    // a dot every metre
    ctx.fillStyle = c.ruleSoft;
    for (let x = Math.ceil(x0); x <= x1; x++) for (let y = Math.ceil(y0); y <= y1; y++) ctx.fillRect(px(x) - 1, py(y) - 1, 2, 2);

    // the field's walls
    if (this.scene.field) {
      const [length, wide] = this.scene.field;
      ctx.strokeStyle = c.rule;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(px(-length / 2), py(wide / 2), length * scale, wide * scale);
    }

    for (const shape of v.shapes) this.drawShape(ctx, shape, px, py, shown(shape.at), font);
    for (const line of v.lines) this.drawLine(ctx, line, px, py, shown(line.at), font);
    for (const mark of v.marks) this.drawMark(ctx, mark, px, py, shown(mark.at), font, v, false);
    for (const track of v.tracks) this.drawTrack(ctx, track, px, py, scale, v, font);
    for (const mark of v.marks) this.drawMark(ctx, mark, px, py, shown(mark.at), font, v, true);

    // a one-metre scale bar
    const bx = width - 14 - scale;
    const by = height - 12;
    ctx.strokeStyle = c.tx3;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(bx, by - 3);
    ctx.lineTo(bx, by);
    ctx.lineTo(bx + scale, by);
    ctx.lineTo(bx + scale, by - 3);
    ctx.stroke();
    ctx.fillStyle = c.tx3;
    ctx.font = font;
    ctx.textAlign = 'center';
    ctx.fillText('1 M', bx + scale / 2, by - 5);
    ctx.textAlign = 'left';
  }

  private drawShape(ctx: CanvasRenderingContext2D, s: Shape, px: (x: number) => number, py: (y: number) => number, alpha: number, font: string): void {
    if (alpha <= 0) return;
    const c = this.colours;
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.beginPath();
    s.pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(px(x), py(y)) : ctx.lineTo(px(x), py(y))));
    ctx.closePath();
    if (s.style === 'obstacle') {
      ctx.fillStyle = c.bg3;
      ctx.fill();
      ctx.strokeStyle = c.tx3;
      ctx.lineWidth = 1.25;
      ctx.stroke();
    } else if (s.style === 'grown') {
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = c.tx3;
      ctx.lineWidth = 1;
      ctx.stroke();
    } else {
      ctx.fillStyle = c.accent;
      ctx.globalAlpha = alpha * 0.12;
      ctx.fill();
    }
    ctx.restore();
    if (s.label) {
      ctx.save();
      ctx.globalAlpha = alpha;
      ctx.font = font;
      ctx.fillStyle = c.tx3;
      ctx.fillText(s.label.toUpperCase(), px(s.pts[0]![0]) + 4, py(s.pts[0]![1]) - 5);
      ctx.restore();
    }
  }

  private drawLine(ctx: CanvasRenderingContext2D, l: Line, px: (x: number) => number, py: (y: number) => number, alpha: number, font: string): void {
    if (alpha <= 0 || l.pts.length < 2) return;
    const c = this.colours;
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    if (l.v) {
      // coloured by planned speed: the cool hue when slow, the accent when fast
      ctx.lineWidth = 3.5;
      for (let i = 1; i < l.pts.length; i++) {
        const share = Math.round(clamp01((l.v[i]! + l.v[i - 1]!) / 2 / this.scene.speedMax) * 100);
        ctx.strokeStyle = `color-mix(in oklch, ${c.accent} ${share}%, ${c.lift})`;
        ctx.beginPath();
        ctx.moveTo(px(l.pts[i - 1]![0]), py(l.pts[i - 1]![1]));
        ctx.lineTo(px(l.pts[i]![0]), py(l.pts[i]![1]));
        ctx.stroke();
      }
    } else {
      ctx.beginPath();
      l.pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(px(x), py(y)) : ctx.lineTo(px(x), py(y))));
      if (l.style === 'route') {
        ctx.strokeStyle = c.tx2;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([2, 5]);
      } else if (l.style === 'rail') {
        ctx.strokeStyle = c.lift;
        ctx.lineWidth = 2;
      } else if (l.style === 'driver') {
        ctx.strokeStyle = c.tx3;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 5]);
      } else {
        ctx.strokeStyle = c.rule;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 6]);
      }
      ctx.stroke();
      if (l.style === 'route') {
        ctx.setLineDash([]);
        ctx.fillStyle = c.tx2;
        for (const [x, y] of l.pts) {
          ctx.beginPath();
          ctx.arc(px(x), py(y), 2.5, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }
    if (l.label) {
      const a = l.pts[Math.floor((l.pts.length - 1) / 2)]!;
      const b = l.pts[Math.ceil((l.pts.length - 1) / 2)]!;
      const x = (a[0] + b[0]) / 2;
      const y = (a[1] + b[1]) / 2;
      ctx.font = font;
      ctx.fillStyle = l.style === 'rail' ? c.lift : c.tx3;
      ctx.fillText(l.label.toUpperCase(), px(x) + 6, py(y) - 7);
    }
    ctx.restore();
  }

  private drawMark(
    ctx: CanvasRenderingContext2D,
    m: Mark,
    px: (x: number) => number,
    py: (y: number) => number,
    alpha: number,
    font: string,
    v: Variant,
    labels: boolean,
  ): void {
    if (alpha <= 0 || (labels && !m.label)) return;
    const c = this.colours;
    const local = Math.max(0, this.t - v.driveAt);
    const x = px(m.xs && m.dt ? at(m.xs, m.dt, local) : m.x);
    const y = py(m.ys && m.dt ? at(m.ys, m.dt, local) : m.y);
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.font = font;
    let labelColour = c.tx3;
    if (labels) {
      const fired = m.style === 'event' && m.firesAt !== undefined && local >= m.firesAt;
      ctx.fillStyle = m.style === 'goal' || fired ? c.accent : m.style === 'event' || (m.style === 'tag' && !m.faint) ? c.tx2 : c.tx3;
      // "above" and "below" clear a robot parked on the mark; the default sits beside it
      const clear = (Math.max(...this.scene.robot) / 2) * ((this.stage.clientWidth || 1) / (this.scene.view[2] - this.scene.view[0])) + 14;
      if (m.style === 'tag' && (m.side === 'above' || m.side === 'below')) {
        ctx.textAlign = 'center';
        ctx.fillText(m.label!.toUpperCase(), x, m.side === 'below' ? y + 30 : y - 22);
      } else if (m.side === 'above' || m.side === 'below') {
        ctx.textAlign = 'center';
        ctx.fillText(m.label!.toUpperCase(), x, m.side === 'below' ? y + clear + 8 : y - clear);
      } else {
        ctx.textAlign = m.side === 'left' ? 'right' : 'left';
        ctx.fillText(m.label!.toUpperCase(), x + (m.side === 'left' ? -11 : 11), y - 9);
      }
      ctx.restore();
      return;
    }
    if (m.style === 'waypoint') {
      ctx.fillStyle = c.bg;
      ctx.strokeStyle = c.accent;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    } else if (m.style === 'event') {
      const since = m.firesAt === undefined ? -1 : local - m.firesAt;
      const fired = since >= 0;
      if (fired && since < 0.7) {
        ctx.strokeStyle = c.accent;
        ctx.globalAlpha = alpha * (1 - since / 0.7);
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, y, 6 + since * 34, 0, Math.PI * 2);
        ctx.stroke();
        ctx.globalAlpha = alpha;
      }
      ctx.fillStyle = fired ? c.accent : c.bg;
      ctx.strokeStyle = fired ? c.accent : c.tx2;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x, y - 6);
      ctx.lineTo(x + 6, y);
      ctx.lineTo(x, y + 6);
      ctx.lineTo(x - 6, y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      labelColour = fired ? c.accent : c.tx2;
    } else if (m.style === 'goal') {
      ctx.strokeStyle = c.accent;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x, y, 7, 0, Math.PI * 2);
      ctx.moveTo(x - 11, y);
      ctx.lineTo(x + 11, y);
      ctx.moveTo(x, y - 11);
      ctx.lineTo(x, y + 11);
      ctx.stroke();
      labelColour = c.accent;
    } else if (m.style === 'piece') {
      ctx.fillStyle = c.accent;
      ctx.beginPath();
      ctx.arc(x, y, 6, 0, Math.PI * 2);
      ctx.fill();
    } else if (m.style === 'tag') {
      ctx.translate(x, y);
      ctx.rotate(-(m.h ?? Math.PI)); // the canvas's y runs down
      ctx.fillStyle = m.faint ? c.tx3 : c.tx;
      ctx.strokeStyle = m.faint ? c.tx3 : c.tx;
      ctx.globalAlpha = alpha * (m.faint ? 0.6 : 1);
      ctx.fillRect(-3, -13, 4, 26); // the plate, edge on
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 3]);
      ctx.beginPath(); // the way it faces
      ctx.moveTo(1, 0);
      ctx.lineTo(22, 0);
      ctx.stroke();
    } else {
      ctx.fillStyle = c.tx3;
      ctx.beginPath();
      ctx.arc(x, y, 3, 0, Math.PI * 2);
      ctx.fill();
    }
    void labelColour;
    ctx.restore();
  }

  private drawTrack(
    ctx: CanvasRenderingContext2D,
    tr: Track,
    px: (x: number) => number,
    py: (y: number) => number,
    scale: number,
    v: Variant,
    font: string,
  ): void {
    const c = this.colours;
    const local = Math.max(0, this.t - v.driveAt);
    const x = px(at(tr.x, tr.dt, local));
    const y = py(at(tr.y, tr.dt, local));
    const heading = angleAt(tr.h, tr.dt, local);
    const [length, wide] = this.scene.robot;

    // where it has been
    if (tr.kind !== 'dot') {
      const upTo = Math.min(tr.x.length - 1, Math.floor(local / tr.dt));
      ctx.save();
      ctx.strokeStyle = tr.kind === 'ghost' ? c.tx3 : c.tx;
      ctx.globalAlpha = tr.kind === 'ghost' ? 0.5 : 0.55;
      ctx.lineWidth = 1.25;
      if (tr.kind === 'ghost') ctx.setLineDash([3, 4]);
      ctx.beginPath();
      for (let i = 0; i <= upTo; i++) (i === 0 ? ctx.moveTo(px(tr.x[i]!), py(tr.y[i]!)) : ctx.lineTo(px(tr.x[i]!), py(tr.y[i]!)));
      ctx.lineTo(x, y);
      ctx.stroke();
      ctx.restore();
    }

    ctx.save();
    ctx.translate(x, y);
    if (tr.kind === 'dot') {
      ctx.fillStyle = c.tx2;
      ctx.beginPath();
      ctx.arc(0, 0, 4, 0, Math.PI * 2);
      ctx.fill();
    } else {
      ctx.rotate(-heading); // the canvas's y runs down
      const l = length * scale;
      const w = wide * scale;
      if (tr.kind === 'ghost') {
        ctx.setLineDash([3, 4]);
        ctx.strokeStyle = c.tx3;
        ctx.lineWidth = 1.25;
        ctx.strokeRect(-l / 2, -w / 2, l, w);
      } else {
        ctx.fillStyle = `color-mix(in oklch, ${c.bg} 72%, transparent)`;
        ctx.strokeStyle = c.tx;
        ctx.lineWidth = 1.75;
        ctx.beginPath();
        ctx.roundRect(-l / 2, -w / 2, l, w, 3);
        ctx.fill();
        ctx.stroke();
        // the front: a filled wedge, so heading reads at a glance
        ctx.fillStyle = c.accent;
        ctx.beginPath();
        ctx.moveTo(l / 2 - 1, 0);
        ctx.lineTo(l / 2 - 1 - l * 0.26, -w * 0.2);
        ctx.lineTo(l / 2 - 1 - l * 0.26, w * 0.2);
        ctx.closePath();
        ctx.fill();
      }
    }
    ctx.restore();
    if (tr.label) {
      ctx.save();
      ctx.font = font;
      ctx.fillStyle = tr.kind === 'ghost' ? c.tx3 : c.tx2;
      ctx.fillText(tr.label.toUpperCase(), x + (length * scale) / 2 + 6, y + 4);
      ctx.restore();
    }
  }

  private drawChart(chart: Chart | undefined): void {
    const canvas = this.chart!;
    canvas.hidden = !chart;
    if (!chart) return;
    const [ctx, width, height] = this.fit(canvas, 120);
    const c = this.colours;
    const v = this.current;
    const left = 34;
    const right = 10;
    const top = 10;
    const bottom = 20;
    const span = Math.max(...chart.series.map((s) => (s.y.length - 1) * chart.dt));
    const px = (t: number) => left + (t / span) * (width - left - right);
    const py = (y: number) => height - bottom - (y / chart.yMax) * (height - top - bottom);
    const mono = getComputedStyle(this.root).getPropertyValue('--sl-font-mono') || 'monospace';
    ctx.font = `10px ${mono}, monospace`;

    ctx.strokeStyle = c.ruleSoft;
    ctx.fillStyle = c.tx3;
    ctx.lineWidth = 1;
    for (let y = 0; y <= chart.yMax; y += chart.yMax > 4 ? 2 : 1) {
      ctx.beginPath();
      ctx.moveTo(left, py(y));
      ctx.lineTo(width - right, py(y));
      ctx.stroke();
      ctx.textAlign = 'right';
      ctx.fillText(String(y), left - 6, py(y) + 3);
    }
    ctx.textAlign = 'left';
    ctx.fillText(chart.yLabel.toUpperCase(), 2, 9);
    for (let t = 0; t <= span; t += span > 6 ? 2 : 1) ctx.fillText(`${t} S`, px(t) - 6, height - 5);

    let labelX = left + 8;
    for (const s of chart.series) {
      ctx.save();
      ctx.beginPath();
      s.y.forEach((y, i) => (i === 0 ? ctx.moveTo(px(0), py(y)) : ctx.lineTo(px(i * chart.dt), py(y))));
      ctx.lineJoin = 'round';
      if (s.style === 'plan') {
        ctx.strokeStyle = c.lift;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 4]);
      } else if (s.style === 'limit') {
        ctx.strokeStyle = c.tx3;
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 4]);
      } else {
        ctx.strokeStyle = c.accent;
        ctx.lineWidth = 2;
      }
      ctx.stroke();
      // the legend, along the top
      ctx.beginPath();
      ctx.moveTo(labelX, top + 3);
      ctx.lineTo(labelX + 16, top + 3);
      ctx.stroke();
      ctx.restore();
      ctx.fillStyle = c.tx2;
      ctx.fillText(s.label.toUpperCase(), labelX + 21, top + 7);
      labelX += 30 + ctx.measureText(s.label.toUpperCase()).width;
    }

    // now
    const local = Math.max(0, Math.min(span, this.t - v.driveAt));
    ctx.strokeStyle = c.tx;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(px(local), top);
    ctx.lineTo(px(local), height - bottom);
    ctx.stroke();
    const actual = chart.series.find((s) => s.style === 'actual');
    if (actual) {
      ctx.fillStyle = c.accent;
      ctx.beginPath();
      ctx.arc(px(local), py(at(actual.y, chart.dt, local)), 3.5, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}

function start(): void {
  for (const root of document.querySelectorAll<HTMLElement>('figure.scene:not([data-ready])')) {
    root.dataset.ready = 'true';
    new Player(root);
  }
}

if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
else start();
