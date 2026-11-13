# MicroDuck Subsystem Inspector: tooltips, HUD bar and explanations

I can't see your actual files, so everything is **attribute-driven** (`data-info="key"`) and uses **event delegation**. That means ToF cells and joint rows that `simulator.js` generates dynamically get tooltips with no extra wiring. Selectors in the `AUTO_TAG` map are guesses, so adjust them to your real IDs. Also check the **Setting R / Soft Hold / Deadman Clamp** wording against your firmware's actual behavior. All explanation text lives in the single `INFO` registry, so it's easy to correct.

---

## A) `styles.css` (append at the end)

```css
/* ==========================================================================
   MicroDuck Subsystem Inspector: tooltips + HUD bar
   ========================================================================== */
:root {
  --md-tip-bg: rgba(15, 23, 42, 0.95);
  --md-tip-border: rgba(56, 189, 248, 0.3);
  --md-text: #e2e8f0;
  --md-muted: #94a3b8;
  --md-tone: #38bdf8;               /* overridden per-badge via data-tone */
}

/* ---------- Tones (set on tooltip + inspector) ---------- */
[data-tone="cyan"]   { --md-tone: #38bdf8; }
[data-tone="green"]  { --md-tone: #4ade80; }
[data-tone="amber"]  { --md-tone: #fbbf24; }
[data-tone="red"]    { --md-tone: #f87171; }
[data-tone="violet"] { --md-tone: #a78bfa; }
[data-tone="pink"]   { --md-tone: #f472b6; }

/* ---------- Inspectable elements ---------- */
[data-info], [data-tooltip] { cursor: help; }
button[data-info], a[data-info], input[data-info], select[data-info],
[role="button"][data-info] { cursor: pointer; }
input[type="range"][data-info] { cursor: ew-resize; }

.md-inspected {
  outline: 1.5px solid var(--md-tone, #38bdf8);
  outline-offset: 2px;
  box-shadow: 0 0 0 4px color-mix(in srgb, var(--md-tone, #38bdf8) 18%, transparent);
  transition: outline-color .15s, box-shadow .15s;
}
:focus-visible[data-info] { outline: 2px solid #38bdf8; outline-offset: 2px; }

/* ---------- Floating glass tooltip ---------- */
.md-tooltip {
  position: fixed; left: 0; top: 0; z-index: 10000;
  width: max-content; max-width: 340px; min-width: 220px;
  padding: 12px 14px 13px;
  border-radius: 12px;
  background: var(--md-tip-bg);
  border: 1px solid var(--md-tip-border);
  -webkit-backdrop-filter: blur(14px) saturate(140%);
  backdrop-filter: blur(14px) saturate(140%);
  box-shadow:
    0 14px 40px rgba(2, 6, 23, .65),
    0 0 26px rgba(56, 189, 248, .12),
    inset 0 1px 0 rgba(255, 255, 255, .05);
  color: var(--md-text);
  font: 12.5px/1.5 Inter, "Segoe UI", system-ui, sans-serif;
  pointer-events: none;
  opacity: 0; visibility: hidden;
  translate: 0 5px; scale: .97;
  transition: opacity .16s ease, translate .16s ease, scale .16s ease, visibility 0s .16s;
}
.md-tooltip[data-place="bottom"] { translate: 0 -5px; }
.md-tooltip.is-visible {
  opacity: 1; visibility: visible; translate: 0 0; scale: 1;
  transition-delay: 0s;
}
.md-tooltip::after {            /* arrow */
  content: ""; position: absolute; left: var(--arrow-x, 50%);
  width: 10px; height: 10px; margin-left: -5px;
  background: var(--md-tip-bg);
  border: 1px solid var(--md-tip-border);
  transform: rotate(45deg);
}
.md-tooltip[data-place="top"]::after    { bottom: -6px; border-top: 0; border-left: 0; }
.md-tooltip[data-place="bottom"]::after { top: -6px;    border-bottom: 0; border-right: 0; }

.md-badge {
  display: inline-block; padding: 2px 8px; border-radius: 999px;
  font: 700 10px/1.5 "JetBrains Mono", ui-monospace, Consolas, monospace;
  letter-spacing: .06em; text-transform: uppercase;
  color: var(--md-tone);
  background: color-mix(in srgb, var(--md-tone) 14%, transparent);
  border: 1px solid color-mix(in srgb, var(--md-tone) 45%, transparent);
}
.md-badge::before { content: "["; opacity: .6; margin-right: 2px; }
.md-badge::after  { content: "]"; opacity: .6; margin-left: 2px; }

.md-title { margin: 7px 0 4px; font-weight: 700; font-size: 13.5px; color: #f8fafc; letter-spacing: .01em; }
.md-text  { margin: 0; color: #cbd5e1; }
.md-facts { margin: 8px 0 0; padding: 0; list-style: none; display: flex; flex-wrap: wrap; gap: 5px; }
.md-facts li {
  padding: 1px 7px; border-radius: 6px; font: 11px/1.6 "JetBrains Mono", ui-monospace, monospace;
  color: var(--md-muted); background: rgba(148, 163, 184, .1); border: 1px solid rgba(148, 163, 184, .18);
}
.md-live {
  margin-top: 8px; padding-top: 7px; border-top: 1px dashed rgba(56, 189, 248, .25);
  font: 11.5px/1.4 "JetBrains Mono", ui-monospace, monospace; color: var(--md-tone);
}
.md-live:empty { display: none; }
.md-live::before { content: "● LIVE  "; font-size: 9.5px; letter-spacing: .1em; opacity: .75