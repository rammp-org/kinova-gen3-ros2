// alex-style interaction touches. Include once per page after tokens.css:
//   <script src="ax-fx.js"></script>
//
// Spinner while something runs or loads:
//   axBusy(el, true, 'Homing')   button: inline spinner + label, disabled until axBusy(el, false)
//                                panel:  dims content, centers a big spinner with the label
//   or set data-busy="Homing" on the element in markup; removing the attribute clears it.
//
// Main-feature hover effect (one per screen, on the control the page exists for):
//   <button class="ax-btn primary" data-fx="sparks">Start run</button>
//   sparks  short streaks thrown outward         start, run, launch, enable motor, power
//   ripple  square rings expanding outward       connect, scan, ping, sync, capture
//   orbit   a dot racing around the border       record, stream, live, auto mode
//   Any other value falls back to sparks.
//
// Value updates:
//   axSet(el, 24.31, { digits: 2 })   tweens the number and flashes it, keeps any <small> unit
//
// Numbers in mono: any table cell holding just a number (with an optional unit), a
// timestamp, or a hex id gets class ax-n, which sets Roboto Mono. Runs on load and
// on rows added later. Alignment is left to the page; add class num to right-align.

(function () {
  const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const NS = 'http://www.w3.org/2000/svg';
  const tok = v => getComputedStyle(document.documentElement).getPropertyValue('--ax-' + v).trim();

  // ── Busy / loading ────────────────────────────────────────────
  function busy(el, on, label) {
    const isControl = el.matches('button, .ax-btn, a, input');
    if (isControl) {
      if (on) {
        if (!el.dataset.axIdle) el.dataset.axIdle = el.innerHTML;
        el.innerHTML = `<span class="ax-spin"></span>${label || el.textContent.trim()}`;
        el.disabled = true; el.setAttribute('aria-busy', 'true');
      } else if (el.dataset.axIdle !== undefined) {
        el.innerHTML = el.dataset.axIdle; delete el.dataset.axIdle;
        el.disabled = false; el.removeAttribute('aria-busy');
      }
      return;
    }
    el.querySelector(':scope > .ax-spin-wrap')?.remove();
    el.classList.toggle('ax-loading', on);
    el.toggleAttribute('aria-busy', on);
    if (on) {
      const w = document.createElement('div');
      w.className = 'ax-spin-wrap';
      w.innerHTML = `<span class="ax-spin lg"></span>${label ? `<span>${label}</span>` : ''}`;
      el.appendChild(w);
    }
  }
  window.axBusy = busy;

  function watchBusy(root) {
    root.querySelectorAll('[data-busy]').forEach(el => busy(el, true, el.dataset.busy));
    new MutationObserver(ms => ms.forEach(m => {
      if (m.attributeName === 'data-busy') busy(m.target, m.target.hasAttribute('data-busy'), m.target.dataset.busy);
    })).observe(root, { attributes: true, subtree: true, attributeFilter: ['data-busy'] });
  }

  // ── Hover effects ─────────────────────────────────────────────
  let layer;
  function getLayer() {
    if (!layer) { layer = document.createElementNS(NS, 'svg'); layer.setAttribute('class', 'ax-fx-layer'); document.body.appendChild(layer); }
    return layer;
  }
  const rand = (a, b) => a + Math.random() * (b - a);

  // A point on the element's border and the outward direction at that point
  function edgePoint(r) {
    const per = 2 * (r.width + r.height); let d = rand(0, per);
    if (d < r.width) return [r.left + d, r.top, 0, -1];
    if ((d -= r.width) < r.height) return [r.right, r.top + d, 1, 0];
    if ((d -= r.height) < r.width) return [r.right - d, r.bottom, 0, 1];
    d -= r.width; return [r.left, r.bottom - d, -1, 0];
  }

  function add(tag, attrs, frames, opts) {
    const e = document.createElementNS(NS, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    getLayer().appendChild(e);
    e.animate(frames, opts).onfinish = () => e.remove();
    return e;
  }

  const effects = {
    sparks(r, accent) {
      const [x, y, nx, ny] = edgePoint(r);
      const a = Math.atan2(ny, nx) + rand(-0.7, 0.7), dist = rand(14, 34);
      const dx = Math.cos(a), dy = Math.sin(a);
      add('path', { d: `M${x} ${y}l${dx * 6} ${dy * 6}`, stroke: Math.random() < 0.3 ? '#fff' : accent, 'stroke-width': 2, 'stroke-linecap': 'square' },
        [{ transform: 'translate(0,0)', opacity: 1 }, { transform: `translate(${dx * dist}px,${dy * dist + 6}px)`, opacity: 0 }],
        { duration: rand(300, 480), easing: 'cubic-bezier(.2,.8,.4,1)' });
    },
    ripple(r, accent) {
      add('rect', { x: r.left, y: r.top, width: r.width, height: r.height, stroke: accent, 'stroke-width': 1.5, style: 'transform-box:fill-box;transform-origin:center' },
        [{ transform: 'scale(1)', opacity: 0.9 }, { transform: 'scale(1.25, 1.6)', opacity: 0 }],
        { duration: 900, easing: 'ease-out' });
    },
    orbit(r, accent) {
      const per = 2 * (r.width + r.height);
      const e = add('rect', { x: r.left - 3, y: r.top - 3, width: r.width + 6, height: r.height + 6, stroke: accent, 'stroke-width': 2, 'stroke-dasharray': `14 ${per}` },
        [{ strokeDashoffset: 0, opacity: 1 }, { strokeDashoffset: -(per + 24), opacity: 1 }], { duration: 900, easing: 'linear' });
      e.style.filter = `drop-shadow(0 0 3px ${accent})`;
    },
  };
  const rate = { sparks: 40, ripple: 450, orbit: 900 };

  function bindFx(el) {
    if (el.dataset.fxBound) return; el.dataset.fxBound = '1';
    const kind = effects[el.dataset.fx] ? el.dataset.fx : 'sparks';
    let timer;
    const fire = n => { const r = el.getBoundingClientRect(), c = tok('accent'); for (let i = 0; i < n; i++) effects[kind](r, c); };
    el.addEventListener('mouseenter', () => {
      if (reduced || el.disabled) return;
      fire(kind === 'sparks' ? 4 : 1);
      timer = setInterval(() => fire(1), rate[kind]);
    });
    el.addEventListener('mouseleave', () => clearInterval(timer));
    el.addEventListener('click', () => { if (!reduced) fire(kind === 'sparks' ? 14 : 2); });
  }

  // ── Value updates ─────────────────────────────────────────────
  window.axSet = function (el, value, { digits = 0, ms = 400 } = {}) {
    const node = [...el.childNodes].find(n => n.nodeType === 3) || el.insertBefore(document.createTextNode(''), el.firstChild);
    const from = parseFloat(node.textContent) || 0, t0 = performance.now();
    el.classList.add('changed');
    requestAnimationFrame(() => requestAnimationFrame(() => el.classList.remove('changed')));
    const step = now => {
      const k = reduced ? 1 : Math.max(0, Math.min(1, (now - t0) / ms)), e = 1 - Math.pow(1 - k, 3);
      node.textContent = (from + (value - from) * e).toFixed(digits);
      if (k < 1) requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  };

  // ── Numbers in mono ───────────────────────────────────────────
  // 61.4 °C · -3 ms · 1,024 · 98% · 13:42:07.114 · 0x12 · 2026-10-15 · 24.1–24.3 V
  const NUMERIC = /^[-+−±~≈]?(0x[0-9a-f]+|\d[\d,.:_'–-]*)\s?(%|°[CF]?|[a-zA-Zµ°Ω/·²³]{1,6})?$/i;
  function markNumbers(root) {
    root.querySelectorAll('td:not(.ax-n), th.num:not(.ax-n)').forEach(td => {
      if (td.children.length === 0 && NUMERIC.test(td.textContent.trim())) td.classList.add('ax-n');
    });
  }

  function init() {
    markNumbers(document.body);
    new MutationObserver(ms => ms.forEach(m => m.addedNodes.forEach(n => n.nodeType === 1 && markNumbers(n.parentNode || n))))
      .observe(document.body, { childList: true, subtree: true });
    watchBusy(document.body);
    document.querySelectorAll('[data-fx]').forEach(bindFx);
    new MutationObserver(() => document.querySelectorAll('[data-fx]:not([data-fx-bound])').forEach(bindFx))
      .observe(document.body, { childList: true, subtree: true });
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init); else init();
})();
