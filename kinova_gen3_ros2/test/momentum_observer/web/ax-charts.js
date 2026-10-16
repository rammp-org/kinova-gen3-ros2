// Dependency-free SVG charts that read alex-style tokens from CSS.
// Usage:
//   <script src="ax-charts.js"></script>
//   axLine(svgEl, { series: [{ name: 'J1', values: [...] }], x: i => `-${i}s`, yTicks: [0, 1, 2], unit: 'A',
//                   xLabel: 'Time before now, s', yLabel: 'Phase current, A' })
//   axBars(svgEl, { rows: [['Pick', 412], ...], max: 500, highlight: 'Reach', unit: 'ms',
//                   xLabel: 'Cycle time, ms', yLabel: 'Test' })
// Both draw axis titles, direct labels, a crosshair or per-bar tooltip, and redraw when called again.
// xLabel and yLabel are required. A missing one renders in red so it shows up in screenshots.

(function () {
  const NS = 'http://www.w3.org/2000/svg';
  const el = (n, a = {}, p) => { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
  const tok = v => getComputedStyle(document.documentElement).getPropertyValue('--ax-' + v).trim();
  const color = i => tok('s' + (i + 1));

  let tip;
  function showTip(ev, html) {
    if (!tip) { tip = document.createElement('div'); tip.className = 'ax-tip'; document.body.appendChild(tip); }
    tip.innerHTML = html; tip.style.display = 'block';
    tip.style.left = ev.clientX + 14 + 'px'; tip.style.top = ev.clientY + 14 + 'px';
  }
  const hideTip = () => tip && (tip.style.display = 'none');

  // Axis titles: quantity, then unit after a comma ("Phase current, A")
  function axisTitles(svg, { xLabel, yLabel, x0, y0, w, h, xGap = 34 }) {
    const miss = '⚠ missing axis label', red = `fill:${tok('critical')}`;
    if (!xLabel || !yLabel) console.error('alex-style: every chart needs xLabel and yLabel', svg);
    el('text', { class: 'ax-axis-title', x: x0 + w / 2, y: y0 + h + xGap, 'text-anchor': 'middle', style: xLabel ? '' : red }, svg).textContent = xLabel || miss;
    const ty = y0 + h / 2, tx = 11;
    el('text', { class: 'ax-axis-title', x: tx, y: ty, 'text-anchor': 'middle', transform: `rotate(-90 ${tx} ${ty})`, style: yLabel ? '' : red }, svg).textContent = yLabel || miss;
  }

  function yGrid(svg, x0, y0, w, h, lo, hi, ticks, unit) {
    ticks.forEach(t => {
      const y = y0 + h - (t - lo) / (hi - lo) * h;
      el('line', { x1: x0, x2: x0 + w, y1: y, y2: y, stroke: tok('grid'), 'stroke-dasharray': tok('grid-dash') }, svg);
      el('text', { x: x0 - 6, y: y + 4, 'text-anchor': 'end' }, svg).textContent = t + (unit || '');
    });
    el('line', { x1: x0, x2: x0 + w, y1: y0 + h, y2: y0 + h, stroke: tok('axis') }, svg);
  }

  // series: [{ name, values, color? }]; color defaults to the fixed categorical order and may be a var(--ax-sN)
  window.axLine = function (svg, { series, yTicks, x = i => i, xTicks, unit = '', width = 640, height = 260, refLine, marks, xLabel, yLabel }) {
    svg.innerHTML = '';
    svg.setAttribute('viewBox', `0 0 ${width} ${height}`);
    const n = series[0].values.length, lo = yTicks[0], hi = yTicks[yTicks.length - 1];
    const x0 = 58, y0 = 8, w = width - x0 - 46, h = height - 52;
    const X = i => x0 + i / (n - 1) * w, Y = v => y0 + h - (v - lo) / (hi - lo) * h;
    yGrid(svg, x0, y0, w, h, lo, hi, yTicks);
    axisTitles(svg, { xLabel, yLabel, x0, y0, w, h });
    (xTicks || [0, Math.round((n - 1) / 2), n - 1]).forEach(i => {
      el('text', { x: X(i), y: y0 + h + 16, 'text-anchor': 'middle' }, svg).textContent = x(i);
    });
    if (refLine) {
      const y = Y(refLine.value);
      el('line', { x1: x0, x2: x0 + w, y1: y, y2: y, stroke: tok(refLine.status || 'warning'), 'stroke-dasharray': '4 3' }, svg);
      el('text', { x: x0 + 4, y: y - 5 }, svg).textContent = refLine.label;
    }
    // Local extension (momentum_observer): vertical event marks, e.g. tap times. [{ i, label }]
    (marks || []).forEach(m => {
      el('line', { x1: X(m.i), x2: X(m.i), y1: y0, y2: y0 + h, stroke: tok('warning'), 'stroke-dasharray': '4 3' }, svg);
      if (m.label) el('text', { x: X(m.i) + 4, y: y0 + 10, style: `fill:${tok('warning')}` }, svg).textContent = m.label;
    });
    series.forEach((s, k) => {
      const d = s.values.map((v, i) => (i ? 'L' : 'M') + X(i).toFixed(1) + ' ' + Y(v).toFixed(1)).join('');
      el('path', { d, fill: 'none', style: `stroke:${s.color || color(k)}`, 'stroke-width': tok('line-w'), 'stroke-linejoin': 'round', 'stroke-linecap': 'round' }, svg);
    });
    // Direct labels at the line ends, nudged apart so close series don't overprint
    if (series.length <= 4) {
      const ends = series.map((s, k) => ({ k, y: Y(s.values[n - 1]) + 4 })).sort((a, b) => a.y - b.y);
      ends.forEach((e, i) => { if (i && e.y - ends[i - 1].y < 13) e.y = ends[i - 1].y + 13; });
      ends.forEach(e => { el('text', { x: X(n - 1) + 6, y: e.y, style: `fill:${tok('ink')}` }, svg).textContent = series[e.k].name; });
    }
    const cross = el('line', { y1: y0, y2: y0 + h, stroke: tok('muted'), visibility: 'hidden' }, svg);
    const hit = el('rect', { x: x0, y: y0, width: w, height: h, fill: 'transparent' }, svg);
    hit.addEventListener('mousemove', ev => {
      const r = svg.getBoundingClientRect(), sx = (ev.clientX - r.left) / r.width * width;
      const i = Math.max(0, Math.min(n - 1, Math.round((sx - x0) / w * (n - 1))));
      cross.setAttribute('x1', X(i)); cross.setAttribute('x2', X(i)); cross.setAttribute('visibility', 'visible');
      showTip(ev, x(i) + '<br>' + series.map((s, k) =>
        `<span style="color:${s.color || color(k)}">■</span> ${s.name} ${s.values[i].toFixed(2)} ${unit}`).join('<br>'));
    });
    hit.addEventListener('mouseleave', () => { cross.setAttribute('visibility', 'hidden'); hideTip(); });
  };

  // rows: [[label, value]]; one series, so one color. highlight names the bar to call out.
  window.axBars = function (svg, { rows, max, ticks, highlight, unit = '', width = 380, xLabel, yLabel }) {
    svg.innerHTML = '';
    const rowH = 40, x0 = 78, y0 = 4, w = width - x0 - 40, height = y0 + rows.length * rowH + 44;
    axisTitles(svg, { xLabel, yLabel, x0, y0, w, h: rows.length * rowH });
    svg.setAttribute('viewBox', `0 0 ${width} ${height}`);
    (ticks || [0, max / 2, max]).forEach(t => {
      const x = x0 + t / max * w;
      el('line', { x1: x, x2: x, y1: y0, y2: y0 + rowH * rows.length, stroke: tok('grid'), 'stroke-dasharray': tok('grid-dash') }, svg);
      el('text', { x, y: y0 + rowH * rows.length + 16, 'text-anchor': 'middle' }, svg).textContent = t;
    });
    rows.forEach(([name, v], i) => {
      const y = y0 + i * rowH + 9, bw = v / max * w;
      el('text', { x: x0 - 8, y: y + 15, 'text-anchor': 'end', style: `fill:${tok('ink')}` }, svg).textContent = name;
      el('rect', { x: x0, y, width: bw, height: 22, fill: name === highlight ? color(1) : color(0) }, svg);
      el('text', { x: x0 + bw + 6, y: y + 15 }, svg).textContent = v;
      const hit = el('rect', { x: 0, y: y - 8, width, height: rowH, fill: 'transparent' }, svg);
      hit.addEventListener('mousemove', ev => showTip(ev, `${name}<br>${v} ${unit}`));
      hit.addEventListener('mouseleave', hideTip);
    });
  };
})();
