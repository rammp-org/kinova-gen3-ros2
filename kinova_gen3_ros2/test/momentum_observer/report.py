#!/usr/bin/env python3
"""Write a self-contained HTML report for a recorded run: tiles, live charts, tables, notes.

  python3 report.py --csv run1.csv --urdf arm.urdf --out run1.html [--notes notes.md]

Same numbers as analyze.py (it calls the same compute step); the charts have a
crosshair tooltip and the page needs nothing but a browser. Style files in web/
are vendored from alex-style-guide with one local extension (event marks).
"""

import argparse
import datetime as dt
import html
import json
import os
import sys

import numpy as np

from analyze import TAP_AFTER_S, TAP_BEFORE_S, compute, summarize

HERE = os.path.dirname(os.path.abspath(__file__))
MAX_POINTS = 2500


def nice_ticks(lim, n=7):
    """Symmetric ticks around zero that cover +/-lim with round numbers."""
    step = 10 ** np.floor(np.log10(lim / (n // 2)))
    for m in (1, 2, 2.5, 5, 10):
        if lim / (step * m) <= n // 2:
            step *= m
            break
    k = int(np.ceil(lim / step))
    return [round(i * step, 6) for i in range(-k, k + 1)]


def per_tap(r, joints):
    """For each marker: the peak of each signal, per joint, inside its window."""
    rows = []
    for m in r["taps"]:
        w = (r["t"] >= m - r["before"]) & (r["t"] <= m + r["after"])
        if not w.any():
            continue
        rows.append(
            {
                "t": float(m),
                "qs": [float(np.abs(r["qs"][w, j]).max()) for j in joints],
                "ob": [float(np.abs(r["ob"][w, j]).max()) for j in joints],
            }
        )
    return rows


def _slow(sig, r, w=29):
    """RMS of the smoothed signal while moving in the tap loops, taps excluded: the pose-dependent part."""
    t, qd = r["t"], r["qd"]
    k = np.ones(w) / w
    sm = np.column_stack(
        [np.convolve(sig[:, j], k, mode="same") for j in range(sig.shape[1])]
    )
    sel = (np.abs(qd).max(axis=1) > 0.05) & (t >= (r["phase"] or 0)) & ~r["tap"]
    if not sel.any():
        sel = (np.abs(qd).max(axis=1) > 0.05) & (t >= (r["phase"] or 0))
    return (
        float(np.sqrt((sm[sel][:, [1, 3, 5]] ** 2).mean()))
        if sel.any()
        else float("nan")
    )


def build(r, joints, notes):
    t, qd, qs, ob = r["t"], r["qd"], r["qs"], r["ob"]
    stride = max(1, int(np.ceil(len(t) / MAX_POINTS)))
    idx = np.arange(0, len(t), stride)
    td = t[idx]
    names = [f"J{j + 1}" for j in joints]
    series = {
        "speed": [[round(float(v), 4) for v in qd[idx, j]] for j in joints],
        "qs": [[round(float(v), 3) for v in qs[idx, j]] for j in joints],
        "ob": [[round(float(v), 3) for v in ob[idx, j]] for j in joints],
    }
    has_bs = r["ob_bs"] is not None
    if has_bs:
        series["obbs"] = [
            [round(float(v), 3) for v in r["ob_bs"][idx, j]] for j in joints
        ]
    marks = [{"i": int(np.argmin(np.abs(td - m))), "label": "tap"} for m in r["taps"]]
    if r["phase"] is not None:
        marks.append(
            {"i": int(np.argmin(np.abs(td - r["phase"]))), "label": "tap loops start"}
        )
    tlim = max(np.abs(qs[:, joints]).max(), np.abs(ob[:, joints]).max(), 0.5)
    data = {
        "names": names,
        "t": [round(float(v), 3) for v in td],
        "series": series,
        "marks": marks,
        "speedTicks": nice_ticks(float(np.abs(qd[:, joints]).max()) or 0.1),
        "torqueTicks": nice_ticks(float(tlim)),
    }

    qs_st = summarize("core", qs, r["moving"], r["tap"], joints)
    ob_st = summarize("observer", ob, r["moving"], r["tap"], joints)
    bs_st = None
    if has_bs:
        # the corrected signal exists only after the boundary, so its no-tap floor is
        # the tap loops' motion outside the tap windows (needs per-tap markers)
        mv = (np.abs(qd).max(axis=1) > 0.05) & (t >= r["phase"])
        bs_st = summarize("observer - baseline", r["ob_bs"], mv, r["tap"], joints)
    jmax = max(joints, key=lambda j: abs(r["qs_off"][j]))
    worst_qs = max(qs_st[j][1] for j in joints)
    worst_ob = max(ob_st[j][1] for j in joints)
    best_ratio_qs = max(qs_st[j][3] for j in joints)
    best_ratio_ob = max(ob_st[j][3] for j in joints)
    taps = per_tap(r, joints)
    stamp = dt.datetime.fromtimestamp(os.path.getmtime(r["csv"])).strftime(
        "%Y-%m-%d %H:%M"
    )
    sign_txt = "command convention" if r["sign"] > 0 else "reaction-signed (raw KORTEX)"

    def esc(s):
        return html.escape(str(s))

    def num(v, d=2):
        return "–" if np.isnan(v) else f"{v:.{d}f}"

    rows = []
    for j in joints:
        kinds = [
            ("core g(q) − τ", qs_st, r["qs_off"]),
            ("observer", ob_st, r["ob_off"]),
        ]
        if bs_st:
            kinds.append(("observer − baseline map", bs_st, np.zeros(7)))
        for label, st, off in kinds:
            rms, qpeak, peak, ratio = st[j]
            rows.append(
                f"<tr><td>J{j + 1}</td><td>{label}</td><td class=num>{off[j]:+.2f}</td>"
                f"<td class=num>{num(rms)}</td><td class=num>{num(qpeak)}</td>"
                f"<td class=num>{num(peak)}</td><td class=num>{num(ratio, 1)}</td></tr>"
            )
    tap_rows = []
    for k, row in enumerate(taps):
        cells = "".join(
            f"<td class=num>{a:.2f}</td><td class=num>{b:.2f}</td>"
            for a, b in zip(row["qs"], row["ob"])
        )
        tap_rows.append(
            f"<tr><td class=time>{row['t']:.3f}</td><td>{k + 1}</td>{cells}</tr>"
        )
    tap_head = "".join(
        f"<th class=num>{n} core</th><th class=num>{n} obs</th>" for n in names
    )
    tap_table = (
        f"<table><thead><tr><th>Time, s</th><th>Tap</th>{tap_head}</tr></thead>"
        f"<tbody>{''.join(tap_rows)}</tbody></table>"
        if taps
        else "<p class=ax-muted>No tap markers in this run. Press Enter in the recorder at each tap.</p>"
    )

    facts = [
        f"Torque sign detected as {sign_txt}; the joint states arrived at {r['rate']:.0f} Hz and the observer ran at {r['gain']:g} rad/s.",
        (
            f"With the arm still at the start pose, g(q) − τ on J{jmax + 1} is {r['qs_off'][jmax]:+.2f} N·m. "
            "That is the mass model's error there (kinova-gen3-driver#5); both signals carry it and it was subtracted before every number above."
        ),
        (
            f"Largest excursion with nobody touching the arm: core {worst_qs:.2f} N·m, observer {worst_ob:.2f} N·m. "
            "A detector's threshold has to sit above this."
        ),
    ]
    note_paras = [f"<p>{esc(p.strip())}</p>" for p in notes.split("\n\n") if p.strip()]

    bs_panel = ""
    if has_bs:
        bs_panel = """<section class="ax-panel chart">
    <h2>Observer − baseline map</h2>
    <div class="ax-cap">Tap loops only. The baseline loop's residual at the same pose and direction subtracted, so the pose-dependent model error is gone; what remains is sensor noise and pushes.</div>
    <svg id="obbs"></svg>
  </section>"""
        facts.append(
            "Baseline map: the baseline loop's residual at the same pose was subtracted from the tap loops. "
            f"Observer slow error (moving, no tap) went from {_slow(ob, r):.2f} to {_slow(r['ob_bs'], r):.2f} N·m RMS on the plotted joints."
        )
    web = {}
    for n in ("tokens.css", "ax-charts.js", "ax-fx.js"):
        with open(os.path.join(HERE, "web", n)) as f:
            # "</" inside an inline <script> or <style> ends the element early; escape it.
            web[n] = f.read().replace("</", "<\\/")
    run = os.path.basename(r["csv"])
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Momentum observer · {esc(run)}</title>
<link href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;500;700&family=Roboto+Condensed:wght@500;700&family=Roboto+Mono:wght@400;500;700&display=swap" rel="stylesheet">
<style>{web["tokens.css"]}</style>
<style>
  body {{ margin: 0; }}
  .ax-page {{ max-width: 1180px; }}
  .chart svg {{ width: 100%; height: auto; }}
  .report {{ display: grid; grid-template-columns: 1fr 1fr; gap: 28px; margin-top: 8px; padding-top: 20px; border-top: 1px solid var(--ax-border); }}
  .report p {{ max-width: 60ch; }}
</style>
<script>{web["ax-charts.js"]}</script>
<script>{web["ax-fx.js"]}</script>
</head>
<body class="ax">
<main class="ax-page">
  <header class="ax-mast">
    <div>
      <h1>Momentum observer</h1>
      <div class="ax-meta"><span>{esc(run)}</span><span>{stamp}</span><span>{esc(os.path.basename(r["urdf"]))}</span><span>{r["rate"]:.0f} Hz</span><span>gain {r["gain"]:g} rad/s</span><span>{len(r["taps"])} taps</span></div>
    </div>
    <div style="display:flex;gap:8px">
      <button class="ax-btn" id="mode">Light mode</button>
    </div>
  </header>

  <section class="ax-tiles">
    <div class="ax-tile"><div class="ax-label">Model gap at rest, J{jmax + 1}</div><div class="ax-value">{r["qs_off"][jmax]:+.2f}<small>N·m</small></div><div class="ax-sub">g(q) − τ with the arm still</div></div>
    <div class="ax-tile"><div class="ax-label">False-alarm floor, core</div><div class="ax-value">{worst_qs:.2f}<small>N·m</small></div><div class="ax-sub">largest excursion, no tap</div></div>
    <div class="ax-tile"><div class="ax-label">False-alarm floor, observer</div><div class="ax-value">{worst_ob:.2f}<small>N·m</small></div><div class="ax-sub">{"lower" if worst_ob < worst_qs else "not lower"} than core</div></div>
    <div class="ax-tile"><div class="ax-label">Tap over floor, best joint</div><div class="ax-value">{num(best_ratio_ob, 1)}<small>×</small></div><div class="ax-sub">core {num(best_ratio_qs, 1)}×</div></div>
  </section>

  <section class="ax-panel chart">
    <h2>Joint speed</h2>
    <div class="ax-cap">Context: when the arm moves under its own command. Dashed lines are tap markers.</div>
    <div class="ax-legend" id="lg"></div>
    <svg id="speed"></svg>
  </section>
  <section class="ax-panel chart">
    <h2>Core: g(q) − τ</h2>
    <div class="ax-cap">Quasi-static residual as published on /ee_state, rest offset removed. Carries the arm's own inertia.</div>
    <svg id="qs"></svg>
  </section>
  <section class="ax-panel chart">
    <h2>Momentum observer</h2>
    <div class="ax-cap">Same q, q̇, τ, rest offset removed. Inertia and Coriolis accounted for; what is left is model error and pushes.</div>
    <svg id="ob"></svg>
  </section>

  {bs_panel}
  <section class="ax-panel">
    <h2>Summary</h2>
    <div class="ax-cap">Motion RMS and motion peak: the signal while the arm moves with no tap in progress. Tap peak: the largest excursion inside a tap window ({r["before"]:g} s before to {r["after"]:g} s after a marker). Ratio: tap peak over motion peak.</div>
    <table>
      <thead><tr><th>Joint</th><th>Signal</th><th class=num>Rest offset, N·m</th><th class=num>Motion RMS, N·m</th><th class=num>Motion peak, N·m</th><th class=num>Tap peak, N·m</th><th class=num>Ratio</th></tr></thead>
      <tbody>{"".join(rows)}</tbody>
    </table>
  </section>

  <section class="ax-panel">
    <h2>Per tap</h2>
    <div class="ax-cap">Peak of each signal inside each marker's window, N·m.</div>
    {tap_table}
  </section>

  <section class="report">
    <div>
      <h2>What the run says</h2>
      {"".join(f"<p>{esc(f)}</p>" for f in facts)}
    </div>
    <div>
      <h2>Notes</h2>
      {"".join(note_paras) or '<p class="ax-muted">No notes. Pass --notes notes.md.</p>'}
    </div>
  </section>
</main>

<script>
const D = {json.dumps(data)};
const n = D.t.length;
const xTicks = (() => {{ const out = []; const last = D.t[n - 1]; const step = last > 40 ? 10 : last > 15 ? 5 : 1;
  for (let s = 0; s <= last; s += step) {{ let best = 0; for (let i = 0; i < n; i++) if (Math.abs(D.t[i] - s) < Math.abs(D.t[best] - s)) best = i; out.push(best); }} return out; }})();
const xl = i => D.t[i].toFixed(1) + ' s';
function mk(key) {{ return D.names.map((name, k) => ({{ name, values: D.series[key][k] }})); }}
function draw() {{
  const lg = document.getElementById('lg'); lg.innerHTML = '';
  D.names.forEach((name, k) => {{ const e = document.createElement('span'); e.style.setProperty('--c', `var(--ax-s${{k + 1}})`); e.textContent = name; lg.appendChild(e); }});
  const common = {{ x: xl, xTicks, width: 1100, height: 250, marks: D.marks, xLabel: 'Time, s' }};
  axLine(document.getElementById('speed'), {{ ...common, series: mk('speed'), yTicks: D.speedTicks, unit: 'rad/s', yLabel: 'Joint speed, rad/s' }});
  axLine(document.getElementById('qs'), {{ ...common, series: mk('qs'), yTicks: D.torqueTicks, unit: 'N·m', yLabel: 'External torque, N·m' }});
  axLine(document.getElementById('ob'), {{ ...common, series: mk('ob'), yTicks: D.torqueTicks, unit: 'N·m', yLabel: 'External torque, N·m' }});
  if (D.series.obbs) axLine(document.getElementById('obbs'), {{ ...common, series: mk('obbs'), yTicks: D.torqueTicks, unit: 'N·m', yLabel: 'External torque, N·m' }});
}}
document.getElementById('mode').onclick = e => {{
  const light = document.documentElement.dataset.mode !== 'light';
  document.documentElement.dataset.mode = light ? 'light' : '';
  e.target.textContent = light ? 'Dark mode' : 'Light mode';
  draw();
}};
if (location.hash === '#light') document.getElementById('mode').click(); else draw();
document.fonts.ready.then(draw);
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True, help="HTML path")
    ap.add_argument("--gain", type=float, default=20.0)
    ap.add_argument("--joints", default="2,4,6")
    ap.add_argument("--tau-sign", type=float, default=0.0, choices=[0.0, 1.0, -1.0])
    ap.add_argument("--tap-before", type=float, default=TAP_BEFORE_S)
    ap.add_argument("--tap-after", type=float, default=TAP_AFTER_S)
    ap.add_argument("--notes", help="markdown/plain text appended as the Notes section")
    args = ap.parse_args()
    joints = [int(s) - 1 for s in args.joints.split(",")]
    if len(joints) > 6 or any(j < 0 or j > 6 for j in joints):
        sys.exit("--joints: up to six values in 1..7")
    r = compute(
        args.csv, args.urdf, args.gain, args.tau_sign, args.tap_before, args.tap_after
    )
    notes = ""
    if args.notes:
        with open(args.notes) as f:
            notes = f.read()
    with open(args.out, "w") as f:
        f.write(build(r, joints, notes))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
