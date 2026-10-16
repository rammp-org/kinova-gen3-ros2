"""Minimal plot theme for analyze.py: dark panel, dashed y-grid, direct line labels.

Self-contained so the arm machine needs only matplotlib. Roboto is used when it
is installed and the default sans otherwise.
"""

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager

PALETTE = {
    "dark": {
        "bg": "#0e0f10",
        "panel": "#141516",
        "ink": "#e4e1d8",
        "muted": "#8a877e",
        "grid": "#26282b",
        "warning": "#e6c229",
        "accent": "#e2683c",
        "series": ["#2f62e8", "#e03a2f", "#0f9a66", "#a45ee0", "#1a9cc0", "#d9539a"],
    },
    "light": {
        "bg": "#ffffff",
        "panel": "#ffffff",
        "ink": "#0b0b0b",
        "muted": "#5f5f5a",
        "grid": "#e6e6e3",
        "warning": "#b58900",
        "accent": "#c4532a",
        "series": ["#1f4fd8", "#e3241b", "#0f8a5f", "#8a3fd1", "#0e7fa3", "#c2307a"],
    },
}

_installed = {f.name for f in font_manager.fontManager.ttflist}
SANS = "Roboto" if "Roboto" in _installed else "DejaVu Sans"
MONO = "Roboto Mono" if "Roboto Mono" in _installed else "DejaVu Sans Mono"
CONDENSED = "Roboto Condensed" if "Roboto Condensed" in _installed else SANS
_pal = PALETTE["dark"]


def use(mode="dark"):
    global _pal
    _pal = PALETTE[mode]
    matplotlib.rcParams.update(
        {
            "figure.facecolor": _pal["bg"],
            "axes.facecolor": _pal["panel"],
            "axes.edgecolor": _pal["grid"],
            "axes.labelcolor": _pal["ink"],
            "xtick.color": _pal["muted"],
            "ytick.color": _pal["muted"],
            "text.color": _pal["ink"],
            "font.family": SANS,
            "font.size": 10,
            "axes.grid": True,
            "axes.grid.axis": "y",
            "grid.color": _pal["grid"],
            "grid.linestyle": (0, (2, 4)),
            "grid.linewidth": 1.0,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "lines.linewidth": 2.0,
            "lines.solid_joinstyle": "round",
            "legend.frameon": False,
            "savefig.dpi": 150,
            "savefig.facecolor": _pal["bg"],
        }
    )
    return _pal


def title(ax, text, sub=None):
    ax.set_title(
        text.upper(),
        loc="left",
        fontfamily=CONDENSED,
        fontweight="bold",
        fontsize=11,
        color=_pal["ink"],
        pad=16 if sub else 6,
    )
    if sub:
        ax.text(
            0.0, 1.02, sub, transform=ax.transAxes, fontsize=8.5, color=_pal["muted"]
        )


def label_lines(ax, min_gap=0.06):
    """Name each line at its right end instead of a legend box, nudged apart."""
    lines = [
        ln
        for ln in ax.get_lines()
        if not ln.get_label().startswith("_") and len(ln.get_xdata())
    ]
    lo, hi = ax.get_ylim()
    gap = min_gap * (hi - lo)
    ends = sorted(((ln.get_ydata()[-1], ln) for ln in lines), key=lambda e: e[0])
    ys = [float(y) for y, _ in ends]
    for i in range(1, len(ys)):
        ys[i] = max(ys[i], ys[i - 1] + gap)
    for y, (_, ln) in zip(ys, ends):
        ax.annotate(
            ln.get_label(),
            (ln.get_xdata()[-1], y),
            xytext=(6, 0),
            textcoords="offset points",
            va="center",
            fontsize=9,
            fontfamily=MONO,
            color=ln.get_color(),
            annotation_clip=False,
        )
    for lab in ax.get_xticklabels() + ax.get_yticklabels():
        lab.set_fontfamily(MONO)


def tap_lines(ax, taps):
    for k, m in enumerate(taps):
        ax.axvline(m, color=_pal["warning"], linestyle=(0, (4, 4)), linewidth=1.0)
        if k == 0:
            ax.text(
                m,
                0.98,
                " tap",
                transform=ax.get_xaxis_transform(),
                va="top",
                fontsize=8,
                fontfamily=MONO,
                color=_pal["warning"],
            )


def page_title(fig, text):
    """Uppercase page title with the terracotta square drawn, not typed (Roboto has no glyph)."""
    from matplotlib.patches import Rectangle

    fig.add_artist(
        Rectangle(
            (0.08, 0.955),
            0.009,
            0.014,
            transform=fig.transFigure,
            color=_pal["accent"],
            clip_on=False,
        )
    )
    fig.text(
        0.095,
        0.955,
        text.upper(),
        fontfamily=CONDENSED,
        fontweight="bold",
        fontsize=13,
        color=_pal["ink"],
        va="baseline",
    )


def save(fig, path):
    for a in fig.get_axes():
        if not a.get_ylabel() or (
            a.get_subplotspec().is_last_row() and not a.get_xlabel()
        ):
            raise ValueError("label every axis before saving")
    fig.savefig(path)
    plt.close(fig)
