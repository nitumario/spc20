#!/usr/bin/env python3
"""
plot_log.py - turn an SPC_20 UART telemetry capture into a chart.

The firmware emits one space-separated key:value line per second from
log_measurements() in main.c, interleaved with event lines (banner,
"EM: IDLE -> CHG_ONLY @ ...", "SLEEP: enter ..."). Raw serial captures
also contain NUL bytes and partial lines from resets, so the parser is
deliberately forgiving: it keeps any line that carries an "ms:" token
and whatever other known keys survived, and drops the rest.

Usage
    tools/plot_log.py serial_20260729_143632.log
    tools/plot_log.py bench.log -o bench.png --from 300 --to 900
    tools/plot_log.py bench.log --csv bench.csv        # for a spreadsheet
    tools/plot_log.py bench.log --list-sessions        # reboots in capture
    tools/plot_log.py bench.log --session 2 --show

A capture that reboots is split into sessions (ms restarts at 0); by
default the longest one is plotted.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

# ---------------------------------------------------------------- schema

# Numeric telemetry fields, keyed by the token name in the log line.
NUMERIC = [
    "ms", "Vbat", "Vchg", "Vout", "Vpanel", "Vusb1", "Vusb2",
    "Ipanel", "Ichg", "Idsg", "Ppanel", "Ibat_net", "Tbat", "Tboard",
    "bat_low", "has_sun", "has_load", "temp_ok", "p_limited", "bat_full",
    "i_buck_max", "allowed_chg", "pwm", "sp",
]
STATE = ["EM", "CHG", "MPPT"]          # categorical: IDLE/CC/TRK/...
HEXFIELD = ["fault", "flt_hist"]       # 4-digit hex bitmasks

TOKEN = re.compile(r"([A-Za-z_][A-Za-z0-9_+]*):(-?[0-9A-Za-z_+]+)")

# fault_ctx_t bit names, in bit order (fault_mgr.h)
FAULT_BITS = [
    "OVERTEMP", "BAT_OVERVOLT", "OVERCURRENT_CHG", "OVERCURRENT_DSG",
    "BAT_UNDERVOLT", "USB_OVERVOLT", "PRECHARGE_TIMEOUT", "TEMP_CHARGE_BLOCK",
    "REVERSE_PUMP",
]

# ---------------------------------------------------------------- palette
# Categorical slots 1-4 of the validated default palette, assigned to
# entities (never to rank) so a field keeps its colour across every run.
C_BLUE, C_ORANGE, C_AQUA, C_YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
C_RED, C_VIOLET = "#e34948", "#4a3aa7"
INK, INK_2, INK_3 = "#0b0b0b", "#52514e", "#8a8880"
SURFACE, GRID = "#fcfcfb", "#e6e5e0"

MIN_WIDTH_IN = 14.0        # never narrower than this

COLOR = {
    "Vpanel": C_BLUE, "sp": C_YELLOW,
    "Vbat": C_BLUE, "Vchg": C_ORANGE, "Vout": C_AQUA,
    "Ichg": C_BLUE, "Idsg": C_ORANGE, "Ipanel": C_AQUA, "allowed_chg": C_YELLOW,
    "Ppanel": C_BLUE, "pwm": C_VIOLET,
    "Tbat": C_BLUE, "Tboard": C_ORANGE,
}


def parse(path: Path) -> pd.DataFrame:
    raw = path.read_bytes().replace(b"\x00", b"")
    rows = []
    for line in raw.decode("utf-8", errors="replace").splitlines():
        if "ms:" not in line:
            continue                       # event line / banner / garbage
        rec = dict(TOKEN.findall(line))
        if "ms" not in rec or "Vbat" not in rec:
            continue                       # truncated line
        rows.append(rec)
    if not rows:
        sys.exit(f"{path}: no telemetry lines found")

    df = pd.DataFrame(rows)
    for col in NUMERIC:
        if col in df:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    for col in HEXFIELD:
        if col in df:
            df[col] = df[col].apply(
                lambda v: int(v, 16) if re.fullmatch(r"[0-9A-Fa-f]{1,4}", str(v)) else np.nan)
    for col in STATE:
        if col not in df:
            df[col] = pd.NA

    df = df.dropna(subset=["ms", "Vbat"]).reset_index(drop=True)
    # A reboot restarts time_now() at 0 -> new session.
    df["session"] = (df["ms"].diff() < 0).cumsum()
    return df


def pick_session(df: pd.DataFrame, want: int | None) -> pd.DataFrame:
    if want is not None:
        sel = df[df["session"] == want]
        if sel.empty:
            sys.exit(f"no session {want} (have 0..{int(df['session'].max())})")
    else:
        sel = df[df["session"] == df["session"].value_counts().idxmax()]
    sel = sel.copy()
    sel["t"] = (sel["ms"] - sel["ms"].iloc[0]) / 1000.0
    return sel


def fault_names(code: int) -> str:
    if not code:
        return ""
    return "+".join(n for i, n in enumerate(FAULT_BITS) if code & (1 << i))


# ---------------------------------------------------------------- drawing

def segments(series: pd.Series, t: np.ndarray):
    """Yield (x_start, width, label) runs of a step-valued series."""
    vals = series.to_numpy()
    start = 0
    for i in range(1, len(vals) + 1):
        if i == len(vals) or vals[i] != vals[start]:
            end = t[i] if i < len(t) else t[-1]
            yield t[start], max(end - t[start], 0.0), vals[start]
            start = i


def place_labels(ax, fig, items):
    """Right-edge direct labels, pushed apart so they never overlap."""
    lo, hi = ax.get_ylim()
    span = hi - lo or 1.0
    px = fig.get_size_inches()[1] * fig.dpi * ax.get_position().height
    minsep = 11.0 / max(px, 1.0)          # 11 px, as an axes fraction

    rows = sorted(((y - lo) / span, name, color) for y, name, color in items)
    for i in range(1, len(rows)):
        if rows[i][0] - rows[i - 1][0] < minsep:
            f, n, c = rows[i]
            rows[i] = (rows[i - 1][0] + minsep, n, c)
    over = rows[-1][0] - 1.0 if rows and rows[-1][0] > 1.0 else 0.0
    for frac, name, color in rows:
        ax.annotate(name, xy=(1.0, frac - over), xycoords="axes fraction",
                    xytext=(5, 0), textcoords="offset points",
                    va="center", ha="left", fontsize=7.5, color=color,
                    annotation_clip=False)


def line_panel(ax, fig, d, fields, ylabel, scale=1.0, invert=False):
    present = [f for f in fields if f in d and d[f].notna().any()]
    items = []
    for f in present:
        y = d[f] * scale
        ax.plot(d["t"], y, lw=1.4, color=COLOR[f], label=f,
                solid_joinstyle="round")
        tail = y.dropna()
        if not tail.empty:
            items.append((tail.iloc[-1], f, COLOR[f]))
    ax.set_ylabel(ylabel, fontsize=8, color=INK_2)
    if invert:
        ax.invert_yaxis()
    if len(present) >= 2:
        ax.legend(loc="lower left", bbox_to_anchor=(0, 1.0), fontsize=7,
                  frameon=False, ncol=len(present), handlelength=1.4,
                  columnspacing=1.2, labelcolor=INK_2, borderpad=0)
    style_axis(ax)
    place_labels(ax, fig, items)


def style_axis(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_3, labelsize=7.5, length=3)


def ribbon(ax, fig, d, t):
    """State/flag timeline: labelled segments, one row per signal."""
    px_per_s = (fig.get_size_inches()[0] * fig.dpi * ax.get_position().width
                / max(t[-1] - t[0], 1e-9))
    rows = [("EM", d["EM"]), ("CHG", d["CHG"]), ("MPPT", d["MPPT"])]
    for flag in ("has_sun", "has_load", "p_limited", "bat_full", "bat_low"):
        if flag in d and d[flag].notna().any():
            rows.append((flag, d[flag].map({1: "on", 0: ""}).fillna("")))
    if "fault" in d and d["fault"].notna().any():
        rows.append(("fault", d["fault"].fillna(0).astype(int).map(fault_names)))

    height, gap = 0.62, 0.38
    tick_labels = []
    for r, (name, series) in enumerate(rows):
        y = len(rows) - 1 - r
        labelled, seen = False, []
        for x, w, val in segments(series, t):
            if val in ("", None) or pd.isna(val):
                continue
            if val not in seen:
                seen.append(val)
            is_fault = name == "fault"
            face = "#f7d7d6" if is_fault else "#eceae4"
            edge = C_RED if is_fault else "#d9d7cf"
            ax.broken_barh([(x, w)], (y - height / 2, height),
                           facecolors=face, edgecolors=edge, linewidth=0.8)
            if w * px_per_s > 7.0 * len(str(val)):   # room for the text
                ax.text(x + w / 2, y, str(val), ha="center", va="center",
                        fontsize=6.5, color=C_RED if is_fault else INK_2)
                labelled = True
        if not labelled and seen:
            # Segments too narrow to write in: name the values at the right
            # edge instead, the same place the line panels put their labels.
            note = "/".join(str(v) for v in seen[:4])
            ax.annotate(note[:18], xy=(1.0, y), xycoords=("axes fraction", "data"),
                        xytext=(5, 0), textcoords="offset points",
                        va="center", ha="left", fontsize=7,
                        color=C_RED if name == "fault" else INK_2,
                        annotation_clip=False)
        tick_labels.append(name)

    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(tick_labels[::-1], fontsize=7.5, color=INK_2)
    ax.set_ylim(-0.5 - gap / 2, len(rows) - 0.5 + gap / 2)
    ax.set_facecolor(SURFACE)
    ax.grid(False)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=INK_3, labelsize=7.5, length=0)


def plot(d, out: Path, title: str, show: bool, *,
         px_per_sample: float, width_in: float | None,
         max_width_px: int, dpi: int, height_in: float):
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    t = d["t"].to_numpy()

    # Width follows the data: a strip chart needs a few px per sample or the
    # traces merge into a solid block. Margins stay fixed in INCHES so a very
    # wide figure doesn't grow a 10-inch left gutter.
    L, R, TOP, BOT = 1.05, 1.30, 0.85, 0.62      # inches
    if width_in is None:
        plot_px = len(d) * px_per_sample
        width_in = (plot_px / dpi) + L + R
        width_in = min(max(width_in, MIN_WIDTH_IN), max_width_px / dpi)

    fig, axes = plt.subplots(
        7, 1, figsize=(width_in, height_in), sharex=True, dpi=dpi,
        gridspec_kw={"height_ratios": [2, 2, 2.4, 1.5, 1.5, 1.2, 2.6], "hspace": 0.30})
    fig.patch.set_facecolor(SURFACE)
    fig.subplots_adjust(left=L / width_in, right=1 - R / width_in,
                        top=1 - TOP / height_in, bottom=BOT / height_in)

    line_panel(axes[0], fig, d, ["Vpanel", "sp"], "panel  V", scale=1e-3)
    line_panel(axes[1], fig, d, ["Vbat", "Vchg", "Vout"], "battery bus  V", scale=1e-3)
    line_panel(axes[2], fig, d, ["Ichg", "Idsg", "Ipanel", "allowed_chg"], "current  mA")
    line_panel(axes[3], fig, d, ["Ppanel"], "panel  mW")
    line_panel(axes[4], fig, d, ["pwm"], "pwm  (inverted:\nup = more current)", invert=True)
    line_panel(axes[5], fig, d, ["Tbat", "Tboard"], "temp  °C")
    axes[2].axhline(0, color=INK_3, lw=0.8, zorder=0)

    ribbon(axes[6], fig, d, t)

    # Vertical guides at every energy-mode transition, on every panel.
    changes = t[np.flatnonzero(d["EM"].ne(d["EM"].shift()).to_numpy()[1:]) + 1]
    for ax in axes:
        for x in changes:
            ax.axvline(x, color=INK_3, lw=0.6, ls=(0, (3, 3)), alpha=0.55, zorder=0)

    axes[-1].set_xlabel("time since session start  (s)", fontsize=8, color=INK_2)
    axes[-1].set_xlim(t[0], t[-1])
    # One tick per ~1.4 in of plot, so a wide figure stays readable across.
    plot_w_in = width_in - L - R
    axes[-1].xaxis.set_major_locator(
        MaxNLocator(nbins=max(int(plot_w_in / 1.4), 6), steps=[1, 2, 2.5, 5, 10]))

    x_left = L / width_in
    fig.suptitle(title, x=x_left, ha="left", fontsize=12, color=INK,
                 y=1 - 0.26 / height_in)
    fig.text(x_left, 1 - 0.55 / height_in,
             f"{len(d)} samples · {t[-1] - t[0]:.0f} s · "
             f"Vbat {d['Vbat'].min() / 1000:.2f}–{d['Vbat'].max() / 1000:.2f} V · "
             f"faults seen 0x{int(d['fault'].fillna(0).max()):04X}",
             ha="left", fontsize=8, color=INK_2)
    fig.savefig(out, dpi=dpi, facecolor=SURFACE)
    print(f"wrote {out}  ({int(width_in * dpi)}x{int(height_in * dpi)} px, "
          f"{len(d)} samples)")
    if show:
        plt.show()


def write_html(log: Path, out: Path) -> None:
    """Emit the interactive viewer with this capture embedded.

    The page also accepts a dropped file, so one copy works for every future
    capture; embedding just means it opens with data already on screen.
    """
    tpl = Path(__file__).with_name("log_viewer.html")
    if not tpl.exists():
        sys.exit(f"missing template: {tpl}")
    raw = log.read_bytes().replace(b"\x00", b"").decode("utf-8", errors="replace")
    # Keep only telemetry lines — a raw capture is mostly noise for this purpose.
    keep = "\n".join(l for l in raw.splitlines() if "ms:" in l and "Vbat:" in l)
    blob = json.dumps({"name": log.name, "text": keep}).replace("</", "<\\/")
    html = tpl.read_text().replace(
        '<script id="log-data" type="application/json">null</script>',
        '<script id="log-data" type="application/json">' + blob + "</script>")
    out.write_text(html)
    print(f"wrote {out}  ({len(html) / 1e6:.1f} MB, {keep.count(chr(10)) + 1} samples embedded)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("-o", "--out", type=Path, help="output PNG (default: <log>.png)")
    ap.add_argument("--csv", type=Path, help="also write the parsed rows as CSV")
    ap.add_argument("--html", nargs="?", const=True, default=None,
                    help="write the interactive viewer instead of a PNG "
                         "(optionally to a given path)")
    ap.add_argument("--session", type=int, help="which reboot session to plot")
    ap.add_argument("--list-sessions", action="store_true")
    ap.add_argument("--from", dest="t0", type=float, help="start time, s into session")
    ap.add_argument("--to", dest="t1", type=float, help="end time, s into session")
    ap.add_argument("--title")
    ap.add_argument("--px-per-sample", type=float, default=2.0,
                    help="horizontal pixels per sample (default 2.0); "
                         "raise it to spread a busy capture out further")
    ap.add_argument("--width", type=float,
                    help="fixed figure width in inches (overrides --px-per-sample)")
    ap.add_argument("--max-width-px", type=int, default=32000,
                    help="cap on output width in pixels (default 32000)")
    ap.add_argument("--height", type=float, default=14.0, help="figure height, inches")
    ap.add_argument("--dpi", type=int, default=140)
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    args = ap.parse_args()

    if not args.show:
        matplotlib.use("Agg")

    if args.html is not None:
        out = Path(args.html) if args.html is not True else args.log.with_suffix(".html")
        write_html(args.log, out)
        return

    df = parse(args.log)

    if args.list_sessions:
        for s, g in df.groupby("session"):
            print(f"session {int(s)}: {len(g):6d} samples, "
                  f"{(g['ms'].iloc[-1] - g['ms'].iloc[0]) / 1000:8.1f} s, "
                  f"Vbat {g['Vbat'].min()}..{g['Vbat'].max()} mV")
        return

    d = pick_session(df, args.session)
    if args.t0 is not None:
        d = d[d["t"] >= args.t0]
    if args.t1 is not None:
        d = d[d["t"] <= args.t1]
    if d.empty:
        sys.exit("no samples in that time window")

    if args.csv:
        d.to_csv(args.csv, index=False)
        print(f"wrote {args.csv}")

    out = args.out or args.log.with_suffix(".png")
    if not args.width and len(d) * args.px_per_sample / args.dpi + 2.2 > \
            args.max_width_px / args.dpi:
        print(f"note: width capped at {args.max_width_px} px for {len(d)} samples — "
              f"narrow the window with --from/--to, or raise --max-width-px",
              file=sys.stderr)

    plot(d, out, args.title or args.log.name, args.show,
         px_per_sample=args.px_per_sample, width_in=args.width,
         max_width_px=args.max_width_px, dpi=args.dpi, height_in=args.height)


if __name__ == "__main__":
    main()
