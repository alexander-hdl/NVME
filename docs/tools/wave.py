#!/usr/bin/env python3
"""wave.py - цифровые временные диаграммы (осциллограммы) -> inline SVG.

Формат входа близок к WaveDrom. Только стандартная библиотека, Python 3.12.

    python wave.py input.json > out.svg
    python wave.py --demo > demo.svg

    from wave import render, render_file
    svg = render({"signal": [{"name": "clk", "wave": "p...."}]})

Символы wave: p n P N (тактовый), 0 1 x z, = 2..9 (шина), . (продолжить),
| (разрыв). Группы: вложенный список ["Имя", {сигнал}, ...]. Пустой {} - отступ.
Узлы: "node": "..a..b" в сигнале и "edge": ["a->b подпись"] на верхнем уровне.
"""
from __future__ import annotations

import hashlib
import json
import re
import sys
from dataclasses import dataclass
from xml.sax.saxutils import escape

PAD, GROUP_W, NAME_FS, BUS_FS = 8, 22, 13, 11
FG = "var(--wave-fg, #222)"
GRID = "var(--wave-grid, rgba(128,128,128,.3))"
EDGE = "var(--wave-edge, #2a6fb0)"
HALO = "var(--wave-bg, transparent)"
BUS_TEXT = "#222"  # пастельные заливки светлые в обеих темах -> текст тёмный
BUS_FILL = {"=": "#e4e4e4", "2": "#fff3a8", "3": "#ffd7a8", "4": "#a8d4ff", "5": "#a8f0ee",
            "6": "#bff0a8", "7": "#e3b8f5", "8": "#f5b8b8", "9": "#d0d0f5"}
EDGE_RE = re.compile(r"^\s*(\w)\s*(<?[-~|]+>?|<>)\s*(\w)\s*(.*)$")


def f(v: float) -> str:
    return f"{v:.2f}".rstrip("0").rstrip(".")


def pts(points) -> str:
    return " ".join(f"{f(x)},{f(y)}" for x, y in points)


def text(x, y, s, size=12, anchor="start", fill=FG, extra="") -> str:
    return (f'<text x="{f(x)}" y="{f(y)}" font-size="{size}" text-anchor="{anchor}" '
            f'fill="{fill}" stroke="none"{extra}>{escape(str(s))}</text>')


# ---------------------------------------------------------------- разбор wave

@dataclass
class Seg:
    kind: str   # level | bus | clk
    ch: str
    start: int
    length: int = 1


def parse_wave(wave: str) -> tuple[list[Seg], list[int]]:
    """Разбивает строку wave на сегменты; возвращает (сегменты, позиции разрывов)."""
    segs: list[Seg] = []
    gaps: list[int] = []
    for i, ch in enumerate(wave):
        if ch == "|":
            gaps.append(i)
            ch = "."
        last = segs[-1] if segs else None
        if ch == ".":
            if last is None:
                continue
            if last.kind == "clk":
                segs.append(Seg("clk", last.ch.lower(), i))
            else:
                last.length += 1
        elif ch in "pnPN":
            segs.append(Seg("clk", ch, i))
        elif ch in "01zx":
            if last and last.ch == ch and last.start + last.length == i:
                last.length += 1
            else:
                segs.append(Seg("bus" if ch == "x" else "level", ch, i))
        elif ch in BUS_FILL:
            segs.append(Seg("bus", ch, i))
        else:
            segs.append(Seg("bus", "x", i))
    return segs, gaps


def fixed_y(seg: Seg | None, Y: dict) -> float | None:
    """Уровень, на котором сегмент начинается и заканчивается (None для шин)."""
    if seg is None or seg.kind == "bus":
        return None
    if seg.kind == "clk":
        return Y["bot"] if seg.ch in "pP" else Y["top"]
    return {"0": Y["bot"], "1": Y["top"], "z": Y["mid"]}[seg.ch]


def draw_wave(wave: str, data: list, X0: float, w: float, s: float, Y: dict, hatch: str):
    """Возвращает (SVG-элементы, список x-координат разрывов)."""
    segs, gaps = parse_wave(wave)
    lines: list[str] = []   # пути без заливки
    shapes: list[str] = []  # шины, стрелки, подписи
    labels = iter(data)
    for k, sg in enumerate(segs):
        prev = segs[k - 1] if k else None
        nxt = segs[k + 1] if k + 1 < len(segs) else None
        if prev and prev.start + prev.length != sg.start:
            prev = None
        if nxt and nxt.start != sg.start + sg.length:
            nxt = None
        x0 = X0 + sg.start * w
        x1 = x0 + sg.length * w
        py = fixed_y(prev, Y)
        if sg.kind == "bus":
            tl = py if py is not None else Y["mid"]
            nx = fixed_y(nxt, Y)
            tr = nx if nx is not None else Y["mid"]
            left = [(x0, Y["bot"]), (x0, Y["top"])] if prev is None else \
                [(x0 + s, Y["bot"]), (x0, tl), (x0 + s, Y["top"])]
            right = [(x1, Y["top"]), (x1, Y["bot"])] if nxt is None else \
                [(x1 - s, Y["top"]), (x1, tr), (x1 - s, Y["bot"])]
            fill = hatch if sg.ch == "x" else BUS_FILL[sg.ch]
            shapes.append(f'<polygon points="{pts(left + right)}" fill="{fill}"/>')
            if sg.ch != "x":
                label = next(labels, None)  # слот данных расходуется даже для None / ""
                if label not in (None, ""):
                    label = str(label)
                    size = min(BUS_FS, (x1 - x0 - 6) / (0.6 * len(label)))
                    if size >= 9:  # иначе подпись не помещается: оставляем только заливку
                        shapes.append(text((x0 + x1) / 2, Y["mid"] + size * 0.35, label,
                                           f(size), "middle", BUS_TEXT))
        elif sg.kind == "level":
            y = fixed_y(sg, Y)
            if py is not None and py != y:
                lines.append(f"M{f(x0)},{f(py)} L{f(x0 + s)},{f(y)} L{f(x1)},{f(y)}")
            else:
                lines.append(f"M{f(x0)},{f(y)} L{f(x1)},{f(y)}")
        else:  # clk, ровно один такт
            a = fixed_y(sg, Y)
            b = Y["top"] if a == Y["bot"] else Y["bot"]
            xm = x0 + w / 2
            d = f"M{f(x0)},{f(a)} L{f(x0 + s)},{f(b)} L{f(xm)},{f(b)} L{f(xm + s)},{f(a)} L{f(x1)},{f(a)}"
            if py is not None and py != a:
                d = f"M{f(x0)},{f(py)} L{f(x0)},{f(a)} " + d[d.index(" ") + 1:]
            lines.append(d)
            if sg.ch in "PN":
                cx, m = x0 + s / 2, Y["mid"]
                tri = [(cx - 3, m + 2.5), (cx + 3, m + 2.5), (cx, m - 3)] if sg.ch == "P" else \
                    [(cx - 3, m - 2.5), (cx + 3, m - 2.5), (cx, m + 3)]
                shapes.append(f'<polygon points="{pts(tri)}" fill="{FG}" stroke="none"/>')
    out = []
    if lines:
        out.append(f'<path d="{" ".join(lines)}" fill="none"/>')
    out += shapes
    return out, [X0 + (g + 0.5) * w for g in gaps]


# ----------------------------------------------------------------- раскладка

def flatten(items: list, depth: int, rows: list, groups: list) -> None:
    """Разворачивает вложенные группы в плоский список строк."""
    for it in items:
        if isinstance(it, list):
            start = len(rows)
            has_label = bool(it) and isinstance(it[0], str)
            flatten(it[1:] if has_label else it, depth + 1, rows, groups)
            groups.append((it[0] if has_label else "", start, len(rows), depth))
        else:
            rows.append(it or {})


def edge_svg(spec_edges: list, nodes: dict, uid: str, bounds: list[float]) -> list[str]:
    out = []
    for raw in spec_edges:
        m = EDGE_RE.match(str(raw))
        if not m or m[1] not in nodes or m[3] not in nodes:
            continue
        (ax, ay), (bx, by) = nodes[m[1]], nodes[m[3]]
        arrow, label = m[2], m[4]
        core = arrow.strip("<>")
        if core == "-|":
            d = f"M{f(ax)},{f(ay)} L{f(bx)},{f(ay)} L{f(bx)},{f(by)}"
        elif core == "|-":
            d = f"M{f(ax)},{f(ay)} L{f(ax)},{f(by)} L{f(bx)},{f(by)}"
        elif core == "-|-":
            mx = (ax + bx) / 2
            d = f"M{f(ax)},{f(ay)} L{f(mx)},{f(ay)} L{f(mx)},{f(by)} L{f(bx)},{f(by)}"
        elif "~" in core:
            mx = (ax + bx) / 2
            d = f"M{f(ax)},{f(ay)} C{f(mx)},{f(ay)} {f(mx)},{f(by)} {f(bx)},{f(by)}"
        else:
            d = f"M{f(ax)},{f(ay)} L{f(bx)},{f(by)}"
        mk = (f' marker-end="url(#{uid}a)"' if arrow.endswith(">") else "") + \
             (f' marker-start="url(#{uid}a)"' if arrow.startswith("<") else "")
        out.append(f'<path d="{d}"{mk}/>')
        if label:
            lx, my = (ax + bx) / 2, (ay + by) / 2
            gaps = [b for b in bounds if min(ay, by) < b < max(ay, by)]
            if abs(ax - bx) < 1 or not gaps:   # вертикальное / внутри строки: над началом стрелки
                lx, ly = ax if abs(ax - bx) < 1 else lx, min(ay, by) - 12
            else:                              # в зазоре между строками
                ly = min(gaps, key=lambda b: abs(b - my))
            tw = len(label) * 0.6 * 11 + 8
            out.append(f'<rect x="{f(lx - tw / 2)}" y="{f(ly - 6)}" width="{f(tw)}" height="12" '
                       f'rx="3" fill="var(--wave-page-bg, #fff)" stroke="none"/>')
            out.append(text(lx, ly + 4, label, 11, "middle", EDGE, ' stroke="none"'))
    return out


def render(spec: dict, cell_width: float = 20, row_height: float = 30,
           font: str = "monospace") -> str:
    """Рисует диаграмму spec (формат WaveDrom) и возвращает строку SVG."""
    cfg = spec.get("config") or {}
    cell_width = float(cfg.get("cell_width", cell_width))
    uid = "wv" + hashlib.md5(
        json.dumps([spec, cell_width, row_height, font], sort_keys=True, default=str)
        .encode()).hexdigest()[:6]
    w = cell_width * float(cfg.get("hscale", 1))
    s = min(3.0, w * 0.15)
    rows: list[dict] = []
    groups: list[tuple] = []
    flatten(spec.get("signal", []), 0, rows, groups)
    n = max([len(r.get("wave", "")) for r in rows] + [1])
    ncols = max([g[3] for g in groups], default=-1) + 1
    name_w = max([len(r.get("name", "")) for r in rows] + [0]) * NAME_FS * 0.6 + 12
    X0 = PAD + ncols * GROUP_W + name_w
    W = n * w

    head, foot = spec.get("head") or {}, spec.get("foot") or {}
    tick = head.get("tick", head.get("tock"))
    y = PAD + (26 if head.get("text") else 0)
    tick_y = y + 10
    y += 14 if tick is not None else 0
    grid_top = y
    hs = [row_height if r else row_height * 0.5 for r in rows]
    ys = []
    for h in hs:
        ys.append(y)
        y += h
    grid_bot = y
    foot_y = y + 16
    total_h = y + (22 if foot.get("text") else 0) + PAD

    hatch = f"url(#{uid}h)"
    defs = [
        f'<pattern id="{uid}h" width="4" height="4" patternUnits="userSpaceOnUse" '
        f'patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="4" '
        f'stroke="var(--wave-x, #888)" stroke-width="1"/></pattern>',
        f'<marker id="{uid}a" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7" '
        f'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L8,4 L0,8 z" '
        f'fill="{EDGE}" stroke="none"/></marker>',
    ]
    body: list[str] = []
    # сетка тактов
    grid = " ".join(f"M{f(X0 + k * w)},{f(grid_top)} V{f(grid_bot)}" for k in range(n + 1))
    body.append(f'<path d="{grid}" stroke="{GRID}" stroke-width="1" stroke-dasharray="2,3" fill="none"/>')
    if tick is not None:
        t0 = int(tick)
        body += [text(X0 + (k + 0.5) * w, tick_y, t0 + k, 9, "middle") for k in range(n)]

    nodes: dict[str, tuple[float, float]] = {}
    for r, (sig, ry, rh) in enumerate(zip(rows, ys, hs)):
        if not sig:
            continue
        m = rh * 0.2
        Y = {"top": ry + m, "bot": ry + rh - m, "mid": ry + rh / 2}
        if sig.get("name"):
            body.append(text(X0 - 8, Y["mid"] + NAME_FS * 0.35, sig["name"], NAME_FS, "end"))
        data = sig.get("data", [])
        data = data.split() if isinstance(data, str) else list(data)
        els, gap_x = draw_wave(sig.get("wave", ""), data, X0, w, s, Y, hatch)
        for i, ch in enumerate(sig.get("node", "")):
            if ch not in ". ":
                nodes[ch] = (X0 + i * w + s / 2, Y["mid"])
        wave_g = f'<g stroke="{FG}" stroke-width="1.5" stroke-linejoin="round">{"".join(els)}</g>'
        if gap_x:
            cuts = "".join(
                f'<polygon points="{pts([(x - 3 - 3, ry + rh), (x + 3 - 3, ry + rh), (x + 3 + 3, ry), (x - 3 + 3, ry)])}" fill="#000"/>'
                for x in gap_x)
            defs.append(f'<mask id="{uid}m{r}" maskUnits="userSpaceOnUse" x="{f(X0)}" y="{f(ry)}" '
                        f'width="{f(W)}" height="{f(rh)}"><rect x="{f(X0)}" y="{f(ry)}" '
                        f'width="{f(W)}" height="{f(rh)}" fill="#fff"/>{cuts}</mask>')
            wave_g = f'<g mask="url(#{uid}m{r})">{wave_g}</g>'
            ticks = "".join(
                f'<path d="M{f(x + dx - 3)},{f(Y["bot"] + 3)} L{f(x + dx + 3)},{f(Y["top"] - 3)}"/>'
                for x in gap_x for dx in (-3, 3))
            wave_g += f'<g stroke="{FG}" stroke-width="1" fill="none">{ticks}</g>'
        body.append(wave_g)

    # скобки групп
    for label, a, b, depth in groups:
        x = PAD + depth * GROUP_W
        y1, y2 = ys[a] + 3, ys[b - 1] + hs[b - 1] - 3
        bx = x + GROUP_W - 6
        body.append(f'<path d="M{f(bx + 4)},{f(y1)} H{f(bx)} V{f(y2)} H{f(bx + 4)}" '
                    f'stroke="{FG}" stroke-width="1.2" fill="none"/>')
        if label:
            cx, cy = x + 8, (y1 + y2) / 2
            body.append(text(cx, cy + 4, label, 11, "middle",
                             extra=f' transform="rotate(-90 {f(cx)} {f(cy)})"'))

    edges = edge_svg(spec.get("edge", []), nodes, uid, ys + [grid_bot])
    if edges:
        body.append(f'<g stroke="{EDGE}" stroke-width="1.2" fill="none">{"".join(edges)}</g>')

    # заголовок / подвал
    min_w = X0 + W + PAD
    for part, ty, size in ((head, PAD + 16, 14), (foot, foot_y, 12)):
        if part.get("text"):
            half = len(part["text"]) * size * 0.3
            cx = max(X0 + W / 2, PAD + half)
            min_w = max(min_w, cx + half + PAD)
            body.append(text(cx, ty, part["text"], size, "middle", extra=' font-weight="600"'))

    total_w = min_w
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{f(total_w)}" height="{f(total_h)}" '
        f'viewBox="0 0 {f(total_w)} {f(total_h)}" font-family="{escape(font)}" role="img" '
        f'style="max-width:100%;height:auto">'
        f'<defs>{"".join(defs)}</defs>{"".join(body)}</svg>'
    )


def render_file(json_path: str, **kwargs) -> str:
    """Читает JSON-файл со спецификацией и возвращает SVG."""
    with open(json_path, encoding="utf-8-sig") as fh:
        return render(json.load(fh), **kwargs)


DEMO = {
    "signal": [
        {"name": "clk", "wave": "P......."},
        {"name": "nclk", "wave": "N......."},
        {"name": "valid", "wave": "01..0.1.", "node": ".a"},
        ["Шина", {"name": "bus", "wave": "x=.=.34x", "data": ["A", "B", "C", "D"]},
         {"name": "addr", "wave": "z.5.6.|z", "data": "ADDR0 ADDR1"}],
        {},
        {"name": "ready", "wave": "0.1.....", "node": "..b"},
    ],
    "head": {"text": "Демо wave.py", "tick": 0},
    "foot": {"text": "p/n/P/N, 0/1/x/z, шины, группы, разрыв, рёбра"},
    "edge": ["a~>b задержка"],
}


def main(argv: list[str]) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if len(argv) != 1:
        print("usage: wave.py input.json > out.svg | wave.py --demo > demo.svg", file=sys.stderr)
        return 2
    print(render(DEMO) if argv[0] == "--demo" else render_file(argv[0]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
