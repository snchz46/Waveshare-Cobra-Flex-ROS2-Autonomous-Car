"""Length-parametric road-tile factory (two-lane, 0.52 m).

Unlike ``road_variants/make_road_variants.py`` and ``road_variants_lot2``
(which emit fixed 3-10 m PNGs that get UV-compressed when applied to a
short Gazebo box), every tile here is generated at the **exact requested
length**, so the dashed-centreline pacing and every painted feature keep
their real-world size. Output files carry the real size in metres in the
name, e.g. ``tile_worn_0p52m_x_4m.png``.

The module is importable: ``make_tile(theme, length_m, out_dir)`` returns
the path of a freshly generated (or cached) PNG. ``scripts/compose_lane_circuit.py``
calls it so the visual road and the logical centreline never drift apart.

Run directly (``python3 make_road_tiles.py``) to regenerate the canonical
4 m preview set next to this file.

Conventions (shared with the rest of road_assets):
  - Pure black asphalt RGB(0,0,0), 1 cm white lines, 24.5 cm useful lane,
    52 cm two-lane total width, dashed centreline 10 cm dash / 10 cm gap.
  - 500 px/m. Road runs along the image **height** (tall images), entry at
    the bottom; dashes start at y=0 so consecutive tiles seam cleanly.
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter

PX_PER_M = 500
ASPHALT = (0, 0, 0)
LINE_WHITE = (255, 255, 255)

LINE_WIDTH_M = 0.01
DASH_MARK_M = 0.10
DASH_GAP_M = 0.10
LANE_USEFUL_M = 0.245
TWO_LANE_TOTAL_M = 2 * LANE_USEFUL_M + 3 * LINE_WIDTH_M  # 0.52

THEMES = (
    # degraded paint
    "clean",
    "worn",
    "wet",
    "faded",
    # missing lines
    "no_lines",
    "no_centreline",
    "no_edges",
    "edge_gap",
    # painted distractors
    "zebra",
    "double_solid",
    "arrows",
)


def m_to_px(m: float) -> int:
    return int(round(m * PX_PER_M))


def _fmt_m(value_m: float) -> str:
    """Metres as a filename token: 4.0 -> '4', 0.52 -> '0p52', 4.5 -> '4p5'."""
    s = f"{value_m:.2f}".rstrip("0").rstrip(".")
    return s.replace(".", "p")


def tile_filename(theme: str, length_m: float, width_m: float = TWO_LANE_TOTAL_M) -> str:
    return f"tile_{theme}_{_fmt_m(width_m)}m_x_{_fmt_m(length_m)}m.png"


# --- primitives (mirrors road_variants/make_road_variants.py) ----------------


def _canvas(width_m: float, length_m: float, base=ASPHALT) -> Image.Image:
    return Image.new("RGB", (m_to_px(width_m), m_to_px(length_m)), base)


def _asphalt_noise(img, strength=6, density_divisor=60, seed=42):
    random.seed(seed)
    px = img.load()
    w, h = img.size
    for _ in range(w * h // density_divisor):
        x = random.randint(0, w - 1)
        y = random.randint(0, h - 1)
        r, g, b = px[x, y]
        d = random.randint(-strength, strength)
        px[x, y] = (max(0, min(255, r + d)), max(0, min(255, g + d)), max(0, min(255, b + d)))
    return img


def _edges(draw, width_m, length_m, color=LINE_WHITE, sides=("left", "right")):
    line_w = m_to_px(LINE_WIDTH_M)
    w, h = m_to_px(width_m), m_to_px(length_m)
    if "left" in sides:
        draw.rectangle([(0, 0), (line_w - 1, h - 1)], fill=color)
    if "right" in sides:
        draw.rectangle([(w - line_w, 0), (w - 1, h - 1)], fill=color)


def _dashed_centre(draw, width_m, length_m, color=LINE_WHITE, y0_m=0.0, y1_m=None):
    line_w = m_to_px(LINE_WIDTH_M)
    w, h = m_to_px(width_m), m_to_px(length_m)
    if y1_m is None:
        y1_m = length_m
    cx = w // 2
    hl, hr = line_w // 2, line_w - line_w // 2
    dash, gap = m_to_px(DASH_MARK_M), m_to_px(DASH_GAP_M)
    y, y_end = m_to_px(y0_m), m_to_px(y1_m)
    while y < y_end:
        draw.rectangle([(cx - hl, y), (cx + hr - 1, min(y + dash, y_end) - 1)], fill=color)
        y += dash + gap


def _line_wear(img, wear_level=0.3, seed=1):
    random.seed(seed)
    w, h = img.size
    px = img.load()
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if r > 200 and g > 200 and b > 200 and random.random() < wear_level:
                fade = random.randint(30, 120)
                px[x, y] = (
                    max(0, r - fade - random.randint(0, 60)),
                    max(0, g - fade - random.randint(0, 60)),
                    max(0, b - fade - random.randint(0, 60)),
                )
    return img


def _patches(img, n=6, seed=2):
    random.seed(seed)
    draw = ImageDraw.Draw(img)
    w, h = img.size
    for _ in range(n):
        pw = random.randint(m_to_px(0.08), m_to_px(0.20))
        ph = random.randint(m_to_px(0.15), m_to_px(0.50))
        x = random.randint(m_to_px(0.02), max(m_to_px(0.02) + 1, w - pw - m_to_px(0.02)))
        y = random.randint(m_to_px(0.10), max(m_to_px(0.10) + 1, h - ph - m_to_px(0.10)))
        shade = random.randint(20, 45)
        draw.rectangle([(x, y), (x + pw, y + ph)], fill=(shade, shade, shade + random.randint(-3, 3)))
    return img


def _potholes_stains(img, n_potholes=4, n_stains=5, seed=3):
    random.seed(seed)
    draw = ImageDraw.Draw(img)
    w, h = img.size
    for _ in range(n_potholes):
        cx = random.randint(m_to_px(0.03), w - m_to_px(0.03))
        cy = random.randint(m_to_px(0.1), h - m_to_px(0.1))
        r = random.randint(m_to_px(0.015), m_to_px(0.04))
        draw.ellipse([(cx - r, cy - r), (cx + r, cy + r)], fill=(15, 15, 17))
    for _ in range(n_stains):
        cx = random.randint(m_to_px(0.03), w - m_to_px(0.03))
        cy = random.randint(m_to_px(0.1), h - m_to_px(0.1))
        r = random.randint(m_to_px(0.04), m_to_px(0.10))
        draw.ellipse([(cx - r, cy - r), (cx + r, cy + r)], fill=(18, 16, 14))
    return img


def _wet_sheen(img, seed=5):
    random.seed(seed)
    px = img.load()
    w, h = img.size
    for _ in range(random.randint(4, 7)):
        y0 = random.randint(0, h - 1)
        band_h = random.randint(m_to_px(0.3), m_to_px(1.2))
        strength = random.randint(6, 14)
        for y in range(y0, min(y0 + band_h, h)):
            for x in range(w):
                r, g, b = px[x, y]
                px[x, y] = (min(255, r + strength // 3), min(255, g + strength // 2), min(255, b + strength))
    return img


def _draw_arrow(draw, cx_px, cy_px, direction="straight", size_m=0.18):
    size_px = m_to_px(size_m)
    shaft_w, head_w, head_len = m_to_px(0.03), m_to_px(0.08), m_to_px(0.06)
    if direction == "straight":
        top, bot = cy_px - size_px // 2, cy_px + size_px // 2
        draw.rectangle([(cx_px - shaft_w // 2, top + head_len), (cx_px + shaft_w // 2, bot)], fill=LINE_WHITE)
        draw.polygon([(cx_px, top), (cx_px - head_w // 2, top + head_len), (cx_px + head_w // 2, top + head_len)], fill=LINE_WHITE)
    elif direction == "right":
        left, right = cx_px - size_px // 3, cx_px + size_px // 2
        bot = cy_px + size_px // 3
        draw.rectangle([(left, cy_px - shaft_w // 2), (right - head_len, cy_px + shaft_w // 2)], fill=LINE_WHITE)
        draw.rectangle([(left, cy_px - shaft_w // 2), (left + shaft_w, bot)], fill=LINE_WHITE)
        draw.polygon([(right, cy_px), (right - head_len, cy_px - head_w // 2), (right - head_len, cy_px + head_w // 2)], fill=LINE_WHITE)


# --- theme assembly ----------------------------------------------------------


def _render(theme: str, length_m: float, width_m: float) -> Image.Image:
    line_w = m_to_px(LINE_WIDTH_M)
    w_px = m_to_px(width_m)
    h_px = m_to_px(length_m)

    if theme == "clean":
        img = _asphalt_noise(_canvas(width_m, length_m), strength=5)
        d = ImageDraw.Draw(img)
        _edges(d, width_m, length_m)
        _dashed_centre(d, width_m, length_m)
        return img

    if theme == "worn":
        img = _asphalt_noise(_canvas(width_m, length_m), strength=10, density_divisor=30)
        img = _patches(img, n=max(3, round(length_m * 1.75)), seed=20)
        d = ImageDraw.Draw(img)
        _edges(d, width_m, length_m)
        _dashed_centre(d, width_m, length_m)
        img = _line_wear(img, wear_level=0.40, seed=21)
        img = _potholes_stains(img, n_potholes=max(2, round(length_m * 1.25)),
                               n_stains=max(2, round(length_m * 1.5)), seed=22)
        return img

    if theme == "wet":
        img = _asphalt_noise(_canvas(width_m, length_m), strength=7, density_divisor=35)
        d = ImageDraw.Draw(img)
        _edges(d, width_m, length_m)
        _dashed_centre(d, width_m, length_m)
        img = _line_wear(img, wear_level=0.25, seed=30)
        img = _wet_sheen(img, seed=31)
        img = _potholes_stains(img, n_potholes=2, n_stains=max(3, round(length_m * 1.75)), seed=32)
        return img.filter(ImageFilter.GaussianBlur(radius=1.8))

    if theme == "faded":
        img = _asphalt_noise(_canvas(width_m, length_m), strength=6)
        d = ImageDraw.Draw(img)
        _edges(d, width_m, length_m)
        _dashed_centre(d, width_m, length_m)
        return _line_wear(img, wear_level=0.6, seed=40)

    if theme == "no_lines":
        # Total paint dropout: black asphalt only, no edges, no centre.
        return _asphalt_noise(_canvas(width_m, length_m), strength=6)

    if theme == "no_centreline":
        img = _asphalt_noise(_canvas(width_m, length_m), strength=5)
        _edges(ImageDraw.Draw(img), width_m, length_m)
        return img

    if theme == "no_edges":
        img = _asphalt_noise(_canvas(width_m, length_m), strength=5)
        _dashed_centre(ImageDraw.Draw(img), width_m, length_m)
        return img

    if theme == "edge_gap":
        # Continuous left edge; right edge has a 1.2 m worn-out gap centred
        # on the tile. Dashed centre intact. (lot2 g3_01, parametric length.)
        img = _asphalt_noise(_canvas(width_m, length_m), strength=5)
        d = ImageDraw.Draw(img)
        gap = m_to_px(1.2)
        g0 = max(0, h_px // 2 - gap // 2)
        g1 = min(h_px, h_px // 2 + gap // 2)
        d.rectangle([(0, 0), (line_w - 1, h_px - 1)], fill=LINE_WHITE)            # left edge full
        d.rectangle([(w_px - line_w, 0), (w_px - 1, g0 - 1)], fill=LINE_WHITE)    # right edge before gap
        d.rectangle([(w_px - line_w, g1), (w_px - 1, h_px - 1)], fill=LINE_WHITE)  # right edge after gap
        random.seed(70)
        px = img.load()
        for y in range(g0, g1):
            for dx in range(line_w + m_to_px(0.02)):
                x = w_px - 1 - dx
                if 0 <= x < w_px and random.random() < 0.15:
                    v = random.randint(50, 110)
                    px[x, y] = (v, v, v - 10)
        _dashed_centre(d, width_m, length_m)
        return img

    if theme == "zebra":
        # Pedestrian crossing + stop line centred on the tile; centreline
        # interrupted across the crossing. (lot2 g1_01, parametric length.)
        img = _asphalt_noise(_canvas(width_m, length_m))
        d = ImageDraw.Draw(img)
        _edges(d, width_m, length_m)
        cz = length_m / 2.0
        z0_m, z1_m = cz - 0.4, cz + 0.4
        _dashed_centre(d, width_m, length_m, y0_m=0.0, y1_m=max(0.0, z0_m - 0.05))
        _dashed_centre(d, width_m, length_m, y0_m=z1_m + 0.05, y1_m=length_m)
        z0, z1 = m_to_px(z0_m), m_to_px(z1_m)
        stripe_w, stripe_gap = m_to_px(0.06), m_to_px(0.04)
        x = line_w + m_to_px(0.01)
        while x + stripe_w < w_px - line_w - m_to_px(0.01):
            d.rectangle([(x, z0), (x + stripe_w - 1, z1 - 1)], fill=LINE_WHITE)
            x += stripe_w + stripe_gap
        stop_y = m_to_px(z0_m - 0.05)
        d.rectangle([(line_w, stop_y - line_w - 1), (w_px - line_w - 1, stop_y - 1)], fill=LINE_WHITE)
        return img

    if theme == "double_solid":
        # Double solid centreline (no-passing). Benign: geometry unchanged,
        # so a well-behaved policy/cage should not react. (lot2 g3_02.)
        img = _asphalt_noise(_canvas(width_m, length_m))
        d = ImageDraw.Draw(img)
        _edges(d, width_m, length_m)
        cx = w_px // 2
        gap_between = m_to_px(0.02)
        d.rectangle([(cx - gap_between // 2 - line_w, 0), (cx - gap_between // 2 - 1, h_px - 1)], fill=LINE_WHITE)
        d.rectangle([(cx + gap_between // 2, 0), (cx + gap_between // 2 + line_w - 1, h_px - 1)], fill=LINE_WHITE)
        return img

    if theme == "arrows":
        img = _asphalt_noise(_canvas(width_m, length_m))
        d = ImageDraw.Draw(img)
        _edges(d, width_m, length_m)
        _dashed_centre(d, width_m, length_m)
        right_lane_cx = line_w + (w_px - 3 * line_w) // 4 + line_w
        _draw_arrow(d, right_lane_cx, m_to_px(length_m * 0.3), direction="straight")
        _draw_arrow(d, right_lane_cx, m_to_px(length_m * 0.75), direction="right")
        return img

    raise ValueError(f"Unknown theme {theme!r}; valid: {', '.join(THEMES)}")


def make_tile(theme: str, length_m: float, out_dir, width_m: float = TWO_LANE_TOTAL_M,
              force: bool = False) -> Path:
    """Generate (or reuse) a tile PNG at the exact length. Returns the path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / tile_filename(theme, length_m, width_m)
    if path.exists() and not force:
        return path
    img = _render(theme, length_m, width_m)
    img.save(path, "PNG", optimize=True)
    return path


def main() -> None:
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description="Regenerate the canonical road-tile set.")
    p.add_argument("--length", type=float, default=4.0, help="Tile length in metres.")
    p.add_argument("--out", type=Path, default=here, help="Output directory.")
    p.add_argument("--force", action="store_true", help="Overwrite existing PNGs.")
    p.add_argument("--themes", nargs="*", default=list(THEMES))
    args = p.parse_args()
    print(f"Generating {len(args.themes)} tiles at {args.length} m, {PX_PER_M} px/m -> {args.out}")
    for theme in args.themes:
        path = make_tile(theme, args.length, args.out, force=args.force)
        im = Image.open(path)
        print(f"  {path.name:<40s} {im.size[0]:>4d} x {im.size[1]:<5d} px")
    print("Done.")


if __name__ == "__main__":
    main()
