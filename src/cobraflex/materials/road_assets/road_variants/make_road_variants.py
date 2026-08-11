"""
Road texture generator for domain randomization in Gazebo.

Produces 9 variants across three road types (single, two-lane same dir,
two-lane opposite dir). Each type has a clean baseline plus 2 adversarial
variants with moderate real-world degradation.

All distances in metres. Output images are oriented with the road running
along the image Y axis (tall images).
"""

from PIL import Image, ImageDraw, ImageFilter
import random
import os
import math

PX_PER_M = 500
ASPHALT = (0, 0, 0)
LINE_WHITE = (255, 255, 255)

LINE_WIDTH_M = 0.01
DASH_MARK_M = 0.10
DASH_GAP_M = 0.10
LANE_USEFUL_M = 0.245

SINGLE_TOTAL_M = LANE_USEFUL_M + 2 * LINE_WIDTH_M
TWO_LANE_TOTAL_M = 2 * LANE_USEFUL_M + 3 * LINE_WIDTH_M

# Fixed-length 10 m / 5 m source PNGs. For tiles sized to a specific Gazebo
# box length (no UV compression) and named with the real size in metres, use
# the length-parametric factory ../road_tiles/make_road_tiles.py instead.
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT_DIR, exist_ok=True)


def m_to_px(m):
    return int(round(m * PX_PER_M))


def make_canvas(width_m, length_m, base_color=ASPHALT):
    w = m_to_px(width_m)
    h = m_to_px(length_m)
    return Image.new("RGB", (w, h), base_color)


def add_asphalt_noise(img, strength=6, density_divisor=60):
    """Subtle per-pixel grain so the asphalt does not read as flat plastic."""
    random.seed(42)
    px = img.load()
    w, h = img.size
    for _ in range(w * h // density_divisor):
        x = random.randint(0, w - 1)
        y = random.randint(0, h - 1)
        r, g, b = px[x, y]
        delta = random.randint(-strength, strength)
        px[x, y] = (
            max(0, min(255, r + delta)),
            max(0, min(255, g + delta)),
            max(0, min(255, b + delta)),
        )
    return img


def draw_continuous_edges(draw, width_m, length_m, color=LINE_WHITE):
    line_w = m_to_px(LINE_WIDTH_M)
    w = m_to_px(width_m)
    h = m_to_px(length_m)
    draw.rectangle([(0, 0), (line_w - 1, h - 1)], fill=color)
    draw.rectangle([(w - line_w, 0), (w - 1, h - 1)], fill=color)


def draw_dashed_centreline(draw, width_m, length_m, color=LINE_WHITE):
    line_w = m_to_px(LINE_WIDTH_M)
    w = m_to_px(width_m)
    h = m_to_px(length_m)
    cx = w // 2
    hl = line_w // 2
    hr = line_w - hl
    dash = m_to_px(DASH_MARK_M)
    gap = m_to_px(DASH_GAP_M)
    period = dash + gap
    y = 0
    while y < h:
        draw.rectangle(
            [(cx - hl, y), (cx + hr - 1, min(y + dash, h) - 1)], fill=color
        )
        y += period


def apply_line_wear(img, width_m, length_m, wear_level=0.3, seed=1):
    """Erode line pixels stochastically to simulate faded paint.

    wear_level is the probability a given line pixel gets darkened or removed.
    """
    random.seed(seed)
    w, h = img.size
    px = img.load()
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if r > 200 and g > 200 and b > 200:
                if random.random() < wear_level:
                    fade = random.randint(30, 120)
                    new_r = max(0, r - fade - random.randint(0, 60))
                    new_g = max(0, g - fade - random.randint(0, 60))
                    new_b = max(0, b - fade - random.randint(0, 60))
                    px[x, y] = (new_r, new_g, new_b)
    return img


def add_patches(img, n=6, seed=2):
    """Dark rectangular repair patches on the asphalt."""
    random.seed(seed)
    draw = ImageDraw.Draw(img)
    w, h = img.size
    for _ in range(n):
        patch_w = random.randint(m_to_px(0.08), m_to_px(0.20))
        patch_h = random.randint(m_to_px(0.15), m_to_px(0.50))
        x = random.randint(m_to_px(0.02), w - patch_w - m_to_px(0.02))
        y = random.randint(m_to_px(0.10), h - patch_h - m_to_px(0.10))
        shade = random.randint(20, 45)
        draw.rectangle(
            [(x, y), (x + patch_w, y + patch_h)],
            fill=(shade, shade, shade + random.randint(-3, 3)),
        )
    return img


def add_potholes_and_stains(img, n_potholes=4, n_stains=5, seed=3):
    """Moderate potholes (dark irregular blobs) and oil stains (darker circles)."""
    random.seed(seed)
    draw = ImageDraw.Draw(img)
    w, h = img.size
    for _ in range(n_potholes):
        cx = random.randint(m_to_px(0.03), w - m_to_px(0.03))
        cy = random.randint(m_to_px(0.1), h - m_to_px(0.1))
        r = random.randint(m_to_px(0.015), m_to_px(0.04))
        draw.ellipse(
            [(cx - r, cy - r), (cx + r, cy + r)], fill=(15, 15, 17)
        )
        r2 = r + random.randint(2, 6)
        for _ in range(3):
            draw.arc(
                [(cx - r2, cy - r2), (cx + r2, cy + r2)],
                start=random.randint(0, 360),
                end=random.randint(0, 360),
                fill=(30, 30, 32),
                width=1,
            )
    for _ in range(n_stains):
        cx = random.randint(m_to_px(0.03), w - m_to_px(0.03))
        cy = random.randint(m_to_px(0.1), h - m_to_px(0.1))
        r = random.randint(m_to_px(0.04), m_to_px(0.10))
        draw.ellipse(
            [(cx - r, cy - r), (cx + r, cy + r)], fill=(18, 16, 14)
        )
    return img


def add_dirt_sprinkle(img, density=0.003, seed=4):
    """Sparse brownish grit points scattered over the surface."""
    random.seed(seed)
    px = img.load()
    w, h = img.size
    n = int(w * h * density)
    for _ in range(n):
        x = random.randint(0, w - 1)
        y = random.randint(0, h - 1)
        kind = random.random()
        if kind < 0.5:
            px[x, y] = (random.randint(55, 85), random.randint(40, 65), random.randint(25, 45))
        else:
            g = random.randint(60, 100)
            px[x, y] = (g, g, g)
    return img


def blur_lines(img, radius=1.2):
    """Light gaussian blur to soften paint edges (rain / wet road look)."""
    return img.filter(ImageFilter.GaussianBlur(radius=radius))


def add_wet_sheen(img, seed=5):
    """Slight bluish tint in random horizontal bands to suggest wet patches."""
    random.seed(seed)
    px = img.load()
    w, h = img.size
    n_bands = random.randint(4, 7)
    for _ in range(n_bands):
        y0 = random.randint(0, h - 1)
        band_h = random.randint(m_to_px(0.3), m_to_px(1.2))
        strength = random.randint(6, 14)
        for y in range(y0, min(y0 + band_h, h)):
            for x in range(w):
                r, g, b = px[x, y]
                px[x, y] = (
                    min(255, r + strength // 3),
                    min(255, g + strength // 2),
                    min(255, b + strength),
                )
    return img


def dirt_road_base(width_m, length_m, seed=10):
    """Brown-grey gravel/dirt surface instead of asphalt."""
    random.seed(seed)
    img = Image.new("RGB", (m_to_px(width_m), m_to_px(length_m)), (92, 70, 48))
    px = img.load()
    w, h = img.size
    for _ in range(w * h // 8):
        x = random.randint(0, w - 1)
        y = random.randint(0, h - 1)
        kind = random.random()
        if kind < 0.6:
            r = random.randint(75, 115)
            g = random.randint(58, 88)
            b = random.randint(35, 58)
            px[x, y] = (r, g, b)
        elif kind < 0.85:
            v = random.randint(130, 170)
            px[x, y] = (v, v - 10, v - 25)
        else:
            v = random.randint(45, 70)
            px[x, y] = (v, v - 8, v - 20)
    return img


def save(img, name, description):
    path = os.path.join(OUT_DIR, name)
    img.save(path, "PNG", optimize=True)
    print(f"  {name:<58s}  {img.size[0]:>4d} x {img.size[1]:>5d} px  |  {description}")


def draw_side_road_tee(img, main_width_m, length_m, side_width_m, side_y_m, side_side="right"):
    """Carve a perpendicular side road opening in the edge line.

    The side road itself is drawn as asphalt extending to the image edge with
    its own edge lines, so when the main road tile butts up against a side
    tile the transition looks continuous.
    """
    draw = ImageDraw.Draw(img)
    line_w = m_to_px(LINE_WIDTH_M)
    main_w_px = m_to_px(main_width_m)
    side_w_px = m_to_px(side_width_m)
    side_y_px = m_to_px(side_y_m)

    gap_start = side_y_px - side_w_px // 2
    gap_end = side_y_px + side_w_px // 2

    if side_side == "right":
        draw.rectangle(
            [(main_w_px - line_w, gap_start), (main_w_px - 1, gap_end)],
            fill=ASPHALT,
        )
        marker_x = main_w_px - line_w - m_to_px(0.03)
    else:
        draw.rectangle(
            [(0, gap_start), (line_w - 1, gap_end)], fill=ASPHALT
        )
        marker_x = line_w + m_to_px(0.03)

    stop_len = m_to_px(0.04)
    draw.rectangle(
        [(marker_x - stop_len // 2, side_y_px - line_w // 2),
         (marker_x + stop_len // 2, side_y_px + line_w // 2)],
        fill=LINE_WHITE,
    )
    return img


print("Generating 9 road texture variants at", PX_PER_M, "px/m")
print()
print("SINGLE LANE (26.5 cm total)")
print("-" * 78)

img = make_canvas(SINGLE_TOTAL_M, 10.0)
img = add_asphalt_noise(img, strength=5)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, SINGLE_TOTAL_M, 10.0)
save(img, "single_01_clean.png", "clean baseline")

img = make_canvas(SINGLE_TOTAL_M, 10.0)
img = add_asphalt_noise(img, strength=8, density_divisor=40)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, SINGLE_TOTAL_M, 10.0)
img = apply_line_wear(img, SINGLE_TOTAL_M, 10.0, wear_level=0.35, seed=11)
img = add_dirt_sprinkle(img, density=0.004, seed=12)
img = add_potholes_and_stains(img, n_potholes=3, n_stains=4, seed=13)
save(img, "single_02_worn.png", "worn lines, light dirt, potholes")

img = dirt_road_base(SINGLE_TOTAL_M, 10.0, seed=14)
draw = ImageDraw.Draw(img)
line_w = m_to_px(LINE_WIDTH_M)
w_px = m_to_px(SINGLE_TOTAL_M)
h_px = m_to_px(10.0)
for x_edge in [0, w_px - line_w]:
    dash = m_to_px(0.4)
    gap = m_to_px(0.3)
    y = 0
    while y < h_px:
        draw.rectangle(
            [(x_edge, y), (x_edge + line_w - 1, min(y + dash, h_px) - 1)],
            fill=(180, 170, 150),
        )
        y += dash + gap
img = apply_line_wear(img, SINGLE_TOTAL_M, 10.0, wear_level=0.6, seed=15)
img = blur_lines(img, radius=1.5)
save(img, "single_03_dirt_road.png", "dirt/gravel path, faint intermittent edge markers")

print()
print("TWO LANES SAME DIRECTION (52 cm total)")
print("-" * 78)

img = make_canvas(TWO_LANE_TOTAL_M, 10.0)
img = add_asphalt_noise(img, strength=5)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, 10.0)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, 10.0)
save(img, "two_same_01_clean.png", "clean baseline")

img = make_canvas(TWO_LANE_TOTAL_M, 10.0)
img = add_asphalt_noise(img, strength=10, density_divisor=30)
img = add_patches(img, n=7, seed=20)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, 10.0)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, 10.0)
img = apply_line_wear(img, TWO_LANE_TOTAL_M, 10.0, wear_level=0.40, seed=21)
img = add_potholes_and_stains(img, n_potholes=5, n_stains=6, seed=22)
img = add_dirt_sprinkle(img, density=0.002, seed=23)
save(img, "two_same_02_patched.png", "repair patches, faded paint, stains")

img = make_canvas(TWO_LANE_TOTAL_M, 5.0)
img = add_asphalt_noise(img, strength=5)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, 5.0)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, 5.0)
img = draw_side_road_tee(
    img,
    main_width_m=TWO_LANE_TOTAL_M,
    length_m=5.0,
    side_width_m=0.20,
    side_y_m=2.5,
    side_side="right",
)
save(img, "two_same_03_tee_junction.png", "T-junction with side road entrance on right, 5 m long")

print()
print("TWO LANES OPPOSITE DIRECTION (52 cm total)")
print("-" * 78)

img = make_canvas(TWO_LANE_TOTAL_M, 10.0)
img = add_asphalt_noise(img, strength=5)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, 10.0)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, 10.0)
save(img, "two_opp_01_clean.png", "clean baseline")

img = make_canvas(TWO_LANE_TOTAL_M, 10.0)
img = add_asphalt_noise(img, strength=7, density_divisor=35)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, 10.0)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, 10.0)
img = apply_line_wear(img, TWO_LANE_TOTAL_M, 10.0, wear_level=0.25, seed=30)
img = add_wet_sheen(img, seed=31)
img = add_potholes_and_stains(img, n_potholes=2, n_stains=7, seed=32)
img = blur_lines(img, radius=1.8)
save(img, "two_opp_02_wet.png", "wet road, blurred paint, oil stains")

img = make_canvas(TWO_LANE_TOTAL_M, 4.0)
img = add_asphalt_noise(img, strength=6)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, 4.0)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, 4.0)
img = draw_side_road_tee(
    img,
    main_width_m=TWO_LANE_TOTAL_M,
    length_m=4.0,
    side_width_m=0.18,
    side_y_m=2.0,
    side_side="left",
)
save(img, "two_opp_03_side_entrance.png", "side road entrance on left, 4 m long")

print()
print("Done. 9 variants in", OUT_DIR)
