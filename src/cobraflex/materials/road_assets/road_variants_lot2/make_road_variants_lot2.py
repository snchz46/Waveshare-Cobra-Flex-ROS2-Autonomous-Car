"""
Modular adversarial road textures, lot 2.

Eight short segments (3-5 m) designed to combine with the 9 textures of lot 1.
Each variant stresses a specific capability: vision estimator, policy planner,
safety cage, or domain shift robustness.

All variants share the same physical spec as lot 1:
  - Pure black asphalt (except dirt-road style where noted)
  - 1 cm white lines, 24.5 cm useful lane, 52 cm two-lane total width
  - 500 px/m resolution
"""

from PIL import Image, ImageDraw, ImageFilter
import random
import math
import os

PX_PER_M = 500
ASPHALT = (0, 0, 0)
LINE_WHITE = (255, 255, 255)

LINE_WIDTH_M = 0.01
DASH_MARK_M = 0.10
DASH_GAP_M = 0.10
LANE_USEFUL_M = 0.245

SINGLE_TOTAL_M = LANE_USEFUL_M + 2 * LINE_WIDTH_M
TWO_LANE_TOTAL_M = 2 * LANE_USEFUL_M + 3 * LINE_WIDTH_M

# Fixed-length 3-5 m source PNGs. For tiles sized to a specific Gazebo box
# length (no UV compression) and named with the real size in metres, use the
# length-parametric factory ../road_tiles/make_road_tiles.py instead.
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT_DIR, exist_ok=True)


def m_to_px(m):
    return int(round(m * PX_PER_M))


def make_canvas(width_m, length_m, base_color=ASPHALT):
    return Image.new("RGB", (m_to_px(width_m), m_to_px(length_m)), base_color)


def add_asphalt_noise(img, strength=5, density_divisor=60, seed=42):
    random.seed(seed)
    px = img.load()
    w, h = img.size
    for _ in range(w * h // density_divisor):
        x = random.randint(0, w - 1)
        y = random.randint(0, h - 1)
        r, g, b = px[x, y]
        d = random.randint(-strength, strength)
        px[x, y] = (
            max(0, min(255, r + d)),
            max(0, min(255, g + d)),
            max(0, min(255, b + d)),
        )
    return img


def draw_continuous_edges(draw, width_m, length_m, color=LINE_WHITE):
    line_w = m_to_px(LINE_WIDTH_M)
    w = m_to_px(width_m)
    h = m_to_px(length_m)
    draw.rectangle([(0, 0), (line_w - 1, h - 1)], fill=color)
    draw.rectangle([(w - line_w, 0), (w - 1, h - 1)], fill=color)


def draw_dashed_centreline(draw, width_m, length_m, color=LINE_WHITE,
                           y_start_m=0.0, y_end_m=None):
    line_w = m_to_px(LINE_WIDTH_M)
    w = m_to_px(width_m)
    h = m_to_px(length_m)
    if y_end_m is None:
        y_end_m = length_m
    cx = w // 2
    hl = line_w // 2
    hr = line_w - hl
    dash = m_to_px(DASH_MARK_M)
    gap = m_to_px(DASH_GAP_M)
    y = m_to_px(y_start_m)
    y_end = m_to_px(y_end_m)
    while y < y_end:
        draw.rectangle(
            [(cx - hl, y), (cx + hr - 1, min(y + dash, y_end) - 1)],
            fill=color,
        )
        y += dash + gap


def save(img, name, description):
    path = os.path.join(OUT_DIR, name)
    img.save(path, "PNG", optimize=True)
    print(f"  {name:<50s}  {img.size[0]:>4d} x {img.size[1]:>5d} px  |  {description}")


print("Generating lot 2: 8 modular adversarial textures (3-5 m each)")
print()
print("GROUP 1 - VISION STRESSORS")
print("-" * 78)

length = 4.0
img = make_canvas(TWO_LANE_TOTAL_M, length)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, length)
line_w = m_to_px(LINE_WIDTH_M)
w_px = m_to_px(TWO_LANE_TOTAL_M)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, length,
                       y_start_m=0, y_end_m=1.5)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, length,
                       y_start_m=2.5, y_end_m=length)
zebra_y0 = m_to_px(1.6)
zebra_y1 = m_to_px(2.4)
stripe_w = m_to_px(0.06)
stripe_gap = m_to_px(0.04)
x = line_w + m_to_px(0.01)
while x + stripe_w < w_px - line_w - m_to_px(0.01):
    draw.rectangle([(x, zebra_y0), (x + stripe_w - 1, zebra_y1 - 1)],
                   fill=LINE_WHITE)
    x += stripe_w + stripe_gap
stop_y = m_to_px(1.55)
draw.rectangle([(line_w, stop_y - line_w - 1),
                (w_px - line_w - 1, stop_y - 1)],
               fill=LINE_WHITE)
save(img, "g1_01_zebra_crossing.png",
     "pedestrian zebra crossing with stop line, 4 m")

length = 5.0
img = make_canvas(TWO_LANE_TOTAL_M, length)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, length)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, length)

def draw_arrow(draw, cx_px, cy_px, direction="straight", size_m=0.18):
    size_px = m_to_px(size_m)
    shaft_w = m_to_px(0.03)
    head_w = m_to_px(0.08)
    head_len = m_to_px(0.06)
    if direction == "straight":
        top = cy_px - size_px // 2
        bot = cy_px + size_px // 2
        draw.rectangle(
            [(cx_px - shaft_w // 2, top + head_len),
             (cx_px + shaft_w // 2, bot)],
            fill=LINE_WHITE,
        )
        draw.polygon(
            [(cx_px, top),
             (cx_px - head_w // 2, top + head_len),
             (cx_px + head_w // 2, top + head_len)],
            fill=LINE_WHITE,
        )
    elif direction == "right":
        left = cx_px - size_px // 3
        right = cx_px + size_px // 2
        top = cy_px - size_px // 2
        bot = cy_px + size_px // 3
        draw.rectangle(
            [(left, cy_px - shaft_w // 2),
             (right - head_len, cy_px + shaft_w // 2)],
            fill=LINE_WHITE,
        )
        draw.rectangle(
            [(left, cy_px - shaft_w // 2),
             (left + shaft_w, bot)],
            fill=LINE_WHITE,
        )
        draw.polygon(
            [(right, cy_px),
             (right - head_len, cy_px - head_w // 2),
             (right - head_len, cy_px + head_w // 2)],
            fill=LINE_WHITE,
        )

right_lane_cx = line_w + (w_px - 3 * line_w) // 4 + line_w
draw_arrow(draw, cx_px=right_lane_cx, cy_px=m_to_px(1.2), direction="straight")
draw_arrow(draw, cx_px=right_lane_cx, cy_px=m_to_px(3.5), direction="right")
save(img, "g1_02_lane_arrows.png",
     "painted directional arrows in right lane, 5 m")

print()
print("GROUP 2 - POLICY STRESSORS")
print("-" * 78)

length = 5.0
img = make_canvas(TWO_LANE_TOTAL_M + 0.05, length)
w_px = m_to_px(TWO_LANE_TOTAL_M + 0.05)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
for y_px in range(m_to_px(length)):
    y_m = y_px / PX_PER_M
    t = y_m / length
    curve_progress = max(0.0, (t - 0.2) / 0.8)
    max_offset_m = 0.15
    offset_m = max_offset_m * (1 - math.cos(curve_progress * math.pi)) / 2
    offset_px = m_to_px(offset_m)
    center = w_px // 2 + offset_px
    half = m_to_px(TWO_LANE_TOTAL_M) // 2
    left_edge = center - half
    right_edge = center + half
    draw.line([(left_edge, y_px), (left_edge + line_w - 1, y_px)],
              fill=LINE_WHITE, width=1)
    draw.line([(right_edge - line_w + 1, y_px), (right_edge, y_px)],
              fill=LINE_WHITE, width=1)
    dash_cycle = m_to_px(DASH_MARK_M + DASH_GAP_M)
    if (y_px % dash_cycle) < m_to_px(DASH_MARK_M):
        draw.line([(center - line_w // 2, y_px),
                   (center + line_w // 2, y_px)],
                  fill=LINE_WHITE, width=1)
save(img, "g2_01_sharp_curve.png",
     "sharp curve, 15 cm lateral shift over 4 m, 5 m total")

length = 5.0
img = make_canvas(TWO_LANE_TOTAL_M, length)
w_px = m_to_px(TWO_LANE_TOTAL_M)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
for y_px in range(m_to_px(length)):
    y_m = y_px / PX_PER_M
    if y_m < 1.5:
        left_x = 0
        right_x = w_px
    elif y_m < 2.5:
        t = (y_m - 1.5) / 1.0
        reduction = t * m_to_px(0.08)
        left_x = reduction
        right_x = w_px - reduction
    elif y_m < 3.5:
        left_x = m_to_px(0.08)
        right_x = w_px - m_to_px(0.08)
    elif y_m < 4.5:
        t = (y_m - 3.5) / 1.0
        reduction = (1 - t) * m_to_px(0.08)
        left_x = reduction
        right_x = w_px - reduction
    else:
        left_x = 0
        right_x = w_px
    draw.rectangle([(left_x, y_px), (left_x + line_w - 1, y_px)],
                   fill=LINE_WHITE)
    draw.rectangle([(right_x - line_w, y_px), (right_x - 1, y_px)],
                   fill=LINE_WHITE)
    center = (left_x + right_x) // 2
    dash_cycle = m_to_px(DASH_MARK_M + DASH_GAP_M)
    if (y_px % dash_cycle) < m_to_px(DASH_MARK_M):
        draw.line([(center - line_w // 2, y_px),
                   (center + line_w // 2, y_px)],
                  fill=LINE_WHITE, width=1)
random.seed(60)
for i in range(6):
    cone_y = m_to_px(1.5) + i * m_to_px(0.3)
    cone_x = m_to_px(0.02)
    cone_size = m_to_px(0.03)
    draw.ellipse([(cone_x, cone_y),
                  (cone_x + cone_size, cone_y + cone_size)],
                 fill=(220, 90, 20))
save(img, "g2_02_lane_narrowing.png",
     "lane narrowing from 52 to 36 cm with cones, 5 m")

print()
print("GROUP 3 - CAGE STRESSORS")
print("-" * 78)

length = 4.0
img = make_canvas(TWO_LANE_TOTAL_M, length)
w_px = m_to_px(TWO_LANE_TOTAL_M)
h_px = m_to_px(length)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
gap_start = m_to_px(1.2)
gap_end = m_to_px(2.4)
draw.rectangle([(0, 0), (line_w - 1, h_px - 1)], fill=LINE_WHITE)
draw.rectangle([(w_px - line_w, 0), (w_px - 1, gap_start - 1)],
               fill=LINE_WHITE)
draw.rectangle([(w_px - line_w, gap_end), (w_px - 1, h_px - 1)],
               fill=LINE_WHITE)
random.seed(70)
px = img.load()
for y in range(gap_start, gap_end):
    for dx in range(line_w + m_to_px(0.02)):
        x = w_px - 1 - dx
        if 0 <= x < w_px:
            if random.random() < 0.15:
                v = random.randint(50, 110)
                px[x, y] = (v, v, v - 10)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, length)
save(img, "g3_01_edge_line_gap.png",
     "1.2 m gap in right edge line simulating worn pavement, 4 m")

length = 5.0
img = make_canvas(TWO_LANE_TOTAL_M, length)
w_px = m_to_px(TWO_LANE_TOTAL_M)
h_px = m_to_px(length)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, length)
cx = w_px // 2
gap_between = m_to_px(0.02)
draw.rectangle([(cx - gap_between // 2 - line_w, 0),
                (cx - gap_between // 2 - 1, h_px - 1)],
               fill=LINE_WHITE)
draw.rectangle([(cx + gap_between // 2, 0),
                (cx + gap_between // 2 + line_w - 1, h_px - 1)],
               fill=LINE_WHITE)
save(img, "g3_02_double_solid_centre.png",
     "double solid centreline (no-passing zone), 5 m")

print()
print("GROUP 4 - DOMAIN SHIFT")
print("-" * 78)

length = 4.0
img = make_canvas(TWO_LANE_TOTAL_M, length)
w_px = m_to_px(TWO_LANE_TOTAL_M)
h_px = m_to_px(length)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, length)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, length)
random.seed(80)
def draw_leaf(draw, cx, cy, size_px, colour):
    pts = []
    for ang_deg in range(0, 360, 30):
        ang = math.radians(ang_deg)
        r_var = size_px * (0.6 + 0.4 * random.random())
        pts.append((cx + r_var * math.cos(ang),
                    cy + r_var * math.sin(ang)))
    draw.polygon(pts, fill=colour)

leaf_palette = [
    (110, 55, 20), (130, 70, 25), (155, 90, 30),
    (90, 45, 18), (175, 105, 40), (80, 40, 15),
]
n_leaves = 40
for _ in range(n_leaves):
    cx = random.randint(line_w, w_px - line_w)
    cy = random.randint(0, h_px - 1)
    size = random.randint(m_to_px(0.015), m_to_px(0.04))
    colour = random.choice(leaf_palette)
    draw_leaf(draw, cx, cy, size, colour)
for _ in range(80):
    cx = random.randint(0, w_px - 1)
    cy = random.randint(0, h_px - 1)
    size = random.randint(1, m_to_px(0.008))
    colour = random.choice(leaf_palette)
    draw.ellipse([(cx, cy), (cx + size, cy + size)], fill=colour)
save(img, "g4_01_fallen_leaves.png",
     "autumn fallen leaves scattered on road, 4 m")

length = 3.0
img = make_canvas(TWO_LANE_TOTAL_M, length)
w_px = m_to_px(TWO_LANE_TOTAL_M)
h_px = m_to_px(length)
img = add_asphalt_noise(img)
draw = ImageDraw.Draw(img)
draw_continuous_edges(draw, TWO_LANE_TOTAL_M, length)
draw_dashed_centreline(draw, TWO_LANE_TOTAL_M, length)
right_lane_cx = w_px - line_w - m_to_px(LANE_USEFUL_M) // 2
drain_w = m_to_px(0.15)
drain_h = m_to_px(0.25)
drain_cy = m_to_px(1.5)
drain_x0 = right_lane_cx - drain_w // 2
drain_y0 = drain_cy - drain_h // 2
draw.rectangle([(drain_x0, drain_y0),
                (drain_x0 + drain_w - 1, drain_y0 + drain_h - 1)],
               fill=(42, 42, 46))
draw.rectangle([(drain_x0, drain_y0),
                (drain_x0 + drain_w - 1, drain_y0 + drain_h - 1)],
               outline=(75, 75, 78), width=2)
n_slots = 8
slot_h = drain_h // (n_slots * 2)
for i in range(n_slots):
    y = drain_y0 + slot_h + i * 2 * slot_h
    draw.rectangle([(drain_x0 + m_to_px(0.01), y),
                    (drain_x0 + drain_w - m_to_px(0.01) - 1, y + slot_h - 1)],
                   fill=(15, 15, 18))
save(img, "g4_02_drain_cover.png",
     "metal drain cover with slots in right lane, 3 m")

print()
print("Done. 8 modular variants in", OUT_DIR)
