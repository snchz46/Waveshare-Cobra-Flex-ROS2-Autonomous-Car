from PIL import Image, ImageDraw
import os

PX_PER_M = 500
LENGTH_M = 10.0
LENGTH_PX = int(LENGTH_M * PX_PER_M)

ASPHALT = (0, 0, 0)
LINE_WHITE = (255, 255, 255)

LINE_WIDTH_M = 0.01
DASH_MARK_M = 0.10
DASH_GAP_M = 0.10

LANE_USEFUL_M = 0.245

SINGLE_TOTAL_M = LANE_USEFUL_M + 2 * LINE_WIDTH_M
TWO_LANE_TOTAL_M = 2 * LANE_USEFUL_M + 3 * LINE_WIDTH_M

OUT_DIR = "/mnt/user-data/outputs"
os.makedirs(OUT_DIR, exist_ok=True)


def m_to_px(m):
    return int(round(m * PX_PER_M))


def make_image(width_m):
    width_px = m_to_px(width_m)
    return Image.new("RGB", (width_px, LENGTH_PX), ASPHALT)


def draw_continuous_edge_lines(draw, width_m, color=LINE_WHITE):
    line_w_px = m_to_px(LINE_WIDTH_M)
    width_px = m_to_px(width_m)
    draw.rectangle([(0, 0), (line_w_px - 1, LENGTH_PX - 1)], fill=color)
    draw.rectangle(
        [(width_px - line_w_px, 0), (width_px - 1, LENGTH_PX - 1)], fill=color
    )


def draw_dashed_centreline(draw, width_m, color=LINE_WHITE):
    line_w_px = m_to_px(LINE_WIDTH_M)
    width_px = m_to_px(width_m)
    cx = width_px // 2
    half_w_left = line_w_px // 2
    half_w_right = line_w_px - half_w_left

    dash_px = m_to_px(DASH_MARK_M)
    gap_px = m_to_px(DASH_GAP_M)
    period_px = dash_px + gap_px

    y = 0
    while y < LENGTH_PX:
        y_end = min(y + dash_px, LENGTH_PX)
        draw.rectangle(
            [(cx - half_w_left, y), (cx + half_w_right - 1, y_end - 1)],
            fill=color,
        )
        y += period_px


def save(img, filename, description):
    path = os.path.join(OUT_DIR, filename)
    img.save(path, "PNG", optimize=True)
    print(f"  {filename}  -> {img.size[0]}x{img.size[1]} px  |  {description}")


print("Generating road textures")
print(f"  single lane total width: {SINGLE_TOTAL_M*100:.1f} cm")
print(f"  two-lane total width:    {TWO_LANE_TOTAL_M*100:.1f} cm")
print(f"  line width: {LINE_WIDTH_M*100:.1f} cm")
print(f"  dash: {DASH_MARK_M*100:.0f} cm on / {DASH_GAP_M*100:.0f} cm off")
print()

img1 = make_image(SINGLE_TOTAL_M)
draw1 = ImageDraw.Draw(img1)
draw_continuous_edge_lines(draw1, SINGLE_TOTAL_M)
save(
    img1,
    "road_single_lane_0p265m_x_10m.png",
    f"single lane {LANE_USEFUL_M*100:.1f} cm useful + 2x1 cm edges = {SINGLE_TOTAL_M*100:.1f} cm total",
)

img2 = make_image(TWO_LANE_TOTAL_M)
draw2 = ImageDraw.Draw(img2)
draw_continuous_edge_lines(draw2, TWO_LANE_TOTAL_M)
draw_dashed_centreline(draw2, TWO_LANE_TOTAL_M)
save(
    img2,
    "road_two_lane_same_direction_0p52m_x_10m.png",
    f"two lanes same direction, total {TWO_LANE_TOTAL_M*100:.0f} cm, dashed white centreline",
)

img3 = make_image(TWO_LANE_TOTAL_M)
draw3 = ImageDraw.Draw(img3)
draw_continuous_edge_lines(draw3, TWO_LANE_TOTAL_M)
draw_dashed_centreline(draw3, TWO_LANE_TOTAL_M)
save(
    img3,
    "road_two_lane_opposite_direction_0p52m_x_10m.png",
    f"two lanes opposite direction, total {TWO_LANE_TOTAL_M*100:.0f} cm, dashed white centreline",
)

print()
print("Done.")
