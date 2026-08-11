"""
Curved road texture modules for Gazebo circuit building.

Generates 16 PNG-RGBA tiles: 4 radii x 4 angles.

Geometry:
  - Two-lane road, 52 cm total width, 24.5 cm useful per lane
  - Radius is measured to the CENTRELINE
  - Inner edge = R - 26 cm, outer edge = R + 26 cm (half of 52 cm)
  - Arc angles: 30, 45, 90, 180 degrees
  - Radii: 30, 50, 80, 120 cm (centreline)

Rendering:
  - 500 px/m resolution
  - Transparent RGBA background outside the road surface
  - Supersampled 3x then downsampled for clean anti-aliased edges
  - Lines are the same 1 cm white, 10 cm dashes with 10 cm gaps, measured
    along the arc of the centreline

Tile endpoints:
  - Entry endpoint: bottom of tile, road pointing up (+Y in image, -Y on
    the ground when Z is up)
  - Exit endpoint: where the arc ends after sweeping by the given angle
    counter-clockwise around the centre of curvature
  - Centre of curvature is always to the LEFT of the entry heading, i.e.
    the tile represents a LEFT turn. Mirror horizontally in Gazebo for
    a right turn.
"""

from PIL import Image, ImageDraw
import math
import os

BASE_PX_PER_M = 500
SUPERSAMPLE = 3
PX_PER_M = BASE_PX_PER_M * SUPERSAMPLE

ROAD_WIDTH_M = 0.52
HALF_WIDTH_M = ROAD_WIDTH_M / 2
LINE_WIDTH_M = 0.01
LANE_USEFUL_M = 0.245
DASH_MARK_M = 0.10
DASH_GAP_M = 0.10

ASPHALT = (0, 0, 0, 255)
LINE_WHITE = (255, 255, 255, 255)
TRANSPARENT = (0, 0, 0, 0)

RADII_M = [0.30, 0.50, 0.80, 1.20]
ANGLES_DEG = [30, 45, 90, 180]

OUT_DIR = "/mnt/user-data/outputs/road_curves"
os.makedirs(OUT_DIR, exist_ok=True)


def m_to_px(m):
    return int(round(m * PX_PER_M))


def compute_bbox(radius_m, angle_deg):
    """Compute the tight bounding box for a left-turning arc.

    Entry is at (0, 0) on the centreline, heading +Y.
    Centre of curvature is at (-R, 0).
    The arc sweeps from angle -90 deg to -90 deg + angle_deg
    (measured standard math convention, CCW positive).

    Returns (xmin, ymin, xmax, ymax) in metres, in the tile's local frame.
    """
    outer_r = radius_m + HALF_WIDTH_M
    inner_r = max(0.0, radius_m - HALF_WIDTH_M)
    cx = -radius_m
    cy = 0.0

    theta_start = -math.pi / 2
    theta_end = theta_start + math.radians(angle_deg)

    xs, ys = [], []
    n = 400
    for i in range(n + 1):
        t = theta_start + (theta_end - theta_start) * i / n
        for r in (outer_r, inner_r):
            xs.append(cx + r * math.cos(t))
            ys.append(cy + r * math.sin(t))

    margin = 0.02
    return (
        min(xs) - margin,
        min(ys) - margin,
        max(xs) + margin,
        max(ys) + margin,
    )


def make_curve_tile(radius_m, angle_deg):
    xmin, ymin, xmax, ymax = compute_bbox(radius_m, angle_deg)
    width_m = xmax - xmin
    height_m = ymax - ymin

    w_px = m_to_px(width_m)
    h_px = m_to_px(height_m)

    img = Image.new("RGBA", (w_px, h_px), TRANSPARENT)
    draw = ImageDraw.Draw(img)

    def world_to_px(x_m, y_m):
        u = (x_m - xmin) * PX_PER_M
        v = h_px - (y_m - ymin) * PX_PER_M
        return (u, v)

    cx_m = -radius_m
    cy_m = 0.0
    cx_px, cy_px = world_to_px(cx_m, cy_m)

    outer_r_m = radius_m + HALF_WIDTH_M
    inner_r_m = max(0.0, radius_m - HALF_WIDTH_M)
    outer_r_px = outer_r_m * PX_PER_M
    inner_r_px = inner_r_m * PX_PER_M

    theta_start = -math.pi / 2
    theta_end = theta_start + math.radians(angle_deg)

    def arc_theta_to_pil_angle(theta_rad):
        """PIL: 0 deg = +X axis, increases CLOCKWISE. Image Y points down.
        World Y points up. So world_theta maps to PIL angle by negation."""
        return -math.degrees(theta_rad)

    n_poly = max(200, int(angle_deg * 4))
    outer_points = []
    for i in range(n_poly + 1):
        t = theta_start + (theta_end - theta_start) * i / n_poly
        x = cx_m + outer_r_m * math.cos(t)
        y = cy_m + outer_r_m * math.sin(t)
        outer_points.append(world_to_px(x, y))
    inner_points = []
    for i in range(n_poly, -1, -1):
        t = theta_start + (theta_end - theta_start) * i / n_poly
        x = cx_m + inner_r_m * math.cos(t)
        y = cy_m + inner_r_m * math.sin(t)
        inner_points.append(world_to_px(x, y))
    asphalt_poly = outer_points + inner_points
    draw.polygon(asphalt_poly, fill=ASPHALT)

    line_half_px = max(1, int(round(LINE_WIDTH_M * PX_PER_M / 2)))

    def draw_arc_line_at_radius(r_m, dashed=False):
        r_px = r_m * PX_PER_M
        r_outer = r_px + line_half_px
        r_inner = max(0, r_px - line_half_px)

        if dashed:
            arc_len_m = r_m * math.radians(angle_deg)
            period_m = DASH_MARK_M + DASH_GAP_M
            n_dashes = int(arc_len_m / period_m) + 1
            for i in range(n_dashes):
                s_start = i * period_m
                s_end = s_start + DASH_MARK_M
                if s_start > arc_len_m:
                    break
                s_end = min(s_end, arc_len_m)
                t0 = theta_start + s_start / r_m
                t1 = theta_start + s_end / r_m

                pts_out = []
                pts_in = []
                nn = max(6, int((t1 - t0) * 180 / math.pi * 4))
                for j in range(nn + 1):
                    t = t0 + (t1 - t0) * j / nn
                    xo = cx_m + (r_m + LINE_WIDTH_M / 2) * math.cos(t)
                    yo = cy_m + (r_m + LINE_WIDTH_M / 2) * math.sin(t)
                    xi = cx_m + (r_m - LINE_WIDTH_M / 2) * math.cos(t)
                    yi = cy_m + (r_m - LINE_WIDTH_M / 2) * math.sin(t)
                    pts_out.append(world_to_px(xo, yo))
                    pts_in.append(world_to_px(xi, yi))
                poly = pts_out + list(reversed(pts_in))
                draw.polygon(poly, fill=LINE_WHITE)
        else:
            pts_out = []
            pts_in = []
            nn = max(200, int(angle_deg * 4))
            for j in range(nn + 1):
                t = theta_start + (theta_end - theta_start) * j / nn
                xo = cx_m + (r_m + LINE_WIDTH_M / 2) * math.cos(t)
                yo = cy_m + (r_m + LINE_WIDTH_M / 2) * math.sin(t)
                xi = cx_m + (r_m - LINE_WIDTH_M / 2) * math.cos(t)
                yi = cy_m + (r_m - LINE_WIDTH_M / 2) * math.sin(t)
                pts_out.append(world_to_px(xo, yo))
                pts_in.append(world_to_px(xi, yi))
            poly = pts_out + list(reversed(pts_in))
            draw.polygon(poly, fill=LINE_WHITE)

    if inner_r_m > 0:
        draw_arc_line_at_radius(inner_r_m, dashed=False)
    draw_arc_line_at_radius(outer_r_m, dashed=False)
    draw_arc_line_at_radius(radius_m, dashed=True)

    final_w = w_px // SUPERSAMPLE
    final_h = h_px // SUPERSAMPLE
    img = img.resize((final_w, final_h), Image.LANCZOS)

    entry_px = (int((0 - xmin) * BASE_PX_PER_M),
                final_h - int((0 - ymin) * BASE_PX_PER_M))
    exit_theta = theta_end
    exit_x = cx_m + radius_m * math.cos(exit_theta)
    exit_y = cy_m + radius_m * math.sin(exit_theta)
    exit_px = (int((exit_x - xmin) * BASE_PX_PER_M),
               final_h - int((exit_y - ymin) * BASE_PX_PER_M))
    exit_heading_deg = angle_deg

    metadata = {
        "radius_m": radius_m,
        "angle_deg": angle_deg,
        "bbox_m": (xmax - xmin, ymax - ymin),
        "image_px": (final_w, final_h),
        "entry_px": entry_px,
        "exit_px": exit_px,
        "exit_heading_deg": exit_heading_deg,
    }
    return img, metadata


def save(img, name, description, metadata):
    path = os.path.join(OUT_DIR, name)
    img.save(path, "PNG", optimize=True)
    bw, bh = metadata["bbox_m"]
    print(f"  {name:<45s}  {img.size[0]:>4d} x {img.size[1]:>4d} px  "
          f"({bw*100:5.1f} x {bh*100:5.1f} cm)  |  {description}")


print(f"Generating {len(RADII_M) * len(ANGLES_DEG)} curve tiles at "
      f"{BASE_PX_PER_M} px/m (supersampled x{SUPERSAMPLE})")
print(f"Road width: {ROAD_WIDTH_M*100:.0f} cm | Line width: {LINE_WIDTH_M*100:.0f} cm")
print()

metadata_log = []
for radius in RADII_M:
    print(f"RADIUS {radius*100:.0f} cm")
    print("-" * 78)
    for angle in ANGLES_DEG:
        img, md = make_curve_tile(radius, angle)
        r_str = f"{int(round(radius*100)):03d}"
        a_str = f"{angle:03d}"
        name = f"curve_R{r_str}cm_A{a_str}deg.png"
        desc = f"R={radius*100:.0f} cm, arc={angle}°, left turn"
        save(img, name, desc, md)
        metadata_log.append((name, md))
    print()

print("Summary of tile dimensions (for Gazebo SDF box sizes):")
print()
print(f"  {'Tile':<40s}  {'Width (m)':<10s}  {'Height (m)':<10s}")
for name, md in metadata_log:
    bw, bh = md["bbox_m"]
    print(f"  {name:<40s}  {bw:>8.3f}    {bh:>8.3f}")

print()
print("Done.")
