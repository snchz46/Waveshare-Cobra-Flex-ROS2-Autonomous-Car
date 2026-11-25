"""Size density of the window: desktop (mouse, large monitor) or touch (finger, small screen).

Sizes in style.qss, style_touch.qss and the widgets are density-independent
pixels (dp). A density sets the size tokens of the style sheets and a factor
that converts dp into screen pixels. The desktop density keeps the sizes that
the window had before densities existed, at factor 1.

The touch density is sized for the screen of the robot, a Joy-IT RB-LCD7-2
(7 inch, 1024 x 600 px over 154.21 x 85.92 mm, about 0.15 mm per pixel). At
factor 1.25 a 50 dp control measures about 9 mm, the usual minimum for a
finger. The factor multiplies the sizes in the application instead of going
through QT_SCALE_FACTOR: fractional Qt 5 scale factors draw uneven one-pixel
lines.

Free of Qt imports, so that the style rendering is testable without a display.
"""

from dataclasses import dataclass, replace
from pathlib import Path
import re
from string import Template
from typing import Dict, Optional, Tuple

from cobraflex_launch_gui.constants import (
    DEFAULT_WINDOW_HEIGHT,
    DEFAULT_WINDOW_WIDTH,
    LOGS_PANEL_MAX_HEIGHT,
    MIN_WINDOW_HEIGHT,
    MIN_WINDOW_WIDTH,
    PALETTE_HEIGHT,
    PALETTE_WIDTH,
)

_HERE = Path(__file__).resolve().parent
STYLE_FILE = _HERE / "style.qss"
TOUCH_STYLE_FILE = _HERE / "style_touch.qss"

#: A pixel length in a style sheet. The look-behind keeps object names and
#: colours out of the match.
_PX_RE = re.compile(r"(?<![\w.#-])(\d+(?:\.\d+)?)px")

#: Size tokens, in dp, as (desktop, touch). The desktop values are the sizes
#: of the style sheet before the touch density existed; tokens with a desktop
#: value of 0 are used by style_touch.qss only.
TOKENS: Dict[str, Tuple[float, float]] = {
    # Font sizes.
    "fs_tiny": (10, 12),
    "fs_small": (11, 13),
    "fs_meta": (12, 14),
    "fs_body": (13, 16),
    "fs_large": (14, 17),
    "fs_icon": (16, 18),
    "fs_title": (20, 22),
    # Minimum heights of controls; 50 dp is about 9 mm on the touch screen.
    "ctl_xs": (20, 44),
    "ctl_s": (22, 48),
    "ctl_m": (24, 48),
    "ctl_l": (26, 50),
    "ctl_chip": (0, 48),
    "ctl_tab": (0, 50),
    "ctl_item": (0, 48),
    "expand": (24, 44),
    "star_min": (18, 44),
    "star_max": (22, 48),
    "check": (14, 26),
    # With kinetic scrolling the scroll bar is an indicator, not a handle.
    "scrollbar": (10, 12),
    # Sizes set from Python rather than from the style sheet.
    "search_h": (32, 50),
    "search_w": (320, 200),
    "arg_key_w": (170, 150),
    "details_w": (320, 300),
    "logs_max_h": (LOGS_PANEL_MAX_HEIGHT, 230),
    "palette_w": (PALETTE_WIDTH, 600),
    "palette_h": (PALETTE_HEIGHT, 380),
    # Distance under which a press and its release count as a tap.
    "tap_slop": (4, 10),
    # List of the touch launches page (touch_page.py); the desktop has cards.
    "list_w": (0, 296),
    "row_h": (0, 68),
}


@dataclass(frozen=True)
class Density:
    """Size tokens and dp-to-pixel factor of one density.

    window_size and minimum_size are screen pixels, not dp: they describe the
    screen the density is meant for.
    """

    name: str
    factor: float
    touch: bool
    window_size: Tuple[int, int]
    minimum_size: Tuple[int, int]

    def px(self, dp: float) -> int:
        """Convert a length in dp into whole screen pixels, rounding half up."""
        return int(dp * self.factor + 0.5)

    def dp(self, token: str) -> float:
        """Return the value of a size token in dp."""
        desktop, touch = TOKENS[token]
        return touch if self.touch else desktop

    def size(self, token: str) -> int:
        """Return the value of a size token in screen pixels."""
        return self.px(self.dp(token))


DESKTOP = Density("desktop", 1.0, False, (DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT),
                  (MIN_WINDOW_WIDTH, MIN_WINDOW_HEIGHT))
TOUCH = Density("touch", 1.25, True, (1024, 600), (640, 400))
DENSITIES = {d.name: d for d in (DESKTOP, TOUCH)}

_current = DESKTOP


def by_name(name: str, scale: Optional[float] = None) -> Density:
    """Return the density called name, with its factor replaced by scale if given."""
    base = DENSITIES[name]
    return base if scale is None else replace(base, factor=scale)


def activate(density: Density) -> None:
    """Make density the one the widgets read; call it before building any widget."""
    global _current
    _current = density


def current() -> Density:
    """Return the active density."""
    return _current


def px(dp: float) -> int:
    """Convert a length in dp into screen pixels with the active density."""
    return _current.px(dp)


def size(token: str) -> int:
    """Return a size token in screen pixels with the active density."""
    return _current.size(token)


def margins(left: float, top: float, right: float, bottom: float) -> Tuple[int, int, int, int]:
    """Return layout margins in screen pixels, for setContentsMargins(*margins(...))."""
    return px(left), px(top), px(right), px(bottom)


def render_style(density: Density, base: str, touch_extra: str = "") -> str:
    """Fill the size tokens of a style sheet and convert its lengths to pixels.

    touch_extra is appended for the touch density only. At factor 1 the
    lengths stay as written.
    """
    text = base + ("\n" + touch_extra if density.touch and touch_extra else "")
    text = Template(text).substitute({key: f"{density.dp(key):g}px" for key in TOKENS})
    if density.factor != 1.0:
        text = _PX_RE.sub(lambda m: f"{density.px(float(m.group(1)))}px", text)
    return text


def load_style(density: Density) -> str:
    """Read style.qss (and style_touch.qss for touch) and render them for density."""
    base = STYLE_FILE.read_text(encoding="utf-8")
    extra = TOUCH_STYLE_FILE.read_text(encoding="utf-8") if density.touch else ""
    return render_style(density, base, extra)
