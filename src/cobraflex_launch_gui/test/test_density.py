"""Unit tests of the size densities and of the rendering of the style sheets."""

import re

from cobraflex_launch_gui.density import by_name, DESKTOP, load_style, render_style, TOUCH


def _rule(qss: str, selector: str) -> str:
    """Return the body of the first rule whose selector is exactly selector."""
    m = re.search(r"(?:^|\})\s*" + re.escape(selector) + r"\s*\{([^}]*)\}", qss)
    assert m, f"no rule {selector!r}"
    return m.group(1)


def test_px_rounds_half_up():
    """Lengths in dp become whole pixels, rounded half up."""
    assert TOUCH.px(50) == 63
    assert TOUCH.px(1) == 1
    assert DESKTOP.px(13) == 13


def test_desktop_keeps_the_sizes_of_the_style_sheet():
    """At the desktop density every token takes the size the sheet had before."""
    qss = load_style(DESKTOP)
    assert "$" not in qss
    assert "font-size: 13px" in _rule(qss, "*")
    assert "min-height: 26px" in _rule(qss, "QPushButton#btnRun")
    assert "min-width: 24px" in _rule(qss, "QPushButton#expandBtn")
    assert "QComboBox QAbstractItemView::item" not in qss


def test_touch_enlarges_controls_and_appends_its_rules():
    """The touch density scales every length by 1.25 and adds style_touch.qss."""
    qss = load_style(TOUCH)
    assert "$" not in qss
    assert "font-size: 20px" in _rule(qss, "*")
    btn_run = _rule(qss, "QPushButton#btnRun")
    assert "min-height: 63px" in btn_run
    assert "padding: 0 14px" in btn_run
    assert "min-height: 60px" in _rule(qss, "QComboBox QAbstractItemView::item")


def test_only_pixel_lengths_scale():
    """Object names and colours keep their digits; lengths with px scale."""
    qss = render_style(TOUCH, "QLabel#a2px { border: 1px solid #12abef; margin: 4px; }")
    assert qss == "QLabel#a2px { border: 1px solid #12abef; margin: 5px; }"


def test_scale_replaces_the_factor_only():
    """A scale override keeps the tokens of the density."""
    touch_at_one = by_name("touch", 1.0)
    assert touch_at_one.touch
    assert touch_at_one.size("ctl_l") == 50
    assert by_name("desktop") is DESKTOP
