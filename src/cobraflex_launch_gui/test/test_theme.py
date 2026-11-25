"""Unit tests of the theme: contrast of the palette and tokens of the style sheets."""

import re

from cobraflex_launch_gui.density import STYLE_FILE, TOKENS, TOUCH_STYLE_FILE
from cobraflex_launch_gui.theme import GRAPH_COLORS, LOG_LEVEL_COLORS, PALETTE, qss_tokens

#: Text colours of the light surfaces, and the surfaces themselves.
_TEXT_ON_LIGHT = ("c_tyre", "c_chassis", "c_muted", "c_signal", "c_amber")
_LIGHT = ("c_lid", "c_shell", "c_well")
#: Text colours of the black command and log panels.
_TEXT_ON_CODE = ("c_code_fg", "c_code_dim", "c_log_info", "c_log_warn", "c_log_error",
                 "c_log_debug", "c_log_ok")
#: Backgrounds of the buttons and bands with white text.
_UNDER_WHITE = ("c_go", "c_go_dark", "c_signal", "c_signal_dark", "c_tyre")


def _luminance(color: str) -> float:
    """Return the relative luminance of a #RRGGBB colour, as defined by WCAG 2."""
    channels = [int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    linear = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in channels]
    return 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2]


def _contrast(fg: str, bg: str) -> float:
    """Return the WCAG contrast ratio between two palette tokens."""
    high, low = sorted((_luminance(PALETTE[fg]), _luminance(PALETTE[bg])), reverse=True)
    return (high + 0.05) / (low + 0.05)


def test_text_colours_keep_four_and_a_half_to_one():
    """Every text colour reaches 4.5:1 on each surface it is drawn on."""
    pairs = [(fg, bg) for fg in _TEXT_ON_LIGHT for bg in _LIGHT]
    pairs += [(fg, "c_code_bg") for fg in _TEXT_ON_CODE]
    pairs += [("c_on_dark", bg) for bg in _UNDER_WHITE]
    low = [(fg, bg, round(_contrast(fg, bg), 2)) for fg, bg in pairs if _contrast(fg, bg) < 4.5]
    assert not low


def test_style_sheets_take_every_colour_from_the_theme():
    """The style sheets name only known tokens and write no colour literally."""
    known = set(qss_tokens()) | set(TOKENS)
    for path in (STYLE_FILE, TOUCH_STYLE_FILE):
        text = re.sub(r"/\*.*?\*/", "", path.read_text(encoding="utf-8"), flags=re.S)
        assert set(re.findall(r"\$\{(\w+)\}", text)) <= known, path.name
        bodies = " ".join(re.findall(r"\{([^{}]*)\}", re.sub(r"\$\{\w+\}", "", text)))
        assert not re.findall(r"#[0-9a-fA-F]{3,8}\b|rgba?\(", bodies), path.name


def test_graph_and_log_colours_come_from_the_palette():
    """The graph and the log panel draw with palette colours only."""
    palette = set(PALETTE.values())
    assert {c for triple in GRAPH_COLORS.values() for c in triple} <= palette
    assert set(LOG_LEVEL_COLORS.values()) <= palette
    assert set(GRAPH_COLORS) == {"node", "topic", "frame", "warn", "error"}
