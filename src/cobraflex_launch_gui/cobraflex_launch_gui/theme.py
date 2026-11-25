"""Colours and type of the window: the "Carrocería" theme, after the materials of the robot.

The palette comes from the V4.1 body of the CobraFlex: the grey PLA lid is
the ground, the white PLA shell the panels, the black of the tyres and
sensors the text, the selection and the dark bands, and the grey of the
aluminium chassis the secondary text. Signal red is the one accent (active
tab, Stop, failures) and green marks Run. Every other colour marks an
abnormal state and comes with a word, so that the screen reads without
colour vision. Text colours keep a contrast of at least 4.5:1 on the lid, the
shell and the black panels (test/test_theme.py).

style.qss reads the palette as ${c_*} tokens and the fonts as ${font_ui} and
${font_mono}; the graph, the log panel and the inline HTML of the widgets read
the same values. Free of Qt imports.
"""

from typing import Dict, Tuple

#: Colour tokens of the style sheets and of the drawing done in Python.
PALETTE: Dict[str, str] = {
    # Neutrals of the body.
    "c_lid": "#E3E1DA",
    "c_shell": "#FBFAF7",
    "c_shell_alt": "#F3F1EC",
    "c_well": "#ECEAE4",
    "c_tyre": "#1C1D1F",
    "c_chassis": "#53565A",
    "c_muted": "#606367",
    "c_faint": "#A7A9AB",
    "c_line": "#CCC9C1",
    "c_line_strong": "#ABA8A0",
    "c_on_dark": "#FFFFFF",
    # Signal red: accent, Stop, failure.
    "c_signal": "#C4161C",
    "c_signal_dark": "#9E1015",
    "c_signal_bg": "#F8DEDB",
    # Run.
    "c_go": "#2E7D32",
    "c_go_dark": "#1B5E20",
    # Stopping, warnings.
    "c_amber": "#8A5600",
    "c_amber_bg": "#F4E7CC",
    # The black panels of commands and logs, and the colours drawn on them.
    "c_code_bg": "#1C1D1F",
    "c_code_fg": "#E9E8E3",
    "c_code_dim": "#8E9093",
    "c_log_info": "#B9B8B2",
    "c_log_warn": "#F0B44C",
    "c_log_error": "#FF7A70",
    "c_log_debug": "#9DB8D9",
    "c_log_ok": "#9FD49F",
    # Graph edges.
    "c_edge": "#8A8780",
}

#: Font stacks. Atkinson Hyperlegible (Braille Institute, OFL) is meant for
#: low-vision reading and tells I, l and 1 apart; Ubuntu ships with the
#: desktop of the lab PCs and of the Jetson and takes over without it.
FONT_UI = '"Atkinson Hyperlegible Next", "Atkinson Hyperlegible", "Ubuntu", "Segoe UI", sans-serif'
FONT_MONO = ('"Atkinson Hyperlegible Mono", "Ubuntu Mono", "JetBrains Mono", "Cascadia Code", '
             'Consolas, monospace')

#: Graph boxes, (fill, border, text) per item kind and state: nodes black like
#: the hardware, topics and frames white like the shell.
GRAPH_COLORS: Dict[str, Tuple[str, str, str]] = {
    "node": (PALETTE["c_tyre"], PALETTE["c_tyre"], PALETTE["c_shell"]),
    "topic": (PALETTE["c_shell"], PALETTE["c_line_strong"], PALETTE["c_tyre"]),
    "frame": (PALETTE["c_shell"], PALETTE["c_tyre"], PALETTE["c_tyre"]),
    "warn": (PALETTE["c_amber_bg"], PALETTE["c_amber"], PALETTE["c_amber"]),
    "error": (PALETTE["c_signal_bg"], PALETTE["c_signal"], PALETTE["c_signal"]),
}
GRAPH_BACKGROUND = PALETTE["c_lid"]
GRAPH_EDGE_COLOR = PALETTE["c_edge"]
#: Selection is black, as everywhere else; red stays for errors.
GRAPH_EDGE_HIGHLIGHT = PALETTE["c_tyre"]
GRAPH_LABEL_COLOR = PALETTE["c_chassis"]

#: Level colours of the log lines, drawn on the black log panel.
LOG_LEVEL_COLORS: Dict[str, str] = {
    "INFO": PALETTE["c_log_info"],
    "WARN": PALETTE["c_log_warn"],
    "ERROR": PALETTE["c_log_error"],
    "DEBUG": PALETTE["c_log_debug"],
}


def color(name: str) -> str:
    """Return a colour of the palette, by token name without the c_ prefix."""
    return PALETTE["c_" + name]


def qss_tokens() -> Dict[str, str]:
    """Return the colour and font tokens of the style sheets."""
    return {**PALETTE, "font_ui": FONT_UI, "font_mono": FONT_MONO}
