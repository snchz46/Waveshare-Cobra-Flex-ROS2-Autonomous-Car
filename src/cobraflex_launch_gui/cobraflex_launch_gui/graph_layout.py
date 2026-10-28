"""Layered layout of directed graphs, for the ROS graph and the TF tree.

Graphviz dot computes the layout when it is installed (`dot -Tplain` returns
node centres and spline control points; the drawing stays native Qt). Without
dot, a built-in layered layout is used: cycle removal by depth-first search,
longest-path ranking, virtual nodes on long edges and barycentre ordering.
It produces more crossings than dot but needs no external program.

All coordinates are in points, with y pointing down.
"""

from dataclasses import dataclass
import math
import shlex
import shutil
import subprocess
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

_RANKSEP = 70.0
_NODESEP = 16.0
_SWEEPS = 8
_DOT_TIMEOUT_S = 10.0


@dataclass
class LayoutNode:
    """A node to place, with its drawn size in points."""

    name: str
    width: float
    height: float


@dataclass
class LayoutEdge:
    """A directed edge, optionally labelled."""

    src: str
    dst: str
    label: str = ""


@dataclass
class EdgeRoute:
    """Placed edge: control points from the source to the destination."""

    src: str
    dst: str
    points: List[Point]
    #: True: cubic B-spline control points (dot); False: a polyline.
    spline: bool
    label: str = ""
    label_pos: Optional[Point] = None


@dataclass
class Layout:
    """Result of a layout run."""

    centers: Dict[str, Point]
    routes: List[EdgeRoute]
    width: float
    height: float
    engine: str


def dot_available() -> bool:
    """Return whether Graphviz dot is on PATH."""
    return shutil.which("dot") is not None


def _unique_edges(nodes: Sequence[LayoutNode], edges: Sequence[LayoutEdge]) -> List[LayoutEdge]:
    """Merge parallel edges (joining their labels) and drop self-loops."""
    known = {n.name for n in nodes}
    merged: Dict[Tuple[str, str], List[str]] = {}
    for e in edges:
        if e.src == e.dst or e.src not in known or e.dst not in known:
            continue
        labels = merged.setdefault((e.src, e.dst), [])
        if e.label and e.label not in labels:
            labels.append(e.label)
    return [LayoutEdge(s, d, ", ".join(lbl)) for (s, d), lbl in merged.items()]


def layout_graph(nodes: Sequence[LayoutNode], edges: Sequence[LayoutEdge],
                 rankdir: str = "LR", use_dot: bool = True) -> Layout:
    """Place nodes and route edges; rankdir is "LR" or "TB"."""
    if not nodes:
        return Layout({}, [], 0.0, 0.0, "none")
    edges = _unique_edges(nodes, edges)
    if use_dot and dot_available():
        try:
            return _layout_dot(nodes, edges, rankdir)
        except (OSError, subprocess.SubprocessError, ValueError, KeyError, IndexError):
            pass  # fall back to the built-in layout
    return layout_layered(nodes, edges, rankdir)


# --- Graphviz ---------------------------------------------------------------


def _dot_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace('"', '\\"')


def build_dot(nodes: Sequence[LayoutNode], edges: Sequence[LayoutEdge],
              rankdir: str) -> Tuple[str, Dict[str, str]]:
    """Return the DOT source and the map from node id to DOT name.

    Nodes have fixed sizes and empty labels: the window draws them itself and
    only needs dot to reserve the space. DOT names are n0, n1, ... so that
    no quoting is needed when the plain output is parsed back.
    """
    names = {n.name: f"n{i}" for i, n in enumerate(nodes)}
    lines = [
        "digraph G {",
        f"  graph [rankdir={rankdir}, nodesep=0.25, ranksep=0.7, splines=true];",
        '  node [shape=box, fixedsize=true, label=""];',
        "  edge [arrowhead=none, fontsize=9];",
    ]
    for n in nodes:
        lines.append(f"  {names[n.name]} [width={n.width / 72.0:.3f}, "
                     f"height={n.height / 72.0:.3f}];")
    for e in edges:
        attr = f' [label="{_dot_escape(e.label)}"]' if e.label else ""
        lines.append(f"  {names[e.src]} -> {names[e.dst]}{attr};")
    lines.append("}")
    return "\n".join(lines) + "\n", names


def parse_plain(text: str, names: Dict[str, str], edges: Sequence[LayoutEdge]) -> Layout:
    """Parse `dot -Tplain` output into a Layout.

    Format (inches, origin at the bottom left):
        graph scale width height
        node name x y width height label style shape color fillcolor
        edge tail head n x1 y1 ... xn yn [label xl yl] style color
        stop
    """
    by_name = {v: k for k, v in names.items()}
    labels = {(e.src, e.dst): e.label for e in edges}
    gw = gh = 0.0
    centers: Dict[str, Point] = {}
    routes: List[EdgeRoute] = []

    def pt(x: str, y: str) -> Point:
        return float(x) * 72.0, (gh - float(y)) * 72.0

    for line in text.splitlines():
        tok = shlex.split(line)
        if not tok:
            continue
        if tok[0] == "graph":
            gw, gh = float(tok[2]), float(tok[3])
        elif tok[0] == "node":
            centers[by_name[tok[1]]] = pt(tok[2], tok[3])
        elif tok[0] == "edge":
            src, dst, n = by_name[tok[1]], by_name[tok[2]], int(tok[3])
            points = [pt(tok[4 + 2 * i], tok[5 + 2 * i]) for i in range(n)]
            rest = tok[4 + 2 * n:]
            label_pos = pt(rest[1], rest[2]) if len(rest) >= 5 else None
            routes.append(EdgeRoute(src, dst, points, True, labels.get((src, dst), ""), label_pos))
        elif tok[0] == "stop":
            break
    for r in routes:
        # Points run from tail to head; reorient defensively if dot reversed a
        # back edge without flipping its points.
        if r.points and _dist(r.points[0], centers[r.dst]) < _dist(r.points[0], centers[r.src]):
            r.points.reverse()
    return Layout(centers, routes, gw * 72.0, gh * 72.0, "dot")


def _layout_dot(nodes: Sequence[LayoutNode], edges: Sequence[LayoutEdge],
                rankdir: str) -> Layout:
    source, names = build_dot(nodes, edges, rankdir)
    result = subprocess.run(["dot", "-Tplain"], input=source, capture_output=True,
                            text=True, timeout=_DOT_TIMEOUT_S, check=True)
    return parse_plain(result.stdout, names, edges)


# --- Built-in layered layout ------------------------------------------------


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def clip_to_box(center: Point, width: float, height: float, toward: Point) -> Point:
    """Return where the segment from a box centre to a point leaves the box."""
    cx, cy = center
    dx, dy = toward[0] - cx, toward[1] - cy
    if dx == 0 and dy == 0:
        return center
    sx = (width / 2.0) / abs(dx) if dx else math.inf
    sy = (height / 2.0) / abs(dy) if dy else math.inf
    s = min(sx, sy, 1.0)
    return cx + dx * s, cy + dy * s


def _back_edges(ids: List[str], succ: Dict[str, List[str]]) -> set:
    """Return the edges that close a cycle in an iterative depth-first search."""
    state = dict.fromkeys(ids, 0)  # 0 new, 1 on stack, 2 done
    back = set()
    for root in ids:
        if state[root]:
            continue
        stack = [(root, iter(succ[root]))]
        state[root] = 1
        while stack:
            node, it = stack[-1]
            nxt = next(it, None)
            if nxt is None:
                state[node] = 2
                stack.pop()
            elif state[nxt] == 1:
                back.add((node, nxt))
            elif state[nxt] == 0:
                state[nxt] = 1
                stack.append((nxt, iter(succ[nxt])))
    return back


def _sweep(layers: List[List[str]], neighbours: Dict[str, List[str]],
           indices: range, step: int) -> None:
    """Reorder layers by the mean position of their neighbours in layer li + step."""
    for li in indices:
        ref = {n: p for p, n in enumerate(layers[li + step])}
        here = {n: p for p, n in enumerate(layers[li])}

        def barycentre(n, ref=ref, here=here):
            ps = [ref[m] for m in neighbours[n] if m in ref]
            return sum(ps) / len(ps) if ps else here[n]
        layers[li] = sorted(layers[li], key=barycentre)


def layout_layered(nodes: Sequence[LayoutNode], edges: Sequence[LayoutEdge],
                   rankdir: str = "LR") -> Layout:
    """Place a graph without Graphviz."""
    ids = [n.name for n in nodes]
    size = {n.name: (n.width, n.height) for n in nodes}
    succ: Dict[str, List[str]] = {i: [] for i in ids}
    for e in edges:
        succ[e.src].append(e.dst)

    back = _back_edges(ids, succ)
    # (from, to, edge, reversed): back edges are reversed to make a DAG and
    # routed in their own direction at the end.
    dag = [(e.dst, e.src, e, True) if (e.src, e.dst) in back else (e.src, e.dst, e, False)
           for e in edges]

    # Longest-path ranking in topological order.
    preds: Dict[str, List[str]] = {i: [] for i in ids}
    outs: Dict[str, List[str]] = {i: [] for i in ids}
    for u, v, _e, _r in dag:
        preds[v].append(u)
        outs[u].append(v)
    indeg = {i: len(preds[i]) for i in ids}
    queue = [i for i in ids if indeg[i] == 0]
    rank = dict.fromkeys(ids, 0)
    while queue:
        u = queue.pop(0)
        for v in outs[u]:
            rank[v] = max(rank[v], rank[u] + 1)
            indeg[v] -= 1
            if indeg[v] == 0:
                queue.append(v)

    # Virtual nodes on edges that span more than one rank.
    vsize = dict(size)
    chains: List[List[str]] = []
    adj_up: Dict[str, List[str]] = {i: [] for i in ids}
    adj_down: Dict[str, List[str]] = {i: [] for i in ids}
    for k, (u, v, _e, _r) in enumerate(dag):
        chain = [u]
        for r in range(rank[u] + 1, rank[v]):
            d = f"\0{k}:{r}"
            rank[d] = r
            vsize[d] = (2.0, 2.0)
            adj_up[d], adj_down[d] = [], []
            chain.append(d)
        chain.append(v)
        for a, b in zip(chain, chain[1:]):
            adj_down[a].append(b)
            adj_up[b].append(a)
        chains.append(chain)

    n_ranks = max(rank.values()) + 1
    layers: List[List[str]] = [[] for _ in range(n_ranks)]
    for i in ids:
        layers[rank[i]].append(i)
    for chain in chains:
        for d in chain[1:-1]:
            layers[rank[d]].append(d)

    # Barycentre ordering, alternating downward and upward sweeps.
    for _ in range(_SWEEPS):
        _sweep(layers, adj_up, range(1, n_ranks), -1)
        _sweep(layers, adj_down, range(n_ranks - 2, -1, -1), 1)

    # Coordinates: the rank axis is x for LR and y for TB.
    horizontal = rankdir == "LR"
    pos: Dict[str, Point] = {}
    thickness = [max((vsize[n][0] if horizontal else vsize[n][1]) for n in layer) if layer else 0
                 for layer in layers]
    extents = [sum((vsize[n][1] if horizontal else vsize[n][0]) for n in layer)
               + _NODESEP * max(len(layer) - 1, 0) for layer in layers]
    span = max(extents) if extents else 0.0
    margin = 10.0
    along = margin
    for li, layer in enumerate(layers):
        centre_r = along + thickness[li] / 2.0
        offset = margin + (span - extents[li]) / 2.0
        for n in layer:
            ext = vsize[n][1] if horizontal else vsize[n][0]
            centre_o = offset + ext / 2.0
            pos[n] = (centre_r, centre_o) if horizontal else (centre_o, centre_r)
            offset += ext + _NODESEP
        along += thickness[li] + _RANKSEP

    routes = []
    for (u, v, e, reversed_edge), chain in zip(dag, chains):
        pts = [pos[n] for n in chain]
        pts[0] = clip_to_box(pos[u], *size[u], pts[1])
        pts[-1] = clip_to_box(pos[v], *size[v], pts[-2])
        if reversed_edge:
            pts.reverse()
        mid = len(pts) // 2
        label_pos = ((pts[mid - 1][0] + pts[mid][0]) / 2.0,
                     (pts[mid - 1][1] + pts[mid][1]) / 2.0) if e.label else None
        routes.append(EdgeRoute(e.src, e.dst, pts, False, e.label, label_pos))

    centers = {i: pos[i] for i in ids}
    width = (along - _RANKSEP + margin) if horizontal else span + 2 * margin
    height = span + 2 * margin if horizontal else (along - _RANKSEP + margin)
    return Layout(centers, routes, width, height, "built-in")
