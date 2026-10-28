"""Launch-file discovery and static extraction of declared arguments.

Two sources:

- discover_installed(): the ament index of the sourced workspaces. Only
  installed launch files are listed, which are exactly those that
  `ros2 launch <package> <file>` can run, and an overlay such as ~/lab_ws
  appears as soon as it is sourced. Packages under /opt/ros are skipped
  unless requested.
- discover_source(): <workspace>/src/**/launch/ in a source tree, for the
  mock mode on a machine without ROS.

Arguments are read without executing the file: the AST of a Python launch
file, the top-level <arg> elements of an XML file, the arg entries of a YAML
file. A default computed at launch time (a substitution, a variable) cannot
be known statically; such an argument is marked with a hint instead.
"""

import ast
from dataclasses import dataclass
import os
from pathlib import Path
import re
from typing import Iterable, List, Optional, Tuple
import xml.etree.ElementTree as ET

from cobraflex_launch_gui.constants import UNTAGGED
from cobraflex_launch_gui.meta import LaunchMeta
from cobraflex_launch_gui.model import instantiate_args, Launch
from cobraflex_launch_gui.state import UserState

#: (suffix, format). Both naming conventions are accepted by ros2 launch.
_SUFFIXES = (
    (".launch.py", "py"),
    (".launch.xml", "xml"),
    (".launch.yaml", "yaml"),
    ("_launch.py", "py"),
    ("_launch.xml", "xml"),
    ("_launch.yaml", "yaml"),
)
_SYSTEM_PREFIX = "/opt/ros/"
_SKIP_DIRS = {"build", "install", "log", "__pycache__"}
_HINT_MAX = 90
_DESC_MAX = 160


@dataclass
class LaunchFile:
    """A launch file found on disk."""

    pkg: str
    name: str
    fmt: str
    path: Path
    workspace: str = ""


def split_launch_name(filename: str) -> Optional[Tuple[str, str]]:
    """Return (name, format) of a launch file name, or None."""
    for suffix, fmt in _SUFFIXES:
        if filename.endswith(suffix) and len(filename) > len(suffix):
            return filename[: -len(suffix)], fmt
    return None


def _files_in(launch_dir: Path, pkg: str, workspace: str) -> Iterable[LaunchFile]:
    if not launch_dir.is_dir():
        return
    for f in sorted(launch_dir.iterdir()):
        parsed = split_launch_name(f.name)
        if parsed and f.is_file():
            yield LaunchFile(pkg, parsed[0], parsed[1], f, workspace)


def workspace_label(prefix: Path, pkg: str) -> str:
    """Name the workspace of an install prefix.

    colcon installs into <ws>/install/<pkg> by default and into <ws>/install
    with --merge-install; the label is <ws> in both cases.
    """
    if prefix.name == pkg and prefix.parent.name == "install":
        return prefix.parent.parent.name
    if prefix.name == "install":
        return prefix.parent.name
    return str(prefix)


def discover_installed(include_system: bool = False) -> List[LaunchFile]:
    """List the launch files of the packages in the ament index."""
    from ament_index_python.packages import get_packages_with_prefixes

    out: List[LaunchFile] = []
    for pkg, prefix in sorted(get_packages_with_prefixes().items()):
        if not include_system and str(prefix).startswith(_SYSTEM_PREFIX):
            continue
        prefix_path = Path(prefix)
        launch_dir = prefix_path / "share" / pkg / "launch"
        out.extend(_files_in(launch_dir, pkg, workspace_label(prefix_path, pkg)))
    return out


def discover_source(workspace: Path) -> List[LaunchFile]:
    """List the launch files of the packages under <workspace>/src."""
    src = workspace / "src"
    out: List[LaunchFile] = []
    if not src.is_dir():
        return out
    for root, dirs, files in os.walk(src):
        dirs[:] = sorted(d for d in dirs if d not in _SKIP_DIRS and not d.startswith("."))
        if "package.xml" in files:
            pkg_dir = Path(root)
            out.extend(_files_in(pkg_dir / "launch", _package_name(pkg_dir), workspace.name))
            dirs[:] = []  # packages do not nest
    return out


def _package_name(pkg_dir: Path) -> str:
    try:
        root = ET.parse(pkg_dir / "package.xml").getroot()
        name = (root.findtext("name") or "").strip()
        if name:
            return name
    except (OSError, ET.ParseError):
        pass
    return pkg_dir.name


# --- Arguments --------------------------------------------------------------


def infer_kind(default: str) -> str:
    """Infer the editor kind of an argument from its default value."""
    d = default.strip().lower()
    if d in ("true", "false"):
        return "bool"
    try:
        float(d)
        return "number"
    except ValueError:
        return "string"


def _hint(text: str) -> str:
    text = " ".join(text.split())
    return text if len(text) <= _HINT_MAX else text[: _HINT_MAX - 1] + "…"


def _const_str(node: ast.AST) -> Optional[str]:
    if isinstance(node, ast.Constant) and isinstance(node.value, (str, int, float, bool)):
        value = node.value
        return str(value).lower() if isinstance(value, bool) else str(value)
    return None


def _is_declare(call: ast.Call) -> bool:
    func = call.func
    name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", None)
    return name == "DeclareLaunchArgument"


def parse_python_args(text: str) -> List[dict]:
    """Extract the DeclareLaunchArgument calls of a Python launch file."""
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return []
    args: List[dict] = []
    seen = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not _is_declare(node):
            continue
        key = _const_str(node.args[0]) if node.args else None
        kw = {k.arg: k.value for k in node.keywords if k.arg}
        if key is None and "name" in kw:
            key = _const_str(kw["name"])
        if not key or key in seen:
            continue
        seen.add(key)
        spec = {"key": key, "default": ""}
        if "default_value" not in kw:
            spec["required"] = True
        else:
            default = _const_str(kw["default_value"])
            if default is None:
                spec["hint"] = _hint(ast.unparse(kw["default_value"]))
            else:
                spec["default"] = default
        if "description" in kw:
            desc = _const_str(kw["description"])
            if desc:
                spec["help"] = " ".join(desc.split())
        choices = kw.get("choices")
        if isinstance(choices, (ast.List, ast.Tuple)):
            options = [s for s in (_const_str(e) for e in choices.elts) if s is not None]
            if options:
                spec["options"] = options
        spec["kind"] = "enum" if spec.get("options") else infer_kind(spec["default"])
        args.append(spec)
    return args


def _arg_from_attrs(name: str, default: Optional[str], desc: Optional[str]) -> dict:
    spec = {"key": name, "default": ""}
    if default is None:
        spec["required"] = True
    elif "$(" in default:
        spec["hint"] = _hint(default)
    else:
        spec["default"] = default
    if desc:
        spec["help"] = " ".join(desc.split())
    spec["kind"] = infer_kind(spec["default"])
    return spec


def parse_xml_args(text: str) -> List[dict]:
    """Extract the top-level <arg> declarations of an XML launch file.

    <arg> elements inside <include> pass values to the included file (they
    carry value=, not default=) and are not declarations of this file.
    """
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        return []
    args = []
    for el in root:
        if el.tag != "arg" or "value" in el.attrib or not el.get("name"):
            continue
        args.append(_arg_from_attrs(el.get("name"), el.get("default"), el.get("description")))
    return args


def parse_yaml_args(text: str) -> List[dict]:
    """Extract the arg entries of a YAML launch file."""
    try:
        import yaml

        data = yaml.safe_load(text) or {}
    except Exception:
        return []
    items = data.get("launch", []) if isinstance(data, dict) else []
    args = []
    for item in items:
        arg = item.get("arg") if isinstance(item, dict) else None
        if isinstance(arg, dict) and arg.get("name") and "value" not in arg:
            default = arg.get("default")
            args.append(_arg_from_attrs(str(arg["name"]),
                                        None if default is None else str(default),
                                        arg.get("description")))
    return args


def parse_launch_args(path: Path, fmt: str) -> List[dict]:
    """Extract the declared arguments of a launch file of any format."""
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    parser = {"py": parse_python_args, "xml": parse_xml_args, "yaml": parse_yaml_args}[fmt]
    return parser(text)


# --- Descriptions -----------------------------------------------------------


def _first_sentence(text: str, name: str) -> str:
    paragraph = " ".join(text.strip().split("\n\n", 1)[0].split())
    # Docstrings such as "foo.launch.py - Demo launch." repeat the file name.
    paragraph = re.sub(
        rf"^{re.escape(name)}(\.launch\.(py|xml|yaml)|_launch\.(py|xml|yaml))?\s*[-:–—]+\s*",
        "", paragraph)
    m = re.search(r"\.(\s|$)", paragraph)
    sentence = paragraph[: m.start()] if m else paragraph
    if len(sentence) > _DESC_MAX:
        sentence = sentence[: _DESC_MAX - 1].rstrip() + "…"
    return sentence[:1].upper() + sentence[1:]


def extract_description(path: Path, fmt: str, name: str) -> str:
    """Return the first sentence of the file's docstring or leading comment."""
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    if fmt == "py":
        try:
            doc = ast.get_docstring(ast.parse(text))
        except SyntaxError:
            doc = None
        return _first_sentence(doc, name) if doc else ""
    if fmt == "xml":
        comments = re.findall(r"<!--(.*?)-->", text, re.DOTALL)
    else:
        comments = re.findall(r"^\s*#(.*)$", text, re.MULTILINE)
    for comment in comments:
        cleaned = comment.strip().strip("=-*# ").strip()
        # Section banners ("ARGS", "Arguments") are not descriptions.
        if len(cleaned.split()) >= 3:
            return _first_sentence(cleaned, name)
    return ""


# --- Assembly ---------------------------------------------------------------


def build_launches(files: List[LaunchFile], meta: LaunchMeta,
                   state: Optional[UserState] = None) -> List[Launch]:
    """Combine discovered files, metadata and user state into Launch objects."""
    out = []
    for f in files:
        m = meta.for_launch(f.pkg, f.name)
        launch = Launch(
            pkg=f.pkg, name=f.name, file_path=f.path, file_format=f.fmt,
            workspace=f.workspace,
            desc=m.get("description") or extract_description(f.path, f.fmt, f.name)
            or "(no description)",
            tags=[str(t) for t in m.get("tags") or []] or [UNTAGGED],
            starred=bool(m.get("starred", False)),
        )
        declared = LaunchMeta.refine_args(parse_launch_args(f.path, f.fmt), m.get("args"))
        launch.args = instantiate_args(declared)
        keys = {a["key"] for a in declared}
        launch.presets = [
            {"name": str(p["name"]), "user": False,
             "args": {k: str(v) for k, v in (p.get("args") or {}).items() if k in keys}}
            for p in m.get("presets") or [] if p.get("name")
        ]
        if state is not None:
            starred = state.starred(launch.key)
            if starred is not None:
                launch.starred = starred
            for p in state.presets(launch.key):
                launch.presets.append({"name": p["name"], "args": p["args"], "user": True})
        out.append(launch)
    out.sort(key=lambda la: (la.pkg, la.name))
    return out
