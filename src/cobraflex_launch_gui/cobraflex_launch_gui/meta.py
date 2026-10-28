"""Curated launch metadata: descriptions, tags, starred entries and presets.

Two files are read: config/launches.yaml of this package, then the per-user
file <config dir>/launches.yaml, whose entries take precedence field by
field. Declared arguments always come from the launch file itself; metadata
only refines them (choices and help text), so a stale entry cannot add an
argument that the file does not declare.
"""

import os
from pathlib import Path
import sys
from typing import Dict, List, Optional

PACKAGE_NAME = "cobraflex_launch_gui"


def user_config_dir() -> Path:
    """Return the per-user configuration directory of the window."""
    base = os.environ.get("XDG_CONFIG_HOME") or str(Path.home() / ".config")
    return Path(base) / PACKAGE_NAME


def package_config_file() -> Optional[Path]:
    """Return config/launches.yaml from the install share, or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        path = Path(get_package_share_directory(PACKAGE_NAME)) / "config" / "launches.yaml"
        if path.is_file():
            return path
    except Exception:
        # Not installed (source tree, mock mode) or ament_index unavailable.
        pass
    path = Path(__file__).resolve().parents[1] / "config" / "launches.yaml"
    return path if path.is_file() else None


def _read_yaml(path: Optional[Path]) -> dict:
    if path is None or not path.is_file():
        return {}
    try:
        import yaml

        with path.open(encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        # A broken metadata file must not prevent the window from starting.
        print(f"[launch_gui] ignoring {path}: {exc}", file=sys.stderr)
        return {}


def _merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


class LaunchMeta:
    """Merged metadata, queried per launch file."""

    def __init__(self, data: Optional[dict] = None):
        """Wrap already-merged metadata; see load_meta() for the files."""
        data = data or {}
        self._packages: Dict[str, dict] = data.get("packages") or {}
        self._launches: Dict[str, dict] = data.get("launches") or {}

    def for_launch(self, pkg: str, name: str) -> dict:
        """Return the metadata of one launch file, package defaults included."""
        return _merge(self._packages.get(pkg) or {}, self._launches.get(f"{pkg}/{name}") or {})

    @staticmethod
    def refine_args(declared: List[dict], refinements: Optional[dict]) -> List[dict]:
        """Apply curated options and help texts to the declared arguments."""
        if not refinements:
            return declared
        out = []
        for a in declared:
            extra = refinements.get(a["key"]) or {}
            b = dict(a)
            if extra.get("options"):
                b["options"] = [str(o) for o in extra["options"]]
                b["kind"] = "enum"
            if extra.get("help") and not b.get("help"):
                b["help"] = str(extra["help"])
            out.append(b)
        return out


def load_meta() -> LaunchMeta:
    """Load and merge the package and per-user metadata files."""
    package = _read_yaml(package_config_file())
    user = _read_yaml(user_config_dir() / "launches.yaml")
    return LaunchMeta(_merge(package, user))
