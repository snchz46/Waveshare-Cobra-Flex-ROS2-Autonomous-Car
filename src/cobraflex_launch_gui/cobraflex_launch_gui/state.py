"""Per-user state that outlives the window: starred launches and presets.

Stored as JSON in <config dir>/state.json, keyed by "<package>/<name>". The
file is written on every change and replaced atomically, so that a crash
cannot leave it truncated.
"""

import json
import os
from pathlib import Path
import sys
from typing import Dict, List, Optional

from cobraflex_launch_gui.meta import user_config_dir


class UserState:
    """Starred flags and user presets of the launch files."""

    def __init__(self, path: Optional[Path] = None):
        """Load the state file; a missing or unreadable file starts empty."""
        self.path = path or user_config_dir() / "state.json"
        self._starred: Dict[str, bool] = {}
        self._presets: Dict[str, List[dict]] = {}
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            self._starred = {str(k): bool(v) for k, v in data.get("starred", {}).items()}
            self._presets = {str(k): list(v) for k, v in data.get("presets", {}).items()}
        except FileNotFoundError:
            pass
        except (OSError, ValueError, AttributeError) as exc:
            print(f"[launch_gui] ignoring {self.path}: {exc}", file=sys.stderr)

    def starred(self, key: str) -> Optional[bool]:
        """Return the starred flag set by the user, or None if never set."""
        return self._starred.get(key)

    def set_starred(self, key: str, value: bool) -> None:
        """Record a starred flag."""
        self._starred[key] = value
        self._save()

    def presets(self, key: str) -> List[dict]:
        """Return the user presets of a launch file."""
        return [dict(p) for p in self._presets.get(key, [])]

    def set_presets(self, key: str, presets: List[dict]) -> None:
        """Replace the user presets of a launch file."""
        cleaned = [{"name": p["name"], "args": dict(p["args"])} for p in presets]
        if cleaned:
            self._presets[key] = cleaned
        else:
            self._presets.pop(key, None)
        self._save()

    def _save(self) -> None:
        data = {"starred": self._starred, "presets": self._presets}
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
            os.replace(tmp, self.path)
        except OSError as exc:
            print(f"[launch_gui] cannot write {self.path}: {exc}", file=sys.stderr)
