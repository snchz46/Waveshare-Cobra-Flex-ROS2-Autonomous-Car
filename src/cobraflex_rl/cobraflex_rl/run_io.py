"""
run_io — pure (ROS-free) helpers for writing reproducible run metadata:
content hashing and git-commit lookup. Shared by `train_ppo` (training run
registration, §7.2.8) and `eval_policy` (§7.5 evaluation runs) so both produce
the same reproducibility fields (see `experiments/README.md`).
"""

from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path
from typing import Optional, Union

PathLike = Union[str, Path]


def sha256_file(path: PathLike) -> Optional[str]:
    """Return the hex SHA-256 of a file, or None if it cannot be read."""
    try:
        digest = hashlib.sha256()
        with open(path, "rb") as handle:
            for chunk in iter(lambda: handle.read(65536), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def git_commit(cwd: PathLike) -> str:
    """Return the current HEAD commit hash, or "unknown" if git is unavailable
    or `cwd` is not inside a repository."""
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(cwd),
            capture_output=True,
            text=True,
            timeout=5,
        )
        return completed.stdout.strip() if completed.returncode == 0 else "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"
