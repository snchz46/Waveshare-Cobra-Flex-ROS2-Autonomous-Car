#!/usr/bin/env python3
"""Execute the numbered setup sections of an installation guide as written.

Every ```bash block of the sections "## 1." to "## 5." of the given Markdown
file is run in a fresh bash shell, in document order. The command
`git clone ... ~/ros2_ws` is skipped, because the workflow has already checked
out the repository and linked it to ~/ros2_ws. All other lines run unchanged.

A failing block does not stop the run: the remaining blocks are still executed
so that one run reports every problem. The exit status is non-zero if any
block failed.
"""
import re
import subprocess
import sys


def sections(text):
    """Yield (title, body) for each '## N.' section."""
    parts = re.split(r"^## ", text, flags=re.M)
    for part in parts[1:]:
        title, _, body = part.partition("\n")
        if re.match(r"\d+\.", title):
            yield title.strip(), body


def main(path):
    """Run the guide and report each block."""
    text = open(path, encoding="utf-8").read()
    results = []
    for title, body in sections(text):
        for i, block in enumerate(re.findall(r"```bash\n(.*?)```", body, flags=re.S), 1):
            lines = [ln for ln in block.splitlines() if not ln.strip().startswith("git clone")]
            script = "\n".join(lines)
            name = f"{title} / block {i}"
            print(f"\n::group::{name}\n{script}", flush=True)
            # stdin closed: a command that prompts fails instead of waiting.
            try:
                rc = subprocess.run(["bash", "-e", "-o", "pipefail", "-c", script],
                                    stdin=subprocess.DEVNULL, timeout=1800).returncode
            except subprocess.TimeoutExpired:
                rc = 124
                print(f"::error title=Guide step timed out::{name} exceeded 30 min")
            print("::endgroup::", flush=True)
            if rc != 0:
                print(f"::error title=Guide step failed::{path}: {name} exited with {rc}")
            results.append((name, rc))
    print("\nSummary of", path)
    for name, rc in results:
        print(f"  {'PASS' if rc == 0 else 'FAIL'}  {name}")
    return 1 if any(rc for _, rc in results) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
