#!/usr/bin/env python3
"""Execute the setup steps of one part of an installation guide as written.

Every ```bash block of the "### " steps of the section "## <part>..." of the
given Markdown file is run in a fresh bash shell, in document order. The
default part is "Lab PC", the procedure that the CI container reproduces; the
robot part requires a Jetson and is not executed. The command
`git clone ... ~/ros2_ws` is skipped, because the workflow has already checked
out the repository and linked it to ~/ros2_ws. All other lines run unchanged.

A failing block does not stop the run: the remaining blocks are still executed
so that one run reports every problem. The exit status is non-zero if any
block failed, or if the part contains no bash block (for example after a
heading was renamed), so that a restructured guide cannot pass unexecuted.
"""
import re
import subprocess
import sys


def steps(text, part):
    """Yield (title, body) for each '### ' step of the '## <part>' section."""
    for section in re.split(r"^## ", text, flags=re.M)[1:]:
        title, _, body = section.partition("\n")
        if title.startswith(part):
            for step in re.split(r"^### ", body, flags=re.M)[1:]:
                step_title, _, step_body = step.partition("\n")
                yield step_title.strip(), step_body


def main(path, part="Lab PC"):
    """Run the guide and report each block."""
    text = open(path, encoding="utf-8").read()
    results = []
    for title, body in steps(text, part):
        for i, block in enumerate(re.findall(r"```bash\n(.*?)```", body, flags=re.S), 1):
            lines = [ln for ln in block.splitlines() if not ln.strip().startswith("git clone")]
            script = "\n".join(lines)
            name = f"{part} / {title} / block {i}"
            print(f"\n::group::{name}\n{script}", flush=True)
            # Interactive shell (-i): each block starts like a new terminal and
            # reads ~/.bashrc, as a student's terminal does. stdin closed: a
            # command that prompts fails instead of waiting.
            try:
                rc = subprocess.run(["bash", "-i", "-e", "-o", "pipefail", "-c", script],
                                    stdin=subprocess.DEVNULL, timeout=1800).returncode
            except subprocess.TimeoutExpired:
                rc = 124
                print(f"::error title=Guide step timed out::{name} exceeded 30 min")
            print("::endgroup::", flush=True)
            if rc != 0:
                print(f"::error title=Guide step failed::{path}: {name} exited with {rc}")
            results.append((name, rc))
    if not results:
        print(f"::error title=Guide part missing::{path}: no bash block under '## {part}'")
        return 1
    print("\nSummary of", path)
    for name, rc in results:
        print(f"  {'PASS' if rc == 0 else 'FAIL'}  {name}")
    return 1 if any(rc for _, rc in results) else 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:3]))
