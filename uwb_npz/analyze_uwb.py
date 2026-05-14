from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from uwb_processing.run_session import main as run_session_main


def main() -> int:
    args = sys.argv[1:]
    if not args:
        args = ["--input", str(Path(__file__).resolve().parent)]
    elif not args[0].startswith("--"):
        args = ["--input", args[0], *args[1:]]
    return run_session_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
