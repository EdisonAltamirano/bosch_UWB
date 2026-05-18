from __future__ import annotations

from pathlib import Path

_inner_package = Path(__file__).resolve().parent / "uwb_processing"
if str(_inner_package) not in __path__:
    __path__.append(str(_inner_package))
