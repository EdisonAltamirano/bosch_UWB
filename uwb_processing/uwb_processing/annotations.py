from __future__ import annotations

from pathlib import Path

import yaml

from .types import AnnotationWindow, SessionAnnotation


def _normalize_window(raw: dict) -> AnnotationWindow:
    expected_range = raw.get("expected_range_m")
    expected_range_tuple = None
    if expected_range is not None:
        expected_range_tuple = (float(expected_range[0]), float(expected_range[1]))
    return AnnotationWindow(
        label=str(raw["label"]),
        start_s=float(raw["start_s"]),
        end_s=float(raw["end_s"]),
        wall_present=bool(raw.get("wall_present", True)),
        expected_range_m=expected_range_tuple,
        notes=str(raw.get("notes", "")),
    )


def load_annotation_file(path: str | Path) -> SessionAnnotation:
    annotation_path = Path(path)
    payload = yaml.safe_load(annotation_path.read_text()) or {}
    windows = [_normalize_window(window) for window in payload.get("windows", [])]
    return SessionAnnotation(
        session_id=str(payload.get("session_id", annotation_path.stem)),
        source=str(payload["source"]),
        wall_description=str(payload.get("wall_description", "")),
        notes=str(payload.get("notes", "")),
        windows=windows,
    )


def load_manifest_file(path: str | Path) -> list[Path]:
    manifest_path = Path(path)
    payload = yaml.safe_load(manifest_path.read_text()) or {}
    sessions = payload.get("sessions", [])
    return [manifest_path.parent / Path(entry["annotation"]) for entry in sessions]
