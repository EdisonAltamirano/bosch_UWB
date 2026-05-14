from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from .annotations import load_annotation_file, load_manifest_file
from .run_session import config_from_args, process_session


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run offline UWB batch evaluation from a manifest.")
    parser.add_argument("--manifest", required=True, help="Manifest YAML listing annotation files.")
    parser.add_argument("--output-dir", help="Directory for aggregated outputs.")
    parser.add_argument("--topic", default="/uwb/frame_raw")
    parser.add_argument("--range-resolution-m", type=float, default=0.15)
    parser.add_argument("--default-range-min-m", type=float, default=0.3)
    parser.add_argument("--default-range-max-m", type=float, default=6.0)
    parser.add_argument("--wall-clip-m", type=float, default=0.3)
    parser.add_argument("--motion-threshold", type=float, default=3.0)
    parser.add_argument("--background-window-s", type=float, default=1.5)
    parser.add_argument("--doppler-window-s", type=float, default=4.0)
    parser.add_argument("--microdoppler-window-s", type=float, default=1.5)
    parser.add_argument("--stft-overlap", type=float, default=0.75)
    parser.add_argument("--top-k-taps", type=int, default=6)
    return parser


def _aggregate_metrics(session_summaries: list[dict[str, Any]]) -> dict[str, Any]:
    tp = fp = tn = fn = 0
    counted = 0
    for session in session_summaries:
        for detection in session["detections"]:
            expected = detection["expected_present"]
            predicted = detection["predicted_present"]
            if expected is None:
                continue
            counted += 1
            if expected and predicted:
                tp += 1
            elif expected and not predicted:
                fn += 1
            elif (not expected) and predicted:
                fp += 1
            else:
                tn += 1
    accuracy = ((tp + tn) / counted) if counted else None
    precision = (tp / (tp + fp)) if (tp + fp) else None
    recall = (tp / (tp + fn)) if (tp + fn) else None
    return {
        "counted_windows": counted,
        "true_positive": tp,
        "false_positive": fp,
        "true_negative": tn,
        "false_negative": fn,
        "accuracy": accuracy,
        "precision": precision,
        "recall": recall,
    }


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = config_from_args(args)
    manifest_path = Path(args.manifest)
    output_dir = Path(args.output_dir) if args.output_dir else manifest_path.parent / "batch_analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    repo_root = manifest_path.resolve().parents[2]

    summaries = []
    for annotation_path in load_manifest_file(manifest_path):
        annotation = load_annotation_file(annotation_path)
        session_summary, session_output = process_session(
            input_path=repo_root / annotation.source,
            annotation_path=annotation_path,
            output_dir=output_dir / annotation.session_id,
            config=config,
            topic=args.topic,
        )
        session_summary["output_dir"] = str(session_output)
        summaries.append(session_summary)

    batch_summary = {
        "manifest": str(manifest_path),
        "session_count": len(summaries),
        "sessions": summaries,
        "metrics": _aggregate_metrics(summaries),
    }
    (output_dir / "batch_summary.json").write_text(json.dumps(batch_summary, indent=2))
    print(json.dumps(batch_summary["metrics"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
