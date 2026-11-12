"""Build an offline index for saved EXP-002 wall-calibration runs.

GPS pose is the source of the offline pose label. This corrects historical
trial labels without editing the logger's original frames.csv or summary.txt.
Non-wall headings and unrecognized poses are excluded.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path


POSES = {
    "P1": -9.0,
    "P2": -8.8,
    "P3": -8.3,
    "extra_z_minus_8_5": -8.5,
}
TRIAL_ID = re.compile(r"calib01_tilt30_p(?P<pose>[123])(?:_r(?P<repeat>[0-9]{2}))?")
INDEX_FIELDS = [
    "run_id",
    "logged_trial_id",
    "offline_pose_label",
    "repetition",
    "gps_x_m",
    "gps_z_m",
    "heading_rad",
    "notes",
]


def _number(value: str | None) -> float | None:
    if value in (None, ""):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _pose_label(gps_z_m: float, tolerance_m: float) -> str | None:
    matches = [label for label, expected in POSES.items()
               if abs(gps_z_m - expected) <= tolerance_m]
    if len(matches) > 1:
        raise ValueError(f"GPS Z={gps_z_m} matches multiple calibration poses")
    return matches[0] if matches else None


def _assign_repetitions(rows: list[dict]) -> None:
    """Disambiguate reused repeat labels offline, per GPS-derived pose."""
    grouped: dict[str, list[dict]] = {}
    for row in rows:
        if row["repetition"] != "legacy_unlabeled_repeat":
            grouped.setdefault(row["offline_pose_label"], []).append(row)

    for pose_label, pose_rows in grouped.items():
        repetitions = [row["repetition"] for row in pose_rows]
        if len(set(repetitions)) == len(repetitions):
            continue
        for ordinal, row in enumerate(sorted(pose_rows, key=lambda item: item["run_id"]), start=1):
            logged_repetition = row["repetition"]
            assigned = f"{ordinal:02d}"
            if assigned != logged_repetition:
                row["notes"] += (
                    f"; repeated logged ID r{logged_repetition} "
                    f"renumbered offline r{assigned} by run order"
                )
            row["repetition"] = assigned


def build_index(results_root: Path, *, z_tolerance_m: float = 0.03) -> tuple[list[dict], int]:
    if not math.isfinite(z_tolerance_m) or z_tolerance_m <= 0:
        raise ValueError("Z tolerance must be a positive finite number")
    rows: list[dict] = []
    skipped = 0
    for run_dir in sorted(results_root.glob("run_*")):
        frames_path = run_dir / "frames.csv"
        if not frames_path.is_file():
            continue
        with frames_path.open(newline="", encoding="utf-8") as source:
            frames = list(csv.DictReader(source))
        saved = [frame for frame in frames
                 if frame.get("image_saved") == "1" and frame.get("image_file")]
        if not saved:
            skipped += 1
            continue
        frame = saved[0]
        trial_id = frame.get("trial_id", "")
        trial_match = TRIAL_ID.fullmatch(trial_id)
        gps_x = _number(frame.get("gps_x_m"))
        gps_z = _number(frame.get("gps_z_m"))
        heading = _number(frame.get("heading_rad"))
        if (trial_match is None or gps_x is None or gps_z is None or heading is None
                or abs(heading) > 0.02):
            skipped += 1
            continue

        pose_label = _pose_label(gps_z, z_tolerance_m)
        if pose_label is None:
            skipped += 1
            continue
        logged_pose = f"P{trial_match.group('pose')}"
        notes = "GPS pose agrees with trial label"
        if logged_pose != pose_label:
            notes = f"GPS-derived {pose_label}; logged ID says {logged_pose}"
        elif trial_match.group("repeat"):
            notes = f"Repeat r{trial_match.group('repeat')}"
        rows.append({
            "run_id": run_dir.name,
            "logged_trial_id": trial_id,
            "offline_pose_label": pose_label,
            "repetition": trial_match.group("repeat") or "legacy_unlabeled_repeat",
            "gps_x_m": gps_x,
            "gps_z_m": gps_z,
            "heading_rad": heading,
            "notes": notes,
        })
    _assign_repetitions(rows)
    return rows, skipped


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Index EXP-002 wall runs by GPS pose without changing raw logs"
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("experiments/exp_002_visual_ranging/results"),
    )
    parser.add_argument("--z-tolerance-m", type=float, default=0.03)
    args = parser.parse_args()
    try:
        rows, skipped = build_index(args.results_root, z_tolerance_m=args.z_tolerance_m)
        output_path = args.results_root / "run_index.csv"
        with output_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=INDEX_FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Indexed {len(rows)} wall runs; skipped {skipped}; wrote {output_path}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
