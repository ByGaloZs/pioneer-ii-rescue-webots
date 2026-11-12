"""Compare EXP-002 geometric candidates against the scenario-01 wall plane.

This is an offline geometric consistency check, not independent metrology.
It uses the wall plane recorded in the WBT and the V3 sector bearing/range.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


WALL_POSES = {"P1", "P2", "P3", "extra_z_minus_8_5"}

SECTOR_FIELDS = [
    "run_id",
    "offline_pose_label",
    "repetition",
    "gps_z_m",
    "logged_trial_id",
    "sector_id",
    "camera_angle_deg",
    "bearing_robot_deg",
    "candidate_distance_m",
    "reference_wall_plane_distance_m",
    "signed_error_m",
    "absolute_error_m",
    "near_field_occluded",
    "status",
    "unknown_reason",
]

SUMMARY_FIELDS = [
    "run_id",
    "offline_pose_label",
    "repetition",
    "gps_z_m",
    "sectors_total",
    "candidate_count",
    "unknown_count",
    "candidate_mae_m",
    "candidate_mean_signed_error_m",
    "near_field_occluded_candidate_count",
    "interpretation",
]


def wall_plane_distance_m(
    *, camera_origin_z_m: float, wall_face_z_m: float, bearing_robot_deg: float
) -> float:
    """Horizontal ray distance from camera origin to a plane z=wall_face_z_m."""
    values = (camera_origin_z_m, wall_face_z_m, bearing_robot_deg)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Wall reference geometry must be finite")
    normal_depth = camera_origin_z_m - wall_face_z_m
    forward_component = math.cos(math.radians(bearing_robot_deg))
    if normal_depth <= 0 or forward_component <= 1e-6:
        raise ValueError("Wall plane is not in front of the camera ray")
    return normal_depth / forward_component


def _float_or_none(value: str | None) -> float | None:
    if value in (None, ""):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def compare_wall_runs(
    run_index_path: Path,
    analysis_dir: Path,
    *,
    wall_face_z_m: float = -9.8,
    camera_offset_z_m: float = -0.05,
) -> tuple[list[dict], list[dict]]:
    """Return sector comparisons and per-run candidate summaries.

    Only wall-calibration poses with heading zero are compared. Other trials,
    including the corridor view, are intentionally excluded from this plane.
    """
    if not math.isfinite(wall_face_z_m) or not math.isfinite(camera_offset_z_m):
        raise ValueError("Wall and camera offsets must be finite")

    with run_index_path.open(newline="", encoding="utf-8") as source:
        run_index = list(csv.DictReader(source))

    sector_rows: list[dict] = []
    summary_rows: list[dict] = []
    all_candidates: list[dict] = []
    total_unknown = 0
    for run in run_index:
        pose_label = run["offline_pose_label"]
        if pose_label not in WALL_POSES:
            continue
        heading = _float_or_none(run.get("heading_rad"))
        if heading is None or abs(heading) > 1e-3:
            raise ValueError(
                f"Wall plane comparison requires heading 0: {run['run_id']}"
            )
        gps_z = _float_or_none(run.get("gps_z_m"))
        if gps_z is None:
            raise ValueError(f"GPS Z missing for {run['run_id']}")

        sectors_path = analysis_dir / run["run_id"] / "v3_sectors.csv"
        with sectors_path.open(newline="", encoding="utf-8") as source:
            sectors = list(csv.DictReader(source))

        candidates: list[dict] = []
        for sector in sectors:
            candidate_range = _float_or_none(sector.get("distance_m"))
            bearing = _float_or_none(sector.get("bearing_robot_deg"))
            reference = None
            signed_error = None
            absolute_error = None
            if sector.get("status") == "occupied" and candidate_range is not None and bearing is not None:
                camera_origin_z = gps_z + camera_offset_z_m
                reference = wall_plane_distance_m(
                    camera_origin_z_m=camera_origin_z,
                    wall_face_z_m=wall_face_z_m,
                    bearing_robot_deg=bearing,
                )
                signed_error = candidate_range - reference
                absolute_error = abs(signed_error)
                candidates.append(
                    {
                        "absolute_error_m": absolute_error,
                        "signed_error_m": signed_error,
                        "near_field_occluded": sector.get("near_field_occluded") == "True",
                    }
                )

            sector_rows.append(
                {
                    "run_id": run["run_id"],
                    "offline_pose_label": pose_label,
                    "repetition": run.get("repetition", ""),
                    "gps_z_m": gps_z,
                    "logged_trial_id": run.get("logged_trial_id", ""),
                    "sector_id": sector.get("sector_id", ""),
                    "camera_angle_deg": sector.get("camera_angle_deg", ""),
                    "bearing_robot_deg": bearing if bearing is not None else "",
                    "candidate_distance_m": candidate_range if candidate_range is not None else "",
                    "reference_wall_plane_distance_m": reference if reference is not None else "",
                    "signed_error_m": signed_error if signed_error is not None else "",
                    "absolute_error_m": absolute_error if absolute_error is not None else "",
                    "near_field_occluded": sector.get("near_field_occluded", ""),
                    "status": sector.get("status", ""),
                    "unknown_reason": sector.get("reason", ""),
                }
            )

        errors = [row["absolute_error_m"] for row in candidates]
        signed_errors = [row["signed_error_m"] for row in candidates]
        all_candidates.extend(candidates)
        total_unknown += sum(row.get("status") == "unknown" for row in sectors)
        summary_rows.append(
            {
                "run_id": run["run_id"],
                "offline_pose_label": pose_label,
                "repetition": run.get("repetition", ""),
                "gps_z_m": gps_z,
                "sectors_total": len(sectors),
                "candidate_count": len(candidates),
                "unknown_count": sum(row.get("status") == "unknown" for row in sectors),
                "candidate_mae_m": sum(errors) / len(errors) if errors else "",
                "candidate_mean_signed_error_m": (
                    sum(signed_errors) / len(signed_errors) if signed_errors else ""
                ),
                "near_field_occluded_candidate_count": sum(
                    row["near_field_occluded"] for row in candidates
                ),
                "interpretation": "Offline WBT-plane consistency only; not validated metric performance",
            }
        )
    all_errors = [row["absolute_error_m"] for row in all_candidates]
    all_signed_errors = [row["signed_error_m"] for row in all_candidates]
    summary_rows.append(
        {
            "run_id": "pooled_wall_runs",
            "offline_pose_label": "pooled",
            "repetition": "",
            "gps_z_m": "",
            "sectors_total": len(sector_rows),
            "candidate_count": len(all_candidates),
            "unknown_count": total_unknown,
            "candidate_mae_m": sum(all_errors) / len(all_errors) if all_errors else "",
            "candidate_mean_signed_error_m": (
                sum(all_signed_errors) / len(all_signed_errors)
                if all_signed_errors else ""
            ),
            "near_field_occluded_candidate_count": sum(
                row["near_field_occluded"] for row in all_candidates
            ),
            "interpretation": "Offline WBT-plane consistency only; not validated metric performance",
        }
    )
    return sector_rows, summary_rows


def write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline candidate-range comparison to the scenario-01 wall plane"
    )
    parser.add_argument(
        "--run-index",
        type=Path,
        default=Path("experiments/exp_002_visual_ranging/results/run_index.csv"),
    )
    parser.add_argument(
        "--analysis-dir",
        type=Path,
        default=Path("experiments/exp_002_visual_ranging/results/analysis_v3_tilt30"),
    )
    parser.add_argument("--wall-face-z-m", type=float, default=-9.8)
    parser.add_argument("--camera-offset-z-m", type=float, default=-0.05)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("experiments/exp_002_visual_ranging/results"),
    )
    args = parser.parse_args()
    try:
        sectors, summaries = compare_wall_runs(
            args.run_index,
            args.analysis_dir,
            wall_face_z_m=args.wall_face_z_m,
            camera_offset_z_m=args.camera_offset_z_m,
        )
        write_csv(args.output_dir / "wall_range_comparison_tilt30.csv", SECTOR_FIELDS, sectors)
        write_csv(
            args.output_dir / "wall_range_summary_tilt30.csv", SUMMARY_FIELDS, summaries
        )
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(f"Compared {len(sectors)} sectors across {len(summaries) - 1} wall runs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
