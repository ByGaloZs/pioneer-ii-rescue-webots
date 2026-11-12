"""Compare EXP-002 camera geometry with independently surveyed references.

Reference values are manually surveyed in the robot frame and are never read
from a WBT. Bearing rows validate pixel-to-azimuth geometry; range rows compare
V3 floor-contact candidates against measured camera-origin-to-contact distance.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from spherical_geometry import SphericalCameraModel


REFERENCE_FIELDS = (
    "measurement_type",
    "run_id",
    "frame_file",
    "sector_id",
    "image_x_px",
    "image_y_px",
    "reference_bearing_deg",
    "reference_distance_m",
    "reference_method",
    "notes",
)
RESULT_FIELDS = (
    "measurement_type", "run_id", "frame_file", "sector_id", "status",
    "observed_value", "reference_value", "signed_error", "absolute_error",
    "near_field_occluded", "reference_method", "notes",
)


def _number(value: str, field: str, *, required: bool = True) -> float | None:
    if value == "" and not required:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {field}: {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"Non-finite {field}")
    return number


def _load_frame(run_dir: Path, frame_file: str) -> tuple[dict, int, int]:
    if Path(frame_file).name != frame_file:
        raise ValueError(f"Unsafe frame_file: {frame_file!r}")
    with (run_dir / "frames.csv").open(newline="", encoding="utf-8") as source:
        frames = list(csv.DictReader(source))
    for frame in frames:
        if frame.get("image_file") == frame_file and frame.get("image_saved") == "1":
            return frame, int(frame["image_width_px"]), int(frame["image_height_px"])
    raise ValueError(f"Saved frame {frame_file!r} not found in {run_dir}")


def _load_sector(analysis_dir: Path, run_id: str, frame_file: str, sector_id: int) -> dict:
    path = analysis_dir / run_id / "v3_sectors.csv"
    with path.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            if row.get("frame_file") == frame_file and int(row["sector_id"]) == sector_id:
                return row
    raise ValueError(f"Sector {sector_id} for {run_id}/{frame_file} not found in {path}")


def validate_references(
    references_path: Path, results_root: Path, analysis_dir: Path,
    *, camera_tilt_rad: float = -0.5236, camera_yaw_rad: float = 0.0,
) -> tuple[list[dict], dict]:
    with references_path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames is None or not set(REFERENCE_FIELDS).issubset(reader.fieldnames):
            raise ValueError("Reference CSV is missing required columns")
        references = list(reader)
    if not references:
        raise ValueError("Reference CSV contains no measurements")

    results: list[dict] = []
    bearing_errors: list[float] = []
    bearing_left = bearing_right = 0
    range_errors: list[float] = []
    range_excluded: dict[str, int] = {}
    range_near_field_occluded_count = 0
    for item in references:
        kind = item["measurement_type"]
        if kind not in ("bearing", "range"):
            raise ValueError(f"Unsupported measurement_type: {kind!r}")
        run_id = item["run_id"].strip()
        if not run_id.startswith("run_") or Path(run_id).name != run_id:
            raise ValueError(f"Invalid run_id: {run_id!r}")
        analysis_summary_path = analysis_dir / run_id / "v3_summary.json"
        analysis_summary = json.loads(analysis_summary_path.read_text(encoding="utf-8"))
        analyzed_model = analysis_summary.get("camera_model", {})
        for parameter, expected in (("tilt_rad", camera_tilt_rad),
                                    ("yaw_rad", camera_yaw_rad)):
            actual = _number(str(analyzed_model.get(parameter, "")), parameter)
            if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
                raise ValueError(
                    f"Camera {parameter} differs between analysis and reference validation "
                    f"for {run_id}"
                )
        frame_file = item["frame_file"].strip()
        run_dir = results_root / run_id
        frame, width, height = _load_frame(run_dir, frame_file)
        model = SphericalCameraModel(
            width=width,
            height=height,
            horizontal_fov_rad=_number(frame["camera_fov_rad"], "camera_fov_rad"),
            tilt_rad=camera_tilt_rad,
            yaw_rad=camera_yaw_rad,
        )
        observed = reference = None
        status = "measured"
        sector_id = ""
        near_field_occluded = ""
        if kind == "bearing":
            x = _number(item["image_x_px"], "image_x_px")
            y = _number(item["image_y_px"], "image_y_px")
            reference = _number(item["reference_bearing_deg"], "reference_bearing_deg")
            ray_x, _, ray_z = model.robot_ray(x, y)
            observed = math.degrees(math.atan2(ray_x, -ray_z))
            # Wrap angular residual to [-180, 180).
            error = (observed - reference + 180.0) % 360.0 - 180.0
            if reference < 0:
                bearing_left += 1
            elif reference > 0:
                bearing_right += 1
            bearing_errors.append(error)
        else:
            sector_id_value = _number(item["sector_id"], "sector_id")
            if sector_id_value is None or not sector_id_value.is_integer():
                raise ValueError("sector_id must be an integer for range rows")
            sector_id = int(sector_id_value)
            reference = _number(item["reference_distance_m"], "reference_distance_m")
            sector = _load_sector(analysis_dir, run_id, frame_file, sector_id)
            if sector.get("status") != "occupied" or not sector.get("distance_m"):
                status = f"excluded_{sector.get('reason') or sector.get('status') or 'no_range'}"
            else:
                observed = _number(sector["distance_m"], "candidate distance")
                error = observed - reference
                range_errors.append(error)
                near_field_occluded = sector.get("near_field_occluded", "").lower() == "true"
                if near_field_occluded:
                    range_near_field_occluded_count += 1
            if status != "measured":
                range_excluded[status] = range_excluded.get(status, 0) + 1
                error = None

        results.append({
            "measurement_type": kind,
            "run_id": run_id,
            "frame_file": frame_file,
            "sector_id": sector_id,
            "status": status,
            "observed_value": observed if observed is not None else "",
            "reference_value": reference if reference is not None else "",
            "signed_error": error if status == "measured" else "",
            "absolute_error": abs(error) if status == "measured" else "",
            "near_field_occluded": near_field_occluded,
            "reference_method": item["reference_method"].strip(),
            "notes": item["notes"].strip(),
        })

    summary = {
        "reference_file": str(references_path),
        "camera_tilt_rad": camera_tilt_rad,
        "camera_yaw_rad": camera_yaw_rad,
        "measurements_total": len(results),
        "bearing": _error_summary(bearing_errors),
        "bearing_reference_left_count": bearing_left,
        "bearing_reference_right_count": bearing_right,
        "range": _error_summary(range_errors),
        "range_excluded": range_excluded,
        "range_candidates_with_unknown_near_field": range_near_field_occluded_count,
        "interpretation": (
            "A visible floor-contact transition is compared even when nearer pixels are masked; "
            "those nearer pixels remain unknown, not free. No acceptance threshold is assumed."
        ),
    }
    return results, summary


def _error_summary(errors: list[float]) -> dict:
    if not errors:
        return {"count": 0, "bias": None, "mae": None, "rmse": None}
    return {
        "count": len(errors),
        "bias": sum(errors) / len(errors),
        "mae": sum(abs(value) for value in errors) / len(errors),
        "rmse": math.sqrt(sum(value * value for value in errors) / len(errors)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--references", type=Path, required=True,
                        help="CSV of independently surveyed marker/contact references")
    parser.add_argument("--results-root", type=Path,
                        default=Path("experiments/exp_002_visual_ranging/results"))
    parser.add_argument("--analysis-dir", type=Path, required=True,
                        help="analyze_runs.py output directory")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--camera-tilt-rad", type=float, default=-0.5236)
    parser.add_argument("--camera-yaw-rad", type=float, default=0.0)
    args = parser.parse_args()
    try:
        rows, summary = validate_references(
            args.references, args.results_root, args.analysis_dir,
            camera_tilt_rad=args.camera_tilt_rad, camera_yaw_rad=args.camera_yaw_rad,
        )
        args.output_dir.mkdir(parents=True, exist_ok=True)
        with (args.output_dir / "independent_reference_comparison.csv").open(
            "w", newline="", encoding="utf-8"
        ) as output:
            writer = csv.DictWriter(output, fieldnames=RESULT_FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        (args.output_dir / "independent_reference_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
