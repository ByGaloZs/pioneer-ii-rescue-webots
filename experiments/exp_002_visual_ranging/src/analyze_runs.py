"""Offline batch analysis of EXP-002 camera captures.

Run from repo root. Does not alter the original logger CSV or infer WBT
geometry at runtime. Unknown sectors have no metric distance or MAE.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from angular_observations import AngularConfig, observe_sectors
from self_occlusion import load_occlusion_mask
from spherical_geometry import SphericalCameraModel


CALIBRATION_TRIAL_ID = re.compile(
    r"(?:calib01_tilt30_p[123](?:_r[0-9]{2})?|geom01_tilt30_(?:right|left|front)_r[0-9]{2})"
)


def analyze_run(
    run_dir: Path,
    *,
    mask_config: Path | None = None,
    output_dir: Path | None = None,
    config: AngularConfig | None = None,
    camera_height_m: float = 0.44,
    camera_tilt_rad: float = -0.5236,
    camera_offset_x_m: float = 0.025,
    camera_offset_z_m: float = -0.05,
    camera_yaw_rad: float = 0.0,
) -> dict:
    csv_path = run_dir / "frames.csv"
    with csv_path.open(newline="", encoding="utf-8") as source:
        frames = list(csv.DictReader(source))
    if not frames:
        raise ValueError(f"No frame metadata: {csv_path}")
    # These tests all have one frame; support multiple saved frames as well.
    saved = [f for f in frames if f.get("image_saved") == "1" and f.get("image_file")]
    if not saved:
        raise ValueError(f"No saved JPEG frames recorded: {csv_path}")
    results: list[dict] = []
    mask_verified = False
    if mask_config is not None:
        config_json = json.loads(mask_config.read_text(encoding="utf-8"))
        mask_verified = config_json.get("verified") is True
    for frame in saved:
        name = Path(frame["image_file"])
        if name.name != str(name) or name.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            raise ValueError("Unsafe image_file in frames.csv")
        image = cv2.imread(str(run_dir / name), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Cannot decode image {name}")
        height, width = image.shape[:2]
        fov = float(frame["camera_fov_rad"])
        trial = frame.get("trial_id", "")
        if CALIBRATION_TRIAL_ID.fullmatch(trial) is None:
            raise ValueError(f"Unknown camera mount calibration for trial ID: {trial!r}")
        model = SphericalCameraModel(
            width=width,
            height=height,
            horizontal_fov_rad=fov,
            height_m=camera_height_m,
            tilt_rad=camera_tilt_rad,
            offset_x_m=camera_offset_x_m,
            offset_z_m=camera_offset_z_m,
            yaw_rad=camera_yaw_rad,
        )
        mask = (load_occlusion_mask(mask_config, (height, width))
                if mask_config is not None else np.zeros((height, width), bool))
        observations, labels = observe_sectors(
            image, model, occlusion_mask=mask, mask_verified=mask_verified, config=config
        )
        for observation in observations:
            results.append({"frame_file": name.name, "trial_id": trial,
                            **observation.as_record()})
        if output_dir is not None:
            output_dir.mkdir(parents=True, exist_ok=True)
            # Visualization only. The palette is indexed by class 0..3.
            palette = np.array([[20, 20, 20], [130, 130, 130], [230, 170, 25],
                                [40, 180, 40]], dtype=np.uint8)
            if not cv2.imwrite(str(output_dir / f"{trial}_{name.stem}_classes.png"),
                               palette[labels]):
                raise OSError("Could not save classification diagnostic")
    statuses = Counter(row["status"] for row in results)
    reasons = Counter(row["reason"] for row in results if row["reason"])
    summary = {
        "run_directory": str(run_dir),
        "mask_config": str(mask_config) if mask_config else None,
        "self_occlusion_mask_verified": mask_verified,
        "frames": len(saved),
        "camera_model": {
            "height_m": camera_height_m,
            "tilt_rad": camera_tilt_rad,
            "offset_x_m": camera_offset_x_m,
            "offset_z_m": camera_offset_z_m,
            "yaw_rad": camera_yaw_rad,
        },
        "sectors_total": len(results),
        "occupied": statuses["occupied"],
        "unknown": statuses["unknown"],
        "status_reasons": dict(sorted(reasons.items())),
        "metric_error_mae_m": None,
        "note": "Geometric candidates are not validated measurements. No free-space inference.",
    }
    if output_dir is not None:
        columns = ["frame_file", "trial_id", "sector_id", "camera_angle_deg", "status",
                   "distance_m", "confidence", "contact_y_px", "bearing_robot_deg",
                   "reason", "near_field_occluded"]
        with (output_dir / "v3_sectors.csv").open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=columns, lineterminator="\n")
            writer.writeheader()
            writer.writerows(results)
        (output_dir / "v3_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Conservative EXP-002 V3 angular analysis")
    parser.add_argument("run_dir", type=Path, nargs="*", help="EXP-002 run_... folders")
    parser.add_argument(
        "--run-index",
        type=Path,
        help="Offline run_index.csv; analyzes only indexed wall-calibration runs",
    )
    parser.add_argument("--mask-config", type=Path,
                        help="Orientation-specific verified mask JSON; omit to report unknown")
    parser.add_argument("--camera-height-m", type=float, default=0.44)
    parser.add_argument("--camera-tilt-rad", type=float, default=-0.5236)
    parser.add_argument("--camera-offset-x-m", type=float, default=0.025)
    parser.add_argument("--camera-offset-z-m", type=float, default=-0.05)
    parser.add_argument("--camera-yaw-rad", type=float, default=0.0)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="Diagnostics directory; one output subfolder per run")
    args = parser.parse_args()
    if args.run_index is not None:
        if args.run_dir:
            parser.error("provide positional runs or --run-index, not both")
        with args.run_index.open(newline="", encoding="utf-8") as source:
            indexed_runs = list(csv.DictReader(source))
        run_dirs = [args.run_index.parent / row["run_id"] for row in indexed_runs]
    else:
        run_dirs = args.run_dir
    if not run_dirs:
        parser.error("provide at least one run directory or --run-index")
    geometry = (args.camera_height_m, args.camera_tilt_rad,
                args.camera_offset_x_m, args.camera_offset_z_m, args.camera_yaw_rad)
    if not all(math.isfinite(value) for value in geometry):
        parser.error("camera geometry values must be finite")
    summaries = [analyze_run(run, mask_config=args.mask_config,
                             output_dir=args.output_dir / run.name,
                             camera_height_m=args.camera_height_m,
                             camera_tilt_rad=args.camera_tilt_rad,
                             camera_offset_x_m=args.camera_offset_x_m,
                             camera_offset_z_m=args.camera_offset_z_m,
                             camera_yaw_rad=args.camera_yaw_rad)
                 for run in run_dirs]
    print(json.dumps(summaries, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
