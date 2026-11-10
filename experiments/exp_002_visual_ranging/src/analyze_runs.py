"""Offline batch analysis of EXP-002 camera captures.

Run from repo root. Does not alter the original logger CSV or infer WBT
geometry at runtime. Unknown sectors have no metric distance or MAE.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from angular_observations import AngularConfig, observe_sectors
from self_occlusion import load_occlusion_mask
from spherical_geometry import SphericalCameraModel


KNOWN_CALIBRATION_TRIALS = frozenset((
    "calib01_tilt30_p1", "calib01_tilt30_p2", "calib01_tilt30_p3",
))


def analyze_run(
    run_dir: Path,
    *,
    mask_config: Path | None = None,
    output_dir: Path | None = None,
    config: AngularConfig | None = None,
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
        if trial not in KNOWN_CALIBRATION_TRIALS:
            raise ValueError(f"Unknown camera mount calibration for trial ID: {trial!r}")
        model = SphericalCameraModel(width=width, height=height, horizontal_fov_rad=fov)
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
            writer = csv.DictWriter(output, fieldnames=columns)
            writer.writeheader()
            writer.writerows(results)
        (output_dir / "v3_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Conservative EXP-002 V3 angular analysis")
    parser.add_argument("run_dir", type=Path, nargs="+", help="EXP-002 run_... folders")
    parser.add_argument("--mask-config", type=Path,
                        help="Orientation-specific verified mask JSON; omit to report unknown")
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="Diagnostics directory; one output subfolder per run")
    args = parser.parse_args()
    summaries = [analyze_run(run, mask_config=args.mask_config,
                             output_dir=args.output_dir / run.name)
                 for run in args.run_dir]
    print(json.dumps(summaries, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
