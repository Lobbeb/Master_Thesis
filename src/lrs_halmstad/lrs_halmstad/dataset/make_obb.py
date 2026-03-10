#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import cv2
except Exception as exc:  # pragma: no cover - import-time environment issue
    raise RuntimeError("OpenCV (cv2) is required for OBB label generation") from exc

import numpy as np

from lrs_halmstad.dataset.sync_check import IMAGE_EXTS, DATASETS_ROOT, resolve_dataset_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            f"Generate Ultralytics OBB labels in labels_obb/ for one or more datasets under {DATASETS_ROOT} "
            "using the saved projected_points metadata."
        )
    )
    parser.add_argument(
        "datasets",
        nargs="+",
        help=(
            f"Dataset path(s), absolute or relative to {DATASETS_ROOT}. "
            "Examples: warehouse_v1/run1 warehouse_v1/run2"
        ),
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Rewrite existing labels_obb files instead of leaving them untouched.",
    )
    parser.add_argument(
        "--show",
        type=int,
        default=10,
        help="Maximum number of warnings to print per dataset. Use 0 to print all.",
    )
    return parser.parse_args()


def _iter_image_stems(images_dir: Path) -> list[str]:
    stems: list[str] = []
    for path in sorted(images_dir.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in IMAGE_EXTS:
            continue
        stems.append(path.relative_to(images_dir).with_suffix("").as_posix())
    return stems


def _polygon_area(points: np.ndarray) -> float:
    xs = points[:, 0]
    ys = points[:, 1]
    return 0.5 * float(np.sum(xs * np.roll(ys, -1) - ys * np.roll(xs, -1)))


def _order_quad(points: np.ndarray) -> np.ndarray:
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    if _polygon_area(ordered) > 0.0:
        ordered = ordered[::-1]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def _obb_from_metadata(metadata: dict) -> tuple[int, np.ndarray] | None:
    bbox_yolo = metadata.get("bbox_yolo")
    if not bbox_yolo:
        return None

    camera_model = metadata.get("camera_model") or {}
    width = float(camera_model.get("width") or 0.0)
    height = float(camera_model.get("height") or 0.0)
    projected_points = metadata.get("projected_points") or []
    if width <= 0.0 or height <= 0.0 or len(projected_points) < 3:
        return None

    pts = np.asarray(projected_points, dtype=np.float32)
    rect = cv2.minAreaRect(pts)
    box = cv2.boxPoints(rect).astype(np.float64)
    box[:, 0] = np.clip(box[:, 0], 0.0, width - 1.0)
    box[:, 1] = np.clip(box[:, 1], 0.0, height - 1.0)
    box = _order_quad(box)
    box[:, 0] /= width
    box[:, 1] /= height
    return int(bbox_yolo.get("class_id", 0)), box


def _write_label(path: Path, obb: tuple[int, np.ndarray] | None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="ascii") as fh:
        if obb is None:
            return
        class_id, box = obb
        coords = " ".join(f"{float(value):.6f}" for value in box.reshape(-1))
        fh.write(f"{class_id} {coords}\n")


def convert_dataset(dataset_dir: Path, overwrite: bool, show_limit: int) -> int:
    if not (dataset_dir / "images").is_dir():
        raise FileNotFoundError(f"Dataset directory is missing images/: {dataset_dir}")
    if not (dataset_dir / "metadata").is_dir():
        raise FileNotFoundError(f"Dataset directory is missing metadata/: {dataset_dir}")

    written = 0
    warnings: list[str] = []
    splits = sorted(path.name for path in (dataset_dir / "images").iterdir() if path.is_dir())
    for split in splits:
        images_dir = dataset_dir / "images" / split
        metadata_dir = dataset_dir / "metadata" / split
        output_dir = dataset_dir / "labels_obb" / split

        for stem in _iter_image_stems(images_dir):
            metadata_path = metadata_dir / f"{stem}.json"
            output_path = output_dir / f"{stem}.txt"
            if output_path.exists() and not overwrite:
                continue
            if not metadata_path.is_file():
                warnings.append(f"missing metadata for {split}/{stem}")
                _write_label(output_path, None)
                written += 1
                continue

            with open(metadata_path, "r", encoding="utf-8") as fh:
                metadata = json.load(fh)
            obb = _obb_from_metadata(metadata)
            if obb is None and metadata.get("bbox_yolo") is not None:
                warnings.append(f"insufficient OBB metadata for {split}/{stem}")
            _write_label(output_path, obb)
            written += 1

    print(f"Dataset: {dataset_dir}")
    print(f"  Wrote {written} labels_obb file(s)")
    if warnings:
        print(f"  Warnings: {len(warnings)}")
        shown = warnings if show_limit == 0 else warnings[:show_limit]
        for item in shown:
            print(f"    {item}")
        if show_limit and len(shown) < len(warnings):
            print(f"    ... {len(warnings) - len(shown)} more")
    return written


def main() -> None:
    args = parse_args()
    total = 0
    for value in args.datasets:
        dataset_dir = resolve_dataset_path(value)
        total += convert_dataset(dataset_dir, overwrite=args.overwrite, show_limit=args.show)
    print(f"Done: wrote {total} OBB label file(s) across {len(args.datasets)} dataset(s).")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        raise SystemExit(130)
