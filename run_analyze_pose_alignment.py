#!/usr/bin/env python3
import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean, pstdev
from typing import Iterable, Optional

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Pose2D:
    t: float
    x: float
    y: float
    yaw: float


@dataclass
class MatchRow:
    t: float
    source_x: float
    source_y: float
    ref_x: float
    ref_y: float
    dx: float
    dy: float
    dist: float
    yaw_err: float
    rigid_x_err: float
    rigid_y_err: float
    rigid_dist_err: float


def pose_from_any_message(msg) -> Optional[Pose2D]:
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        header = msg.header
        pose = msg.pose.pose
    elif hasattr(msg, "pose") and hasattr(msg.pose, "position"):
        header = msg.header
        pose = msg.pose
    else:
        return None

    stamp = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
    position = pose.position
    orientation = pose.orientation
    return Pose2D(
        t=stamp,
        x=float(position.x),
        y=float(position.y),
        yaw=yaw_from_quat(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
    )


def read_topic_poses(bag_path: str, topic_name: str) -> list[Pose2D]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"), ConverterOptions("", ""))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        raise KeyError(topic_name)

    msg_type = get_message(topic_types[topic_name])
    rows: list[Pose2D] = []

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"), ConverterOptions("", ""))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue
        pose = pose_from_any_message(deserialize_message(data, msg_type))
        if pose is not None:
            rows.append(pose)
    return rows


def interpolate_pose(rows: list[Pose2D], t: float) -> Optional[Pose2D]:
    if len(rows) < 2:
        return None

    ts = [row.t for row in rows]
    idx = bisect.bisect_left(ts, t)
    if idx == 0 or idx >= len(rows):
        return None

    a = rows[idx - 1]
    b = rows[idx]
    if b.t <= a.t:
        return Pose2D(t=t, x=a.x, y=a.y, yaw=a.yaw)

    alpha = (t - a.t) / (b.t - a.t)
    dyaw = wrap_pi(b.yaw - a.yaw)
    return Pose2D(
        t=t,
        x=a.x + alpha * (b.x - a.x),
        y=a.y + alpha * (b.y - a.y),
        yaw=wrap_pi(a.yaw + alpha * dyaw),
    )


def fit_rigid_transform(source: Iterable[Pose2D], ref: Iterable[Pose2D]) -> tuple[float, float, float]:
    source_rows = list(source)
    ref_rows = list(ref)
    if len(source_rows) != len(ref_rows) or not source_rows:
        raise ValueError("Source/ref lists must be the same non-zero length")

    src_x_mean = fmean(row.x for row in source_rows)
    src_y_mean = fmean(row.y for row in source_rows)
    ref_x_mean = fmean(row.x for row in ref_rows)
    ref_y_mean = fmean(row.y for row in ref_rows)

    sxx = 0.0
    sxy = 0.0
    for src_row, ref_row in zip(source_rows, ref_rows):
        ax = src_row.x - src_x_mean
        ay = src_row.y - src_y_mean
        bx = ref_row.x - ref_x_mean
        by = ref_row.y - ref_y_mean
        sxx += ax * bx + ay * by
        sxy += ax * by - ay * bx

    theta = math.atan2(sxy, sxx)
    c = math.cos(theta)
    s = math.sin(theta)
    tx = ref_x_mean - (c * src_x_mean - s * src_y_mean)
    ty = ref_y_mean - (s * src_x_mean + c * src_y_mean)
    return theta, tx, ty


def apply_rigid(theta: float, tx: float, ty: float, pose: Pose2D) -> tuple[float, float]:
    c = math.cos(theta)
    s = math.sin(theta)
    return c * pose.x - s * pose.y + tx, s * pose.x + c * pose.y + ty


def summarize(name: str, values: list[float], unit: str = "") -> str:
    if not values:
        return f"{name}: n/a"

    values_sorted = sorted(values)
    def percentile(p: float) -> float:
        idx = min(len(values_sorted) - 1, max(0, int(round((len(values_sorted) - 1) * p))))
        return values_sorted[idx]

    return (
        f"{name}: mean={fmean(values):.4f}{unit} "
        f"std={pstdev(values):.4f}{unit} "
        f"p95={percentile(0.95):.4f}{unit} "
        f"max={values_sorted[-1]:.4f}{unit}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze how simple or time-varying a pose-source correction is")
    parser.add_argument("bag", help="Path to rosbag directory")
    parser.add_argument("--source", default="/a201_0000/platform/odom", help="Pose topic to correct")
    parser.add_argument("--reference", default="/a201_0000/amcl_pose_odom", help="Reference pose topic")
    parser.add_argument("--csv", default="", help="Optional CSV output path for matched samples")
    args = parser.parse_args()

    bag_path = str(Path(args.bag).resolve())
    try:
        source_rows = read_topic_poses(bag_path, args.source)
    except KeyError:
        raise SystemExit(f"Missing source topic in bag: {args.source}")
    try:
        ref_rows = read_topic_poses(bag_path, args.reference)
    except KeyError:
        raise SystemExit(f"Missing reference topic in bag: {args.reference}")

    matched_source: list[Pose2D] = []
    matched_ref: list[Pose2D] = []
    for source_row in source_rows:
        ref_interp = interpolate_pose(ref_rows, source_row.t)
        if ref_interp is None:
            continue
        matched_source.append(source_row)
        matched_ref.append(ref_interp)

    if len(matched_source) < 10:
        raise SystemExit("Not enough overlapping samples to analyze")

    theta, tx, ty = fit_rigid_transform(matched_source, matched_ref)
    rows: list[MatchRow] = []
    for source_row, ref_row in zip(matched_source, matched_ref):
        dx = ref_row.x - source_row.x
        dy = ref_row.y - source_row.y
        rigid_x, rigid_y = apply_rigid(theta, tx, ty, source_row)
        rigid_x_err = ref_row.x - rigid_x
        rigid_y_err = ref_row.y - rigid_y
        rows.append(
            MatchRow(
                t=source_row.t,
                source_x=source_row.x,
                source_y=source_row.y,
                ref_x=ref_row.x,
                ref_y=ref_row.y,
                dx=dx,
                dy=dy,
                dist=math.hypot(dx, dy),
                yaw_err=wrap_pi(ref_row.yaw - source_row.yaw),
                rigid_x_err=rigid_x_err,
                rigid_y_err=rigid_y_err,
                rigid_dist_err=math.hypot(rigid_x_err, rigid_y_err),
            )
        )

    first = rows[0]
    last = rows[-1]
    print(f"Bag: {bag_path}")
    print(f"Source topic:     {args.source}")
    print(f"Reference topic:  {args.reference}")
    print(f"Matched samples:  {len(rows)}")
    print(f"Time span:        {first.t:.3f}s -> {last.t:.3f}s ({last.t - first.t:.3f}s)")
    print("")
    print(
        "Best rigid source->reference: "
        f"theta={math.degrees(theta):.3f} deg  tx={tx:.4f} m  ty={ty:.4f} m"
    )
    print("")
    print(summarize("Raw xy delta", [row.dist for row in rows], " m"))
    print(summarize("Raw dx", [row.dx for row in rows], " m"))
    print(summarize("Raw dy", [row.dy for row in rows], " m"))
    print(summarize("Raw yaw err", [math.degrees(row.yaw_err) for row in rows], " deg"))
    print("")
    print(summarize("Rigid-fit residual", [row.rigid_dist_err for row in rows], " m"))
    print(summarize("Rigid-fit x residual", [row.rigid_x_err for row in rows], " m"))
    print(summarize("Rigid-fit y residual", [row.rigid_y_err for row in rows], " m"))
    print("")
    print(
        "Start/end raw delta: "
        f"start=({first.dx:.4f}, {first.dy:.4f}) m  "
        f"end=({last.dx:.4f}, {last.dy:.4f}) m"
    )
    print(
        "Start/end rigid residual: "
        f"start=({first.rigid_x_err:.4f}, {first.rigid_y_err:.4f}) m  "
        f"end=({last.rigid_x_err:.4f}, {last.rigid_y_err:.4f}) m"
    )
    print("")

    residual_p95 = sorted(row.rigid_dist_err for row in rows)[int(round((len(rows) - 1) * 0.95))]
    if residual_p95 <= 0.10:
        print("Assessment: a single 2D rigid transform is a plausible approximation for this run.")
    else:
        print("Assessment: the correction is meaningfully time-varying; a single 2D transform is weak.")

    if args.csv:
        csv_path = Path(args.csv).resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "t_s",
                    "source_x",
                    "source_y",
                    "ref_x",
                    "ref_y",
                    "dx_m",
                    "dy_m",
                    "dist_m",
                    "yaw_err_deg",
                    "rigid_x_err_m",
                    "rigid_y_err_m",
                    "rigid_dist_err_m",
                ]
            )
            for row in rows:
                writer.writerow(
                    [
                        f"{row.t:.6f}",
                        f"{row.source_x:.6f}",
                        f"{row.source_y:.6f}",
                        f"{row.ref_x:.6f}",
                        f"{row.ref_y:.6f}",
                        f"{row.dx:.6f}",
                        f"{row.dy:.6f}",
                        f"{row.dist:.6f}",
                        f"{math.degrees(row.yaw_err):.6f}",
                        f"{row.rigid_x_err:.6f}",
                        f"{row.rigid_y_err:.6f}",
                        f"{row.rigid_dist_err:.6f}",
                    ]
                )
        print(f"CSV written to:   {csv_path}")


if __name__ == "__main__":
    main()
