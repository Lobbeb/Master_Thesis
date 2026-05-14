#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import yaml
from ament_index_python.packages import get_package_share_directory


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _shell_join(parts: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in parts)


def _require_dict(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    return value


@dataclass(frozen=True)
class CaptureConfig:
    uav_name: str
    hz: float
    duration_s: float
    save_overlay: bool
    save_metadata: bool
    save_negative_examples: bool
    make_obb: bool
    obb_overlay: bool
    obb_overwrite: bool


@dataclass(frozen=True)
class StartupConfig:
    warmup_s: float
    stop_on_error: bool
    scenario_retry_count: int
    retry_delay_s: float


@dataclass(frozen=True)
class GimbalConfig:
    mode: str
    interval_s: float
    hold_s: float
    pan_center_deg: float
    pan_amplitude_deg: float
    pan_min_deg: float
    pan_max_deg: float
    tilt_center_deg: float
    tilt_amplitude_deg: float
    tilt_min_deg: float
    tilt_max_deg: float


@dataclass(frozen=True)
class PoseVariation:
    name: str
    leader_heading_offset_deg: float
    d_target_m: float
    hold_s: float


@dataclass(frozen=True)
class Scenario:
    name: str
    waypoint: str
    nav2_goals: str
    uav_pattern: str


@dataclass(frozen=True)
class Manifest:
    world: str
    output_dir: str
    capture: CaptureConfig
    startup: StartupConfig
    gimbal: GimbalConfig
    pose_variations: list[PoseVariation]
    scenarios: list[Scenario]


class ScenarioFailure(RuntimeError):
    pass


class InterruptedFailure(RuntimeError):
    pass


class LeaderDatasetCollector:
    def __init__(self, argv: list[str]) -> None:
        self.package_share = Path(get_package_share_directory("lrs_halmstad"))
        self.ws_root = Path(os.environ.get("LRS_HALMSTAD_WS_ROOT", os.getcwd())).resolve()
        self.state_dir = Path("/tmp/halmstad_ws")
        self.state_dir.mkdir(parents=True, exist_ok=True)

        self.world_arg: Optional[str] = None
        self.manifest_arg: Optional[str] = None
        self.output_override: Optional[str] = None
        self.session_name = "halmstad-baylands-leader-dataset"
        self.launcher = "nav2_tuning"
        self.scenario_filter_names: list[str] = []
        self.dry_run = False

        self._stop_requested = False
        self._active_capture: Optional[subprocess.Popen[str]] = None
        self._temp_params_file: Optional[Path] = None
        self._summary_path: Optional[Path] = None
        self._run_started_at = _utc_now_iso()
        self._summary: dict[str, Any] = {
            "started_at": self._run_started_at,
            "dry_run": False,
            "launcher": self.launcher,
            "scenario_filter": [],
            "manifest_path": "",
            "output_dir": "",
            "session_name": self.session_name,
            "params_file": "",
            "params_snapshot": "",
            "scenarios": [],
        }

        self._parse_args(argv)
        self.manifest_path = self._resolve_manifest_path()
        self.manifest = self._load_manifest(self.manifest_path)
        self.selected_scenarios = self._select_scenarios(self.manifest.scenarios)
        self.world = self._resolve_world()
        self.output_dir = self._resolve_output_dir()
        self._summary["dry_run"] = self.dry_run
        self._summary["manifest_path"] = str(self.manifest_path)
        self._summary["output_dir"] = str(self.output_dir)
        self._summary["session_name"] = self.session_name
        self._summary["launcher"] = self.launcher
        self._summary["scenario_filter"] = self.scenario_filter_names

        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _parse_args(self, argv: list[str]) -> None:
        args = list(argv)
        if args and ":=" not in args[0] and "=" not in args[0]:
            self.world_arg = args.pop(0)

        for arg in args:
            if arg.startswith("manifest:="):
                self.manifest_arg = arg.split(":=", 1)[1]
            elif arg.startswith("out:="):
                self.output_override = arg.split(":=", 1)[1]
            elif arg.startswith("session:="):
                self.session_name = arg.split(":=", 1)[1].strip() or self.session_name
            elif arg.startswith("launcher:="):
                self.launcher = arg.split(":=", 1)[1].strip() or self.launcher
                if self.launcher not in ("nav2_tuning", "tmux_1to1", "external", "manual"):
                    raise ValueError("launcher must be nav2_tuning, tmux_1to1, external, or manual")
            elif arg.startswith("route:=") or arg.startswith("scenario:="):
                self.scenario_filter_names.append(arg.split(":=", 1)[1].strip())
            elif arg.startswith("routes:=") or arg.startswith("scenarios:="):
                raw = arg.split(":=", 1)[1]
                self.scenario_filter_names.extend(item.strip() for item in raw.split(","))
            elif arg.startswith("dry_run:="):
                self.dry_run = _coerce_bool(arg.split(":=", 1)[1])
            else:
                raise ValueError(
                    "Unknown argument: "
                    f"{arg}\nUsage: ./run.sh collect_leader_dataset [world] "
                    "[manifest:=path.yaml] [out:=datasets/name] [session:=name] "
                    "[route:=name] [routes:=a,b] [launcher:=nav2_tuning|tmux_1to1|external] "
                    "[dry_run:=true|false]"
                )

    def _resolve_manifest_path(self) -> Path:
        source_default_path = (
            self.ws_root / "src" / "lrs_halmstad" / "config" / "baylands_leader_dataset_manifest.yaml"
        )
        default_path = (
            source_default_path
            if source_default_path.is_file()
            else self.package_share / "config" / "baylands_leader_dataset_manifest.yaml"
        )
        raw = self.manifest_arg
        if not raw:
            return default_path

        raw_path = Path(os.path.expanduser(raw))
        candidates = []
        if raw_path.is_absolute():
            candidates.append(raw_path)
        else:
            candidates.append((Path.cwd() / raw_path).resolve())
            candidates.append((self.ws_root / raw_path).resolve())
            candidates.append((self.package_share / "config" / raw_path).resolve())

        for candidate in candidates:
            if candidate.is_file():
                return candidate

        raise FileNotFoundError(f"Manifest not found: {raw}")

    def _load_manifest(self, path: Path) -> Manifest:
        with path.open("r", encoding="utf-8") as handle:
            raw_data = yaml.safe_load(handle) or {}

        data = _require_dict(raw_data, "manifest")
        capture = _require_dict(data.get("capture", {}), "capture")
        startup = _require_dict(data.get("startup", {}), "startup")
        gimbal = _require_dict(data.get("gimbal", {}), "gimbal")
        pose_variations_raw = _require_list(data.get("pose_variations", []), "pose_variations")
        scenarios_raw = _require_list(data.get("scenarios", []), "scenarios")

        pose_variations = [
            PoseVariation(
                name=str(_require_dict(item, "pose_variation").get("name", "")).strip()
                or f"variation_{index + 1}",
                leader_heading_offset_deg=float(_require_dict(item, "pose_variation").get("leader_heading_offset_deg")),
                d_target_m=float(_require_dict(item, "pose_variation").get("d_target_m")),
                hold_s=float(_require_dict(item, "pose_variation").get("hold_s")),
            )
            for index, item in enumerate(pose_variations_raw)
        ]
        scenarios = []
        for index, item in enumerate(scenarios_raw):
            scenario_data = _require_dict(item, "scenario")
            scenarios.append(
                Scenario(
                    name=str(scenario_data.get("name", "")).strip() or f"scenario_{index + 1}",
                    waypoint=str(scenario_data.get("waypoint", "")).strip(),
                    nav2_goals=str(scenario_data.get("nav2_goals", "")).strip(),
                    uav_pattern=str(scenario_data.get("uav_pattern", "manual")).strip() or "manual",
                )
            )
        capture_duration_s = float(capture.get("duration_s", 0.0))
        if capture_duration_s <= 0.0:
            capture_duration_s = sum(item.hold_s for item in pose_variations)

        manifest = Manifest(
            world=str(data.get("world", "baylands")).strip() or "baylands",
            output_dir=str(data.get("output_dir", "datasets/baylands_leader_varied")).strip()
            or "datasets/baylands_leader_varied",
            capture=CaptureConfig(
                uav_name=str(capture.get("uav_name", "dji0")).strip() or "dji0",
                hz=float(capture.get("hz", 1.0)),
                duration_s=capture_duration_s,
                save_overlay=_coerce_bool(capture.get("save_overlay", False)),
                save_metadata=_coerce_bool(capture.get("save_metadata", True)),
                save_negative_examples=_coerce_bool(capture.get("save_negative_examples", True)),
                make_obb=_coerce_bool(capture.get("make_obb", True)),
                obb_overlay=_coerce_bool(capture.get("obb_overlay", capture.get("save_overlay", False))),
                obb_overwrite=_coerce_bool(capture.get("obb_overwrite", False)),
            ),
            startup=StartupConfig(
                warmup_s=float(startup.get("warmup_s", 12.0)),
                stop_on_error=_coerce_bool(startup.get("stop_on_error", True)),
                scenario_retry_count=max(0, int(startup.get("scenario_retry_count", 2))),
                retry_delay_s=max(0.0, float(startup.get("retry_delay_s", 8.0))),
            ),
            gimbal=GimbalConfig(
                mode=str(gimbal.get("mode", "random_hold")).strip() or "random_hold",
                interval_s=float(gimbal.get("interval_s", 6.0)),
                hold_s=float(gimbal.get("hold_s", 7.0)),
                pan_center_deg=float(gimbal.get("pan_center_deg", 0.0)),
                pan_amplitude_deg=float(gimbal.get("pan_amplitude_deg", 18.0)),
                pan_min_deg=float(gimbal.get("pan_min_deg", -30.0)),
                pan_max_deg=float(gimbal.get("pan_max_deg", 30.0)),
                tilt_center_deg=float(gimbal.get("tilt_center_deg", -45.0)),
                tilt_amplitude_deg=float(gimbal.get("tilt_amplitude_deg", 10.0)),
                tilt_min_deg=float(gimbal.get("tilt_min_deg", -60.0)),
                tilt_max_deg=float(gimbal.get("tilt_max_deg", -25.0)),
            ),
            pose_variations=pose_variations,
            scenarios=scenarios,
        )

        self._validate_manifest(manifest)
        return manifest

    def _validate_manifest(self, manifest: Manifest) -> None:
        if manifest.world != "baylands":
            raise ValueError("Only world=baylands is supported by collect_leader_dataset right now")
        if manifest.capture.hz <= 0.0:
            raise ValueError("capture.hz must be > 0")
        if manifest.capture.duration_s <= 0.0:
            raise ValueError("capture.duration_s must be > 0 when no pose variation hold times are configured")
        if manifest.startup.warmup_s < 0.0:
            raise ValueError("startup.warmup_s must be >= 0")
        if manifest.startup.scenario_retry_count < 0:
            raise ValueError("startup.scenario_retry_count must be >= 0")
        if manifest.startup.retry_delay_s < 0.0:
            raise ValueError("startup.retry_delay_s must be >= 0")
        if manifest.gimbal.mode != "random_hold":
            raise ValueError("gimbal.mode must be random_hold")
        if manifest.gimbal.interval_s <= 0.0:
            raise ValueError("gimbal.interval_s must be > 0")
        if manifest.gimbal.hold_s <= 0.0:
            raise ValueError("gimbal.hold_s must be > 0")
        if manifest.gimbal.hold_s < manifest.gimbal.interval_s:
            raise ValueError("gimbal.hold_s must be >= gimbal.interval_s")
        if not manifest.scenarios:
            raise ValueError("manifest must contain at least one scenario")
        for scenario in manifest.scenarios:
            if not scenario.waypoint or not scenario.nav2_goals:
                raise ValueError(f"scenario '{scenario.name}' must define waypoint and nav2_goals")
            if scenario.uav_pattern not in ("manual", "scripted"):
                raise ValueError(f"scenario '{scenario.name}' uav_pattern must be manual or scripted")
            if scenario.uav_pattern == "scripted" and not manifest.pose_variations:
                raise ValueError(f"scenario '{scenario.name}' uses uav_pattern=scripted but pose_variations is empty")
        for variation in manifest.pose_variations:
            if variation.hold_s <= 0.0:
                raise ValueError(f"pose variation '{variation.name}' hold_s must be > 0")
            if variation.d_target_m <= 0.0:
                raise ValueError(f"pose variation '{variation.name}' d_target_m must be > 0")

    def _resolve_world(self) -> str:
        if self.world_arg and self.world_arg != self.manifest.world:
            raise ValueError(
                f"World mismatch: command requested '{self.world_arg}' but manifest uses '{self.manifest.world}'"
            )
        return self.world_arg or self.manifest.world

    def _select_scenarios(self, scenarios: list[Scenario]) -> list[Scenario]:
        requested = [item for item in self.scenario_filter_names if item]
        if not requested:
            return scenarios

        selected: list[Scenario] = []
        selected_names: set[str] = set()
        for name in requested:
            match = next(
                (
                    scenario
                    for scenario in scenarios
                    if scenario.name == name or scenario.nav2_goals == name or scenario.waypoint == name
                ),
                None,
            )
            if match is None:
                valid = ", ".join(scenario.name for scenario in scenarios)
                raise ValueError(f"Unknown route/scenario '{name}'. Valid scenarios: {valid}")
            if match.name not in selected_names:
                selected.append(match)
                selected_names.add(match.name)
        return selected

    def _resolve_output_dir(self) -> Path:
        raw = self.output_override or self.manifest.output_dir
        path = Path(os.path.expanduser(raw))
        if not path.is_absolute():
            path = (self.ws_root / path).resolve()
        return path

    def _handle_signal(self, signum: int, _frame) -> None:
        signal_name = signal.Signals(signum).name
        print(f"[collect_leader_dataset] Received {signal_name}; stopping after cleanup.", flush=True)
        self._stop_requested = True

    def _ensure_not_stopped(self) -> None:
        if self._stop_requested:
            raise InterruptedFailure("interrupted by signal")

    def _print_command(self, label: str, command: list[str]) -> None:
        print(f"[collect_leader_dataset] {label}: {_shell_join(command)}", flush=True)

    def _run(
        self,
        command: list[str],
        *,
        label: str,
        timeout_s: Optional[float] = None,
        allow_failure: bool = False,
        capture_output: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        self._print_command(label, command)
        if self.dry_run:
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

        completed = subprocess.run(
            command,
            cwd=self.ws_root,
            text=True,
            timeout=timeout_s,
            capture_output=capture_output,
            check=False,
        )
        if completed.returncode != 0 and not allow_failure:
            raise ScenarioFailure(
                f"{label} failed with exit code {completed.returncode}"
                + (f": {completed.stderr.strip()}" if completed.stderr else "")
            )
        return completed

    def _spawn(self, command: list[str], *, label: str) -> Optional[subprocess.Popen[str]]:
        self._print_command(label, command)
        if self.dry_run:
            return None
        return subprocess.Popen(
            command,
            cwd=self.ws_root,
            text=True,
            stdout=None,
            stderr=None,
            start_new_session=True,
        )

    def _stop_process(self, process: Optional[subprocess.Popen[str]], label: str) -> None:
        if process is None or process.poll() is not None:
            return

        print(f"[collect_leader_dataset] Stopping {label}", flush=True)
        for sig, wait_s in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 0.0)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                return
            if wait_s <= 0.0:
                break
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    return
                time.sleep(0.2)
        process.wait(timeout=1.0)

    def _sleep(self, duration_s: float, *, label: str) -> None:
        if duration_s <= 0.0:
            return
        print(f"[collect_leader_dataset] Sleeping {duration_s:.1f}s for {label}", flush=True)
        if self.dry_run:
            return
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._ensure_not_stopped()
            self._check_helper_processes()
            time.sleep(min(0.5, deadline - time.monotonic()))

    def _check_helper_processes(self) -> None:
        for process, label in ((self._active_capture, "capture_dataset"),):
            if process is not None and process.poll() is not None:
                raise ScenarioFailure(f"{label} exited early with code {process.returncode}")

    def _wait_for_topic(self, topic: str, timeout_s: float) -> None:
        print(f"[collect_leader_dataset] Waiting for topic: {topic}", flush=True)
        if self.dry_run:
            return

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._ensure_not_stopped()
            remaining = max(1.0, min(6.0, deadline - time.monotonic()))
            try:
                result = subprocess.run(
                    ["ros2", "topic", "echo", "--no-daemon", "--once", topic],
                    cwd=self.ws_root,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=remaining,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                continue
            if result.returncode == 0:
                return
        raise ScenarioFailure(f"Timed out waiting for topic {topic}")

    def _wait_for_param(self, node_name: str, param_name: str, timeout_s: float) -> None:
        print(f"[collect_leader_dataset] Waiting for param service: {node_name} {param_name}", flush=True)
        if self.dry_run:
            return

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._ensure_not_stopped()
            try:
                result = subprocess.run(
                    ["ros2", "param", "get", "--no-daemon", node_name, param_name],
                    cwd=self.ws_root,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=min(4.0, max(1.0, deadline - time.monotonic())),
                    check=False,
                )
            except subprocess.TimeoutExpired:
                time.sleep(0.5)
                continue
            if result.returncode == 0:
                return
            time.sleep(0.5)
        raise ScenarioFailure(f"Timed out waiting for param {param_name} on {node_name}")

    def _write_effective_params_file(self) -> Path:
        source_params_path = self.ws_root / "src" / "lrs_halmstad" / "config" / "run_follow_defaults.yaml"
        params_path = (
            source_params_path
            if source_params_path.is_file()
            else self.package_share / "config" / "run_follow_defaults.yaml"
        )
        with params_path.open("r", encoding="utf-8") as handle:
            params_data = yaml.safe_load(handle) or {}

        camera_tracker = _require_dict(params_data.setdefault("camera_tracker", {}), "camera_tracker")
        ros_params = _require_dict(camera_tracker.setdefault("ros__parameters", {}), "camera_tracker.ros__parameters")
        ros_params["gimbal_override_hold_s"] = float(self.manifest.gimbal.hold_s)

        temp_handle = tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix="leader_dataset_params_",
            suffix=".yaml",
            dir=self.state_dir,
            delete=False,
        )
        with temp_handle:
            yaml.safe_dump(params_data, temp_handle, sort_keys=False)

        temp_path = Path(temp_handle.name)
        self._temp_params_file = temp_path
        self._summary["params_file"] = str(temp_path)
        print(f"[collect_leader_dataset] Generated params file: {temp_path}", flush=True)
        return temp_path

    def _prepare_output(self) -> None:
        if self.dry_run:
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        manifest_snapshot = self.output_dir / f"collection_manifest_{timestamp}.yaml"
        shutil.copy2(self.manifest_path, manifest_snapshot)
        if self._temp_params_file is not None and self._temp_params_file.is_file():
            params_snapshot = self.output_dir / f"collection_params_{timestamp}.yaml"
            shutil.copy2(self._temp_params_file, params_snapshot)
            self._summary["params_snapshot"] = str(params_snapshot)
        self._summary_path = self.output_dir / f"collection_summary_{timestamp}.json"
        self._write_summary()

    def _write_summary(self) -> None:
        if self.dry_run or self._summary_path is None:
            return
        self._summary["updated_at"] = _utc_now_iso()
        with self._summary_path.open("w", encoding="utf-8") as handle:
            json.dump(self._summary, handle, indent=2, sort_keys=True)

    def _cleanup_session(self) -> None:
        if self.launcher in ("external", "manual"):
            print("[collect_leader_dataset] External launcher mode: leaving Nav2/follow stack running", flush=True)
            return

        if self.launcher == "tmux_1to1":
            self._run(
                ["./stop.sh", "tmux_1to1", self.world, f"session:={self.session_name}"],
                label="stop tmux_1to1",
                allow_failure=True,
            )
            return

        self._run(
            [
                "./run.sh",
                "nav2_tuning",
                "stop",
                f"world:={self.world}",
                f"session:={self.session_name}",
            ],
            label="stop nav2_tuning",
            allow_failure=True,
        )

    def _start_scenario_stack(self, scenario: Scenario) -> None:
        params_file = self._temp_params_file
        if params_file is None:
            raise RuntimeError("params file was not prepared")

        if self.launcher in ("external", "manual"):
            print(
                "[collect_leader_dataset] External launcher mode: "
                f"expecting route '{scenario.name}' to already be running "
                f"(waypoint={scenario.waypoint}, nav2_goals={scenario.nav2_goals})",
                flush=True,
            )
            return

        if self.launcher == "tmux_1to1":
            self._run(
                [
                    "./run.sh",
                    "tmux_1to1",
                    self.world,
                    "mode:=follow",
                    "tmux_attach:=false",
                    f"session:={self.session_name}",
                    f"params_file:={params_file}",
                    f"waypoint:={scenario.waypoint}",
                    f"nav2_goals:={scenario.nav2_goals}",
                ],
                label=f"start scenario {scenario.name}",
            )
            return

        self._run(
            [
                "./run.sh",
                "nav2_tuning",
                "start",
                f"world:={self.world}",
                "tmux_attach:=false",
                f"session:={self.session_name}",
                f"params_file:={params_file}",
                f"waypoint:={scenario.waypoint}",
                f"nav2_goals:={scenario.nav2_goals}",
                "lidar:=3d",
                "spawn_uav:=true",
                "start_camera_tracker:=true",
                "with_route_driver:=true",
                "start_optional_teleop:=false",
                "mute_ugv_camera:=true",
            ],
            label=f"start scenario {scenario.name}",
        )

    def _wait_for_scenario_ready(self) -> None:
        topics = [
            f"/{self.manifest.capture.uav_name}/camera0/camera_info",
            f"/{self.manifest.capture.uav_name}/camera0/image_raw",
            f"/{self.manifest.capture.uav_name}/camera0/actual/center_pose",
            "/a201_0000/amcl_pose_odom",
        ]
        for topic in topics:
            self._wait_for_topic(topic, timeout_s=60.0)
        self._wait_for_param("/follow_uav", "d_target", timeout_s=60.0)

    def _start_capture(self) -> Optional[subprocess.Popen[str]]:
        capture = self.manifest.capture
        save_metadata = capture.save_metadata or capture.make_obb
        return self._spawn(
            [
                "./run.sh",
                "capture_dataset",
                self.world,
                f"out:={self.output_dir}",
                f"uav_name:={capture.uav_name}",
                f"hz:={capture.hz}",
                f"save_overlay:={'true' if capture.save_overlay else 'false'}",
                f"save_metadata:={'true' if save_metadata else 'false'}",
                f"save_negative_examples:={'true' if capture.save_negative_examples else 'false'}",
            ],
            label="start capture_dataset",
        )

    def _run_obb_generation(self) -> None:
        capture = self.manifest.capture
        if not capture.make_obb:
            return
        command = [
            "./run.sh",
            "dataset_make_obb",
            str(self.output_dir),
        ]
        if capture.obb_overlay:
            command.append("--overlay")
        if capture.obb_overwrite:
            command.append("--overwrite")
        self._run(command, label="generate OBB labels")

    def _set_follow_param(self, name: str, value: float) -> None:
        self._run(
            ["ros2", "param", "set", "--no-daemon", "/follow_uav", name, f"{value:.6f}"],
            label=f"set {name}",
        )

    def _run_scripted_uav_pattern(self, scenario: Scenario) -> None:
        for variation in self.manifest.pose_variations:
            self._ensure_not_stopped()
            print(
                "[collect_leader_dataset] UAV pattern "
                f"{scenario.name}/{variation.name}: "
                f"heading_offset_deg={variation.leader_heading_offset_deg:.1f} "
                f"d_target_m={variation.d_target_m:.2f} hold_s={variation.hold_s:.1f}",
                flush=True,
            )
            if not self.dry_run:
                self._set_follow_param("leader_heading_offset_deg", variation.leader_heading_offset_deg)
                self._set_follow_param("d_target", variation.d_target_m)
            self._sleep(variation.hold_s, label=f"UAV pattern {scenario.name}/{variation.name}")

    def _poll_child_or_fail(self, process: Optional[subprocess.Popen[str]], label: str) -> None:
        if process is None or self.dry_run:
            return
        time.sleep(1.0)
        if process.poll() is not None:
            raise ScenarioFailure(f"{label} exited early with code {process.returncode}")

    def _run_scenario_attempt(self, scenario: Scenario, attempt_summary: dict[str, Any]) -> None:
        self._active_capture = None
        try:
            self._cleanup_session()
            self._start_scenario_stack(scenario)
            self._wait_for_scenario_ready()
            self._sleep(self.manifest.startup.warmup_s, label=f"warmup {scenario.name}")

            self._active_capture = self._start_capture()
            self._poll_child_or_fail(self._active_capture, "capture_dataset")

            if scenario.uav_pattern == "scripted":
                self._run_scripted_uav_pattern(scenario)
            else:
                self._sleep(self.manifest.capture.duration_s, label=f"capture window {scenario.name}")
            self._stop_process(self._active_capture, "capture_dataset")
            self._active_capture = None
            self._run_obb_generation()

            attempt_summary["status"] = "completed"
            attempt_summary["finished_at"] = _utc_now_iso()
        except InterruptedFailure:
            attempt_summary["status"] = "interrupted"
            attempt_summary["finished_at"] = _utc_now_iso()
            raise
        except Exception as exc:
            attempt_summary["status"] = "failed"
            attempt_summary["finished_at"] = _utc_now_iso()
            attempt_summary["error"] = str(exc)
            raise
        finally:
            self._stop_process(self._active_capture, "capture_dataset")
            self._active_capture = None
            self._cleanup_session()
            self._write_summary()

    def _run_scenario(self, scenario: Scenario) -> dict[str, Any]:
        started_at = _utc_now_iso()
        scenario_summary: dict[str, Any] = {
            "name": scenario.name,
            "waypoint": scenario.waypoint,
            "nav2_goals": scenario.nav2_goals,
            "uav_pattern": scenario.uav_pattern,
            "started_at": started_at,
            "status": "running",
            "attempts": [],
        }
        self._summary["scenarios"].append(scenario_summary)
        self._write_summary()

        max_attempts = 1 + self.manifest.startup.scenario_retry_count
        last_error: Optional[Exception] = None
        for attempt_index in range(1, max_attempts + 1):
            attempt_summary: dict[str, Any] = {
                "attempt": attempt_index,
                "started_at": _utc_now_iso(),
                "status": "running",
            }
            scenario_summary["attempts"].append(attempt_summary)
            scenario_summary["status"] = "running"
            self._write_summary()
            try:
                self._run_scenario_attempt(scenario, attempt_summary)
                scenario_summary["status"] = "completed"
                scenario_summary["finished_at"] = _utc_now_iso()
                scenario_summary["completed_attempt"] = attempt_index
                return scenario_summary
            except InterruptedFailure:
                scenario_summary["status"] = "interrupted"
                scenario_summary["finished_at"] = _utc_now_iso()
                raise
            except Exception as exc:
                last_error = exc
                scenario_summary["last_error"] = str(exc)
                if attempt_index >= max_attempts:
                    scenario_summary["status"] = "failed"
                    scenario_summary["finished_at"] = _utc_now_iso()
                    scenario_summary["error"] = str(exc)
                    raise
                scenario_summary["status"] = "retrying"
                scenario_summary["retry_delay_s"] = self.manifest.startup.retry_delay_s
                self._write_summary()
                print(
                    f"[collect_leader_dataset] Scenario '{scenario.name}' attempt {attempt_index}/{max_attempts} failed: {exc}",
                    flush=True,
                )
                print(
                    f"[collect_leader_dataset] Retrying scenario '{scenario.name}' in {self.manifest.startup.retry_delay_s:.1f}s",
                    flush=True,
                )
                self._sleep(
                    self.manifest.startup.retry_delay_s,
                    label=f"retry delay for {scenario.name}",
                )

        if last_error is not None:
            raise last_error
        raise ScenarioFailure(f"Scenario '{scenario.name}' ended without a result")

    def run(self) -> int:
        if self.world != "baylands":
            raise ValueError("collect_leader_dataset currently supports baylands only")

        self._write_effective_params_file()
        self._prepare_output()
        had_failures = False

        print(
            f"[collect_leader_dataset] Starting collection: world={self.world} "
            f"output_dir={self.output_dir} manifest={self.manifest_path} "
            f"capture_duration_s={self.manifest.capture.duration_s:.1f} "
            f"make_obb={self.manifest.capture.make_obb} launcher={self.launcher}",
            flush=True,
        )

        try:
            for scenario in self.selected_scenarios:
                self._ensure_not_stopped()
                print(f"[collect_leader_dataset] === Scenario: {scenario.name} ===", flush=True)
                try:
                    self._run_scenario(scenario)
                except InterruptedFailure:
                    raise
                except Exception as exc:
                    had_failures = True
                    print(f"[collect_leader_dataset] Scenario '{scenario.name}' failed: {exc}", flush=True)
                    if self.manifest.startup.stop_on_error:
                        raise
            self._summary["status"] = "completed_with_failures" if had_failures else "completed"
            self._summary["finished_at"] = _utc_now_iso()
            self._write_summary()
            return 0
        except InterruptedFailure:
            self._summary["status"] = "interrupted"
            self._summary["finished_at"] = _utc_now_iso()
            self._write_summary()
            return 130
        except Exception as exc:
            self._summary["status"] = "failed"
            self._summary["finished_at"] = _utc_now_iso()
            self._summary["error"] = str(exc)
            self._write_summary()
            raise
        finally:
            self._stop_process(self._active_capture, "capture_dataset")
            self._cleanup_session()
            if self._temp_params_file is not None:
                try:
                    self._temp_params_file.unlink(missing_ok=True)
                except Exception:
                    pass


def main(argv: Optional[list[str]] = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    collector = LeaderDatasetCollector(argv)
    rc = 0
    try:
        rc = collector.run()
    except Exception as exc:
        print(f"[collect_leader_dataset] ERROR: {exc}", file=sys.stderr, flush=True)
        rc = 1
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
