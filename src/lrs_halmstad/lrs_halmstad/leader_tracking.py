from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_path

from lrs_halmstad.leader_types import Detection2D

try:
    from ultralytics import YOLO  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    YOLO = None


class UltralyticsTargetTracker:
    def __init__(
        self,
        *,
        models_root: str,
        yolo_weights: str,
        device: str,
        conf_threshold: float,
        iou_threshold: float,
        imgsz: int,
        target_class_id: int,
        target_class_name: str,
        tracker_enable: bool,
        tracker_name: str,
        tracker_yaml: str,
        tracker_persist: bool,
    ) -> None:
        self.models_root = os.path.expanduser(models_root)
        self.weights_path = self._resolve_weights_path(yolo_weights)
        self.device = device or "cpu"
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.imgsz = int(imgsz)
        self.target_class_id = int(target_class_id)
        self.target_class_name = str(target_class_name).strip()
        self.tracker_enable = bool(tracker_enable)
        self.tracker_name = str(tracker_name).strip().lower() or "botsort"
        self.tracker_yaml = str(tracker_yaml).strip()
        self.tracker_persist = bool(tracker_persist)

        self.tracker_config_path = self._resolve_tracker_yaml_path(self.tracker_yaml, self.tracker_name)
        self.model = None
        self.ready = False
        self.error: Optional[str] = None

        self.active_track_id: Optional[int] = None
        self.active_track_hits = 0
        self.active_track_first_seen_ns: Optional[int] = None
        self.active_track_last_seen_ns: Optional[int] = None

        self._load_model()

    @staticmethod
    def _workspace_root() -> Path:
        return Path(__file__).resolve().parents[3]

    @staticmethod
    def _package_share() -> Optional[Path]:
        try:
            return get_package_share_path("lrs_halmstad").resolve()
        except Exception:
            return None

    @staticmethod
    def _first_existing_path(*paths: Path) -> Optional[Path]:
        for path in paths:
            if path.is_file():
                return path
        return None

    def _resolve_weights_path(self, configured: str) -> str:
        configured = str(configured).strip()
        if not configured:
            return ""
        expanded = Path(os.path.expanduser(configured))
        if expanded.is_file():
            return str(expanded.resolve())
        if expanded.is_absolute():
            return str(expanded)
        candidate = Path(self.models_root) / expanded
        return str(candidate.resolve())

    def _resolve_tracker_yaml_path(self, configured: str, tracker_name: str) -> str:
        configured = str(configured).strip()
        if configured:
            expanded = Path(os.path.expanduser(configured))
            if expanded.is_file():
                return str(expanded.resolve())
            return str(expanded)
        filename = f"{tracker_name}.yaml"
        pkg_share = self._package_share()
        search_paths = []
        if pkg_share is not None:
            search_paths.append(pkg_share / "config" / "trackers" / filename)
        search_paths.append(self._workspace_root() / "src" / "lrs_halmstad" / "config" / "trackers" / filename)
        search_paths.append(
            self._workspace_root() / "install" / "lrs_halmstad" / "share" / "lrs_halmstad" / "config" / "trackers" / filename
        )
        match = self._first_existing_path(*search_paths)
        return str(match) if match is not None else filename

    def _load_model(self) -> None:
        if YOLO is None:
            self.error = "ultralytics_import_failed"
            return
        if not self.weights_path:
            self.error = "weights_unset"
            return
        if not Path(self.weights_path).is_file():
            self.error = f"weights_missing:{self.weights_path}"
            return
        try:
            self.model = YOLO(self.weights_path)
            try:
                self.model.fuse()
            except Exception:
                pass
            self.ready = True
            self.error = None
        except Exception as exc:
            self.model = None
            self.ready = False
            self.error = f"model_load_failed:{exc}"

    def _prediction_kwargs(self) -> dict:
        kwargs = {
            "verbose": False,
            "conf": self.conf_threshold,
            "iou": self.iou_threshold,
            "imgsz": self.imgsz,
            "device": self.device,
        }
        if self.target_class_id >= 0:
            kwargs["classes"] = [self.target_class_id]
        return kwargs

    def _candidate_ok(self, cls_id: Optional[int], cls_name: str) -> bool:
        if self.target_class_id >= 0 and cls_id != self.target_class_id:
            return False
        if self.target_class_name and cls_name.strip().lower() != self.target_class_name.lower():
            return False
        return True

    def _extract_detections(self, result) -> list[Detection2D]:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []
        ids = None
        names = getattr(result, "names", {}) or {}
        try:
            if getattr(boxes, "is_track", False) and getattr(boxes, "id", None) is not None:
                ids = boxes.id.tolist()
        except Exception:
            ids = None

        detections: list[Detection2D] = []
        for idx in range(len(boxes)):
            try:
                x1, y1, x2, y2 = [float(v) for v in boxes.xyxy[idx].tolist()]
                conf = float(boxes.conf[idx].item()) if getattr(boxes, "conf", None) is not None else 0.0
                cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
                track_id = None
                if ids is not None and idx < len(ids) and ids[idx] is not None:
                    track_id = int(ids[idx])
            except Exception:
                continue
            if not self._candidate_ok(cls_id, cls_name):
                continue
            detections.append(
                Detection2D(
                    u=0.5 * (x1 + x2),
                    v=0.5 * (y1 + y2),
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cls_id=cls_id,
                    cls_name=cls_name,
                    track_id=track_id,
                    track_state="tracked" if track_id is not None else "raw",
                )
            )
        return detections

    @staticmethod
    def _bbox_area(bbox: tuple[float, float, float, float]) -> float:
        x1, y1, x2, y2 = bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    def infer(self, image_bgr) -> list[Detection2D]:
        if not self.ready or self.model is None:
            return []
        kwargs = self._prediction_kwargs()
        try:
            if self.tracker_enable:
                results = self.model.track(
                    source=image_bgr,
                    persist=self.tracker_persist,
                    tracker=self.tracker_config_path,
                    **kwargs,
                )
            else:
                results = self.model.predict(source=image_bgr, **kwargs)
        except Exception as exc:
            self.error = f"infer_failed:{exc}"
            return []
        if not results:
            return []
        return self._extract_detections(results[0])

    def select(self, detections: list[Detection2D], *, stamp_ns: int, ok_debounce_frames: int) -> Optional[Detection2D]:
        if not detections:
            return None
        chosen = self._choose_detection(detections)
        if chosen is None:
            return None
        self._note_selection(chosen, stamp_ns=stamp_ns, ok_debounce_frames=ok_debounce_frames)
        return chosen

    def _choose_detection(self, detections: list[Detection2D]) -> Optional[Detection2D]:
        if self.tracker_enable:
            if self.active_track_id is not None:
                for det in detections:
                    if det.track_id == self.active_track_id:
                        return det
            tracked = [det for det in detections if det.track_id is not None]
            if tracked:
                return max(tracked, key=lambda det: (det.conf, self._bbox_area(det.bbox)))
        return max(detections, key=lambda det: (det.conf, self._bbox_area(det.bbox)))

    def _note_selection(self, det: Detection2D, *, stamp_ns: int, ok_debounce_frames: int) -> None:
        if not self.tracker_enable or det.track_id is None:
            det.track_state = "raw"
            det.track_hits = 0
            det.track_age_s = 0.0
            det.track_switched = False
            return

        switched = det.track_id != self.active_track_id
        if switched:
            self.active_track_id = det.track_id
            self.active_track_hits = 1
            self.active_track_first_seen_ns = stamp_ns
        else:
            self.active_track_hits += 1
        self.active_track_last_seen_ns = stamp_ns

        det.track_switched = switched
        det.track_hits = self.active_track_hits
        det.track_age_s = self.active_track_age_s(stamp_ns)
        det.track_state = "tracked" if self.active_track_hits >= max(1, ok_debounce_frames) else "reacquire"

    def reset_active_track(self) -> None:
        self.active_track_id = None
        self.active_track_hits = 0
        self.active_track_first_seen_ns = None
        self.active_track_last_seen_ns = None

    def active_track_age_s(self, stamp_ns: int) -> float:
        if self.active_track_first_seen_ns is None:
            return 0.0
        return max(0.0, (stamp_ns - self.active_track_first_seen_ns) * 1e-9)

    def track_summary(self, stamp_ns: int) -> tuple[str, str, str]:
        if self.active_track_id is None:
            return ("na", "0", "0.0")
        return (str(self.active_track_id), str(self.active_track_hits), f"{self.active_track_age_s(stamp_ns):.2f}")
