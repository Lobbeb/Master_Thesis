from __future__ import annotations

from typing import Optional

from std_msgs.msg import String

from lrs_halmstad.perception.detection_protocol import Detection2D


def parse_status_line(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in str(line).split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value
    return fields


def build_detection_status_line(
    *,
    state: str,
    reason: str,
    task: str,
    det: Optional[Detection2D],
) -> str:
    perception = "none" if det is None else (det.source or "external")
    conf = -1.0 if det is None else float(det.conf)
    cls_id = "none" if det is None or det.cls_id is None else str(int(det.cls_id))
    cls_name = "" if det is None else str(det.cls_name or "")
    track_id = "none" if det is None or det.track_id is None else str(int(det.track_id))
    track_hits = 0 if det is None else int(det.track_hits)
    track_age_s = 0.0 if det is None else float(det.track_age_s)
    track_state = "na" if det is None else str(det.track_state or "na")
    track_switched = False if det is None else bool(det.track_switched)
    return (
        f"task={str(task).strip() or 'detection'} "
        f"state={str(state).strip() or 'UNKNOWN'} "
        f"reason={str(reason).strip() or 'none'} "
        f"perception={perception} "
        f"conf={conf:.3f} "
        f"cls_id={cls_id} "
        f"cls_name={cls_name} "
        f"track_id={track_id} "
        f"track_hits={track_hits} "
        f"track_age_s={track_age_s:.2f} "
        f"track_state={track_state} "
        f"track_switched={'true' if track_switched else 'false'}"
    )


def overlay_lines_from_status(line: str) -> list[str]:
    fields = parse_status_line(line)
    if not fields:
        return []
    age_raw = fields.get("track_age_s", "0.0")
    try:
        age_text = f"{float(age_raw):.1f}s"
    except Exception:
        age_text = f"{age_raw}s"
    return [
        f"src: {fields.get('perception', 'none')}",
        f"id: {fields.get('track_id', 'none')}",
        f"hits: {fields.get('track_hits', '0')}",
        f"age: {age_text}",
    ]


class DetectionStatusPublisher:
    def __init__(self, node, topic: str):
        self._pub = node.create_publisher(String, str(topic).strip(), 10)

    def publish(self, *, state: str, reason: str, task: str, det: Optional[Detection2D]) -> None:
        msg = String()
        msg.data = build_detection_status_line(state=state, reason=reason, task=task, det=det)
        self._pub.publish(msg)
