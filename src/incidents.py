"""Incident logging utilities for swarm mission failsafe events."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Dict, List, Optional


class IncidentCode:
    """Canonical incident code list used across swarm failsafe checks."""

    COLLISION_MIN_DISTANCE = "C001"
    COLLISION_OBSTACLE_DISTANCE = "C002"
    FORMATION_ENVELOPE_VIOLATION = "F001"
    CAMERA_FRAME_TIMEOUT = "H001"
    CAMERA_CONFIDENCE_LOW = "H002"
    QR_READ_FAILURE = "H003"
    DRIFT_EXCESSIVE = "W001"
    LANDING_ZONE_UNSUITABLE = "L001"


@dataclass
class Incident:
    """Single failsafe event record."""

    timestamp: str
    code: str
    drone_id: Optional[int]
    action_taken: str
    swarm_state: Dict[str, Any]
    details: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "code": self.code,
            "drone_id": self.drone_id,
            "action_taken": self.action_taken,
            "swarm_state": self.swarm_state,
            "details": self.details,
        }


class IncidentLog:
    """Collects and saves incidents with lightweight summaries."""

    def __init__(self):
        self._incidents: List[Incident] = []

    @staticmethod
    def now_iso() -> str:
        return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    def record(
        self,
        code: str,
        action_taken: str,
        swarm_state: Dict[str, Any],
        drone_id: Optional[int] = None,
        details: str = "",
    ) -> Incident:
        incident = Incident(
            timestamp=self.now_iso(),
            code=code,
            drone_id=drone_id,
            action_taken=action_taken,
            swarm_state=swarm_state,
            details=details,
        )
        self._incidents.append(incident)
        return incident

    def all(self) -> List[Incident]:
        return list(self._incidents)

    def summary(self) -> Dict[str, Any]:
        by_code: Dict[str, int] = {}
        for incident in self._incidents:
            by_code[incident.code] = by_code.get(incident.code, 0) + 1
        return {
            "total_incidents": len(self._incidents),
            "by_code": by_code,
        }

    def save_json(self, path_template: str) -> str:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        resolved_path = path_template.format(timestamp=timestamp)
        output = Path(resolved_path)
        output.parent.mkdir(parents=True, exist_ok=True)

        payload = {
            "generated_at": self.now_iso(),
            "incidents": [item.to_dict() for item in self._incidents],
            "summary": self.summary(),
        }
        output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return str(output)
