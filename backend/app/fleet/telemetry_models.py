"""
Fleet Telemetry & Dynamic Node Data Models
Phase 05: Dynamic Drone Registry & Local Addressing
Author: Principal Software Architect
"""

import time
from enum import Enum
from typing import List, Optional, Dict, Any
from urllib.parse import urlparse
from pydantic import BaseModel, Field, field_validator

class DroneStatus(str, Enum):
    READY = "READY"
    STANDBY = "STANDBY"
    SEARCHING = "SEARCHING"
    MAPPING = "MAPPING"
    RTL = "RTL"
    OFFLINE = "OFFLINE"
    SORTIE_FLYING = "SORTIE_FLYING"
    RETURNING_FOR_SWAP = "RETURNING_FOR_SWAP"
    BATTERY_SWAPPING = "BATTERY_SWAPPING"
    RELAUNCHING_SORTIE = "RELAUNCHING_SORTIE"
    MISSION_COMPLETED = "MISSION_COMPLETED"

class DroneNodeConfig(BaseModel):
    """Configuration payload for registering a drone dynamically."""
    drone_id: str = Field(..., min_length=2, max_length=32, description="Unique Drone Identifier (e.g. UAV-Alpha)")
    name: str = Field(default="", description="Human-readable label")
    endpoint_uri: str = Field(..., description="Local network address (ws://, wss://, udp://, mavlink://, sim://)")
    battery_capacity_wh: float = Field(default=120.0, gt=10.0, le=1000.0, description="Battery capacity in Watt-hours")
    cruise_speed_mps: float = Field(default=14.0, gt=2.0, le=40.0, description="Nominal cruise speed (m/s)")
    initial_lat: float = Field(default=33.3152, ge=-90.0, le=90.0)
    initial_lon: float = Field(default=44.3661, ge=-180.0, le=180.0)
    initial_alt_m: float = Field(default=60.0, ge=0.0, le=5000.0)

    @field_validator('endpoint_uri')
    def validate_endpoint_scheme(cls, v: str) -> str:
        parsed = urlparse(v)
        allowed_schemes = {"ws", "wss", "udp", "mavlink", "sim"}
        if parsed.scheme.lower() not in allowed_schemes:
            raise ValueError(f"Invalid URI scheme '{parsed.scheme}'. Allowed local transports: {allowed_schemes}")
        if not parsed.netloc and not parsed.path:
            raise ValueError("Endpoint URI must specify a valid host, port or simulation identifier.")
        return v

    def model_post_init(self, __context: Any) -> None:
        if not self.name:
            self.name = self.drone_id


class DroneTelemetry(BaseModel):
    """Real-time immutable telemetry reading emitted by a drone node."""
    drone_id: str
    endpoint_uri: str
    latitude: float
    longitude: float
    altitude_m: float
    speed_mps: float
    battery_pct: float
    status: DroneStatus
    last_heartbeat: float = Field(default_factory=time.time)
    assigned_cells: List[int] = Field(default_factory=list)
    current_cell_id: Optional[int] = None
    laser_distance_m: Optional[float] = None
    lasers_active: bool = True
    current_sortie: int = 1
    total_sorties: int = 1
    swap_remaining_sec: float = 0.0

    def is_alive(self, timeout_sec: float = 5.0) -> bool:
        """Determines if the node is actively transmitting telemetry."""
        return (time.time() - self.last_heartbeat) < timeout_sec
