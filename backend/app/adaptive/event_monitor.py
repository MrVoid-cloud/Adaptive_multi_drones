"""
Closed-Loop Adaptive Event Monitor & Anomaly Detector
Phase 08: Real-Time Event-Driven Anomaly Detection & Dynamic Warm-Start
Author: Principal Software Architect
"""

import time
from enum import Enum
from typing import Dict, Any, List, Optional, Tuple

class AdaptiveTriggerType(str, Enum):
    LOW_BATTERY_RTL = "LOW_BATTERY_RTL"
    FLEET_TOPOLOGY_CHANGED = "FLEET_TOPOLOGY_CHANGED"
    EARLY_SECTOR_COMPLETION = "EARLY_SECTOR_COMPLETION"
    MAP_DISCOVERY_UPDATE = "MAP_DISCOVERY_UPDATE"

class AdaptiveEvent:
    """Encapsulates a triggered re-planning event with diagnostic telemetry."""
    def __init__(
        self,
        trigger_type: AdaptiveTriggerType,
        drone_id: Optional[str] = None,
        description: str = "",
        metadata: Optional[Dict[str, Any]] = None
    ):
        self.trigger_type = trigger_type
        self.drone_id = drone_id
        self.description = description
        self.metadata = metadata or {}
        self.timestamp = time.time()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "trigger_type": self.trigger_type.value,
            "drone_id": self.drone_id,
            "description": self.description,
            "timestamp": round(self.timestamp, 3),
            "metadata": self.metadata
        }


class AdaptiveEventMonitor:
    """
    Monitors simulation ticks and telemetry to detect any of the 4 triggers.
    Includes debouncing & cooldown timers to prevent re-planning thrashing.
    """
    def __init__(self, cooldown_seconds: float = 3.0):
        self.cooldown_sec = cooldown_seconds
        self.last_trigger_times: Dict[AdaptiveTriggerType, float] = {}
        self.audit_log: List[Dict[str, Any]] = []

    def is_cooling_down(self, trigger_type: AdaptiveTriggerType) -> bool:
        last_t = self.last_trigger_times.get(trigger_type, 0.0)
        return (time.time() - last_t) < self.cooldown_sec

    def record_event(self, event: AdaptiveEvent):
        self.last_trigger_times[event.trigger_type] = event.timestamp
        self.audit_log.append(event.to_dict())

    def inspect_simulation_state(
        self,
        drone_states: Dict[str, Dict[str, Any]],
        routes: Dict[str, List[Dict[str, Any]]],
        waypoint_indices: Dict[str, int]
    ) -> Optional[AdaptiveEvent]:
        """
        Evaluates simulation kernel telemetry frame.
        Returns the highest-priority AdaptiveEvent if a trigger condition is met, else None.
        """
        # 1. TRIGGER 1: LOW_BATTERY_RTL (Emergency Priority)
        if not self.is_cooling_down(AdaptiveTriggerType.LOW_BATTERY_RTL):
            for d_id, state in drone_states.items():
                if state.get("status") == "RTL":
                    # Check if drone still has uncompleted waypoints assigned to it
                    current_idx = waypoint_indices.get(d_id, 0)
                    total_wps = len(routes.get(d_id, []))
                    if current_idx < total_wps:
                        event = AdaptiveEvent(
                            trigger_type=AdaptiveTriggerType.LOW_BATTERY_RTL,
                            drone_id=d_id,
                            description=f"UAV {d_id} triggered RTL with {total_wps - current_idx} unvisited cells remaining.",
                            metadata={
                                "battery_pct": state.get("battery_pct"),
                                "unvisited_cells": total_wps - current_idx
                            }
                        )
                        self.record_event(event)
                        return event

        # 2. TRIGGER 3: EARLY_SECTOR_COMPLETION (Load Rebalancing)
        if not self.is_cooling_down(AdaptiveTriggerType.EARLY_SECTOR_COMPLETION):
            idle_drones = []
            busy_drones = []
            for d_id, state in drone_states.items():
                current_idx = waypoint_indices.get(d_id, 0)
                total_wps = len(routes.get(d_id, []))
                
                # Drone finished its cells, battery is healthy (> 35%), but not in RTL
                if current_idx >= total_wps and state.get("battery_pct", 0) > 35.0 and state.get("status") != "RTL":
                    idle_drones.append(d_id)
                elif current_idx < total_wps:
                    busy_drones.append((d_id, total_wps - current_idx))

            # If at least one drone is idle while another drone has >= 4 unvisited cells
            if idle_drones and busy_drones:
                max_busy_id, remaining = max(busy_drones, key=lambda x: x[1])
                if remaining >= 4:
                    event = AdaptiveEvent(
                        trigger_type=AdaptiveTriggerType.EARLY_SECTOR_COMPLETION,
                        drone_id=idle_drones[0],
                        description=f"UAV {idle_drones[0]} completed sector early. Assisting {max_busy_id} ({remaining} cells left).",
                        metadata={
                            "idle_drone": idle_drones[0],
                            "donor_drone": max_busy_id,
                            "remaining_cells": remaining
                        }
                    )
                    self.record_event(event)
                    return event

        return None
