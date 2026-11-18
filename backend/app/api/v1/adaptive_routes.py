from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import Literal, Optional, List, Dict, Any
from app.adaptive.event_monitor import AdaptiveTriggerType, AdaptiveEventMonitor, AdaptiveEvent
from app.adaptive.warm_start_replanner import WarmStartAdaptiveReplanner
from app.simulation.simulator_engine import global_simulator
from app.api.v1.grid_routes import _ACTIVE_GRID
from app.gis.grid import GeodesicGridDiscretizer

router = APIRouter(prefix="/adaptive", tags=["Closed-Loop Adaptive Engine & Warm-Start Re-Planner"])

# Global Adaptive Services Singletons
adaptive_monitor = AdaptiveEventMonitor(cooldown_seconds=3.0)
adaptive_replanner = WarmStartAdaptiveReplanner(simulator=global_simulator)

class InjectEventPayload(BaseModel):
    trigger_type: Literal["LOW_BATTERY_RTL", "FLEET_TOPOLOGY_CHANGED", "EARLY_SECTOR_COMPLETION", "MAP_DISCOVERY_UPDATE"]
    drone_id: Optional[str] = Field(default="UAV-01", description="ID of drone involved in the event")
    description: Optional[str] = Field(default="Injected dynamic anomaly event")

@router.post(
    "/inject-event",
    summary="Simulate / Inject Anomaly Event to Trigger Dynamic Warm-Start Re-Planning"
)
def inject_adaptive_event(payload: InjectEventPayload):
    """
    Manually triggers an adaptive event (e.g. UAV battery drop, early sector completion, or drone drop-out).
    Initiates State Freezing, Warm-Start Seeding, and In-Flight Route Hot-Swapping.
    """
    global _ACTIVE_GRID
    grid = _ACTIVE_GRID
    if grid is None:
        # Fallback test grid
        grid = GeodesicGridDiscretizer(
            lat_min=33.3000, lat_max=33.3150, lon_min=44.3500, lon_max=44.3700, manual_cell_size_m=120.0
        )

    # Ensure simulator has active mission state for re-planning demo
    if not global_simulator.routes:
        from app.fleet.telemetry_models import DroneTelemetry, DroneStatus
        mock_routes = {
            "UAV-01": [{"cell_id": grid.cells[0].cell_id, "row": 0, "col": 0, "lat": 33.301, "lon": 44.351, "alt_m": 60, "elevation_m": 10, "roughness": 1.0}],
            "UAV-02": [{"cell_id": grid.cells[1].cell_id, "row": 0, "col": 1, "lat": 33.302, "lon": 44.352, "alt_m": 60, "elevation_m": 10, "roughness": 1.0}]
        }
        mock_fleet = [
            DroneTelemetry(drone_id="UAV-01", endpoint_uri="sim://uav-01", latitude=33.301, longitude=44.351, altitude_m=60, speed_mps=14, battery_pct=14.0, status=DroneStatus.READY),
            DroneTelemetry(drone_id="UAV-02", endpoint_uri="sim://uav-02", latitude=33.302, longitude=44.352, altitude_m=60, speed_mps=14, battery_pct=85.0, status=DroneStatus.READY)
        ]
        global_simulator.load_mission({"routes": mock_routes}, mock_fleet)

    # 1. Update simulator drone state if low battery event
    if payload.trigger_type == "LOW_BATTERY_RTL" and payload.drone_id in global_simulator.drone_states:
        global_simulator.drone_states[payload.drone_id]["battery_pct"] = 14.0 # Force below 15% threshold
        global_simulator.drone_states[payload.drone_id]["status"] = "RTL"

    event = AdaptiveEvent(
        trigger_type=AdaptiveTriggerType(payload.trigger_type),
        drone_id=payload.drone_id,
        description=payload.description or "Manual event injection"
    )
    adaptive_monitor.record_event(event)

    # 2. Execute Warm-Start Re-planning
    replan_result = adaptive_replanner.execute_replan(
        event_dict=event.to_dict(),
        all_grid_cells=grid.cells
    )

    return {
        "status": "EVENT_INJECTED_AND_PROCESSED",
        "event": event.to_dict(),
        "replan_result": replan_result
    }

@router.get(
    "/logs",
    summary="Get Adaptive Event Audit Log & Re-Planning History"
)
def get_adaptive_logs():
    """Returns chronological audit trail of all anomalies detected, triggers fired, and latency metrics."""
    return {
        "audit_events_count": len(adaptive_monitor.audit_log),
        "replan_executions_count": len(adaptive_replanner.replan_history),
        "events": adaptive_monitor.audit_log,
        "replan_history": adaptive_replanner.replan_history
    }
