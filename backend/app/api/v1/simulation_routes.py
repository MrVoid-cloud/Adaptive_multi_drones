"""
Unified Mission Deploy & Real-Time Physics Simulation Control Routes
Phase 07 & 10: Atomic Mission Deployment, Staggered Launch & WebSocket Pipeline
Author: Principal Software Architect
"""

import asyncio
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import Literal, Optional, Dict, Any, List
from app.simulation.simulator_engine import global_simulator
from app.fleet.drone_registry import fleet_registry
from app.fleet.telemetry_models import DroneNodeConfig, DroneStatus
from app.core.mission_state import mission_state

router = APIRouter(prefix="/sim", tags=["Real-Time Physics Simulation Control"])

class SimControlPayload(BaseModel):
    action: Literal["START", "PAUSE", "RESUME", "RESET", "STOP", "SET_SPEED"]
    time_scale: Optional[float] = Field(default=1.0, ge=0.25, le=10.0)

class DeployAndStartPayload(BaseModel):
    mission_id: Optional[str] = Field(default=None, description="Active session epoch ID (UUID)")
    algorithm_name: str = Field(..., description="Name of the approved optimization engine (e.g. Hybrid GA-PSO)")
    routes: Dict[str, List[Dict[str, Any]]] = Field(..., description="Per-drone ordered waypoint lists")
    time_scale: Optional[float] = Field(default=1.0, ge=0.25, le=10.0)

@router.post(
    "/deploy-and-start",
    summary="Atomically Deploy Plan, Initialize Fleet at Shared Home Base, and Launch Physics Loop"
)
async def deploy_and_start_simulation(payload: DeployAndStartPayload):
    """
    Unified Atomic Mission Launch Pipeline:
    1. Validates mission session consistency against current active epoch.
    2. Stops and resets any prior running mission.
    3. Validates route non-emptiness.
    4. Resolves Shared Home Base geodetic coordinates (relocated to new search area).
    5. Resets all fleet drones to Shared Home Base with 100% battery and STANDBY state.
    6. Injects approved routes and staggered launch schedules into the physics simulator.
    7. Initiates the non-blocking background asyncio physics loop broadcasting live telemetry.
    """
    # 0. Validate Session Epoch Consistency
    current_mission_id = mission_state.mission_id
    if payload.mission_id and current_mission_id and payload.mission_id != current_mission_id:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"Stale Mission ID '{payload.mission_id}'. Active session is '{current_mission_id}'."
        )

    # 1. Stop and reset any ongoing simulation
    await global_simulator.stop_and_reset()

    # 2. Validate non-empty routes
    if not payload.routes or not any(len(wps) > 0 for wps in payload.routes.values()):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot deploy mission: No waypoints provided in routes for any drone."
        )

    # 3. Resolve Shared Home Base from mission_state, active grid, or waypoint centroid
    if mission_state.home_base and "lat" in mission_state.home_base and "lon" in mission_state.home_base:
        home_lat = mission_state.home_base["lat"]
        home_lon = mission_state.home_base["lon"]
    else:
        from app.api.v1 import grid_routes
        grid = getattr(grid_routes, "_ACTIVE_GRID", None)
        if grid and grid.active_cell_count > 0:
            home_lat = grid.mean_lat
            home_lon = (grid.lon_min + grid.lon_max) / 2.0
        else:
            all_lats = [float(wp["lat"]) for wps in payload.routes.values() for wp in wps if "lat" in wp]
            all_lons = [float(wp["lon"]) for wps in payload.routes.values() for wp in wps if "lon" in wp]
            home_lat = sum(all_lats) / len(all_lats) if all_lats else 33.3075
            home_lon = sum(all_lons) / len(all_lons) if all_lons else 44.3600

    # 4. Synchronize Fleet Registry with payload routes
    fleet_registry.clear_all()
    drone_ids = list(payload.routes.keys())
    for idx, d_id in enumerate(drone_ids):
        fleet_registry.add_drone(DroneNodeConfig(
            drone_id=d_id,
            endpoint_uri=f"sim://{d_id.lower()}",
            cruise_speed_mps=14.0 + (idx * 0.4),
            battery_capacity_wh=120.0,
            initial_lat=home_lat,
            initial_lon=home_lon,
            initial_alt_m=0.0
        ))
    fleet = fleet_registry.get_immutable_snapshot()

    # 5. Inject Routes into Simulator Kernel
    effective_mission_id = current_mission_id or payload.mission_id
    global_simulator.load_mission_routes(
        algorithm_name=payload.algorithm_name,
        routes=payload.routes,
        fleet=fleet,
        home_base=(home_lat, home_lon),
        mission_id=effective_mission_id
    )

    if payload.time_scale:
        global_simulator.set_time_scale(payload.time_scale)

    # 6. Start Physics Loop Task
    global_simulator.start()

    return {
        "status": "SIMULATION_STARTED",
        "mission_id": effective_mission_id,
        "algorithm_name": payload.algorithm_name,
        "drones_count": len(global_simulator.drone_states),
        "total_targets": global_simulator.total_cells_to_scan,
        "time_scale": global_simulator.time_scale,
        "home_base": {"lat": round(home_lat, 6), "lon": round(home_lon, 6)},
        "message": f"Simulation successfully deployed and launched with {payload.algorithm_name}."
    }

@router.post(
    "/launch",
    summary="Legacy Launch Endpoint Alias"
)
async def launch_mission(payload: DeployAndStartPayload):
    """Alias for /deploy-and-start to maintain full backwards compatibility."""
    return await deploy_and_start_simulation(payload)

@router.post(
    "/control",
    summary="Control Simulation Kernel Execution State"
)
async def control_simulation(payload: SimControlPayload):
    """
    Direct operational control of the physics loop: START, PAUSE, RESUME, RESET, STOP, or adjust speed multiplier.
    """
    action = payload.action.upper()
    if action in ["START", "RESUME"]:
        global_simulator.start()
    elif action == "PAUSE":
        global_simulator.pause()
    elif action in ["RESET", "STOP"]:
        await global_simulator.stop_and_reset()
    elif action == "SET_SPEED":
        if payload.time_scale is not None:
            global_simulator.set_time_scale(payload.time_scale)

    return {
        "status": "COMMAND_PROCESSED",
        "action": action,
        "current_state": global_simulator.state,
        "time_scale": global_simulator.time_scale
    }

@router.get(
    "/status",
    summary="Get Real-Time Simulation Status & Metrics"
)
def get_simulation_status():
    """Returns real-time mission execution state, elapsed time, and completion progress."""
    return {
        "state": global_simulator.state,
        "mission_elapsed_sec": round(global_simulator.mission_elapsed_sec, 1),
        "time_scale": global_simulator.time_scale,
        "scanned_cells": global_simulator.scanned_cells_count,
        "total_cells": global_simulator.total_cells_to_scan,
        "progress_pct": round(
            (global_simulator.scanned_cells_count / max(1, global_simulator.total_cells_to_scan)) * 100.0, 1
        ),
        "drones": global_simulator.drone_states,
        "home_base": global_simulator.home_base
    }

@router.post(
    "/step",
    summary="Trigger a Single Synchronous Simulation Step (Deterministic Testing)"
)
def step_simulation():
    """Advances simulation by one dt and returns generated telemetry frame."""
    if global_simulator.state != "RUNNING":
        global_simulator.state = "RUNNING"
    frame = global_simulator.step()
    return frame
