from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from app.fleet.telemetry_models import DroneNodeConfig, DroneTelemetry
from app.fleet.drone_registry import fleet_registry

router = APIRouter(prefix="/fleet", tags=["Fleet Dynamic Registry & Direct Links"])

@router.post(
    "/add",
    response_model=DroneTelemetry,
    status_code=status.HTTP_201_CREATED,
    summary="Manually Register New Drone Node (Zero Hardcoding)"
)
def add_drone_node(payload: DroneNodeConfig):
    """
    Registers a drone dynamically via its local URI endpoint (ws://, udp://, sim://).
    Initializes transport adapter and verifies handshake without server restart.
    """
    try:
        telemetry = fleet_registry.add_drone(payload)
        return telemetry
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Failed to connect: {str(e)}")

@router.delete(
    "/{drone_id}",
    summary="Remove / De-register Drone from Active Fleet"
)
def remove_drone_node(drone_id: str):
    """Disconnects and removes a drone from the active fleet."""
    success = fleet_registry.remove_drone(drone_id)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Drone '{drone_id}' not found in registry."
        )
    return {"status": "DELETED", "drone_id": drone_id, "message": "Drone unregistered successfully."}

@router.put(
    "/reorder",
    summary="Reorder Drone Priority Indexing"
)
def reorder_fleet_nodes(new_order: List[str]):
    """Reorders drone priority list for optimization algorithms."""
    try:
        updated_order = fleet_registry.reorder_fleet(new_order)
        return {"status": "REORDERED", "fleet_order": updated_order}
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))

@router.get(
    "",
    summary="Get All Registered Drones & Live Telemetry"
)
def get_all_fleet_nodes():
    """Returns immutable telemetry snapshot of all active fleet nodes."""
    configs = fleet_registry.get_ordered_configs()
    snapshot = fleet_registry.get_immutable_snapshot()
    return {
        "count": len(configs),
        "fleet": [
            {
                "config": config.model_dump(),
                "telemetry": telem.model_dump()
            }
            for config, telem in zip(configs, snapshot)
        ]
    }


class HomeBasePayload(BaseModel):
    lat: float = Field(..., description="Latitude of the shared base station")
    lon: float = Field(..., description="Longitude of the shared base station")
    alt_m: float = Field(default=0.0, description="Altitude of base station")


@router.post(
    "/home-base",
    summary="Update Shared Home Base for Active Fleet"
)
def update_shared_home_base(payload: HomeBasePayload):
    """
    Updates the shared home base location for all registered drones and the simulator.
    Ensures drones take off from the newly selected region.
    """
    from app.simulation.simulator_engine import global_simulator
    from app.core.mission_state import mission_state
    
    global_simulator.set_home_base(payload.lat, payload.lon)
    mission_state.home_base = {"lat": payload.lat, "lon": payload.lon, "alt_m": payload.alt_m}
    
    # Update fleet registry drones initial locations
    configs = fleet_registry.get_ordered_configs()
    for cfg in configs:
        cfg.initial_lat = payload.lat
        cfg.initial_lon = payload.lon
        cfg.initial_alt_m = payload.alt_m
        
    return {
        "status": "HOME_BASE_UPDATED",
        "home_base": {"lat": payload.lat, "lon": payload.lon, "alt_m": payload.alt_m}
    }
