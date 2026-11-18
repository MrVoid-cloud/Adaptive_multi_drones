"""Re-exports for backend/fleet."""
from app.fleet.telemetry_models import DroneStatus, DroneNodeConfig, DroneTelemetry
from app.fleet.transport_adapters import (
    DroneTransportAdapter,
    SimulatedNodeAdapter,
    LocalWebSocketAdapter,
    MavlinkUdpAdapter,
    AdapterFactory
)
from app.fleet.drone_registry import DynamicDroneRegistry, fleet_registry

__all__ = [
    "DroneStatus", "DroneNodeConfig", "DroneTelemetry",
    "DroneTransportAdapter", "SimulatedNodeAdapter", "LocalWebSocketAdapter", "MavlinkUdpAdapter", "AdapterFactory",
    "DynamicDroneRegistry", "fleet_registry"
]
