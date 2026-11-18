"""
Dynamic Drone Registry & Fleet Concurrency Manager
Phase 05: Dynamic Drone Registry & Local Addressing
Author: Principal Software Architect
"""

import threading
from typing import Dict, List, Optional, Tuple
from app.fleet.telemetry_models import DroneNodeConfig, DroneTelemetry, DroneStatus
from app.fleet.transport_adapters import DroneTransportAdapter, AdapterFactory

class DynamicDroneRegistry:
    """
    Central Thread-Safe Service Registry for the UAV fleet.
    Enforces strict Zero Hardcoded Binding. Drones can be registered, reordered,
    or deleted in-flight without interrupting the server process.
    Uses re-entrant thread-safe locking to support concurrent solver threads and async API handlers.
    """
    def __init__(self):
        self._lock = threading.RLock()
        # Storage: drone_id -> (DroneNodeConfig, DroneTransportAdapter)
        self._registry: Dict[str, Tuple[DroneNodeConfig, DroneTransportAdapter]] = {}
        self._order: List[str] = []

    def add_drone(self, config: DroneNodeConfig) -> DroneTelemetry:
        """Dynamically instantiates adapter, connects, and registers drone in fleet."""
        with self._lock:
            if config.drone_id in self._registry:
                raise ValueError(f"Drone with ID '{config.drone_id}' is already registered.")

            adapter = AdapterFactory.create_adapter(config)
            adapter.connect_sync()

            self._registry[config.drone_id] = (config, adapter)
            if config.drone_id not in self._order:
                self._order.append(config.drone_id)

            return adapter.fetch_telemetry_sync()

    def remove_drone(self, drone_id: str) -> bool:
        """Safely disconnects and unregisters a drone."""
        with self._lock:
            if drone_id not in self._registry:
                return False

            _, adapter = self._registry[drone_id]
            try:
                adapter.disconnect_sync()
            except Exception:
                pass

            del self._registry[drone_id]
            if drone_id in self._order:
                self._order.remove(drone_id)

            return True

    def reorder_fleet(self, new_order: List[str]) -> List[str]:
        """Re-prioritizes drone indexing for optimization engines."""
        with self._lock:
            if set(new_order) != set(self._registry.keys()):
                raise ValueError("Reorder list must contain all registered active drone IDs exactly once.")
            self._order = list(new_order)
            return self._order

    def get_ordered_configs(self) -> List[DroneNodeConfig]:
        """Returns registered configs in operator-defined priority order."""
        with self._lock:
            return [self._registry[d_id][0] for d_id in self._order if d_id in self._registry]

    def get_immutable_snapshot(self) -> List[DroneTelemetry]:
        """
        Creates an immutable, deep-copied telemetry snapshot for GA/PSO solvers.
        Prevents race conditions where in-flight fleet edits corrupt optimization matrices.
        """
        with self._lock:
            snapshot: List[DroneTelemetry] = []
            for d_id in self._order:
                if d_id in self._registry:
                    _, adapter = self._registry[d_id]
                    telemetry = adapter.fetch_telemetry_sync()
                    snapshot.append(telemetry.model_copy(deep=True))
            return snapshot

    def check_heartbeats(self, timeout_sec: float = 5.0) -> List[str]:
        """Watchdog routine: Detects nodes that have stopped responding."""
        offline_nodes = []
        with self._lock:
            for d_id in self._order:
                if d_id in self._registry:
                    _, adapter = self._registry[d_id]
                    telemetry = adapter.fetch_telemetry_sync()
                    if not telemetry.is_alive(timeout_sec):
                        telemetry.status = DroneStatus.OFFLINE
                        offline_nodes.append(d_id)
        return offline_nodes

    def clear_all(self):
        """Clears all registered drones and disconnects sockets."""
        with self._lock:
            for _, adapter in self._registry.values():
                try:
                    adapter.disconnect_sync()
                except Exception:
                    pass
            self._registry.clear()
            self._order.clear()

# Singleton registry instance
fleet_registry = DynamicDroneRegistry()
