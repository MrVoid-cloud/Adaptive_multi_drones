"""
Mission Lifecycle & Session Epoch State Manager
Ensures deterministic synchronization across REST APIs, WebSockets,
the Physics Simulator, and GIS Grid Indexers.
Author: Principal Software Architect
"""

import time
import sys
import logging
from uuid import uuid4
from typing import Optional, Dict, Any, List

logger = logging.getLogger("MissionState")

class MissionState:
    """
    Centralized, thread-safe session epoch manager.
    Tracks the active mission UUID, boundaries, shared base, and cached results.
    """
    def __init__(self):
        self.mission_id: Optional[str] = None
        self.created_at: Optional[float] = None
        self.bounds: Optional[Dict[str, float]] = None
        self.vertices: Optional[List[Dict[str, float]]] = None
        self.home_base: Optional[Dict[str, float]] = None
        self.active_grid: Optional[Any] = None
        self.active_hybrid_builder: Optional[Any] = None
        self.active_targets: Optional[List[Any]] = None
        self.last_benchmark_results: Optional[Dict[str, Any]] = None

    def start_new_mission(
        self,
        bounds: Optional[Dict[str, float]] = None,
        vertices: Optional[List[Dict[str, float]]] = None,
        home_base: Optional[Dict[str, float]] = None
    ) -> str:
        """
        Generates a new mission epoch UUID and initializes metadata.
        """
        new_id = str(uuid4())
        self.mission_id = new_id
        self.created_at = time.time()
        self.bounds = bounds
        self.vertices = vertices
        self.home_base = home_base
        self.last_benchmark_results = None
        
        # Link mission_id into global simulator
        try:
            from app.simulation.simulator_engine import global_simulator
            global_simulator.mission_id = new_id
            if home_base and "lat" in home_base and "lon" in home_base:
                global_simulator.set_home_base(home_base["lat"], home_base["lon"])
        except Exception as e:
            logger.warning(f"[MissionState] Could not attach to global simulator: {e}")

        logger.info(f"[MissionState] New mission session started: {self.mission_id}")
        return self.mission_id

    async def hard_reset(self):
        """
        Immediately halts any active simulation loop, invalidates cached
        targets, clears the mission UUID, and broadcasts MISSION_CLEARED to all clients.
        """
        old_id = self.mission_id
        logger.info(f"[MissionState] Executing hard reset for mission session: {old_id}")
        
        # 1. Stop and reset physical simulation engine
        try:
            from app.simulation.simulator_engine import global_simulator
            await global_simulator.hard_reset()
            global_simulator.mission_id = None
        except Exception as e:
            logger.warning(f"[MissionState] Simulator hard_reset error: {e}")

        # 2. Reset central state variables
        self.mission_id = None
        self.created_at = None
        self.bounds = None
        self.vertices = None
        self.home_base = None
        self.active_grid = None
        self.active_hybrid_builder = None
        self.active_targets = None
        self.last_benchmark_results = None

        # 3. Clear module-level singletons if loaded (using sys.modules to prevent circular locks)
        grid_mod = sys.modules.get('app.api.v1.grid_routes')
        if grid_mod:
            setattr(grid_mod, '_ACTIVE_GRID', None)
            setattr(grid_mod, '_ACTIVE_HYBRID_BUILDER', None)
            setattr(grid_mod, '_ACTIVE_TARGETS', None)

        map_mod = sys.modules.get('app.api.v1.map_routes')
        if map_mod:
            setattr(map_mod, '_ACTIVE_MISSION_ROI', None)

        # 4. Broadcast MISSION_CLEARED event across WebSocket telemetry hub
        try:
            from app.api.ws.telemetry import telemetry_hub
            await telemetry_hub.broadcast({
                "type": "MISSION_CLEARED",
                "payload": {
                    "previous_mission_id": old_id,
                    "timestamp": time.time()
                }
            })
            logger.info("[MissionState] Broadcasted MISSION_CLEARED via telemetry hub.")
        except Exception as e:
            logger.warning(f"[MissionState] WebSocket broadcast warning: {e}")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mission_id": self.mission_id,
            "created_at": self.created_at,
            "has_active_session": self.mission_id is not None,
            "bounds": self.bounds,
            "home_base": self.home_base,
            "targets_count": len(self.active_targets) if self.active_targets else 0
        }

# Global Mission State Singleton
mission_state = MissionState()
