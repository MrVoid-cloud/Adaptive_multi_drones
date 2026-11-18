"""
Time-Stepped Multi-Drone Physics Simulator & Live LiDAR Scanning Engine
Phase 02: Next-Gen Multi-Sortie Simulation, Battery-Swap Lifecycle & Telemetry Streaming
Author: Principal Software Architect
"""

import time
import math
import asyncio
import logging
import numpy as np
from typing import Dict, Any, List, Optional, Set, Tuple
from app.simulation.kinematics import GeodesicKinematics
from app.physics.flight_physics import UAVPhysicsEngine, LiDARTerrainProcessor
from app.fleet.telemetry_models import DroneTelemetry, DroneStatus
from app.api.ws.telemetry import telemetry_hub

logger = logging.getLogger("SimulatorEngine")
logging.basicConfig(level=logging.INFO)

BATTERY_SWAP_DURATION_SEC: float = 120.0


class MultiDronePhysicsSimulator:
    """
    Time-stepped asynchronous simulation kernel.
    Simulates multi-UAV kinematic trajectory execution, staggered launch,
    terrain roughness friction, battery discharge, multi-sortie return & battery-swap cycles,
    and live airborne LiDAR scanning in real time.
    Streams high-frequency telemetry via WebSockets to connected Mission Control dashboards.

    Key Architectural Capabilities:
    1. Multi-Sortie Breakpoint & Battery-Swap Lifecycle:
       - Automatically tracks SORTIE_FLYING -> RETURNING_FOR_SWAP -> BATTERY_SWAPPING -> RELAUNCHING_SORTIE.
       - Manages a 120-second countdown during battery swap with smooth recharging to 100%.
       - Seamlessly advances to the next sortie (Sortie #N+1) until 100% terrain coverage is reached.
    2. N-Agnostic Fleet Scalability (N >= 1):
       - Directly supports Single-UAV (N=1) up to large swarms (N >= 6).
       - Surplus standby drones (N_fleet > M_targets) hold safely at SharedHomeBase without blocking completion.
    3. Spatial Memory & Opportunistic In-Flight Swath Capture:
       - Senses all targets within sensor_radius_m as drones fly along their path.
       - Opportunistically captured targets are immediately marked as searched and dynamically skipped
         from upcoming waypoints of any drone, completely eliminating redundant re-scanning and fly-overs.
    4. Resilient Session Epoch Synchronization & Non-blocking asyncio control.
    """

    def __init__(
        self,
        dt_seconds: float = 0.25,
        time_scale: float = 1.0,
        sensor_radius_m: float = 45.0
    ):
        self.dt = dt_seconds
        self.time_scale = time_scale
        self.sensor_radius_m = sensor_radius_m
        self.state: str = "IDLE"  # IDLE, RUNNING, PAUSED, COMPLETED
        self._sim_task: Optional[asyncio.Task] = None

        self.physics = UAVPhysicsEngine()
        self.routes: Dict[str, List[Dict[str, Any]]] = {}
        self.waypoint_indices: Dict[str, int] = {}
        self.drone_states: Dict[str, Dict[str, Any]] = {}
        self.home_base: Tuple[float, float] = (33.3075, 44.3600)
        self.algorithm_name: str = "Hybrid GA-PSO"
        self.mission_id: Optional[str] = None

        # Mission targets registry for global spatial memory & opportunistic capture
        self.mission_targets_registry: Dict[str, Dict[str, Any]] = {}
        self.unsearched_target_ids: Set[str] = set()

        self.total_cells_to_scan: int = 0
        self.scanned_cells_count: int = 0
        self.mission_elapsed_sec: float = 0.0
        self.scanned_target_ids: Set[str] = set()

    def set_home_base(self, lat: float, lon: float):
        """Updates shared home base coordinates for the fleet."""
        self.home_base = (lat, lon)
        logger.info(f"[Simulator] Shared home base relocated to ({lat:.6f}, {lon:.6f}).")

    def load_mission_routes(
        self,
        algorithm_name: str,
        routes: Dict[str, List[Dict[str, Any]]],
        fleet: List[DroneTelemetry],
        home_base: Optional[Tuple[float, float]] = None,
        mission_id: Optional[str] = None
    ):
        """
        Loads approved routes into the physics kernel and sets up initial fleet states at Home Base.
        Indexes all terrain targets in global spatial memory for opportunistic in-flight scanning.
        """
        self.algorithm_name = algorithm_name
        self.routes = routes
        if home_base:
            self.home_base = home_base
        if mission_id:
            self.mission_id = mission_id

        self.waypoint_indices = {d_id: 0 for d_id in self.routes.keys()}
        self.drone_states.clear()
        self.scanned_target_ids.clear()
        self.mission_targets_registry.clear()
        self.unsearched_target_ids.clear()

        # Ingest only actual target waypoints (excluding base return/relaunch markers)
        for d_id, wps in self.routes.items():
            for wp in wps:
                wp_type = str(wp.get("waypoint_type", "TARGET"))
                is_base = wp.get("metadata", {}).get("is_base", False)
                if wp_type in ("SORTIE_RETURN_BASE", "SORTIE_RELAUNCH_BASE") or is_base:
                    continue

                t_id = str(wp.get("target_id", wp.get("cell_id", "")))
                if t_id and t_id not in self.mission_targets_registry:
                    self.mission_targets_registry[t_id] = {
                        "target_id": t_id,
                        "cell_id": t_id,
                        "lat": float(wp["lat"]),
                        "lon": float(wp["lon"]),
                        "alt_m": float(wp.get("safe_alt_m", wp.get("alt_m", 60.0))),
                        "elevation_m": float(wp.get("elevation_m", 0.0)),
                        "row": wp.get("row", 0),
                        "col": wp.get("col", 0),
                        "node_type": wp.get("node_type", "OPEN_TERRAIN_CELL")
                    }

        self.unsearched_target_ids = set(self.mission_targets_registry.keys())
        self.total_cells_to_scan = len(self.unsearched_target_ids)
        self.scanned_cells_count = 0
        self.mission_elapsed_sec = 0.0

        for idx, drone in enumerate(fleet):
            d_id = drone.drone_id
            stagger_delay = idx * 3.5

            initial_heading = 0.0
            drone_route = self.routes.get(d_id, [])
            if drone_route and len(drone_route) > 0:
                first_wp = drone_route[0]
                try:
                    bearing_rad = GeodesicKinematics.calculate_bearing(
                        self.home_base[0], self.home_base[1], first_wp["lat"], first_wp["lon"]
                    )
                    initial_heading = round(math.degrees(bearing_rad) % 360.0, 1)
                except Exception:
                    initial_heading = 0.0

            # Compute total scheduled sorties for this drone
            total_drone_sorties = 1
            if drone_route:
                total_drone_sorties = max(1, max([int(wp.get("sortie_index", 1)) for wp in drone_route], default=1))

            has_route = len(drone_route) > 0
            self.drone_states[d_id] = {
                "lat": self.home_base[0],
                "lon": self.home_base[1],
                "alt_m": 0.0,
                "speed_mps": drone.speed_mps if has_route else 0.0,
                "heading_deg": initial_heading,
                "battery_pct": 100.0,
                "status": DroneStatus.STANDBY,
                "current_sortie": 1,
                "total_sorties": total_drone_sorties,
                "swap_remaining_sec": 0.0,
                "swap_start_battery": 100.0,
                "launch_delay_sec": stagger_delay if has_route else 0.0,
                "total_dist_covered_m": 0.0,
                "energy_consumed_wh": 0.0,
                "base_lat": self.home_base[0],
                "base_lon": self.home_base[1],
                "is_active_mission_drone": has_route
            }

        self.state = "IDLE"
        logger.info(
            f"[Simulator] Loaded mission for {len(self.drone_states)} UAVs. "
            f"Total unique targets: {self.total_cells_to_scan}, Algorithm: {self.algorithm_name}, Home Base: {self.home_base}"
        )

    def load_mission(self, algorithm_result: Dict[str, Any], initial_fleet: List[DroneTelemetry]):
        """Backward compatibility wrapper."""
        routes = algorithm_result.get("routes", {})
        algo_name = algorithm_result.get("algorithm", "Optimized Plan")
        home = (initial_fleet[0].latitude, initial_fleet[0].longitude) if initial_fleet else (33.3075, 44.3600)
        self.load_mission_routes(algo_name, routes, initial_fleet, home)

    def step(self) -> Dict[str, Any]:
        """
        Advances the simulation by one discrete time step (dt * time_scale).
        Executes kinematics, multi-sortie battery-swap state transitions, battery drain,
        and Opportunistic In-Flight Scanning.
        """
        if self.state not in ["RUNNING"]:
            return {}

        effective_dt = self.dt * self.time_scale
        self.mission_elapsed_sec += effective_dt

        newly_scanned_ids: List[str] = []
        newly_scanned_cells: List[Dict[str, Any]] = []
        all_active_drones_finished = True

        # Phase 1: Kinematic propagation and Multi-Sortie State Machine
        for d_id, state in self.drone_states.items():
            if not state.get("is_active_mission_drone", True):
                state["status"] = DroneStatus.STANDBY
                state["speed_mps"] = 0.0
                continue

            route = self.routes.get(d_id, [])
            idx = self.waypoint_indices.get(d_id, 0)

            # Staggered Launch Check
            if self.mission_elapsed_sec < state["launch_delay_sec"]:
                all_active_drones_finished = False
                state["status"] = DroneStatus.STANDBY
                state["speed_mps"] = 0.0
                continue

            # Check Battery Swapping State Machine
            if state["status"] == DroneStatus.BATTERY_SWAPPING:
                all_active_drones_finished = False
                state["speed_mps"] = 0.0
                state["alt_m"] = 0.0
                state["swap_remaining_sec"] = max(0.0, state["swap_remaining_sec"] - effective_dt)

                # Smooth battery replenishment up to 100.0%
                elapsed_swap = BATTERY_SWAP_DURATION_SEC - state["swap_remaining_sec"]
                ratio = min(1.0, max(0.0, elapsed_swap / BATTERY_SWAP_DURATION_SEC))
                start_b = state.get("swap_start_battery", 20.0)
                state["battery_pct"] = min(100.0, round(start_b + (100.0 - start_b) * ratio, 2))

                # Battery Swap Completed -> Relaunch next sortie
                if state["swap_remaining_sec"] <= 0.0:
                    state["battery_pct"] = 100.0
                    state["current_sortie"] += 1
                    state["status"] = DroneStatus.RELAUNCHING_SORTIE
                    logger.info(f"[Simulator] {d_id} completed battery swap. Relaunching Sortie #{state['current_sortie']}.")

                    # Advance waypoint past the SORTIE_RETURN_BASE node
                    self.waypoint_indices[d_id] += 1
                    # If next node is SORTIE_RELAUNCH_BASE, advance past it as well
                    if self.waypoint_indices[d_id] < len(route):
                        next_wp = route[self.waypoint_indices[d_id]]
                        if next_wp.get("waypoint_type") == "SORTIE_RELAUNCH_BASE":
                            self.waypoint_indices[d_id] += 1
                continue

            # Skip targets that were already scanned opportunistically (only for TARGET type)
            while idx < len(route):
                wp = route[idx]
                wp_type = str(wp.get("waypoint_type", "TARGET"))
                if wp_type == "TARGET":
                    wp_target_id = str(wp.get("target_id", wp.get("cell_id", "")))
                    if wp_target_id in self.scanned_target_ids:
                        idx += 1
                        self.waypoint_indices[d_id] = idx
                        continue
                break

            if idx < len(route):
                all_active_drones_finished = False
                target_wp = route[idx]
                wp_type = str(target_wp.get("waypoint_type", "TARGET"))
                target_lat = float(target_wp["lat"])
                target_lon = float(target_wp["lon"])
                target_alt = float(target_wp.get("safe_alt_m", target_wp.get("alt_m", 60.0)))
                roughness = float(target_wp.get("roughness", 1.0))
                cruise_speed = float(target_wp.get("speed_mps", 14.0))

                # Dynamic Flight Status Classification
                if wp_type == "SORTIE_RETURN_BASE":
                    state["status"] = DroneStatus.RETURNING_FOR_SWAP
                    target_alt = 0.0  # Descend to base
                elif state["status"] == DroneStatus.RELAUNCHING_SORTIE:
                    if abs(state["alt_m"] - target_alt) < 6.0:
                        state["status"] = DroneStatus.SORTIE_FLYING
                elif state["status"] not in [DroneStatus.RTL]:
                    state["status"] = DroneStatus.SORTIE_FLYING

                state["speed_mps"] = cruise_speed

                # Heading calculation
                try:
                    bearing_rad = GeodesicKinematics.calculate_bearing(
                        state["lat"], state["lon"], target_lat, target_lon
                    )
                    state["heading_deg"] = round(math.degrees(bearing_rad) % 360.0, 1)
                except Exception:
                    pass

                # Geodesic Kinematic step
                new_lat, new_lon, step_dist, arrived = GeodesicKinematics.step_towards(
                    current_lat=state["lat"],
                    current_lon=state["lon"],
                    target_lat=target_lat,
                    target_lon=target_lon,
                    speed_mps=state["speed_mps"],
                    dt_seconds=effective_dt,
                    arrival_threshold_m=4.5
                )

                state["lat"] = new_lat
                state["lon"] = new_lon

                # Smooth Altitude Transition
                if wp_type == "SORTIE_RETURN_BASE":
                    state["alt_m"] = max(0.0, state["alt_m"] - 4.0 * effective_dt)
                else:
                    if state["alt_m"] < target_alt:
                        state["alt_m"] = min(target_alt, state["alt_m"] + 5.0 * effective_dt)
                    else:
                        state["alt_m"] = target_alt

                state["total_dist_covered_m"] += step_dist

                # Battery drain during flight
                p_current = self.physics.p_cruise * roughness
                energy_wh = (p_current * effective_dt) / 3600.0
                state["energy_consumed_wh"] += energy_wh
                battery_drop_pct = (energy_wh / self.physics.battery_capacity_wh) * 100.0
                state["battery_pct"] = max(0.0, round(state["battery_pct"] - battery_drop_pct, 2))

                # Dynamic RTL safety monitor (only during active target search)
                if wp_type not in ("SORTIE_RETURN_BASE", "SORTIE_RELAUNCH_BASE"):
                    rtl_data = self.physics.compute_dynamic_rtl_reserve(
                        current_lat=new_lat,
                        current_lon=new_lon,
                        base_lat=state["base_lat"],
                        base_lon=state["base_lon"],
                        current_battery_pct=state["battery_pct"]
                    )
                    if rtl_data["must_trigger_rtl"] and state["status"] not in [DroneStatus.RTL, DroneStatus.RETURNING_FOR_SWAP]:
                        state["status"] = DroneStatus.RTL

                # Arrival at Waypoint
                if arrived:
                    if wp_type == "SORTIE_RETURN_BASE":
                        # Arrived at base -> enter BATTERY_SWAPPING
                        state["status"] = DroneStatus.BATTERY_SWAPPING
                        state["swap_remaining_sec"] = float(target_wp.get("metadata", {}).get("swap_time_sec", BATTERY_SWAP_DURATION_SEC))
                        state["swap_start_battery"] = state["battery_pct"]
                        state["speed_mps"] = 0.0
                        state["alt_m"] = 0.0
                        logger.info(f"[Simulator] {d_id} landed at Base for Battery Swap (Sortie #{state['current_sortie']} -> #{state['current_sortie']+1}).")
                    elif wp_type == "SORTIE_RELAUNCH_BASE":
                        self.waypoint_indices[d_id] += 1
                        state["status"] = DroneStatus.SORTIE_FLYING
                    else:
                        # Target waypoint arrived
                        self.waypoint_indices[d_id] += 1
                        target_id = str(target_wp.get("target_id", target_wp.get("cell_id", f"{d_id}_{idx}")))

                        if target_id not in self.scanned_target_ids:
                            self.scanned_target_ids.add(target_id)
                            self.unsearched_target_ids.discard(target_id)
                            self.scanned_cells_count += 1
                            newly_scanned_ids.append(target_id)

                            laser_dist = max(5.0, target_alt - float(target_wp.get("elevation_m", 0.0)) + float(np.random.normal(0, 0.15)))
                            est_elevation = LiDARTerrainProcessor.extract_elevation(target_alt, laser_dist)

                            newly_scanned_cells.append({
                                "cell_id": target_id,
                                "drone_id": d_id,
                                "elevation_m": round(est_elevation, 2),
                                "row": target_wp.get("row", 0),
                                "col": target_wp.get("col", 0),
                                "node_type": target_wp.get("node_type", "OPEN_TERRAIN_CELL")
                            })
            else:
                # Route completed for this drone
                state["status"] = DroneStatus.MISSION_COMPLETED
                state["speed_mps"] = 0.0

        # Phase 2: Opportunistic In-Flight Corridor Scanning
        if self.unsearched_target_ids:
            for d_id, state in self.drone_states.items():
                if state.get("status") not in [DroneStatus.SORTIE_FLYING, DroneStatus.SEARCHING, DroneStatus.RTL]:
                    continue

                d_lat = state["lat"]
                d_lon = state["lon"]
                d_alt = state["alt_m"]
                cos_d = math.cos(math.radians(d_lat))

                captured_this_step: List[str] = []
                for t_id in list(self.unsearched_target_ids):
                    target_info = self.mission_targets_registry.get(t_id)
                    if not target_info:
                        continue

                    dist_m = math.hypot(
                        (target_info["lat"] - d_lat) * 111320.0,
                        (target_info["lon"] - d_lon) * (111320.0 * cos_d)
                    )

                    if dist_m <= self.sensor_radius_m:
                        captured_this_step.append(t_id)
                        self.scanned_target_ids.add(t_id)
                        newly_scanned_ids.append(t_id)
                        self.scanned_cells_count += 1

                        laser_dist = max(5.0, d_alt - float(target_info.get("elevation_m", 0.0)) + float(np.random.normal(0, 0.15)))
                        est_elevation = LiDARTerrainProcessor.extract_elevation(d_alt, laser_dist)

                        newly_scanned_cells.append({
                            "cell_id": t_id,
                            "drone_id": d_id,
                            "elevation_m": round(est_elevation, 2),
                            "row": target_info.get("row", 0),
                            "col": target_info.get("col", 0),
                            "node_type": target_info.get("node_type", "OPEN_TERRAIN_CELL")
                        })

                for cid in captured_this_step:
                    self.unsearched_target_ids.discard(cid)

        # Dynamic Route Skip: advance all drones past newly scanned targets
        for d_id in self.routes.keys():
            r = self.routes[d_id]
            curr_idx = self.waypoint_indices.get(d_id, 0)
            while curr_idx < len(r):
                wp_curr = r[curr_idx]
                if wp_curr.get("waypoint_type", "TARGET") == "TARGET":
                    wid = str(wp_curr.get("target_id", wp_curr.get("cell_id", "")))
                    if wid not in self.scanned_target_ids:
                        break
                    curr_idx += 1
                    self.waypoint_indices[d_id] = curr_idx
                else:
                    break

        # Mission completion condition
        if (all_active_drones_finished and self.total_cells_to_scan > 0) or len(self.unsearched_target_ids) == 0:
            self.state = "COMPLETED"

        # Overall cumulative coverage percentage
        overall_cov = round((self.scanned_cells_count / max(1, self.total_cells_to_scan)) * 100.0, 1)
        overall_cov = min(100.0, max(0.0, overall_cov))

        fleet_data = [
            {
                "drone_id": d_id,
                "latitude": round(s["lat"], 6),
                "longitude": round(s["lon"], 6),
                "lat": round(s["lat"], 6),
                "lon": round(s["lon"], 6),
                "altitude_m": round(s["alt_m"], 1),
                "alt_m": round(s["alt_m"], 1),
                "speed_mps": round(s["speed_mps"], 1),
                "heading_deg": round(s.get("heading_deg", 0.0), 1),
                "battery_pct": round(s["battery_pct"], 1),
                "status": s["status"].value if hasattr(s["status"], "value") else str(s["status"]),
                "current_sortie": s.get("current_sortie", 1),
                "total_sorties": s.get("total_sorties", 1),
                "swap_remaining_sec": round(s.get("swap_remaining_sec", 0.0), 1),
                "overall_coverage_pct": overall_cov,
                "current_waypoint_idx": self.waypoint_indices.get(d_id, 0),
                "current_target_index": self.waypoint_indices.get(d_id, 0),
                "total_waypoints": len(self.routes.get(d_id, [])),
                "total_dist_m": round(s["total_dist_covered_m"], 1)
            }
            for d_id, s in self.drone_states.items()
        ]

        telemetry_frame = {
            "type": "TELEMETRY_TICK",
            "mission_id": self.mission_id,
            "payload": {
                "mission_id": self.mission_id,
                "mission_state": self.state,
                "sim_status": self.state,
                "elapsed_time_sec": round(self.mission_elapsed_sec, 1),
                "sim_time_sec": round(self.mission_elapsed_sec, 1),
                "progress_pct": overall_cov,
                "overall_coverage_pct": overall_cov,
                "cells_scanned_total": self.scanned_cells_count,
                "fleet": fleet_data,
                "drones": fleet_data,
                "newly_scanned_ids": newly_scanned_ids,
                "newly_scanned_cells": newly_scanned_cells,
                "home_base": {"lat": round(self.home_base[0], 6), "lon": round(self.home_base[1], 6)}
            }
        }

        return telemetry_frame

    async def run_simulation_loop(self):
        """Continuous non-blocking asynchronous simulation loop."""
        logger.info(f"[Simulator] Running real-time physics loop (dt={self.dt}s, time_scale={self.time_scale}x)...")
        try:
            while self.state in ["RUNNING", "PAUSED"]:
                if self.state == "RUNNING":
                    frame = self.step()
                    if frame:
                        try:
                            await telemetry_hub.broadcast(frame)
                        except Exception as e:
                            logger.warning(f"[Simulator] WebSocket broadcast warning: {e}")

                    if self.state == "COMPLETED":
                        logger.info("[Simulator] Mission completed across all drones.")
                        final_frame = self.step()
                        if final_frame:
                            await telemetry_hub.broadcast(final_frame)
                        break

                sleep_duration = max(0.03, self.dt / max(0.1, self.time_scale))
                await asyncio.sleep(sleep_duration)

        except asyncio.CancelledError:
            logger.info("[Simulator] Physics loop cancelled cleanly.")
        except Exception as e:
            logger.error(f"[Simulator] Exception in physics loop: {e}", exc_info=True)
        finally:
            logger.info("[Simulator] Physics loop finished.")

    def start(self):
        """Starts or resumes simulation."""
        if self.state in ["IDLE", "PAUSED"]:
            self.state = "RUNNING"
            if self._sim_task is None or self._sim_task.done():
                try:
                    loop = asyncio.get_running_loop()
                    self._sim_task = loop.create_task(self.run_simulation_loop())
                    logger.info("[Simulator] Background simulation task created successfully.")
                except RuntimeError:
                    logger.warning("[Simulator] No running event loop detected in current thread.")

    def pause(self):
        """Pauses simulation progression."""
        if self.state == "RUNNING":
            self.state = "PAUSED"
            logger.info("[Simulator] Simulation paused.")

    def resume(self):
        """Resumes simulation progression."""
        self.start()

    async def stop_and_reset(self):
        """Stops background loop and resets simulation state."""
        self.state = "IDLE"
        if self._sim_task and not self._sim_task.done():
            self._sim_task.cancel()
            try:
                await self._sim_task
            except asyncio.CancelledError:
                pass
            self._sim_task = None

        self.routes.clear()
        self.waypoint_indices.clear()
        self.drone_states.clear()
        self.scanned_target_ids.clear()
        self.mission_targets_registry.clear()
        self.unsearched_target_ids.clear()
        self.scanned_cells_count = 0
        self.total_cells_to_scan = 0
        self.mission_elapsed_sec = 0.0
        logger.info("[Simulator] Hard reset completed. Simulator is in clean IDLE state.")

    async def hard_reset(self):
        """Alias for stop_and_reset."""
        await self.stop_and_reset()

    def set_time_scale(self, scale: float):
        """Adjusts physics time dilation factor dynamically."""
        self.time_scale = max(0.1, min(10.0, float(scale)))
        logger.info(f"[Simulator] Time scale adjusted to {self.time_scale}x.")


global_simulator = MultiDronePhysicsSimulator()
