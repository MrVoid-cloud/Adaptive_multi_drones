import sys
import time
from pathlib import Path

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from app.gis.grid import GeodesicGridDiscretizer
from app.simulation.simulator_engine import MultiDronePhysicsSimulator
from app.adaptive.event_monitor import AdaptiveEventMonitor, AdaptiveTriggerType, AdaptiveEvent
from app.adaptive.warm_start_replanner import WarmStartAdaptiveReplanner
from app.fleet.telemetry_models import DroneTelemetry, DroneStatus
from app.api.v1.adaptive_routes import inject_adaptive_event, get_adaptive_logs, InjectEventPayload

def test_adaptive_event_monitor():
    print("[*] Testing AdaptiveEventMonitor & Anomaly Triggers...")
    monitor = AdaptiveEventMonitor(cooldown_seconds=2.0)

    # 1. State: UAV-01 in RTL with 5 unvisited cells
    drone_states = {
        "UAV-01": {"status": "RTL", "battery_pct": 14.0},
        "UAV-02": {"status": "SEARCHING", "battery_pct": 80.0}
    }
    routes = {
        "UAV-01": [{"cell_id": i} for i in range(10)],
        "UAV-02": [{"cell_id": i + 10} for i in range(10)]
    }
    waypoint_indices = {"UAV-01": 5, "UAV-02": 3}

    event = monitor.inspect_simulation_state(drone_states, routes, waypoint_indices)
    assert event is not None
    assert event.trigger_type == AdaptiveTriggerType.LOW_BATTERY_RTL
    assert event.drone_id == "UAV-01"
    print("  [✓] Detected LOW_BATTERY_RTL trigger correctly.")

    # 2. Cooldown check: Immediate second inspection should be suppressed by debouncing
    event_suppressed = monitor.inspect_simulation_state(drone_states, routes, waypoint_indices)
    assert event_suppressed is None, "Debouncing cooldown must suppress immediate duplicate trigger"
    print("  [✓] Debouncing & Cooldown window verified.")

    # 3. State: UAV-02 finished early (waypoint_idx = 10 == total), UAV-01 has 6 cells left
    monitor.last_trigger_times.clear() # clear cooldown
    drone_states2 = {
        "UAV-01": {"status": "SEARCHING", "battery_pct": 75.0},
        "UAV-02": {"status": "READY", "battery_pct": 85.0}
    }
    waypoint_indices2 = {"UAV-01": 2, "UAV-02": 10} # UAV-01 has 8 left, UAV-02 has 0
    event2 = monitor.inspect_simulation_state(drone_states2, routes, waypoint_indices2)
    assert event2 is not None
    assert event2.trigger_type == AdaptiveTriggerType.EARLY_SECTOR_COMPLETION
    assert event2.drone_id == "UAV-02"
    print("  [✓] Detected EARLY_SECTOR_COMPLETION dynamic load balancing trigger.")

def test_warm_start_replanner_execution():
    print("[*] Testing WarmStartAdaptiveReplanner & In-Flight Hot-Swapping...")
    grid = GeodesicGridDiscretizer(
        lat_min=33.3000, lat_max=33.3100, lon_min=44.3500, lon_max=44.3650, manual_cell_size_m=120.0
    )

    sim = MultiDronePhysicsSimulator(dt_seconds=0.5, time_scale=1.0)
    mock_routes = {
        "UAV-01": [
            {"cell_id": grid.cells[0].cell_id, "row": 0, "col": 0, "lat": 33.301, "lon": 44.351, "alt_m": 60, "elevation_m": 10, "roughness": 1.0},
            {"cell_id": grid.cells[1].cell_id, "row": 0, "col": 1, "lat": 33.302, "lon": 44.352, "alt_m": 60, "elevation_m": 10, "roughness": 1.0},
            {"cell_id": grid.cells[2].cell_id, "row": 0, "col": 2, "lat": 33.303, "lon": 44.353, "alt_m": 60, "elevation_m": 10, "roughness": 1.0}
        ],
        "UAV-02": [
            {"cell_id": grid.cells[3].cell_id, "row": 1, "col": 0, "lat": 33.304, "lon": 44.354, "alt_m": 60, "elevation_m": 10, "roughness": 1.0}
        ],
        "UAV-03": [
            {"cell_id": grid.cells[4].cell_id, "row": 1, "col": 1, "lat": 33.305, "lon": 44.355, "alt_m": 60, "elevation_m": 10, "roughness": 1.0}
        ]
    }
    mock_fleet = [
        DroneTelemetry(drone_id="UAV-01", endpoint_uri="sim://uav-01", latitude=33.301, longitude=44.351, altitude_m=60, speed_mps=14, battery_pct=14.0, status=DroneStatus.READY),
        DroneTelemetry(drone_id="UAV-02", endpoint_uri="sim://uav-02", latitude=33.304, longitude=44.354, altitude_m=60, speed_mps=14, battery_pct=85.0, status=DroneStatus.READY),
        DroneTelemetry(drone_id="UAV-03", endpoint_uri="sim://uav-03", latitude=33.305, longitude=44.355, altitude_m=60, speed_mps=14, battery_pct=90.0, status=DroneStatus.READY)
    ]
    sim.load_mission({"routes": mock_routes}, mock_fleet)

    replanner = WarmStartAdaptiveReplanner(simulator=sim)

    # UAV-01 triggers RTL event: its 3 cells should be absorbed by UAV-02 and UAV-03!
    event = {
        "trigger_type": "LOW_BATTERY_RTL",
        "drone_id": "UAV-01"
    }

    replan_record = replanner.execute_replan(event_dict=event, all_grid_cells=grid.cells)
    
    assert replan_record["status"] == "HOTSWAP_SUCCESS"
    assert replan_record["surviving_drones_count"] == 2 # UAV-02 and UAV-03
    assert replan_record["latency_ms"] < 1000.0 # Fast convergence
    assert sim.drone_states["UAV-01"]["status"] == DroneStatus.RTL
    assert len(sim.routes["UAV-01"]) == 0 # Truncated to RTL directly
    assert len(sim.routes["UAV-02"]) + len(sim.routes["UAV-03"]) == 5 # All 5 cells preserved!

    print(f"  [✓] Re-planning completed in {replan_record['latency_ms']:.1f}ms: 5 cells distributed across 2 surviving UAVs.")

def test_adaptive_api_endpoints():
    print("[*] Testing Adaptive REST API Routes...")
    payload = InjectEventPayload(
        trigger_type="LOW_BATTERY_RTL",
        drone_id="UAV-01",
        description="Simulated emergency battery threshold"
    )
    res = inject_adaptive_event(payload)
    assert res["status"] == "EVENT_INJECTED_AND_PROCESSED"
    
    logs = get_adaptive_logs()
    assert logs["audit_events_count"] >= 1
    assert logs["replan_executions_count"] >= 1
    print("  [✓] Injected event and fetched audit logs successfully.")

if __name__ == "__main__":
    test_adaptive_event_monitor()
    test_warm_start_replanner_execution()
    test_adaptive_api_endpoints()
    print("\n[SUCCESS] All Phase 08 Closed-Loop Adaptation & Warm-Start Re-Planner Tests Passed!")
