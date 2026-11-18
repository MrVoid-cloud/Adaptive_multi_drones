import sys
import time
from pathlib import Path

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from app.simulation.kinematics import GeodesicKinematics
from app.simulation.simulator_engine import MultiDronePhysicsSimulator, global_simulator
from app.simulation.kpi_comparator import TriAlgorithmKPIComparator
from app.fleet.telemetry_models import DroneTelemetry, DroneStatus
from app.api.v1.simulation_routes import control_simulation, get_simulation_status, step_simulation, SimControlPayload

def test_geodesic_kinematics_and_anti_overshoot():
    print("[*] Testing Geodesic Kinematics & Waypoint Anti-Overshoot...")
    start_lat, start_lon = 33.3000, 44.3500
    target_lat, target_lon = 33.3010, 44.3510
    
    # 1. Normal small step (speed = 10 m/s, dt = 1.0s -> 10m step)
    n_lat, n_lon, dist, arrived = GeodesicKinematics.step_towards(
        current_lat=start_lat, current_lon=start_lon,
        target_lat=target_lat, target_lon=target_lon,
        speed_mps=10.0, dt_seconds=1.0
    )
    assert arrived is False
    assert abs(dist - 10.0) < 0.1
    assert n_lat > start_lat
    assert n_lon > start_lon
    print("  [✓] Small step propagation verified without arrival.")

    # 2. Giant step (speed = 500 m/s, dt = 10s -> 5000m step, much larger than distance to target ~144m)
    n_lat2, n_lon2, dist2, arrived2 = GeodesicKinematics.step_towards(
        current_lat=start_lat, current_lon=start_lon,
        target_lat=target_lat, target_lon=target_lon,
        speed_mps=500.0, dt_seconds=10.0
    )
    assert arrived2 is True
    assert n_lat2 == target_lat, "Must clamp to target coordinate without overshooting"
    assert n_lon2 == target_lon, "Must clamp to target coordinate without overshooting"
    print("  [✓] Anti-overshoot clamping verified: UAV snapped to exact target waypoint.")

def test_simulator_execution_and_lidar_scanning():
    print("[*] Testing MultiDronePhysicsSimulator Kernel & Live LiDAR...")
    sim = MultiDronePhysicsSimulator(dt_seconds=0.5, time_scale=2.0)

    # Fake mission routes
    mock_routes = {
        "UAV-01": [
            {"cell_id": 10, "row": 1, "col": 0, "lat": 33.3020, "lon": 44.3510, "alt_m": 60.0, "elevation_m": 12.0, "roughness": 1.2},
            {"cell_id": 11, "row": 1, "col": 1, "lat": 33.3040, "lon": 44.3520, "alt_m": 60.0, "elevation_m": 15.0, "roughness": 1.5}
        ],
        "UAV-02": [
            {"cell_id": 20, "row": 2, "col": 0, "lat": 33.3015, "lon": 44.3530, "alt_m": 60.0, "elevation_m": 8.0, "roughness": 1.0}
        ]
    }

    mock_fleet = [
        DroneTelemetry(drone_id="UAV-01", endpoint_uri="sim://uav-01", latitude=33.3000, longitude=44.3500, altitude_m=60.0, speed_mps=15.0, battery_pct=100.0, status=DroneStatus.READY),
        DroneTelemetry(drone_id="UAV-02", endpoint_uri="sim://uav-02", latitude=33.3000, longitude=44.3500, altitude_m=60.0, speed_mps=14.0, battery_pct=95.0, status=DroneStatus.READY)
    ]

    sim.load_mission({"routes": mock_routes}, mock_fleet)
    assert sim.total_cells_to_scan == 3
    assert sim.scanned_cells_count == 0

    sim.start()
    assert sim.state == "RUNNING"

    # Step forward 20 steps (effective time = 20 * 0.5s * 2.0 = 20 seconds)
    total_newly_scanned = 0
    for _ in range(20):
        frame = sim.step()
        assert frame["type"] == "TELEMETRY_TICK"
        total_newly_scanned += len(frame["payload"]["newly_scanned_cells"])

    assert sim.drone_states["UAV-01"]["battery_pct"] < 100.0, "Battery must discharge"
    assert sim.drone_states["UAV-01"]["total_dist_covered_m"] > 0.0, "Distance must accumulate"
    assert sim.scanned_cells_count > 0, "At least one cell must be scanned"
    print(f"  [✓] Simulator stepped: {sim.scanned_cells_count} cells scanned, battery discharged to {sim.drone_states['UAV-01']['battery_pct']:.1f}%.")

def test_kpi_comparator():
    print("[*] Testing TriAlgorithmKPIComparator...")
    pso_res = {
        "algorithm": "Discrete PSO",
        "makespan_sec": 620.0,
        "total_energy_wh": 75.0,
        "workload_std_sec": 45.0,
        "compute_time_ms": 350.0,
        "best_fitness": 450.0,
        "routes": {"UAV-01": [{}, {}], "UAV-02": [{}]}
    }
    ga_res = {
        "algorithm": "Modified GA",
        "makespan_sec": 590.0,
        "total_energy_wh": 72.0,
        "workload_std_sec": 38.0,
        "compute_time_ms": 420.0,
        "best_fitness": 470.0,
        "routes": {"UAV-01": [{}, {}], "UAV-02": [{}]}
    }
    hybrid_res = {
        "algorithm": "Hybrid GA-PSO",
        "makespan_sec": 510.0,
        "total_energy_wh": 65.0,
        "workload_std_sec": 18.0,
        "compute_time_ms": 510.0,
        "best_fitness": 540.0,
        "routes": {"UAV-01": [{}, {}], "UAV-02": [{}]}
    }

    report = TriAlgorithmKPIComparator.generate_comparison_report(
        pso_res=pso_res,
        ga_res=ga_res,
        hybrid_res=hybrid_res,
        total_cells_count=3
    )

    assert report["benchmark_order"] == ["Discrete PSO", "Modified GA", "Hybrid GA-PSO"]
    assert report["kpi_summary"]["optimal_solver"] == "Hybrid GA-PSO"
    assert report["kpi_summary"]["makespan_reduction_vs_pso_pct"] > 15.0 # (620-510)/620 = 17.74%
    assert report["algorithms"]["hybrid"]["coverage_pct"] == 100.0
    print(f"  [✓] KPI Benchmark verified: Hybrid achieved {report['kpi_summary']['makespan_reduction_vs_pso_pct']}% makespan reduction.")

def test_simulation_api_endpoints():
    print("[*] Testing Simulation REST API Routes...")
    import asyncio
    # 1. Set speed
    res = asyncio.run(control_simulation(SimControlPayload(action="SET_SPEED", time_scale=3.0)))
    assert res["time_scale"] == 3.0

    # 2. Start
    res_start = asyncio.run(control_simulation(SimControlPayload(action="START")))
    assert res_start["current_state"] == "RUNNING"

    # 3. Status
    status_res = get_simulation_status()
    assert status_res["state"] == "RUNNING"

    # 4. Step
    step_res = step_simulation()
    assert isinstance(step_res, dict)

    # 5. Pause
    res_pause = asyncio.run(control_simulation(SimControlPayload(action="PAUSE")))
    assert res_pause["current_state"] == "PAUSED"
    print("  [✓] Simulation REST control endpoints verified.")

def test_deploy_and_start_pipeline():
    print("[*] Testing Atomic Deploy & Start Endpoint Pipeline...")
    import asyncio
    from app.api.v1.simulation_routes import deploy_and_start_simulation, DeployAndStartPayload

    routes = {
        "UAV-01": [
            {"cell_id": 101, "target_id": "ROAD_0001", "node_type": "ROAD_INTERSECTION", "lat": 33.3080, "lon": 44.3600, "safe_alt_m": 70.0},
            {"cell_id": 102, "target_id": "CELL_0002", "node_type": "OPEN_TERRAIN_CELL", "lat": 33.3100, "lon": 44.3620, "safe_alt_m": 60.0}
        ],
        "UAV-02": [
            {"cell_id": 201, "target_id": "ROAD_0002", "node_type": "ROAD_INTERSECTION", "lat": 33.3090, "lon": 44.3610, "safe_alt_m": 75.0}
        ]
    }

    res = asyncio.run(deploy_and_start_simulation(DeployAndStartPayload(
        algorithm_name="Hybrid GA-PSO",
        routes=routes,
        time_scale=2.0
    )))

    assert res["status"] == "SIMULATION_STARTED"
    assert res["drones_count"] == 2
    assert res["total_targets"] == 3
    assert res["time_scale"] == 2.0
    assert "home_base" in res
    print("  [✓] Deploy & Start atomic pipeline verified successfully.")

if __name__ == "__main__":
    test_geodesic_kinematics_and_anti_overshoot()
    test_simulator_execution_and_lidar_scanning()
    test_kpi_comparator()
    test_simulation_api_endpoints()
    test_deploy_and_start_pipeline()
    print("\n[SUCCESS] All Phase 07 Simulation & KPI Comparator Tests Passed!")
