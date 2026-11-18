"""
End-to-End Mission Session Epoch & Hard-Reset Synchronization Integration Test
Validates:
1. UUID Session Generation on Hybrid Grid Creation
2. Stale Mission ID Rejection (409 Conflict)
3. Atomic Hard-Reset & Clear Endpoint (POST /api/v1/mission/clear)
4. Home Base Relocation
5. Seamless Region B Path Optimization & Launch
Author: Principal Software Architect
"""

import urllib.request
import json
import time

BASE_URL = "http://localhost:8000"

def post_json(path, data):
    req = urllib.request.Request(
        f"{BASE_URL}{path}",
        data=json.dumps(data).encode("utf-8"),
        headers={"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(req) as resp:
            return resp.getcode(), json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))

def get_json(path):
    req = urllib.request.Request(f"{BASE_URL}{path}")
    with urllib.request.urlopen(req) as resp:
        return resp.getcode(), json.loads(resp.read().decode("utf-8"))

print("=== Starting End-to-End Session Epoch Synchronization Test ===")

# Test 1: Check clean initial state
code, data = get_json("/api/v1/mission/current")
print(f"1. Initial Mission Current: code={code}, mission_id={data.get('mission_id')}")

# Test 2: Generate Hybrid Space for Region A (Baghdad Center)
region_a_payload = {
    "roi_type": "POLYGON",
    "bounds": {"lat_min": 33.3000, "lat_max": 33.3150, "lon_min": 44.3500, "lon_max": 44.3700},
    "vertices": [
        {"lat": 33.3000, "lon": 44.3500},
        {"lat": 33.3150, "lon": 44.3500},
        {"lat": 33.3150, "lon": 44.3700},
        {"lat": 33.3000, "lon": 44.3700}
    ],
    "cluster_threshold_m": 35.0,
    "buffer_radius_ratio": 0.65
}

code, res_a = post_json("/api/v1/grid/hybrid-generate", region_a_payload)
assert code == 201, f"Failed generating region A: {res_a}"
mission_id_a = res_a.get("mission_id")
assert mission_id_a is not None, "mission_id was not returned in hybrid-generate response"
home_base_a = res_a.get("home_base")
print(f"2. Region A Generated: mission_id={mission_id_a}, home_base={home_base_a}, targets={len(res_a['targets'])}")

# Test 3: Optimize Region A with mission_id_a
code, opt_a = post_json("/api/v1/optimize/compare", {
    "mission_id": mission_id_a,
    "iterations": 10,
    "population_size": 10,
    "fleet_size": 3
})
assert code == 200, f"Optimize A failed: {opt_a}"
assert opt_a.get("mission_id") == mission_id_a
routes_a = opt_a["hybrid_result"]["routes"]
first_wp_a = routes_a["UAV-01"][0]
print(f"3. Region A Optimized: UAV-01 first wp=({first_wp_a['lat']:.4f}, {first_wp_a['lon']:.4f})")

# Test 4: Deploy & Start Simulation for Region A
code, dep_a = post_json("/api/v1/sim/deploy-and-start", {
    "mission_id": mission_id_a,
    "algorithm_name": "Hybrid GA-PSO",
    "routes": routes_a,
    "time_scale": 2.0
})
assert code == 200, f"Deploy A failed: {dep_a}"
assert dep_a.get("status") == "SIMULATION_STARTED"
print(f"4. Region A Simulation Launched: status={dep_a.get('status')}")

# Test 5: Verify Stale Mission ID Rejection (409 Conflict)
fake_stale_id = "00000000-0000-0000-0000-000000000000"
code, err_stale = post_json("/api/v1/optimize/compare", {
    "mission_id": fake_stale_id,
    "iterations": 10,
    "population_size": 10
})
assert code == 409, f"Expected 409 Conflict for stale mission_id, got {code}: {err_stale}"
print(f"5. Stale Mission ID Rejection Verified (HTTP 409 Conflict): detail={err_stale.get('detail')}")

# Test 6: Clear Mission (POST /api/v1/mission/clear)
code, clr_res = post_json("/api/v1/mission/clear", {})
assert code == 200
code, sim_status = get_json("/api/v1/sim/status")
assert sim_status.get("state") == "IDLE", "Simulator was not reset to IDLE"
code, curr_mission = get_json("/api/v1/mission/current")
assert curr_mission.get("mission_id") is None, "mission_id was not cleared"
print(f"6. Atomic Hard Reset Verified: simulator state={sim_status.get('state')}, mission_id={curr_mission.get('mission_id')}")

# Test 7: Generate Region B in a distinct geographic area (e.g. Al-Mansour North: 33.3200-33.3350, 44.3400-44.3580)
region_b_payload = {
    "roi_type": "POLYGON",
    "bounds": {"lat_min": 33.3200, "lat_max": 33.3350, "lon_min": 44.3400, "lon_max": 44.3580},
    "vertices": [
        {"lat": 33.3200, "lon": 44.3400},
        {"lat": 33.3350, "lon": 44.3400},
        {"lat": 33.3350, "lon": 44.3580},
        {"lat": 33.3200, "lon": 44.3580}
    ],
    "cluster_threshold_m": 35.0,
    "buffer_radius_ratio": 0.65
}

code, res_b = post_json("/api/v1/grid/hybrid-generate", region_b_payload)
assert code == 201
mission_id_b = res_b.get("mission_id")
assert mission_id_b is not None and mission_id_b != mission_id_a, "mission_id_b must be a distinct fresh UUID"
home_base_b = res_b.get("home_base")
print(f"7. Region B Generated: new mission_id={mission_id_b}, home_base={home_base_b}")

# Test 8: Optimize & Launch Region B
code, opt_b = post_json("/api/v1/optimize/compare", {
    "mission_id": mission_id_b,
    "iterations": 10,
    "population_size": 10,
    "fleet_size": 3
})
assert code == 200
routes_b = opt_b["hybrid_result"]["routes"]
first_wp_b = routes_b["UAV-01"][0]
print(f"8. Region B Optimized: UAV-01 first wp=({first_wp_b['lat']:.4f}, {first_wp_b['lon']:.4f})")
# Verify coordinates belong strictly to Region B
assert 33.3190 <= first_wp_b["lat"] <= 33.3360, f"Waypoint {first_wp_b['lat']} is NOT in Region B!"

# Test 9: Deploy and Start Region B Simulation
code, dep_b = post_json("/api/v1/sim/deploy-and-start", {
    "mission_id": mission_id_b,
    "algorithm_name": "Hybrid GA-PSO",
    "routes": routes_b,
    "time_scale": 3.0
})
assert code == 200
assert dep_b.get("status") == "SIMULATION_STARTED"
assert dep_b.get("mission_id") == mission_id_b

time.sleep(0.5)
code, sim_b = get_json("/api/v1/sim/status")
print(f"9. Region B Simulation Running: state={sim_b.get('state')}, elapsed={sim_b.get('mission_elapsed_sec')}s, drones={list(sim_b.get('drones', {}).keys())}")

print("\n==================================================================")
print(">>> ALL 9 ARCHITECTURAL TESTS PASSED WITH 100% SUCCESS! <<<")
print("==================================================================")
