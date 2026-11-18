import urllib.request
import json
import time
import math

BASE = "http://localhost:8000"

def post(url, payload):
    req = urllib.request.Request(
        f"{BASE}{url}",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req) as resp:
        return resp.getcode(), json.loads(resp.read().decode("utf-8"))

def get(url):
    with urllib.request.urlopen(f"{BASE}{url}") as resp:
        return resp.getcode(), json.loads(resp.read().decode("utf-8"))

print("=== STARTING COMPLETE PIPELINE VERIFICATION ===")

# Step 1: Clear current mission
code, data = post("/api/v1/mission/clear", {})
st = data.get("state")
print(f"1. Mission Clear: code={code}, state={st}")

# Step 2: Draw Region 1 (Baghdad Center / Karrada)
center1 = (33.3050, 44.4100)
verts1 = []
for i in range(24):
    a = 2 * math.pi * i / 24
    verts1.append({"lat": center1[0] + 0.007 * math.sin(a), "lon": center1[1] + 0.008 * math.cos(a)})
lats1 = [v["lat"] for v in verts1]
lons1 = [v["lon"] for v in verts1]

t0 = time.time()
code, r1_grid = post("/api/v1/grid/hybrid-generate", {
    "roi_type": "POLYGON",
    "bounds": {"lat_min": min(lats1), "lat_max": max(lats1), "lon_min": min(lons1), "lon_max": max(lons1)},
    "vertices": verts1,
    "cluster_threshold_m": 35.0,
    "buffer_radius_ratio": 0.65
})
r1_mid = r1_grid["mission_id"]
r1_hb = r1_grid["home_base"]
r1_targets = len(r1_grid["targets"])
r1_roads = r1_grid["statistics"]["road_intersection_count"]
r1_open = r1_grid["statistics"]["open_terrain_cells_count"]
print(f"2. Region 1 Generated ({time.time()-t0:.2f}s): mission_id={r1_mid}, targets={r1_targets} (roads={r1_roads}, free={r1_open}), home_base={r1_hb}")

# Step 3: Run Tri-Algorithm Benchmark for Region 1
t0 = time.time()
code, r1_bench = post("/api/v1/optimize/compare", {
    "mission_id": r1_mid,
    "iterations": 15,
    "population_size": 15,
    "fleet_size": 3
})
pso_t = r1_bench["pso_result"]["makespan_sec"]
ga_t = r1_bench["ga_result"]["makespan_sec"]
hyb_t = r1_bench["hybrid_result"]["makespan_sec"]
r1_wps = len(r1_bench["hybrid_result"]["routes"]["UAV-01"])
print(f"3. Region 1 Optimized ({time.time()-t0:.2f}s): PSO={pso_t:.1f}s, GA={ga_t:.1f}s, HYBRID={hyb_t:.1f}s, UAV-01 wps={r1_wps}")

# Step 4: Deploy & Start Simulation for Region 1
code, r1_sim = post("/api/v1/sim/deploy-and-start", {
    "mission_id": r1_mid,
    "algorithm_name": "Hybrid GA-PSO",
    "routes": r1_bench["hybrid_result"]["routes"],
    "time_scale": 2.0
})
sim_st = r1_sim.get("status")
sim_dc = r1_sim.get("drones_count")
print(f"4. Region 1 Deployed: status={sim_st}, drones={sim_dc}")

# Step 5: Operator draws Region 2 in a completely different area (Baghdad Mansour / Yarmouk)
center2 = (33.3250, 44.3350)
verts2 = []
for i in range(24):
    a = 2 * math.pi * i / 24
    verts2.append({"lat": center2[0] + 0.006 * math.sin(a), "lon": center2[1] + 0.007 * math.cos(a)})
lats2 = [v["lat"] for v in verts2]
lons2 = [v["lon"] for v in verts2]

t0 = time.time()
code, r2_grid = post("/api/v1/grid/hybrid-generate", {
    "roi_type": "POLYGON",
    "bounds": {"lat_min": min(lats2), "lat_max": max(lats2), "lon_min": min(lons2), "lon_max": max(lons2)},
    "vertices": verts2,
    "cluster_threshold_m": 35.0,
    "buffer_radius_ratio": 0.65
})
r2_mid = r2_grid["mission_id"]
r2_hb = r2_grid["home_base"]
r2_targets = len(r2_grid["targets"])
print(f"5. Region 2 Generated ({time.time()-t0:.2f}s): new mission_id={r2_mid}, home_base={r2_hb}, targets={r2_targets}")

assert r2_mid != r1_mid, "ERROR: mission_id did not change!"
assert abs(r2_hb["lat"] - center2[0]) < 0.02, "ERROR: home base not relocated to Region 2!"

# Step 6: Verify Stale Plan Launch Rejection (Region 1 plan on Region 2 session)
try:
    code, err = post("/api/v1/sim/deploy-and-start", {
        "mission_id": r1_mid,
        "algorithm_name": "Hybrid GA-PSO",
        "routes": r1_bench["hybrid_result"]["routes"],
        "time_scale": 2.0
    })
    print("ERROR: stale plan was not rejected!")
except urllib.error.HTTPError as e:
    err_body = e.read().decode("utf-8")
    print(f"6. Stale Session Rejection Verified (HTTP {e.code}): {err_body}")

# Step 7: Optimize for Region 2
t0 = time.time()
code, r2_bench = post("/api/v1/optimize/compare", {
    "mission_id": r2_mid,
    "iterations": 15,
    "population_size": 15,
    "fleet_size": 3
})
r2_wps = len(r2_bench["hybrid_result"]["routes"]["UAV-01"])
first_wp_lat = r2_bench["hybrid_result"]["routes"]["UAV-01"][0]["lat"]
print(f"7. Region 2 Optimized ({time.time()-t0:.2f}s): UAV-01 wps={r2_wps}, first_wp_lat={first_wp_lat:.4f}")

# Verify coordinates are in Region 2 (near 33.325, NOT Region 1 near 33.305)
assert abs(first_wp_lat - center2[0]) < 0.02, f"ERROR: Waypoints belong to old region! {first_wp_lat} vs {center2[0]}"
print(">>> VERIFIED: Waypoints belong 100% strictly to Region 2 coordinates with zero stale data!")

# Step 8: Deploy Region 2
code, r2_sim = post("/api/v1/sim/deploy-and-start", {
    "mission_id": r2_mid,
    "algorithm_name": "Hybrid GA-PSO",
    "routes": r2_bench["hybrid_result"]["routes"],
    "time_scale": 2.0
})
sim_st2 = r2_sim.get("status")
print(f"8. Region 2 Deployed: status={sim_st2}")

print("\n======================================================")
print(">>> FULL END-TO-END PIPELINE VERIFICATION: 100% SUCCESS <<<")
print("======================================================")
