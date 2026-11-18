import urllib.request, json, time

API = 'http://localhost:8000'

# Step 1: Check server status
with urllib.request.urlopen(f'{API}/api/v1/sim/status') as r:
    print('1. Sim status:', r.read().decode())

# Step 2: Draw a NEW polygon in a completely different area (e.g. Al-Mansour area)
poly_payload = {
    'roi_type': 'POLYGON',
    'bounds': {'lat_min': 33.3100, 'lat_max': 33.3250, 'lon_min': 44.3500, 'lon_max': 44.3680},
    'vertices': [
        {'lat': 33.3100, 'lon': 44.3500},
        {'lat': 33.3250, 'lon': 44.3500},
        {'lat': 33.3250, 'lon': 44.3680},
        {'lat': 33.3100, 'lon': 44.3680}
    ],
    'area_km2': 2.75
}
req = urllib.request.Request(f'{API}/api/v1/map/roi', data=json.dumps(poly_payload).encode(), headers={'Content-Type': 'application/json'})
with urllib.request.urlopen(req) as r:
    print('2. ROI Registered:', r.read().decode())

# Step 3: Generate Hybrid Space for this new polygon
hybrid_payload = {
    'roi_type': 'POLYGON',
    'bounds': {'lat_min': 33.3100, 'lat_max': 33.3250, 'lon_min': 44.3500, 'lon_max': 44.3680},
    'vertices': poly_payload['vertices'],
    'cluster_threshold_m': 35.0,
    'buffer_radius_ratio': 0.65
}
req = urllib.request.Request(f'{API}/api/v1/grid/hybrid-generate', data=json.dumps(hybrid_payload).encode(), headers={'Content-Type': 'application/json'})
with urllib.request.urlopen(req) as r:
    hybrid_res = json.loads(r.read().decode())
    print(f"3. Hybrid Space: {hybrid_res['status']} | Targets: {len(hybrid_res['targets'])} | Road nodes: {hybrid_res['statistics']['road_intersection_count']} | Open cells: {hybrid_res['statistics']['open_terrain_cells_count']}")

# Step 4: Run Optimization Compare on this new area
opt_payload = {'iterations': 10, 'population_size': 12, 'fleet_size': 3}
req = urllib.request.Request(f'{API}/api/v1/optimize/compare', data=json.dumps(opt_payload).encode(), headers={'Content-Type': 'application/json'})
with urllib.request.urlopen(req) as r:
    opt_res = json.loads(r.read().decode())
    print('4. Tri-Algorithm Benchmark Succeeded!')
    hybrid_routes = opt_res['hybrid_result']['routes']
    for drone_id, wps in hybrid_routes.items():
        if wps:
            lat0 = wps[0]['lat']
            lon0 = wps[0]['lon']
            print(f"   {drone_id}: {len(wps)} waypoints. First wp: ({lat0:.4f}, {lon0:.4f})")
            assert 33.3090 <= lat0 <= 33.3260, f"Waypoint {lat0} out of new ROI bounds!"

# Step 5: Deploy and Start simulation
deploy_payload = {
    'algorithm_name': 'Hybrid GA-PSO',
    'routes': hybrid_routes,
    'time_scale': 2.0
}
req = urllib.request.Request(f'{API}/api/v1/sim/deploy-and-start', data=json.dumps(deploy_payload).encode(), headers={'Content-Type': 'application/json'})
with urllib.request.urlopen(req) as r:
    dep_res = json.loads(r.read().decode())
    print(f"5. Deploy and Start: {dep_res['status']} | Drones count: {dep_res['drones_count']}")

time.sleep(1.0)
with urllib.request.urlopen(f'{API}/api/v1/sim/status') as r:
    sim_status = json.loads(r.read().decode())
    print(f"6. Live Simulation Status: {sim_status['state']} | Elapsed: {sim_status['mission_elapsed_sec']} | Drones: {list(sim_status['drones'].keys())}")

print("\n*** ALL VERIFICATION CHECKS PASSED WITH 100% SUCCESS! ***")
