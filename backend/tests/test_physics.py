import sys
import math
from pathlib import Path
import numpy as np

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from physics.flight_physics import (
    haversine_distance,
    LiDARTerrainProcessor,
    UAVPhysicsEngine
)
from physics.synthetic_terrain import SyntheticTerrainGenerator

def test_haversine_distance():
    print("[*] Testing Geodesic Haversine Distance...")
    # Exact 0.05 degree latitude step along the same meridian
    lat1, lon1 = 33.3000, 44.3500
    lat2, lon2 = 33.3500, 44.3500
    
    d_m = haversine_distance(lat1, lon1, lat2, lon2)
    # Expected ~5559.7 meters
    expected_m = 5559.7
    assert abs(d_m - expected_m) < 10.0, f"Expected ~{expected_m}m, got {d_m}m"
    
    # Distance between identical points must be 0
    assert haversine_distance(lat1, lon1, lat1, lon1) == 0.0
    print(f"  [✓] Haversine calculation verified: {d_m:.2f} m.")

def test_synthetic_terrain_and_lidar_filtering():
    print("[*] Testing Synthetic DEM & LiDAR Canopy Ground Filtering...")
    gen = SyntheticTerrainGenerator(seed=123)
    dem = gen.generate_dem(rows=30, cols=30, base_elevation_m=100.0, max_mountain_height_m=300.0)
    
    assert dem.shape == (30, 30)
    assert np.min(dem) >= 50.0
    assert np.max(dem) > 200.0
    
    # Simulate LiDAR returns with high canopy spikes
    raw_ranges, uav_altitudes = gen.simulate_lidar_scan(dem, drone_altitude_agl_m=60.0, canopy_density=0.4)
    raw_elevations = LiDARTerrainProcessor.extract_elevation(uav_altitudes, raw_ranges)
    
    # Ground filter
    filtered_dem = LiDARTerrainProcessor.filter_canopy_noise(raw_elevations, window_size=3)
    
    # Verify canopy spikes are attenuated
    mean_err_raw = np.mean(np.abs(raw_elevations - dem))
    mean_err_filtered = np.mean(np.abs(filtered_dem - dem))
    assert mean_err_filtered < mean_err_raw, "Filtered DEM must reduce canopy error"
    print(f"  [✓] Filtered canopy clutter: raw error = {mean_err_raw:.2f}m -> filtered error = {mean_err_filtered:.2f}m.")

def test_terrain_factor_and_classification():
    print("[*] Testing Terrain Factor (tau_j) Calculation...")
    # 1. Flat terrain
    flat_patch = np.ones((5, 5)) * 100.0
    tau_flat, class_flat = LiDARTerrainProcessor.compute_terrain_factor(flat_patch, cell_size_m=50.0)
    assert tau_flat == 1.0
    assert class_flat == "OPEN_GROUND"
    
    # 2. Rugged Mountainous terrain with steep slope
    mountain_patch = np.linspace(100.0, 220.0, 25).reshape((5, 5)) # Delta Z = 120m across 50m cell!
    tau_mtn, class_mtn = LiDARTerrainProcessor.compute_terrain_factor(mountain_patch, cell_size_m=50.0)
    assert 2.4 <= tau_mtn <= 3.0
    assert class_mtn == "RUGGED_MOUNTAINOUS"
    print(f"  [✓] Flat terrain: tau = {tau_flat} ({class_flat}), Mountain: tau = {tau_mtn} ({class_mtn}).")

def test_uav_physics_energy_and_vertical_penalty():
    print("[*] Testing UAV Aerodynamics & Vertical Climb Energy Penalty...")
    engine = UAVPhysicsEngine()
    
    # Flat transit: 500m horizontal, 0m climb
    cost_flat = engine.calculate_transition_cost(
        start_lat=33.3000, start_lon=44.3500, start_alt_m=60.0,
        target_lat=33.3045, target_lon=44.3500, target_alt_m=60.0,
        terrain_factor_tau=1.0
    )
    
    # Climbing transit: 500m horizontal + 80m vertical climb
    cost_climb = engine.calculate_transition_cost(
        start_lat=33.3000, start_lon=44.3500, start_alt_m=60.0,
        target_lat=33.3045, target_lon=44.3500, target_alt_m=140.0,
        terrain_factor_tau=1.0
    )
    
    # Verify vertical climb consumes substantially more energy and time
    assert cost_climb["energy_consumed_wh"] > cost_flat["energy_consumed_wh"]
    assert cost_climb["battery_pct_cost"] > cost_flat["battery_pct_cost"]
    
    # Rough terrain transit: tau = 2.5
    cost_rough = engine.calculate_transition_cost(
        start_lat=33.3000, start_lon=44.3500, start_alt_m=60.0,
        target_lat=33.3045, target_lon=44.3500, target_alt_m=60.0,
        terrain_factor_tau=2.5
    )
    assert cost_rough["energy_consumed_wh"] > cost_flat["energy_consumed_wh"]
    print(f"  [✓] Energy breakdown: Flat = {cost_flat['battery_pct_cost']}% BAT, With 80m Climb = {cost_climb['battery_pct_cost']}% BAT, Rough tau=2.5 = {cost_rough['battery_pct_cost']}% BAT.")

def test_dynamic_rtl_reserve():
    print("[*] Testing Dynamic RTL Safety Battery Reservation...")
    engine = UAVPhysicsEngine()
    base_lat, base_lon = 33.3000, 44.3500
    
    # Drone is nearby base (300m away, 80% battery)
    rtl_near = engine.compute_dynamic_rtl_reserve(
        current_lat=33.3027, current_lon=44.3500,
        base_lat=base_lat, base_lon=base_lon,
        current_battery_pct=80.0
    )
    assert rtl_near["must_trigger_rtl"] is False
    assert rtl_near["rtl_threshold_pct"] < 20.0 # Close to 15% floor
    
    # Drone is far away (3.5 km away, 22% battery left)
    rtl_far = engine.compute_dynamic_rtl_reserve(
        current_lat=33.3315, current_lon=44.3500,
        base_lat=base_lat, base_lon=base_lon,
        current_battery_pct=22.0
    )
    # RTL threshold should be around 28-35% due to 3.5 km flight time + wind margin + 15% landing reserve
    assert rtl_far["rtl_threshold_pct"] > 25.0
    assert rtl_far["must_trigger_rtl"] is True, "Must trigger emergency RTL when battery < dynamic threshold"
    assert rtl_far["status"] == "CRITICAL_RTL_REQUIRED"
    print(f"  [✓] Dynamic RTL: Near = {rtl_near['rtl_threshold_pct']}% floor, Far (3.5km) = {rtl_far['rtl_threshold_pct']}%, Triggered: {rtl_far['must_trigger_rtl']}.")

if __name__ == "__main__":
    test_haversine_distance()
    test_synthetic_terrain_and_lidar_filtering()
    test_terrain_factor_and_classification()
    test_uav_physics_energy_and_vertical_penalty()
    test_dynamic_rtl_reserve()
    print("\n[SUCCESS] All Phase 04 UAV Physics, LiDAR & RTL Battery Tests Passed!")
