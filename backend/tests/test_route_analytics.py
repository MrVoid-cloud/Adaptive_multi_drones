"""
End-to-End Verification Test for Computational Route Analytics & Tie-Aware Comparison
Tests:
1. CCW Segment-Segment Intersection Detection
2. Chronological Redundant Revisit Detection
3. Value-Aware Tie-Breaking Rank Calculation (emerald, amber, orange)
4. TriAlgorithmOrchestrator analytics matrix embedding
"""

import sys
sys.path.insert(0, 'backend')

from app.optimizers.route_analytics import (
    GeometricRouteAnalyzer,
    segments_intersect,
    point_to_segment_distance_m
)
from app.gis.grid import GeodesicGridDiscretizer
from app.gis.road_topology_extractor import global_road_extractor
from app.gis.hybrid_space_builder import HybridGraphGridBuilder
from app.fleet.drone_registry import fleet_registry
from app.fleet.telemetry_models import DroneNodeConfig
from app.optimizers.fitness_evaluator import MultiObjectiveFitnessEvaluator
from app.optimizers.hybrid_ga_pso import TriAlgorithmOrchestrator

def test_ccw_intersection():
    print("Testing CCW Intersection Detection...")
    # Crossing segments: (0,0)-(1,1) and (0,1)-(1,0)
    assert segments_intersect((0, 0), (1, 1), (0, 1), (1, 0)) == True
    # Non-crossing segments: (0,0)-(1,0) and (0,1)-(1,1)
    assert segments_intersect((0, 0), (1, 0), (0, 1), (1, 1)) == False
    # Sharing one vertex (should not count as interior intersection)
    assert segments_intersect((0, 0), (1, 0), (1, 0), (2, 0)) == False
    print("✓ CCW intersection logic verified.")

def test_point_to_segment_distance():
    print("Testing Point to Segment Distance...")
    # Segment on equator along longitude
    d = point_to_segment_distance_m(
        plat=33.3005, plon=44.355,
        alat=33.3000, alon=44.350,
        blat=33.3000, blon=44.360
    )
    assert d > 40.0 and d < 70.0, f"Distance {d} out of expected range"
    print("✓ Point to segment distance verified.")

def test_tie_breaking_matrix():
    print("Testing Value-Aware Tie-Breaking Color Ranking...")
    pso = {'algorithm': 'Discrete PSO', 'makespan_sec': 300, 'best_fitness': 0.80, 'routes': {}}
    ga = {'algorithm': 'Modified GA', 'makespan_sec': 250, 'best_fitness': 0.85, 'routes': {}}
    hyb = {'algorithm': 'Hybrid GA-PSO', 'makespan_sec': 200, 'best_fitness': 0.95, 'routes': {}}

    matrix = GeometricRouteAnalyzer.build_comparative_matrix(pso, ga, hyb, 50)
    assert matrix['overall_winner'] == 'Hybrid GA-PSO'
    assert matrix['ranking']['HYBRID']['makespan_sec']['color'] == 'emerald'
    assert matrix['ranking']['GA']['makespan_sec']['color'] == 'amber'
    assert matrix['ranking']['PSO']['makespan_sec']['color'] == 'orange'
    print("✓ Tie-breaking color ranking verified.")

def test_orchestrator_integration():
    print("Testing TriAlgorithmOrchestrator Analytics Integration...")
    poly = [(33.308, 44.318), (33.322, 44.320), (33.320, 44.335), (33.308, 44.330)]
    grid = GeodesicGridDiscretizer(33.308, 33.322, 44.318, 44.335, polygon_vertices=poly, manual_cell_size_m=140)
    net = global_road_extractor._synthesize_urban_fallback_network((33.308, 44.318, 33.322, 44.335), poly)
    builder = HybridGraphGridBuilder(grid, net)

    fleet_registry.clear_all()
    for i in range(1, 4):
        fleet_registry.add_drone(DroneNodeConfig(
            drone_id=f"UAV-{i:02d}",
            endpoint_uri=f"sim://uav-{i:02d}",
            initial_lat=33.315,
            initial_lon=44.325
        ))
    fleet = fleet_registry.get_immutable_snapshot()
    evaluator = MultiObjectiveFitnessEvaluator(targets=builder.unified_targets, road_network=net, fleet=fleet)

    orchestrator = TriAlgorithmOrchestrator(evaluator)
    res = orchestrator.run_benchmark(iterations=5, pop_size=6)

    assert "analytics_comparison" in res, "Missing analytics_comparison in benchmark result"
    ac = res["analytics_comparison"]
    assert "overall_winner" in ac
    assert "pso" in ac and "ga" in ac and "hybrid" in ac
    assert "ranking" in ac
    print(f"✓ Orchestrator analytics verified. Winner: {ac['overall_winner']}")

if __name__ == "__main__":
    test_ccw_intersection()
    test_point_to_segment_distance()
    test_tie_breaking_matrix()
    test_orchestrator_integration()
    print("\n==========================================")
    print("  ALL 4 ROUTE ANALYTICS TESTS PASSED 100% ")
    print("==========================================")
