"""
Automated Test Suite for Phase 02: Graph-Grid Fitness & Hybrid GA-PSO Adaptation
Author: Principal Software Architect
"""

import sys
from pathlib import Path
import numpy as np

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from app.gis.hybrid_space_builder import SearchTargetNode
from app.gis.road_topology_extractor import (
    RoadIntersectionNode,
    RoadSegmentEdge,
    ExtractedRoadNetwork
)
from app.fleet.telemetry_models import DroneTelemetry, DroneStatus
from app.optimizers.fitness_evaluator import MultiObjectiveFitnessEvaluator
from app.optimizers.hybrid_ga_pso import HybridGAPSOOptimizer, TriAlgorithmOrchestrator
from app.optimizers.discrete_pso import DiscretePSOOptimizer
from app.optimizers.modified_ga import ModifiedGAOptimizer


def build_synthetic_hybrid_fixture():
    # 3 Drones at launch coordinates
    fleet = [
        DroneTelemetry(
            drone_id="UAV-01",
            endpoint_uri="sim://uav-01",
            latitude=33.3100,
            longitude=44.3600,
            altitude_m=0.0,
            speed_mps=14.0,
            battery_pct=100.0,
            status=DroneStatus.READY
        ),
        DroneTelemetry(
            drone_id="UAV-02",
            endpoint_uri="sim://uav-02",
            latitude=33.3100,
            longitude=44.3640,
            altitude_m=0.0,
            speed_mps=14.5,
            battery_pct=100.0,
            status=DroneStatus.READY
        ),
        DroneTelemetry(
            drone_id="UAV-03",
            endpoint_uri="sim://uav-03",
            latitude=33.3140,
            longitude=44.3620,
            altitude_m=0.0,
            speed_mps=15.0,
            battery_pct=100.0,
            status=DroneStatus.READY
        )
    ]

    # Road Network with 3 interconnected intersections
    road_nodes = {
        "ROAD_1": RoadIntersectionNode(node_id="ROAD_1", lat=33.3110, lon=44.3610, degree=3, adjacent_intersection_ids=["ROAD_2"]),
        "ROAD_2": RoadIntersectionNode(node_id="ROAD_2", lat=33.3125, lon=44.3625, degree=4, adjacent_intersection_ids=["ROAD_1", "ROAD_3"]),
        "ROAD_3": RoadIntersectionNode(node_id="ROAD_3", lat=33.3140, lon=44.3640, degree=3, adjacent_intersection_ids=["ROAD_2"])
    }
    road_edges = [
        RoadSegmentEdge(edge_id="E1", u="ROAD_1", v="ROAD_2", length_m=220.0, geometry=[(33.3110, 44.3610), (33.3125, 44.3625)]),
        RoadSegmentEdge(edge_id="E2", u="ROAD_2", v="ROAD_3", length_m=230.0, geometry=[(33.3125, 44.3625), (33.3140, 44.3640)])
    ]
    network = ExtractedRoadNetwork(
        nodes=road_nodes,
        edges=road_edges,
        bbox=(33.3100, 44.3600, 33.3150, 44.3650)
    )

    # Mixed Targets: 3 Road Intersections + 15 Open Terrain Cells
    targets = [
        SearchTargetNode(
            target_id="ROAD_1",
            node_type="ROAD_INTERSECTION",
            lat=33.3110,
            lon=44.3610,
            elevation_m=35.0,
            safe_alt_m=115.0, # Urban high clearance (80m AGL)
            priority=2.5,
            adjacent_node_ids=["ROAD_2"]
        ),
        SearchTargetNode(
            target_id="ROAD_2",
            node_type="ROAD_INTERSECTION",
            lat=33.3125,
            lon=44.3625,
            elevation_m=36.0,
            safe_alt_m=116.0,
            priority=3.0,
            adjacent_node_ids=["ROAD_1", "ROAD_3"]
        ),
        SearchTargetNode(
            target_id="ROAD_3",
            node_type="ROAD_INTERSECTION",
            lat=33.3140,
            lon=44.3640,
            elevation_m=37.0,
            safe_alt_m=117.0,
            priority=2.5,
            adjacent_node_ids=["ROAD_2"]
        )
    ]

    # Generate 15 open terrain cells
    cell_idx = 1
    for r in range(3):
        for c in range(5):
            c_lat = 33.3105 + (r * 0.0015)
            c_lon = 44.3605 + (c * 0.0010)
            targets.append(SearchTargetNode(
                target_id=f"CELL_{cell_idx:04d}",
                node_type="OPEN_TERRAIN_CELL",
                lat=c_lat,
                lon=c_lon,
                elevation_m=34.0,
                safe_alt_m=84.0, # Open terrain clearance (50m AGL)
                priority=1.0,
                terrain_roughness=1.0 + (r * 0.3),
                metadata={"cell_id": cell_idx, "row": r, "col": c}
            ))
            cell_idx += 1

    return fleet, network, targets


def test_dual_mode_evaluator_and_apsp():
    print("[*] Testing MultiObjectiveFitnessEvaluator with Dual-Mode Cost Engine...")
    fleet, network, targets = build_synthetic_hybrid_fixture()

    evaluator = MultiObjectiveFitnessEvaluator(
        targets=targets,
        road_network=network,
        fleet=fleet
    )

    print(f"  [✓] Evaluator initialized with {evaluator.m_cells} mixed targets and {evaluator.n_drones} drones.")
    assert evaluator.m_cells == 18
    assert evaluator.n_drones == 3

    # Check APSP: ROAD_1 to ROAD_3 through ROAD_2 should be 220 + 230 = 450m
    pair = ("ROAD_1", "ROAD_3")
    assert pair in evaluator.road_apsp, "APSP between ROAD_1 and ROAD_3 not found!"
    assert abs(evaluator.road_apsp[pair] - 450.0) < 1e-3, f"Expected 450m, got {evaluator.road_apsp[pair]}"
    print(f"  [✓] APSP Path ROAD_1 -> ROAD_3 verified: {evaluator.road_apsp[pair]}m")

    # Evaluate balanced dummy chromosome
    chromosome = np.array([i % 3 for i in range(18)])
    fitness, metrics = evaluator.evaluate(chromosome)
    print(f"  [✓] Evaluator computed fitness: {fitness:.2f} (Makespan: {metrics['makespan_sec']:.1f}s, Energy: {metrics['total_energy_wh']:.1f}Wh)")
    assert fitness > 0
    assert metrics["makespan_sec"] > 0
    assert metrics["total_energy_wh"] > 0


def test_altitude_yo_yo_suppression():
    print("[*] Testing Nearest-Neighbor Route Sequencer & Altitude Yo-Yo Suppression...")
    fleet, network, targets = build_synthetic_hybrid_fixture()
    evaluator = MultiObjectiveFitnessEvaluator(
        targets=targets,
        road_network=network,
        fleet=fleet,
        altitude_switch_penalty_factor=5.0
    )

    # Assign Drone 0 all 3 road nodes + 3 open cells
    assigned = [t for t in targets if t.node_type == "ROAD_INTERSECTION"] + [t for t in targets if t.node_type == "OPEN_TERRAIN_CELL"][:3]
    ordered_route, total_time, total_energy, rtl_thresh = evaluator.sort_route_nearest_neighbor(0, assigned)

    print(f"  [✓] Ordered route generated with {len(ordered_route)} waypoints.")
    # Check that road nodes are clustered together rather than interleaved with open terrain cells
    node_types = [w.node_type for w in ordered_route]
    print(f"  [✓] Route node sequence types: {node_types}")

    # Count transitions between different node types (should be minimized, ideally 1 switch)
    switches = sum(1 for i in range(len(node_types) - 1) if node_types[i] != node_types[i+1])
    print(f"  [✓] Zone switches count: {switches} (Low switch count confirms anti-yo-yo clustering)")
    assert switches <= 2, f"Too many altitude yo-yo switches ({switches}) detected!"


def test_hybrid_ga_pso_optimization():
    print("[*] Testing HybridGAPSOOptimizer with Context-Aware Voronoi & Spatial Memetics...")
    fleet, network, targets = build_synthetic_hybrid_fixture()
    evaluator = MultiObjectiveFitnessEvaluator(
        targets=targets,
        road_network=network,
        fleet=fleet
    )

    optimizer = HybridGAPSOOptimizer(
        evaluator=evaluator,
        generations=15,
        pop_swarm_size=20,
        seed=42
    )

    res = optimizer.optimize()
    print(f"  [✓] Optimization finished in {res['compute_time_ms']} ms.")
    print(f"  [✓] Best fitness: {res['best_fitness']}, Makespan: {res['makespan_sec']} s, Energy: {res['total_energy_wh']} Wh")
    assert res["best_fitness"] > 0
    assert len(res["convergence"]) == 16  # Gen 0 + 15 generations

    # Verify route serialization format includes required keys: node_type, safe_alt_m, target_id
    routes = res["routes"]
    assert len(routes) == 3
    for drone_id, waypoints in routes.items():
        assert len(waypoints) > 0, f"Drone {drone_id} received 0 waypoints!"
        for wp in waypoints:
            assert "node_type" in wp, "Missing node_type in waypoint!"
            assert "safe_alt_m" in wp, "Missing safe_alt_m in waypoint!"
            assert "target_id" in wp, "Missing target_id in waypoint!"
            assert wp["node_type"] in ["ROAD_INTERSECTION", "OPEN_TERRAIN_CELL"]
    print("  [✓] Route serialization conforms 100% with required schema.")


def test_tri_algorithm_orchestrator():
    print("[*] Testing TriAlgorithmOrchestrator Benchmark Suite...")
    fleet, network, targets = build_synthetic_hybrid_fixture()
    evaluator = MultiObjectiveFitnessEvaluator(
        targets=targets,
        road_network=network,
        fleet=fleet
    )

    orchestrator = TriAlgorithmOrchestrator(evaluator)
    res = orchestrator.run_benchmark(iterations=12, pop_size=15)

    assert res["status"] == "TRI_ALGORITHM_OPTIMIZATION_COMPLETE"
    assert "pso_result" in res
    assert "ga_result" in res
    assert "hybrid_result" in res
    assert "kpi_comparison" in res
    print(f"  [✓] Tri-algorithm benchmark completed.")
    print(f"  [✓] Makespan reduction vs PSO: {res['kpi_comparison']['makespan_reduction_vs_pso_pct']}%")
    print(f"  [✓] Makespan reduction vs GA: {res['kpi_comparison']['makespan_reduction_vs_ga_pct']}%")
    print("  [✓] Benchmark verified successfully.")


if __name__ == "__main__":
    test_dual_mode_evaluator_and_apsp()
    test_altitude_yo_yo_suppression()
    test_hybrid_ga_pso_optimization()
    test_tri_algorithm_orchestrator()
    print("\n[SUCCESS] All Phase 02 Graph-Grid Fitness & Hybrid GA-PSO Adaptation Tests Passed!")
