import sys
import time
from pathlib import Path

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from app.gis.grid import GeodesicGridDiscretizer
from app.fleet.telemetry_models import DroneNodeConfig
from app.fleet.drone_registry import DynamicDroneRegistry
from app.optimizers.fitness_evaluator import MultiObjectiveFitnessEvaluator
from app.optimizers.discrete_pso import DiscretePSOOptimizer
from app.optimizers.modified_ga import ModifiedGAOptimizer
from app.optimizers.hybrid_ga_pso import HybridGAPSOOptimizer, TriAlgorithmOrchestrator
from app.api.v1.optimize_routes import compare_tri_algorithms, OptimizationRequestSchema

def setup_test_context():
    # 1. Grid of ~36 cells
    grid = GeodesicGridDiscretizer(
        lat_min=33.3000,
        lat_max=33.3100,
        lon_min=44.3500,
        lon_max=44.3650,
        manual_cell_size_m=120.0
    )

    # 2. Fleet of 3 UAVs
    registry = DynamicDroneRegistry()
    for i in range(1, 4):
        registry.add_drone(DroneNodeConfig(
            drone_id=f"UAV-0{i}",
            endpoint_uri=f"sim://uav-0{i}",
            cruise_speed_mps=14.0,
            battery_capacity_wh=120.0,
            initial_lat=33.3050,
            initial_lon=44.3575
        ))
    fleet = registry.get_immutable_snapshot()
    return grid, fleet

def test_fitness_evaluator_and_repair():
    print("[*] Testing Unified Multi-Objective Fitness Evaluator & Nearest-Neighbor...")
    grid, fleet = setup_test_context()
    evaluator = MultiObjectiveFitnessEvaluator(cells=grid.cells, fleet=fleet)
    
    assert evaluator.m_cells > 0
    assert evaluator.n_drones == 3

    # Degenerate chromosome where all cells assigned to drone 0
    bad_chromosome = [0] * evaluator.m_cells
    repaired = evaluator.repair_chromosome(bad_chromosome, evaluator.n_drones)
    assert 1 in repaired
    assert 2 in repaired
    print(f"  [✓] Chromosome Repair Heuristic rebalanced {evaluator.m_cells} cells across all 3 drones.")

    # Evaluate repaired chromosome
    fitness, metrics = evaluator.evaluate(repaired)
    assert fitness > 0.0
    assert metrics["makespan_sec"] > 0.0
    assert metrics["total_energy_wh"] > 0.0
    assert len(metrics["routes"]) == 3
    print(f"  [✓] Evaluated: Fitness = {fitness:.2f}, Makespan = {metrics['makespan_sec']:.1f}s, Energy = {metrics['total_energy_wh']:.2f}Wh.")

def test_discrete_pso():
    print("[*] Testing Standalone Discrete PSO Engine...")
    grid, fleet = setup_test_context()
    evaluator = MultiObjectiveFitnessEvaluator(cells=grid.cells, fleet=fleet)

    pso = DiscretePSOOptimizer(evaluator=evaluator, swarm_size=20, max_iterations=15, seed=42)
    res = pso.optimize()

    assert res["algorithm"] == "Discrete PSO"
    assert res["best_fitness"] > 0.0
    assert len(res["convergence"]) == 16
    assert len(res["routes"]) == 3
    print(f"  [✓] Discrete PSO finished in {res['compute_time_ms']:.1f}ms: Fitness = {res['best_fitness']}, Makespan = {res['makespan_sec']}s.")

def test_modified_ga():
    print("[*] Testing Standalone Modified GA Engine...")
    grid, fleet = setup_test_context()
    evaluator = MultiObjectiveFitnessEvaluator(cells=grid.cells, fleet=fleet)

    ga = ModifiedGAOptimizer(evaluator=evaluator, population_size=20, generations=15, seed=42)
    res = ga.optimize()

    assert res["algorithm"] == "Modified GA"
    assert res["best_fitness"] > 0.0
    assert len(res["convergence"]) == 16
    assert len(res["routes"]) == 3
    print(f"  [✓] Modified GA finished in {res['compute_time_ms']:.1f}ms: Fitness = {res['best_fitness']}, Makespan = {res['makespan_sec']}s.")

def test_hybrid_ga_pso_and_orchestrator():
    print("[*] Testing Hybrid GA-PSO & Tri-Algorithm Orchestrator...")
    grid, fleet = setup_test_context()
    evaluator = MultiObjectiveFitnessEvaluator(cells=grid.cells, fleet=fleet)

    orchestrator = TriAlgorithmOrchestrator(evaluator)
    benchmark = orchestrator.run_benchmark(iterations=12, pop_size=16)

    assert benchmark["status"] == "TRI_ALGORITHM_OPTIMIZATION_COMPLETE"
    assert "pso_result" in benchmark
    assert "ga_result" in benchmark
    assert "hybrid_result" in benchmark
    assert "kpi_comparison" in benchmark

    pso_fit = benchmark["pso_result"]["best_fitness"]
    ga_fit = benchmark["ga_result"]["best_fitness"]
    hyb_fit = benchmark["hybrid_result"]["best_fitness"]

    print(f"  [✓] Fitness Benchmark -> PSO: {pso_fit} | GA: {ga_fit} | Hybrid GA-PSO: {hyb_fit}")
    print(f"  [✓] Makespan Improvement: {benchmark['kpi_comparison']['makespan_reduction_vs_pso_pct']}% vs standalone PSO.")

def test_api_endpoint():
    print("[*] Testing POST /api/v1/optimize/compare Endpoint...")
    req = OptimizationRequestSchema(iterations=10, population_size=12)
    response = compare_tri_algorithms(req)
    assert response["status"] == "TRI_ALGORITHM_OPTIMIZATION_COMPLETE"
    assert "hybrid_result" in response
    print("  [✓] REST API endpoint validated successfully.")

if __name__ == "__main__":
    test_fitness_evaluator_and_repair()
    test_discrete_pso()
    test_modified_ga()
    test_hybrid_ga_pso_and_orchestrator()
    test_api_endpoint()
    print("\n[SUCCESS] All Phase 06 Tri-Algorithm Optimization Tests Passed!")
