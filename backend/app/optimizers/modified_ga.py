"""
Modified Genetic Algorithm (Modified GA) for Multi-UAV Mission Planning
N-Agnostic Fleet Scalability, Polar Quantile Seeding, 2-Opt Route Untangling
Author: Principal Software Architect
"""

import time
import numpy as np
from typing import Dict, Any, List, Optional
from app.optimizers.fitness_evaluator import MultiObjectiveFitnessEvaluator

class ModifiedGAOptimizer:
    """
    Standalone Modified Genetic Algorithm.
    N-Agnostic: Fully supports single UAV (N=1) or swarms (N >= 6).
    Employs 25% Elitism, Tournament Selection, Uniform Crossover,
    and Dual Mutation (Swap + Uniform) to maintain population diversity.
    """
    def __init__(
        self,
        evaluator: MultiObjectiveFitnessEvaluator,
        population_size: int = 40,
        generations: int = 40,
        crossover_rate: float = 0.85,
        mutation_rate: float = 0.08,
        elitism_ratio: float = 0.25,
        seed: Optional[int] = 42
    ):
        self.evaluator = evaluator
        self.m = evaluator.m_cells
        self.n = evaluator.n_drones
        self.n_eff = min(self.n, max(1, self.m)) if self.n > 0 else 0

        # Adaptive hyperparameter scaling (respects explicit caller settings)
        self.pop_size = population_size if population_size and population_size > 0 else max(15, min(40, self.n_eff * 6))
        self.generations = generations if generations and generations > 0 else max(15, min(40, self.n_eff * 6))

        self.p_cross = crossover_rate
        self.p_mut = mutation_rate
        self.n_elites = max(1, int(self.pop_size * elitism_ratio))
        self.rng = np.random.RandomState(seed)

    def optimize(self) -> Dict[str, Any]:
        """Runs the GA evolutionary loop and returns optimal individual + metrics."""
        t_start = time.perf_counter()

        if self.m == 0 or self.n == 0:
            return {"error": "Empty search space or zero active drones."}

        # Fast path for Single-UAV mode
        if self.n_eff == 1:
            sol = np.zeros(self.m, dtype=int)
            fit, metrics = self.evaluator.evaluate(sol)
            elapsed_ms = (time.perf_counter() - t_start) * 1000.0
            return {
                "algorithm": "Modified GA",
                "chromosome": sol.tolist(),
                "best_fitness": round(fit, 2),
                "makespan_sec": metrics.get("makespan_sec", 0.0),
                "total_energy_wh": metrics.get("total_energy_wh", 0.0),
                "workload_std_sec": 0.0,
                "priority_covered": metrics.get("priority_covered", 0.0),
                "convergence": [round(fit, 2)],
                "compute_time_ms": round(elapsed_ms, 2),
                "routes": self._serialize_routes(metrics.get("routes", {}))
            }

        # 1. Initialize Population
        population = np.zeros((self.pop_size, self.m), dtype=int)
        for i in range(self.pop_size):
            if i == 0:
                population[i] = self._generate_longitude_quantile_seed()
            elif i == 1:
                population[i] = self._generate_polar_quantile_seed()
            else:
                population[i] = self.rng.randint(0, self.n_eff, size=self.m)
            population[i] = self.evaluator.repair_chromosome(population[i], self.n)

        fitness_scores = np.zeros(self.pop_size)
        all_metrics: List[Dict[str, Any]] = [{} for _ in range(self.pop_size)]

        for i in range(self.pop_size):
            fit, metrics = self.evaluator.evaluate(population[i])
            fitness_scores[i] = fit
            all_metrics[i] = metrics

        convergence_curve: List[float] = [round(float(np.max(fitness_scores)), 2)]

        # 2. Generational Evolution
        for gen in range(self.generations):
            sorted_indices = np.argsort(-fitness_scores)
            population = population[sorted_indices]
            fitness_scores = fitness_scores[sorted_indices]
            all_metrics = [all_metrics[idx] for idx in sorted_indices]

            next_population = np.zeros_like(population)

            # Preserve Elites
            for e in range(self.n_elites):
                next_population[e] = np.copy(population[e])

            # Fill remaining slots via Selection -> Crossover -> Mutation
            current_idx = self.n_elites
            while current_idx < self.pop_size:
                p1 = self._tournament_select(population, fitness_scores, k=3)
                p2 = self._tournament_select(population, fitness_scores, k=3)

                # Crossover
                if self.rng.rand() < self.p_cross:
                    mask = self.rng.rand(self.m) < 0.5
                    child1 = np.where(mask, p1, p2)
                    child2 = np.where(mask, p2, p1)
                else:
                    child1, child2 = np.copy(p1), np.copy(p2)

                # Mutation
                child1 = self._mutate(child1)
                child2 = self._mutate(child2)

                # Constraint Repair
                child1 = self.evaluator.repair_chromosome(child1, self.n)
                child2 = self.evaluator.repair_chromosome(child2, self.n)

                next_population[current_idx] = child1
                current_idx += 1
                if current_idx < self.pop_size:
                    next_population[current_idx] = child2
                    current_idx += 1

            population = next_population
            for i in range(self.pop_size):
                fit, metrics = self.evaluator.evaluate(population[i])
                fitness_scores[i] = fit
                all_metrics[i] = metrics

            convergence_curve.append(round(float(np.max(fitness_scores)), 2))

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        best_idx = int(np.argmax(fitness_scores))
        best_sol = population[best_idx]
        # Final Untangling Refinement Pass on Global Best Solution
        best_fit, best_metrics = self.evaluator.evaluate(best_sol, final_pass=True)
        fitness_scores[best_idx] = best_fit
        serialized_routes = self._serialize_routes(best_metrics.get("routes", {}))

        return {
            "algorithm": "Modified GA",
            "chromosome": best_sol.tolist(),
            "best_fitness": round(float(fitness_scores[best_idx]), 2),
            "makespan_sec": best_metrics.get("makespan_sec", 0.0),
            "total_energy_wh": best_metrics.get("total_energy_wh", 0.0),
            "workload_std_sec": best_metrics.get("workload_std_sec", 0.0),
            "priority_covered": best_metrics.get("priority_covered", 0.0),
            "convergence": convergence_curve,
            "compute_time_ms": round(elapsed_ms, 2),
            "total_sorties": best_metrics.get("total_sorties", 1),
            "sorties_per_drone": best_metrics.get("sorties_per_drone", {}),
            "total_physical_cells": best_metrics.get("total_physical_cells", getattr(self.evaluator, "total_physical_cells", self.m)),
            "covered_physical_cells": best_metrics.get("covered_physical_cells", getattr(self.evaluator, "total_physical_cells", self.m)),
            "physical_coverage_pct": best_metrics.get("physical_coverage_pct", 100.0),
            "compressed_strips_count": best_metrics.get("compressed_strips_count", getattr(self.evaluator, "compressed_strips_count", 0)),
            "routes": serialized_routes
        }

    def _generate_longitude_quantile_seed(self) -> np.ndarray:
        """Equal-Cardinality Longitude Quantile Seeding across West-East sectors."""
        chromosome = np.zeros(self.m, dtype=int)
        lons = np.array([t.lon for t in self.evaluator.active_targets])
        sorted_indices = np.argsort(lons)

        chunk_size = self.m // self.n_eff
        remainder = self.m % self.n_eff
        cur_pos = 0
        for d in range(self.n_eff):
            take = chunk_size + (1 if d < remainder else 0)
            chromosome[sorted_indices[cur_pos:cur_pos + take]] = d
            cur_pos += take
        return chromosome

    def _generate_polar_quantile_seed(self) -> np.ndarray:
        """Equal-Cardinality Polar Quantile Seeding around Shared Home Base."""
        chromosome = np.zeros(self.m, dtype=int)
        base_lat = self.evaluator.fleet[0].latitude if self.evaluator.fleet else 33.3100
        base_lon = self.evaluator.fleet[0].longitude if self.evaluator.fleet else 44.3600
        cos_base = np.cos(np.radians(base_lat))

        lats = np.array([t.lat for t in self.evaluator.active_targets])
        lons = np.array([t.lon for t in self.evaluator.active_targets])
        angles = np.arctan2(lats - base_lat, (lons - base_lon) * cos_base)
        sorted_indices = np.argsort(angles)

        chunk_size = self.m // self.n_eff
        remainder = self.m % self.n_eff
        cur_pos = 0
        for d in range(self.n_eff):
            take = chunk_size + (1 if d < remainder else 0)
            chromosome[sorted_indices[cur_pos:cur_pos + take]] = d
            cur_pos += take
        return chromosome

    def _tournament_select(self, pop: np.ndarray, fits: np.ndarray, k: int = 3) -> np.ndarray:
        candidates = self.rng.choice(len(fits), size=min(k, len(fits)), replace=False)
        best_candidate = candidates[np.argmax(fits[candidates])]
        return np.copy(pop[best_candidate])

    def _mutate(self, chromosome: np.ndarray) -> np.ndarray:
        mutated = np.copy(chromosome)
        if self.rng.rand() < self.p_mut:
            mut_genes = self.rng.choice(self.m, size=max(1, int(self.m * 0.05)), replace=False)
            mutated[mut_genes] = self.rng.randint(0, self.n_eff, size=len(mut_genes))
        if self.rng.rand() < (self.p_mut * 0.5) and self.m >= 2:
            i, j = self.rng.choice(self.m, size=2, replace=False)
            mutated[i], mutated[j] = mutated[j], mutated[i]
        return mutated

    def _serialize_routes(self, routes: Dict[int, list]) -> Dict[str, list]:
        serialized: Dict[str, list] = {}
        for drone_idx in range(self.n):
            drone_id = self.evaluator.fleet[drone_idx].drone_id if drone_idx < len(self.evaluator.fleet) else f"UAV-{drone_idx+1:02d}"
            target_list = routes.get(drone_idx, [])
            serialized[drone_id] = [
                {
                    "cell_id": getattr(t, "target_id", getattr(t, "cell_id", f"NODE_{idx}")),
                    "target_id": getattr(t, "target_id", f"TARGET_{idx}"),
                    "node_type": getattr(t, "node_type", "OPEN_TERRAIN_CELL"),
                    "waypoint_type": getattr(t, "waypoint_type", "TARGET"),
                    "sortie_index": getattr(t, "sortie_index", 1),
                    "row": getattr(t, "row", 0),
                    "col": getattr(t, "col", 0),
                    "lat": round(getattr(t, "center_lat", getattr(t, "lat", 0.0)), 6),
                    "lon": round(getattr(t, "center_lon", getattr(t, "lon", 0.0)), 6),
                    "alt_m": round(getattr(t, "safe_alt_m", getattr(t, "elevation_m", 0.0) + 50.0), 1),
                    "safe_alt_m": round(getattr(t, "safe_alt_m", getattr(t, "elevation_m", 0.0) + 50.0), 1),
                    "elevation_m": round(getattr(t, "elevation_m", 0.0), 1),
                    "roughness": round(getattr(t, "terrain_roughness", 1.0), 2),
                    "priority": round(getattr(t, "priority", 1.0), 2),
                    "metadata": getattr(t, "metadata", {}),
                    "covered_cell_ids": getattr(t, "metadata", {}).get("covered_cell_ids", []),
                    "covered_cells_count": getattr(t, "metadata", {}).get("covered_cells_count", 1)
                }
                for idx, t in enumerate(target_list)
            ]
        return serialized
