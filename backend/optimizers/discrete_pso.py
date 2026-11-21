"""
Discrete Particle Swarm Optimization (Discrete PSO) for Multi-UAV Mission Planning
N-Agnostic Fleet Scalability, Single-UAV Optimization, 2-Opt Route Untangling
Author: Principal Software Architect
"""

import time
import numpy as np
from typing import Dict, Any, List, Optional
from app.optimizers.fitness_evaluator import MultiObjectiveFitnessEvaluator

class DiscretePSOOptimizer:
    """
    Standalone Discrete Particle Swarm Optimizer.
    N-Agnostic: Fully supports single UAV (N=1) or swarms (N >= 6).
    Uses probabilistic velocity & coordinate updates on integer chromosomes
    to find an optimal balanced coverage partition.
    """
    def __init__(
        self,
        evaluator: MultiObjectiveFitnessEvaluator,
        swarm_size: int = 30,
        max_iterations: int = 40,
        c_cognitive: float = 0.35,
        c_social: float = 0.45,
        c_mutation: float = 0.05,
        seed: Optional[int] = 42
    ):
        self.evaluator = evaluator
        self.m = evaluator.m_cells
        self.n = evaluator.n_drones
        self.n_eff = min(self.n, max(1, self.m)) if self.n > 0 else 0

        # Adaptive hyperparameter scaling (respects explicit caller settings)
        self.swarm_size = swarm_size if swarm_size and swarm_size > 0 else max(15, min(40, self.n_eff * 6))
        self.max_iterations = max_iterations if max_iterations and max_iterations > 0 else max(15, min(40, self.n_eff * 6))

        self.c1 = c_cognitive
        self.c2 = c_social
        self.c_mut = c_mutation
        self.rng = np.random.RandomState(seed)

    def optimize(
        self,
        warm_start_particles: Optional[List[np.ndarray]] = None
    ) -> Dict[str, Any]:
        """
        Executes Discrete PSO search loop.
        Single-UAV mode (N_eff = 1): directly builds optimal untangled tour.
        """
        t_start = time.perf_counter()

        if self.m == 0 or self.n == 0:
            return {"error": "Empty search space or zero active drones."}

        # Fast path for Single-UAV mode
        if self.n_eff == 1:
            sol = np.zeros(self.m, dtype=int)
            fit, metrics = self.evaluator.evaluate(sol)
            elapsed_ms = (time.perf_counter() - t_start) * 1000.0
            return {
                "algorithm": "Discrete PSO",
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

        # 1. Swarm Initialization
        particles = np.zeros((self.swarm_size, self.m), dtype=int)
        for i in range(self.swarm_size):
            if warm_start_particles and i < len(warm_start_particles):
                particles[i] = np.copy(warm_start_particles[i])
            elif i == 0:
                particles[i] = self._generate_longitude_quantile_seed()
            elif i == 1:
                particles[i] = self._generate_polar_quantile_seed()
            else:
                particles[i] = self.rng.randint(0, self.n_eff, size=self.m)
            particles[i] = self.evaluator.repair_chromosome(particles[i], self.n)

        pbest = np.copy(particles)
        pbest_scores = np.zeros(self.swarm_size)

        gbest = None
        gbest_score = -float('inf')
        gbest_metrics: Dict[str, Any] = {}
        convergence_curve: List[float] = []

        # Evaluate initial swarm
        for i in range(self.swarm_size):
            fit, metrics = self.evaluator.evaluate(particles[i])
            pbest_scores[i] = fit
            if fit > gbest_score:
                gbest_score = fit
                gbest = np.copy(particles[i])
                gbest_metrics = metrics

        convergence_curve.append(round(gbest_score, 2))

        # 2. Main Optimization Loop
        for iteration in range(self.max_iterations):
            for i in range(self.swarm_size):
                r = self.rng.rand(self.m)
                mask_pbest = (r < self.c1)
                mask_gbest = ((r >= self.c1) & (r < self.c1 + self.c2))
                mask_mut = ((r >= self.c1 + self.c2) & (r < self.c1 + self.c2 + self.c_mut))

                particles[i][mask_pbest] = pbest[i][mask_pbest]
                if gbest is not None:
                    particles[i][mask_gbest] = gbest[mask_gbest]
                if np.any(mask_mut):
                    particles[i][mask_mut] = self.rng.randint(0, self.n_eff, size=np.sum(mask_mut))

                particles[i] = self.evaluator.repair_chromosome(particles[i], self.n)

                fit, metrics = self.evaluator.evaluate(particles[i])
                if fit > pbest_scores[i]:
                    pbest_scores[i] = fit
                    pbest[i] = np.copy(particles[i])

                if fit > gbest_score:
                    gbest_score = fit
                    gbest = np.copy(particles[i])
                    gbest_metrics = metrics

            convergence_curve.append(round(gbest_score, 2))

        # Final Untangling Refinement Pass on Global Best Solution
        if gbest is not None:
            gbest_score, gbest_metrics = self.evaluator.evaluate(gbest, final_pass=True)

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        serialized_routes = self._serialize_routes(gbest_metrics.get("routes", {}))

        return {
            "algorithm": "Discrete PSO",
            "chromosome": gbest.tolist() if gbest is not None else [],
            "best_fitness": round(gbest_score, 2),
            "makespan_sec": gbest_metrics.get("makespan_sec", 0.0),
            "total_energy_wh": gbest_metrics.get("total_energy_wh", 0.0),
            "workload_std_sec": gbest_metrics.get("workload_std_sec", 0.0),
            "priority_covered": gbest_metrics.get("priority_covered", 0.0),
            "convergence": convergence_curve,
            "compute_time_ms": round(elapsed_ms, 2),
            "total_sorties": gbest_metrics.get("total_sorties", 1),
            "sorties_per_drone": gbest_metrics.get("sorties_per_drone", {}),
            "total_physical_cells": gbest_metrics.get("total_physical_cells", getattr(self.evaluator, "total_physical_cells", self.m)),
            "covered_physical_cells": gbest_metrics.get("covered_physical_cells", getattr(self.evaluator, "total_physical_cells", self.m)),
            "physical_coverage_pct": gbest_metrics.get("physical_coverage_pct", 100.0),
            "compressed_strips_count": gbest_metrics.get("compressed_strips_count", getattr(self.evaluator, "compressed_strips_count", 0)),
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
