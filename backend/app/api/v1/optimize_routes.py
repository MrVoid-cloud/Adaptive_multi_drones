import asyncio
import time
import logging
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import Dict, Any, Optional, List

logger = logging.getLogger("OptimizationRouter")
from app.gis.grid import GeodesicGridDiscretizer
import app.api.v1.grid_routes as grid_routes
import app.api.v1.map_routes as map_routes
from app.core.mission_state import mission_state
from app.fleet.drone_registry import fleet_registry
from app.fleet.telemetry_models import DroneNodeConfig, DroneStatus
from app.optimizers.fitness_evaluator import MultiObjectiveFitnessEvaluator
from app.optimizers.hybrid_ga_pso import TriAlgorithmOrchestrator

router = APIRouter(prefix="/optimize", tags=["Tri-Algorithm Optimization Engine"])

class OptimizationRequestSchema(BaseModel):
    mission_id: Optional[str] = Field(default=None, description="Active session epoch ID (UUID)")
    iterations: int = Field(default=18, ge=5, le=150, description="Iterations / Generations per algorithm")
    population_size: int = Field(default=18, ge=6, le=100, description="Swarm / Population size")
    weights: Optional[Dict[str, float]] = None
    fleet_size: Optional[int] = Field(default=None, ge=1, le=8, description="Operator-defined UAV count")

@router.post(
    "/compare",
    summary="Execute and Benchmark Tri-Algorithm Suite (Discrete PSO, Modified GA, Hybrid GA-PSO)"
)
async def compare_tri_algorithms(payload: OptimizationRequestSchema):
    """
    Executes all three optimization engines on the exact same mission context.
    Validates mission_id consistency to eliminate stale calculation drift.
    Returns results ordered: 1. Discrete PSO, 2. Modified GA, 3. Hybrid GA-PSO.
    """
    # 0. Session Epoch Consistency Check
    current_mission_id = mission_state.mission_id
    if payload.mission_id and current_mission_id and payload.mission_id != current_mission_id:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"Stale Mission ID '{payload.mission_id}'. Active session is '{current_mission_id}'."
        )
    # 1. Resolve Active Grid (dynamically always read latest user drawn ROI)
    grid = grid_routes._ACTIVE_GRID
    if (grid is None or grid.active_cell_count == 0) and map_routes._ACTIVE_MISSION_ROI:
        req = map_routes._ACTIVE_MISSION_ROI.get("request", {})
        b = req.get("bounds", {})
        poly_verts = [(v.get("lat", v.get("latitude")), v.get("lon", v.get("longitude"))) for v in req.get("vertices", [])] if req.get("vertices") else None
        grid = GeodesicGridDiscretizer(
            lat_min=b.get("lat_min", 33.3000),
            lat_max=b.get("lat_max", 33.3150),
            lon_min=b.get("lon_min", 44.3500),
            lon_max=b.get("lon_max", 44.3700),
            polygon_vertices=poly_verts,
            manual_cell_size_m=120.0
        )
        grid_routes._ACTIVE_GRID = grid
    elif grid is None or grid.active_cell_count == 0:
        # Default fallback test grid for initial dashboard loading
        grid = GeodesicGridDiscretizer(
            lat_min=33.3000,
            lat_max=33.3150,
            lon_min=44.3500,
            lon_max=44.3700,
            manual_cell_size_m=120.0
        )

    # 2. Dynamic Fleet Sizing: Adjust fleet to requested size and place at active grid/targets center
    current_configs = fleet_registry.get_ordered_configs()
    target_count = payload.fleet_size if payload.fleet_size is not None else (len(current_configs) if current_configs else 3)
    
    if getattr(grid_routes, "_ACTIVE_TARGETS", None) and len(grid_routes._ACTIVE_TARGETS) > 0:
        mean_lat = sum(t.lat for t in grid_routes._ACTIVE_TARGETS) / len(grid_routes._ACTIVE_TARGETS)
        mean_lon = sum(t.lon for t in grid_routes._ACTIVE_TARGETS) / len(grid_routes._ACTIVE_TARGETS)
    else:
        mean_lat = grid.mean_lat if grid else 33.3075
        mean_lon = (grid.lon_min + grid.lon_max) / 2 if grid else 44.3600

    fleet_registry.clear_all()
    for i in range(1, target_count + 1):
        fleet_registry.add_drone(DroneNodeConfig(
            drone_id=f"UAV-{i:02d}",
            endpoint_uri=f"sim://uav-{i:02d}",
            cruise_speed_mps=14.0 + (i * 0.4),
            battery_capacity_wh=120.0,
            initial_lat=mean_lat,
            initial_lon=mean_lon
        ))
    fleet = fleet_registry.get_immutable_snapshot()

    # 3. Instantiate Unified Fitness Evaluator (Supports mixed hybrid targets or standard grid cells)
    t_start = time.time()
    try:
        if getattr(grid_routes, "_ACTIVE_TARGETS", None) and len(grid_routes._ACTIVE_TARGETS) > 0:
            target_list = grid_routes._ACTIVE_TARGETS
            road_net = grid_routes._ACTIVE_HYBRID_BUILDER.road_network if getattr(grid_routes, "_ACTIVE_HYBRID_BUILDER", None) else None
            evaluator = MultiObjectiveFitnessEvaluator(
                targets=target_list,
                road_network=road_net,
                fleet=fleet,
                weights=payload.weights
            )
        else:
            evaluator = MultiObjectiveFitnessEvaluator(
                cells=grid.cells,
                fleet=fleet,
                weights=payload.weights
            )

        if evaluator.m_cells == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="No active search cells found in grid. All cells might be masked out."
            )

        t_eval_init = time.time() - t_start
        logger.info(
            f"[Optimizer] Evaluator initialized in {t_eval_init * 1000:.1f}ms for {evaluator.m_cells} targets "
            f"across {len(fleet)} UAVs."
        )

        # 4. Orchestrate Benchmark Run in Non-Blocking Thread Pool
        orchestrator = TriAlgorithmOrchestrator(evaluator)
        t_orch_start = time.time()
        benchmark_result = await asyncio.to_thread(
            orchestrator.run_benchmark,
            iterations=payload.iterations,
            pop_size=payload.population_size
        )
        t_orch_duration = time.time() - t_orch_start
        t_total = time.time() - t_start

        logger.info(
            f"[Optimizer] Tri-Algorithm benchmark finished in {t_orch_duration:.2f}s "
            f"(Total API: {t_total:.2f}s). Best: {benchmark_result.get('kpi_comparison', {}).get('recommended_algorithm')}"
        )

        # Attach verified session epoch metadata
        benchmark_result["mission_id"] = current_mission_id
        mission_state.last_benchmark_results = benchmark_result

        return benchmark_result

    except HTTPException:
        raise
    except Exception as exc:
        logger.error(f"[Optimizer] Unexpected error during benchmark optimization: {exc}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Optimization benchmark engine error: {str(exc)}"
        )
