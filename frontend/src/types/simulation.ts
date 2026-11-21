/**
 * Unified Simulation, Telemetry & Multi-Algorithm TypeScript Models
 * Phase 09: Segmented Tri-Algorithm Output & Live Multi-Layer Visualization
 */

export interface Waypoint {
  cell_id: number | string;
  target_id?: string;
  node_type?: 'ROAD_INTERSECTION' | 'OPEN_TERRAIN_CELL';
  waypoint_type?: 'TARGET' | 'SORTIE_RETURN_BASE' | 'SORTIE_RELAUNCH_BASE' | 'TARGET_STRIP_ENTRY' | 'TARGET_STRIP_EXIT' | string;
  sortie_index?: number;
  row: number;
  col: number;
  lat: number;
  lon: number;
  alt_m: number;
  safe_alt_m?: number;
  priority: number;
  roughness: number;
  elevation_m?: number;
  metadata?: Record<string, any>;
  covered_cell_ids?: string[];
  covered_cells_count?: number;
  is_strip_entry?: boolean;
  is_strip_exit?: boolean;
  strip_id?: string;
}

export interface AlgorithmResult {
  algorithm: 'Discrete PSO' | 'Modified GA' | 'Hybrid GA-PSO' | string;
  chromosome: number[];
  best_fitness: number;
  makespan_sec: number;
  total_energy_wh: number;
  workload_std_sec: number;
  priority_covered: number;
  convergence: number[];
  compute_time_ms: number;
  routes: Record<string, Waypoint[]>;
  total_sorties?: number;
  sorties_per_drone?: Record<string, number>;
  total_physical_cells?: number;
  covered_physical_cells?: number;
  physical_coverage_pct?: number;
  compressed_strips_count?: number;
  rl_adaptation_log?: Array<{
    generation: number;
    state: number;
    state_label: string;
    action: number;
    action_label: string;
    w: number;
    c1: number;
    c2: number;
    p_mut: number;
    diversity: number;
    reward: number;
    gbest_fitness: number;
  }>;
  stage_breakdown?: {
    ga_time_ms: number;
    pso_refinement_time_ms: number;
  };
}

export interface TriBenchmarkComparison {
  makespan_reduction_vs_pso_pct: number;
  best_makespan_sec: number;
  lowest_energy_wh: number;
  recommended_algorithm: string;
}

export interface AlgorithmAnalyticsEvaluation {
  algorithm: string;
  makespan_sec: number;
  formatted_time: string;
  effective_coverage_pct: number;
  route_intersections_count: number;
  redundant_revisits_count: number;
  total_sorties?: number;
  best_fitness: number;
  elite_mean_fitness: number;
  compute_time_ms: number;
}

export interface MetricRankCard {
  value: number;
  rank: number;
  color: 'emerald' | 'amber' | 'orange';
}

export interface RouteAnalyticsMatrix {
  pso: AlgorithmAnalyticsEvaluation;
  ga: AlgorithmAnalyticsEvaluation;
  hybrid: AlgorithmAnalyticsEvaluation;
  ranking: {
    PSO: Record<string, MetricRankCard>;
    GA: Record<string, MetricRankCard>;
    HYBRID: Record<string, MetricRankCard>;
  };
  overall_winner: string;
  summary_scores: {
    PSO: number;
    GA: number;
    HYBRID: number;
  };
}

export interface TriAlgorithmBenchmarkResponse {
  status: string;
  fleet_size: number;
  active_cells_count: number;
  total_physical_cells?: number;
  compressed_strips_count?: number;
  kpi_comparison: TriBenchmarkComparison;
  analytics_comparison?: RouteAnalyticsMatrix;
  pso_result: AlgorithmResult;
  ga_result: AlgorithmResult;
  hybrid_result: AlgorithmResult;
}

export interface LiveDroneTelemetryState {
  drone_id: string;
  latitude: number;
  longitude: number;
  altitude_m: number;
  speed_mps: number;
  battery_pct: number;
  heading_deg?: number;
  status: 'READY' | 'STANDBY' | 'SEARCHING' | 'MAPPING' | 'RTL' | 'OFFLINE' | 'SORTIE_FLYING' | 'RETURNING_FOR_SWAP' | 'BATTERY_SWAPPING' | 'RELAUNCHING_SORTIE' | 'MISSION_COMPLETED' | string;
  current_waypoint_idx: number;
  total_waypoints: number;
  current_sortie?: number;
  total_sorties?: number;
  swap_remaining_sec?: number;
  overall_coverage_pct?: number;
}

export interface ScannedCellUpdate {
  cell_id: number;
  drone_id: string;
  elevation_m: number;
  row: number;
  col: number;
}

export interface LiveTelemetryFrame {
  mission_state: 'IDLE' | 'RUNNING' | 'PAUSED' | 'COMPLETED';
  elapsed_time_sec: number;
  progress_pct: number;
  cells_scanned_total: number;
  fleet: LiveDroneTelemetryState[];
  newly_scanned_cells: ScannedCellUpdate[];
}

export interface AdaptiveEventAuditLog {
  trigger_type: string;
  drone_id?: string;
  description: string;
  timestamp: number;
  metadata?: Record<string, any>;
}
