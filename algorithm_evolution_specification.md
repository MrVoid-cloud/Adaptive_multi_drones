# ==============================================================================
# ATP (ALGORITHMIC TECHNICAL PROFILE) - MULTI-UAV MISSION PLANNING ENGINE
# Document ID: ATP-UAV-2026-V3
# Domain: Computational GIS, Swarm Intelligence & Multi-Sortie UAV Routing
# ==============================================================================

[METADATA]
Project_Name = Adaptive Multi-Drone Search & Terrain Mapping System
Architecture_Type = Single-Depot Hybrid Graph-Grid Multi-UAV Planner
Spatial_Environment = Mixed Urban (OSM Road Topology) + Open Terrain (LiDAR Grid)
Depot_Configuration = Shared Home Base with Stratified Transit Altitudes (+5m/UAV)
Target_Coverage_Standard = 100.0% Physical Sensor Swath Coverage (55.0m Width)

# ==============================================================================
# SECTION 1: BENCHMARK DATASET & EXPERIMENTAL PARAMETERS
# ==============================================================================
[BENCHMARK_DATASET]
Region_Area_km2 = 6.40
Region_Dimensions_km = 3.20 x 2.00
Total_Active_Targets = 420
Urban_Road_Intersections = 68
Open_Terrain_Grid_Cells = 352
Active_Fleet_Size = 4
UAV_Cruise_Speed_ms = 14.0
UAV_Vertical_Speed_ms = 4.0
Sensor_Swath_Width_m = 55.0
Sensor_Capture_Radius_m = 27.5
Usable_Battery_Limit_min = 18.0
Depot_Battery_Swap_Time_sec = 120.0

# ==============================================================================
# SECTION 2: EVOLUTIONARY STAGES OF THE OPTIMIZATION ALGORITHM
# ==============================================================================

[GENERATION_01_BASELINE_SOLVERS]
Stage_Name = Independent Discrete Metaheuristics (Discrete PSO & Modified GA)
Architecture_Summary = Decoupled target assignment followed by greedy Nearest-Neighbor ordering.

  [[Discrete_PSO]]
  Mechanism = Integer assignment vector updated via static velocity probabilities.
  Parameters = w=0.70, c1=0.40, c2=0.45, Swarm_Size=30, Generations=30
  Strengths = Fast initial exploration (310 ms compute time).
  Limitations = Premature convergence to local minima; severe path crossings (47); high load imbalance (±24.6%).

  [[Modified_GA]]
  Mechanism = Integer chromosome evolution with tournament selection, 2-point crossover, and random mutation.
  Parameters = Mutation_Rate=0.12, Elitism=1, Pop_Size=30, Generations=30
  Strengths = Better preservation of contiguous target clusters than PSO.
  Limitations = Boundary fragmentation; 29 route crossings; incomplete coverage (68.8%) on single battery cycle.

[GENERATION_02_MEMETIC_HYBRID_GAPSO]
Stage_Name = Memetic Hybrid GA-PSO with Polar Seeding & 2-Opt Untangling
Architecture_Summary = Combines GA crossover/elitism with PSO velocity memory, spatial seeding, and geometric polishing.
Key_Innovations =
  1. Equal-Cardinality Polar Quantile Seeding around Shared Home Base (prevents depot exit crossings).
  2. Time-Aware Spatial Memetic Load Rebalance (transfers boundary targets from T_max UAV to T_min UAV).
  3. Post-Evolution 2-Opt & Or-Opt Route Untangling (reduces 2D route intersections to 0).
  4. Opportunistic Corridor Capture (marks targets scanned en-route within sensor radius).
Parameters = w=0.65, c1=0.50, c2=0.50, Pop_Swarm_Size=32, Generations=30
Strengths = Zero route intersections (0); tight fleet balance (±3.1%); 84.1% reduction in redundant revisits.
Limitations = Single-sortie energy constraint caps safe coverage at 74.5% on large maps (>5 km2).

[GENERATION_03_NEXTGEN_RL_MULTISORTIE]
Stage_Name = Next-Gen Online RL-Guided Self-Adaptive Hybrid GA-PSO (Multi-Sortie Engine)
Architecture_Summary = Zero-pretraining Online Q-Learning parameter control coupled with automated Multi-Sortie battery-swap scheduling.
Key_Innovations =
  1. Online 3x3 Tabular Q-Learning Controller: Monitors Shannon population diversity and stagnation counters at each generation to dynamically switch between Exploration (w=0.85, mut=0.35), Balanced, and Memetic Exploitation (c2=0.80) without offline training.
  2. True Sensor-Swath Resolution Lock (55.0m): Eliminates blind gaps across dense urban neighborhoods.
  3. Automated Multi-Sortie Breakpoint Scheduler: Monitors cumulative energy + RTL reserve; inserts SORTIE_RETURN_BASE, 120s battery swap, and SORTIE_RELAUNCH_BASE to resume from exact breakpoint.
  4. Per-Sortie 2-Opt + Or-Opt Polishing: Guarantees zero crossings inside every individual flight sortie.
Strengths = 100.0% physical area coverage; 0 route crossings; ±1.4% load variance; peak fitness 194.85.

# ==============================================================================
# SECTION 3: QUANTITATIVE BENCHMARK COMPARISON MATRIX
# ==============================================================================
[BENCHMARK_RESULTS_TABLE]
# Format: Metric | Gen1_PSO | Gen1_GA | Gen2_Hybrid | Gen3_NextGen_RL_MultiSortie
Total_Makespan_sec        = 2058.0 (34m18s) | 1865.0 (31m05s) | 1540.0 (25m40s) | 1332.0 (22m12s) [OPTIMAL]
True_Safe_Coverage_pct    = 64.2%           | 68.8%           | 74.5%           | 100.0%          [OPTIMAL]
Route_Intersections_2D    = 47              | 29              | 0 [OPTIMAL]     | 0               [OPTIMAL]
Redundant_Revisits        = 63              | 41              | 8               | 2               [OPTIMAL]
Inter_UAV_Load_Variance   = ±24.6%          | ±16.2%          | ±3.1%           | ±1.4%           [OPTIMAL]
Total_Fleet_Sorties       = 4 (Incomplete)  | 4 (Incomplete)  | 4 (Incomplete)  | 8 (2/UAV Full)  [OPTIMAL]
Peak_Fitness_gbest        = 118.40          | 134.75          | 169.20          | 194.85          [OPTIMAL]
Solver_Compute_Time_ms    = 310 ms          | 485 ms          | 620 ms          | 695 ms

# ==============================================================================
# END OF FILE: algorithm_evolution.atp
# ==============================================================================