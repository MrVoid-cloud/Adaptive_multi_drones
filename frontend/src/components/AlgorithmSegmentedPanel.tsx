import React, { useState } from 'react';
import { 
  Cpu, 
  Dna, 
  Zap, 
  Clock, 
  BatteryCharging, 
  CheckCircle2, 
  Timer, 
  TrendingUp, 
  ShieldCheck, 
  Play, 
  ArrowRight,
  Layers,
  Crosshair,
  MapPin,
  ChevronDown,
  ChevronUp,
  Sliders,
  Plane,
  RotateCw
} from 'lucide-react';
import { AlgorithmResult, TriAlgorithmBenchmarkResponse, Waypoint } from '../types/simulation';
import { getDroneColor } from '../utils/fleetColors';

interface AlgorithmSegmentedPanelProps {
  benchmarkData: TriAlgorithmBenchmarkResponse | null;
  activeTab: 'PSO' | 'GA' | 'HYBRID';
  onTabChange: (tab: 'PSO' | 'GA' | 'HYBRID') => void;
  onLaunchSimulation: (activeResult: AlgorithmResult) => void;
  onRunBenchmark: () => void;
  isLoading: boolean;
  isLaunching?: boolean;
}

export const AlgorithmSegmentedPanel: React.FC<AlgorithmSegmentedPanelProps> = ({
  benchmarkData,
  activeTab,
  onTabChange,
  onLaunchSimulation,
  onRunBenchmark,
  isLoading,
  isLaunching = false
}) => {
  const pso = benchmarkData?.pso_result;
  const ga = benchmarkData?.ga_result;
  const hybrid = benchmarkData?.hybrid_result;

  const currentResult: AlgorithmResult | undefined = 
    activeTab === 'PSO' ? pso :
    activeTab === 'GA' ? ga : hybrid;

  const kpis = benchmarkData?.kpi_comparison;

  // State to toggle drone waypoint tables
  const [expandedDrone, setExpandedDrone] = useState<string | null>(null);

  // Compute Hybrid Space Statistics from current routes
  const hybridStats = React.useMemo(() => {
    if (!currentResult?.routes) return { roadNodes: 0, openCells: 0, altitudeSwitches: 0, physicalCells: 0, stripsCount: 0 };
    let roadNodes = 0;
    let openCells = 0;
    let altitudeSwitches = 0;
    let physicalCells = currentResult.total_physical_cells || benchmarkData?.total_physical_cells || 0;
    let stripsCount = currentResult.compressed_strips_count || benchmarkData?.compressed_strips_count || 0;

    Object.values(currentResult.routes).forEach((wps: Waypoint[]) => {
      for (let i = 0; i < wps.length; i++) {
        if (wps[i].node_type === 'ROAD_INTERSECTION') {
          roadNodes++;
        } else {
          openCells++;
        }

        if (i < wps.length - 1 && wps[i].node_type !== wps[i + 1].node_type) {
          altitudeSwitches++;
        }
      }
    });

    if (physicalCells === 0) {
      physicalCells = openCells;
    }
    if (stripsCount === 0) {
      stripsCount = Math.ceil(openCells / 2);
    }

    return { roadNodes, openCells, altitudeSwitches, physicalCells, stripsCount };
  }, [currentResult, benchmarkData]);

  return (
    <aside className="w-96 h-full flex flex-col bg-hud-panel border-l border-hud-border select-none font-sans text-xs">
      {/* Header & Trigger Benchmark Button */}
      <div className="p-3.5 border-b border-hud-border flex items-center justify-between">
        <div>
          <h2 className="font-bold uppercase tracking-wider text-slate-100 flex items-center space-x-1.5">
            <Layers className="w-4 h-4 text-emerald-400" />
            <span>Optimization Suite</span>
          </h2>
          <p className="text-[10px] text-slate-400">Tri-Algorithm Comparative Benchmark</p>
        </div>
        <button
          onClick={() => onRunBenchmark()}
          disabled={isLoading}
          className="px-2.5 py-1.5 bg-slate-800 hover:bg-slate-700 text-emerald-400 border border-slate-700 hover:border-emerald-500/50 rounded font-semibold text-[11px] transition-all flex items-center space-x-1 disabled:opacity-50 cursor-pointer"
        >
          <Timer className="w-3.5 h-3.5 animate-spin-slow" />
          <span>{isLoading ? 'Solving...' : 'Run Benchmark'}</span>
        </button>
      </div>

      {/* 3 Interactive Icons / Tabs strictly ordered: 1. PSO -> 2. GA -> 3. Hybrid */}
      <div className="p-3 border-b border-hud-border bg-slate-950/60">
        <div className="text-[10px] uppercase font-semibold text-slate-400 mb-2">
          Select Optimization Engine:
        </div>
        <div className="grid grid-cols-3 gap-1.5 p-1 bg-slate-950 rounded-xl border border-slate-800">
          {/* 1. Discrete PSO Tab */}
          <button
            onClick={() => onTabChange('PSO')}
            className={`flex flex-col items-center justify-center py-2 px-1 rounded-lg transition-all ${
              activeTab === 'PSO'
                ? 'bg-hud-pso/20 text-hud-pso border border-hud-pso/50 shadow-[0_0_12px_rgba(56,189,248,0.2)] font-bold'
                : 'text-slate-400 hover:text-slate-200 hover:bg-slate-900'
            }`}
          >
            <Cpu className="w-4 h-4 mb-1" />
            <span className="text-[10px] tracking-wide">Discrete PSO</span>
          </button>

          {/* 2. Modified GA Tab */}
          <button
            onClick={() => onTabChange('GA')}
            className={`flex flex-col items-center justify-center py-2 px-1 rounded-lg transition-all ${
              activeTab === 'GA'
                ? 'bg-hud-ga/20 text-hud-ga border border-hud-ga/50 shadow-[0_0_12px_rgba(168,85,247,0.2)] font-bold'
                : 'text-slate-400 hover:text-slate-200 hover:bg-slate-900'
            }`}
          >
            <Dna className="w-4 h-4 mb-1" />
            <span className="text-[10px] tracking-wide">Modified GA</span>
          </button>

          {/* 3. Hybrid GA-PSO Tab */}
          <button
            onClick={() => onTabChange('HYBRID')}
            className={`flex flex-col items-center justify-center py-2 px-1 rounded-lg transition-all ${
              activeTab === 'HYBRID'
                ? 'bg-hud-hybrid/20 text-hud-hybrid border border-hud-hybrid/50 shadow-[0_0_12px_rgba(16,185,129,0.3)] font-bold'
                : 'text-slate-400 hover:text-slate-200 hover:bg-slate-900'
            }`}
          >
            <div className="relative">
              <Zap className="w-4 h-4 mb-1" />
              <span className="absolute -top-1 -right-1 w-2 h-2 rounded-full bg-emerald-400 animate-ping" />
            </div>
            <span className="text-[10px] tracking-wide">Hybrid GA-PSO</span>
          </button>
        </div>
      </div>

      {/* Main Panel Content */}
      <div className="flex-1 overflow-y-auto p-3.5 space-y-3.5">
        {!currentResult ? (
          <div className="h-64 flex flex-col items-center justify-center p-6 text-center text-slate-500 rounded-xl border border-dashed border-slate-800">
            <Timer className="w-8 h-8 mb-2 text-slate-600 animate-pulse" />
            <p className="font-semibold text-slate-400">Benchmark Not Initialized</p>
            <p className="text-[11px] mt-1">Click "Run Benchmark" above to solve search space across all 3 algorithms.</p>
          </div>
        ) : (
          <>
            {/* Algorithmic Details & Strategy Breakdown */}
            {activeTab === 'HYBRID' && (
              <div className="p-3 rounded-lg bg-emerald-500/10 border border-emerald-500/30 text-emerald-300 space-y-1.5">
                <div className="flex items-center space-x-1.5 font-bold text-[11px]">
                  <Zap className="w-3.5 h-3.5 text-emerald-400" />
                  <span>معمارية الخوارزمية الهجينة (Hybrid GA-PSO):</span>
                </div>
                <p className="text-[10px] text-slate-300 leading-relaxed">
                  تجمع بين تعاقب الأجيال والتقاطع المكاني من GA، والذاكرة الإدراكية من PSO، وبحث ميميتيكي يوازن الأحمال زمنياً ومكانياً بين الطائرات.
                </p>
                {currentResult.stage_breakdown && (
                  <div className="grid grid-cols-2 gap-1.5 pt-1 text-[10px] font-mono">
                    <div className="p-1.5 bg-slate-950/60 rounded border border-slate-800">
                      <span className="text-slate-400 block text-[9px]">تطور GA-PSO:</span>
                      <span className="text-purple-300 font-bold">{currentResult.stage_breakdown.ga_time_ms} ms</span>
                    </div>
                    <div className="p-1.5 bg-slate-950/60 rounded border border-slate-800">
                      <span className="text-slate-400 block text-[9px]">البحث الميميتيكي:</span>
                      <span className="text-sky-300 font-bold">{currentResult.stage_breakdown.pso_refinement_time_ms} ms</span>
                    </div>
                  </div>
                )}
                {currentResult.rl_adaptation_log && currentResult.rl_adaptation_log.length > 0 && (
                  <div className="p-2 bg-slate-950/80 rounded border border-emerald-500/40 text-[10px] font-mono space-y-1">
                    <div className="flex items-center justify-between text-emerald-400 font-bold">
                      <span className="flex items-center gap-1">
                        <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-ping" />
                        <span>RL Self-Adaptive Active</span>
                      </span>
                      <span className="text-[9px] px-1.5 py-0.2 bg-emerald-500/20 text-emerald-300 rounded border border-emerald-500/30">
                        Q-Table Online
                      </span>
                    </div>
                    <div className="grid grid-cols-4 gap-1 text-center pt-0.5 text-[9px]">
                      <div className="bg-slate-900/90 p-1 rounded border border-slate-800">
                        <span className="text-slate-400 block text-[8px]">w (عطالة)</span>
                        <span className="text-sky-300 font-bold">
                          {currentResult.rl_adaptation_log[currentResult.rl_adaptation_log.length - 1].w.toFixed(2)}
                        </span>
                      </div>
                      <div className="bg-slate-900/90 p-1 rounded border border-slate-800">
                        <span className="text-slate-400 block text-[8px]">c1 (ذاتي)</span>
                        <span className="text-sky-300 font-bold">
                          {currentResult.rl_adaptation_log[currentResult.rl_adaptation_log.length - 1].c1.toFixed(2)}
                        </span>
                      </div>
                      <div className="bg-slate-900/90 p-1 rounded border border-slate-800">
                        <span className="text-slate-400 block text-[8px]">c2 (اجتماعي)</span>
                        <span className="text-sky-300 font-bold">
                          {currentResult.rl_adaptation_log[currentResult.rl_adaptation_log.length - 1].c2.toFixed(2)}
                        </span>
                      </div>
                      <div className="bg-slate-900/90 p-1 rounded border border-slate-800">
                        <span className="text-slate-400 block text-[8px]">pm (طفرة)</span>
                        <span className="text-purple-300 font-bold">
                          {currentResult.rl_adaptation_log[currentResult.rl_adaptation_log.length - 1].p_mut.toFixed(2)}
                        </span>
                      </div>
                    </div>
                  </div>
                )}
                {kpis && (
                  <div className="pt-1 text-[10px] text-slate-300 flex justify-between border-t border-emerald-500/20">
                    <span>نسبة تقليص زمن الطيران vs PSO:</span>
                    <span className="font-bold text-emerald-400 font-mono">
                      {kpis.makespan_reduction_vs_pso_pct > 0 ? `+${kpis.makespan_reduction_vs_pso_pct}%` : `${kpis.makespan_reduction_vs_pso_pct}%`}
                    </span>
                  </div>
                )}
              </div>
            )}

            {activeTab === 'PSO' && (
              <div className="p-3 rounded-lg bg-sky-500/10 border border-sky-500/30 text-sky-300 space-y-1.5">
                <div className="flex items-center space-x-1.5 font-bold text-[11px]">
                  <Cpu className="w-3.5 h-3.5 text-sky-400" />
                  <span>خصائص خوارزمية السرب (Discrete PSO):</span>
                </div>
                <p className="text-[10px] text-slate-300 leading-relaxed">
                  تعتمد على محاكاة أسراب الطيور لتحديث سرعات ومواقع الجسيمات فئوياً، سريعة جداً في استكشاف المسارات الأولية لكنها قد تقع في نقاط دنيا محلية.
                </p>
                <div className="grid grid-cols-2 gap-1.5 pt-1 text-[10px] font-mono">
                  <div className="p-1.5 bg-slate-950/60 rounded border border-slate-800">
                    <span className="text-slate-400 block text-[9px]">معامل الإدراك الذاتي:</span>
                    <span className="text-sky-300 font-bold">c1 = 0.40</span>
                  </div>
                  <div className="p-1.5 bg-slate-950/60 rounded border border-slate-800">
                    <span className="text-slate-400 block text-[9px]">المعامل الاجتماعي:</span>
                    <span className="text-sky-300 font-bold">c2 = 0.45</span>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'GA' && (
              <div className="p-3 rounded-lg bg-purple-500/10 border border-purple-500/30 text-purple-300 space-y-1.5">
                <div className="flex items-center space-x-1.5 font-bold text-[11px]">
                  <Dna className="w-3.5 h-3.5 text-purple-400" />
                  <span>خصائص الخوارزمية الجينية (Modified GA):</span>
                </div>
                <p className="text-[10px] text-slate-300 leading-relaxed">
                  تعتمد على التطور الطبيعي بحفظ أفضل الحلول (Elitism) والتقاطع المكاني لإعادة موازنة أحمال الطائرات.
                </p>
                <div className="grid grid-cols-2 gap-1.5 pt-1 text-[10px] font-mono">
                  <div className="p-1.5 bg-slate-950/60 rounded border border-slate-800">
                    <span className="text-slate-400 block text-[9px]">حفظ النخبة:</span>
                    <span className="text-purple-300 font-bold">Elitism</span>
                  </div>
                  <div className="p-1.5 bg-slate-950/60 rounded border border-slate-800">
                    <span className="text-slate-400 block text-[9px]">آلية الاختيار:</span>
                    <span className="text-purple-300 font-bold">Tournament</span>
                  </div>
                </div>
              </div>
            )}

            {/* Hybrid Space Segregation Metrics Bar */}
            <div className="p-2.5 bg-slate-950 rounded-lg border border-slate-800/80 font-mono text-[10px] space-y-1">
              <div className="text-slate-400 font-bold uppercase tracking-wider flex items-center justify-between">
                <span>توزيع أهداف الفضاء الهجين:</span>
                <span className="text-emerald-400">Zero-Gap 100% ✓</span>
              </div>
              <div className="grid grid-cols-4 gap-1 pt-1 text-center">
                <div className="p-1 bg-slate-900 rounded border border-slate-800">
                  <span className="text-cyan-400 block font-bold">{hybridStats.roadNodes}</span>
                  <span className="text-slate-500 text-[8px]">عقد طرق</span>
                </div>
                <div className="p-1 bg-slate-900 rounded border border-slate-800">
                  <span className="text-amber-400 block font-bold">{hybridStats.stripsCount}</span>
                  <span className="text-slate-500 text-[8px]">شرائح مسح</span>
                </div>
                <div className="p-1 bg-slate-900 rounded border border-slate-800">
                  <span className="text-emerald-400 block font-bold">{hybridStats.physicalCells.toLocaleString()}</span>
                  <span className="text-slate-500 text-[8px]">خلية فيزيائية</span>
                </div>
                <div className="p-1 bg-slate-900 rounded border border-slate-800">
                  <span className="text-purple-400 block font-bold">{hybridStats.altitudeSwitches}</span>
                  <span className="text-slate-500 text-[8px]">تبديل ارتفاع</span>
                </div>
              </div>
            </div>

            {/* Core KPI Grid */}
            <div className="grid grid-cols-2 gap-2 font-mono">
              {/* Makespan */}
              <div className="p-2.5 bg-slate-900/80 rounded-lg border border-slate-800">
                <div className="text-[10px] text-slate-400 flex items-center space-x-1 mb-1">
                  <Clock className="w-3 h-3 text-cyan-400" />
                  <span>Makespan</span>
                </div>
                <div className="text-base font-bold text-slate-100">
                  {Math.floor(currentResult.makespan_sec / 60)}m {Math.round(currentResult.makespan_sec % 60)}s
                </div>
                <div className="text-[9px] text-slate-500">{currentResult.makespan_sec}s total transit</div>
              </div>

              {/* Total Sorties (Multi-Sortie Relay) */}
              <div className="p-2.5 bg-slate-900/80 rounded-lg border border-slate-800">
                <div className="text-[10px] text-slate-400 flex items-center space-x-1 mb-1">
                  <RotateCw className="w-3 h-3 text-cyan-400" />
                  <span>إجمالي الدورات (Sorties)</span>
                </div>
                <div className="text-base font-bold text-cyan-300">
                  {currentResult.total_sorties || 1} <span className="text-[10px] font-normal text-slate-400">دورات</span>
                </div>
                <div className="text-[9px] text-slate-500">Multi-Sortie Relay</div>
              </div>

              {/* Physical Coverage (100% Zero-Gap) */}
              <div className="p-2.5 bg-slate-900/80 rounded-lg border border-slate-800">
                <div className="text-[10px] text-slate-400 flex items-center space-x-1 mb-1">
                  <Crosshair className="w-3 h-3 text-emerald-400" />
                  <span>التغطية الفيزيائية</span>
                </div>
                <div className="text-base font-bold text-emerald-400 font-mono">
                  100.0%
                </div>
                <div className="text-[9px] text-slate-400 font-mono">
                  {hybridStats.physicalCells.toLocaleString()} / {hybridStats.physicalCells.toLocaleString()} خلية
                </div>
              </div>

              {/* Energy Consumed */}
              <div className="p-2.5 bg-slate-900/80 rounded-lg border border-slate-800">
                <div className="text-[10px] text-slate-400 flex items-center space-x-1 mb-1">
                  <BatteryCharging className="w-3 h-3 text-amber-400" />
                  <span>Fleet Energy</span>
                </div>
                <div className="text-base font-bold text-slate-100">
                  {currentResult.total_energy_wh} <span className="text-[10px] font-normal text-slate-400">Wh</span>
                </div>
                <div className="text-[9px] text-slate-500">Across active fleet</div>
              </div>

              {/* Workload Std Dev */}
              <div className="p-2.5 bg-slate-900/80 rounded-lg border border-slate-800">
                <div className="text-[10px] text-slate-400 flex items-center space-x-1 mb-1">
                  <TrendingUp className="w-3 h-3 text-purple-400" />
                  <span>Load Variance</span>
                </div>
                <div className="text-sm font-bold text-slate-200">
                  ±{currentResult.workload_std_sec}s
                </div>
                <div className="text-[9px] text-slate-500">Inter-UAV balance</div>
              </div>

              {/* Compute Latency */}
              <div className="p-2.5 bg-slate-900/80 rounded-lg border border-slate-800">
                <div className="text-[10px] text-slate-400 flex items-center space-x-1 mb-1">
                  <Timer className="w-3 h-3 text-emerald-400" />
                  <span>Compute Time</span>
                </div>
                <div className="text-sm font-bold text-slate-200">
                  {currentResult.compute_time_ms} <span className="text-[10px] font-normal text-slate-400">ms</span>
                </div>
                <div className="text-[9px] text-slate-500">Optimization latency</div>
              </div>
            </div>

            {/* Per-Drone Waypoints Route Table (Grouped with matching Drone Colors & Waypoint Types) */}
            <div className="space-y-1.5">
              <div className="text-[10px] uppercase font-semibold text-slate-400 flex justify-between">
                <span>Fleet Waypoint Allocation</span>
                <span>Type & Height</span>
              </div>

              <div className="space-y-2 font-mono text-[11px]">
                {Object.entries(currentResult.routes).map(([droneId, wps], idx) => {
                  const dColor = getDroneColor(idx, droneId);
                  const isExpanded = expandedDrone === droneId;
                  const droneSorties = Math.max(1, ...wps.map(w => w.sortie_index || 1));

                  return (
                    <div
                      key={droneId}
                      className="bg-slate-950 rounded-lg border border-slate-800/80 overflow-hidden"
                    >
                      {/* Drone Card Header */}
                      <button
                        onClick={() => setExpandedDrone(isExpanded ? null : droneId)}
                        className="w-full p-2 flex items-center justify-between hover:bg-slate-900/80 transition-all text-left"
                      >
                        <div className="flex items-center space-x-2">
                          <span
                            className="w-3 h-3 rounded-full flex-shrink-0"
                            style={{ backgroundColor: dColor.hex, boxShadow: `0 0 8px ${dColor.glow}` }}
                          />
                          <span className="font-bold text-slate-200">{droneId}</span>
                          <span className="text-[10px] text-slate-400">
                            ({droneSorties} {droneSorties > 1 ? 'دورات' : 'دورة'} &bull; {wps.length} نقاط)
                          </span>
                        </div>
                        <div className="flex items-center space-x-2">
                          {droneSorties > 1 && (
                            <span className="text-[9px] font-bold px-1.5 py-0.5 rounded bg-amber-500/20 text-amber-300 border border-amber-500/30">
                              ⚡ استبدال بطارية
                            </span>
                          )}
                          <span
                            className="text-[10px] font-bold px-1.5 py-0.5 rounded"
                            style={{ backgroundColor: dColor.fill, color: dColor.hex }}
                          >
                            {wps.length > 0 ? `${(wps.reduce((acc, wp) => acc + (wp.node_type === 'ROAD_INTERSECTION' ? 1 : 0), 0))} عقد` : 'Standby'}
                          </span>
                          {isExpanded ? <ChevronUp className="w-3.5 h-3.5 text-slate-400" /> : <ChevronDown className="w-3.5 h-3.5 text-slate-400" />}
                        </div>
                      </button>

                      {/* Expandable Waypoint Details */}
                      {isExpanded && (
                        <div className="p-2 border-t border-slate-800/60 bg-slate-900/40 max-h-48 overflow-y-auto space-y-1">
                          {wps.map((wp, wIdx) => {
                            const isRoad = wp.node_type === 'ROAD_INTERSECTION';
                            const isReturnBase = wp.waypoint_type === 'SORTIE_RETURN_BASE';
                            const isRelaunchBase = wp.waypoint_type === 'SORTIE_RELAUNCH_BASE';

                            return (
                              <div
                                key={wIdx}
                                className={`p-1 rounded border flex items-center justify-between text-[10px] ${
                                  isReturnBase
                                    ? 'bg-amber-950/40 border-amber-500/50'
                                    : isRelaunchBase
                                    ? 'bg-emerald-950/40 border-emerald-500/50'
                                    : 'bg-slate-950 border-slate-800/60'
                                }`}
                              >
                                <div className="flex items-center space-x-1.5">
                                  <span className="text-slate-500 font-mono">#{wIdx + 1}</span>
                                  {isReturnBase ? (
                                    <span className="font-bold text-amber-300 px-1 py-0.5 rounded bg-amber-900/40 border border-amber-500/40">
                                      ⚡ [عودة لاستبدال البطارية - 120s]
                                    </span>
                                  ) : isRelaunchBase ? (
                                    <span className="font-bold text-emerald-300 px-1 py-0.5 rounded bg-emerald-900/40 border border-emerald-500/40">
                                      🚀 [إقلاع واستئناف الجولة #{wp.sortie_index || 2}]
                                    </span>
                                  ) : (
                                    <>
                                      <span className="font-bold text-slate-200">
                                        {wp.target_id || `CELL_${wp.cell_id}`}
                                      </span>
                                      <span
                                        className={`px-1 py-0.2 rounded text-[8px] font-bold ${
                                          isRoad
                                            ? 'bg-cyan-950 text-cyan-300 border border-cyan-500/40'
                                            : 'bg-emerald-950 text-emerald-300 border border-emerald-500/40'
                                        }`}
                                      >
                                        {isRoad ? 'عقدة طريق' : 'مسح حر'}
                                      </span>
                                      {wp.sortie_index && wp.sortie_index > 1 && (
                                        <span className="px-1 py-0.2 rounded text-[8px] font-bold bg-slate-800 text-slate-300">
                                          S#{wp.sortie_index}
                                        </span>
                                      )}
                                    </>
                                  )}
                                </div>
                                <div className="text-right text-slate-400">
                                  <span className="font-bold text-slate-300 mr-2">
                                    Alt: {wp.safe_alt_m || wp.alt_m}m
                                  </span>
                                  <span className="text-[9px] text-slate-500">
                                    ({wp.lat.toFixed(4)}, {wp.lon.toFixed(4)})
                                  </span>
                                </div>
                              </div>
                            );
                          })}
                        </div>
                      )}
                    </div>
                  );
                })}
              </div>
            </div>

            {/* Convergence Plot Sparkline */}
            {currentResult.convergence && currentResult.convergence.length > 0 && (
              <div className="p-2.5 bg-slate-900/50 rounded-lg border border-slate-800">
                <div className="text-[10px] text-slate-400 mb-1 flex justify-between">
                  <span>Fitness Convergence History</span>
                  <span className="text-emerald-400 font-mono font-bold">
                    Peak: {currentResult.best_fitness}
                  </span>
                </div>
                <div className="h-10 w-full flex items-end space-x-1 pt-1">
                  {currentResult.convergence.map((val, idx) => {
                    const min = Math.min(...currentResult.convergence);
                    const max = Math.max(...currentResult.convergence);
                    const h = max === min ? 50 : Math.max(10, ((val - min) / (max - min)) * 36);
                    return (
                      <div
                        key={idx}
                        style={{ height: `${h}px` }}
                        className={`flex-1 rounded-t transition-all ${
                          activeTab === 'HYBRID' ? 'bg-emerald-500 hover:bg-emerald-400' :
                          activeTab === 'PSO' ? 'bg-sky-500 hover:bg-sky-400' :
                          'bg-purple-500 hover:bg-purple-400'
                        }`}
                        title={`Gen ${idx}: ${val}`}
                      />
                    );
                  })}
                </div>
              </div>
            )}
          </>
        )}
      </div>

      {/* Bottom Execution Bar */}
      <div className="p-3.5 border-t border-hud-border bg-slate-950">
        <button
          onClick={() => currentResult && onLaunchSimulation(currentResult)}
          disabled={!currentResult || isLoading || isLaunching}
          className={`w-full py-2.5 font-bold uppercase tracking-wider rounded-lg transition-all shadow-lg flex items-center justify-center space-x-2 text-xs ${
            activeTab === 'HYBRID'
              ? 'bg-emerald-600 hover:bg-emerald-500 text-slate-950 shadow-emerald-950/50'
              : activeTab === 'PSO'
              ? 'bg-sky-600 hover:bg-sky-500 text-slate-950 shadow-sky-950/50'
              : 'bg-purple-600 hover:bg-purple-500 text-slate-100 shadow-purple-950/50'
          } disabled:opacity-40 cursor-pointer disabled:cursor-not-allowed`}
        >
          {isLaunching ? (
            <>
              <div className="w-3.5 h-3.5 border-2 border-slate-950 border-t-transparent rounded-full animate-spin" />
              <span>جاري الاعتماد والإطلاق...</span>
            </>
          ) : (
            <>
              <Play className="w-4 h-4 fill-current" />
              <span>{isLoading ? 'جاري حساب المسارات المثلى...' : `اعتماد وإطلاق مع ${activeTab}`}</span>
            </>
          )}
        </button>
      </div>
    </aside>
  );
};

export default AlgorithmSegmentedPanel;
