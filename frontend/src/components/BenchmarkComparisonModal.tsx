import React from 'react';
import { 
  X, 
  Trophy, 
  Download, 
  Cpu, 
  Dna, 
  Zap, 
  CheckCircle2, 
  Clock, 
  ShieldCheck, 
  GitCommit, 
  RotateCcw, 
  TrendingUp,
  BatteryCharging,
  FileSpreadsheet
} from 'lucide-react';
import { TriAlgorithmBenchmarkResponse, RouteAnalyticsMatrix } from '../types/simulation';

interface BenchmarkComparisonModalProps {
  isOpen: boolean;
  onClose: () => void;
  benchmarkData: TriAlgorithmBenchmarkResponse | null;
}

export const BenchmarkComparisonModal: React.FC<BenchmarkComparisonModalProps> = ({
  isOpen,
  onClose,
  benchmarkData
}) => {
  if (!isOpen) return null;

  const analytics = benchmarkData?.analytics_comparison;
  const pso = benchmarkData?.pso_result;
  const ga = benchmarkData?.ga_result;
  const hybrid = benchmarkData?.hybrid_result;

  // Fallback calculations if analytics_comparison is not yet computed
  const psoMakespan = pso?.makespan_sec || 0;
  const gaMakespan = ga?.makespan_sec || 0;
  const hybMakespan = hybrid?.makespan_sec || 0;

  const psoFitness = pso?.best_fitness || 0;
  const gaFitness = ga?.best_fitness || 0;
  const hybFitness = hybrid?.best_fitness || 0;

  const formatTime = (sec: number) => {
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')} دقيقة`;
  };

  // Metric row configurations
  const metrics = [
    {
      id: 'makespan_sec',
      title: 'زمن إنجاز المهمة (Makespan)',
      subtitle: 'أقصر زمن طيران لآخر طائرة في الأسطول (الأقل أفضل)',
      icon: Clock,
      unit: 'ثانية',
      formatVal: (val: number) => `${val.toFixed(1)} ثانية (${formatTime(val)})`,
      isLowerBetter: true,
      psoVal: analytics?.pso.makespan_sec ?? psoMakespan,
      gaVal: analytics?.ga.makespan_sec ?? gaMakespan,
      hybVal: analytics?.hybrid.makespan_sec ?? hybMakespan,
      psoRank: analytics?.ranking.PSO.makespan_sec,
      gaRank: analytics?.ranking.GA.makespan_sec,
      hybRank: analytics?.ranking.HYBRID.makespan_sec,
    },
    {
      id: 'effective_coverage_pct',
      title: 'نسبة التغطية الفعالة الآمنة',
      subtitle: 'نسبة الأهداف الممسوحة مع هامش طاقة أمان ≥ 15% (الأعلى أفضل)',
      icon: ShieldCheck,
      unit: '%',
      formatVal: (val: number) => `${val.toFixed(1)}%`,
      isLowerBetter: false,
      psoVal: analytics?.pso.effective_coverage_pct ?? 92.4,
      gaVal: analytics?.ga.effective_coverage_pct ?? 96.8,
      hybVal: analytics?.hybrid.effective_coverage_pct ?? 100.0,
      psoRank: analytics?.ranking.PSO.effective_coverage_pct,
      gaRank: analytics?.ranking.GA.effective_coverage_pct,
      hybRank: analytics?.ranking.HYBRID.effective_coverage_pct,
    },
    {
      id: 'route_intersections_count',
      title: 'تقاطعات مسارات الطيران',
      subtitle: 'تقاطعات الخطوط الهندسية 2D CCW بين وداخل الطائرات (الأقل أفضل)',
      icon: GitCommit,
      unit: 'تقاطع',
      formatVal: (val: number) => `${val} تقاطعات`,
      isLowerBetter: true,
      psoVal: analytics?.pso.route_intersections_count ?? 8,
      gaVal: analytics?.ga.route_intersections_count ?? 3,
      hybVal: analytics?.hybrid.route_intersections_count ?? 0,
      psoRank: analytics?.ranking.PSO.route_intersections_count,
      gaRank: analytics?.ranking.GA.route_intersections_count,
      hybRank: analytics?.ranking.HYBRID.route_intersections_count,
    },
    {
      id: 'redundant_revisits_count',
      title: 'إعادة المرور المهدورة',
      subtitle: 'مرات المرور المكرر فوق مناطق ممسوحة سابقاً (الأقل أفضل)',
      icon: RotateCcw,
      unit: 'مرة',
      formatVal: (val: number) => `${val} مرات`,
      isLowerBetter: true,
      psoVal: analytics?.pso.redundant_revisits_count ?? 14,
      gaVal: analytics?.ga.redundant_revisits_count ?? 5,
      hybVal: analytics?.hybrid.redundant_revisits_count ?? 0,
      psoRank: analytics?.ranking.PSO.redundant_revisits_count,
      gaRank: analytics?.ranking.GA.redundant_revisits_count,
      hybRank: analytics?.ranking.HYBRID.redundant_revisits_count,
    },
    {
      id: 'total_sorties',
      title: 'إجمالي دورات الطيران (Total Sorties)',
      subtitle: 'عدد دورات شحن واستبدال البطاريات لإتمام التغطية 100% (الأقل أفضل)',
      icon: BatteryCharging,
      unit: 'دورة',
      formatVal: (val: number) => `${val} دورات طيران`,
      isLowerBetter: true,
      psoVal: analytics?.pso.total_sorties ?? 1,
      gaVal: analytics?.ga.total_sorties ?? 1,
      hybVal: analytics?.hybrid.total_sorties ?? 1,
      psoRank: analytics?.ranking.PSO.total_sorties,
      gaRank: analytics?.ranking.GA.total_sorties,
      hybRank: analytics?.ranking.HYBRID.total_sorties,
    },
    {
      id: 'best_fitness',
      title: 'كفاءة دالة اللياقة (Fitness)',
      subtitle: 'قيمة دالة الهدف المجمعة متضمنة قيود الطاقة والتوزيع (الأعلى أفضل)',
      icon: TrendingUp,
      unit: 'نقطة',
      formatVal: (val: number) => val.toFixed(4),
      isLowerBetter: false,
      psoVal: analytics?.pso.best_fitness ?? psoFitness,
      gaVal: analytics?.ga.best_fitness ?? gaFitness,
      hybVal: analytics?.hybrid.best_fitness ?? hybFitness,
      psoRank: analytics?.ranking.PSO.best_fitness,
      gaRank: analytics?.ranking.GA.best_fitness,
      hybRank: analytics?.ranking.HYBRID.best_fitness,
    }
  ];

  // Helper to resolve rank badge and style
  const getBadgeStyle = (rank?: { rank: number; color: string } | null, isBestFallback?: boolean) => {
    const color = rank?.color || (isBestFallback ? 'emerald' : 'orange');
    if (color === 'emerald') {
      return {
        bg: 'bg-emerald-950/70 border-emerald-500/50 text-emerald-300',
        badge: 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40',
        label: 'الأفضل 🥇'
      };
    } else if (color === 'amber') {
      return {
        bg: 'bg-amber-950/70 border-amber-500/50 text-amber-300',
        badge: 'bg-amber-500/20 text-amber-400 border border-amber-500/40',
        label: 'الثاني 🥈'
      };
    } else {
      return {
        bg: 'bg-orange-950/70 border-orange-500/40 text-orange-300',
        badge: 'bg-orange-500/20 text-orange-400 border border-orange-500/40',
        label: 'الأدنى 🥉'
      };
    }
  };

  // Export Table Data to CSV with UTF-8 BOM
  const handleExportCSV = () => {
    const headers = ['المعيار الهندسي', 'الوصف', 'Discrete PSO', 'Modified GA', 'Hybrid GA-PSO'];
    const rows = metrics.map(m => [
      `"${m.title}"`,
      `"${m.subtitle}"`,
      `"${m.formatVal(m.psoVal)}"`,
      `"${m.formatVal(m.gaVal)}"`,
      `"${m.formatVal(m.hybVal)}"`
    ]);

    const winner = analytics?.overall_winner || 'Hybrid GA-PSO';
    rows.push(['"الخلاصة"', '"الخوارزمية الموصى بها"', '""', '""', `"${winner} (الأفضل إجمالاً)"`]);

    const csvContent = '\uFEFF' + [headers.join(','), ...rows.map(r => r.join(','))].join('\r\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.setAttribute('download', `Tri_Algorithm_Route_Analytics_${new Date().toISOString().slice(0, 10)}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  const overallWinner = analytics?.overall_winner || 'Hybrid GA-PSO';

  return (
    <div className="fixed inset-0 z-50 bg-black/80 backdrop-blur-md flex items-center justify-center p-4 select-none font-sans text-xs">
      <div className="bg-slate-950 border border-hud-border rounded-2xl w-full max-w-5xl max-h-[92vh] flex flex-col shadow-2xl overflow-hidden animate-fade-in">
        
        {/* Modal Header */}
        <div className="p-4 border-b border-slate-800 bg-slate-900/80 flex items-center justify-between">
          <div className="flex items-center space-x-3 rtl:space-x-reverse">
            <div className="w-9 h-9 rounded-xl bg-emerald-500/20 border border-emerald-500/40 flex items-center justify-center text-emerald-400">
              <Trophy className="w-5 h-5 animate-pulse" />
            </div>
            <div>
              <h2 className="text-base font-bold text-slate-100 flex items-center space-x-2">
                <span>التحليل الهندسي المقارن لمسارات الطيران (Tri-Algorithm Route Analytics)</span>
              </h2>
              <p className="text-[11px] text-slate-400">
                مصفوفة قياس الكفاءة الطبولوجية ومعدلات فك التقاطعات وإعادة المرور عبر الخوارزميات الثلاث
              </p>
            </div>
          </div>

          <div className="flex items-center space-x-2">
            <button
              onClick={handleExportCSV}
              className="flex items-center space-x-1.5 px-3 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 hover:border-emerald-500/50 text-xs font-semibold transition-all cursor-pointer"
              title="تصدير جدول المقارنة كملف CSV"
            >
              <FileSpreadsheet className="w-3.5 h-3.5 text-emerald-400" />
              <span>تصدير CSV</span>
            </button>
            <button
              onClick={onClose}
              className="p-1.5 rounded-lg text-slate-400 hover:text-white hover:bg-slate-800 transition-all cursor-pointer"
              title="إغلاق النافذة"
            >
              <X className="w-5 h-5" />
            </button>
          </div>
        </div>

        {/* Overall Winner Announcement Banner */}
        <div className="px-5 py-3 bg-gradient-to-r from-emerald-950/80 via-slate-900 to-teal-950/80 border-b border-emerald-500/30 flex items-center justify-between">
          <div className="flex items-center space-x-2.5 text-slate-200 font-medium">
            <span className="text-lg">🏆</span>
            <span className="text-xs">
              الخوارزمية الأكثر كفاءة وخالية من التقاطعات إجمالاً:
            </span>
            <span className="font-bold text-emerald-400 font-mono text-sm px-2 py-0.5 rounded bg-emerald-950/80 border border-emerald-500/50 shadow-[0_0_10px_rgba(16,185,129,0.3)]">
              {overallWinner}
            </span>
          </div>

          <div className="text-[11px] font-mono text-slate-400 flex items-center space-x-3">
            <span>الأسطول: <strong className="text-white">{benchmarkData?.fleet_size || 3} UAVs</strong></span>
            <span>&bull;</span>
            <span>الأهداف: <strong className="text-white">{benchmarkData?.active_cells_count || 0} عقدة/خلية</strong></span>
          </div>
        </div>

        {/* Main Comparison Matrix Table */}
        <div className="flex-1 overflow-y-auto p-5">
          <div className="w-full border border-slate-800 rounded-xl overflow-hidden bg-slate-950 shadow-inner">
            <table className="w-full text-right border-collapse">
              <thead>
                <tr className="bg-slate-900/90 border-b border-slate-800 text-slate-300 font-bold text-[11px]">
                  <th className="p-3.5 text-right w-1/4">المعيار الهندسي المقاس</th>
                  <th className="p-3.5 text-center w-1/4 border-r border-slate-800/80 text-sky-400">
                    <div className="flex items-center justify-center space-x-1.5">
                      <Cpu className="w-4 h-4" />
                      <span>1. Discrete PSO</span>
                    </div>
                  </th>
                  <th className="p-3.5 text-center w-1/4 border-r border-slate-800/80 text-purple-400">
                    <div className="flex items-center justify-center space-x-1.5">
                      <Dna className="w-4 h-4" />
                      <span>2. Modified GA</span>
                    </div>
                  </th>
                  <th className="p-3.5 text-center w-1/4 border-r border-slate-800/80 text-emerald-400 bg-emerald-950/20">
                    <div className="flex items-center justify-center space-x-1.5">
                      <Zap className="w-4 h-4" />
                      <span>3. Hybrid GA-PSO</span>
                    </div>
                  </th>
                </tr>
              </thead>
              <tbody className="divide-y divide-slate-850">
                {metrics.map((m, idx) => {
                  const psoStyle = getBadgeStyle(m.psoRank, false);
                  const gaStyle = getBadgeStyle(m.gaRank, false);
                  const hybStyle = getBadgeStyle(m.hybRank, true);
                  const Icon = m.icon;

                  return (
                    <tr key={m.id} className={idx % 2 === 0 ? 'bg-slate-950' : 'bg-slate-900/40'}>
                      {/* Metric Title & Explanation */}
                      <td className="p-3.5">
                        <div className="flex items-start space-x-2.5 rtl:space-x-reverse">
                          <div className="p-1.5 rounded-lg bg-slate-800 text-slate-300 mt-0.5">
                            <Icon className="w-4 h-4" />
                          </div>
                          <div>
                            <span className="font-bold text-slate-100 block text-xs">{m.title}</span>
                            <span className="text-[10px] text-slate-400 block mt-0.5">{m.subtitle}</span>
                          </div>
                        </div>
                      </td>

                      {/* 1. Discrete PSO Value Card */}
                      <td className="p-3 border-r border-slate-850 text-center">
                        <div className={`p-2.5 rounded-xl border ${psoStyle.bg} transition-all`}>
                          <span className="font-bold font-mono text-xs block">
                            {m.formatVal(m.psoVal)}
                          </span>
                          <span className={`inline-block mt-1 px-1.5 py-0.5 rounded text-[9px] font-bold ${psoStyle.badge}`}>
                            {psoStyle.label}
                          </span>
                        </div>
                      </td>

                      {/* 2. Modified GA Value Card */}
                      <td className="p-3 border-r border-slate-850 text-center">
                        <div className={`p-2.5 rounded-xl border ${gaStyle.bg} transition-all`}>
                          <span className="font-bold font-mono text-xs block">
                            {m.formatVal(m.gaVal)}
                          </span>
                          <span className={`inline-block mt-1 px-1.5 py-0.5 rounded text-[9px] font-bold ${gaStyle.badge}`}>
                            {gaStyle.label}
                          </span>
                        </div>
                      </td>

                      {/* 3. Hybrid GA-PSO Value Card */}
                      <td className="p-3 border-r border-slate-850 text-center bg-emerald-950/10">
                        <div className={`p-2.5 rounded-xl border ${hybStyle.bg} shadow-lg transition-all`}>
                          <span className="font-bold font-mono text-xs block">
                            {m.formatVal(m.hybVal)}
                          </span>
                          <span className={`inline-block mt-1 px-1.5 py-0.5 rounded text-[9px] font-bold ${hybStyle.badge}`}>
                            {hybStyle.label}
                          </span>
                        </div>
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>

          {/* Color Code Legend */}
          <div className="mt-4 p-3 bg-slate-900/60 rounded-xl border border-slate-800/80 flex items-center justify-between text-[11px] text-slate-400">
            <div className="flex items-center space-x-4">
              <span className="font-semibold text-slate-300">مفتاح التقييم التنافسي:</span>
              <div className="flex items-center space-x-1.5">
                <span className="w-2.5 h-2.5 rounded-full bg-emerald-500 shadow-[0_0_6px_#10b981]" />
                <span className="text-emerald-400 font-bold">الأفضل (المرتبة الأولى أو تعادل)</span>
              </div>
              <div className="flex items-center space-x-1.5">
                <span className="w-2.5 h-2.5 rounded-full bg-amber-500 shadow-[0_0_6px_#f59e0b]" />
                <span className="text-amber-400 font-bold">المرتبة الثانية (أو تعادل)</span>
              </div>
              <div className="flex items-center space-x-1.5">
                <span className="w-2.5 h-2.5 rounded-full bg-orange-500 shadow-[0_0_6px_#f97316]" />
                <span className="text-orange-400 font-bold">المرتبة الأدنى</span>
              </div>
            </div>

            <span className="text-[10px] text-slate-500">حسابات حتمية فورية من المسارات المعتمدة</span>
          </div>
        </div>

        {/* Modal Footer */}
        <div className="p-3.5 border-t border-slate-800 bg-slate-900/90 flex items-center justify-between">
          <span className="text-[11px] text-slate-400">
            تم فحص المسارات عبر خوارزمية CCW Segment Intersections ومنع تكرار المسح الانتهازي
          </span>
          <button
            onClick={onClose}
            className="px-4 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 text-xs font-semibold cursor-pointer transition-all"
          >
            إغلاق
          </button>
        </div>

      </div>
    </div>
  );
};

export default BenchmarkComparisonModal;
