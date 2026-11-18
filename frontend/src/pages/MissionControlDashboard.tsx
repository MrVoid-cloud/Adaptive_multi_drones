import React, { useState, useEffect, useRef } from 'react';
import { 
  Radio, 
  MapPin, 
  Layers, 
  Activity, 
  Settings2, 
  ShieldAlert, 
  Play, 
  Pause, 
  RotateCcw, 
  FastForward,
  CheckCircle2,
  AlertTriangle,
  FileDown,
  FileSpreadsheet,
  Download,
  Trophy,
  BarChart3
} from 'lucide-react';
import { useTelemetrySocket } from '../hooks/useTelemetrySocket';
import { AlgorithmSegmentedPanel } from '../components/AlgorithmSegmentedPanel';
import { MultiLayerMissionMap, MultiLayerMissionMapRef } from '../components/MultiLayerMissionMap';
import { ManualFleetManager } from '../components/ManualFleetManager';
import { BenchmarkComparisonModal } from '../components/BenchmarkComparisonModal';
import { InstallAppBanner } from '../components/InstallAppBanner';
import { 
  exportMissionGeoJSON, 
  exportElevationDEM, 
  exportTriAlgorithmBenchmarkReport 
} from '../utils/missionExporter';
import { getApiBaseUrl } from '../utils/apiConfig';
import { 
  TriAlgorithmBenchmarkResponse, 
  AlgorithmResult, 
  LiveDroneTelemetryState 
} from '../types/simulation';

export const MissionControlDashboard: React.FC = () => {
  // Session Epoch & Map References
  const [currentMissionId, setCurrentMissionId] = useState<string | null>(null);
  const [homeBase, setHomeBase] = useState<{ lat: number; lon: number } | null>(null);
  const mapRef = useRef<MultiLayerMissionMapRef>(null);

  const { 
    isConnected, 
    telemetry, 
    scannedElevationMap,
    scannedTargetIds,
    clearScanned,
    resetTelemetry,
    setCurrentMissionId: setSocketMissionId,
    recentAlert 
  } = useTelemetrySocket({ missionId: currentMissionId });

  // Benchmark & Active Tab State
  const [benchmarkData, setBenchmarkData] = useState<TriAlgorithmBenchmarkResponse | null>(null);
  const [activeTab, setActiveTab] = useState<'PSO' | 'GA' | 'HYBRID'>('HYBRID');
  const [isSolving, setIsSolving] = useState<boolean>(false);
  const [isLaunching, setIsLaunching] = useState<boolean>(false);
  const [simSpeed, setSimSpeed] = useState<number>(1.0);
  const [launchNotification, setLaunchNotification] = useState<{ type: 'SUCCESS' | 'ERROR'; message: string } | null>(null);
  const [isFleetModalOpen, setIsFleetModalOpen] = useState<boolean>(false);
  const [isComparisonModalOpen, setIsComparisonModalOpen] = useState<boolean>(false);
  const [fleetSize, setFleetSize] = useState<number>(3);

  // Map overlays state
  const [overlayMode, setOverlayMode] = useState<'PRIORITY' | 'TERRAIN' | 'NONE'>('PRIORITY');

  // Comprehensive Mission Hard Reset & Session Invalidation
  const handleHardResetMission = async () => {
    console.log('[Dashboard] Purging current mission session and clearing map overlays...');
    // 1. Purge local React states
    setBenchmarkData(null);
    setCurrentMissionId(null);
    setSocketMissionId(null);
    clearScanned();
    resetTelemetry();

    // 2. Clear all Map overlays & canvas elements
    if (mapRef.current) {
      mapRef.current.clearAllMapOverlays();
    }

    // 3. Command Backend to invalidate mission session and halt simulations
    try {
      await fetch(`${getApiBaseUrl()}/api/v1/mission/clear`, { method: 'POST' });
    } catch (err) {
      console.error('Failed to notify backend of mission clear:', err);
    }
  };

  // Trigger Benchmark with Session Epoch ID verification & safety timeout
  const runBenchmark = async (targetFleetSize?: number, targetMissionId?: string | null) => {
    const size = targetFleetSize || fleetSize;
    const mid = (targetMissionId !== undefined && targetMissionId !== null)
      ? targetMissionId
      : currentMissionId;
    setIsSolving(true);

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 45000); // 45s safety timeout

    try {
      const res = await fetch(`${getApiBaseUrl()}/api/v1/optimize/compare`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        signal: controller.signal,
        body: JSON.stringify({
          mission_id: mid,
          iterations: 15,
          population_size: 15,
          fleet_size: size
        })
      });
      clearTimeout(timeoutId);

      if (res.ok) {
        const data: TriAlgorithmBenchmarkResponse = await res.json();
        setBenchmarkData(data);
        const resolvedMissionId = (data as any).mission_id || mid;
        if (resolvedMissionId) {
          setCurrentMissionId(resolvedMissionId);
          setSocketMissionId(resolvedMissionId);
        }
      } else if (res.status === 409) {
        console.warn('Stale benchmark request rejected by server session lock.');
        setLaunchNotification({
          type: 'ERROR',
          message: 'Conflict: Stale mission session. Re-synced to active map epoch.'
        });
      } else {
        const errJson = await res.json().catch(() => ({}));
        const errMsg = errJson.detail || `Benchmark optimization failed (HTTP ${res.status})`;
        console.error('Benchmark optimization failed:', errMsg);
        setLaunchNotification({
          type: 'ERROR',
          message: errMsg
        });
      }
    } catch (err: any) {
      clearTimeout(timeoutId);
      if (err.name === 'AbortError') {
        console.warn('Benchmark optimization request timed out after 45s.');
        setLaunchNotification({
          type: 'ERROR',
          message: 'Benchmark optimization timed out after 45s. Check network or reduce area.'
        });
      } else {
        console.error('Failed to run optimization benchmark:', err);
        setLaunchNotification({
          type: 'ERROR',
          message: `Network/Server Error: ${err.message || 'Unable to connect to optimizer service.'}`
        });
      }
    } finally {
      setIsSolving(false);
    }
  };

  const handleFleetSizeSelect = (size: number) => {
    setFleetSize(size);
    runBenchmark(size);
  };

  const handleROIStartDrawing = () => {
    // Instantly purge old session, routes, and overlays when operator starts drawing
    handleHardResetMission();
  };

  const handleROICleared = async () => {
    await handleHardResetMission();
    try {
      await fetch(`${getApiBaseUrl()}/api/v1/map/roi`, { method: 'DELETE' });
    } catch (err) {
      console.error('Failed to clear ROI on backend:', err);
    }
  };

  const handleROISubmitted = async (roiData: any) => {
    const newMissionId = roiData?.mission_id || null;
    if (newMissionId) {
      setCurrentMissionId(newMissionId);
      setSocketMissionId(newMissionId);
    }

    if (roiData?.home_base) {
      setHomeBase(roiData.home_base);
      if (mapRef.current) {
        mapRef.current.setHomeBaseLocation(roiData.home_base.lat, roiData.home_base.lon);
      }
      // Auto-relocate shared home base in backend
      try {
        await fetch(`${getApiBaseUrl()}/api/v1/fleet/home-base`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            lat: roiData.home_base.lat,
            lon: roiData.home_base.lon,
            alt_m: 0.0
          })
        });
      } catch (err) {
        console.warn('Failed to update shared home base in backend:', err);
      }
    }

    // Trigger instant tri-algorithm benchmark for the newly confirmed session epoch
    runBenchmark(fleetSize, newMissionId);
  };

  // Deploy and Launch Simulation with Selected Plan (Atomic Pipeline)
  const handleDeployAndLaunch = async (plan: AlgorithmResult) => {
    setIsLaunching(true);
    clearScanned();
    try {
      const res = await fetch(`${getApiBaseUrl()}/api/v1/sim/deploy-and-start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          mission_id: currentMissionId,
          algorithm_name: plan.algorithm,
          routes: plan.routes,
          time_scale: simSpeed
        })
      });

      if (res.ok) {
        const data = await res.json();
        setLaunchNotification({
          type: 'SUCCESS',
          message: `تم اعتماد خطة ${plan.algorithm} (${data.drones_count} طائرات) وإطلاق المحاكاة اللحظية بنجاح!`
        });
        setTimeout(() => setLaunchNotification(null), 4000);
      } else {
        const err = await res.json().catch(() => ({}));
        setLaunchNotification({
          type: 'ERROR',
          message: `فشل الإطلاق: ${err.detail || 'خطأ في خادم المحاكاة'}`
        });
        setTimeout(() => setLaunchNotification(null), 5000);
      }
    } catch (err) {
      console.error('Failed to deploy simulation:', err);
      setLaunchNotification({
        type: 'ERROR',
        message: 'تعذر الاتصال بخادم المحاكاة. تأكد من تشغيل الخادم على المنفذ 8000.'
      });
      setTimeout(() => setLaunchNotification(null), 5000);
    } finally {
      setIsLaunching(false);
    }
  };

  // Simulation Controls
  const handleSimAction = async (action: 'START' | 'PAUSE' | 'RESET' | 'SET_SPEED', speed?: number) => {
    if (action === 'SET_SPEED' && speed) {
      setSimSpeed(speed);
    }
    if (action === 'RESET') {
      clearScanned();
    }
    try {
      await fetch(`${getApiBaseUrl()}/api/v1/sim/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action, time_scale: speed || simSpeed })
      });
    } catch (err) {
      console.error('Sim action failed:', err);
    }
  };

  // Initial Benchmark Fetch
  useEffect(() => {
    runBenchmark();
  }, []);

  const activeResult: AlgorithmResult | undefined = 
    activeTab === 'PSO' ? benchmarkData?.pso_result :
    activeTab === 'GA' ? benchmarkData?.ga_result :
    benchmarkData?.hybrid_result;

  const liveFleet: LiveDroneTelemetryState[] = telemetry?.fleet || [];

  return (
    <div className="flex flex-col h-screen w-screen bg-hud-bg text-slate-100 overflow-hidden font-sans select-none">
      {/* Top HUD Master Bar: Auto-Layout Responsive Header (Zero Overflow / Zero Overlap) */}
      <header className="flex flex-wrap items-center justify-between gap-2.5 px-4 py-2 w-full border-b border-slate-800 bg-hud-panel/95 backdrop-blur z-20 min-h-[56px] shrink-0">
        {/* Brand & Mission Title */}
        <div className="flex items-center space-x-3 shrink-0">
          <div className="w-8 h-8 rounded-lg bg-emerald-500/20 border border-emerald-500/40 flex items-center justify-center text-emerald-400 shrink-0">
            <Radio className="w-5 h-5 animate-pulse" />
          </div>
          <div className="shrink-0">
            <h1 className="text-sm font-bold tracking-wider uppercase text-slate-100 whitespace-nowrap">
              Autonomous Multi-Drone Mission Command
            </h1>
            <p className="text-[10px] text-slate-400 whitespace-nowrap">
              Geodesic GIS Discretization &bull; Hybrid GA-PSO &bull; Real-Time LiDAR DEM
            </p>
          </div>
        </div>

        {/* Adaptive Ticker Alert if any */}
        {recentAlert && (
          <div className="whitespace-nowrap inline-flex items-center space-x-2 px-3 py-1 bg-amber-500/20 border border-amber-500/40 rounded-full text-amber-300 text-xs animate-pulse shrink-0">
            <AlertTriangle className="w-3.5 h-3.5" />
            <span className="font-mono text-[11px]">{recentAlert}</span>
          </div>
        )}

        {/* Simulation Execution Controls & Badges (Clean Auto-Layout Flex Wrap) */}
        <div className="flex items-center flex-wrap gap-2 text-xs font-mono shrink-0">
          {/* Dynamic Fleet Size Selector (2 to 6 UAVs) */}
          <div className="flex items-center space-x-1 bg-slate-900 border border-slate-800 rounded-lg p-1 text-[11px] shrink-0">
            <span className="text-[10px] text-slate-400 px-1 font-sans font-bold whitespace-nowrap">الأسطول:</span>
            {[2, 3, 4, 5, 6].map(n => (
              <button
                key={n}
                onClick={() => handleFleetSizeSelect(n)}
                className={`whitespace-nowrap inline-flex items-center justify-center px-2 py-0.5 rounded text-[11px] font-bold transition-all shrink-0 ${
                  fleetSize === n
                    ? 'bg-emerald-600 text-slate-950 shadow-md shadow-emerald-950/40'
                    : 'text-slate-400 hover:text-white hover:bg-slate-800'
                }`}
                title={`توزيع فضاء البحث على ${n} طائرات مسيرة`}
              >
                {n} UAVs
              </button>
            ))}
          </div>

          {/* Sim Speed Multiplier */}
          <div className="flex items-center space-x-1 bg-slate-900 border border-slate-800 rounded-lg p-1 shrink-0">
            {[1.0, 2.0, 5.0].map(spd => (
              <button
                key={spd}
                onClick={() => handleSimAction('SET_SPEED', spd)}
                className={`whitespace-nowrap inline-flex items-center justify-center px-2 py-0.5 rounded text-[10px] font-bold transition-all shrink-0 ${
                  simSpeed === spd
                    ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40'
                    : 'text-slate-400 hover:text-slate-200'
                }`}
              >
                {spd}x
              </button>
            ))}
          </div>

          {/* Start / Pause / Reset Buttons */}
          <div className="flex items-center space-x-1 shrink-0">
            <button
              onClick={() => handleSimAction('START')}
              className="p-1.5 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-slate-950 shrink-0"
              title="Start Simulation"
            >
              <Play className="w-3.5 h-3.5 fill-current" />
            </button>
            <button
              onClick={() => handleSimAction('PAUSE')}
              className="p-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-300 shrink-0"
              title="Pause Simulation"
            >
              <Pause className="w-3.5 h-3.5" />
            </button>
            <button
              onClick={() => handleSimAction('RESET')}
              className="p-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-300 shrink-0"
              title="Reset Simulation"
            >
              <RotateCcw className="w-3.5 h-3.5" />
            </button>
          </div>

          {/* Live Telemetry Ping */}
          <div className="whitespace-nowrap inline-flex items-center space-x-2 px-3 py-1 rounded-full bg-slate-900 border border-slate-800 shrink-0">
            <span className={`w-2.5 h-2.5 rounded-full shrink-0 ${isConnected ? 'bg-emerald-500 shadow-[0_0_8px_#10b981]' : 'bg-red-500'}`} />
            <span className="text-[11px] font-mono">{isConnected ? 'ONLINE (10 Hz)' : 'OFFLINE'}</span>
          </div>

          {/* Export Mission Artifacts Dropdown / Button Group */}
          <div className="flex items-center space-x-1 bg-slate-900 border border-slate-800 rounded-lg p-1 text-[10px] shrink-0">
            <button
              onClick={() => {
                if (activeResult) {
                  exportMissionGeoJSON(activeResult, { missionName: `${activeTab}_Search_Plan` });
                }
              }}
              disabled={!activeResult}
              className="whitespace-nowrap inline-flex items-center justify-center gap-1 px-2 py-0.5 rounded hover:bg-slate-800 text-slate-300 disabled:opacity-40 shrink-0"
              title="Download Flight Paths & Waypoints (GeoJSON)"
            >
              <FileDown className="w-3 h-3 text-cyan-400" />
              <span>GeoJSON</span>
            </button>
            <button
              onClick={() => exportElevationDEM(scannedElevationMap, 'CSV')}
              disabled={scannedElevationMap.size === 0}
              className="whitespace-nowrap inline-flex items-center justify-center gap-1 px-2 py-0.5 rounded hover:bg-slate-800 text-slate-300 disabled:opacity-40 shrink-0"
              title="Download Pseudo-3D LiDAR Elevation Grid (CSV)"
            >
              <FileSpreadsheet className="w-3 h-3 text-emerald-400" />
              <span>DEM</span>
            </button>
            {benchmarkData && (
              <>
                <button
                  onClick={() => exportTriAlgorithmBenchmarkReport(benchmarkData, 'CSV')}
                  className="whitespace-nowrap inline-flex items-center justify-center gap-1 px-2 py-0.5 rounded hover:bg-slate-800 text-slate-300 shrink-0"
                  title="Download Tri-Algorithm Benchmark Report (CSV)"
                >
                  <Download className="w-3 h-3 text-purple-400" />
                  <span>KPIs</span>
                </button>
                <button
                  onClick={() => setIsComparisonModalOpen(true)}
                  className="whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-2.5 py-0.5 rounded-lg bg-emerald-950/80 hover:bg-emerald-900/80 text-emerald-300 border border-emerald-500/50 font-bold transition-all shadow-[0_0_8px_rgba(16,185,129,0.25)] shrink-0"
                  title="عرض مصفوفة التحليل الهندسي المقارن لمسارات الخوارزميات الثلاث"
                >
                  <Trophy className="w-3 h-3 text-emerald-400" />
                  <span>مقارنة النتائج</span>
                </button>
              </>
            )}
          </div>

          {/* Manage Fleet Trigger */}
          <button
            onClick={() => setIsFleetModalOpen(true)}
            className="whitespace-nowrap inline-flex items-center justify-center gap-1 px-2.5 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-emerald-400 border border-slate-700 text-xs font-sans shrink-0"
          >
            <Settings2 className="w-3.5 h-3.5" />
            <span>Fleet</span>
          </button>

          {/* PWA Standalone Install Trigger */}
          <InstallAppBanner />
        </div>
      </header>

      {/* Launch Notification Toast */}
      {launchNotification && (
        <div className="absolute top-16 left-1/2 -translate-x-1/2 z-50 px-4 py-2 rounded-xl backdrop-blur shadow-2xl flex items-center space-x-2 text-xs font-mono transition-all animate-fade-in border"
             style={{
               backgroundColor: launchNotification.type === 'SUCCESS' ? 'rgba(6, 78, 59, 0.95)' : 'rgba(127, 29, 29, 0.95)',
               borderColor: launchNotification.type === 'SUCCESS' ? '#10b981' : '#ef4444',
               color: '#f8fafc'
             }}
        >
          <span>{launchNotification.type === 'SUCCESS' ? '🚀' : '⚠️'}</span>
          <span className="font-bold">{launchNotification.message}</span>
        </div>
      )}

      {/* Main View: Full-Height Map (Left) & Segmented Tri-Algorithm Panel (Right) */}
      <main className="flex-1 flex overflow-hidden">
        {/* Left Column: Full-Height Geodesic Map */}
        <div className="flex-1 relative overflow-hidden border-r border-hud-border">
          <MultiLayerMissionMap
            ref={mapRef}
            activeAlgorithmResult={activeResult}
            liveFleet={liveFleet}
            activeOverlayMode={overlayMode}
            onOverlayModeChange={setOverlayMode}
            onROIStartDrawing={handleROIStartDrawing}
            onROICleared={handleROICleared}
            onROISubmitted={handleROISubmitted}
            isSolving={isSolving}
            scannedTargetIds={scannedTargetIds}
            homeBase={homeBase}
          />
        </div>

        {/* Right Column: Ordered Tri-Algorithm Panel (PSO -> GA -> Hybrid) */}
        <AlgorithmSegmentedPanel
          benchmarkData={benchmarkData}
          activeTab={activeTab}
          onTabChange={setActiveTab}
          onLaunchSimulation={handleDeployAndLaunch}
          onRunBenchmark={runBenchmark}
          isLoading={isSolving}
          isLaunching={isLaunching}
        />
      </main>

      {/* Manual Fleet Manager Modal */}
      {isFleetModalOpen && (
        <div className="fixed inset-0 z-50 bg-black/75 backdrop-blur-sm flex items-center justify-center p-4">
          <div className="bg-slate-950 border border-hud-border rounded-2xl w-full max-w-xl max-h-[85vh] flex flex-col shadow-2xl overflow-hidden">
            <div className="flex items-center justify-between p-4 border-b border-slate-800">
              <h3 className="text-sm font-bold uppercase tracking-wider text-slate-100 flex items-center space-x-2">
                <Settings2 className="w-4 h-4 text-emerald-400" />
                <span>Manual Dynamic Drone Registry</span>
              </h3>
              <button
                onClick={() => setIsFleetModalOpen(false)}
                className="text-slate-400 hover:text-slate-200 text-xs px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700"
              >
                Close
              </button>
            </div>
            <div className="flex-1 overflow-y-auto">
              <ManualFleetManager />
            </div>
          </div>
        </div>
      )}
      {/* Computational Route Analytics & Benchmark Comparison Modal */}
      <BenchmarkComparisonModal
        isOpen={isComparisonModalOpen}
        onClose={() => setIsComparisonModalOpen(false)}
        benchmarkData={benchmarkData}
      />
    </div>
  );
};

export default MissionControlDashboard;
