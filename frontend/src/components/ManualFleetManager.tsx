import React, { useState, useEffect } from 'react';
import { 
  Plus, 
  Trash2, 
  ArrowUp, 
  ArrowDown, 
  Radio, 
  Wifi, 
  Cpu, 
  Battery, 
  Gauge, 
  AlertCircle,
  RefreshCw,
  CheckCircle2
} from 'lucide-react';
import { getApiBaseUrl } from '../utils/apiConfig';

interface DroneConfig {
  drone_id: string;
  name: string;
  endpoint_uri: string;
  battery_capacity_wh: number;
  cruise_speed_mps: number;
  initial_lat: number;
  initial_lon: number;
  initial_alt_m: number;
}

interface DroneTelemetry {
  drone_id: string;
  endpoint_uri: string;
  latitude: number;
  longitude: number;
  altitude_m: number;
  speed_mps: number;
  battery_pct: number;
  status: 'READY' | 'STANDBY' | 'SEARCHING' | 'MAPPING' | 'RTL' | 'OFFLINE' | 'SORTIE_FLYING' | 'RETURNING_FOR_SWAP' | 'BATTERY_SWAPPING' | 'RELAUNCHING_SORTIE' | 'MISSION_COMPLETED' | string;
  laser_distance_m?: number;
}

interface FleetItem {
  config: DroneConfig;
  telemetry: DroneTelemetry;
}

export const ManualFleetManager: React.FC = () => {
  const [fleet, setFleet] = useState<FleetItem[]>([]);
  const [loading, setLoading] = useState<boolean>(false);
  const [errorMessage, setErrorMessage] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  // Form State for manual entry
  const [droneId, setDroneId] = useState<string>('UAV-01');
  const [endpointUri, setEndpointUri] = useState<string>('sim://uav-01');
  const [batteryWh, setBatteryWh] = useState<number>(120);
  const [cruiseSpeed, setCruiseSpeed] = useState<number>(14);

  // Fetch active fleet
  const fetchFleet = async () => {
    try {
      const res = await fetch(`${getApiBaseUrl()}/api/v1/fleet`);
      if (res.ok) {
        const data = await res.json();
        setFleet(data.fleet);
      }
    } catch (err) {
      console.warn('Backend offline or not reachable yet.');
    }
  };

  useEffect(() => {
    fetchFleet();
    const interval = setInterval(fetchFleet, 3000);
    return () => clearInterval(interval);
  }, []);

  // Handle Add Drone
  const handleAddDrone = async (e: React.FormEvent) => {
    e.preventDefault();
    setErrorMessage(null);
    setSuccessMessage(null);
    setLoading(true);

    const payload: DroneConfig = {
      drone_id: droneId.trim(),
      name: droneId.trim(),
      endpoint_uri: endpointUri.trim(),
      battery_capacity_wh: Number(batteryWh),
      cruise_speed_mps: Number(cruiseSpeed),
      initial_lat: 33.3152,
      initial_lon: 44.3661,
      initial_alt_m: 60.0
    };

    try {
      const res = await fetch(`${getApiBaseUrl()}/api/v1/fleet/add`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      if (!res.ok) {
        const errData = await res.json();
        throw new Error(errData.detail || 'Failed to add drone');
      }

      setSuccessMessage(`Drone ${droneId} registered successfully!`);
      // Auto-increment ID for next entry convenience
      const match = droneId.match(/\d+$/);
      if (match) {
        const nextNum = String(parseInt(match[0], 10) + 1).padStart(match[0].length, '0');
        const nextId = droneId.replace(/\d+$/, nextNum);
        setDroneId(nextId);
        setEndpointUri(`sim://${nextId.toLowerCase()}`);
      }
      await fetchFleet();
    } catch (err: any) {
      setErrorMessage(err.message || 'Error registering drone');
    } finally {
      setLoading(false);
    }
  };

  // Handle Remove Drone
  const handleRemoveDrone = async (id: string) => {
    setErrorMessage(null);
    try {
      const res = await fetch(`${getApiBaseUrl()}/api/v1/fleet/${id}`, {
        method: 'DELETE'
      });
      if (!res.ok) throw new Error('Failed to remove drone');
      await fetchFleet();
    } catch (err: any) {
      setErrorMessage(err.message || 'Error removing drone');
    }
  };

  // Handle Reorder
  const handleMove = async (index: number, direction: 'UP' | 'DOWN') => {
    const targetIndex = direction === 'UP' ? index - 1 : index + 1;
    if (targetIndex < 0 || targetIndex >= fleet.length) return;

    const newFleet = [...fleet];
    const temp = newFleet[index];
    newFleet[index] = newFleet[targetIndex];
    newFleet[targetIndex] = temp;
    setFleet(newFleet);

    const newOrder = newFleet.map(item => item.config.drone_id);
    try {
      await fetch(`${getApiBaseUrl()}/api/v1/fleet/reorder`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(newOrder)
      });
    } catch (err) {
      console.error('Reorder failed', err);
    }
  };

  return (
    <div className="flex flex-col h-full bg-slate-950 text-slate-200 p-4 select-none font-sans overflow-y-auto">
      {/* Header */}
      <div className="flex items-center justify-between pb-3 border-b border-slate-800 mb-4">
        <div className="flex items-center space-x-2">
          <Radio className="w-5 h-5 text-emerald-400 animate-pulse" />
          <h2 className="text-sm font-bold uppercase tracking-wider text-slate-100">
            Manual Dynamic Drone Registry
          </h2>
        </div>
        <button
          onClick={fetchFleet}
          className="p-1.5 hover:bg-slate-800 rounded text-slate-400 hover:text-slate-200 transition-colors"
          title="Refresh Fleet Status"
        >
          <RefreshCw className="w-4 h-4" />
        </button>
      </div>

      {/* Manual Entry Form */}
      <form onSubmit={handleAddDrone} className="bg-slate-900/80 p-3.5 rounded-xl border border-slate-800 mb-4 space-y-3">
        <div className="text-xs font-semibold text-slate-300 flex items-center justify-between">
          <span>Register New Drone Endpoint</span>
          <span className="text-[10px] text-emerald-400 font-mono">Zero Hardcoding Mode</span>
        </div>

        <div className="grid grid-cols-2 gap-2 text-xs">
          <div>
            <label className="text-[10px] text-slate-400 block mb-1">Drone Identifier</label>
            <input
              type="text"
              value={droneId}
              onChange={e => setDroneId(e.target.value)}
              className="w-full bg-slate-950 border border-slate-800 rounded px-2.5 py-1.5 text-slate-200 focus:outline-none focus:border-emerald-500 font-mono text-xs"
              placeholder="e.g. UAV-Alpha"
              required
            />
          </div>

          <div>
            <label className="text-[10px] text-slate-400 block mb-1">Endpoint URI</label>
            <input
              type="text"
              value={endpointUri}
              onChange={e => setEndpointUri(e.target.value)}
              className="w-full bg-slate-950 border border-slate-800 rounded px-2.5 py-1.5 text-slate-200 focus:outline-none focus:border-emerald-500 font-mono text-xs"
              placeholder="ws:// or sim://"
              required
            />
          </div>

          <div>
            <label className="text-[10px] text-slate-400 block mb-1">Battery (Wh)</label>
            <input
              type="number"
              value={batteryWh}
              onChange={e => setBatteryWh(Number(e.target.value))}
              className="w-full bg-slate-950 border border-slate-800 rounded px-2.5 py-1.5 text-slate-200 focus:outline-none focus:border-emerald-500 font-mono text-xs"
              min="10"
              max="1000"
              required
            />
          </div>

          <div>
            <label className="text-[10px] text-slate-400 block mb-1">Cruise Speed (m/s)</label>
            <input
              type="number"
              value={cruiseSpeed}
              onChange={e => setCruiseSpeed(Number(e.target.value))}
              className="w-full bg-slate-950 border border-slate-800 rounded px-2.5 py-1.5 text-slate-200 focus:outline-none focus:border-emerald-500 font-mono text-xs"
              min="2"
              max="40"
              required
            />
          </div>
        </div>

        {/* Quick Protocol Presets */}
        <div className="flex items-center space-x-1.5 text-[10px] text-slate-400 pt-1">
          <span>Presets:</span>
          <button
            type="button"
            onClick={() => setEndpointUri(`sim://${droneId.toLowerCase()}`)}
            className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300"
          >
            sim://
          </button>
          <button
            type="button"
            onClick={() => setEndpointUri(`ws://192.168.1.101:8765`)}
            className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300"
          >
            ws://
          </button>
          <button
            type="button"
            onClick={() => setEndpointUri(`udp://192.168.1.102:14550`)}
            className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300"
          >
            udp://
          </button>
        </div>

        {errorMessage && (
          <div className="flex items-center space-x-1.5 text-[11px] text-red-400 bg-red-500/10 p-2 rounded border border-red-500/20">
            <AlertCircle className="w-3.5 h-3.5 flex-shrink-0" />
            <span>{errorMessage}</span>
          </div>
        )}

        {successMessage && (
          <div className="flex items-center space-x-1.5 text-[11px] text-emerald-400 bg-emerald-500/10 p-2 rounded border border-emerald-500/20">
            <CheckCircle2 className="w-3.5 h-3.5 flex-shrink-0" />
            <span>{successMessage}</span>
          </div>
        )}

        <button
          type="submit"
          disabled={loading}
          className="w-full py-2 bg-emerald-600 hover:bg-emerald-500 text-slate-950 font-bold text-xs uppercase tracking-wider rounded transition-colors flex items-center justify-center space-x-1.5 disabled:opacity-50"
        >
          <Plus className="w-4 h-4" />
          <span>{loading ? 'Connecting Endpoint...' : 'Add Drone to Active Fleet'}</span>
        </button>
      </form>

      {/* Fleet List */}
      <div className="flex-1 space-y-2">
        <div className="flex justify-between items-center text-[11px] text-slate-400 px-1">
          <span>Active Endpoints ({fleet.length})</span>
          <span>Priority Sequence</span>
        </div>

        {fleet.length === 0 ? (
          <div className="p-6 rounded-xl border border-dashed border-slate-800 text-center text-slate-500 text-xs">
            No drones registered. Add local or simulated endpoints above.
          </div>
        ) : (
          fleet.map((item, idx) => (
            <div
              key={item.config.drone_id}
              className="p-3 bg-slate-900/60 rounded-xl border border-slate-800 flex items-center justify-between text-xs hover:border-slate-700 transition-colors"
            >
              {/* Drone Info */}
              <div className="space-y-1">
                <div className="flex items-center space-x-2">
                  <span className="font-mono font-bold text-slate-200">{item.config.drone_id}</span>
                  <span className={`text-[9px] px-1.5 py-0.5 rounded font-mono font-bold ${
                    item.telemetry.status === 'READY' || item.telemetry.status === 'MISSION_COMPLETED' ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/30' :
                    item.telemetry.status === 'SEARCHING' || item.telemetry.status === 'SORTIE_FLYING' ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/30' :
                    item.telemetry.status === 'BATTERY_SWAPPING' ? 'bg-amber-500/20 text-amber-300 border border-amber-500/30 animate-pulse' :
                    item.telemetry.status === 'RETURNING_FOR_SWAP' || item.telemetry.status === 'RTL' ? 'bg-amber-500/20 text-amber-400 border border-amber-500/30' :
                    item.telemetry.status === 'RELAUNCHING_SORTIE' ? 'bg-emerald-500/20 text-emerald-300 border border-emerald-500/30' :
                    'bg-slate-500/20 text-slate-400 border border-slate-500/30'
                  }`}>
                    {item.telemetry.status}
                  </span>
                  <span className="text-[10px] text-slate-500 font-mono">
                    #{idx + 1}
                  </span>
                </div>

                <div className="text-[11px] font-mono text-slate-400">
                  {item.config.endpoint_uri}
                </div>

                {/* Metrics */}
                <div className="flex items-center space-x-3 text-[10px] text-slate-400 font-mono pt-1">
                  <span className="flex items-center space-x-1">
                    <Battery className={`w-3 h-3 ${item.telemetry.battery_pct > 30 ? 'text-emerald-400' : 'text-red-400'}`} />
                    <span>{item.telemetry.battery_pct}%</span>
                  </span>
                  <span className="flex items-center space-x-1">
                    <Gauge className="w-3 h-3 text-cyan-400" />
                    <span>{item.config.cruise_speed_mps} m/s</span>
                  </span>
                  {item.telemetry.laser_distance_m && (
                    <span className="text-slate-500">
                      LiDAR: {item.telemetry.laser_distance_m}m
                    </span>
                  )}
                </div>
              </div>

              {/* Action Buttons: Move Up/Down & Delete */}
              <div className="flex items-center space-x-1">
                <button
                  onClick={() => handleMove(idx, 'UP')}
                  disabled={idx === 0}
                  className="p-1 text-slate-400 hover:text-slate-200 disabled:opacity-20 hover:bg-slate-800 rounded"
                  title="Move Priority Up"
                >
                  <ArrowUp className="w-3.5 h-3.5" />
                </button>

                <button
                  onClick={() => handleMove(idx, 'DOWN')}
                  disabled={idx === fleet.length - 1}
                  className="p-1 text-slate-400 hover:text-slate-200 disabled:opacity-20 hover:bg-slate-800 rounded"
                  title="Move Priority Down"
                >
                  <ArrowDown className="w-3.5 h-3.5" />
                </button>

                <button
                  onClick={() => handleRemoveDrone(item.config.drone_id)}
                  className="p-1 text-red-400 hover:text-red-300 hover:bg-red-500/10 rounded ml-1"
                  title="Remove from Fleet"
                >
                  <Trash2 className="w-3.5 h-3.5" />
                </button>
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
};

export default ManualFleetManager;
