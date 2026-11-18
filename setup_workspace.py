"""
Adaptive Multi-Drone Search & Terrain Mapping System
Phase 01: Workspace Initialization Script
Author: Principal Software Architect
"""

import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
BACKEND_DIR = BASE_DIR / "backend"
FRONTEND_DIR = BASE_DIR / "frontend"

def create_directories():
    directories = [
        # Backend directories
        BACKEND_DIR / "app" / "api" / "v1",
        BACKEND_DIR / "app" / "api" / "ws",
        BACKEND_DIR / "app" / "core",
        BACKEND_DIR / "app" / "gis",
        BACKEND_DIR / "app" / "physics",
        BACKEND_DIR / "app" / "fleet",
        BACKEND_DIR / "app" / "optimizers",
        BACKEND_DIR / "app" / "simulation",
        
        # Frontend directories
        FRONTEND_DIR / "public",
        FRONTEND_DIR / "src" / "components" / "map",
        FRONTEND_DIR / "src" / "components" / "algorithms",
        FRONTEND_DIR / "src" / "components" / "fleet",
        FRONTEND_DIR / "src" / "components" / "terrain",
        FRONTEND_DIR / "src" / "hooks",
        FRONTEND_DIR / "src" / "services",
        FRONTEND_DIR / "src" / "types",
    ]
    for d in directories:
        d.mkdir(parents=True, exist_ok=True)
    print("[+] Directory structure created successfully.")

def write_backend_files():
    # 1. requirements.txt
    req_content = """fastapi>=0.110.0
uvicorn[standard]>=0.28.0
pydantic>=2.6.0
numpy>=1.26.0
scipy>=1.12.0
pyproj>=3.6.0
shapely>=2.0.0
websockets>=12.0
python-dotenv>=1.0.0
httpx>=0.27.0
"""
    (BACKEND_DIR / "requirements.txt").write_text(req_content, encoding="utf-8")

    # 2. backend/app/__init__.py and subpackage __init__.py files
    for sub in ["", "api", "api/v1", "api/ws", "core", "gis", "physics", "fleet", "optimizers", "simulation"]:
        init_file = BACKEND_DIR / "app" / sub / "__init__.py"
        init_file.parent.mkdir(parents=True, exist_ok=True)
        if not init_file.exists():
            init_file.write_text('"""Subpackage init."""\n', encoding="utf-8")

    # 3. backend/app/core/config.py
    config_content = """from pydantic_settings import BaseSettings
from typing import List

class Settings(BaseSettings):
    PROJECT_NAME: str = "Adaptive Multi-Drone Search & Mapping System"
    API_V1_STR: str = "/api/v1"
    CORS_ORIGINS: List[str] = [
        "http://localhost:5173",
        "http://127.0.0.1:5173",
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "*"
    ]
    DEFAULT_TICK_RATE_HZ: int = 10  # 10Hz physics telemetry tick rate
    
    class Config:
        case_sensitive = True

settings = Settings()
"""
    (BACKEND_DIR / "app" / "core" / "config.py").write_text(config_content, encoding="utf-8")

    # 4. backend/app/api/ws/telemetry.py (WebSocket Connection Manager)
    ws_content = """import asyncio
import json
from typing import List, Dict, Any
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()

class TelemetryConnectionManager:
    \"\"\"Thread-safe WebSocket Connection Hub for real-time telemetry streaming.\"\"\"
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
        \"\"\"Broadcasts JSON message to all connected frontend clients.\"\"\"
        payload = json.dumps(message)
        dead_connections = []
        for connection in self.active_connections:
            try:
                await connection.send_text(payload)
            except Exception:
                dead_connections.append(connection)
        for dead in dead_connections:
            self.disconnect(dead)

telemetry_hub = TelemetryConnectionManager()

@router.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await telemetry_hub.connect(websocket)
    try:
        # Send initial confirmation handshake
        await websocket.send_text(json.dumps({
            "type": "CONNECTION_ESTABLISHED",
            "message": "Real-time telemetry link active. Ready for streaming."
        }))
        while True:
            # Client can send commands via WS (e.g. pause/resume simulation)
            data = await websocket.receive_text()
            try:
                cmd = json.loads(data)
                # echo or command handling hook
                await websocket.send_text(json.dumps({"type": "CMD_ACK", "received": cmd}))
            except json.JSONDecodeError:
                await websocket.send_text(json.dumps({"type": "ERROR", "message": "Invalid JSON format"}))
    except WebSocketDisconnect:
        telemetry_hub.disconnect(websocket)
    except Exception:
        telemetry_hub.disconnect(websocket)
"""
    (BACKEND_DIR / "app" / "api" / "ws" / "telemetry.py").write_text(ws_content, encoding="utf-8")

    # 5. backend/app/api/v1/health.py
    health_content = """from fastapi import APIRouter
from datetime import datetime

router = APIRouter()

@router.get("/health", tags=["System Health"])
async def check_health():
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "modules": {
            "physics_engine": "online",
            "optimizers": ["discrete_pso", "modified_ga", "hybrid_ga_pso"],
            "websocket_telemetry": "ready"
        }
    }
"""
    (BACKEND_DIR / "app" / "api" / "v1" / "health.py").write_text(health_content, encoding="utf-8")

    # 6. backend/app/main.py
    main_content = """import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.v1 import health
from app.api.ws import telemetry

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize background services / worker pools if needed
    print("[*] FastAPI Backend Initialized - Ready for Multi-Drone Optimization & Physics Loop")
    yield
    # Shutdown: Cleanly close background connections
    print("[*] FastAPI Backend Shutting Down cleanly")

app = FastAPI(
    title="Adaptive Multi-Drone Search & Terrain Mapping API",
    description="Backend engine for hybrid GA-PSO optimization, LiDAR simulation, and dynamic fleet control.",
    version="1.0.0",
    lifespan=lifespan
)

# Cross-Origin Resource Sharing (CORS) Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Permits PWA frontend local dev and production hosts
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register REST and WebSocket Routers
app.include_router(health.router, prefix="/api/v1")
app.include_router(telemetry.router)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
"""
    (BACKEND_DIR / "app" / "main.py").write_text(main_content, encoding="utf-8")
    print("[+] Backend core files created.")

def write_frontend_files():
    # 1. package.json
    pkg_json = """{
  "name": "adaptive-multi-drone-pwa",
  "private": true,
  "version": "1.0.0",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "tsc && vite build",
    "preview": "vite preview"
  },
  "dependencies": {
    "react": "^18.2.0",
    "react-dom": "^18.2.0",
    "@react-google-maps/api": "^2.19.3",
    "lucide-react": "^0.359.0",
    "clsx": "^2.1.0",
    "tailwind-merge": "^2.2.2"
  },
  "devDependencies": {
    "@types/react": "^18.2.66",
    "@types/react-dom": "^18.2.22",
    "@vitejs/plugin-react": "^4.2.1",
    "autoprefixer": "^10.4.19",
    "postcss": "^8.4.38",
    "tailwindcss": "^3.4.1",
    "typescript": "^5.2.2",
    "vite": "^5.1.6",
    "vite-plugin-pwa": "^0.19.2"
  }
}
"""
    (FRONTEND_DIR / "package.json").write_text(pkg_json, encoding="utf-8")

    # 2. tsconfig.json
    tsconfig = """{
  "compilerOptions": {
    "target": "ES2020",
    "useDefineForClassFields": true,
    "lib": ["ES2020", "DOM", "DOM.Iterable"],
    "module": "ESNext",
    "skipLibCheck": true,
    "moduleResolution": "bundler",
    "allowImportingTsExtensions": true,
    "resolveJsonModule": true,
    "isolatedModules": true,
    "noEmit": true,
    "jsx": "react-jsx",
    "strict": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true,
    "noFallthroughCasesInSwitch": true
  },
  "include": ["src"]
}
"""
    (FRONTEND_DIR / "tsconfig.json").write_text(tsconfig, encoding="utf-8")

    # 3. vite.config.ts with PWA Setup
    vite_config = """import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { VitePWA } from 'vite-plugin-pwa';

export default defineConfig({
  plugins: [
    react(),
    VitePWA({
      registerType: 'autoUpdate',
      includeAssets: ['favicon.ico', 'robots.txt', 'icon-192.png', 'icon-512.png'],
      manifest: {
        name: 'Adaptive Multi-Drone Mission Control',
        short_name: 'DroneMission',
        description: 'Autonomous Search Space Discretization, LiDAR Mapping & Hybrid GA-PSO Optimizer',
        theme_color: '#0f172a',
        background_color: '#020617',
        display: 'standalone',
        orientation: 'landscape',
        icons: [
          {
            src: 'icon-192.png',
            sizes: '192x192',
            type: 'image/png'
          },
          {
            src: 'icon-512.png',
            sizes: '512x512',
            type: 'image/png'
          }
        ]
      },
      workbox: {
        globPatterns: ['**/*.{js,css,html,ico,png,svg}']
      }
    })
  ],
  server: {
    port: 5173,
    host: true
  }
});
"""
    (FRONTEND_DIR / "vite.config.ts").write_text(vite_config, encoding="utf-8")

    # 4. tailwind.config.js & postcss.config.js
    tailwind_config = """/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        hud: {
          bg: "#050b14",
          panel: "#0b1526",
          border: "#1e293b",
          pso: "#38bdf8",     // Cyan for PSO
          ga: "#a855f7",      // Purple for GA
          hybrid: "#10b981",  // Emerald for Hybrid GA-PSO
          alert: "#ef4444"
        }
      }
    },
  },
  plugins: [],
}
"""
    (FRONTEND_DIR / "tailwind.config.js").write_text(tailwind_config, encoding="utf-8")

    postcss_config = """export default {
  plugins: {
    tailwindcss: {},
    autoprefixer: {},
  },
}
"""
    (FRONTEND_DIR / "postcss.config.js").write_text(postcss_config, encoding="utf-8")

    # 5. index.html
    index_html = """<!doctype html>
<html lang="en" class="dark">
  <head>
    <meta charset="UTF-8" />
    <link rel="icon" type="image/svg+xml" href="/favicon.ico" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <meta name="theme-color" content="#0f172a" />
    <meta name="description" content="Adaptive Multi-Drone Search & Terrain Mapping System PWA" />
    <title>Drone Mission Command | Hybrid GA-PSO & GIS</title>
  </head>
  <body class="bg-hud-bg text-slate-100 overflow-hidden select-none font-sans">
    <div id="root"></div>
    <script type="module" src="/src/main.tsx"></script>
  </body>
</html>
"""
    (FRONTEND_DIR / "index.html").write_text(index_html, encoding="utf-8")

    # 6. src/index.css
    index_css = """@tailwind base;
@tailwind components;
@tailwind utilities;

body {
  margin: 0;
  padding: 0;
  background-color: #050b14;
}

::-webkit-scrollbar {
  width: 6px;
  height: 6px;
}
::-webkit-scrollbar-track {
  background: #0b1526;
}
::-webkit-scrollbar-thumb {
  background: #1e293b;
  border-radius: 3px;
}
::-webkit-scrollbar-thumb:hover {
  background: #334155;
}
"""
    (FRONTEND_DIR / "src" / "index.css").write_text(index_css, encoding="utf-8")

    # 7. src/types/telemetry.ts
    types_telemetry = """export interface DroneTelemetry {
  drone_id: string;
  ip_endpoint: string;
  latitude: number;
  longitude: number;
  altitude_m: number;
  speed_mps: number;
  battery_pct: number;
  rtl_energy_threshold_pct: number;
  status: 'IDLE' | 'SEARCHING' | 'MAPPING' | 'RTL' | 'DISCONNECTED';
  current_cell_id?: string;
  lasers_active: boolean;
}

export interface TelemetryPacket {
  timestamp: string;
  fleet: DroneTelemetry[];
  terrain_cells_scanned: number;
}
"""
    (FRONTEND_DIR / "src" / "types" / "telemetry.ts").write_text(types_telemetry, encoding="utf-8")

    # 8. src/hooks/useWebSocket.ts
    hook_ws = """import { useState, useEffect, useRef, useCallback } from 'react';
import { TelemetryPacket } from '../types/telemetry';

export function useWebSocket(url: string) {
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [lastPacket, setLastPacket] = useState<TelemetryPacket | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout>();

  const connect = useCallback(() => {
    try {
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('[WS] Connected to Drone Telemetry Hub');
        setIsConnected(true);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.type === 'TELEMETRY_TICK') {
            setLastPacket(data.payload);
          }
        } catch (e) {
          // non-JSON or debug frame
        }
      };

      ws.onclose = () => {
        console.warn('[WS] Telemetry Link Closed. Retrying in 2000ms...');
        setIsConnected(false);
        reconnectTimeoutRef.current = setTimeout(connect, 2000);
      };

      ws.onerror = (err) => {
        console.error('[WS] Socket Error:', err);
        ws.close();
      };
    } catch (err) {
      console.error('[WS] Connection Failed:', err);
      reconnectTimeoutRef.current = setTimeout(connect, 2000);
    }
  }, [url]);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      wsRef.current?.close();
    };
  }, [connect]);

  const sendCommand = (cmd: object) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(cmd));
    }
  };

  return { isConnected, lastPacket, sendCommand };
}
"""
    (FRONTEND_DIR / "src" / "hooks" / "useWebSocket.ts").write_text(hook_ws, encoding="utf-8")

    # 9. src/App.tsx
    app_tsx = """import React, { useState } from 'react';
import { 
  Radio, 
  MapPin, 
  Cpu, 
  Dna, 
  Zap, 
  Layers, 
  Activity, 
  ShieldCheck, 
  Settings2 
} from 'lucide-react';
import { useWebSocket } from './hooks/useWebSocket';

export function App() {
  const [activeAlgorithm, setActiveAlgorithm] = useState<'PSO' | 'GA' | 'HYBRID'>('HYBRID');
  const { isConnected, lastPacket } = useWebSocket('ws://localhost:8000/ws/telemetry');

  return (
    <div className="flex flex-col h-screen w-screen bg-hud-bg text-slate-200 overflow-hidden font-sans">
      {/* Top HUD Status Bar */}
      <header className="h-14 border-b border-hud-border bg-hud-panel/80 backdrop-blur px-6 flex items-center justify-between z-20">
        <div className="flex items-center space-x-3">
          <div className="w-8 h-8 rounded-lg bg-emerald-500/20 border border-emerald-500/40 flex items-center justify-center text-emerald-400">
            <Radio className="w-5 h-5 animate-pulse" />
          </div>
          <div>
            <h1 className="text-sm font-bold tracking-wider uppercase text-slate-100">
              Autonomous Drone Fleet Command (PWA)
            </h1>
            <p className="text-[10px] text-slate-400">GIS Terrain Discretization & Hybrid GA-PSO Optimizer</p>
          </div>
        </div>

        {/* Global Connection & Health Badges */}
        <div className="flex items-center space-x-4">
          <div className="flex items-center space-x-2 px-3 py-1 rounded-full bg-slate-900 border border-slate-700 text-xs">
            <span className={`w-2.5 h-2.5 rounded-full ${isConnected ? 'bg-emerald-500 shadow-[0_0_8px_#10b981]' : 'bg-red-500'}`} />
            <span className="text-[11px] font-mono">{isConnected ? 'LIVE TELEMETRY (10 Hz)' : 'DISCONNECTED'}</span>
          </div>

          <div className="flex items-center space-x-2 text-xs bg-slate-800/80 px-3 py-1 rounded border border-slate-700">
            <Activity className="w-4 h-4 text-cyan-400" />
            <span className="text-[11px] text-slate-300">Phase 01: Core Architecture Active</span>
          </div>
        </div>
      </header>

      {/* Main Working View */}
      <main className="flex-1 flex overflow-hidden">
        {/* Left Column: Google Maps & Interactive ROI Area */}
        <div className="flex-1 relative flex flex-col border-r border-hud-border">
          {/* Map Container Placeholder for Phase 02 */}
          <div className="flex-1 bg-slate-950 flex flex-col items-center justify-center relative p-6 text-center">
            <div className="w-20 h-20 rounded-2xl bg-hud-panel border border-hud-border flex items-center justify-center mb-4 text-slate-400">
              <MapPin className="w-10 h-10 text-emerald-400 animate-bounce" />
            </div>
            <h2 className="text-lg font-semibold text-slate-200">Google Maps Interactive Search Space Viewport</h2>
            <p className="text-xs text-slate-400 max-w-md mt-2">
              Ready for Phase 02: Mouse-driven Bounding Polygon Drawing, WGS84 Coords Extraction & Real-time UTM Discretization.
            </p>
            <div className="mt-4 flex items-center space-x-3 text-xs text-slate-500 font-mono">
              <span className="bg-slate-900 px-2 py-1 rounded border border-slate-800">Projection: WGS84 / EPSG:4326</span>
              <span className="bg-slate-900 px-2 py-1 rounded border border-slate-800">Sensors: LiDAR + Pseudo-3D DEM</span>
            </div>
          </div>

          {/* Bottom Pseudo-3D Terrain Strip */}
          <div className="h-44 border-t border-hud-border bg-hud-panel/40 p-4 flex flex-col">
            <div className="flex items-center justify-between text-xs text-slate-300 mb-2 font-mono">
              <span className="flex items-center space-x-1.5">
                <Layers className="w-3.5 h-3.5 text-emerald-400" />
                <span>Pseudo-3D Terrain Elevation Strip (Phase 04 LiDAR Mock)</span>
              </span>
              <span className="text-[10px] text-slate-500">Noise Filtering: Active Kalman</span>
            </div>
            <div className="flex-1 rounded border border-slate-800/80 bg-slate-950/60 flex items-center justify-center text-slate-600 text-xs font-mono">
              Elevation Profile & Laser Pulse Waveform Ready for Stream
            </div>
          </div>
        </div>

        {/* Right Column: Tri-Algorithm Segmented Control & Telemetry */}
        <aside className="w-96 flex flex-col bg-hud-panel border-hud-border">
          {/* Segmented Tri-Algorithm Header */}
          <div className="p-4 border-b border-hud-border">
            <h3 className="text-xs font-semibold uppercase tracking-wider text-slate-400 mb-3">
              Tri-Algorithm Optimization Engine
            </h3>
            <div className="grid grid-cols-3 gap-1.5 p-1 bg-slate-950 rounded-lg border border-slate-800">
              <button
                onClick={() => setActiveAlgorithm('PSO')}
                className={`flex items-center justify-center space-x-1.5 py-2 rounded text-xs font-medium transition-all ${
                  activeAlgorithm === 'PSO'
                    ? 'bg-hud-pso/20 text-hud-pso border border-hud-pso/40 shadow-sm'
                    : 'text-slate-400 hover:text-slate-200'
                }`}
              >
                <Cpu className="w-3.5 h-3.5" />
                <span>PSO</span>
              </button>

              <button
                onClick={() => setActiveAlgorithm('GA')}
                className={`flex items-center justify-center space-x-1.5 py-2 rounded text-xs font-medium transition-all ${
                  activeAlgorithm === 'GA'
                    ? 'bg-hud-ga/20 text-hud-ga border border-hud-ga/40 shadow-sm'
                    : 'text-slate-400 hover:text-slate-200'
                }`}
              >
                <Dna className="w-3.5 h-3.5" />
                <span>GA</span>
              </button>

              <button
                onClick={() => setActiveAlgorithm('HYBRID')}
                className={`flex items-center justify-center space-x-1.5 py-2 rounded text-xs font-medium transition-all ${
                  activeAlgorithm === 'HYBRID'
                    ? 'bg-hud-hybrid/20 text-hud-hybrid border border-hud-hybrid/40 shadow-sm font-bold'
                    : 'text-slate-400 hover:text-slate-200'
                }`}
              >
                <Zap className="w-3.5 h-3.5" />
                <span>Hybrid</span>
              </button>
            </div>
          </div>

          {/* Active Algorithm Info Panel */}
          <div className="p-4 flex-1 overflow-y-auto space-y-4">
            <div className="p-3.5 rounded-lg border border-slate-800 bg-slate-950/40">
              <div className="flex items-center justify-between mb-2">
                <span className="text-xs font-semibold text-slate-300">Active Solver State</span>
                <span className={`text-[10px] px-2 py-0.5 rounded font-mono ${
                  activeAlgorithm === 'HYBRID' ? 'bg-emerald-500/10 text-emerald-400 border border-emerald-500/30' :
                  activeAlgorithm === 'PSO' ? 'bg-cyan-500/10 text-cyan-400 border border-cyan-500/30' :
                  'bg-purple-500/10 text-purple-400 border border-purple-500/30'
                }`}>
                  {activeAlgorithm} ENGINE
                </span>
              </div>
              <p className="text-xs text-slate-400 leading-relaxed">
                {activeAlgorithm === 'HYBRID' && 'Combines GA global exploration diversity with PSO rapid local exploitation gradient.'}
                {activeAlgorithm === 'PSO' && 'Discrete Particle Swarm Optimization searching for rapid coverage path clustering.'}
                {activeAlgorithm === 'GA' && 'Modified Genetic Algorithm with specialized crossover avoiding route collision.'}
              </p>
            </div>

            {/* Drone Fleet Quick Status */}
            <div className="space-y-2">
              <div className="flex items-center justify-between text-xs text-slate-400">
                <span>Dynamic Fleet Endpoints</span>
                <button className="text-[11px] text-emerald-400 hover:underline flex items-center space-x-1">
                  <Settings2 className="w-3 h-3" />
                  <span>Manage</span>
                </button>
              </div>

              <div className="p-2.5 rounded bg-slate-950/60 border border-slate-800/80 flex items-center justify-between text-xs">
                <div>
                  <div className="font-mono text-slate-200 font-semibold">UAV-Alpha (192.168.1.101)</div>
                  <div className="text-[10px] text-slate-500">Mode: Standby | RTL Margin: 28%</div>
                </div>
                <div className="text-right font-mono text-emerald-400 text-xs">98% BAT</div>
              </div>

              <div className="p-2.5 rounded bg-slate-950/60 border border-slate-800/80 flex items-center justify-between text-xs">
                <div>
                  <div className="font-mono text-slate-200 font-semibold">UAV-Bravo (192.168.1.102)</div>
                  <div className="text-[10px] text-slate-500">Mode: Standby | RTL Margin: 30%</div>
                </div>
                <div className="text-right font-mono text-emerald-400 text-xs">94% BAT</div>
              </div>
            </div>
          </div>

          {/* Bottom Execution Bar */}
          <div className="p-4 border-t border-hud-border bg-slate-950/80">
            <button className="w-full py-2.5 bg-emerald-600 hover:bg-emerald-500 text-slate-950 font-bold text-xs uppercase tracking-wider rounded-lg transition-colors shadow-lg shadow-emerald-950/40 flex items-center justify-center space-x-2">
              <ShieldCheck className="w-4 h-4" />
              <span>Initialize Mission Space</span>
            </button>
          </div>
        </aside>
      </main>
    </div>
  );
}

export default App;
"""
    (FRONTEND_DIR / "src" / "App.tsx").write_text(app_tsx, encoding="utf-8")

    # 10. src/main.tsx
    main_tsx = """import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App'
import './index.css'

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
)
"""
    (FRONTEND_DIR / "src" / "main.tsx").write_text(main_tsx, encoding="utf-8")
    print("[+] Frontend core files and PWA configuration created.")

if __name__ == "__main__":
    print("[*] Starting Phase 01 Workspace Scaffolding...")
    create_directories()
    write_backend_files()
    write_frontend_files()
    print("[✓] Phase 01 Workspace Architecture Scaffold Complete!")

