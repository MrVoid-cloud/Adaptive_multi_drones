#!/usr/bin/env python3
"""
Autonomous Multi-Drone Search & Terrain Mapping System
Unified Field Launch Orchestrator (Phase 10)

Spawns:
1. FastAPI Backend + Telemetry WebSocket Hub (0.0.0.0:8000)
2. React + TypeScript + Vite PWA Field Dashboard (0.0.0.0:5173)

Enables field engineers and UAV ground operators to connect instantly via local Wi-Fi / Mesh
from any laptop, tablet, or smartphone without internet access.
"""

import sys
import os
import socket
import subprocess
import time
import signal
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent
BACKEND_DIR = ROOT_DIR / "backend"
FRONTEND_DIR = ROOT_DIR / "frontend"


def get_local_ip() -> str:
    """Discovers primary local mesh / LAN IP address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Does not actually send data; determines routing interface
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def print_banner(lan_ip: str):
    """Displays HUD startup banner and field access instructions."""
    banner = f"""
========================================================================================
       AUTONOMOUS MULTI-DRONE MISSION COMMAND - FIELD DEPLOYMENT ORCHESTRATOR
               Progressive Web App (PWA) & Local Mesh Gateway (Phase 10)
========================================================================================
[*] LOCAL MESH IP DETECTED: {lan_ip}

[+] SERVICES LAUNCH MAP:
  * PWA Web & Field App   : http://localhost:5173  |  http://{lan_ip}:5173
  * FastAPI REST API Core  : http://localhost:8000  |  http://{lan_ip}:8000
  * Live Telemetry Stream  : ws://localhost:8000/ws/telemetry
  * OpenAPI / Swagger Docs : http://localhost:8000/docs

[+] FIELD OPERATOR INSTRUCTIONS (Zero-Packaging PWA Installation):
  1. Connect field tablets / laptops to the local drone Wi-Fi mesh.
  2. Navigate to http://{lan_ip}:5173 in Chrome, Safari, or Edge.
  3. Click 'Install App' in the HUD header or browser menu for native full-screen mode.
  4. Workbox Service Worker will automatically cache GIS assets for offline field operations.

========================================================================================
[!] Press Ctrl+C at any time to gracefully terminate all drone services.
========================================================================================
"""
    print(banner)


def main():
    lan_ip = get_local_ip()
    print_banner(lan_ip)

    processes = []

    # 1. Launch FastAPI Backend Process
    backend_cmd = [
        sys.executable,
        "-m", "uvicorn",
        "app.main:app",
        "--host", "0.0.0.0",
        "--port", "8000",
        "--reload"
    ]

    print("[*] Starting FastAPI Backend on 0.0.0.0:8000...")
    try:
        backend_proc = subprocess.Popen(
            backend_cmd,
            cwd=str(BACKEND_DIR),
            env=os.environ.copy()
        )
        processes.append(("FastAPI Backend", backend_proc))
    except Exception as e:
        print(f"[!] Failed to launch backend: {e}")
        sys.exit(1)

    # 2. Launch Vite Frontend PWA Process
    npm_cmd = "npm.cmd" if os.name == "nt" else "npm"
    frontend_cmd = [npm_cmd, "run", "dev", "--", "--host", "0.0.0.0", "--port", "5173"]

    print("[*] Starting React PWA Frontend on 0.0.0.0:5173...")
    try:
        frontend_proc = subprocess.Popen(
            frontend_cmd,
            cwd=str(FRONTEND_DIR),
            env=os.environ.copy(),
            shell=(os.name == "nt")
        )
        processes.append(("Vite PWA Frontend", frontend_proc))
    except Exception as e:
        print(f"[!] Failed to launch frontend: {e}")
        print("[i] Backend is still running. You can run frontend manually using 'cd frontend && npm run dev'.")

    # 3. Graceful Signal Management
    def shutdown(signum, frame):
        print("\n[*] Shutting down all mission services...")
        for name, proc in processes:
            print(f"  -> Terminating {name} (PID: {proc.pid})...")
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                proc.kill()
        print("[✓] All mission services stopped cleanly.")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, shutdown)

    # Keep orchestrator alive
    try:
        while True:
            time.sleep(1)
            for name, proc in processes:
                if proc.poll() is not None:
                    print(f"[!] Warning: {name} exited prematurely with code {proc.returncode}.")
                    shutdown(None, None)
    except KeyboardInterrupt:
        shutdown(None, None)


if __name__ == "__main__":
    main()
