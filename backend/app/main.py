import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.v1 import health, map_routes, grid_routes, fleet_routes, optimize_routes, simulation_routes, adaptive_routes, mission_routes
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
app.include_router(map_routes.router, prefix="/api/v1")
app.include_router(grid_routes.router, prefix="/api/v1")
app.include_router(fleet_routes.router, prefix="/api/v1")
app.include_router(optimize_routes.router, prefix="/api/v1")
app.include_router(simulation_routes.router, prefix="/api/v1")
app.include_router(adaptive_routes.router, prefix="/api/v1")
app.include_router(mission_routes.router, prefix="/api/v1")
app.include_router(telemetry.router)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
