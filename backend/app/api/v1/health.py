from fastapi import APIRouter
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
