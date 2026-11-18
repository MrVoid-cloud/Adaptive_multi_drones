from pydantic_settings import BaseSettings
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
