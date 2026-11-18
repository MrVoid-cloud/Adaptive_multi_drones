"""
Mission Session Epoch Management Routes
Handles atomic session invalidation, clear commands, and mission synchronization.
Author: Principal Software Architect
"""

from fastapi import APIRouter, HTTPException, status
from app.core.mission_state import mission_state

router = APIRouter(prefix="/mission", tags=["Mission Lifecycle & Session Epoch"])

@router.post(
    "/clear",
    summary="Hard Reset Active Mission & Invalidate Stale Loops"
)
async def clear_mission():
    """
    Forcibly halts running simulations, clears active targets,
    sets mission_id to None, and broadcasts MISSION_CLEARED via WebSocket.
    """
    await mission_state.hard_reset()
    return {
        "status": "MISSION_CLEARED",
        "message": "Active mission session hard-reset successfully. All simulations and targets wiped."
    }

@router.get(
    "/current",
    summary="Get Current Mission Session Epoch Information"
)
def get_current_mission():
    """Returns metadata for the currently active mission session."""
    return mission_state.to_dict()
