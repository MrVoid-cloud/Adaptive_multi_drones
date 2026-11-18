import sys
sys.path.insert(0, 'backend')
print('Importing time...', flush=True)
import time
print('Importing logging...', flush=True)
import logging
print('Importing uuid...', flush=True)
from uuid import uuid4
print('Importing simulator_engine...', flush=True)
from app.simulation.simulator_engine import global_simulator
print('Importing telemetry...', flush=True)
from app.api.ws.telemetry import telemetry_hub
print('Importing mission_state...', flush=True)
from app.core.mission_state import mission_state
print('All imports done successfully!', flush=True)
import asyncio

async def test_run():
    print('Testing start_new_mission...', flush=True)
    mid = mission_state.start_new_mission(home_base={'lat': 33.31, 'lon': 44.36})
    print('Mission started:', mid, flush=True)
    print('Testing hard_reset...', flush=True)
    await mission_state.hard_reset()
    print('Hard reset completed! mission_id is:', mission_state.mission_id, flush=True)

asyncio.run(test_run())
print('SUCCESSFULLY FINISHED TEST!', flush=True)
