import asyncio
import json
from typing import List, Dict, Any
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()

class TelemetryConnectionManager:
    """Thread-safe WebSocket Connection Hub for real-time telemetry streaming."""
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
        """Broadcasts JSON message to all connected frontend clients."""
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
