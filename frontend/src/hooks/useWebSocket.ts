import { useState, useEffect, useRef, useCallback } from 'react';
import { TelemetryPacket } from '../types/telemetry';

export function useWebSocket(url: string) {
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [lastPacket, setLastPacket] = useState<TelemetryPacket | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<any>();

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
