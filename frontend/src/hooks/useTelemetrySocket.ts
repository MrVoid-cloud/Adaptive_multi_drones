import { useState, useEffect, useRef, useCallback } from 'react';
import { LiveTelemetryFrame, ScannedCellUpdate } from '../types/simulation';
import { getWsBaseUrl } from '../utils/apiConfig';

interface UseTelemetrySocketOptions {
  url?: string;
  throttleMs?: number;
  missionId?: string | null;
}

export function useTelemetrySocket({
  url,
  throttleMs = 80, // ~12Hz throttle batching for smooth 60 FPS UI
  missionId = null
}: UseTelemetrySocketOptions = {}) {
  const dynamicUrl = url || getWsBaseUrl();

  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [telemetry, setTelemetry] = useState<LiveTelemetryFrame | null>(null);
  const [recentAlert, setRecentAlert] = useState<string | null>(null);

  // Active Session Epoch Tracking (UUID)
  const currentMissionIdRef = useRef<string | null>(missionId || null);

  useEffect(() => {
    currentMissionIdRef.current = missionId || null;
  }, [missionId]);

  const setCurrentMissionId = useCallback((id: string | null) => {
    currentMissionIdRef.current = id;
  }, []);

  // Scanned targets cache: cell_id / node_id set
  const [scannedTargetIds, setScannedTargetIds] = useState<Set<string | number>>(new Set());
  // Scanned cells elevation cache: cell_id -> elevation_m
  const [scannedElevationMap, setScannedElevationMap] = useState<Map<number, number>>(new Map());

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<any>();
  const lastUpdateRef = useRef<number>(0);
  const pendingFrameRef = useRef<LiveTelemetryFrame | null>(null);

  const clearScanned = useCallback(() => {
    setScannedTargetIds(new Set());
    setScannedElevationMap(new Map());
  }, []);

  const resetTelemetry = useCallback(() => {
    setTelemetry(null);
    clearScanned();
    pendingFrameRef.current = null;
  }, [clearScanned]);

  const connect = useCallback(() => {
    try {
      const ws = new WebSocket(dynamicUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setIsConnected(true);
        console.log('[Telemetry WS] Connected to Mission Control Hub at', dynamicUrl);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          // Handle atomic mission invalidation broadcast from server
          if (data.type === 'MISSION_CLEARED') {
            console.log('[Telemetry WS] Received MISSION_CLEARED - Purging local telemetry');
            resetTelemetry();
            return;
          }

          if (data.type === 'TELEMETRY_TICK') {
            const frame: LiveTelemetryFrame = data.payload;

            // Session Epoch Filter: Reject ghost packets from prior missions
            const frameMissionId = (data.payload as any)?.mission_id || data.mission_id;
            if (
              currentMissionIdRef.current &&
              frameMissionId &&
              frameMissionId !== currentMissionIdRef.current
            ) {
              return; // Stale session packet dropped
            }

            pendingFrameRef.current = frame;

            // Ingest newly scanned target IDs
            const anyPayload = data.payload as any;
            if (anyPayload.newly_scanned_ids && anyPayload.newly_scanned_ids.length > 0) {
              setScannedTargetIds(prev => {
                const next = new Set(prev);
                anyPayload.newly_scanned_ids.forEach((id: string | number) => next.add(String(id)));
                return next;
              });
            }

            // Ingest new scanned cells incrementally
            if (frame.newly_scanned_cells && frame.newly_scanned_cells.length > 0) {
              setScannedTargetIds(prev => {
                const next = new Set(prev);
                frame.newly_scanned_cells.forEach((sc: any) => next.add(String(sc.cell_id)));
                return next;
              });

              setScannedElevationMap(prev => {
                const updated = new Map(prev);
                frame.newly_scanned_cells.forEach((sc: ScannedCellUpdate) => {
                  updated.set(Number(sc.cell_id), sc.elevation_m);
                });
                return updated;
              });
            }

            // Throttled UI State Batching
            const now = performance.now();
            if (now - lastUpdateRef.current >= throttleMs) {
              lastUpdateRef.current = now;
              setTelemetry(pendingFrameRef.current);
            }
          } else if (data.type === 'ADAPTIVE_REPLAN_ALERT') {
            setRecentAlert(data.message || 'Adaptive Re-planning Triggered');
          }
        } catch (err) {
          // ignore non-json frames
        }
      };

      ws.onclose = () => {
        setIsConnected(false);
        reconnectTimeoutRef.current = setTimeout(connect, 2000);
      };

      ws.onerror = () => {
        ws.close();
      };
    } catch (err) {
      reconnectTimeoutRef.current = setTimeout(connect, 2000);
    }
  }, [dynamicUrl, throttleMs]);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      wsRef.current?.close();
    };
  }, [connect]);

  const sendCommand = useCallback((cmd: object) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(cmd));
    }
  }, []);

  return {
    isConnected,
    telemetry,
    scannedTargetIds,
    scannedElevationMap,
    clearScanned,
    resetTelemetry,
    setCurrentMissionId,
    recentAlert,
    sendCommand
  };
}
