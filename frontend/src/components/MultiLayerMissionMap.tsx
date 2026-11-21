import React, { useState, useRef, useEffect, useMemo, forwardRef, useImperativeHandle, useCallback } from 'react';
import { 
  Layers, 
  Hand, 
  Pencil, 
  RotateCcw, 
  CheckCircle2, 
  Navigation,
  ZoomIn,
  ZoomOut,
  Radio,
  Battery,
  Gauge,
  Compass,
  Cpu,
  Dna,
  Zap,
  Activity,
  Eye,
  EyeOff,
  Crosshair,
  MapPin,
  Route,
  Maximize2
} from 'lucide-react';
import { AlgorithmResult, LiveDroneTelemetryState, Waypoint } from '../types/simulation';
import { getDroneColor, DroneColorDefinition } from '../utils/fleetColors';
import { HybridGridFeature, HybridSpaceStatistics } from '../types/hybrid_geo';
import { getApiBaseUrl } from '../utils/apiConfig';

export interface MultiLayerMissionMapRef {
  clearAllMapOverlays: () => void;
  setHomeBaseLocation: (lat: number, lon: number) => void;
  fitBoundsROI: () => void;
}

interface MultiLayerMissionMapProps {
  activeAlgorithmResult?: AlgorithmResult;
  liveFleet: LiveDroneTelemetryState[];
  activeOverlayMode: 'PRIORITY' | 'TERRAIN' | 'NONE';
  onOverlayModeChange: (mode: 'PRIORITY' | 'TERRAIN' | 'NONE') => void;
  onROISubmitted?: (roi: any) => void;
  onROIStartDrawing?: () => void;
  onROICleared?: () => void;
  onError?: (message: string) => void;
  isSolving?: boolean;
  scannedTargetIds?: Set<string | number>;
  homeBase?: { lat: number; lon: number } | null;
}

// Convert geographic Lon/Lat to World Mercator pixel coordinates at zoom level Z
function lonToWorldX(lon: number, zoom: number): number {
  const scale = 256 * Math.pow(2, zoom);
  return ((lon + 180) / 360) * scale;
}

function latToWorldY(lat: number, zoom: number): number {
  const scale = 256 * Math.pow(2, zoom);
  const sinLat = Math.sin((lat * Math.PI) / 180);
  const clampedSin = Math.max(-0.9999, Math.min(0.9999, sinLat));
  return (0.5 - Math.log((1 + clampedSin) / (1 - clampedSin)) / (4 * Math.PI)) * scale;
}

// Convert screen pixel coordinates back to real geographic Lon/Lat
function pixelToLonLat(
  px: number,
  py: number,
  centerLon: number,
  centerLat: number,
  zoom: number,
  viewWidth: number,
  viewHeight: number
) {
  const scale = 256 * Math.pow(2, zoom);
  const centerWorldX = lonToWorldX(centerLon, zoom);
  const centerWorldY = latToWorldY(centerLat, zoom);

  const worldX = centerWorldX + (px - viewWidth / 2);
  const worldY = centerWorldY + (py - viewHeight / 2);

  const lon = (worldX / scale) * 360 - 180;
  const n = Math.PI - (2 * Math.PI * worldY) / scale;
  const lat = (180 / Math.PI) * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));

  return { lon, lat };
}

// Map geographic Lon/Lat to screen pixel coordinates
function lonLatToScreen(
  lon: number,
  lat: number,
  centerLon: number,
  centerLat: number,
  zoom: number,
  viewWidth: number,
  viewHeight: number
) {
  const centerWorldX = lonToWorldX(centerLon, zoom);
  const centerWorldY = latToWorldY(centerLat, zoom);

  const ptWorldX = lonToWorldX(lon, zoom);
  const ptWorldY = latToWorldY(lat, zoom);

  return {
    x: viewWidth / 2 + (ptWorldX - centerWorldX),
    y: viewHeight / 2 + (ptWorldY - centerWorldY)
  };
}

// Compute visible map tiles for continuous real-world slippy map panning
function getVisibleTiles(
  centerLon: number,
  centerLat: number,
  zoom: number,
  width: number,
  height: number,
  tileUrlPattern: string
) {
  const centerWorldX = lonToWorldX(centerLon, zoom);
  const centerWorldY = latToWorldY(centerLat, zoom);

  const minX = centerWorldX - width / 2;
  const maxX = centerWorldX + width / 2;
  const minY = centerWorldY - height / 2;
  const maxY = centerWorldY + height / 2;

  const minTileX = Math.floor(minX / 256);
  const maxTileX = Math.floor(maxX / 256);
  const minTileY = Math.floor(minY / 256);
  const maxTileY = Math.floor(maxY / 256);

  const maxTiles = Math.pow(2, zoom);
  const tiles: { key: string; url: string; left: number; top: number }[] = [];

  for (let tx = minTileX; tx <= maxTileX; tx++) {
    for (let ty = minTileY; ty <= maxTileY; ty++) {
      if (ty >= 0 && ty < maxTiles) {
        const wrappedTx = ((tx % maxTiles) + maxTiles) % maxTiles;
        tiles.push({
          key: `${zoom}-${wrappedTx}-${ty}`,
          url: tileUrlPattern
            .replace('{z}', zoom.toString())
            .replace('{x}', wrappedTx.toString())
            .replace('{y}', ty.toString()),
          left: tx * 256 - minX,
          top: ty * 256 - minY
        });
      }
    }
  }

  return tiles;
}

// Spherical geodesic polygon area calculation (km^2)
function calculatePolygonAreaKm2(coords: { lat: number; lon: number }[]): number {
  if (coords.length < 3) return 0;
  const R = 6378.137; // Earth radius in km
  let total = 0;

  for (let i = 0; i < coords.length; i++) {
    const j = (i + 1) % coords.length;
    const phi1 = (coords[i].lat * Math.PI) / 180;
    const phi2 = (coords[j].lat * Math.PI) / 180;
    const dLam = ((coords[j].lon - coords[i].lon) * Math.PI) / 180;
    total += dLam * (2 + Math.sin(phi1) + Math.sin(phi2));
  }

  return Math.abs((total * R * R) / 2.0);
}

export const MultiLayerMissionMap = forwardRef<MultiLayerMissionMapRef, MultiLayerMissionMapProps>(({
  activeAlgorithmResult,
  liveFleet,
  activeOverlayMode,
  onOverlayModeChange,
  onROISubmitted,
  onROIStartDrawing,
  onROICleared,
  onError,
  isSolving = false,
  scannedTargetIds = new Set(),
  homeBase = null
}, ref) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Overlay references for deterministic resource disposal
  const polylinesRef = useRef<any[]>([]);
  const droneMarkersRef = useRef<any[]>([]);
  const roadNodesRef = useRef<any[]>([]);
  const roadEdgesRef = useRef<any[]>([]);
  const gridPolygonsRef = useRef<any[]>([]);
  const homeBaseMarkerRef = useRef<any | null>(null);

  // Local Home Base State
  const [localHomeBase, setLocalHomeBase] = useState<{ lat: number; lon: number } | null>(homeBase || null);

  useEffect(() => {
    if (homeBase) {
      setLocalHomeBase(homeBase);
    }
  }, [homeBase]);

  // Map Navigation State (Centered initially on Baghdad Airport / Al-Mansour area)
  const [center, setCenter] = useState<{ lat: number; lon: number }>({ lat: 33.3080, lon: 44.3620 });
  const [zoom, setZoom] = useState<number>(14);
  const [basemap, setBasemap] = useState<'SATELLITE' | 'STREET'>('SATELLITE');

  // Interaction Mode: FREEHAND drawing vs PAN dragging vs SET_BASE
  const [interactionMode, setInteractionMode] = useState<'PAN' | 'FREEHAND' | 'SET_BASE'>('PAN');
  const [isPanning, setIsPanning] = useState<boolean>(false);
  const [panStart, setPanStart] = useState<{ x: number; y: number } | null>(null);

  // Drawing & Pinned ROI State
  const [isDrawing, setIsDrawing] = useState<boolean>(false);
  const [isGenerating, setIsGenerating] = useState<boolean>(false);
  const [currentDrawPixels, setCurrentDrawPixels] = useState<{ x: number; y: number }[]>([]);
  const [pinnedPolygon, setPinnedPolygon] = useState<{ lat: number; lon: number }[] | null>(null);
  const [roiStats, setRoiStats] = useState<{ areaKm2: number; widthKm: number; heightKm: number; vertices: number } | null>(null);

  // Dual-Layer Hybrid GIS State
  const [hybridFeatures, setHybridFeatures] = useState<HybridGridFeature[]>([]);
  const [hybridStats, setHybridStats] = useState<HybridSpaceStatistics | null>(null);
  const [showRoads, setShowRoads] = useState<boolean>(true);
  const [showGrid, setShowGrid] = useState<boolean>(true);
  const [showRoutes, setShowRoutes] = useState<boolean>(true);
  const [highlightedDroneId, setHighlightedDroneId] = useState<string | null>(null);

  // Viewport Dimensions
  const [dimensions, setDimensions] = useState<{ width: number; height: number }>({ width: 1200, height: 800 });

  const handleFitROI = useCallback(() => {
    if (!pinnedPolygon || pinnedPolygon.length === 0) return;
    const lats = pinnedPolygon.map(p => p.lat);
    const lons = pinnedPolygon.map(p => p.lon);
    const minLat = Math.min(...lats);
    const maxLat = Math.max(...lats);
    const minLon = Math.min(...lons);
    const maxLon = Math.max(...lons);
    const newCenter = { lat: (minLat + maxLat) / 2, lon: (minLon + maxLon) / 2 };
    setCenter(newCenter);

    const dLat = Math.max(0.003, maxLat - minLat);
    const dLon = Math.max(0.003, maxLon - minLon);
    const maxSpan = Math.max(dLat, dLon * Math.cos(newCenter.lat * Math.PI / 180));
    const targetZoom = Math.min(17, Math.max(11, Math.round(Math.log2(360 / maxSpan)) - 8));
    setZoom(targetZoom);
  }, [pinnedPolygon]);

  // Comprehensive Resource Disposal & Overlay Purge
  const clearAllMapOverlays = useCallback(() => {
    // 1. Dispose any map overlay references
    polylinesRef.current.forEach(p => p?.setMap && p.setMap(null));
    polylinesRef.current = [];

    droneMarkersRef.current.forEach(m => m?.setMap && m.setMap(null));
    droneMarkersRef.current = [];

    roadNodesRef.current.forEach(n => n?.setMap && n.setMap(null));
    roadNodesRef.current = [];

    roadEdgesRef.current.forEach(e => e?.setMap && e.setMap(null));
    roadEdgesRef.current = [];

    gridPolygonsRef.current.forEach(g => g?.setMap && g.setMap(null));
    gridPolygonsRef.current = [];

    if (homeBaseMarkerRef.current?.setMap) {
      homeBaseMarkerRef.current.setMap(null);
    }
    homeBaseMarkerRef.current = null;

    // 2. Wipe React and Canvas Overlay state
    setPinnedPolygon(null);
    setCurrentDrawPixels([]);
    setHybridFeatures([]);
    setHybridStats(null);
    setRoiStats(null);
    setHighlightedDroneId(null);

    // 3. Clear HTML5 2D Canvas immediately
    const canvas = canvasRef.current;
    if (canvas) {
      const ctx = canvas.getContext('2d');
      if (ctx) {
        ctx.clearRect(0, 0, dimensions.width, dimensions.height);
      }
    }
    console.log('[MultiLayerMissionMap] clearAllMapOverlays executed - All layers and references purged.');
  }, [dimensions]);

  useImperativeHandle(ref, () => ({
    clearAllMapOverlays,
    setHomeBaseLocation: (lat: number, lon: number) => {
      setLocalHomeBase({ lat, lon });
    },
    fitBoundsROI: handleFitROI
  }), [clearAllMapOverlays, handleFitROI]);

  useEffect(() => {
    const updateSize = () => {
      if (containerRef.current) {
        setDimensions({
          width: containerRef.current.clientWidth,
          height: containerRef.current.clientHeight
        });
      }
    };
    updateSize();
    window.addEventListener('resize', updateSize);
    return () => window.removeEventListener('resize', updateSize);
  }, []);

  // Fetch initial hybrid road topology and grid space for default area
  useEffect(() => {
    const fetchDefaultHybridSpace = async () => {
      try {
        const res = await fetch(`${getApiBaseUrl()}/api/v1/grid/hybrid-generate`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            roi_type: 'RECTANGLE',
            bounds: {
              lat_min: 33.3000,
              lat_max: 33.3150,
              lon_min: 44.3500,
              lon_max: 44.3700
            },
            cluster_threshold_m: 35.0,
            manual_cell_size_m: 120.0,
            buffer_radius_ratio: 0.65
          })
        });
        if (res.ok) {
          const data = await res.json();
          if (data.geojson?.features) {
            setHybridFeatures(data.geojson.features);
            setHybridStats(data.statistics);
          }
        }
      } catch (err) {
        console.warn('Initial hybrid space auto-fetch skipped:', err);
      }
    };
    fetchDefaultHybridSpace();
  }, []);

  // Slippy Map Tiles generator
  const tileUrlPattern = basemap === 'SATELLITE'
    ? 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
    : 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';

  const visibleTiles = useMemo(() => {
    return getVisibleTiles(center.lon, center.lat, zoom, dimensions.width, dimensions.height, tileUrlPattern);
  }, [center, zoom, dimensions, tileUrlPattern]);

  // Determine Active Algorithm Theme Indicator
  const activeAlgorithmName = activeAlgorithmResult?.algorithm || 'Hybrid GA-PSO';
  const algorithmTheme = useMemo(() => {
    if (activeAlgorithmName.includes('PSO') && !activeAlgorithmName.includes('Hybrid')) {
      return {
        name: 'Discrete PSO',
        primary: '#38bdf8', // Cyan
        glow: 'rgba(56, 189, 248, 0.4)'
      };
    } else if (activeAlgorithmName.includes('GA') && !activeAlgorithmName.includes('Hybrid')) {
      return {
        name: 'Modified GA',
        primary: '#a855f7', // Purple
        glow: 'rgba(168, 85, 247, 0.4)'
      };
    } else {
      return {
        name: 'Hybrid GA-PSO',
        primary: '#10b981', // Emerald
        glow: 'rgba(16, 185, 129, 0.4)'
      };
    }
  }, [activeAlgorithmName]);

  // Canvas Drawing: Renders Dual-Layer GIS, Per-Drone Colored Routes & Markers
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, dimensions.width, dimensions.height);

    // 0. LAYER D0: Shared Home Base Launch Pad
    if (localHomeBase) {
      const { x, y } = lonLatToScreen(localHomeBase.lon, localHomeBase.lat, center.lon, center.lat, zoom, dimensions.width, dimensions.height);
      
      // Concentric Radar Rings
      ctx.beginPath();
      ctx.arc(x, y, 22, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(16, 185, 129, 0.4)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.beginPath();
      ctx.arc(x, y, 14, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(6, 78, 59, 0.6)';
      ctx.fill();
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 2.0;
      ctx.stroke();

      // Launch Pad 'H' symbol
      ctx.font = 'bold 12px monospace';
      ctx.fillStyle = '#ffffff';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('H', x, y);

      // Label
      ctx.font = 'bold 9px monospace';
      ctx.fillStyle = '#34d399';
      ctx.fillText('SHARED BASE', x, y + 20);
      ctx.textAlign = 'start';
      ctx.textBaseline = 'alphabetic';
    }

    // 1. Draw Pinned Polygon ROI (Locked to Geographic Coordinates)
    if (pinnedPolygon && pinnedPolygon.length > 2) {
      ctx.beginPath();
      pinnedPolygon.forEach((pt, idx) => {
        const { x, y } = lonLatToScreen(pt.lon, pt.lat, center.lon, center.lat, zoom, dimensions.width, dimensions.height);
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();

      // Semi-transparent geographic boundary fill
      ctx.fillStyle = 'rgba(16, 185, 129, 0.12)';
      ctx.fill();
      ctx.strokeStyle = '#34d399';
      ctx.lineWidth = 2.0;
      ctx.stroke();

      pinnedPolygon.forEach((pt) => {
        const { x, y } = lonLatToScreen(pt.lon, pt.lat, center.lon, center.lat, zoom, dimensions.width, dimensions.height);
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fillStyle = '#10b981';
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1;
        ctx.stroke();
      });
    }

    // 2. LAYER A: Free Open Terrain Grid Cells (Polygons) - High-Performance Compound Path Batching
    if (showGrid && hybridFeatures.length > 0) {
      const unscannedCoords: number[][][] = [];
      const scannedCoords: number[][][] = [];

      for (let f = 0; f < hybridFeatures.length; f++) {
        const feat = hybridFeatures[f];
        if (feat.geometry.type === 'Polygon' && feat.properties?.is_masked_out === false) {
          const coords = feat.geometry.coordinates[0];
          if (!coords || coords.length < 3) continue;

          const targetId = String(feat.properties?.cell_id ?? feat.id ?? '');
          const isScanned = scannedTargetIds && (scannedTargetIds.has(targetId) || scannedTargetIds.has(String(feat.properties?.cell_id)));

          if (isScanned) {
            scannedCoords.push(coords);
          } else {
            unscannedCoords.push(coords);
          }
        }
      }

      // Batch 1: Unscanned Cells (A single compound path draw call for thousands of polygons)
      if (unscannedCoords.length > 0) {
        ctx.beginPath();
        for (let i = 0; i < unscannedCoords.length; i++) {
          const coords = unscannedCoords[i];
          for (let j = 0; j < coords.length; j++) {
            const { x, y } = lonLatToScreen(coords[j][0], coords[j][1], center.lon, center.lat, zoom, dimensions.width, dimensions.height);
            if (j === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          }
          ctx.closePath();
        }
        ctx.fillStyle = 'rgba(16, 185, 129, 0.08)';
        ctx.fill();
        ctx.strokeStyle = 'rgba(52, 211, 153, 0.35)';
        ctx.lineWidth = 0.8;
        ctx.stroke();
      }

      // Batch 2: Scanned Cells
      if (scannedCoords.length > 0) {
        ctx.beginPath();
        for (let i = 0; i < scannedCoords.length; i++) {
          const coords = scannedCoords[i];
          for (let j = 0; j < coords.length; j++) {
            const { x, y } = lonLatToScreen(coords[j][0], coords[j][1], center.lon, center.lat, zoom, dimensions.width, dimensions.height);
            if (j === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          }
          ctx.closePath();
        }
        ctx.fillStyle = 'rgba(16, 185, 129, 0.40)';
        ctx.fill();
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.85)';
        ctx.lineWidth = 1.2;
        ctx.stroke();
      }
    }

    // 3. LAYER B1: Urban Road Network Edges (LineStrings) - High-Performance Batch Rendering
    if (showRoads && hybridFeatures.length > 0) {
      ctx.beginPath();
      let hasRoadEdges = false;
      for (let f = 0; f < hybridFeatures.length; f++) {
        const feat = hybridFeatures[f];
        if (feat.geometry.type === 'LineString' && feat.properties?.layer === 'ROAD_EDGES') {
          const coords = feat.geometry.coordinates;
          if (!coords || coords.length < 2) continue;
          hasRoadEdges = true;
          for (let j = 0; j < coords.length; j++) {
            const { x, y } = lonLatToScreen(coords[j][0], coords[j][1], center.lon, center.lat, zoom, dimensions.width, dimensions.height);
            if (j === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          }
        }
      }
      if (hasRoadEdges) {
        ctx.strokeStyle = 'rgba(6, 182, 212, 0.40)';
        ctx.lineWidth = 2.5;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.stroke();
      }
    }

    // 4. LAYER B2: Urban Road Intersection Nodes (Degree >= 2 or Intersections)
    if (showRoads && hybridFeatures.length > 0) {
      hybridFeatures.forEach(feat => {
        if (feat.geometry.type === 'Point' && feat.properties?.layer === 'ROAD_INTERSECTIONS') {
          const [lon, lat] = feat.geometry.coordinates;
          const { x, y } = lonLatToScreen(lon, lat, center.lon, center.lat, zoom, dimensions.width, dimensions.height);

          const targetId = String(feat.properties?.node_id ?? feat.id ?? '');
          const isScanned = scannedTargetIds && (scannedTargetIds.has(targetId) || scannedTargetIds.has(String(feat.properties?.node_id)));

          if (isScanned) {
            // Scanned Road Intersection: Emerald halo & checkmark
            ctx.beginPath();
            ctx.arc(x, y, 11, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(16, 185, 129, 0.50)';
            ctx.fill();

            ctx.beginPath();
            ctx.arc(x, y, 6, 0, Math.PI * 2);
            ctx.fillStyle = '#10b981';
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2.0;
            ctx.stroke();

            // Small checkmark
            ctx.font = 'bold 9px sans-serif';
            ctx.fillStyle = '#ffffff';
            ctx.fillText('✓', x - 3.5, y + 3.5);
          } else {
            // Outer glowing halo
            ctx.beginPath();
            ctx.arc(x, y, 10, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(6, 182, 212, 0.35)';
            ctx.fill();

            // Circle: 6px radius, bright cyan center (#00f0ff), white border (#ffffff)
            ctx.beginPath();
            ctx.arc(x, y, 6, 0, Math.PI * 2);
            ctx.fillStyle = '#00f0ff';
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2.0;
            ctx.stroke();

            // Label Degree if zoom >= 14
            if (zoom >= 14 && feat.properties?.degree) {
              ctx.font = 'bold 9px monospace';
              ctx.fillStyle = '#67e8f9';
              ctx.fillText(`d=${feat.properties.degree}`, x + 8, y - 4);
            }
          }
        }
      });
    }

    // 5. LAYER C: Deterministic Per-Drone Flight Routes (Solid for Roads, Dashed for Free Cells)
    if (showRoutes && !isSolving && activeAlgorithmResult?.routes) {
      const activeRoutes = activeAlgorithmResult.routes;
      const droneIds = Object.keys(activeRoutes);

      droneIds.forEach((droneId, dIdx) => {
        const wps: Waypoint[] = activeRoutes[droneId] || [];
        if (!wps || wps.length === 0) return;

        // Unique deterministic color for this drone
        const droneColor = getDroneColor(dIdx, droneId);
        const isDimmed = highlightedDroneId !== null && highlightedDroneId !== droneId;

        ctx.save();
        ctx.globalAlpha = isDimmed ? 0.20 : 1.0;

        // Draw segmented line hops
        for (let i = 0; i < wps.length - 1; i++) {
          const wp1 = wps[i];
          const wp2 = wps[i + 1];

          const p1 = lonLatToScreen(wp1.lon, wp1.lat, center.lon, center.lat, zoom, dimensions.width, dimensions.height);
          const p2 = lonLatToScreen(wp2.lon, wp2.lat, center.lon, center.lat, zoom, dimensions.width, dimensions.height);

          ctx.beginPath();
          ctx.moveTo(p1.x, p1.y);
          ctx.lineTo(p2.x, p2.y);

          ctx.strokeStyle = droneColor.stroke;
          ctx.lineWidth = highlightedDroneId === droneId ? 4.5 : 3.0;

          // Solid line for normal survey path; light dotted stroke for battery return / relaunch transit
          const isTransit = wp1.waypoint_type === 'SORTIE_RETURN_BASE' || wp2.waypoint_type === 'SORTIE_RETURN_BASE' ||
                            wp1.waypoint_type === 'SORTIE_RELAUNCH_BASE' || wp2.waypoint_type === 'SORTIE_RELAUNCH_BASE';

          if (isTransit) {
            ctx.setLineDash([3, 4]);
            ctx.strokeStyle = '#f59e0b'; // Light amber transit line
            ctx.lineWidth = 1.8;
          } else {
            ctx.setLineDash([]);
            ctx.strokeStyle = droneColor.stroke;
            ctx.lineWidth = highlightedDroneId === droneId ? 4.2 : 2.8;
          }

          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Draw Waypoint Target Nodes
        wps.forEach((wp, wIdx) => {
          const { x, y } = lonLatToScreen(wp.lon, wp.lat, center.lon, center.lat, zoom, dimensions.width, dimensions.height);

          if (wp.waypoint_type === 'SORTIE_RETURN_BASE') {
            // Battery Swap Breakpoint Return Pad Marker
            ctx.beginPath();
            ctx.arc(x, y, 10, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(245, 158, 11, 0.4)';
            ctx.fill();
            ctx.strokeStyle = '#f59e0b';
            ctx.lineWidth = 2.0;
            ctx.stroke();

            ctx.font = 'bold 8px monospace';
            ctx.fillStyle = '#fbbf24';
            ctx.fillText(`⚡SWAP #${wp.sortie_index ?? 1}`, x + 12, y + 3);
          } else if (wp.waypoint_type === 'SORTIE_RELAUNCH_BASE') {
            // Battery Swap Relaunch Pad Marker
            ctx.beginPath();
            ctx.arc(x, y, 10, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(16, 185, 129, 0.4)';
            ctx.fill();
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 2.0;
            ctx.stroke();

            ctx.font = 'bold 8px monospace';
            ctx.fillStyle = '#34d399';
            ctx.fillText(`🚀SORTIE #${wp.sortie_index ?? 2}`, x + 12, y - 5);
          } else if (wp.node_type === 'ROAD_INTERSECTION') {
            // Road intersection waypoint: diamond node with halo
            ctx.beginPath();
            ctx.arc(x, y, 7, 0, Math.PI * 2);
            ctx.fillStyle = droneColor.fill;
            ctx.fill();

            ctx.beginPath();
            ctx.moveTo(x, y - 4);
            ctx.lineTo(x + 4, y);
            ctx.lineTo(x, y + 4);
            ctx.lineTo(x - 4, y);
            ctx.closePath();
            ctx.fillStyle = droneColor.hex;
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1.2;
            ctx.stroke();
          } else {
            // Open terrain cell waypoint: circular point
            ctx.beginPath();
            ctx.arc(x, y, 3.5, 0, Math.PI * 2);
            ctx.fillStyle = droneColor.hex;
            ctx.fill();
            ctx.strokeStyle = '#020617';
            ctx.lineWidth = 1;
            ctx.stroke();
          }

          // Sequence Index for first and last waypoints
          if (wIdx === 0 || wIdx === wps.length - 1) {
            ctx.font = 'bold 9px monospace';
            ctx.fillStyle = droneColor.hex;
            ctx.fillText(wIdx === 0 ? 'START' : 'END', x + 5, y + 10);
          }
        });

        ctx.restore();
      });
    }

    // 6. LAYER D: Active Freehand Drawing Stroke (In Progress)
    if (isDrawing && currentDrawPixels.length > 1) {
      ctx.beginPath();
      currentDrawPixels.forEach((pt, idx) => {
        if (idx === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
      });
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 3;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.stroke();

      const first = currentDrawPixels[0];
      const last = currentDrawPixels[currentDrawPixels.length - 1];
      ctx.beginPath();
      ctx.setLineDash([4, 4]);
      ctx.moveTo(last.x, last.y);
      ctx.lineTo(first.x, first.y);
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 7. LAYER E: Live Fleet Drones & Kinematic Heading
    liveFleet.forEach((drone, idx) => {
      const { x, y } = lonLatToScreen(drone.longitude, drone.latitude, center.lon, center.lat, zoom, dimensions.width, dimensions.height);
      const droneColor = getDroneColor(idx, drone.drone_id);

      if (drone.status === 'STANDBY') {
        // Drone on ground at base station waiting for staggered takeoff
        ctx.beginPath();
        ctx.arc(x, y, 12, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.7)';
        ctx.setLineDash([3, 3]);
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fillStyle = droneColor.hex;
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        ctx.font = 'bold 9px monospace';
        ctx.fillStyle = '#cbd5e1';
        ctx.fillText(`${drone.drone_id} [STANDBY]`, x + 9, y - 4);
      } else if (drone.status === 'BATTERY_SWAPPING') {
        // Drone actively swapping battery at Shared Home Base
        ctx.beginPath();
        ctx.arc(x, y, 16, 0, Math.PI * 2);
        ctx.strokeStyle = '#f59e0b';
        ctx.setLineDash([4, 2]);
        ctx.lineWidth = 2.0;
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.beginPath();
        ctx.arc(x, y, 8, 0, Math.PI * 2);
        ctx.fillStyle = '#f59e0b';
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        ctx.font = 'bold 10px monospace';
        ctx.fillStyle = '#fbbf24';
        ctx.shadowColor = '#000000';
        ctx.shadowBlur = 4;
        const swapSec = drone.swap_remaining_sec !== undefined ? `${drone.swap_remaining_sec.toFixed(0)}s` : 'SWAPPING';
        ctx.fillText(`${drone.drone_id} ⚡[SWAP ${swapSec}] ${drone.battery_pct.toFixed(0)}%`, x + 12, y - 4);
        ctx.shadowBlur = 0;
      } else {
        // Active in-flight drone
        // Radar scanning wave pulse
        ctx.beginPath();
        ctx.arc(x, y, 18, 0, Math.PI * 2);
        ctx.fillStyle = droneColor.fill;
        ctx.fill();

        // Glowing center core
        ctx.beginPath();
        ctx.arc(x, y, 8, 0, Math.PI * 2);
        ctx.fillStyle = droneColor.hex;
        ctx.shadowColor = droneColor.hex;
        ctx.shadowBlur = 8;
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.shadowBlur = 0;

        // Heading directional pointer arrow
        const headingRad = (drone.heading_deg || 0) * (Math.PI / 180);
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + Math.sin(headingRad) * 18, y - Math.cos(headingRad) * 18);
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2.5;
        ctx.stroke();

        // Drone Telemetry Label
        ctx.font = 'bold 10px monospace';
        ctx.fillStyle = '#ffffff';
        ctx.shadowColor = '#000000';
        ctx.shadowBlur = 4;
        const sTag = drone.current_sortie ? `S#${drone.current_sortie} ` : '';
        const statLabel = drone.status === 'RETURNING_FOR_SWAP' ? '⚡RTL-SWAP' :
                          drone.status === 'RELAUNCHING_SORTIE' ? '🚀RELAUNCH' :
                          `${drone.battery_pct.toFixed(0)}%`;
        ctx.fillText(
          `${drone.drone_id} | ${sTag}${statLabel} | ${drone.speed_mps.toFixed(0)}m/s`, 
          x + 12, 
          y - 6
        );
        ctx.shadowBlur = 0;
      }
    });

  }, [
    dimensions, 
    center, 
    zoom, 
    pinnedPolygon, 
    isDrawing, 
    currentDrawPixels, 
    activeAlgorithmResult, 
    liveFleet, 
    hybridFeatures, 
    showRoads, 
    showGrid, 
    showRoutes, 
    highlightedDroneId,
    isSolving,
    scannedTargetIds,
    localHomeBase
  ]);

  // Mouse Handlers: Panning vs Freehand Drawing
  const handleMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    if (e.button !== 0) return;

    const rect = containerRef.current?.getBoundingClientRect();
    if (!rect) return;
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    if (interactionMode === 'SET_BASE') {
      const geo = pixelToLonLat(x, y, center.lon, center.lat, zoom, dimensions.width, dimensions.height);
      setLocalHomeBase(geo);
      setInteractionMode('PAN');
      fetch(`${getApiBaseUrl()}/api/v1/fleet/home-base`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ lat: geo.lat, lon: geo.lon, alt_m: 0.0 })
      }).catch(err => console.warn('Failed to update base in backend:', err));
      return;
    }

    if (interactionMode === 'FREEHAND') {
      setIsDrawing(true);
      setCurrentDrawPixels([{ x, y }]);
      setPinnedPolygon(null);
      setRoiStats(null);
      setHybridFeatures([]);
      setHybridStats(null);
      if (onROIStartDrawing) onROIStartDrawing();
    } else {
      setIsPanning(true);
      setPanStart({ x: e.clientX, y: e.clientY });
    }
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLDivElement>) => {
    const rect = containerRef.current?.getBoundingClientRect();
    if (!rect) return;

    if (isDrawing && interactionMode === 'FREEHAND') {
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const last = currentDrawPixels[currentDrawPixels.length - 1];
      if (!last || Math.hypot(x - last.x, y - last.y) > 6) {
        setCurrentDrawPixels(prev => [...prev, { x, y }]);
      }
    } else if (isPanning && panStart) {
      const dx = e.clientX - panStart.x;
      const dy = e.clientY - panStart.y;
      setPanStart({ x: e.clientX, y: e.clientY });

      const scale = 256 * Math.pow(2, zoom);
      const dLon = (-dx / scale) * 360;
      const dLat = (dy / scale) * 180;

      setCenter(prev => ({
        lat: Math.max(-85, Math.min(85, prev.lat + dLat)),
        lon: ((prev.lon + dLon + 180) % 360) - 180
      }));
    }
  };

  const handleMouseUp = async () => {
    if (isDrawing && currentDrawPixels.length > 5) {
      setIsDrawing(false);

      const geographicPoints = currentDrawPixels.map(pt =>
        pixelToLonLat(pt.x, pt.y, center.lon, center.lat, zoom, dimensions.width, dimensions.height)
      );

      setPinnedPolygon(geographicPoints);
      setCurrentDrawPixels([]);
      // Switch back to PAN mode so operator can pan/inspect without accidental drawing reset
      setInteractionMode('PAN');

      const areaKm2 = calculatePolygonAreaKm2(geographicPoints);
      const lats = geographicPoints.map(c => c.lat);
      const lons = geographicPoints.map(c => c.lon);
      const minLat = Math.min(...lats);
      const maxLat = Math.max(...lats);
      const minLon = Math.min(...lons);
      const maxLon = Math.max(...lons);

      const dLatKm = ((maxLat - minLat) * 111.32);
      const dLonKm = ((maxLon - minLon) * 111.32 * Math.cos(((minLat + maxLat) / 2 * Math.PI) / 180));

      const stats = {
        areaKm2: parseFloat(areaKm2.toFixed(3)),
        widthKm: parseFloat(dLonKm.toFixed(2)),
        heightKm: parseFloat(dLatKm.toFixed(2)),
        vertices: geographicPoints.length
      };
      setRoiStats(stats);
      setIsGenerating(true);

      // Smoothly re-center viewport on newly drawn search domain
      setCenter({ lat: (minLat + maxLat) / 2, lon: (minLon + maxLon) / 2 });

      // 1. Submit ROI to Backend & Generate Dual-Layer Hybrid Space
      try {
        const hybridRes = await fetch(`${getApiBaseUrl()}/api/v1/grid/hybrid-generate`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            roi_type: 'POLYGON',
            bounds: { lat_min: minLat, lat_max: maxLat, lon_min: minLon, lon_max: maxLon },
            vertices: geographicPoints.map(c => ({ lat: c.lat, lon: c.lon })),
            cluster_threshold_m: 25.0,
            buffer_radius_ratio: 0.65
          })
        });

        if (hybridRes.ok) {
          const hybridData = await hybridRes.json();
          if (hybridData.geojson?.features) {
            setHybridFeatures(hybridData.geojson.features);
            setHybridStats(hybridData.statistics);
          }
          if (hybridData.home_base) {
            setLocalHomeBase(hybridData.home_base);
          }
          // Notify parent dashboard with the complete hybrid response including new session UUID!
          if (onROISubmitted) {
            onROISubmitted(hybridData);
          }
        } else {
          console.warn('Hybrid space generation failed with status:', hybridRes.status);
          const errBody = await hybridRes.json().catch(() => ({}));
          console.error('Server error detail:', errBody);
          const detail = typeof errBody.detail === 'string' ? errBody.detail : `HTTP ${hybridRes.status}`;
          if (onError) {
            onError(`فشل توليد الفضاء الهجين للمنطقة (${detail})`);
          }
        }
      } catch (err: any) {
        console.error('Failed to submit ROI / hybrid generate:', err);
        if (onError) {
          onError(`خطأ في الاتصال بالخادم: ${err?.message || 'تعذر الوصول إلى مسار الفضاء الهجين'}`);
        }
      } finally {
        setIsGenerating(false);
      }
    } else {
      setIsDrawing(false);
      setCurrentDrawPixels([]);
    }

    setIsPanning(false);
    setPanStart(null);
  };

  const handleClear = () => {
    clearAllMapOverlays();
    if (onROICleared) onROICleared();
  };

  const handleZoomIn = () => setZoom(z => Math.min(18, z + 1));
  const handleZoomOut = () => setZoom(z => Math.max(10, z - 1));

  // Extract unique active drone IDs from routes
  const activeFleetIds = useMemo(() => {
    if (activeAlgorithmResult?.routes) {
      return Object.keys(activeAlgorithmResult.routes);
    }
    return liveFleet.map(d => d.drone_id);
  }, [activeAlgorithmResult, liveFleet]);

  return (
    <div
      ref={containerRef}
      className={`w-full h-full relative select-none overflow-hidden bg-slate-950 ${
        interactionMode === 'FREEHAND' ? 'cursor-crosshair' : 'cursor-grab active:cursor-grabbing'
      }`}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
    >
      {/* Real Multi-Tile Slippy Map Layer */}
      <div className="absolute inset-0 pointer-events-none">
        {visibleTiles.map(tile => (
          <img
            key={tile.key}
            src={tile.url}
            alt=""
            loading="eager"
            className="absolute select-none pointer-events-none transition-opacity duration-150"
            style={{
              left: `${tile.left}px`,
              top: `${tile.top}px`,
              width: '256px',
              height: '256px',
              filter: basemap === 'SATELLITE' ? 'contrast(1.15) brightness(0.95)' : 'invert(0.9) hue-rotate(180deg)'
            }}
          />
        ))}
      </div>

      {/* Vector Graphics Canvas */}
      <canvas
        ref={canvasRef}
        width={dimensions.width}
        height={dimensions.height}
        className="absolute inset-0 w-full h-full pointer-events-none z-10"
      />

      {/* Solving & Generating Hybrid Space Indicator */}
      {(isSolving || isGenerating) && (
        <div className="absolute top-16 left-1/2 -translate-x-1/2 z-30 px-5 py-2.5 bg-slate-900/95 border border-cyan-500/80 rounded-2xl shadow-2xl backdrop-blur-md flex items-center space-x-3 text-cyan-300">
          <div className="w-4 h-4 border-2 border-cyan-400 border-t-transparent rounded-full animate-spin" />
          <span className="text-xs font-bold tracking-wide">
            {isGenerating ? 'جاري استخراج طوبولوجيا الطرق وتوليد فضاء البحث الهجين...' : 'جاري تخطيط المسارات الهجينة لأسطول الدرونات...'}
          </span>
        </div>
      )}

      {/* Top Left Interaction Toolbar & Context Ribbon (Auto-Layout 2 Rows with Safe Spacing) */}
      <div className="absolute top-4 left-4 z-20 flex flex-col items-start gap-2 max-w-[calc(100%-120px)] pointer-events-none">
        
        {/* ROW 1: Interaction & Navigation Tools */}
        <div className="flex items-center gap-2 flex-wrap pointer-events-auto">
          <div className="flex items-center space-x-1 bg-slate-950/90 backdrop-blur p-1 rounded-xl border border-hud-border shadow-2xl text-xs">
            <button
              onClick={() => setInteractionMode('FREEHAND')}
              className={`whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-semibold leading-none shrink-0 transition-all ${
                interactionMode === 'FREEHAND'
                  ? 'bg-gradient-to-r from-emerald-600 to-teal-600 text-slate-950 font-bold shadow-lg shadow-emerald-950/50'
                  : 'text-slate-300 hover:text-white hover:bg-slate-800'
              }`}
              title="اضغط واسحب بالماوس لتحديد أي منطقة على الخريطة"
            >
              <Pencil className="w-3.5 h-3.5" />
              <span>رسم حر (Freehand ROI)</span>
            </button>

            <button
              onClick={() => setInteractionMode('PAN')}
              className={`whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-semibold leading-none shrink-0 transition-all ${
                interactionMode === 'PAN'
                  ? 'bg-cyan-500 text-slate-950 font-bold shadow-lg shadow-cyan-950/50'
                  : 'text-slate-300 hover:text-white hover:bg-slate-800'
              }`}
              title="تحريك وتصفح الخريطة مع بقاء المنطقة المحددة ثابتة في مكانها"
            >
              <Hand className="w-3.5 h-3.5" />
              <span>تحريك الخريطة (Pan)</span>
            </button>

            <button
              onClick={() => setInteractionMode(prev => prev === 'SET_BASE' ? 'PAN' : 'SET_BASE')}
              className={`whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-semibold leading-none shrink-0 transition-all ${
                interactionMode === 'SET_BASE'
                  ? 'bg-amber-500 text-slate-950 font-bold shadow-lg shadow-amber-950/50 animate-pulse'
                  : 'text-slate-300 hover:text-white hover:bg-slate-800'
              }`}
              title="انقر لتحديد موقع القاعدة المشتركة بالماوس"
            >
              <MapPin className="w-3.5 h-3.5" />
              <span>تحديد نقطة الانطلاق (Set Base)</span>
            </button>

            {pinnedPolygon && (
              <>
                <button
                  onClick={handleFitROI}
                  className="whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-semibold leading-none shrink-0 text-slate-300 hover:text-emerald-400 hover:bg-slate-800 transition-all border-l border-slate-800 ml-1"
                  title="تمركز الخريطة على مضلع البحث المعتمد"
                >
                  <Maximize2 className="w-3.5 h-3.5 text-emerald-400" />
                  <span>تمركز على المنطقة (Fit ROI)</span>
                </button>

                <button
                  onClick={handleClear}
                  className="whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-2.5 py-1.5 rounded-lg text-xs font-semibold leading-none shrink-0 text-slate-400 hover:text-red-400 hover:bg-slate-800 transition-all border-l border-slate-800 ml-1"
                  title="مسح المنطقة المحددة وإعادة ضبط الجلسة"
                >
                  <RotateCcw className="w-3.5 h-3.5" />
                  <span>مسح</span>
                </button>
              </>
            )}
          </div>
        </div>

        {/* ROW 2: Layer Toggles & Engine Badge */}
        <div className="flex items-center gap-2 flex-wrap pointer-events-auto">
          {/* Basemap Switcher */}
          <div className="flex items-center space-x-1 bg-slate-950/90 backdrop-blur p-1 rounded-xl border border-hud-border shadow-2xl text-xs">
            <button
              onClick={() => setBasemap('SATELLITE')}
              className={`whitespace-nowrap inline-flex items-center justify-center px-2.5 py-1.5 rounded-lg text-xs font-semibold leading-none shrink-0 ${
                basemap === 'SATELLITE' ? 'bg-slate-800 text-emerald-400 border border-slate-700' : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              قمر صناعي (Satellite)
            </button>
            <button
              onClick={() => setBasemap('STREET')}
              className={`whitespace-nowrap inline-flex items-center justify-center px-2.5 py-1.5 rounded-lg text-xs font-semibold leading-none shrink-0 ${
                basemap === 'STREET' ? 'bg-slate-800 text-emerald-400 border border-slate-700' : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              تضاريس / شوارع
            </button>
          </div>

          {/* Dual-Layer Visibility Toggles */}
          <div className="flex items-center space-x-1 bg-slate-950/90 backdrop-blur p-1 rounded-xl border border-hud-border shadow-2xl text-xs">
            <button
              onClick={() => setShowRoads(prev => !prev)}
              className={`whitespace-nowrap inline-flex items-center justify-center gap-1 px-2.5 py-1.5 rounded-lg text-[11px] font-semibold leading-none shrink-0 transition-all ${
                showRoads ? 'bg-cyan-950/80 text-cyan-300 border border-cyan-500/40' : 'text-slate-500 hover:text-slate-300'
              }`}
              title="إظهار / إخفاء بيان شبكة الطرق والتقاطعات الحضرية"
            >
              <Crosshair className="w-3 h-3" />
              <span>عقد الطرق (OSM)</span>
            </button>
            <button
              onClick={() => setShowGrid(prev => !prev)}
              className={`whitespace-nowrap inline-flex items-center justify-center gap-1 px-2.5 py-1.5 rounded-lg text-[11px] font-semibold leading-none shrink-0 transition-all ${
                showGrid ? 'bg-emerald-950/80 text-emerald-300 border border-emerald-500/40' : 'text-slate-500 hover:text-slate-300'
              }`}
              title="إظهار / إخفاء خلايا الفضاء المفتوح"
            >
              <Layers className="w-3 h-3" />
              <span>خلايا البحث الحر</span>
            </button>
            <button
              onClick={() => setShowRoutes(prev => !prev)}
              className={`whitespace-nowrap inline-flex items-center justify-center gap-1 px-2.5 py-1.5 rounded-lg text-[11px] font-semibold leading-none shrink-0 transition-all ${
                showRoutes ? 'bg-purple-950/80 text-purple-300 border border-purple-500/40' : 'text-slate-500 hover:text-slate-300'
              }`}
              title="إظهار / إخفاء مسارات الدرونات الملونة"
            >
              <Route className="w-3 h-3" />
              <span>المسارات</span>
            </button>
          </div>

          {/* Extracted Road Nodes Status Pill */}
          {hybridStats && (
            <div className="whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-3 py-1.5 rounded-xl bg-slate-950/90 border border-cyan-500/50 text-cyan-300 font-mono text-[11px] shadow-2xl backdrop-blur shrink-0">
              <Crosshair className="w-3.5 h-3.5 text-cyan-400" />
              <span className="font-bold">{hybridStats.road_intersection_count} عقد طرق</span>
            </div>
          )}

          {/* Active Engine Badge */}
          <div 
            className="whitespace-nowrap inline-flex items-center justify-center gap-1.5 px-3 py-1.5 rounded-xl bg-slate-950/90 border shadow-2xl backdrop-blur text-xs font-mono shrink-0"
            style={{ borderColor: algorithmTheme.primary }}
          >
            {algorithmTheme.name === 'Hybrid GA-PSO' && <Zap className="w-3.5 h-3.5 text-emerald-400" />}
            {algorithmTheme.name === 'Discrete PSO' && <Cpu className="w-3.5 h-3.5 text-sky-400" />}
            {algorithmTheme.name === 'Modified GA' && <Dna className="w-3.5 h-3.5 text-purple-400" />}
            <span className="text-slate-400">النمط المعروض:</span>
            <span className="font-bold" style={{ color: algorithmTheme.primary }}>
              {algorithmTheme.name}
            </span>
          </div>
        </div>

        {/* Dynamic Context Hint Bar */}
        <div className="pointer-events-auto">
          <div className="bg-black/85 backdrop-blur px-3 py-1.5 rounded-xl border border-slate-800 text-[11px] text-slate-200 shadow-xl flex items-center space-x-2">
            {interactionMode === 'FREEHAND' ? (
              <>
                <span className="w-2 h-2 rounded-full bg-emerald-400 animate-ping" />
                <span>✍️ <strong>وضع الرسم الحر:</strong> ارسم المضلع لاستخراج تقاطعات الطرق وشبكة الخلايا المفتوحة فوراً!</span>
              </>
            ) : interactionMode === 'SET_BASE' ? (
              <>
                <span className="w-2 h-2 rounded-full bg-amber-400 animate-ping" />
                <span>📍 <strong>وضع تحديد القاعدة:</strong> انقر في أي مكان على الخريطة لنقل موقع القاعدة المشتركة (Shared Home Base).</span>
              </>
            ) : (
              <>
                <span className="w-2 h-2 rounded-full bg-cyan-400" />
                <span>✋ <strong>وضع تحريك الخريطة:</strong> اسحب الخريطة لتصفح التضاريس مع بقاء المنطقة المحددة ثابتة تماماً.</span>
              </>
            )}
          </div>
        </div>
      </div>

      {/* Top Right Zoom Controls */}
      <div className="absolute top-4 right-4 z-20 flex flex-col space-y-1 bg-slate-950/90 backdrop-blur p-1 rounded-xl border border-hud-border shadow-2xl">
        <button onClick={handleZoomIn} className="p-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800" title="تكبير">
          <ZoomIn className="w-4 h-4" />
        </button>
        <button onClick={handleZoomOut} className="p-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800" title="تصغير">
          <ZoomOut className="w-4 h-4" />
        </button>
      </div>

      {/* Interactive Fleet Color Legend (Allows isolate/dimming single drone routes) */}
      {activeFleetIds.length > 0 && showRoutes && (
        <div className="absolute top-20 right-4 z-20 bg-slate-950/90 backdrop-blur p-2 rounded-xl border border-slate-800/80 shadow-2xl flex flex-col space-y-1.5">
          <div className="text-[10px] text-slate-400 font-bold uppercase tracking-wider flex items-center justify-between border-b border-slate-800/60 pb-1">
            <span>مفتاح مسارات الأسطول:</span>
            {highlightedDroneId && (
              <button
                onClick={() => setHighlightedDroneId(null)}
                className="text-[9px] text-cyan-400 hover:underline"
              >
                إظهار الكل
              </button>
            )}
          </div>
          <div className="flex flex-col space-y-1">
            {activeFleetIds.map((droneId, idx) => {
              const dColor = getDroneColor(idx, droneId);
              const isSelected = highlightedDroneId === droneId;
              return (
                <button
                  key={droneId}
                  onClick={() => setHighlightedDroneId(isSelected ? null : droneId)}
                  className={`flex items-center space-x-2 px-2 py-1 rounded text-left transition-all text-xs font-mono ${
                    isSelected
                      ? 'bg-slate-800 border border-slate-600 shadow-md font-bold'
                      : 'hover:bg-slate-900 text-slate-300'
                  }`}
                  title="انقر لتسليط الضوء على مسار هذا الدرون بمفرده"
                >
                  <span className="w-2.5 h-2.5 rounded-full" style={{ backgroundColor: dColor.hex }} />
                  <span style={{ color: dColor.hex }}>{droneId}</span>
                  <span className="text-[9px] text-slate-500">
                    {activeAlgorithmResult?.routes?.[droneId]?.length || 0} pts
                  </span>
                </button>
              );
            })}
          </div>
        </div>
      )}

      {/* Bottom-Left Pinned ROI Measurement & Dual-Layer Stats HUD */}
      {(roiStats || hybridStats) && (
        <div className="absolute bottom-4 left-4 z-20 bg-slate-950/95 backdrop-blur px-4 py-2.5 rounded-2xl border border-emerald-500/50 shadow-2xl flex items-center space-x-4 text-xs font-mono text-slate-200">
          <div className="flex items-center space-x-2 text-emerald-400 font-bold border-r border-slate-800 pr-3">
            <CheckCircle2 className="w-4 h-4 text-emerald-400" />
            <span>فضاء هجين موحد</span>
          </div>
          {roiStats && (
            <>
              <div>
                <span className="text-slate-500 text-[10px] block">المساحة</span>
                <span className="font-bold text-emerald-300">{roiStats.areaKm2} كم²</span>
              </div>
              <div>
                <span className="text-slate-500 text-[10px] block">الأبعاد</span>
                <span className="text-cyan-300">{roiStats.widthKm} &times; {roiStats.heightKm} كم</span>
              </div>
            </>
          )}
          {hybridStats && (
            <>
              <div className="border-l border-slate-800 pl-3">
                <span className="text-slate-500 text-[10px] block">عقد الطرق</span>
                <span className="font-bold text-cyan-400">{hybridStats.road_intersection_count} تقاطع</span>
              </div>
              <div>
                <span className="text-slate-500 text-[10px] block">الخلايا المفتوحة</span>
                <span className="font-bold text-emerald-400">{hybridStats.open_terrain_cells_count} خلية</span>
              </div>
              <div>
                <span className="text-slate-500 text-[10px] block">منع التداخل</span>
                <span className="font-bold text-emerald-400">Zero-Overlap ✓</span>
              </div>
            </>
          )}
        </div>
      )}

      {/* Bottom-Right Live Fleet Telemetry Simulation HUD */}
      {liveFleet.length > 0 && (
        <div className="absolute bottom-4 right-4 z-20 bg-slate-950/95 backdrop-blur p-2.5 rounded-2xl border border-slate-800 shadow-2xl flex items-center space-x-3 text-xs font-mono">
          <div className="flex items-center space-x-1.5 text-slate-400 border-r border-slate-800 pr-2">
            <Activity className="w-3.5 h-3.5 text-emerald-400 animate-pulse" />
            <span className="text-[10px] uppercase font-bold">حالة الأسطول:</span>
          </div>
          <div className="flex items-center space-x-2">
            {liveFleet.map((drone, idx) => {
              const dColor = getDroneColor(idx, drone.drone_id);
              return (
                <div key={drone.drone_id} className="px-2 py-1 rounded bg-slate-900 border border-slate-800 text-[11px] flex items-center space-x-1.5">
                  <span className="w-2 h-2 rounded-full" style={{ backgroundColor: dColor.hex }} />
                  <span className="font-bold text-slate-200">{drone.drone_id}</span>
                  <span className="text-slate-400 text-[10px]">{drone.speed_mps.toFixed(1)}m/s</span>
                  <span className={`text-[10px] font-bold ${drone.battery_pct > 25 ? 'text-emerald-400' : 'text-red-400'}`}>
                    {drone.battery_pct.toFixed(0)}%
                  </span>
                </div>
              );
            })}
          </div>
        </div>
      )}
    </div>
  );
});

export default MultiLayerMissionMap;
