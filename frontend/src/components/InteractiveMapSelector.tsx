import React, { useState, useRef, useCallback, useEffect } from 'react';
import { 
  GoogleMap, 
  useJsApiLoader, 
  DrawingManager, 
  Rectangle, 
  Polygon 
} from '@react-google-maps/api';
import { 
  Square, 
  Hexagon, 
  Trash2, 
  CheckCircle2, 
  Layers, 
  Navigation, 
  Globe2, 
  Crosshair,
  AlertCircle
} from 'lucide-react';
import { GeoPoint, BoundingBox, SearchSpaceROI, SearchSpaceROIResponse } from '../types/geo';
import { getApiBaseUrl } from '../utils/apiConfig';

const LIBRARIES: ("drawing" | "geometry")[] = ["drawing", "geometry"];

const DEFAULT_CENTER = {
  lat: 33.3152, // Default: Middle East / Baghdad test coordinates (or any global terrain)
  lng: 44.3661
};

const MAP_CONTAINER_STYLE = {
  width: '100%',
  height: '100%'
};

interface InteractiveMapSelectorProps {
  onConfirmROI?: (roiData: SearchSpaceROIResponse) => void;
}

export const InteractiveMapSelector: React.FC<InteractiveMapSelectorProps> = ({ onConfirmROI }) => {
  const apiKey = (import.meta as any).env?.VITE_GOOGLE_MAPS_API_KEY || '';

  const { isLoaded, loadError } = useJsApiLoader({
    googleMapsApiKey: apiKey,
    libraries: LIBRARIES
  });

  // Map state
  const [map, setMap] = useState<google.maps.Map | null>(null);
  const [mapTypeId, setMapTypeId] = useState<'hybrid' | 'terrain' | 'satellite'>('hybrid');
  
  // Drawing tools state
  const [activeDrawingMode, setActiveDrawingMode] = useState<google.maps.drawing.OverlayType | null>(null);
  const [drawnType, setDrawnType] = useState<'RECTANGLE' | 'POLYGON' | null>(null);
  
  // Stored shapes and geometries
  const [rectangleBounds, setRectangleBounds] = useState<google.maps.LatLngBoundsLiteral | null>(null);
  const [polygonPath, setPolygonPath] = useState<{ lat: number; lng: number }[]>([]);
  
  // Calculated Spatial Metrics
  const [boundingBox, setBoundingBox] = useState<BoundingBox | null>(null);
  const [calculatedAreaKm2, setCalculatedAreaKm2] = useState<number>(0);
  const [isSubmitting, setIsSubmitting] = useState<boolean>(false);
  const [serverMessage, setServerMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  const rectangleRef = useRef<google.maps.Rectangle | null>(null);
  const polygonRef = useRef<google.maps.Polygon | null>(null);

  // Helper: Calculate polygon area
  const computePolygonArea = (coords: { lat: number; lng: number }[]): number => {
    if (!window.google?.maps?.geometry?.spherical || coords.length < 3) return 0;
    const path = coords.map(c => new window.google.maps.LatLng(c.lat, c.lng));
    const areaSqMeters = window.google.maps.geometry.spherical.computeArea(path);
    return areaSqMeters / 1_000_000; // Convert to km²
  };

  // Helper: Calculate rectangle area
  const computeRectangleArea = (bounds: google.maps.LatLngBoundsLiteral): number => {
    if (!window.google?.maps?.geometry?.spherical) return 0;
    const sw = new window.google.maps.LatLng(bounds.south, bounds.west);
    const ne = new window.google.maps.LatLng(bounds.north, bounds.east);
    const nw = new window.google.maps.LatLng(bounds.north, bounds.west);
    const se = new window.google.maps.LatLng(bounds.south, bounds.east);
    const areaSqMeters = window.google.maps.geometry.spherical.computeArea([sw, nw, ne, se]);
    return areaSqMeters / 1_000_000;
  };

  // Handle Rectangle Completion
  const onRectangleComplete = (rect: google.maps.Rectangle) => {
    const bounds = rect.getBounds();
    if (!bounds) return;

    const ne = bounds.getNorthEast();
    const sw = bounds.getSouthWest();

    const boundsLiteral: google.maps.LatLngBoundsLiteral = {
      north: ne.lat(),
      south: sw.lat(),
      east: ne.lng(),
      west: sw.lng()
    };

    rect.setMap(null); // Remove default drawn rectangle, replaced by managed React <Rectangle>

    setDrawnType('RECTANGLE');
    setPolygonPath([]);
    setRectangleBounds(boundsLiteral);

    const bBox: BoundingBox = {
      lat_min: boundsLiteral.south,
      lat_max: boundsLiteral.north,
      lon_min: boundsLiteral.west,
      lon_max: boundsLiteral.east
    };
    setBoundingBox(bBox);
    setCalculatedAreaKm2(computeRectangleArea(boundsLiteral));
    setActiveDrawingMode(null); // Finish drawing mode
  };

  // Handle Polygon Completion
  const onPolygonComplete = (poly: google.maps.Polygon) => {
    const path = poly.getPath();
    const coords: { lat: number; lng: number }[] = [];

    let latMin = 90, latMax = -90, lonMin = 180, lonMax = -180;

    for (let i = 0; i < path.getLength(); i++) {
      const pt = path.getAt(i);
      const lat = pt.lat();
      const lng = pt.lng();
      coords.push({ lat, lng });

      if (lat < latMin) latMin = lat;
      if (lat > latMax) latMax = lat;
      if (lng < lonMin) lonMin = lng;
      if (lng > lonMax) lonMax = lng;
    }

    poly.setMap(null); // Remove default, replaced by managed React <Polygon>

    setDrawnType('POLYGON');
    setRectangleBounds(null);
    setPolygonPath(coords);

    setBoundingBox({
      lat_min: latMin,
      lat_max: latMax,
      lon_min: lonMin,
      lon_max: lonMax
    });
    setCalculatedAreaKm2(computePolygonArea(coords));
    setActiveDrawingMode(null);
  };

  // Listen to edits on Rectangle
  const onRectangleBoundsChanged = () => {
    if (!rectangleRef.current) return;
    const bounds = rectangleRef.current.getBounds();
    if (!bounds) return;
    const ne = bounds.getNorthEast();
    const sw = bounds.getSouthWest();

    const bLiteral: google.maps.LatLngBoundsLiteral = {
      north: ne.lat(),
      south: sw.lat(),
      east: ne.lng(),
      west: sw.lng()
    };
    setRectangleBounds(bLiteral);
    setBoundingBox({
      lat_min: bLiteral.south,
      lat_max: bLiteral.north,
      lon_min: bLiteral.west,
      lon_max: bLiteral.east
    });
    setCalculatedAreaKm2(computeRectangleArea(bLiteral));
  };

  // Listen to edits on Polygon path
  const onPolygonEdit = () => {
    if (!polygonRef.current) return;
    const path = polygonRef.current.getPath();
    const coords: { lat: number; lng: number }[] = [];
    let latMin = 90, latMax = -90, lonMin = 180, lonMax = -180;

    for (let i = 0; i < path.getLength(); i++) {
      const pt = path.getAt(i);
      const lat = pt.lat();
      const lng = pt.lng();
      coords.push({ lat, lng });
      if (lat < latMin) latMin = lat;
      if (lat > latMax) latMax = lat;
      if (lng < lonMin) lonMin = lng;
      if (lng > lonMax) lonMax = lng;
    }

    setPolygonPath(coords);
    setBoundingBox({
      lat_min: latMin,
      lat_max: latMax,
      lon_min: lonMin,
      lon_max: lonMax
    });
    setCalculatedAreaKm2(computePolygonArea(coords));
  };

  // Clear current selection
  const handleClearROI = () => {
    setDrawnType(null);
    setRectangleBounds(null);
    setPolygonPath([]);
    setBoundingBox(null);
    setCalculatedAreaKm2(0);
    setServerMessage(null);
    setActiveDrawingMode(null);
  };

  // Submit ROI to Backend
  const handleConfirmROI = async () => {
    if (!boundingBox || !drawnType || calculatedAreaKm2 <= 0) return;

    setIsSubmitting(true);
    setServerMessage(null);

    const vertices: GeoPoint[] = drawnType === 'POLYGON' 
      ? polygonPath.map(p => ({ lat: p.lat, lon: p.lng }))
      : [
          { lat: boundingBox.lat_min, lon: boundingBox.lon_min },
          { lat: boundingBox.lat_max, lon: boundingBox.lon_min },
          { lat: boundingBox.lat_max, lon: boundingBox.lon_max },
          { lat: boundingBox.lat_min, lon: boundingBox.lon_max }
        ];

    const payload: SearchSpaceROI = {
      roi_type: drawnType,
      bounds: boundingBox,
      vertices: vertices,
      area_km2: calculatedAreaKm2
    };

    try {
      const response = await fetch(`${getApiBaseUrl()}/api/v1/map/roi`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to register ROI');
      }

      const data: SearchSpaceROIResponse = await response.json();
      setServerMessage({
        type: 'success',
        text: `Search Space Confirmed: ${data.estimated_50m_cells} cells ready for Discretization.`
      });
      if (onConfirmROI) {
        onConfirmROI(data);
      }
    } catch (err: any) {
      setServerMessage({
        type: 'error',
        text: err.message || 'Error connecting to FastAPI backend'
      });
    } finally {
      setIsSubmitting(false);
    }
  };

  // Fallback view when Google API Key is not set or maps cannot load
  if (!apiKey || loadError) {
    return (
      <div className="w-full h-full flex flex-col items-center justify-center bg-slate-950 p-6 text-center text-slate-300 relative select-none">
        <div className="p-4 rounded-xl bg-amber-500/10 border border-amber-500/30 text-amber-400 mb-4 max-w-lg flex items-start space-x-3 text-left">
          <AlertCircle className="w-6 h-6 flex-shrink-0 mt-0.5" />
          <div className="text-xs">
            <p className="font-semibold text-amber-300">Google Maps API Key Required</p>
            <p className="mt-1 text-slate-300">
              Provide your API key in <code className="bg-slate-900 px-1 py-0.5 rounded text-amber-200">frontend/.env</code> as <code className="text-amber-200">VITE_GOOGLE_MAPS_API_KEY</code>.
            </p>
          </div>
        </div>

        {/* Interactive Mock Canvas for Sandbox / Demo Testing */}
        <div className="w-full max-w-xl h-80 rounded-xl border border-hud-border bg-hud-panel/40 relative flex flex-col items-center justify-center p-6">
          <Globe2 className="w-12 h-12 text-emerald-400 mb-3 animate-pulse" />
          <h3 className="text-sm font-bold text-slate-200 uppercase tracking-wider">
            Interactive Search Space Sandbox Mode
          </h3>
          <p className="text-xs text-slate-400 max-w-md mt-1">
            Simulate ROI definition for testing Geodesic Discretization & Multi-Drone Optimizers.
          </p>

          <button
            onClick={() => {
              const mockBounds: BoundingBox = {
                lat_min: 33.3000,
                lat_max: 33.3250,
                lon_min: 44.3500,
                lon_max: 44.3800
              };
              setDrawnType('RECTANGLE');
              setBoundingBox(mockBounds);
              setCalculatedAreaKm2(7.68);
            }}
            className="mt-4 px-4 py-2 bg-emerald-600/30 hover:bg-emerald-600/50 border border-emerald-500 text-emerald-300 text-xs rounded-lg transition-all"
          >
            Load Realistic Demo ROI (7.68 km²)
          </button>

          {boundingBox && (
            <div className="mt-4 text-xs font-mono bg-slate-900/90 p-3 rounded border border-slate-700 w-full text-left">
              <div className="text-emerald-400 font-bold mb-1">Simulated ROI Active:</div>
              <div>Lat: [{boundingBox.lat_min}, {boundingBox.lat_max}]</div>
              <div>Lon: [{boundingBox.lon_min}, {boundingBox.lon_max}]</div>
              <div>Area: {calculatedAreaKm2} km² (~3072 cells @ 50m)</div>

              <button
                onClick={handleConfirmROI}
                disabled={isSubmitting}
                className="mt-3 w-full py-1.5 bg-emerald-600 text-slate-950 font-bold rounded hover:bg-emerald-500 transition-colors"
              >
                {isSubmitting ? 'Registering with FastAPI...' : 'Send to Backend (/api/v1/map/roi)'}
              </button>
            </div>
          )}

          {serverMessage && (
            <div className={`mt-2 text-[11px] font-mono ${serverMessage.type === 'success' ? 'text-emerald-400' : 'text-red-400'}`}>
              {serverMessage.text}
            </div>
          )}
        </div>
      </div>
    );
  }

  if (!isLoaded) {
    return (
      <div className="w-full h-full flex items-center justify-center bg-slate-950 text-slate-400 font-mono text-xs">
        <Navigation className="w-5 h-5 animate-spin mr-2 text-emerald-400" />
        Loading Google Maps GIS Layer & Drawing Libraries...
      </div>
    );
  }

  return (
    <div className="w-full h-full relative select-none">
      {/* Top Map HUD: View Mode Switcher and Drawing Tools */}
      <div className="absolute top-4 left-4 z-10 flex items-center space-x-2 bg-slate-950/80 backdrop-blur p-1.5 rounded-lg border border-hud-border shadow-xl">
        {/* Map Type Switcher */}
        <div className="flex items-center space-x-1 border-r border-slate-800 pr-2 mr-1">
          <button
            onClick={() => setMapTypeId('hybrid')}
            className={`px-2.5 py-1 text-xs rounded font-medium transition-all ${
              mapTypeId === 'hybrid' ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40' : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Hybrid
          </button>
          <button
            onClick={() => setMapTypeId('terrain')}
            className={`px-2.5 py-1 text-xs rounded font-medium transition-all ${
              mapTypeId === 'terrain' ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40' : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Terrain
          </button>
          <button
            onClick={() => setMapTypeId('satellite')}
            className={`px-2.5 py-1 text-xs rounded font-medium transition-all ${
              mapTypeId === 'satellite' ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40' : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Satellite
          </button>
        </div>

        {/* Drawing Tools */}
        <div className="flex items-center space-x-1">
          <button
            onClick={() => {
              if (window.google?.maps?.drawing) {
                setActiveDrawingMode(window.google.maps.drawing.OverlayType.RECTANGLE);
              }
            }}
            className={`flex items-center space-x-1 px-3 py-1 text-xs rounded transition-all ${
              activeDrawingMode === window.google?.maps?.drawing?.OverlayType.RECTANGLE
                ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50 shadow-sm'
                : 'text-slate-300 hover:bg-slate-900'
            }`}
            title="Draw Bounding Box Rectangle"
          >
            <Square className="w-3.5 h-3.5" />
            <span>Rectangle ROI</span>
          </button>

          <button
            onClick={() => {
              if (window.google?.maps?.drawing) {
                setActiveDrawingMode(window.google.maps.drawing.OverlayType.POLYGON);
              }
            }}
            className={`flex items-center space-x-1 px-3 py-1 text-xs rounded transition-all ${
              activeDrawingMode === window.google?.maps?.drawing?.OverlayType.POLYGON
                ? 'bg-purple-500/20 text-purple-400 border border-purple-500/50 shadow-sm'
                : 'text-slate-300 hover:bg-slate-900'
            }`}
            title="Draw Custom Polygon"
          >
            <Hexagon className="w-3.5 h-3.5" />
            <span>Polygon ROI</span>
          </button>

          {drawnType && (
            <button
              onClick={handleClearROI}
              className="flex items-center space-x-1 px-2.5 py-1 text-xs rounded text-red-400 hover:bg-red-500/10 border border-transparent hover:border-red-500/30 transition-all ml-1"
              title="Clear Selection"
            >
              <Trash2 className="w-3.5 h-3.5" />
              <span>Clear</span>
            </button>
          )}
        </div>
      </div>

      {/* Real Google Map Component */}
      <GoogleMap
        mapContainerStyle={MAP_CONTAINER_STYLE}
        center={DEFAULT_CENTER}
        zoom={13}
        mapTypeId={mapTypeId}
        onLoad={mapInstance => setMap(mapInstance)}
        options={{
          disableDefaultUI: false,
          zoomControl: true,
          streetViewControl: false,
          mapTypeControl: false,
          fullscreenControl: false,
          styles: [
            { elementType: 'geometry', stylers: [{ color: '#1e293b' }] },
            { elementType: 'labels.text.stroke', stylers: [{ color: '#0f172a' }] },
            { elementType: 'labels.text.fill', stylers: [{ color: '#94a3b8' }] }
          ]
        }}
      >
        {/* Drawing Manager Hook */}
        <DrawingManager
          drawingMode={activeDrawingMode}
          onRectangleComplete={onRectangleComplete}
          onPolygonComplete={onPolygonComplete}
          options={{
            drawingControl: false,
            rectangleOptions: {
              fillColor: '#10b981',
              fillOpacity: 0.25,
              strokeWeight: 2,
              strokeColor: '#10b981',
              clickable: true,
              editable: true,
              zIndex: 1
            },
            polygonOptions: {
              fillColor: '#38bdf8',
              fillOpacity: 0.25,
              strokeWeight: 2,
              strokeColor: '#38bdf8',
              clickable: true,
              editable: true,
              zIndex: 1
            }
          }}
        />

        {/* Managed Rectangle if drawn */}
        {drawnType === 'RECTANGLE' && rectangleBounds && (
          <Rectangle
            ref={rectangleRef as any}
            bounds={rectangleBounds}
            editable={true}
            draggable={true}
            onBoundsChanged={onRectangleBoundsChanged}
            options={{
              fillColor: '#10b981',
              fillOpacity: 0.25,
              strokeColor: '#10b981',
              strokeWeight: 2
            }}
          />
        )}

        {/* Managed Polygon if drawn */}
        {drawnType === 'POLYGON' && polygonPath.length > 0 && (
          <Polygon
            ref={polygonRef as any}
            paths={polygonPath}
            editable={true}
            draggable={true}
            onMouseUp={onPolygonEdit}
            options={{
              fillColor: '#38bdf8',
              fillOpacity: 0.25,
              strokeColor: '#38bdf8',
              strokeWeight: 2
            }}
          />
        )}
      </GoogleMap>

      {/* Floating Spatial Information Card & Confirmation */}
      {boundingBox && (
        <div className="absolute bottom-6 left-6 z-10 bg-slate-950/90 backdrop-blur border border-hud-border p-4 rounded-xl shadow-2xl w-80 text-xs font-mono">
          <div className="flex items-center justify-between pb-2 mb-2 border-b border-slate-800">
            <span className="text-slate-400 flex items-center space-x-1.5">
              <Crosshair className="w-3.5 h-3.5 text-emerald-400" />
              <span className="font-bold text-slate-200">Active Search Space</span>
            </span>
            <span className="bg-emerald-500/20 text-emerald-400 px-2 py-0.5 rounded text-[10px] uppercase font-bold border border-emerald-500/40">
              {drawnType}
            </span>
          </div>

          <div className="space-y-1 text-slate-300 text-[11px]">
            <div className="flex justify-between">
              <span className="text-slate-500">Lat Span:</span>
              <span>{boundingBox.lat_min.toFixed(5)}° to {boundingBox.lat_max.toFixed(5)}°</span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-500">Lon Span:</span>
              <span>{boundingBox.lon_min.toFixed(5)}° to {boundingBox.lon_max.toFixed(5)}°</span>
            </div>
            <div className="flex justify-between font-bold text-emerald-400 pt-1 border-t border-slate-800">
              <span>Estimated Area:</span>
              <span>{calculatedAreaKm2.toFixed(3)} km²</span>
            </div>
            <div className="flex justify-between text-slate-400">
              <span>Estimated 50m Cells:</span>
              <span>~{Math.round(calculatedAreaKm2 / 0.0025)} cells</span>
            </div>
          </div>

          {serverMessage && (
            <div className={`mt-3 p-2 rounded text-[10px] ${
              serverMessage.type === 'success' 
                ? 'bg-emerald-500/10 text-emerald-400 border border-emerald-500/30' 
                : 'bg-red-500/10 text-red-400 border border-red-500/30'
            }`}>
              {serverMessage.text}
            </div>
          )}

          <button
            onClick={handleConfirmROI}
            disabled={isSubmitting}
            className="mt-3 w-full py-2 bg-emerald-600 hover:bg-emerald-500 text-slate-950 font-bold uppercase tracking-wider rounded-lg transition-all shadow-md flex items-center justify-center space-x-1.5 text-xs disabled:opacity-50"
          >
            <CheckCircle2 className="w-4 h-4" />
            <span>{isSubmitting ? 'Registering...' : 'Confirm Search Space'}</span>
          </button>
        </div>
      )}
    </div>
  );
};

export default InteractiveMapSelector;
