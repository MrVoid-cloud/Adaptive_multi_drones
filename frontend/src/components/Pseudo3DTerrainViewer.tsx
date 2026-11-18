import React, { useRef, useEffect, useState } from 'react';
import { Layers, RotateCcw, Maximize2, Compass } from 'lucide-react';

interface Pseudo3DTerrainViewerProps {
  scannedElevationMap: Map<number, number>; // cell_id -> elevation_m
  gridRows?: number;
  gridCols?: number;
}

export const Pseudo3DTerrainViewer: React.FC<Pseudo3DTerrainViewerProps> = ({
  scannedElevationMap,
  gridRows = 16,
  gridCols = 16
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  // Rotation & projection state
  const [angleDeg, setAngleDeg] = useState<number>(35);
  const [elevationScale, setElevationScale] = useState<number>(0.85);
  const [isDragging, setIsDragging] = useState<boolean>(false);
  const lastMouseX = useRef<number>(0);

  // Build synthetic baseline DEM matrix combined with live scanned points
  const getElevation = (r: number, c: number): number => {
    const cellId = r * gridCols + c;
    if (scannedElevationMap.has(cellId)) {
      return scannedElevationMap.get(cellId)!;
    }
    // Baseline terrain shape (harmonic mountain ridge + valley)
    const normX = (c / gridCols - 0.5) * 2.0;
    const normY = (r / gridRows - 0.5) * 2.0;
    const peak = 45.0 * Math.exp(-(normX * normX + normY * normY) / 0.4);
    const wave = 12.0 * Math.sin(normX * 3.0) * Math.cos(normY * 3.0);
    return Math.max(5.0, 15.0 + peak + wave);
  };

  // Color mapping by elevation
  const getHypsometricColor = (z: number): string => {
    if (z < 15.0) return '#065f46'; // Deep valley green
    if (z < 25.0) return '#10b981'; // Meadow emerald
    if (z < 40.0) return '#d97706'; // Rolling amber
    if (z < 55.0) return '#b45309'; // Mountain brown
    return '#f1f5f9';               // Snow peak white
  };

  // Draw Isometric 3D Mesh
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;
    ctx.clearRect(0, 0, width, height);

    // Center coordinates
    const originX = width / 2;
    const originY = height / 2 + 30;

    const rad = (angleDeg * Math.PI) / 180;
    const cosAngle = Math.cos(rad);
    const sinAngle = Math.sin(rad);

    const cellSpacing = Math.min(width, height) / (gridRows + gridCols) * 1.6;

    // Project grid point (r, c, z) to 2D isometric screen coords
    const project = (r: number, c: number, z: number) => {
      const xOffset = (c - gridCols / 2) * cellSpacing;
      const yOffset = (r - gridRows / 2) * cellSpacing;

      const rotX = xOffset * cosAngle - yOffset * sinAngle;
      const rotY = xOffset * sinAngle + yOffset * cosAngle;

      const screenX = originX + rotX;
      const screenY = originY + rotY * 0.5 - z * elevationScale;
      return { x: screenX, y: screenY };
    };

    // Draw terrain quads from back to front (Painter's Algorithm)
    for (let r = 0; r < gridRows - 1; r++) {
      for (let c = 0; c < gridCols - 1; c++) {
        const z00 = getElevation(r, c);
        const z01 = getElevation(r, c + 1);
        const z11 = getElevation(r + 1, c + 1);
        const z10 = getElevation(r + 1, c);

        const p00 = project(r, c, z00);
        const p01 = project(r, c + 1, z01);
        const p11 = project(r + 1, c + 1, z11);
        const p10 = project(r + 1, c, z10);

        const avgZ = (z00 + z01 + z11 + z10) / 4.0;
        const fillColor = getHypsometricColor(avgZ);

        ctx.beginPath();
        ctx.moveTo(p00.x, p00.y);
        ctx.lineTo(p01.x, p01.y);
        ctx.lineTo(p11.x, p11.y);
        ctx.lineTo(p10.x, p10.y);
        ctx.closePath();

        ctx.fillStyle = fillColor;
        ctx.globalAlpha = 0.85;
        ctx.fill();

        ctx.strokeStyle = '#0f172a';
        ctx.lineWidth = 0.6;
        ctx.globalAlpha = 0.5;
        ctx.stroke();
      }
    }
  }, [scannedElevationMap, angleDeg, elevationScale, gridRows, gridCols]);

  // Mouse interaction handlers for rotation
  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    lastMouseX.current = e.clientX;
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (!isDragging) return;
    const deltaX = e.clientX - lastMouseX.current;
    lastMouseX.current = e.clientX;
    setAngleDeg(prev => (prev + deltaX * 0.5) % 360);
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  return (
    <div className="w-full h-full relative select-none bg-slate-950 flex flex-col overflow-hidden">
      {/* HUD Header */}
      <div className="h-8 border-b border-hud-border px-3 flex items-center justify-between text-xs text-slate-300 font-mono bg-hud-panel/40">
        <span className="flex items-center space-x-1.5">
          <Layers className="w-3.5 h-3.5 text-emerald-400" />
          <span className="font-bold">Pseudo-3D LiDAR DEM Surface</span>
        </span>
        <div className="flex items-center space-x-2 text-[10px] text-slate-400">
          <span className="flex items-center space-x-1">
            <Compass className="w-3 h-3 text-cyan-400" />
            <span>Azimuth: {Math.round(angleDeg)}°</span>
          </span>
          <button
            onClick={() => setAngleDeg(35)}
            className="p-1 hover:bg-slate-800 rounded text-slate-400 hover:text-slate-200"
            title="Reset View"
          >
            <RotateCcw className="w-3 h-3" />
          </button>
        </div>
      </div>

      {/* Interactive 3D Canvas */}
      <div
        className="flex-1 relative cursor-grab active:cursor-grabbing flex items-center justify-center"
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      >
        <canvas
          ref={canvasRef}
          width={650}
          height={180}
          className="w-full h-full object-contain"
        />

        {/* Hypsometric Tint Elevation Legend */}
        <div className="absolute bottom-2 right-3 flex items-center space-x-1 text-[9px] font-mono text-slate-400 bg-slate-950/80 px-2 py-0.5 rounded border border-slate-800">
          <span>0m</span>
          <div className="w-16 h-2 rounded bg-gradient-to-r from-emerald-700 via-amber-500 via-amber-800 to-slate-100" />
          <span>70m</span>
        </div>
      </div>
    </div>
  );
};

export default Pseudo3DTerrainViewer;
