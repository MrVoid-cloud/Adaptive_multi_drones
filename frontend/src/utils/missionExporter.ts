/**
 * Mission Data Exporters for Geographic, LiDAR DEM & Quantitative Analysis
 * Formats: GeoJSON (QGIS/ArcGIS/Cesium), LiDAR DEM CSV/JSON, Benchmark Reports
 */

import { AlgorithmResult, TriAlgorithmBenchmarkResponse } from '../types/simulation';

function downloadFile(content: string, filename: string, mimeType: string) {
  const blob = new Blob([content], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  URL.revokeObjectURL(url);
}

/**
 * 1. Export Mission Plan to Standard GeoJSON FeatureCollection
 */
export function exportMissionGeoJSON(
  plan: AlgorithmResult,
  missionMetadata: Record<string, any> = {}
) {
  const features: any[] = [];

  Object.entries(plan.routes || {}).forEach(([droneId, waypoints]) => {
    // Linestring for drone flight corridor
    features.push({
      type: 'Feature',
      id: `corridor-${droneId}`,
      properties: {
        featureType: 'FLIGHT_CORRIDOR',
        droneId,
        algorithm: plan.algorithm,
        totalWaypoints: waypoints.length
      },
      geometry: {
        type: 'LineString',
        coordinates: waypoints.map(wp => [wp.lon, wp.lat, wp.alt_m])
      }
    });

    // Point features for individual waypoints
    waypoints.forEach((wp, wIdx) => {
      features.push({
        type: 'Feature',
        id: `waypoint-${droneId}-${wp.cell_id}`,
        properties: {
          featureType: 'WAYPOINT',
          droneId,
          cellId: wp.cell_id,
          sequence: wIdx + 1,
          altitudeMeters: wp.alt_m,
          roughness: wp.roughness,
          priority: wp.priority
        },
        geometry: {
          type: 'Point',
          coordinates: [wp.lon, wp.lat, wp.alt_m]
        }
      });
    });
  });

  const totalCells = Object.values(plan.routes || {}).reduce((acc, r) => acc + r.length, 0);

  const geoJson = {
    type: 'FeatureCollection',
    metadata: {
      generatedBy: 'Adaptive Multi-Drone Mission Command (PWA)',
      algorithm: plan.algorithm,
      makespanSeconds: plan.makespan_sec,
      totalEnergyWh: plan.total_energy_wh,
      totalCellsCovered: totalCells,
      timestamp: missionMetadata.timestamp || new Date().toISOString(),
      ...missionMetadata
    },
    features
  };

  const fileName = `mission_plan_${plan.algorithm.replace(/\s+/g, '_').toLowerCase()}_${Date.now()}.geojson`;
  downloadFile(JSON.stringify(geoJson, null, 2), fileName, 'application/geo+json');
}

/**
 * 2. Export Pseudo-3D LiDAR Scanned DEM Elevation Matrix (CSV & JSON)
 */
export function exportElevationDEM(
  elevationGrid: Record<number, number> | Map<number, number>,
  format: 'CSV' | 'JSON' = 'CSV'
) {
  const gridObj: Record<number, number> = elevationGrid instanceof Map 
    ? Object.fromEntries(elevationGrid) 
    : elevationGrid;

  const cellIds = Object.keys(gridObj).map(Number).sort((a, b) => a - b);

  if (format === 'JSON') {
    const jsonContent = JSON.stringify({
      title: 'Airborne LiDAR Digital Elevation Model (Pseudo-3D DEM)',
      totalScannedCells: cellIds.length,
      timestamp: new Date().toISOString(),
      elevations: gridObj
    }, null, 2);

    downloadFile(jsonContent, `lidar_dem_${Date.now()}.json`, 'application/json');
    return;
  }

  // Format as CSV
  let csvContent = 'cell_id,elevation_m\n';
  cellIds.forEach(id => {
    csvContent += `${id},${gridObj[id].toFixed(2)}\n`;
  });

  downloadFile(csvContent, `lidar_dem_${Date.now()}.csv`, 'text/csv;charset=utf-8;');
}

/**
 * 3. Export Tri-Algorithm Quantitative Benchmark Comparison Report (CSV & Markdown)
 */
export function exportTriAlgorithmBenchmarkReport(
  benchmark: TriAlgorithmBenchmarkResponse,
  format: 'CSV' | 'MD' = 'CSV'
) {
  const algorithms = [
    { key: 'Discrete PSO', res: benchmark.pso_result },
    { key: 'Modified GA', res: benchmark.ga_result },
    { key: 'Hybrid GA-PSO', res: benchmark.hybrid_result }
  ];

  if (format === 'MD') {
    let md = '# Tri-Algorithm Multi-Drone Optimization Benchmark Report\n\n';
    md += `**Generated:** ${new Date().toISOString()}\n`;
    md += `**Total Search Space Cells:** ${benchmark.active_cells_count || 0}\n`;
    md += `**Fleet Size:** ${benchmark.fleet_size} UAVs\n\n`;
    md += '| Metric | 1. Discrete PSO | 2. Modified GA | 3. Hybrid GA-PSO |\n';
    md += '| :--- | :--- | :--- | :--- |\n';
    md += `| **Fitness Score** | ${benchmark.pso_result.best_fitness.toFixed(2)} | ${benchmark.ga_result.best_fitness.toFixed(2)} | **${benchmark.hybrid_result.best_fitness.toFixed(2)}** |\n`;
    md += `| **Makespan (s)** | ${benchmark.pso_result.makespan_sec.toFixed(1)}s | ${benchmark.ga_result.makespan_sec.toFixed(1)}s | **${benchmark.hybrid_result.makespan_sec.toFixed(1)}s** |\n`;
    md += `| **Energy (Wh)** | ${benchmark.pso_result.total_energy_wh.toFixed(1)}Wh | ${benchmark.ga_result.total_energy_wh.toFixed(1)}Wh | **${benchmark.hybrid_result.total_energy_wh.toFixed(1)}Wh** |\n`;
    md += `| **Compute Time (ms)** | ${benchmark.pso_result.compute_time_ms.toFixed(1)}ms | ${benchmark.ga_result.compute_time_ms.toFixed(1)}ms | **${benchmark.hybrid_result.compute_time_ms.toFixed(1)}ms** |\n\n`;
    
    if (benchmark.kpi_comparison) {
      md += `### Hybrid vs PSO Advantage:\n`;
      md += `- Makespan Reduction: **${benchmark.kpi_comparison.makespan_reduction_vs_pso_pct.toFixed(1)}%**\n`;
    }

    downloadFile(md, `benchmark_report_${Date.now()}.md`, 'text/markdown');
    return;
  }

  // Format as CSV
  let csv = 'algorithm,fitness_score,makespan_s,energy_wh,compute_time_ms\n';
  algorithms.forEach(alg => {
    csv += `"${alg.key}",${alg.res.best_fitness.toFixed(2)},${alg.res.makespan_sec.toFixed(1)},${alg.res.total_energy_wh.toFixed(1)},${alg.res.compute_time_ms.toFixed(1)}\n`;
  });

  downloadFile(csv, `tri_algorithm_benchmark_${Date.now()}.csv`, 'text/csv;charset=utf-8;');
}
