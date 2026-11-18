/**
 * Geospatial Types and Interfaces for Mission Control ROI & Geodesic Grid
 */

export interface GeoPoint {
  lat: number;
  lon: number;
}

export interface BoundingBox {
  lat_min: number;
  lat_max: number;
  lon_min: number;
  lon_max: number;
}

export interface SearchSpaceROI {
  roi_type: 'RECTANGLE' | 'POLYGON';
  bounds: BoundingBox;
  vertices: GeoPoint[];
  area_km2: number;
}

export interface SearchSpaceROIResponse {
  status: string;
  roi_id: string;
  roi_type: string;
  bounds: BoundingBox;
  vertices_count: number;
  area_km2: number;
  estimated_50m_cells: number;
  created_at: string;
  message: string;
}

export interface GridMetadata {
  rows: number;
  cols: number;
  total_cells: number;
  active_cells: number;
  masked_cells: number;
  cell_size_m: number;
  delta_lat_deg: number;
  delta_lon_deg: number;
  mean_latitude: number;
}

export interface GridCellFeature {
  type: 'Feature';
  id: number;
  geometry: {
    type: 'Polygon';
    coordinates: number[][][]; // GeoJSON [lon, lat] rings
  };
  properties: {
    cell_id: number;
    row: number;
    col: number;
    center_lat: number;
    center_lon: number;
    elevation_m: number;
    terrain_roughness: number;
    priority: number;
    is_masked_out: boolean;
    is_scanned: boolean;
  };
}

export interface GridGeoJsonResponse {
  type: 'FeatureCollection';
  properties: GridMetadata;
  features: GridCellFeature[];
}

export interface GridGenerationResponse {
  status: string;
  metadata: GridMetadata;
  geojson: GridGeoJsonResponse;
  message: string;
}
