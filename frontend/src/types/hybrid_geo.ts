/**
 * Types & Interfaces for Context-Aware Hybrid Graph-Grid Planning
 * Phase 03: Dual-Mode Google Maps Visualization & Mixed Waypoint Schema
 * Author: Principal Software Architect
 */

import { BoundingBox, GeoPoint } from './geo';
import { Waypoint } from './simulation';

export type NodeType = 'ROAD_INTERSECTION' | 'OPEN_TERRAIN_CELL';

export interface SearchTargetNode {
  target_id: string;
  node_type: NodeType;
  lat: number;
  lon: number;
  elevation_m: number;
  safe_alt_m: number;
  priority: number;
  terrain_roughness: number;
  adjacent_node_ids: string[];
  metadata?: Record<string, any>;
}

export interface RoadSegmentEdge {
  edge_id: string;
  u: string;
  v: string;
  length_m: number;
  geometry: [number, number][]; // [lat, lon] tuples
  highway_type: string;
}

export interface HybridSpaceStatistics {
  total_targets: number;
  road_intersection_count: number;
  open_terrain_cells_count: number;
  spatial_excluded_cells_count: number;
  road_edges_count: number;
  buffer_radius_m: number;
  zero_overlap_verified: boolean;
  source: string;
}

export interface HybridGridFeature {
  type: 'Feature';
  id?: string | number;
  geometry: {
    type: 'Point' | 'LineString' | 'Polygon';
    coordinates: any;
  };
  properties: {
    layer?: 'ROAD_EDGES' | 'ROAD_INTERSECTIONS' | 'OPEN_SPACE_CELLS';
    node_type?: NodeType;
    target_id?: string;
    cell_id?: number | string;
    safe_alt_m?: number;
    elevation_m?: number;
    priority?: number;
    degree?: number;
    cluster_size?: number;
    length_m?: number;
    highway_type?: string;
    is_masked_out?: boolean;
    is_scanned?: boolean;
    terrain_roughness?: number;
    [key: string]: any;
  };
}

export interface DualLayerGeoJsonResponse {
  type: 'FeatureCollection';
  metadata: {
    total_features: number;
    road_nodes_count: number;
    open_cells_count: number;
    buffer_radius_m: number;
    zero_overlap_guaranteed: boolean;
  };
  features: HybridGridFeature[];
}

export interface HybridGridGenerationResponse {
  status: string;
  statistics: HybridSpaceStatistics;
  targets: SearchTargetNode[];
  geojson: DualLayerGeoJsonResponse;
  message: string;
}

export interface HybridWaypoint extends Waypoint {
  target_id?: string;
  node_type?: NodeType;
  safe_alt_m?: number;
}
