"""
Unified Hybrid Search Space Builder & Spatial Exclusion Buffer
Phase 01: Context-Aware Hybrid Graph-Grid Planning
Author: Principal Software Architect
"""

import math
from typing import Dict, Any, List, Optional, Literal, Tuple
from collections import defaultdict
from pydantic import BaseModel, Field

from app.gis.grid import GeodesicGridDiscretizer, GeoCell
from app.gis.road_topology_extractor import (
    RoadIntersectionNode,
    RoadSegmentEdge,
    ExtractedRoadNetwork,
    haversine_distance_m
)


class SearchTargetNode(BaseModel):
    r"""
    Unified target entity in the hybrid search space:
    T = V_road U (C_grid \ Buffer(V_road, R))
    Enhanced with Multi-Sortie Breakpoints & Waypoint Type Classification.
    """
    target_id: str
    node_type: Literal["ROAD_INTERSECTION", "OPEN_TERRAIN_CELL"]
    waypoint_type: str = "TARGET"  # "TARGET" | "SORTIE_RETURN_BASE" | "SORTIE_RELAUNCH_BASE"
    sortie_index: int = 1
    lat: float
    lon: float
    elevation_m: float = 35.0
    safe_alt_m: float = 60.0
    priority: float = 1.0
    terrain_roughness: float = 1.0
    adjacent_node_ids: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)

    @property
    def center_lat(self) -> float:
        """Alias for backward compatibility with GeoCell."""
        return self.lat

    @property
    def center_lon(self) -> float:
        """Alias for backward compatibility with GeoCell."""
        return self.lon

    @property
    def cell_id(self) -> Any:
        """Alias for backward compatibility with GeoCell."""
        return self.target_id

    @property
    def row(self) -> int:
        """Alias for backward compatibility with GeoCell."""
        return int(self.metadata.get("row", 0))

    @property
    def col(self) -> int:
        """Alias for backward compatibility with GeoCell."""
        return int(self.metadata.get("col", 0))

    @property
    def is_masked_out(self) -> bool:
        """Alias for backward compatibility with GeoCell."""
        return False

    def to_geojson_feature(self) -> Dict[str, Any]:
        """Converts target to a GeoJSON Point feature."""
        return {
            "type": "Feature",
            "id": self.target_id,
            "geometry": {
                "type": "Point",
                "coordinates": [round(self.lon, 7), round(self.lat, 7), round(self.safe_alt_m, 1)]
            },
            "properties": {
                "target_id": self.target_id,
                "node_type": self.node_type,
                "waypoint_type": self.waypoint_type,
                "sortie_index": self.sortie_index,
                "priority": self.priority,
                "safe_alt_m": self.safe_alt_m,
                "terrain_roughness": self.terrain_roughness,
                "elevation_m": self.elevation_m,
                "adjacent_count": len(self.adjacent_node_ids),
                **self.metadata
            }
        }


class HybridGraphGridBuilder:
    """
    Merges Road Topology Graph with Geodesic Discretized Grid.
    Enforces a Geodesic Spatial Exclusion Buffer around road intersection nodes,
    preventing duplicate search cell allocations in urban zones.
    """
    def __init__(
        self,
        grid: GeodesicGridDiscretizer,
        road_network: ExtractedRoadNetwork,
        buffer_radius_ratio: float = 0.65,
        urban_safe_altitude_agl_m: float = 80.0,
        open_terrain_safe_altitude_agl_m: float = 50.0,
        road_priority: float = 2.5
    ):
        self.grid = grid
        self.road_network = road_network
        self.buffer_radius_m = grid.cell_size_m * buffer_radius_ratio
        self.urban_safe_altitude_agl_m = urban_safe_altitude_agl_m
        self.open_terrain_safe_altitude_agl_m = open_terrain_safe_altitude_agl_m
        self.road_priority = road_priority

        self.unified_targets: List[SearchTargetNode] = []
        self.spatial_excluded_cells_count: int = 0
        self.active_open_cells: List[GeoCell] = []
        self.active_road_nodes: List[RoadIntersectionNode] = []

        self._build_hybrid_space()

    def _build_hybrid_space(self):
        """Executes spatial filtering and compiles unified target collection."""
        road_nodes_list = list(self.road_network.nodes.values())
        self.active_road_nodes = road_nodes_list

        # 1. Ingest Road Intersection Targets (V_road)
        for r_node in road_nodes_list:
            # Query approximate elevation from nearest grid cell
            cell_near = self.grid.coord_to_cell(r_node.lat, r_node.lon)
            elev = cell_near.elevation_m if cell_near else 35.0

            target = SearchTargetNode(
                target_id=r_node.node_id,
                node_type="ROAD_INTERSECTION",
                lat=r_node.lat,
                lon=r_node.lon,
                elevation_m=elev,
                safe_alt_m=elev + self.urban_safe_altitude_agl_m,
                priority=self.road_priority,
                terrain_roughness=1.0,  # Paved road corridor has baseline friction
                adjacent_node_ids=r_node.adjacent_intersection_ids,
                metadata={
                    "degree": r_node.degree,
                    "cluster_size": r_node.cluster_size,
                    "connected_osm_ids": r_node.connected_osm_ids
                }
            )
            self.unified_targets.append(target)

        # 2. Spatial Exclusion Buffer Filtering on Grid Cells:
        # Exclude any cell whose center lies within buffer_radius_m of any road intersection
        self.spatial_excluded_cells_count = 0
        self.active_open_cells = []

        for cell in self.grid.cells:
            if cell.is_masked_out:
                continue

            # Check distance against all road intersection nodes
            is_in_road_buffer = False
            for r_node in road_nodes_list:
                dist = haversine_distance_m(cell.center_lat, cell.center_lon, r_node.lat, r_node.lon)
                if dist <= self.buffer_radius_m:
                    is_in_road_buffer = True
                    break

            if is_in_road_buffer:
                # Mask out this cell to enforce ZERO-OVERLAP with road nodes
                cell.is_masked_out = True
                self.spatial_excluded_cells_count += 1
            else:
                self.active_open_cells.append(cell)

        # 3. Macro-Waypoint Ingestion for Combinatorial Scalability:
        # If open cells <= 400, keep atomic cell targets.
        # If open cells > 400 (e.g. large 13+ km² areas with 3,000+ cells), compress contiguous
        # horizontal strips into Boustrophedon Macro-Waypoints (Entry & Exit endpoints)
        # to ensure < 1.0s optimization convergence without losing 1m² of 100% full physical coverage.
        if len(self.active_open_cells) <= 400:
            for cell in self.active_open_cells:
                target = SearchTargetNode(
                    target_id=f"CELL_{cell.cell_id:04d}",
                    node_type="OPEN_TERRAIN_CELL",
                    lat=cell.center_lat,
                    lon=cell.center_lon,
                    elevation_m=cell.elevation_m,
                    safe_alt_m=cell.elevation_m + self.open_terrain_safe_altitude_agl_m,
                    priority=cell.priority,
                    terrain_roughness=cell.terrain_roughness,
                    adjacent_node_ids=[],
                    metadata={
                        "cell_id": cell.cell_id,
                        "row": cell.row,
                        "col": cell.col,
                        "area_m2": cell.area_m2,
                        "covered_cell_ids": [str(cell.cell_id)]
                    }
                )
                self.unified_targets.append(target)
        else:
            # Group open cells by row
            row_map = defaultdict(list)
            for cell in self.active_open_cells:
                row_map[cell.row].append(cell)

            for row_idx in sorted(row_map.keys()):
                cells_in_row = sorted(row_map[row_idx], key=lambda c: c.col)
                # Split row into contiguous column strips
                strips = []
                current_strip = [cells_in_row[0]]
                for c in cells_in_row[1:]:
                    if c.col == current_strip[-1].col + 1:
                        current_strip.append(c)
                    else:
                        strips.append(current_strip)
                        current_strip = [c]
                if current_strip:
                    strips.append(current_strip)

                for strip in strips:
                    if len(strip) == 1:
                        c = strip[0]
                        target = SearchTargetNode(
                            target_id=f"CELL_{c.cell_id:04d}",
                            node_type="OPEN_TERRAIN_CELL",
                            lat=c.center_lat,
                            lon=c.center_lon,
                            elevation_m=c.elevation_m,
                            safe_alt_m=c.elevation_m + self.open_terrain_safe_altitude_agl_m,
                            priority=c.priority,
                            terrain_roughness=c.terrain_roughness,
                            adjacent_node_ids=[],
                            metadata={
                                "cell_id": c.cell_id,
                                "row": c.row,
                                "col": c.col,
                                "area_m2": c.area_m2,
                                "covered_cell_ids": [str(c.cell_id)]
                            }
                        )
                        self.unified_targets.append(target)
                    else:
                        first_c = strip[0]
                        last_c = strip[-1]
                        strip_covered_ids = [str(c.cell_id) for c in strip]
                        avg_rough = sum(c.terrain_roughness for c in strip) / len(strip)
                        max_prio = max(c.priority for c in strip)
                        strip_id = f"STRIP_R{row_idx:03d}_C{first_c.col:03d}_{last_c.col:03d}"

                        # Entry Macro-Waypoint
                        entry_target = SearchTargetNode(
                            target_id=f"{strip_id}_START",
                            node_type="OPEN_TERRAIN_CELL",
                            lat=first_c.center_lat,
                            lon=first_c.center_lon,
                            elevation_m=first_c.elevation_m,
                            safe_alt_m=first_c.elevation_m + self.open_terrain_safe_altitude_agl_m,
                            priority=max_prio,
                            terrain_roughness=avg_rough,
                            adjacent_node_ids=[f"{strip_id}_END"],
                            metadata={
                                "is_strip_macro": True,
                                "strip_id": strip_id,
                                "strip_role": "ENTRY",
                                "peer_id": f"{strip_id}_END",
                                "row": row_idx,
                                "start_col": first_c.col,
                                "end_col": last_c.col,
                                "covered_cells_count": len(strip),
                                "covered_cell_ids": strip_covered_ids,
                                "cell_id": first_c.cell_id
                            }
                        )
                        # Exit Macro-Waypoint
                        exit_target = SearchTargetNode(
                            target_id=f"{strip_id}_END",
                            node_type="OPEN_TERRAIN_CELL",
                            lat=last_c.center_lat,
                            lon=last_c.center_lon,
                            elevation_m=last_c.elevation_m,
                            safe_alt_m=last_c.elevation_m + self.open_terrain_safe_altitude_agl_m,
                            priority=max_prio,
                            terrain_roughness=avg_rough,
                            adjacent_node_ids=[f"{strip_id}_START"],
                            metadata={
                                "is_strip_macro": True,
                                "strip_id": strip_id,
                                "strip_role": "EXIT",
                                "peer_id": f"{strip_id}_START",
                                "row": row_idx,
                                "start_col": first_c.col,
                                "end_col": last_c.col,
                                "covered_cells_count": len(strip),
                                "covered_cell_ids": strip_covered_ids,
                                "cell_id": last_c.cell_id
                            }
                        )
                        self.unified_targets.append(entry_target)
                        self.unified_targets.append(exit_target)

        # Update grid active cell count
        self.grid.active_cell_count = len(self.active_open_cells)
        self.grid.masked_cell_count = len(self.grid.cells) - len(self.active_open_cells)

    def to_dual_layer_geojson(self) -> Dict[str, Any]:
        """
        Exports comprehensive multi-layer GeoJSON for Map Visualizer:
        Layer 1: Road Graph Edges (LineStrings) & Road Intersection Nodes (Points)
        Layer 2: Free Open Terrain Cells (Polygons)
        Layer 3: Unified Targets (Points)
        """
        features: List[Dict[str, Any]] = []

        # 1. Road Edges
        for edge in self.road_network.edges:
            coords = [[round(lon, 7), round(lat, 7)] for lat, lon in edge.geometry]
            features.append({
                "type": "Feature",
                "id": edge.edge_id,
                "geometry": {
                    "type": "LineString",
                    "coordinates": coords
                },
                "properties": {
                    "layer": "ROAD_EDGES",
                    "edge_id": edge.edge_id,
                    "u": edge.u,
                    "v": edge.v,
                    "length_m": edge.length_m,
                    "highway_type": edge.highway_type
                }
            })

        # 2. Road Intersection Nodes
        for r_node in self.active_road_nodes:
            features.append({
                "type": "Feature",
                "id": f"marker-{r_node.node_id}",
                "geometry": {
                    "type": "Point",
                    "coordinates": [round(r_node.lon, 7), round(r_node.lat, 7)]
                },
                "properties": {
                    "layer": "ROAD_INTERSECTIONS",
                    "node_id": r_node.node_id,
                    "degree": r_node.degree,
                    "cluster_size": r_node.cluster_size
                }
            })

        # 3. Active Open Space Grid Cells (Polygons)
        for cell in self.active_open_cells:
            features.append(cell.to_geojson_feature())

        return {
            "type": "FeatureCollection",
            "metadata": {
                "total_features": len(features),
                "road_nodes_count": len(self.active_road_nodes),
                "open_cells_count": len(self.active_open_cells),
                "buffer_radius_m": round(self.buffer_radius_m, 1),
                "zero_overlap_guaranteed": True
            },
            "features": features
        }
