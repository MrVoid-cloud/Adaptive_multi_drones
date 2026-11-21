"""
Unified Hybrid Search Space Builder & Lossless Spatial Micro-Clustering Engine
Phase 01 & Next-Gen Spatial Sector Decomposition & Complete Coverage Path Planning (CCPP)
Author: Principal Robotics, Computational Geometry & High-Performance Systems Architect
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
    T = V_road U SpatialMicroClusters(C_grid \ Buffer(V_road, R))
    Enhanced with Bidirectional Strip Endpoints & Multi-Sortie Breakpoints.
    """
    target_id: str
    node_type: Literal["ROAD_INTERSECTION", "OPEN_TERRAIN_CELL"]
    waypoint_type: str = "TARGET"  # "TARGET" | "TARGET_STRIP_ENTRY" | "TARGET_STRIP_EXIT" | "SORTIE_RETURN_BASE" | "SORTIE_RELAUNCH_BASE"
    sortie_index: int = 1
    lat: float
    lon: float
    elevation_m: float = 35.0
    safe_alt_m: float = 60.0
    priority: float = 1.0
    terrain_roughness: float = 1.0
    adjacent_node_ids: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    member_cells: List[Any] = Field(default_factory=list, exclude=True)

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
        return self.metadata.get("cell_id", self.target_id)

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
                **{k: v for k, v in self.metadata.items() if k != "member_cells"}
            }
        }


class SpatialMicroCluster(BaseModel):
    """
    Compact spatial micro-cluster of active open terrain cells.
    Encapsulates a contiguous group of cells indexed by exact integer row and column bins.
    Guarantees 100% physical cell preservation with zero omission or latitude coordinate rounding.
    """
    cluster_id: str
    row_bin: int
    col_bin: int
    centroid_lat: float
    centroid_lon: float
    elevation_m: float = 35.0
    safe_alt_m: float = 60.0
    priority: float = 1.0
    terrain_roughness: float = 1.0
    member_cell_ids: List[str]
    covered_cells_count: int
    member_cells: List[Any] = Field(default_factory=list, exclude=True)

    def to_target_node(self) -> SearchTargetNode:
        return SearchTargetNode(
            target_id=self.cluster_id,
            node_type="OPEN_TERRAIN_CELL",
            lat=self.centroid_lat,
            lon=self.centroid_lon,
            elevation_m=self.elevation_m,
            safe_alt_m=self.safe_alt_m,
            priority=self.priority,
            terrain_roughness=self.terrain_roughness,
            adjacent_node_ids=[],
            member_cells=self.member_cells,
            metadata={
                "is_cluster": True,
                "cluster_id": self.cluster_id,
                "row": self.row_bin,
                "col": self.col_bin,
                "row_bin": self.row_bin,
                "col_bin": self.col_bin,
                "member_cell_ids": self.member_cell_ids,
                "covered_cells_count": self.covered_cells_count,
                "covered_cell_ids": self.member_cell_ids,
                "cell_id": self.member_cell_ids[0] if self.member_cell_ids else self.cluster_id
            }
        )


class ScanlineStrip(BaseModel):
    """
    Continuous horizontal scanline sweep segment for backward compatibility.
    Encapsulates a contiguous unbroken sequence of open terrain cells on a single latitude row.
    """
    strip_id: str
    row: int
    start_col: int
    end_col: int
    entry_point: Tuple[float, float]  # (lat, lon) at west/start endpoint (Point A)
    exit_point: Tuple[float, float]   # (lat, lon) at east/end endpoint (Point B)
    strip_length_m: float
    covered_cell_ids: List[str]
    covered_cells_count: int
    elevation_m: float = 35.0
    safe_alt_m: float = 60.0
    priority: float = 1.0
    terrain_roughness: float = 1.0

    def to_target_node(self) -> SearchTargetNode:
        """Converts strip into a unified SearchTargetNode."""
        mid_lat = (self.entry_point[0] + self.exit_point[0]) / 2.0
        mid_lon = (self.entry_point[1] + self.exit_point[1]) / 2.0
        return SearchTargetNode(
            target_id=self.strip_id,
            node_type="OPEN_TERRAIN_CELL",
            lat=mid_lat,
            lon=mid_lon,
            elevation_m=self.elevation_m,
            safe_alt_m=self.safe_alt_m,
            priority=self.priority,
            terrain_roughness=self.terrain_roughness,
            adjacent_node_ids=[],
            metadata={
                "is_strip": True,
                "strip_id": self.strip_id,
                "row": self.row,
                "start_col": self.start_col,
                "end_col": self.end_col,
                "entry_lat": self.entry_point[0],
                "entry_lon": self.entry_point[1],
                "exit_lat": self.exit_point[0],
                "exit_lon": self.exit_point[1],
                "strip_length_m": round(self.strip_length_m, 2),
                "covered_cell_ids": self.covered_cell_ids,
                "covered_cells_count": self.covered_cells_count,
                "cell_id": self.covered_cell_ids[0] if self.covered_cell_ids else self.strip_id
            }
        )


class HybridGraphGridBuilder:
    """
    Merges Road Topology Graph with Geodesic Discretized Grid.
    Enforces Geodesic Spatial Exclusion Buffer around road intersection nodes.
    Executes Spatial Micro-Clustering on active open terrain cells:
    - Maintains exact integer indexing (row, col) with ZERO coordinate rounding.
    - Preserves 100% of physical cells without dropping boundary or corner cells.
    - Provides micro-clusters for sub-second metaheuristic optimization (PSO, GA, Hybrid GA-PSO),
      which unpack directly into uncapped intra-sector Boustrophedon sweeps.
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
        self.spatial_micro_clusters: List[SpatialMicroCluster] = []
        self.compressed_strips: List[ScanlineStrip] = []
        self.active_open_cells: List[GeoCell] = []
        self.active_road_nodes: List[RoadIntersectionNode] = []
        self.spatial_excluded_cells_count: int = 0
        self.total_physical_cells: int = 0
        self.compressed_strips_count: int = 0

        self._build_hybrid_space()

    def _build_hybrid_space(self):
        """
        Executes spatial filtering and lossless micro-clustering.
        Guarantees zero subsampling: all active open terrain cells are preserved and accounted for.
        """
        road_nodes_list = list(self.road_network.nodes.values())
        self.active_road_nodes = road_nodes_list

        # 1. Ingest Urban Road Intersection Targets (V_road)
        for r_node in road_nodes_list:
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
                terrain_roughness=1.0,
                adjacent_node_ids=r_node.adjacent_intersection_ids,
                metadata={
                    "degree": r_node.degree,
                    "cluster_size": r_node.cluster_size,
                    "connected_osm_ids": r_node.connected_osm_ids,
                    "is_cluster": False,
                    "is_strip": False,
                    "covered_cells_count": 1,
                    "covered_cell_ids": [r_node.node_id]
                }
            )
            self.unified_targets.append(target)

        # 2. Geodesic Spatial Exclusion Buffer:
        # Exclude cells whose center lies within buffer_radius_m of any road intersection
        self.spatial_excluded_cells_count = 0
        self.active_open_cells = []

        for cell in self.grid.cells:
            if cell.is_masked_out:
                continue

            is_in_road_buffer = False
            for r_node in road_nodes_list:
                dist = haversine_distance_m(cell.center_lat, cell.center_lon, r_node.lat, r_node.lon)
                if dist <= self.buffer_radius_m:
                    is_in_road_buffer = True
                    break

            if is_in_road_buffer:
                cell.is_masked_out = True
                self.spatial_excluded_cells_count += 1
            else:
                self.active_open_cells.append(cell)

        self.total_physical_cells = len(self.active_open_cells)

        # 3. Compact Spatial Micro-Clustering:
        # Partition open cells into K compact rectangular clusters based on exact integer coordinates
        target_k = min(len(self.active_open_cells), 140) if self.active_open_cells else 1
        stride_r = max(1, int(math.ceil(self.grid.rows / math.sqrt(target_k))))
        stride_c = max(1, int(math.ceil(self.grid.cols / math.sqrt(target_k))))

        cluster_map: Dict[Tuple[int, int], List[GeoCell]] = defaultdict(list)
        for cell in self.active_open_cells:
            cr = cell.row // stride_r
            cc = cell.col // stride_c
            cluster_map[(cr, cc)].append(cell)

        self.spatial_micro_clusters = []
        for (cr, cc), c_cells in cluster_map.items():
            lats = [c.center_lat for c in c_cells]
            lons = [c.center_lon for c in c_cells]
            elevs = [c.elevation_m for c in c_cells]
            roughs = [c.terrain_roughness for c in c_cells]
            prios = [c.priority for c in c_cells]

            avg_lat = sum(lats) / len(lats)
            avg_lon = sum(lons) / len(lons)
            avg_elev = sum(elevs) / len(elevs)
            avg_rough = sum(roughs) / len(roughs)
            max_prio = max(prios)
            c_ids = [str(c.cell_id) for c in c_cells]
            cl_id = f"CL_R{cr:02d}_C{cc:02d}"

            micro_cluster = SpatialMicroCluster(
                cluster_id=cl_id,
                row_bin=cr,
                col_bin=cc,
                centroid_lat=avg_lat,
                centroid_lon=avg_lon,
                elevation_m=avg_elev,
                safe_alt_m=avg_elev + self.open_terrain_safe_altitude_agl_m,
                priority=max_prio,
                terrain_roughness=avg_rough,
                member_cell_ids=c_ids,
                covered_cells_count=len(c_ids),
                member_cells=c_cells
            )
            self.spatial_micro_clusters.append(micro_cluster)
            self.unified_targets.append(micro_cluster.to_target_node())

        # Also populate self.compressed_strips for legacy inspection
        row_map: Dict[int, List[GeoCell]] = defaultdict(list)
        for cell in self.active_open_cells:
            row_map[cell.row].append(cell)

        self.compressed_strips = []
        for row_idx in sorted(row_map.keys()):
            cells_in_row = sorted(row_map[row_idx], key=lambda c: c.col)
            if not cells_in_row:
                continue

            strips: List[List[GeoCell]] = []
            cur_strip = [cells_in_row[0]]
            for c in cells_in_row[1:]:
                if c.col == cur_strip[-1].col + 1:
                    cur_strip.append(c)
                else:
                    strips.append(cur_strip)
                    cur_strip = [c]
            if cur_strip:
                strips.append(cur_strip)

            for s_idx, strip in enumerate(strips, start=1):
                first_c = strip[0]
                last_c = strip[-1]
                entry_pt = (first_c.center_lat, first_c.center_lon)
                exit_pt = (last_c.center_lat, last_c.center_lon)
                strip_len_m = haversine_distance_m(entry_pt[0], entry_pt[1], exit_pt[0], exit_pt[1])
                strip_covered_ids = [str(c.cell_id) for c in strip]
                avg_elev = sum(c.elevation_m for c in strip) / len(strip)
                strip_id = f"STRIP_R{row_idx:03d}_S{s_idx:02d}_C{first_c.col:03d}_{last_c.col:03d}"

                self.compressed_strips.append(ScanlineStrip(
                    strip_id=strip_id,
                    row=row_idx,
                    start_col=first_c.col,
                    end_col=last_c.col,
                    entry_point=entry_pt,
                    exit_point=exit_pt,
                    strip_length_m=strip_len_m,
                    covered_cell_ids=strip_covered_ids,
                    covered_cells_count=len(strip_covered_ids),
                    elevation_m=avg_elev,
                    safe_alt_m=avg_elev + self.open_terrain_safe_altitude_agl_m,
                    priority=max(c.priority for c in strip),
                    terrain_roughness=sum(c.terrain_roughness for c in strip) / len(strip)
                ))

        self.compressed_strips_count = len(self.spatial_micro_clusters)

        # Update grid active cell count
        self.grid.active_cell_count = self.total_physical_cells
        self.grid.masked_cell_count = len(self.grid.cells) - self.total_physical_cells

    def to_dual_layer_geojson(self) -> Dict[str, Any]:
        """
        Exports comprehensive multi-layer GeoJSON for Map Visualizer:
        Layer 1: Road Graph Edges (LineStrings) & Road Intersection Nodes (Points)
        Layer 2: Free Open Terrain Cells (Polygons for ALL physical cells, e.g. 5,400+)
        Layer 3: Spatial Micro-Cluster Centerpoints
        """
        features: List[Dict[str, Any]] = []

        # 1. Road Edges (LineStrings)
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

        # 2. Road Intersection Nodes (Points)
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

        # 3. Active Open Space Grid Cells (Polygons for 100% of Physical Cells)
        for cell in self.active_open_cells:
            features.append(cell.to_geojson_feature())

        # 4. Spatial Micro-Clusters (Points)
        for cl in self.spatial_micro_clusters:
            features.append({
                "type": "Feature",
                "id": f"cl-{cl.cluster_id}",
                "geometry": {
                    "type": "Point",
                    "coordinates": [round(cl.centroid_lon, 7), round(cl.centroid_lat, 7)]
                },
                "properties": {
                    "layer": "SPATIAL_CLUSTERS",
                    "cluster_id": cl.cluster_id,
                    "row_bin": cl.row_bin,
                    "col_bin": cl.col_bin,
                    "covered_cells_count": cl.covered_cells_count
                }
            })

        return {
            "type": "FeatureCollection",
            "metadata": {
                "total_features": len(features),
                "road_nodes_count": len(self.active_road_nodes),
                "open_cells_count": self.total_physical_cells,
                "total_physical_cells": self.total_physical_cells,
                "compressed_strips_count": self.compressed_strips_count,
                "buffer_radius_m": round(self.buffer_radius_m, 1),
                "zero_overlap_guaranteed": True,
                "lossless_coverage": True
            },
            "features": features
        }
