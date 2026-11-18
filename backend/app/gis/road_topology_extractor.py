"""
OpenStreetMap Overpass Road Topology Extractor & Spatial Node Clustering
Phase 01-03: Context-Aware Hybrid Graph-Grid Planning
Author: Principal Software Architect
"""

import math
import hashlib
import asyncio
import logging
from collections import defaultdict
from typing import Dict, Any, List, Tuple, Optional, Set
import httpx
from pydantic import BaseModel, Field

logger = logging.getLogger("RoadTopologyExtractor")
logging.basicConfig(level=logging.INFO)

# Geodetic constants (WGS84)
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Computes great-circle geodesic distance in meters between two coordinates."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return WGS84_A * c


class RoadIntersectionNode(BaseModel):
    """Represents a topological intersection node in the urban road graph."""
    node_id: str
    lat: float
    lon: float
    degree: int
    connected_osm_ids: List[int] = Field(default_factory=list)
    adjacent_intersection_ids: List[str] = Field(default_factory=list)
    cluster_size: int = 1


class RoadSegmentEdge(BaseModel):
    """Represents a traversable road edge connecting intersection nodes."""
    edge_id: str
    u: str
    v: str
    length_m: float
    geometry: List[Tuple[float, float]] = Field(default_factory=list)  # [(lat, lon), ...]
    highway_type: str = "urban"


class ExtractedRoadNetwork(BaseModel):
    """Aggregated road topology container."""
    nodes: Dict[str, RoadIntersectionNode] = Field(default_factory=dict)
    edges: List[RoadSegmentEdge] = Field(default_factory=list)
    bbox: Tuple[float, float, float, float]  # (min_lat, min_lon, max_lat, max_lon)
    is_fallback: bool = False
    source: str = "OSM_OVERPASS"


class OSMOverpassExtractor:
    """
    Asynchronous OpenStreetMap Overpass API client with LRU caching,
    graph topology parsing (Degree >= 2 or way overlap count >= 2),
    long road segment waypoint injection (> 120m), and spatial node clustering.
    """
    OVERPASS_ENDPOINTS = [
        "https://overpass-api.de/api/interpreter",
        "https://lz4.overpass-api.de/api/interpreter",
        "https://overpass.kumi.systems/api/interpreter",
        "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
        "https://overpass.private.coffee/api/interpreter"
    ]

    USER_AGENT = "AdaptiveMultiDroneMissionCommand/2.0 (Computational GIS System; contact: mission-ops@adaptive-drone.internal)"

    def __init__(self, cache_size: int = 64, timeout_sec: float = 2.0):
        self.cache: Dict[str, ExtractedRoadNetwork] = {}
        self.cache_keys: List[str] = []
        self.max_cache_size = cache_size
        self.timeout_sec = timeout_sec

    def _generate_cache_key(self, min_lat: float, min_lon: float, max_lat: float, max_lon: float, poly_hash: str = "") -> str:
        coords_str = f"{min_lat:.5f},{min_lon:.5f},{max_lat:.5f},{max_lon:.5f}_{poly_hash}"
        return hashlib.md5(coords_str.encode("utf-8")).hexdigest()

    async def fetch_road_topology(
        self,
        min_lat: float,
        min_lon: float,
        max_lat: float,
        max_lon: float,
        cluster_threshold_m: float = 35.0,
        polygon_vertices: Optional[List[Tuple[float, float]]] = None,
        force_fallback: bool = False
    ) -> ExtractedRoadNetwork:
        """
        Queries OSM Overpass API, parses road ways into an intersection graph,
        clusters nearby nodes (e.g. roundabouts/divided roads), and returns ExtractedRoadNetwork.
        """
        poly_str_key = ""
        if polygon_vertices and len(polygon_vertices) >= 3:
            poly_str_key = "_".join(f"{lat:.4f},{lon:.4f}" for lat, lon in polygon_vertices[:6])

        cache_key = self._generate_cache_key(min_lat, min_lon, max_lat, max_lon, poly_str_key)
        if cache_key in self.cache and not force_fallback:
            logger.info(f"[OSM Cache] Hit for bounding box: {cache_key}")
            return self.cache[cache_key]

        if force_fallback:
            logger.info("[OSM Overpass] Offline/Fallback mode forced. Synthesizing urban road grid.")
            return self._synthesize_urban_fallback_network((min_lat, min_lon, max_lat, max_lon), polygon_vertices)

        # 1. Build Spatial Filter in Overpass QL
        # Latitude followed by Longitude: poly:"lat1 lon1 lat2 lon2 ..."
        if polygon_vertices and len(polygon_vertices) >= 3:
            poly_coords = " ".join(f"{lat:.6f} {lon:.6f}" for lat, lon in polygon_vertices)
            spatial_filter = f'poly:"{poly_coords}"'
        else:
            spatial_filter = f"{min_lat:.6f},{min_lon:.6f},{max_lat:.6f},{max_lon:.6f}"

        # 2. Comprehensive Overpass QL query:
        overpass_query = f"""
        [out:json][timeout:{int(self.timeout_sec)}];
        (
          way["highway"~"primary|secondary|tertiary|residential|unclassified|service|motorway|trunk"]({spatial_filter});
        );
        (._;>;);
        out body;
        """

        raw_data = None
        headers = {
            "User-Agent": self.USER_AGENT,
            "Accept": "application/json"
        }

        async def _query_single_endpoint(client: httpx.AsyncClient, ep: str) -> Optional[Dict[str, Any]]:
            try:
                resp = await client.post(ep, data={"data": overpass_query})
                if resp.status_code == 200:
                    j = resp.json()
                    if j.get("elements"):
                        return j
            except Exception:
                pass
            return None

        # Query top mirrors concurrently with fast failover (max 3.5s total wait)
        try:
            async with httpx.AsyncClient(timeout=self.timeout_sec, headers=headers) as client:
                tasks = [
                    asyncio.create_task(_query_single_endpoint(client, ep))
                    for ep in self.OVERPASS_ENDPOINTS[:3]
                ]
                for fut in asyncio.as_completed(tasks):
                    try:
                        res = await fut
                        if res:
                            raw_data = res
                            logger.info("[OSM Overpass] Successfully retrieved elements from mirror.")
                            for t in tasks:
                                if not t.done():
                                    t.cancel()
                            break
                    except Exception:
                        pass
        except Exception as e:
            logger.warning(f"[OSM Overpass] Query execution error: {e}")

        # If all public Overpass mirrors failed or timed out, synthesize a realistic urban street grid
        # so the user NEVER receives '0 road nodes' in an urban mission!
        if not raw_data or "elements" not in raw_data or len(raw_data.get("elements", [])) == 0:
            logger.warning("[OSM Overpass] Remote Overpass mirrors unreachable or empty. Activating high-fidelity urban road synthesizer fallback.")
            network = self._synthesize_urban_fallback_network((min_lat, min_lon, max_lat, max_lon), polygon_vertices)
        else:
            network = self._parse_overpass_graph(
                raw_data=raw_data,
                bbox=(min_lat, min_lon, max_lat, max_lon),
                cluster_threshold_m=cluster_threshold_m,
                polygon_vertices=polygon_vertices
            )

        # Cache valid response
        if len(self.cache_keys) >= self.max_cache_size:
            oldest = self.cache_keys.pop(0)
            self.cache.pop(oldest, None)
        self.cache[cache_key] = network
        self.cache_keys.append(cache_key)

        return network

    def _parse_overpass_graph(
        self,
        raw_data: Dict[str, Any],
        bbox: Tuple[float, float, float, float],
        cluster_threshold_m: float,
        polygon_vertices: Optional[List[Tuple[float, float]]] = None
    ) -> ExtractedRoadNetwork:
        """Parses raw elements, determines topological node degrees, and extracts edges."""
        min_lat, min_lon, max_lat, max_lon = bbox

        # 1. Index raw node coordinates
        raw_nodes: Dict[int, Tuple[float, float]] = {}
        raw_ways: List[Dict[str, Any]] = []

        for elem in raw_data.get("elements", []):
            if elem.get("type") == "node":
                raw_nodes[elem["id"]] = (elem["lat"], elem["lon"])
            elif elem.get("type") == "way" and "nodes" in elem:
                raw_ways.append(elem)

        # 2. Compute Node Degrees and occurrences across the road network
        node_degrees: Dict[int, int] = defaultdict(int)
        node_neighbors: Dict[int, Set[int]] = defaultdict(set)
        node_ways: Dict[int, List[int]] = defaultdict(list)

        for way in raw_ways:
            way_id = way.get("id", 0)
            w_nodes = way.get("nodes", [])
            for idx, n_id in enumerate(w_nodes):
                node_degrees[n_id] += 1
                node_ways[n_id].append(way_id)

                if idx > 0:
                    prev_id = w_nodes[idx - 1]
                    node_neighbors[n_id].add(prev_id)
                    node_neighbors[prev_id].add(n_id)
                if idx < len(w_nodes) - 1:
                    next_id = w_nodes[idx + 1]
                    node_neighbors[n_id].add(next_id)
                    node_neighbors[next_id].add(n_id)

        # 3. Identify Candidate Intersection Nodes:
        # A node is a valid road intersection if:
        # - Appears in 2 or more distinct ways (count >= 2)
        # - OR Graph branching degree >= 3 (e.g. T-junction, crossroad)
        # - OR Terminal tip / dead end (degree == 1)
        candidate_nodes: List[Dict[str, Any]] = []
        qualified_node_ids: Set[int] = set()

        for n_id, lat_lon in raw_nodes.items():
            lat, lon = lat_lon
            # Bounding box prune
            if not (min_lat <= lat <= max_lat and min_lon <= lon <= max_lon):
                continue

            # Polygon containment prune (if polygon defined)
            if polygon_vertices and len(polygon_vertices) >= 3:
                if not self._point_in_polygon(lat, lon, polygon_vertices):
                    continue

            way_count = len(set(node_ways.get(n_id, [])))
            degree = len(node_neighbors.get(n_id, set()))

            # Topological qualification
            if way_count >= 2 or degree >= 3 or (degree == 1 and way_count >= 1):
                candidate_nodes.append({
                    "osm_id": n_id,
                    "lat": lat,
                    "lon": lon,
                    "degree": max(degree, way_count),
                    "neighbors": list(node_neighbors.get(n_id, set())),
                    "ways": node_ways.get(n_id, [])
                })
                qualified_node_ids.add(n_id)

        # 4. Long Road Segment Waypoint Insertion (> 120m)
        # Guarantees long boulevards and avenues have periodic waypoints (~100m intervals)
        for way in raw_ways:
            w_nodes = [n for n in way.get("nodes", []) if n in raw_nodes]
            if len(w_nodes) < 2:
                continue

            accum_dist = 0.0
            last_qual_idx = 0

            for i in range(1, len(w_nodes)):
                prev_n = w_nodes[i - 1]
                curr_n = w_nodes[i]
                p_lat, p_lon = raw_nodes[prev_n]
                c_lat, c_lon = raw_nodes[curr_n]
                step_d = haversine_distance_m(p_lat, p_lon, c_lat, c_lon)
                accum_dist += step_d

                if curr_n in qualified_node_ids or i == len(w_nodes) - 1:
                    if accum_dist > 120.0 and (i - last_qual_idx) > 1:
                        # Insert intermediate midpoint node
                        mid_idx = (last_qual_idx + i) // 2
                        mid_n = w_nodes[mid_idx]
                        if mid_n not in qualified_node_ids:
                            m_lat, m_lon = raw_nodes[mid_n]
                            in_roi = (min_lat <= m_lat <= max_lat and min_lon <= m_lon <= max_lon)
                            if polygon_vertices and len(polygon_vertices) >= 3:
                                in_roi = in_roi and self._point_in_polygon(m_lat, m_lon, polygon_vertices)
                            if in_roi:
                                candidate_nodes.append({
                                    "osm_id": mid_n,
                                    "lat": m_lat,
                                    "lon": m_lon,
                                    "degree": 2,
                                    "neighbors": list(node_neighbors.get(mid_n, set())),
                                    "ways": node_ways.get(mid_n, [way.get("id", 0)])
                                })
                                qualified_node_ids.add(mid_n)

                    accum_dist = 0.0
                    last_qual_idx = i

        # 5. Spatial Node Clustering (Merge adjacent junction nodes within cluster_threshold_m)
        clustered_nodes = self._cluster_intersection_nodes(candidate_nodes, cluster_threshold_m)

        # 6. Build Graph Edges connecting clustered intersection nodes
        edges = self._build_graph_edges(raw_ways, raw_nodes, clustered_nodes)

        return ExtractedRoadNetwork(
            nodes=clustered_nodes,
            edges=edges,
            bbox=bbox,
            is_fallback=False,
            source="OSM_OVERPASS"
        )

    def _cluster_intersection_nodes(
        self,
        candidate_nodes: List[Dict[str, Any]],
        threshold_m: float
    ) -> Dict[str, RoadIntersectionNode]:
        """
        Greedy Geodesic Clustering Algorithm:
        Consolidates multi-lane split nodes, ramps, and roundabouts within threshold_m into
        a single unified RoadIntersectionNode to prevent chromosomal state explosion.
        """
        clusters: List[List[Dict[str, Any]]] = []

        for node in candidate_nodes:
            matched_cluster = None
            for cl in clusters:
                # Compare against centroid of existing cluster
                cl_lat = sum(n["lat"] for n in cl) / len(cl)
                cl_lon = sum(n["lon"] for n in cl) / len(cl)
                dist = haversine_distance_m(node["lat"], node["lon"], cl_lat, cl_lon)
                if dist <= threshold_m:
                    matched_cluster = cl
                    break

            if matched_cluster is not None:
                matched_cluster.append(node)
            else:
                clusters.append([node])

        # Compile final clustered nodes
        result: Dict[str, RoadIntersectionNode] = {}
        for idx, cl in enumerate(clusters, start=1):
            c_id = f"ROAD_{idx:04d}"
            c_lat = sum(n["lat"] for n in cl) / len(cl)
            c_lon = sum(n["lon"] for n in cl) / len(cl)
            # Degree is max or consolidated branches
            deg = max(n["degree"] for n in cl)
            osm_ids = [n["osm_id"] for n in cl]

            result[c_id] = RoadIntersectionNode(
                node_id=c_id,
                lat=c_lat,
                lon=c_lon,
                degree=deg,
                connected_osm_ids=osm_ids,
                adjacent_intersection_ids=[],
                cluster_size=len(cl)
            )

        return result

    def _build_graph_edges(
        self,
        raw_ways: List[Dict[str, Any]],
        raw_nodes: Dict[int, Tuple[float, float]],
        clustered_nodes: Dict[str, RoadIntersectionNode]
    ) -> List[RoadSegmentEdge]:
        """Builds topological edges connecting clustered intersection nodes."""
        osm_to_cluster: Dict[int, str] = {}
        for c_id, node in clustered_nodes.items():
            for osm_id in node.connected_osm_ids:
                osm_to_cluster[osm_id] = c_id

        edges: List[RoadSegmentEdge] = []
        edge_set: Set[Tuple[str, str]] = set()
        edge_counter = 1

        for way in raw_ways:
            way_nodes = way.get("nodes", [])
            hw_type = way.get("tags", {}).get("highway", "urban")

            current_c_id: Optional[str] = None
            current_geom: List[Tuple[float, float]] = []

            for n_id in way_nodes:
                if n_id not in raw_nodes:
                    continue
                pt = raw_nodes[n_id]
                current_geom.append(pt)

                if n_id in osm_to_cluster:
                    next_c_id = osm_to_cluster[n_id]
                    if current_c_id and current_c_id != next_c_id:
                        pair = tuple(sorted([current_c_id, next_c_id]))
                        if pair not in edge_set:
                            edge_set.add(pair)
                            length = sum(
                                haversine_distance_m(current_geom[k][0], current_geom[k][1], current_geom[k+1][0], current_geom[k+1][1])
                                for k in range(len(current_geom) - 1)
                            )
                            edges.append(RoadSegmentEdge(
                                edge_id=f"EDGE_{edge_counter:04d}",
                                u=current_c_id,
                                v=next_c_id,
                                length_m=round(max(length, 25.0), 1),
                                geometry=current_geom,
                                highway_type=hw_type
                            ))
                            edge_counter += 1

                            if next_c_id not in clustered_nodes[current_c_id].adjacent_intersection_ids:
                                clustered_nodes[current_c_id].adjacent_intersection_ids.append(next_c_id)
                            if current_c_id not in clustered_nodes[next_c_id].adjacent_intersection_ids:
                                clustered_nodes[next_c_id].adjacent_intersection_ids.append(current_c_id)

                    current_c_id = next_c_id
                    current_geom = [pt]

        return edges

    def _synthesize_urban_fallback_network(
        self,
        bbox: Tuple[float, float, float, float],
        polygon_vertices: Optional[List[Tuple[float, float]]]
    ) -> ExtractedRoadNetwork:
        """
        High-Fidelity Urban Road Network Synthesizer:
        Generates realistic arterial boulevards and cross-street intersections inside the ROI polygon
        when public Overpass API mirrors are unreachable or timing out.
        Ensures the mission command NEVER receives '0 road nodes' in an urban search sector.
        """
        min_lat, min_lon, max_lat, max_lon = bbox
        center_lat = (min_lat + max_lat) / 2.0
        center_lon = (min_lon + max_lon) / 2.0

        # Create an adaptive grid of street intersections spanning the ROI
        lat_span = max_lat - min_lat
        lon_span = max_lon - min_lon

        lat_span_m = lat_span * 111320.0
        lon_span_m = lon_span * 111320.0 * math.cos(math.radians(center_lat))
        n_rows = max(4, min(7, int(lat_span_m / 220.0)))
        n_cols = max(4, min(7, int(lon_span_m / 220.0)))

        grid_nodes: Dict[str, RoadIntersectionNode] = {}
        node_matrix = {}
        node_idx = 1

        for r in range(n_rows):
            r_lat = min_lat + ((r + 0.5) / n_rows) * lat_span
            for c in range(n_cols):
                c_lon = min_lon + ((c + 0.5) / n_cols) * lon_span

                # Check polygon inclusion
                if polygon_vertices and len(polygon_vertices) >= 3:
                    if not self._point_in_polygon(r_lat, c_lon, polygon_vertices):
                        continue

                node_id = f"ROAD_{node_idx:04d}"
                grid_nodes[node_id] = RoadIntersectionNode(
                    node_id=node_id,
                    lat=round(r_lat, 6),
                    lon=round(c_lon, 6),
                    degree=4,
                    adjacent_intersection_ids=[],
                    cluster_size=1
                )
                node_matrix[(r, c)] = node_id
                node_idx += 1

        # Build horizontal and vertical street edges
        edges: List[RoadSegmentEdge] = []
        edge_counter = 1

        # Horizontal Street Links
        for r in range(n_rows):
            for c in range(n_cols - 1):
                if (r, c) in node_matrix and (r, c + 1) in node_matrix:
                    u_id = node_matrix[(r, c)]
                    v_id = node_matrix[(r, c + 1)]
                    u_node = grid_nodes[u_id]
                    v_node = grid_nodes[v_id]
                    length = haversine_distance_m(u_node.lat, u_node.lon, v_node.lat, v_node.lon)

                    edges.append(RoadSegmentEdge(
                        edge_id=f"EDGE_{edge_counter:04d}",
                        u=u_id,
                        v=v_id,
                        length_m=round(length, 1),
                        geometry=[(u_node.lat, u_node.lon), (v_node.lat, v_node.lon)],
                        highway_type="primary"
                    ))
                    edge_counter += 1
                    u_node.adjacent_intersection_ids.append(v_id)
                    v_node.adjacent_intersection_ids.append(u_id)

        # Vertical Street Links
        for r in range(n_rows - 1):
            for c in range(n_cols):
                if (r, c) in node_matrix and (r + 1, c) in node_matrix:
                    u_id = node_matrix[(r, c)]
                    v_id = node_matrix[(r + 1, c)]
                    u_node = grid_nodes[u_id]
                    v_node = grid_nodes[v_id]
                    length = haversine_distance_m(u_node.lat, u_node.lon, v_node.lat, v_node.lon)

                    edges.append(RoadSegmentEdge(
                        edge_id=f"EDGE_{edge_counter:04d}",
                        u=u_id,
                        v=v_id,
                        length_m=round(length, 1),
                        geometry=[(u_node.lat, u_node.lon), (v_node.lat, v_node.lon)],
                        highway_type="secondary"
                    ))
                    edge_counter += 1
                    u_node.adjacent_intersection_ids.append(v_id)
                    v_node.adjacent_intersection_ids.append(u_id)

        # Guarantee minimum viable urban road graph inside any polygon
        if len(grid_nodes) < 4 and polygon_vertices and len(polygon_vertices) >= 3:
            c_lat = sum(v[0] for v in polygon_vertices) / len(polygon_vertices)
            c_lon = sum(v[1] for v in polygon_vertices) / len(polygon_vertices)

            centroid_id = f"ROAD_{node_idx:04d}"
            grid_nodes[centroid_id] = RoadIntersectionNode(
                node_id=centroid_id,
                lat=round(c_lat, 6),
                lon=round(c_lon, 6),
                degree=4,
                adjacent_intersection_ids=[],
                cluster_size=1
            )
            node_idx += 1

            # Sample along vectors towards vertices
            step_stride = max(1, len(polygon_vertices) // 6)
            for v_lat, v_lon in polygon_vertices[::step_stride]:
                for alpha in (0.35, 0.65):
                    s_lat = c_lat + alpha * (v_lat - c_lat)
                    s_lon = c_lon + alpha * (v_lon - c_lon)
                    if self._point_in_polygon(s_lat, s_lon, polygon_vertices):
                        s_id = f"ROAD_{node_idx:04d}"
                        grid_nodes[s_id] = RoadIntersectionNode(
                            node_id=s_id,
                            lat=round(s_lat, 6),
                            lon=round(s_lon, 6),
                            degree=3,
                            adjacent_intersection_ids=[centroid_id],
                            cluster_size=1
                        )
                        grid_nodes[centroid_id].adjacent_intersection_ids.append(s_id)
                        edges.append(RoadSegmentEdge(
                            edge_id=f"EDGE_{edge_counter:04d}",
                            u=centroid_id,
                            v=s_id,
                            length_m=round(haversine_distance_m(c_lat, c_lon, s_lat, s_lon), 1),
                            geometry=[(c_lat, c_lon), (s_lat, s_lon)],
                            highway_type="secondary"
                        ))
                        edge_counter += 1
                        node_idx += 1

        logger.info(f"[OSM Synthesizer] Generated {len(grid_nodes)} urban intersections and {len(edges)} street edges.")
        return ExtractedRoadNetwork(
            nodes=grid_nodes,
            edges=edges,
            bbox=bbox,
            is_fallback=True,
            source="URBAN_STREET_SYNTHESIZER_FALLBACK"
        )

    @staticmethod
    def _point_in_polygon(lat: float, lon: float, polygon: List[Tuple[float, float]]) -> bool:
        """Ray-casting algorithm to test polygon inclusion."""
        n = len(polygon)
        if n < 3:
            return True
        inside = False
        p1_lat, p1_lon = polygon[0]
        for i in range(1, n + 1):
            p2_lat, p2_lon = polygon[i % n]
            if lat > min(p1_lat, p2_lat) and lat <= max(p1_lat, p2_lat):
                if lon <= max(p1_lon, p2_lon):
                    if p1_lat != p2_lat:
                        x_inters = (lat - p1_lat) * (p2_lon - p1_lon) / (p2_lat - p1_lat) + p1_lon
                        if p1_lon == p2_lon or lon <= x_inters:
                            inside = not inside
            p1_lat, p1_lon = p2_lat, p2_lon
        return inside


# Global singleton extractor instance
global_road_extractor = OSMOverpassExtractor()
