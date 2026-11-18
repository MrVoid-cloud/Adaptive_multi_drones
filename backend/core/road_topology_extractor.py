"""
Re-export module for backend/core/road_topology_extractor.py
"""
from app.gis.road_topology_extractor import (
    RoadIntersectionNode,
    RoadSegmentEdge,
    ExtractedRoadNetwork,
    OSMOverpassExtractor,
    global_road_extractor,
    haversine_distance_m
)

__all__ = [
    "RoadIntersectionNode",
    "RoadSegmentEdge",
    "ExtractedRoadNetwork",
    "OSMOverpassExtractor",
    "global_road_extractor",
    "haversine_distance_m"
]
