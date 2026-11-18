import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_health_check():
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"

def test_valid_rectangle_roi():
    payload = {
        "roi_type": "RECTANGLE",
        "bounds": {
            "lat_min": 33.3000,
            "lat_max": 33.3250,
            "lon_min": 44.3500,
            "lon_max": 44.3800
        },
        "vertices": [],
        "area_km2": 7.68
    }
    response = client.post("/api/v1/map/roi", json=payload)
    assert response.status_code == 201
    data = response.json()
    assert data["status"] == "VALIDATED_AND_ACTIVE"
    assert data["roi_type"] == "RECTANGLE"
    assert data["vertices_count"] == 4
    assert data["estimated_50m_cells"] == int(7.68 / 0.0025)

def test_valid_polygon_roi():
    payload = {
        "roi_type": "POLYGON",
        "bounds": {
            "lat_min": 33.3000,
            "lat_max": 33.3500,
            "lon_min": 44.3500,
            "lon_max": 44.4000
        },
        "vertices": [
            {"lat": 33.3000, "lon": 44.3500},
            {"lat": 33.3500, "lon": 44.3600},
            {"lat": 33.3300, "lon": 44.4000}
        ],
        "area_km2": 12.4
    }
    response = client.post("/api/v1/map/roi", json=payload)
    assert response.status_code == 201
    data = response.json()
    assert data["vertices_count"] == 3
    assert data["estimated_50m_cells"] == int(12.4 / 0.0025)

def test_invalid_polygon_too_few_vertices():
    payload = {
        "roi_type": "POLYGON",
        "bounds": {
            "lat_min": 33.30,
            "lat_max": 33.35,
            "lon_min": 44.35,
            "lon_max": 44.40
        },
        "vertices": [
            {"lat": 33.30, "lon": 44.35},
            {"lat": 33.35, "lon": 44.40}
        ],
        "area_km2": 5.0
    }
    response = client.post("/api/v1/map/roi", json=payload)
    assert response.status_code == 422  # Unprocessable Entity (Pydantic validation failure)

def test_invalid_bounds_inverted():
    payload = {
        "roi_type": "RECTANGLE",
        "bounds": {
            "lat_min": 33.50, # Inverted: min > max
            "lat_max": 33.20,
            "lon_min": 44.35,
            "lon_max": 44.40
        },
        "vertices": [],
        "area_km2": 2.0
    }
    response = client.post("/api/v1/map/roi", json=payload)
    assert response.status_code == 422

def test_excessive_area_rejection():
    payload = {
        "roi_type": "RECTANGLE",
        "bounds": {
            "lat_min": 30.0,
            "lat_max": 40.0,
            "lon_min": 40.0,
            "lon_max": 50.0
        },
        "vertices": [],
        "area_km2": 850.0 # Exceeds 500 km2 max drone mission capacity
    }
    response = client.post("/api/v1/map/roi", json=payload)
    assert response.status_code == 422

def test_get_and_clear_roi():
    # First set
    payload = {
        "roi_type": "RECTANGLE",
        "bounds": {
            "lat_min": 33.30,
            "lat_max": 33.35,
            "lon_min": 44.35,
            "lon_max": 44.40
        },
        "vertices": [],
        "area_km2": 4.0
    }
    client.post("/api/v1/map/roi", json=payload)
    get_res = client.get("/api/v1/map/roi")
    assert get_res.status_code == 200
    assert get_res.json()["area_km2"] == 4.0

    # Clear
    del_res = client.delete("/api/v1/map/roi")
    assert del_res.status_code == 200

    # Check 404 after clear
    get_res2 = client.get("/api/v1/map/roi")
    assert get_res2.status_code == 404
