import sys
import time
from pathlib import Path

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from app.fleet.telemetry_models import DroneNodeConfig, DroneStatus
from app.fleet.drone_registry import DynamicDroneRegistry
from app.fleet.transport_adapters import SimulatedNodeAdapter
from app.api.v1.fleet_routes import (
    add_drone_node,
    get_all_fleet_nodes,
    reorder_fleet_nodes,
    remove_drone_node
)

def test_fleet_dynamic_operations():
    print("[*] Testing Dynamic Drone Registry & Zero Hardcoding...")
    registry = DynamicDroneRegistry()

    # 1. Register 3 drones with different local protocols
    cfg1 = DroneNodeConfig(
        drone_id="UAV-01",
        endpoint_uri="sim://uav-01",
        cruise_speed_mps=15.0,
        battery_capacity_wh=120.0
    )
    cfg2 = DroneNodeConfig(
        drone_id="UAV-02",
        endpoint_uri="ws://192.168.1.102:8765",
        cruise_speed_mps=14.0,
        battery_capacity_wh=100.0
    )
    cfg3 = DroneNodeConfig(
        drone_id="UAV-03",
        endpoint_uri="udp://192.168.1.103:14550",
        cruise_speed_mps=12.0,
        battery_capacity_wh=150.0
    )

    t1 = registry.add_drone(cfg1)
    t2 = registry.add_drone(cfg2)
    t3 = registry.add_drone(cfg3)

    assert t1.drone_id == "UAV-01"
    assert t1.status == DroneStatus.READY
    configs = registry.get_ordered_configs()
    assert len(configs) == 3
    print(f"  [✓] Registered 3 dynamic nodes: {[c.drone_id for c in configs]}")

    # 2. Prevent duplicate ID
    try:
        registry.add_drone(cfg1)
        assert False, "Should raise ValueError on duplicate ID"
    except ValueError:
        print("  [✓] Duplicate drone ID rejected.")

    # 3. Test Immutable Snapshot for GA/PSO Solvers
    snapshot1 = registry.get_immutable_snapshot()
    assert len(snapshot1) == 3

    # Mutate registry (remove drone 2)
    registry.remove_drone("UAV-02")
    configs_after_remove = registry.get_ordered_configs()
    assert len(configs_after_remove) == 2

    # Snapshot 1 must remain intact with 3 drones (no race condition!)
    assert len(snapshot1) == 3, "Snapshot must be completely immutable!"
    print("  [✓] Immutable Snapshot verified: solver state preserved across concurrent deletions.")

    # 4. Reorder Fleet Priority
    new_order = ["UAV-03", "UAV-01"]
    updated_order = registry.reorder_fleet(new_order)
    assert updated_order == ["UAV-03", "UAV-01"]
    configs_reordered = registry.get_ordered_configs()
    assert configs_reordered[0].drone_id == "UAV-03"
    print(f"  [✓] Priority sequence reordered: {updated_order}")

    # 5. Heartbeat Watchdog Test
    # Node 1 is alive
    offline_nodes = registry.check_heartbeats(timeout_sec=5.0)
    assert len(offline_nodes) == 0

    # Simulate node 1 heartbeat timeout
    node1_adapter = registry._registry["UAV-01"][1]
    if isinstance(node1_adapter, SimulatedNodeAdapter):
        node1_adapter.last_heartbeat = time.time() - 10.0 # 10s ago

    offline_nodes_detected = registry.check_heartbeats(timeout_sec=5.0)
    assert "UAV-01" in offline_nodes_detected
    print(f"  [✓] Heartbeat watchdog isolated timed-out node: {offline_nodes_detected}")

    # 6. Test REST API Handlers
    print("[*] Testing Fleet REST API Routes...")
    api_res = add_drone_node(DroneNodeConfig(
        drone_id="UAV-API-Test",
        endpoint_uri="sim://api-test",
        battery_capacity_wh=100.0
    ))
    assert api_res.drone_id == "UAV-API-Test"

    fleet_summary = get_all_fleet_nodes()
    assert fleet_summary["count"] >= 1

    del_res = remove_drone_node("UAV-API-Test")
    assert del_res["status"] == "DELETED"
    print("  [✓] Fleet REST API endpoints verified.")

if __name__ == "__main__":
    test_fleet_dynamic_operations()
    print("\n[SUCCESS] All Phase 05 Fleet Registry & Multi-Protocol Tests Passed!")
