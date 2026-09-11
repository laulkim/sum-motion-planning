import ast
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "simp_planner_tools"
    / "scenario_manager_node.py"
)


def _class_method(tree: ast.Module, class_name: str, method_name: str) -> ast.FunctionDef:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == method_name:
                    return item
    raise AssertionError(f"Missing {class_name}.{method_name}")


def _self_attributes(node: ast.AST, context_type: type[ast.expr_context]) -> set[str]:
    result: set[str] = set()
    for item in ast.walk(node):
        if (
            isinstance(item, ast.Attribute)
            and isinstance(item.value, ast.Name)
            and item.value.id == "self"
            and isinstance(item.ctx, context_type)
        ):
            result.add(item.attr)
    return result


def test_status_fields_are_initialized_before_constructor_publishes_status() -> None:
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    init = _class_method(tree, "ScenarioManagerNode", "__init__")
    status = _class_method(tree, "ScenarioManagerNode", "publish_status")

    assigned = _self_attributes(init, ast.Store)
    status_reads = _self_attributes(status, ast.Load)

    class_node = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "ScenarioManagerNode"
    )
    class_members = {
        item.name for item in class_node.body
        if isinstance(item, ast.FunctionDef)
    }
    required_state = status_reads - class_members

    missing = sorted(required_state - assigned)
    assert not missing, f"publish_status reads uninitialized state: {missing}"


def test_costmap_uses_one_odom_snapshot_without_path_extent() -> None:
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    publish_costmap = _class_method(
        tree, "ScenarioManagerNode", "publish_costmap"
    )
    source = ast.unparse(publish_costmap)

    assert "snapshot = self.costmap_odom_snapshot" in source
    assert "rasterize_vehicle_costmap" in source
    assert "self.last_projection" not in source
    assert "local_slice" not in source
    assert "message.header.stamp.sec = snapshot.stamp_sec" in source
    assert "message.header.stamp.nanosec = snapshot.stamp_nanosec" in source
    assert "message.info.map_load_time.sec = snapshot.stamp_sec" in source
    assert "message.info.map_load_time.nanosec = snapshot.stamp_nanosec" in source


def test_odom_callback_captures_pose_and_source_stamp_together() -> None:
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    odom_callback = _class_method(tree, "ScenarioManagerNode", "odom_callback")
    source = ast.unparse(odom_callback)

    assert "self.costmap_odom_snapshot = OdomPoseSnapshot" in source
    assert "stamp_sec=int(message.header.stamp.sec)" in source
    assert "stamp_nanosec=int(message.header.stamp.nanosec)" in source
    assert "body_yaw=body_yaw" in source
