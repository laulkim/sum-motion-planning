from pathlib import Path

from simp_planner_tools.run_simulation_comparison import (
    build_launch_command,
    create_batch_directory,
    find_run_directory,
)


def test_launch_command_selects_model_and_output_directory(tmp_path: Path) -> None:
    command = build_launch_command(
        scenario="stadium",
        target_speed=4.0,
        model="noisy",
        save_period=10.0,
        debug_output_root=tmp_path,
    )

    assert command[:4] == [
        "ros2",
        "launch",
        "simp_planner_tools",
        "simulation.launch.py",
    ]
    assert "scenario:=stadium" in command
    assert "target_speed:=4.0" in command
    assert "kinematics_model:=noisy" in command
    assert f"debug_output_dir:={tmp_path}" in command


def test_batch_and_run_directories_are_unambiguous(tmp_path: Path) -> None:
    batch = create_batch_directory(tmp_path, "stadium")
    model_root = batch / "raw" / "ideal"
    session = model_root / "stadium" / "20260908_140855"
    session.mkdir(parents=True)
    (session / "odom_history.csv").touch()
    (session / "command_history.csv").touch()

    assert find_run_directory(model_root, "stadium") == session
