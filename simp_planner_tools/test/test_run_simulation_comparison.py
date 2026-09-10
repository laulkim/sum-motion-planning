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
    assert "tracking_enabled:=true" in command


def test_noisy_off_only_changes_tracking_flag(tmp_path: Path) -> None:
    on = build_launch_command("stadium", 4.0, "noisy", 10.0, tmp_path, True)
    off = build_launch_command("stadium", 4.0, "noisy", 10.0, tmp_path, False)
    assert on[:-1] == off[:-1]
    assert off[-1] == "tracking_enabled:=false"


def test_comparison_runs_three_variants_and_retains_original_pair(tmp_path, monkeypatch):
    from simp_planner_tools import run_simulation_comparison as runner

    launches = []
    comparisons = []
    monkeypatch.setattr(runner, "run_for_duration", lambda command, duration: launches.append((command, duration)))
    monkeypatch.setattr(runner, "find_run_directory", lambda root, scenario: root / scenario / "session")
    monkeypatch.setattr(runner, "compare_runs", lambda *args, **kwargs: comparisons.append((args, kwargs)))
    ideal, noisy, output = runner.run_comparison("stadium", 4.0, 100.0, tmp_path, 10.0, .01, True)
    assert len(launches) == 3
    assert ["tracking_enabled:=false" in command for command, _ in launches] == [False, False, True]
    assert all(duration == 100.0 for _, duration in launches)
    assert comparisons[0][0] == (ideal, noisy, output)
    assert comparisons[0][1]["show"] is True
    assert "noisy_off" in str(comparisons[0][1]["noisy_off_directory"])


def test_batch_and_run_directories_are_unambiguous(tmp_path: Path) -> None:
    batch = create_batch_directory(tmp_path, "stadium")
    model_root = batch / "raw" / "ideal"
    session = model_root / "stadium" / "20260908_140855"
    session.mkdir(parents=True)
    (session / "odom_history.csv").touch()
    (session / "command_history.csv").touch()

    assert find_run_directory(model_root, "stadium") == session
