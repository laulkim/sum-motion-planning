from planar_velocity_sim.mode_transition import DriveModeTransitionModel


def test_mode_change_requires_standstill_and_feedback_completion():
    model = DriveModeTransitionModel(
        initial_mode=0, transition_duration_sec=2.0, stop_speed_threshold=0.03
    )
    assert not model.command(2, measured_speed=0.2, now_sec=0.0)
    assert model.current_mode == 0
    assert model.command(2, measured_speed=0.0, now_sec=1.0)
    assert model.transition_in_progress
    assert model.applied_velocity(1.0, 2.0, 0.5) == (0.0, 0.0, 0.0)
    assert not model.update(2.99)
    assert model.current_mode == 0
    assert model.update(3.0)
    feedback = model.feedback()
    assert feedback.current_mode == 2
    assert feedback.transition_complete
    assert not feedback.transition_in_progress


def test_same_mode_command_completes_immediately():
    model = DriveModeTransitionModel(initial_mode=1)
    assert model.command(1, measured_speed=0.0, now_sec=0.0)
    assert model.feedback().transition_complete
