"""Small regression for the connection to the existing ROS plant models."""
import math
import csv
import json
from pathlib import Path

import pytest

from planar_velocity_sim.kinematics import integrate_body_velocity
from planar_velocity_sim.vehicle_response import AR2Noise, AR2_NOISE_DEFAULTS, FirstOrderResponse, RESPONSE_PARAMETER_DEFAULTS
from simp_planner_tools.offline_gain_sim import replay, load_references, response_parameters
from simp_planner_tools.gain_evaluation import load_manifest
from simp_planner_tools.gain_lut import lookup

SETTINGS = dict(sensor_delay_sec=0.0, sensor_noise_enabled=False, sensor_noise_scale=1.0,
                mode_transition_duration_sec=2.0)


def reference(t, **values):
    return dict(valid=1, xr=0.0, yr=0.0, yaw_r=0.0, vx_ref=1.0, vy_ref=0.0, omega_ref=0.0,
                requested_mode=0, control_stamp_ns=round(t * 1e9), reference_stamp_ns=round(t * 1e9), **values)


def test_replay_connects_identified_response_pose_noise_prediction_and_mode_gate(tmp_path):
    rows = [reference(i * 0.01) for i in range(8)]
    history = replay(rows, [1, 2, 1], 42, SETTINGS)
    plant = FirstOrderResponse(RESPONSE_PARAMETER_DEFAULTS['vx_gain'], RESPONSE_PARAMETER_DEFAULTS['vx_tau_sec'],
                               RESPONSE_PARAMETER_DEFAULTS['vx_delay_sec'], 0.0)
    x = 0.0
    for i, row in enumerate(history):
        now = i * 0.01
        actual = plant.update(now)
        if i:
            x, _, _ = integrate_body_velocity(x, 0, 0, actual, 0, 0, 0.01)
        assert row['x'] == pytest.approx(x, abs=1e-12)
        assert row['vx_actual'] == pytest.approx(actual, abs=1e-12)
        assert row['ex'] == pytest.approx(-x, abs=1e-12)
        assert row['vx_cmd'] == pytest.approx(1 - x, abs=1e-12)
        plant.command(1 - x, now)
    assert [row['vx_actual'] for row in history[:3]] == [0, 0, 0]  # actuator dead time survives sensor delay=0
    assert history[3]['vx_actual'] > 0
    noisy = replay(rows, [1, 2, 1], 42, {**SETTINGS, 'sensor_noise_enabled': True})
    assert noisy == replay(rows, [1, 2, 1], 42, {**SETTINGS, 'sensor_noise_enabled': True})
    a1, a2, sigma = AR2_NOISE_DEFAULTS['vx']
    assert noisy[1]['vx'] - noisy[1]['vx_actual'] == pytest.approx(AR2Noise(a1, a2, sigma, 42).sample())
    assert [(r['x'], r['vx_actual']) for r in noisy] == [(r['x'], r['vx_actual']) for r in history]
    ahead = [dict(row, reference_stamp_ns=row['control_stamp_ns'] + 5000000) for row in rows]
    predicted = replay(ahead, [1, 2, 1], 42, SETTINGS)
    assert predicted[3]['predicted_x'] == pytest.approx(predicted[3]['x'] + 0.005 * predicted[3]['vx'])
    modes = [reference(i * 0.01) for i in range(207)]
    for i, row in enumerate(modes):
        row.update(vx_ref=0.0, vy_ref=1.0 if i else 0.0, requested_mode=2 if i else 0)
    transition = replay(modes, [1, 2, 1], 42, SETTINGS)
    assert all(row['vy_actual'] == 0 for row in transition[:202])
    assert transition[202]['vy_actual'] > 0  # 2 s ALIGNING gates input, then identified lateral response
    with pytest.raises(ValueError, match='sensor_delay'):
        replay(rows, [1, 2, 1], 42, {**SETTINGS, 'sensor_delay_sec': 0.1})
    config = load_manifest(Path(__file__).resolve().parents[1] / 'experiments/gain_smoke.json')
    source = tmp_path / 'tracking.csv'
    with source.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    summary = tmp_path / 'summary.json'
    summary.write_text(json.dumps(dict(role='tuning', success=True, sensor_delay_sec=0.0,
        configuration=dict(config=config, response_parameters=response_parameters()),
        results=[dict(case=config['cases'][0], tracking_csv=str(source))])))
    datasets, identity = load_references(summary, config)
    assert datasets[0][1] == rows
    assert config['cases'][0]['id'] in identity['csv_sha256']
    source.write_text(source.read_text().replace(',requested_mode,', ',old_missing_mode,'))
    with pytest.raises(ValueError, match='recollect CSV'):
        load_references(summary, config)


def test_lut_uses_existing_weights_and_exact_spot_constraint_after_lookup():
    axes = {'vx': [-2, 0, 4], 'vy': [0, 2], 'yaw_rate': [-1, 0, 1]}
    values = [[1 + vx * 0.1, 2 + vy, 3 + omega] for vx in axes['vx'] for vy in axes['vy'] for omega in axes['yaw_rate']]
    assert lookup(axes, values, (2, 1, 0.5)) == pytest.approx([1.2, 3, 3.5])
    assert lookup(axes, values, (100, -100, 100)) == pytest.approx([1.4, 2, 4])
    rows = [reference(i * 0.01) for i in range(10)]
    rows[5].update(vx_ref=0, vy_ref=0, omega_ref=0.5)
    rows[6].update(vx_ref=0, vy_ref=0, omega_ref=0)
    history = replay(rows, [1, 2, 1], 42, SETTINGS, (axes, values))
    assert history[5]['vx_cmd'] == history[5]['vy_cmd'] == 0
    assert history[5]['omega_cmd'] == pytest.approx(0.5 + 3.5 * history[5]['etheta'])
    assert history[6]['vx_cmd'] != 0  # complete stop still corrects position error
    assert all(math.isfinite(r['x']) for r in history)
