# Planar velocity simulator

Data flow:

`/cmd_vel` body-frame reference → empirical closed-loop response → actual
`vx, vy, yaw_rate` → ground-truth pose integration → sensor delay → AR(2)
velocity measurement noise → `/odom`.

The identified response includes the lower velocity controller, actuator and
vehicle. It is not an open-loop mass/inertia or tire model. Each channel uses
`tau * dy/dt + y = gain * u(t - delay)` with exact
exponential discretization. Commands are held from their receipt timestamps;
updates split at delayed input changes, including changes between timer ticks.
Ground-truth pose uses the resulting actual velocity and existing midpoint
kinematics at the simulator update rate.

## Parameters

These startup ROS parameters are also exposed as arguments in
`simp_planner_tools/launch/simulation.launch.py`.

| Channel | Gain parameter (default) | Time constant in seconds | Dead time in seconds |
| --- | --- | --- | --- |
| vx | `vx_gain` (1.00073) | `vx_tau_sec` (0.00939) | `vx_delay_sec` (0.020) |
| vy | `vy_gain` (0.94555) | `vy_tau_sec` (0.02441) | `vy_delay_sec` (0.000) |
| yaw rate | `yaw_rate_gain` (0.93197) | `yaw_rate_tau_sec` (0.00527) | `yaw_rate_delay_sec` (0.020) |

`sensor_delay_sec` defaults to 0.100. Time constants must be positive, delays
non-negative, and all response parameters finite. Defaults are centralized in
`vehicle_response.py`.

The sensor buffer holds timestamped ground-truth pose and body velocity
samples. `/odom` publishes the newest sample whose age is at least the sensor
delay, preserving its acquisition timestamp in `header.stamp`. There is no
publication until the initial delay window has elapsed. Sampling/holding can
add up to one simulation update interval to the age, rather than rounding the
configured delay to a fixed number of samples. A zero sensor delay publishes
the current sample. The node's `x, y, yaw` and `applied_*` fields remain current
ground truth; odometry is the delayed observation.

`sensor_noise_enabled` (true), `sensor_noise_scale` (1.0, finite and non-negative)
and `sensor_noise_seed` (42) configure zero-mean AR(2) noise added only to delayed
`/odom` velocity. Pose, actual velocity and mode acceptance are unaffected directly.
The tracker receives noisy velocity, so its feedback can change subsequent commands.
Each published sample advances `n = a1*n_prev1 + a2*n_prev2 + w` (normally 100 Hz).
The scale multiplies all innovation standard deviations; zero disables noise.
Channel RNGs use seed, seed+1, seed+2 and correlated stationary initial states.
The fixed residual fits are in `vehicle_response.py` (yaw innovation is rad/s).
The same three settings are launch arguments, e.g. `sensor_noise_enabled:=false`.

Drive-mode acceptance uses actual translational speed, not delayed odometry.
ALIGNING sets the response input reference to zero without resetting actual
velocity or clearing delayed commands. Commands already in flight can still
arrive, then actual velocity decays according to the channel dead time and time
constant. Residual motion continues to integrate ground-truth pose and appears
in delayed odometry. References received while ALIGNING are retained as the
latest command. On completion they resume through the same continuous response
state and channel dead time. Mode feedback is not subject to GNSS sensor delay.

Example:

```bash
ros2 launch simp_planner_tools simulation.launch.py sensor_delay_sec:=0.100 vx_delay_sec:=0.020
```

Run the tests after sourcing the ROS workspace:

```bash
PYTHONPATH="$PWD/planar_velocity_sim:$PYTHONPATH" python3 -m pytest planar_velocity_sim/test -q
```
