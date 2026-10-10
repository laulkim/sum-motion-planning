"""CSV LUT utilities for initialization, active blocks and regularization.

Same row order as the controller: vx outermost, yaw_rate innermost.
Axis sizes and locations are supplied by the experimenter, never fixed here.
"""
import bisect
import csv
import itertools
import math
from pathlib import Path

from .gain_evaluation import GAIN_NAMES, positive

AXIS_NAMES = ("vx", "vy", "yaw_rate")
HEADER = (*AXIS_NAMES, *GAIN_NAMES)


def validate_axes(axes):
    for name in AXIS_NAMES:
        values = axes[name]
        if not values or any(not math.isfinite(v) for v in values):
            raise ValueError(f"{name}: finite nonempty axis required")
        if any(b <= a for a, b in zip(values, values[1:])):
            raise ValueError(f"{name}: knots must be strictly increasing")
    return axes


def load_lut(path):
    with Path(path).open() as file:
        reader = csv.reader(file)
        if next(reader) != list(HEADER):
            raise ValueError("LUT header must be vx,vy,yaw_rate,kx,ky,ktheta")
        rows = [list(map(float, row)) for row in reader]
    if not rows or any(len(row) != 6 or not all(math.isfinite(v) for v in row) for row in rows):
        raise ValueError("LUT must contain finite six-column rows")
    axes = validate_axes({name: sorted({row[i] for row in rows}) for i, name in enumerate(AXIS_NAMES)})
    coordinates = list(itertools.product(*(axes[name] for name in AXIS_NAMES)))
    if [tuple(row[:3]) for row in rows] != coordinates:
        raise ValueError("CSV must be a complete ordered Cartesian grid (vx,vy,yaw_rate)")
    gains = [row[3:] for row in rows]
    for gain in gains:
        for name, value in zip(GAIN_NAMES, gain):
            positive(value, name, zero=name != "ktheta")
    return axes, gains


def save_lut(path, axes, gains):
    validate_axes(axes)
    coordinates = list(itertools.product(*(axes[name] for name in AXIS_NAMES)))
    if len(coordinates) != len(gains):
        raise ValueError("wrong LUT point count")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    with temporary.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(HEADER)
        writer.writerows((*p, *g) for p, g in zip(coordinates, gains))
    temporary.replace(path)


def axis_weights(axis, value):
    if not math.isfinite(value):
        raise ValueError("nonfinite LUT query")
    value = min(max(value, axis[0]), axis[-1])
    hi = bisect.bisect_left(axis, value)
    if axis[hi] == value:
        return [(hi, 1.0)]
    lo = hi - 1
    fraction = (value - axis[lo]) / (axis[hi] - axis[lo])
    return [(lo, 1 - fraction), (hi, fraction)]


def active_points(axes, samples_path):
    active = set()
    sizes = [len(axes[name]) for name in AXIS_NAMES]
    with Path(samples_path).open() as file:
        for row in csv.DictReader(file):
            neighbors = [axis_weights(axes[name], float(row[name])) for name in AXIS_NAMES]
            for (i, wi), (j, wj), (k, wk) in itertools.product(*neighbors):
                if wi * wj * wk > 0:
                    active.add((i * sizes[1] + j) * sizes[2] + k)
    if not active:
        raise ValueError("no active interpolation neighborhoods")
    return sorted(active)


def smoothness(axes, gains, coefficient):
    positive(coefficient, "smoothness_lambda", zero=True)
    sizes = [len(axes[name]) for name in AXIS_NAMES]
    total = 0.0
    for i, j, k in itertools.product(*(range(n) for n in sizes)):
        index = (i * sizes[1] + j) * sizes[2] + k
        for direction, stride in enumerate((sizes[1] * sizes[2], sizes[2], 1)):
            if (i, j, k)[direction] + 1 < sizes[direction]:
                total += sum((a - b) ** 2 for a, b in zip(gains[index], gains[index + stride]))
    return coefficient * total
