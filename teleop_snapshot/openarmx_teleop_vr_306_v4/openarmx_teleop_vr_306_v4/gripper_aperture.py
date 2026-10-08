"""Nominal aperture; empirical linkage calibration can replace this mapping.

URDF display endpoints: motor 10=open (95 mm), motor 360=closed.
The motor/linkage nonlinearity has not been measured on robot 283.
"""
import json
import math
import os
from pathlib import Path

STATE_PATH = Path.home() / '.config/openarmx_vr/gripper_aperture.json'


def checked_width(value):
    if isinstance(value, bool):
        raise ValueError('闭合宽度必须是0–9cm的数值')
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= 9.0:
        raise ValueError('闭合宽度必须在0–9cm之间')
    return value


def width_to_motor(value):
    return 360.0 - checked_width(value) / 9.5 * 350.0


def load_width(path=STATE_PATH):
    if not path.exists():
        return 0.0
    return checked_width(json.loads(path.read_text())['closed_width_cm'])


def save_width(value, path=STATE_PATH):
    value = checked_width(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps({'closed_width_cm': value, 'mapping': 'nominal_95mm_v1'}))
    os.replace(tmp, path)
