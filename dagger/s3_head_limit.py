"""Optional S3 head bound, applied to physical chunks before proposal hashing."""
from dataclasses import replace
import math
import numpy as np


class HeadPitchLimitedGenerator:
    def __init__(self, generator, upper_degrees):
        upper_degrees = float(upper_degrees)
        if not math.isfinite(upper_degrees) or not 0 < upper_degrees <= 34.5:
            raise ValueError('S3 head pitch upper bound must be in (0, 34.5] degrees')
        self.generator = generator
        self.upper_degrees = upper_degrees

    def __getattr__(self, name):
        return getattr(self.generator, name)

    def generate_candidates(self, *args, **kwargs):
        batch = self.generator.generate_candidates(*args, **kwargs)
        actions = np.asarray(batch.physical_actions)
        if (actions.ndim != 3 or actions.shape[-1] != 21
                or actions.dtype != np.float32 or not np.isfinite(actions).all()):
            raise ValueError('Head bound requires finite float32 [candidate, time, 21] physical actions')
        physical = actions.copy()
        # 16/17/18 = neck roll/pitch/yaw. Arms, grippers and waist are unchanged.
        physical[..., 17] = np.minimum(physical[..., 17], self.upper_degrees)
        return replace(batch, physical_actions=physical)
