"""Keyframe interpolation engine (linear + cubic spline)."""

from typing import Dict, List, Tuple

import numpy as np


class InterpolationEngine:
    """Interpolates joint positions between user-defined keyframes."""

    @staticmethod
    def interpolate_joint(
        keyframes: List[Tuple[int, float, float]],
        mode: str = "linear",
    ) -> Dict[int, Tuple[float, float]]:
        """Compute interpolated (u, v) for every frame between keyframes.

        Args:
            keyframes: sorted list of (frame_idx, u, v).
            mode: 'linear' (numpy interp) or 'cubic' (scipy CubicSpline).

        Returns:
            dict mapping frame_idx -> (u, v) for *non-keyframe* intermediate
            frames only.
        """
        if len(keyframes) < 2:
            return {}

        keyframes = sorted(keyframes, key=lambda k: k[0])
        frames = np.array([k[0] for k in keyframes], dtype=np.float64)
        us = np.array([k[1] for k in keyframes], dtype=np.float64)
        vs = np.array([k[2] for k in keyframes], dtype=np.float64)

        all_frames = np.arange(int(frames[0]), int(frames[-1]) + 1)

        if mode == "cubic":
            from scipy.interpolate import CubicSpline

            cs_u = CubicSpline(frames, us, bc_type="clamped")
            cs_v = CubicSpline(frames, vs, bc_type="clamped")
            interp_u = cs_u(all_frames)
            interp_v = cs_v(all_frames)
        else:  # linear
            interp_u = np.interp(all_frames, frames, us)
            interp_v = np.interp(all_frames, frames, vs)

        kf_set = set(int(f) for f in frames)
        result: Dict[int, Tuple[float, float]] = {}
        for i, f in enumerate(all_frames):
            f_int = int(f)
            if f_int not in kf_set:
                result[f_int] = (float(interp_u[i]), float(interp_v[i]))
        return result
