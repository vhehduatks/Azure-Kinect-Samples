"""IK-aware back-projection: ray-sphere intersection + FABRIK solver.

Pure functions with no Qt/model dependencies.
"""

import math
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Skeleton topology -- child -> parent mapping derived from BONE_CONNECTIONS
# ---------------------------------------------------------------------------
PARENT_JOINT: Dict[int, int] = {
    # Spine
    1: 0, 2: 1, 3: 2, 26: 3,
    # Head / face
    27: 26, 28: 27, 29: 28, 30: 27, 31: 30,
    # Left arm
    4: 2, 5: 4, 6: 5, 7: 6, 8: 7, 9: 8, 10: 7,
    # Right arm
    11: 2, 12: 11, 13: 12, 14: 13, 15: 14, 16: 15, 17: 14,
    # Left leg
    18: 0, 19: 18, 20: 19, 21: 20,
    # Right leg
    22: 0, 23: 22, 24: 23, 25: 24,
}

# Joints that act as IK chain anchors (roots that don't move)
ANCHOR_JOINTS = {0, 2, 26}


def get_subchain(joint_id: int) -> List[int]:
    """Walk from *joint_id* up to the nearest anchor, return [anchor, ..., joint_id].

    Examples:
        get_subchain(7)  -> [2, 4, 5, 6, 7]   (SPINE_CHEST -> WRIST_LEFT)
        get_subchain(21) -> [0, 18, 19, 20, 21] (PELVIS -> FOOT_LEFT)
        get_subchain(29) -> [26, 27, 28, 29]    (HEAD -> EAR_LEFT)
        get_subchain(0)  -> [0]                  (PELVIS is an anchor)
    """
    chain = [joint_id]
    current = joint_id
    while current not in ANCHOR_JOINTS and current in PARENT_JOINT:
        current = PARENT_JOINT[current]
        chain.append(current)
    chain.reverse()
    return chain


# ---------------------------------------------------------------------------
# Ray-sphere geometry
# ---------------------------------------------------------------------------
def ray_sphere_intersect(
    origin: np.ndarray,
    direction: np.ndarray,
    center: np.ndarray,
    radius: float,
) -> Optional[Tuple[float, float]]:
    """Intersect ray (origin + t*direction) with sphere |P - center|² = r².

    Returns (t1, t2) with t1 <= t2, or None if no real intersection.
    """
    oc = origin - center
    a = float(np.dot(direction, direction))
    b = 2.0 * float(np.dot(oc, direction))
    c = float(np.dot(oc, oc)) - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0 or a < 1e-15:
        return None
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    return (t1, t2)


def closest_point_on_sphere_to_ray(
    origin: np.ndarray,
    direction: np.ndarray,
    center: np.ndarray,
    radius: float,
    hint: np.ndarray,
) -> np.ndarray:
    """Fallback when the ray misses the sphere: project radially to surface.

    1. Find the point on the ray closest to *center*.
    2. Project that point radially onto the sphere surface.
    3. If the ray passes through *center* (degenerate), use *hint* direction.
    """
    d_dot_d = float(np.dot(direction, direction))
    if d_dot_d < 1e-15:
        # Zero-length direction -- just use hint
        diff = hint - center
        n = np.linalg.norm(diff)
        if n < 1e-10:
            return center + np.array([0.0, 0.0, radius])
        return center + diff / n * radius

    t = float(np.dot(center - origin, direction)) / d_dot_d
    closest = origin + t * direction
    diff = closest - center
    n = np.linalg.norm(diff)
    if n < 1e-10:
        # Ray goes through center -- use hint to pick a direction
        diff = hint - center
        n = np.linalg.norm(diff)
        if n < 1e-10:
            return center + np.array([0.0, 0.0, radius])
        return center + diff / n * radius
    return center + diff / n * radius


def ray_sphere_backproject(
    u: float,
    v: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    parent_3d: np.ndarray,
    bone_length: float,
    original_3d: np.ndarray,
) -> np.ndarray:
    """Back-project pixel (u, v) onto a sphere centered at parent_3d.

    1. Build camera ray through (u, v).
    2. Intersect with sphere(parent_3d, bone_length).
    3. Pick intersection closest to original_3d (forward t only).
    4. No intersection -> closest_point_on_sphere_to_ray fallback.
    5. Zero bone length -> return parent position.
    """
    if bone_length < 1e-3:
        return parent_3d.copy()

    # Camera ray: origin at (0,0,0), direction through pixel
    origin = np.zeros(3)
    direction = np.array([
        (u - cx) / fx,
        (v - cy) / fy,
        1.0,
    ])

    hit = ray_sphere_intersect(origin, direction, parent_3d, bone_length)
    if hit is not None:
        t1, t2 = hit
        # Consider only forward intersections (t > 0)
        candidates = []
        if t1 > 0:
            candidates.append(origin + t1 * direction)
        if t2 > 0:
            candidates.append(origin + t2 * direction)
        if candidates:
            # Pick the one closest to the original 3D position
            best = min(candidates, key=lambda p: float(np.sum((p - original_3d) ** 2)))
            return best

    # No valid intersection -- snap to closest point on sphere surface
    return closest_point_on_sphere_to_ray(
        origin, direction, parent_3d, bone_length, original_3d
    )


# ---------------------------------------------------------------------------
# FABRIK IK solver
# ---------------------------------------------------------------------------
def fabrik(
    positions: np.ndarray,
    target: np.ndarray,
    bone_lengths: List[float],
    max_iter: int = 20,
    tol: float = 0.5,
) -> np.ndarray:
    """FABRIK (Forward And Backward Reaching Inverse Kinematics).

    Parameters
    ----------
    positions : (N, 3) array -- joint positions from root to end-effector.
    target : (3,) array -- desired end-effector position.
    bone_lengths : list of N-1 floats -- bone length between consecutive joints.
    max_iter : maximum iterations.
    tol : convergence tolerance in mm.

    Returns
    -------
    (N, 3) array of adjusted positions (root is pinned).
    """
    n = len(positions)
    if n < 2:
        return positions.copy()

    pts = positions.astype(np.float64).copy()
    root = pts[0].copy()
    total_length = sum(bone_lengths)

    # Check reachability
    dist_to_target = float(np.linalg.norm(target - root))
    if dist_to_target > total_length + tol:
        # Unreachable: extend chain fully toward target
        direction = target - root
        d = np.linalg.norm(direction)
        if d < 1e-10:
            return pts
        direction /= d
        for i in range(1, n):
            pts[i] = pts[i - 1] + direction * bone_lengths[i - 1]
        return pts

    for _ in range(max_iter):
        # Check convergence
        err = float(np.linalg.norm(pts[-1] - target))
        if err < tol:
            break

        # --- Forward pass: end-effector -> root ---
        pts[-1] = target.copy()
        for i in range(n - 2, -1, -1):
            diff = pts[i] - pts[i + 1]
            d = np.linalg.norm(diff)
            if d < 1e-10:
                # Coincident points: nudge along a small offset
                diff = np.array([1e-3, 0.0, 0.0])
                d = 1e-3
            pts[i] = pts[i + 1] + diff / d * bone_lengths[i]

        # --- Backward pass: root -> end-effector ---
        pts[0] = root
        for i in range(1, n):
            diff = pts[i] - pts[i - 1]
            d = np.linalg.norm(diff)
            if d < 1e-10:
                diff = np.array([1e-3, 0.0, 0.0])
                d = 1e-3
            pts[i] = pts[i - 1] + diff / d * bone_lengths[i - 1]

    return pts


# ---------------------------------------------------------------------------
# Combined entry point
# ---------------------------------------------------------------------------
def ik_backproject(
    joint_id: int,
    u: float,
    v: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    get_3d: Callable[[int], Optional[Tuple[float, float, float]]],
    get_original_3d: Callable[[int], Optional[Tuple[float, float, float]]],
) -> Dict[int, np.ndarray]:
    """IK-aware back-projection combining ray-sphere + FABRIK.

    Parameters
    ----------
    joint_id : the joint being dragged.
    u, v : new pixel coordinates.
    fx, fy, cx, cy : pinhole intrinsics.
    get_3d : callable(jid) -> (x, y, z) or None for the *current/effective*
             3D position of any joint.
    get_original_3d : callable(jid) -> (x, y, z) or None for the *original*
                      (dataset) 3D position of any joint.

    Returns
    -------
    Dict mapping joint_id -> np.ndarray(3,) for all joints whose 3D
    position changed (the dragged joint + any intermediates adjusted by IK).
    """
    result: Dict[int, np.ndarray] = {}
    chain = get_subchain(joint_id)

    # If the joint IS the root or has no parent (PELVIS), use pinhole fallback
    if len(chain) <= 1:
        orig = get_original_3d(joint_id)
        if orig is None:
            return result
        z = orig[2]
        if z < 1:
            return result
        result[joint_id] = np.array([
            (u - cx) * z / fx,
            (v - cy) * z / fy,
            z,
        ])
        return result

    # Gather chain positions (current/effective) and bone lengths (from original)
    chain_positions = []
    chain_ok = True
    for jid in chain:
        pos = get_3d(jid)
        if pos is None:
            chain_ok = False
            break
        chain_positions.append(np.array(pos, dtype=np.float64))

    if not chain_ok or len(chain_positions) < 2:
        # Fallback to pinhole
        orig = get_original_3d(joint_id)
        if orig is None:
            return result
        z = orig[2]
        if z < 1:
            return result
        result[joint_id] = np.array([
            (u - cx) * z / fx,
            (v - cy) * z / fy,
            z,
        ])
        return result

    # Compute bone lengths from ORIGINAL 3D positions
    bone_lengths = []
    for i in range(len(chain) - 1):
        p1 = get_original_3d(chain[i])
        p2 = get_original_3d(chain[i + 1])
        if p1 is None or p2 is None:
            bone_lengths.append(0.0)
        else:
            a = np.array(p1, dtype=np.float64)
            b = np.array(p2, dtype=np.float64)
            bone_lengths.append(float(np.linalg.norm(b - a)))

    # Step 1: Ray-sphere intersection for the end-effector
    parent_idx = len(chain) - 2  # index in chain
    parent_3d = chain_positions[parent_idx]
    parent_bone_length = bone_lengths[-1]
    # Use CURRENT effective position as disambiguation hint so the solver
    # picks the sphere intersection nearest to where the joint actually is
    # (not where it was in the original dataset).  This prevents depth-flip
    # jumps when extrinsic deltas or prior IK edits have shifted the joint.
    current_end_3d = chain_positions[-1]

    end_effector_3d = ray_sphere_backproject(
        u, v, fx, fy, cx, cy,
        parent_3d, parent_bone_length, current_end_3d,
    )

    # Step 2: FABRIK to adjust intermediate joints
    positions = np.array(chain_positions)
    solved = fabrik(positions, end_effector_3d, bone_lengths)

    # Collect results: only joints that actually moved
    for i, jid in enumerate(chain):
        if i == 0:
            continue  # anchor is pinned, never moves
        old_pos = chain_positions[i]
        new_pos = solved[i]
        dist = float(np.linalg.norm(new_pos - old_pos))
        if dist > 0.01:  # 0.01mm threshold
            result[jid] = new_pos

    return result
