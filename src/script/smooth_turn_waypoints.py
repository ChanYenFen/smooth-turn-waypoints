__author__ = "Yen-Fen Chan"
__version__ = "0.1.0"
__date__ = '2025.08.05'
__updated__ = '2026.05.13'

import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th
import math


# ── vector helpers ──────────────────────────────────────────────────────────

def unit_vec(v):
    """Return a unitized copy of v. Returns zero vector if input is zero."""
    v = rg.Vector3d(v)
    if v.IsZero:
        return v
    v.Unitize()
    return v


def angle_between(u, v):
    """
    Signed angle (radians) from u to v in the XY plane.
    Positive = counter-clockwise.
    """
    u = unit_vec(u)
    v = unit_vec(v)
    d = max(-1.0, min(1.0, rg.Vector3d.Multiply(u, v)))
    z = rg.Vector3d.CrossProduct(u, v).Z
    ang = math.acos(d)
    return ang if z >= 0 else -ang


# ── turn waypoint builder ────────────────────────────────────────────────────

def build_turn_vias(E, a_vec, S, b_vec,
                    theta_max_deg=30.0,
                    step_len=5.0,
                    extend_len=0.0):
    """
    Build polygonal turn waypoints between path end E and next path start S.

    Strategy:
      1. Estimate space needed for the turn.
      2. If space is insufficient or turn is near 180°, fall back to
         extend-only mode: push one exit point forward from E and one
         entry point backward from S, each using at most 40% of available
         distance. This avoids crossing or jagged paths.
      3. Otherwise, build a proper angle-capped polygonal turn with an
         optional forward offset (extend_len) before the first step.

    Parameters
    ----------
    E            : rg.Point3d  — end of current path
    a_vec        : vector      — current heading at E
    S            : rg.Point3d  — start of next path
    b_vec        : vector      — desired heading into S
    theta_max_deg: max turn angle per step (degrees)
    step_len     : arc step length
    extend_len   : forward offset before starting the turn

    Returns
    -------
    exit_pts  : list[rg.Point3d]  — waypoints leaving E
    entry_pts : list[rg.Point3d]  — waypoints arriving at S
    """
    a_h = unit_vec(a_vec)
    b_h = unit_vec(b_vec)

    beta     = angle_between(a_h, b_h)
    beta_abs = abs(beta)

    # Number of rotation steps needed
    theta_rad = math.radians(theta_max_deg) if theta_max_deg > 1e-6 else math.radians(1.0)
    k_steps = int(math.ceil(beta_abs / theta_rad)) if beta_abs > 1e-6 else 0

    # Space required vs available
    needed    = extend_len + k_steps * step_len
    available = E.DistanceTo(S)

    # ── Insufficient space: just extend forward from both ends ────────────
    if needed > available:
        b_h       = unit_vec(b_vec)
        exit_pts  = [rg.Point3d(E + a_h * extend_len)]
        entry_pts = [rg.Point3d(S + b_h * extend_len)]
        return exit_pts, entry_pts

    # ── Normal polygonal turn ──────────────────────────────────────────────
    exit_pts = []

    # Straight ahead — no rotation needed
    if k_steps == 0:
        return exit_pts, []

    # Optional forward offset before turning
    if extend_len > 0.0:
        P_start = rg.Point3d(E + a_h * extend_len)
        exit_pts.append(P_start)
        E = P_start

    # Rotate heading step by step
    dtheta = beta / k_steps
    cur_pt = rg.Point3d(E)
    cur_h  = rg.Vector3d(a_h)

    for _ in range(k_steps - 1):
        cs = math.cos(dtheta)
        sn = math.sin(dtheta)
        cur_h  = rg.Vector3d(cur_h.X * cs - cur_h.Y * sn,
                             cur_h.X * sn + cur_h.Y * cs,
                             0.0)
        cur_pt = rg.Point3d(cur_pt + unit_vec(cur_h) * step_len)
        exit_pts.append(cur_pt)

    return exit_pts, []          # entry_pts handled symmetrically in main


# ── helpers ──────────────────────────────────────────────────────────────────

def dedup_chain(pts, min_dist):
    """Remove consecutive points that are closer than min_dist."""
    out = []
    for p in pts:
        if not out or out[-1].DistanceTo(p) >= min_dist:
            out.append(p)
    return out


# ── main ─────────────────────────────────────────────────────────────────────

points_list = th.tree_to_list(points_list)

all_curves        = []
all_travel_points = []
all_end_points    = []

for i, pts in enumerate(points_list or []):
    if not pts or len(pts) < 2:
        continue

    # Keep the original stroke polyline
    all_curves.append(rg.PolylineCurve(rg.Polyline(pts)))

    # Build travel segment to the next stroke
    if i >= len(points_list) - 1:
        continue

    nxt = points_list[i + 1]
    if not nxt or len(nxt) < 1:
        continue

    E      = pts[-1]
    E_prev = pts[-2]
    S      = nxt[0]

    a_vec       = E - E_prev   # heading leaving E
    b_vec_entry = nxt[1] - nxt[0] if len(nxt) >= 2 else S - E  # entry uses next stroke direction

    # ── Exit waypoints (leaving current stroke) — extend only ────────────
    a_h      = unit_vec(a_vec)
    exit_pts = [rg.Point3d(E + a_h * extend_len)]

    # ── Entry waypoints (approaching next stroke, computed in reverse) ────
    entry_pts = []
    if len(nxt) >= 2:
        E2      = nxt[0]
        E2_prev = nxt[1]
        S2      = pts[-1]

        a_vec2 = E2 - E2_prev    # heading into nxt[0] (reverse direction)
        b_vec2 = S2 - E2         # desired heading toward current end

        entry_rev, _ = build_turn_vias(
            E2, a_vec2, S2, b_vec2,
            theta_max_deg, step_len*i*0.2, extend_len*i**0.1
        )
        entry_pts = list(reversed(entry_rev))

    # ── Merge and deduplicate ─────────────────────────────────────────────
    travel_pts = exit_pts + entry_pts
    travel_pts = dedup_chain(travel_pts, step_len * 0.25)

    all_travel_points.append(travel_pts)
    all_end_points.append([E, S])

    if travel_pts:
        all_curves.append(rg.PolylineCurve(rg.Polyline(travel_pts)))

all_way_points = th.list_to_tree(all_travel_points)
all_end_points = th.list_to_tree(all_end_points)