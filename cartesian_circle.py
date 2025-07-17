#!/usr/bin/env python3
"""
cartesian_circle.py
────────────────────
Reusable helper for drawing a closed Cartesian circle with MoveIt!.

Typical import:

    from cartesian_circle import plan_cartesian_circle
"""

import math, copy
from typing import Tuple, List
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory


def _build_waypoints(start_pose: Pose,
                     radius: float,
                     n_points: int,
                     plane: str) -> List[Pose]:
    """Return way-points forming a full circle that starts/ends at start_pose."""
    axis = {"x": 0, "y": 1, "z": 2}
    a, b = plane[0], plane[1]                       # e.g. "x","y" for XY plane

    # Circle centre: one radius behind start_pose along +a
    centre = [start_pose.position.x,
              start_pose.position.y,
              start_pose.position.z]
    centre[axis[a]] -= radius                       # shift back along +a
    cx, cy, cz = centre

    waypoints = []
    for i in range(n_points + 1):                   # +1 duplicates the start
        theta = 2.0 * math.pi * i / n_points

        w = copy.deepcopy(start_pose)               # keep orientation unchanged
        delta = [0.0, 0.0, 0.0]
        delta[axis[a]] = radius * math.cos(theta)
        delta[axis[b]] = radius * math.sin(theta)

        w.position.x = cx + delta[0]
        w.position.y = cy + delta[1]
        w.position.z = cz + delta[2]
        waypoints.append(w)

    return waypoints


def plan_cartesian_circle(
    move_group: MoveGroupCommander,
    radius: float = 0.10,
    n_points: int = 120,
    plane: str = "xy",
    eef_step: float = 0.005,
    jump_threshold: float = 0.0,
    velocity_scale: float = 0.2,
    execute: bool = True,
    completeness_threshold: float = 0.95
) -> Tuple[RobotTrajectory, float]:
    """
    Plan (and optionally execute) a closed Cartesian circle.

    Args:
        move_group:           active MoveIt! MoveGroupCommander
        radius:               circle radius [m]
        n_points:             coarse samples around 360°
        plane:                "xy", "yz", or "xz"
        eef_step:             IK resolution passed to compute_cartesian_path
        jump_threshold:       as in MoveIt! API (0 → disable check)
        velocity_scale:       applied with set_max_velocity_scaling_factor
        execute:              if True and completeness ≥ threshold, executes
        completeness_threshold: minimum fraction required for execution

    Returns:
        trajectory, fraction  same as what compute_cartesian_path returns.
    """
    move_group.set_max_velocity_scaling_factor(velocity_scale)

    start_pose = move_group.get_current_pose().pose
    waypoints = _build_waypoints(start_pose, radius, n_points, plane)

    traj, fraction = move_group.compute_cartesian_path(
        waypoints,
        eef_step=eef_step,
        jump_threshold=jump_threshold,
        avoid_collisions=True
    )

    if execute and fraction >= completeness_threshold:
        move_group.execute(traj, wait=True)

    return traj, fraction
