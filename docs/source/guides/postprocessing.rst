Path Post-Processing and Timing
===============================

The search returns a path made of motion primitives: straight joint-space segments with
corners between them. Before :meth:`~srmp.PlannerInterface.plan` returns, SRMP shortens and
smooths that path, then gives it times that respect each joint's velocity and acceleration
limits. Every change is collision-checked, so the returned trajectory is as valid as the raw
path.

What happens to a path
----------------------

1. **Shortcut.** Waypoints that a straight motion can skip are removed, preferring the
   farthest reachable waypoint.
2. **Smooth.** The path is subdivided into short segments (``postprocess_densify_resolution``)
   and its corners are relaxed toward their neighbours (Laplacian smoothing), alternating with
   another shortcut pass, up to ``postprocess_max_iterations`` times. A move is kept only if
   the motions on both sides stay collision-free.
3. **Resample.** Waypoints are spaced at most ``postprocess_output_resolution`` apart (radians
   per joint), keeping a waypoint wherever dropping it would cut through an obstacle.
4. **Time.** A clamped cubic spline that starts and ends at rest is fitted through the
   waypoints, and each interval is slowed until no joint exceeds its limits. ``velocities``
   and ``accelerations`` in the trajectory are the spline's own derivatives.

Steps 1–3 stop improving after ``postprocess_timeout`` seconds (default 1) and return the
best valid path so far. Step 4 always runs.

Multi-robot plans (:meth:`~srmp.PlannerInterface.plan_multi`) are not post-processed: their
waypoints and times come straight from the timed search.

Changing it
-----------

Post-processing settings go in the same dictionary as the planner settings:

.. code-block:: python

   import numpy as np
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   planner.add_box("box", size=[0.1, 0.4, 0.4], pose=srmp.Pose(p=[0.45, 0.0, 0.2]))
   start = np.radians([-60, 30, 0, -100, 0, 130, 45])
   goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [np.radians([60, 30, 0, -100, 0, 130, 45])])

   base = {"planner_id": "wAstar", "heuristic": "bfs", "weight": "50"}
   for label, extra in [
       ("default", {}),
       ("raw path", {"postprocess_preset": "none"}),
       ("slower", {"postprocess_velocity_scale": "0.5"}),
       ("real accel limits", {"postprocess_accel_limits": "15,7.5,10,12.5,15,20,20"}),
   ]:
       planner.make_planner(["panda"], {**base, **extra})
       traj = planner.plan(start, goal)
       print(f"{label:18s} {len(traj.positions):4d} waypoints, {traj.times[-1]:.2f} s")

The raw path has a handful of waypoints at the primitives' corners; the default output has
many closely spaced ones along a smoother curve, and usually a shorter duration.

**Presets.** ``postprocess_preset`` sets a starting point; individual keys override it.

- ``default``: the settings below.
- ``none``: no shortcutting, smoothing or resampling. The raw search path, still timed.
- ``match_vamp``: smoothing set close to VAMP's own simplifier, for comparing search planners
  with ``vamp_rrtc`` on equal terms.

**Speed.** Joint velocity limits come from the URDF and are scaled by
``postprocess_velocity_scale`` (default 0.9). URDFs carry no acceleration limits, so by
default each joint's acceleration limit is ``postprocess_accel_scale`` (default 8) times its
scaled velocity limit; this means ``postprocess_velocity_scale`` slows acceleration too. To
use your robot's real acceleration limits, pass them as ``postprocess_accel_limits``: one
number per joint, in rad/s², comma-separated. Then ``postprocess_velocity_scale`` affects
velocity only.

All settings
------------

.. list-table::
   :header-rows: 1
   :widths: 34 14 52

   * - Key
     - Default
     - Meaning
   * - ``postprocess_preset``
     - ``default``
     - ``default``, ``none`` or ``match_vamp``.
   * - ``postprocess_enable``
     - ``true``
     - ``false`` skips steps 1–3 (same as preset ``none``).
   * - ``postprocess_smooth``
     - ``true``
     - ``false`` skips step 2 (shortcut and resample still run).
   * - ``postprocess_timeout``
     - ``1.0``
     - Seconds for shortcutting and smoothing.
   * - ``postprocess_max_iterations``
     - ``5``
     - Smooth/shortcut rounds.
   * - ``postprocess_densify_resolution``
     - ``0.05``
     - Segment length (rad) before smoothing.
   * - ``postprocess_output_resolution``
     - ``0.05``
     - Largest gap (rad) between returned waypoints.
   * - ``postprocess_alpha``
     - ``0.5``
     - How far each smoothing step moves a waypoint toward its neighbours' midpoint.
   * - ``postprocess_min_change``
     - ``0.02``
     - Smaller smoothing moves are skipped; a fraction of
       ``postprocess_densify_resolution``.
   * - ``postprocess_chunk_size``
     - ``8``
     - Shortcut candidates checked per batch (larger suits VAMP).
   * - ``postprocess_velocity_scale``
     - ``0.9``
     - Fraction of each joint's URDF velocity limit.
   * - ``postprocess_accel_scale``
     - ``8``
     - Acceleration limit as a multiple of the scaled velocity limit.
   * - ``postprocess_accel_limits``
     - (none)
     - Per-joint acceleration limits in rad/s², e.g. ``"15,7.5,10,12.5,15,20,20"``.

Booleans accept ``true``/``false``/``1``/``0``; any other value raises, as does a negative or
non-finite number.

.. note::

   **Unreleased** (in ``main``, not yet on PyPI): ``"postprocess_vamp_simplify": "true"`` runs
   VAMP's own shortcut-and-B-spline simplifier (the one ``vamp_rrtc`` uses) on the search path
   before the steps above. It needs ``validity_backend`` ``"vamp"``.

``planner.get_planning_stats()`` reports ``postprocess_time`` after each plan.
