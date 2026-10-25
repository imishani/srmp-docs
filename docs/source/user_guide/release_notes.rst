Release Notes
=============

0.2.1
-----

Linux x86-64 wheels for Python 3.9–3.14, as in 0.2.0. The published VAMP plugins still apply.

Fixes
~~~~~

- **Objects held by the robot get one collision verdict everywhere.** Contact between two held
  objects was skipped when the check named the robot (:meth:`~srmp.PlannerInterface.is_state_colliding`,
  and the start check of wAstar and Astar) but counted by wPASE, so a state could be free in
  one place and in collision in another. Now every check treats them alike: objects attached
  to the **same link** may touch (they move rigidly together, as with a multi-piece grasp),
  and objects on different links or different robots may not.

0.2.0
-----

Linux x86-64 wheels for Python 3.9–3.14. There are no macOS wheels in this release; on macOS,
``pip install srmp`` stays on 0.1.6.

New
~~~

- **VAMP plugins.** Any robot can get a VAMP model without recompiling srmp. Registry robots
  with a published plugin get it automatically on :meth:`~srmp.PlannerInterface.add_robot`
  (the first is ``kinova_gen3_robotiq_2f85``), and ``srmp vamp add-robot --urdf`` builds one
  from a sphere URDF. See :doc:`vamp_plugins`.
- **VAMP scene-translation settings** in the planner context (``mesh_sampling_resolution``,
  ``attachment_max_spheres`` and others), so an attached mesh over the sphere budget can be
  planned with. See :doc:`api`.
- :meth:`~srmp.PlannerInterface.get_vamp_supported_robots`,
  :meth:`~srmp.PlannerInterface.is_robot_vamp_supported` and
  :meth:`~srmp.PlannerInterface.is_vamp_state_valid`.
- The agent mode knows about the validity backends and plugins.

Behavior changes
~~~~~~~~~~~~~~~~

These make planning stricter or more honest. Plans that passed before may now be rejected, or
planned differently, where the old behavior was wrong.

- **Robot vs held objects and robot vs robot are checked where the links are now.** Before,
  those checks used link positions from when the robot was added, so they could miss real
  collisions (and wPASE rejected every state once an object was attached). Touch links are now
  detected when an object is attached.
- **Motions are checked every 0.03 rad** (per joint) instead of up to ~0.2 rad, so paths no
  longer step through thin obstacles, and post-processing no longer produces waypoints inside
  them. Planning takes about 10% longer.
- **A VAMP model is used only for a robot whose kinematics match it.** VAMP's built-in ``ur5``
  model does not describe the registry's ``ur5``, so ``ur5`` now plans on coal (``auto``
  chooses coal; ``vamp`` raises).
- :meth:`~srmp.PlannerInterface.set_allowed_collision` accepts link names as
  :meth:`~srmp.PlannerInterface.get_link_names` returns them, and robot names; a name that
  matches nothing now raises ``ValueError`` instead of being silently ignored.
- :meth:`~srmp.PlannerInterface.attach_object`'s fourth argument is ``touch_links``; an
  attachment ``pose`` is keyword-only (and still unsupported).
- :meth:`~srmp.PlannerInterface.collision_aware_ik` no longer gives up early: ``'unreachable'``
  means IK did not succeed within ``timeout``.

Fixes
~~~~~

- Calling :meth:`~srmp.PlannerInterface.plan` a second time on the same planner (wAstar, Astar)
  could corrupt memory and crash.
- wPASE worker threads no longer move the shared robot (a data race on its kinematics and
  collision geometry).
- IK converges for robots with continuous joints (Kinova Gen3).
- ``detach_object(..., also_remove=True)`` also clears the object's occupancy-grid cells.
- A scene saved after :meth:`~srmp.PlannerInterface.set_grid` records the new grid, and saved
  scenes keep explicit touch links.
