Release Notes
=============

0.2.2
-----

New
~~~

- **SRMP for coding assistants.** ``pip install "srmp[mcp]"`` adds a skill and an MCP server
  (``srmp mcp``) for Claude Code, Codex, Gemini CLI, Antigravity and Cursor. The assistant can
  write SRMP code from verified examples, and run plans in a live session with a 3D viewer.
  ``srmp skill install --for <tool>`` and ``srmp mcp install --for <tool>`` set it up. Claude
  Code users can install both as a plugin from ``imishani/srmp-assistant``. See
  :doc:`coding_assistants`.
- **Path simplification on Linux.** The opt-in ``postprocess_vamp_simplify`` setting, which
  0.2.1 shipped only in the macOS wheels, is now in the Linux wheels too (see
  :doc:`../guides/postprocessing`).

Fixes
~~~~~

- **Agent Mode CLI returned stale results after a plan.** SRMP's C++ core prints progress
  lines during ``plan()``, and those lines broke the CLI's link to its worker process. From
  the first plan on, every tool call returned the previous call's output. Each call now gets
  its own output, C++ lines included. The worker no longer freezes on heavy error output, and
  a crash reports the worker's last error lines.
- **Quaternion order in examples and docs.** ``Pose.q`` is ``[w, x, y, z]``. The README quick
  start, Agent Mode's built-in examples and several documentation examples used
  ``[0, 0, 0, 1]``, which is a 180° turn about z, not the identity.
- **Type stubs.** The stubs now include ``plan_screw``, ``attach_visualizer``,
  ``detach_visualizer``, ``is_vamp_state_valid`` and ``Pose.to_transformation_matrix``.
  ``collision_aware_ik`` is now documented as returning ``(q, status)``, which matches what
  it actually returns.

0.2.1
-----

Wheels for Python 3.9–3.14 on Linux x86-64, as in 0.2.0, and on **macOS 11+ on Apple Silicon**,
the first macOS wheels since 0.1.6. The published VAMP plugins still apply (Linux only).

The macOS wheels were built a little later than the Linux ones and also include the opt-in
``postprocess_vamp_simplify`` setting (see :doc:`../guides/postprocessing`). It is off by
default, so planning behaves the same on both platforms.

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
