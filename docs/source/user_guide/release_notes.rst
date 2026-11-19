Release Notes
=============

0.3.0
-----

Wheels for Python 3.9–3.14 on Linux x86-64 and macOS 11+ on Apple Silicon.

.. important::

   **VAMP plugins must be rebuilt.** This release changes the plugin ABI tag. Published
   registry plugins (``kinova_gen3_robotiq_2f85``) are fetched again for the new tag
   automatically. Plugins you built yourself are skipped at ``import srmp`` with a warning
   until you run ``srmp vamp rebuild-all`` (or set ``SRMP_VAMP_AUTO_REBUILD=1``). See
   :doc:`vamp_plugins`.

New
~~~

- **A fixed robot base can rest on a table.** When a table sits flush with a robot's base
  (common in scenes imported with :meth:`~srmp.PlannerInterface.read_sim`), the base link
  touches it in every configuration, so every state, the start included, was reported in
  collision ("Start state is not valid").
  :meth:`~srmp.PlannerInterface.set_base_contact_allowed` lets a robot's fixed links touch
  static scene objects. It is off by default, and moving links, held objects and other
  robots are still checked. :meth:`~srmp.PlannerInterface.get_base_contacts` lists what the
  base touches, and ``read_sim`` warns when an imported scene has such a contact. VAMP
  cannot skip its base check: with the switch on and the base overlapping an object,
  ``validity_backend="auto"`` falls back to coal and ``"vamp"`` raises with an explanation.
  See :ref:`base-contact`.

Fixes
~~~~~

- **plan_screw ended off target when the robot's base was moved.** World-frame Jacobians
  ignored the base's position (only its orientation was applied), so with
  :meth:`~srmp.PlannerInterface.set_base_pose`, any screw motion with a rotation ended up to
  tens of centimetres off. Affects :meth:`~srmp.PlannerInterface.get_jacobian` (world frame),
  :meth:`~srmp.PlannerInterface.compute_jacobian`,
  :meth:`~srmp.PlannerInterface.compute_joint_velocities` and
  :meth:`~srmp.PlannerInterface.plan_screw`. World-frame Jacobians are now spatial twists
  about the world origin.
- **Held objects were misplaced when the robot's base was moved.** An attached object was
  placed as if the robot stood at the origin, under both coal and VAMP, so collisions were
  checked in the wrong place. VAMP was also off by its end-effector offset (about 10 cm on
  the Panda). Held objects now follow the hand wherever the base is.
- **VAMP's held cylinders are tight.** They reached 2–3 cm past the real cylinder whatever
  ``attachment_spherization_resolution`` said. They now follow that setting: within a few
  millimetres at ``"0.005"``.
- **VAMP replanning after a scene edit is fast with many meshes.** Any edit, such as adding a
  small object, made VAMP resample every static mesh. VAMP now keeps the mesh point cloud
  when it is unchanged (with 30 static meshes, an edit plus replan went from about 740 ms to
  20 ms).
- **is_vamp_state_valid uses the planner's VAMP scene.** It used to build a second scene with
  default settings, which disagreed with the planner when the context changed
  ``mesh_sampling_resolution`` or ``pointcloud_point_radius``, and doubled the memory.
- **Viewer: the IK ghost follows the robot's base.** The Plan panel's IK preview stayed at
  the old base pose after the base was moved.
- ``read_sim`` keeps the orientation of SAPIEN planes (tilted planes were flattened).

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
