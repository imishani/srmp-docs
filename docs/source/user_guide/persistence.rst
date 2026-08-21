Saving and Loading
==================

SRMP saves three kinds of file — scenes, agent chats, and plans — each standalone and each
round-tripping through the same code the GUI buttons call. Nothing here is a separate export
format: the GUI writes exactly the files the Python API writes.

.. versionadded:: 0.1.4.7

Scenes
------

A *scene* is the planning world: every articulation, every collision object, and every
attachment, with their poses and configurations.

.. code-block:: python

   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   planner.add_box("table", [0.8, 0.6, 0.05], table_pose)

   planner.save_scene("bin_picking.json")

Reading it back, either into a fresh planner or an existing one:

.. code-block:: python

   planner = srmp.load_scene("bin_picking.json")          # build a new planner
   planner.load_scene("bin_picking.json", clear=True)     # load into an existing one

:meth:`~srmp.PlannerInterface.save_scene` writes ``bin_picking.json``, plus a
``bin_picking.npz`` sidecar beside it when the scene holds geometry with no file behind it —
meshes imported from a simulator, and point clouds. Keep the two together; the manifest is
not loadable without its sidecar.

What travels, and what does not
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Robots added with ``add_robot("panda")`` are recorded by **registry name**, so the scene
works on another machine — the robot re-downloads if missing. Robots and meshes given as
paths are recorded relative to the scene file when they live beside it, and absolute
otherwise, so a scene saved next to its assets stays movable.

Loading in place validates the **whole** manifest first — every URDF, SRDF, mesh, and
sidecar array — before it clears the live world. A manifest whose assets have since moved
therefore fails with your current scene still intact, rather than leaving you with neither.

.. note::

   Use the module-level :func:`srmp.load_scene` rather than
   :meth:`~srmp.PlannerInterface.load_scene` when the scene has a custom grid config.
   Planning-world bounds are fixed at construction, so only a newly built planner can honor
   them; loading into an existing planner warns and keeps that planner's own bounds.

Merging instead of replacing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``clear=False`` merges the manifest into the current world instead of resetting it, and
raises on a name collision rather than silently overwriting:

.. code-block:: python

   planner.load_scene("shelf.json")                  # replaces the world
   planner.load_scene("mug.json", clear=False)        # adds to it

Plans
-----

A *plan* is one planning episode: the trajectories, the start states, the goal constraints,
and the planner context that produced them.

.. code-block:: python

   traj = planner.plan(start, goal)
   planner.save_plan("pick.json")        # saves the last plan() / plan_multi()

   trajectories, meta = srmp.load_plan("pick.json")
   viz.animate_trajectory(trajectories)

``load_plan`` always returns the ``{robot: Trajectory}`` dict form, so a single-robot plan is
a one-entry dict. ``meta`` carries the rest: the scene it was made against,
``planner_context``, ``start``, ``goal``, and each trajectory's joint names.

Pass ``planner=`` to have the saved joint names checked against the robot's current move
group — animating a mismatched trajectory silently drives the wrong joints:

.. code-block:: python

   trajectories, meta = srmp.load_plan("pick.json", planner=planner)

.. note::

   :meth:`~srmp.PlannerInterface.save_plan` raises ``RuntimeError`` until a real plan exists.
   A search that comes back with zero waypoints — what the planner returns for an invalid
   start state, without raising — does not count as a plan and is not recorded as the "last
   plan". This is deliberate: a failed search cannot be persisted as a file that looks like a
   success.

Saving a trajectory that did not come from ``plan()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:meth:`~srmp.PlannerInterface.plan_screw` and friends do not register a "last plan", so use
the explicit module-level form, keyed by robot name:

.. code-block:: python

   traj = planner.plan_screw("panda", "panda_hand", end_pose=grasp_pose)

   srmp.save_plan(
       "screw.json",
       trajectories={"panda": traj},
       joint_names={"panda": planner.get_move_group_joint_names("panda")},
       start={"panda": start_qpos},
   )

``joint_names`` travels with each trajectory so a saved plan stays interpretable without its
scene, and so loading can verify the move group still matches.

.. note::

   A saved plan's goal is a ``GoalConstraint`` of type ``JOINTS`` or ``POSE`` — the only two
   types constructible in this build. ``POSITION`` and the ``MULTI_GOAL_*`` types are not
   supported by the plan format.

Agent chats
-----------

.. code-block:: python

   agent.save_chat("session.json")
   agent.save_chat("session.json", scene="bin_picking.json")   # record provenance
   agent.load_chat("session.json")

Loading restores **the messages only**. No saved code is re-executed — that would re-run side
effects like long solves and downloads — and the Python interpreter is not reset either, so
in the GUI (where the executor shares the live planner) whatever you had defined before the
load is still there.

The agent is told that the transcript came from a file, and that the interpreter and planning
world may not match what the conversation describes, so it verifies rather than assumes.
Rebuild the world with ``load_scene()`` if you want the saved scene back.

Chats are backend-neutral: one captured on Anthropic loads into an OpenAI-backed agent.

The agent methods take the path alone — ``agent.save_chat(path, scene=None)`` and
``agent.load_chat(path)``. There is also a module-level pair, but note it lives under
``srmp.persistence`` and takes the agent as its first argument; unlike ``save_scene`` and
``save_plan``, it is not re-exported at the top level:

.. code-block:: python

   from srmp.persistence import save_chat, load_chat

   save_chat(agent, "session.json")
   load_chat(agent, "session.json")

See :ref:`agent-chat-persistence` in :doc:`agent_mode` for the GUI buttons.

In the GUI
----------

**Controls → Save / Load** holds a **Scene path** field with Save/Load Scene buttons, and a
separate **Plan path** field with Save Plan / Load & Replay. The two paths are independent,
so saving a plan can never overwrite the scene file. **Save Chat** and **Load Chat** live in
the **AI Agent** panel.

Paths are **server-side** — files land on the machine running the planner, not in the
browser's Downloads folder. ``~`` is expanded, so ``~/scenes/bin.json`` works. The result of
every button, success or failure, appears in the panel's status line.
