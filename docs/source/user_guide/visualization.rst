Visualization
=============

SRMP ships an interactive 3D visualizer built on `Viser <https://viser.studio>`_, with full
bidirectional browser ↔ Python communication: sliders, dropdowns, and buttons in the browser
directly invoke Python callbacks. It watches a :class:`~srmp.PlannerInterface` and keeps the
3D scene in sync automatically as robots and objects are added, moved, or removed.

Installation
------------

``viser`` (and its ``trimesh`` dependency, used for mesh-file loading) are installed
automatically with ``srmp`` — no separate install step is needed:

.. code-block:: console

   $ pip install srmp

Getting a Visualizer
---------------------

Attach a visualizer to a planner with :meth:`~srmp.PlannerInterface.start_visualizer`:

.. code-block:: python

   import srmp
   import numpy as np

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")

   viz = planner.start_visualizer(type="viser", port=8080)
   print(viz.url)  # → http://localhost:8080

``viz`` is a *visualizer/listener* attached to ``planner`` — it is notified whenever the scene
changes (``add_robot``, ``add_box``, …) and exposes visualization-specific methods
(``add_robot_controls``, ``animate_trajectory``, …, all documented below). Any planning method
not defined on ``viz`` itself (``add_box``, ``plan``, ``read_sim``, …) is forwarded straight
through to ``planner``, so you can call it on either object interchangeably.

.. note::

   :class:`srmp.ViserPlannerInterface` is the same visualizer class, constructed directly —
   ``srmp.ViserPlannerInterface(port=8080)`` builds its own internal ``PlannerInterface`` and
   attaches itself to it in one step. This is what SRMP's own :doc:`agent_mode` GUI/CLI and
   several existing tests use, and it behaves identically to ``start_visualizer()`` — pick
   whichever is more convenient for your code.

.. note::

   A MeshCat backend (``start_visualizer(type="meshcat")``) exists in the codebase but is
   currently being reworked and isn't functional yet — Viser is the only supported backend
   right now.

Basic Usage
-----------

.. code-block:: python

   import srmp
   import numpy as np

   planner = srmp.PlannerInterface()
   viz = planner.start_visualizer(type="viser", port=8080)

   # Add robot and obstacles — tracked automatically
   planner.add_articulation(
       name="panda",
       end_effector="panda_hand",
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",   # optional
   )

   box_pose = srmp.Pose()
   box_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), box_pose)

   # Plan and animate a trajectory
   planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "10."})
   start = np.radians([0, -45, 0, -135, 0, 90, 45])
   goal = srmp.GoalConstraint(srmp.GoalType.JOINTS,
                              [np.radians([45, -30, 0, -120, 0, 90, 0])])
   trajectory = planner.plan(start, goal)
   viz.animate_trajectory(trajectory, dt=0.05)

   # Stop the server when done
   viz.stop()

Controls Panel (added automatically)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Starting a visualizer automatically builds a top-level "Controls" folder in the sidebar with
on-demand launch buttons: **Robot Controls**, **Object Controls**, **Plan Controls**,
**Console**, and **AI Agent**. Each button lazily builds the corresponding panel
(``add_robot_controls``, ``add_object_controls``, ``add_plan_controls``, …) the first time
it's clicked, so you don't need to call those methods yourself just to make them available —
you can still call them directly (as shown below) to have a panel open immediately instead of
waiting for a click. This launcher is also the entry point the :doc:`agent_mode` GUI uses to
embed its chat panel.

Interactive Joint Controls
~~~~~~~~~~~~~~~~~~~~~~~~~~~

``add_robot_controls(robot_name)`` creates per-joint sliders in the browser sidebar. Moving a
slider immediately updates both the Viser geometry and the planner backend:

.. code-block:: python

   viz.add_robot_controls("panda")

   # The browser now shows joint sliders with a Reset button and
   # visibility toggles for visual / collision meshes.

If the robot was added with ``gripper_joint_names`` (see
:meth:`~srmp.PlannerInterface.add_articulation`), its gripper sliders are included in the same
panel. They drive :meth:`~srmp.PlannerInterface.set_gripper_qpos` instead of
:meth:`~srmp.PlannerInterface.set_qpos`, so moving them does not change the arm move group's
qpos dimension. Reset also restores gripper joints to their default position alongside the arm.

Goal-Driven Planning (Plan Controls)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``add_plan_controls()`` adds a "Plan" folder to the sidebar for dragging a goal into place and
planning to it:

.. code-block:: python

   viz.add_plan_controls()

   # In the browser:
   # 1. Check a robot's box in the "Plan" folder — this drops a 6-DOF goal
   #    gizmo at its current end-effector pose, plus a semi-transparent
   #    "ghost" preview robot that follows the gizmo via live IK.
   # 2. Drag the gizmo to the desired end-effector pose.
   # 3. Click "Plan to goal" — SRMP plans to the dragged goal(s) with the
   #    selected planner and animates the resulting trajectory.
   #    "Reset EE" snaps the gizmo back to the robot's actual current pose.

Object Drag Controls
~~~~~~~~~~~~~~~~~~~~~

``add_object_controls()`` adds a drag gizmo to every object currently in the scene, letting you
reposition obstacles directly in the browser instead of via ``update_object_from_gui``:

.. code-block:: python

   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), box_pose)
   viz.add_object_controls()

   # Drag any object's gizmo in the browser. The visual mesh moves
   # immediately; the collision world updates ~0.4s after dragging stops.

GUI Object Controls
~~~~~~~~~~~~~~~~~~~~

``add_gui_controls()`` adds a browser panel with dropdowns, sliders, text inputs, and
**Add / Update / Remove** buttons that directly invoke Python:

.. code-block:: python

   viz.add_gui_controls()

   # In the browser:
   # 1. Select Object Type → "box"
   # 2. Adjust Position, Width/Radius, Height, Depth sliders
   # 3. Type a name in the "Object Name" field
   # 4. Click "Add Object"  → calls viz.add_obstacle_from_gui()
   #    Click "Update Object" → calls viz.update_object_from_gui(name)
   #    Click "Remove Object" → calls viz.remove_object(name)

   # You can also load an existing object into the GUI for editing:
   viz.load_object_to_gui("obstacle")

   # For mesh objects, click "Browse Mesh File..." to open a file dialog
   # (requires tkinter) or type the path directly.

Multiple Visualizers
~~~~~~~~~~~~~~~~~~~~~

A planner can have more than one visualizer attached at once. Use
:meth:`~srmp.PlannerInterface.get_visualizer` to look one up later (e.g. from another function),
and :meth:`~srmp.PlannerInterface.detach_visualizer` /
:meth:`~srmp.PlannerInterface.attach_visualizer` to manage them manually:

.. code-block:: python

   viz = planner.get_visualizer()       # index=0 by default
   planner.detach_visualizer(viz)       # stops receiving scene updates

Shareable URLs
~~~~~~~~~~~~~~~

Pass ``share=True`` to generate a publicly accessible Viser share URL:

.. code-block:: python

   viz = planner.start_visualizer(type="viser", port=8080, share=True)
   # viz.url prints both the local URL and a public share URL

Feature Summary
---------------

- **Dependency**: ``viser``, ``trimesh``
- **Browser → Python communication**: Bidirectional — sliders, dropdowns, and buttons
  trigger Python callbacks
- **Interactive joint sliders**: Fully interactive, including gripper sliders
- **Goal-driven planning**: Drag-to-goal + "Plan to goal" via :meth:`add_plan_controls`
- **Object editing**: Drag gizmos (:meth:`add_object_controls`) and a buttons panel
  (:meth:`add_gui_controls`)
- **Shareable URLs**: Available via ``share=True``
- **Mesh loading**: Any format supported by ``trimesh``
