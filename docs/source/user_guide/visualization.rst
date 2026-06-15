Visualization
=============

SRMP provides two visualization backends that wrap :class:`~srmp.PlannerInterface` with
automatic scene tracking and 3D rendering:

- :class:`~srmp.VisualPlannerInterface` — lightweight MeshCat-based visualizer
- :class:`~srmp.ViserPlannerInterface` — interactive Viser-based visualizer with full
  browser-to-Python bidirectional communication

Both classes inherit from :class:`~srmp.PlannerInterface`, so every planning method
(``add_articulation``, ``add_box``, ``plan``, ``read_sim``, …) is available and the
scene is automatically kept in sync with the 3D view.

Installation
------------

MeshCat visualizer:

.. code-block:: console

   $ pip install meshcat

Viser visualizer:

.. code-block:: console

   $ pip install viser trimesh

.. note::

   The visualization dependencies are optional. When they are not installed, importing
   ``VisualPlannerInterface`` or ``ViserPlannerInterface`` is silently skipped and
   ``srmp.PlannerInterface`` remains fully functional.


VisualPlannerInterface (MeshCat)
---------------------------------

``VisualPlannerInterface`` connects to a `MeshCat <https://github.com/rdeits/meshcat-python>`_
WebGL viewer running in the browser. It is a good choice for quick scene inspection and
trajectory animation when interactive widget editing is not required.

**Limitations**

- MeshCat does not support reading slider values from the browser back into Python.
  Use the ``set_gui_*`` helper methods to drive GUI state programmatically.

Basic Usage
~~~~~~~~~~~

.. code-block:: python

   from srmp import VisualPlannerInterface
   import numpy as np

   # Create planner with MeshCat visualization
   planner = VisualPlannerInterface()

   # Add robot and obstacles (tracked automatically)
   planner.add_articulation(
       name="panda",
       end_effector="panda_hand",
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",   # optional
   )

   import srmp
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), box_pose)

   # Open the 3D viewer (prints the browser URL)
   planner.visualize()

   # Animate a planned trajectory
   planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "10."})
   start = np.radians([0, -45, 0, -135, 0, 90, 45])

   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.6, 0.0, 0.5])
   goal_pose.q = np.array([1.0, 0.0, 0.0, 0.0])
   goal = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start, goal)
   planner.animate_trajectory(trajectory, dt=0.05)

GUI Controls (MeshCat)
~~~~~~~~~~~~~~~~~~~~~~

Because MeshCat cannot relay browser input back to Python, obstacle editing is driven
entirely from Python using the ``set_gui_*`` helpers:

.. code-block:: python

   # Must call visualize() first
   planner.visualize()
   planner.add_gui_controls()

   # Set obstacle properties in Python, then add to the scene
   planner.set_gui_object_type("box")
   planner.set_gui_position(0.5, 0.0, 0.5)
   planner.set_gui_size(0.1, 0.1, 0.1)
   planner.set_gui_object_name("my_box")
   planner.add_obstacle_from_gui()

   # Load an existing object into the GUI state for editing
   planner.load_object_to_gui("my_box")
   planner.set_gui_position(0.6, 0.0, 0.5)
   planner.update_object_from_gui("my_box")

   # Access the browser URL
   print(planner.url)


ViserPlannerInterface (Viser)
------------------------------

``ViserPlannerInterface`` uses `Viser <https://viser.studio>`_, a modern 3D web
visualizer that provides *true bidirectional communication*. Browser sliders, dropdowns,
and buttons trigger Python callbacks in real time, making this backend ideal for
interactive scene authoring and IK-driven teleoperation.

Basic Usage
~~~~~~~~~~~

.. code-block:: python

   from srmp import ViserPlannerInterface
   import srmp
   import numpy as np

   # Start Viser server (default port 8080)
   planner = ViserPlannerInterface(port=8080)

   planner.add_articulation(
       name="panda",
       end_effector="panda_hand",
       urdf_path="/path/to/panda.urdf",
   )

   box_pose = srmp.Pose()
   box_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), box_pose)

   # Open the 3D viewer
   planner.visualize()
   print(planner.url)  # → http://localhost:8080

   # Animate a trajectory
   planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "10."})
   start = np.radians([0, -45, 0, -135, 0, 90, 45])

   goal = srmp.GoalConstraint(srmp.GoalType.JOINTS,
                              [np.radians([45, -30, 0, -120, 0, 90, 0])])
   trajectory = planner.plan(start, goal)
   planner.animate_trajectory(trajectory, dt=0.05)

   # Stop the server when done
   planner.stop()

Interactive Joint Controls
~~~~~~~~~~~~~~~~~~~~~~~~~~~

``add_robot_controls()`` creates per-joint sliders in the browser sidebar.
Moving a slider immediately updates both the Viser geometry and the planner backend:

.. code-block:: python

   planner.visualize()
   planner.add_robot_controls("panda")

   # The browser now shows joint sliders with a Reset button and
   # visibility toggles for visual / collision meshes.

End-Effector Drag Control (IK)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``add_ee_drag_control()`` places a 6-DOF transform handle at the robot's
end-effector. Dragging the gizmo in the browser triggers real-time IK:

.. code-block:: python

   planner.visualize()
   planner.add_robot_controls("panda")   # optional — syncs joint sliders
   planner.add_ee_drag_control("panda")

   # The browser shows a 3-axis gizmo at the end-effector.
   # Drag it to move the robot via IK.  On failure the gizmo
   # snaps back to the actual EE pose.

   import time
   time.sleep(60)  # Keep server alive while interacting

GUI Object Controls (Viser)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``add_gui_controls()`` adds a browser panel with dropdowns, sliders, text
inputs, and **Add / Update / Remove** buttons that directly invoke Python:

.. code-block:: python

   planner.visualize()
   planner.add_gui_controls()

   # In the browser:
   # 1. Select Object Type → "box"
   # 2. Adjust Position, Width/Radius, Height, Depth sliders
   # 3. Type a name in the "Object Name" field
   # 4. Click "Add Object"  → calls planner.add_obstacle_from_gui()
   #    Click "Update Object" → calls planner.update_object_from_gui(name)
   #    Click "Remove Object" → calls planner.remove_object(name)

   # You can also load an existing object into the GUI for editing:
   planner.load_object_to_gui("obstacle")

   # For mesh objects, click "Browse Mesh File..." to open a file dialog
   # (requires tkinter) or type the path directly.

   import time
   time.sleep(120)  # Keep server alive

Shareable URLs
~~~~~~~~~~~~~~~

Pass ``share=True`` to generate a publicly accessible Viser share URL:

.. code-block:: python

   planner = ViserPlannerInterface(port=8080, share=True)
   planner.visualize()
   # Prints both the local URL and a public share URL


MeshCat vs Viser Comparison
-----------------------------

.. list-table::
   :header-rows: 1
   :widths: 35 30 30

   * - Feature
     - VisualPlannerInterface
     - ViserPlannerInterface
   * - Dependency
     - meshcat
     - viser, trimesh
   * - Browser → Python communication
     - One-way (Python drives GUI)
     - Bidirectional
   * - Interactive joint sliders
     - Display only
     - Fully interactive (callbacks)
   * - End-effector drag / IK
     - Not available
     - Available
   * - Object editing buttons
     - Python-driven only
     - Buttons trigger Python callbacks
   * - Shareable URLs
     - Not available
     - Available (``share=True``)
   * - Mesh loading
     - STL, OBJ, DAE
     - Any format supported by trimesh
