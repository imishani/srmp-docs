Visualization Examples
======================

These examples show how to use SRMP's two visualization backends.  See the
:doc:`../user_guide/visualization` page for installation instructions and a feature
comparison.

MeshCat Visualization
----------------------

Basic scene setup and trajectory animation using
:class:`~srmp.VisualPlannerInterface`:

.. code-block:: python

   from srmp import VisualPlannerInterface
   import srmp
   import numpy as np

   # Create planner with MeshCat visualization
   planner = VisualPlannerInterface()

   # Add robot
   planner.add_articulation(
       name="panda",
       end_effector="panda_hand",
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",   # optional
   )

   # Add obstacles
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), box_pose)

   sphere_pose = srmp.Pose()
   sphere_pose.p = np.array([0.3, -0.3, 0.6])
   planner.add_sphere("sphere_obs", 0.08, sphere_pose)

   # Open the viewer in your browser
   planner.visualize()
   print(f"Open: {planner.url}")

   # Plan a trajectory
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   start = np.radians([0, -45, 0, -135, 0, 90, 45])
   goal = srmp.GoalConstraint(
       srmp.GoalType.JOINTS,
       [np.radians([45, -30, 0, -120, 0, 90, 0])]
   )
   trajectory = planner.plan(start, goal)

   # Animate the trajectory in the viewer
   planner.animate_trajectory(trajectory, dt=0.05)

MeshCat GUI Controls
~~~~~~~~~~~~~~~~~~~~~

Because MeshCat cannot relay browser input back to Python, obstacle editing
is driven from Python using the ``set_gui_*`` helpers:

.. code-block:: python

   from srmp import VisualPlannerInterface
   import srmp
   import numpy as np

   planner = VisualPlannerInterface()
   planner.add_articulation("panda", "panda_hand", "/path/to/panda.urdf")
   planner.visualize()

   # Enable the GUI panel
   planner.add_gui_controls()

   # Add a box using Python-driven GUI state
   planner.set_gui_object_type("box")
   planner.set_gui_position(0.5, 0.0, 0.5)
   planner.set_gui_size(0.1, 0.1, 0.2)
   planner.set_gui_object_name("table_leg")
   planner.add_obstacle_from_gui()

   # Move an existing object
   planner.load_object_to_gui("table_leg")
   planner.set_gui_position(0.6, 0.0, 0.5)
   planner.update_object_from_gui("table_leg")

   # Add a sphere
   planner.set_gui_object_type("sphere")
   planner.set_gui_position(0.3, 0.3, 0.6)
   planner.set_gui_size(0.08, 0.08, 0.08)  # width used as radius
   planner.set_gui_object_name("ball_1")
   planner.add_obstacle_from_gui()

Viser Interactive Visualization
---------------------------------

Full interactive visualization with joint sliders and end-effector drag
using :class:`~srmp.ViserPlannerInterface`:

.. code-block:: python

   from srmp import ViserPlannerInterface
   import srmp
   import numpy as np
   import time

   # Create planner with Viser visualization
   planner = ViserPlannerInterface(port=8080)

   # Add robot
   planner.add_articulation(
       name="panda",
       end_effector="panda_hand",
       urdf_path="/path/to/panda.urdf",
   )

   # Add obstacles (appear immediately in the browser)
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), box_pose)

   # Start the Viser server
   planner.visualize()
   print(f"Open: {planner.url}")   # → http://localhost:8080

   # Add per-joint sliders (drag them in the browser to move the robot)
   planner.add_robot_controls("panda")

   # Add an end-effector drag gizmo (drag to move the robot via IK)
   planner.add_ee_drag_control("panda")

   # Keep the server alive for interactive use
   time.sleep(120)

   planner.stop()

Viser GUI Object Controls
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from srmp import ViserPlannerInterface
   import srmp
   import numpy as np
   import time

   planner = ViserPlannerInterface(port=8080)
   planner.add_articulation("panda", "panda_hand", "/path/to/panda.urdf")
   planner.visualize()

   # Add the interactive "Object Controls" panel to the browser sidebar.
   # Clicking "Add Object", "Update Object", or "Remove Object" in the
   # browser directly calls the corresponding Python method.
   planner.add_gui_controls()

   # You can still load an existing object into the GUI from Python
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.4, 0.0, 0.5])
   planner.add_box("my_box", np.array([0.1, 0.1, 0.1]), box_pose)
   planner.load_object_to_gui("my_box")

   print(f"Open {planner.url} and use the Object Controls panel")
   time.sleep(120)

Viser Multi-Robot Animation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from srmp import ViserPlannerInterface
   import srmp
   import numpy as np

   planner = ViserPlannerInterface(port=8080)

   # Add two robots
   for i in range(2):
       planner.add_articulation(
           name=f"panda{i}",
           end_effector=f"panda{i}_hand",
           urdf_path=f"/path/to/panda{i}.urdf",
       )

   # Set base poses
   pose0 = srmp.Pose()
   pose0.p = np.array([-0.5, 0.5, 0.0])
   pose0.q = np.array([1, 0, 0, 0])
   planner.set_base_pose("panda0", pose0)

   pose1 = srmp.Pose()
   pose1.p = np.array([0.5, 0.5, 0.0])
   pose1.q = np.array([0, 0, 0, 1])
   planner.set_base_pose("panda1", pose1)

   planner.visualize()

   # Configure and run multi-robot planner
   names = ["panda0", "panda1"]
   planner_context = {
       "planner_id": "xECBS",
       "weight_low_level_heuristic": "55.0",
       "high_level_focal_suboptimality": "1.8",
       "low_level_focal_suboptimality": "1.0",
   }
   for name in names:
       planner_context[f"heuristic_{name}"] = "joint_euclidean_remove_time"
       planner_context[f"mprim_path_{name}"] = "/path/to/manip_7dof_timed_mprim.yaml"

   planner.make_planner(names, planner_context)

   start_states = {
       "panda0": np.radians([-40, 0, 0, -85, 0, 57, 0]),
       "panda1": np.radians([-40, 0, 0, -85, 0, 57, 0]),
   }
   goal_states = {
       "panda0": np.radians([40, 0, 0, -70, 0, 50, 0]),
       "panda1": np.radians([40, 0, 0, -95, 0, 67, 0]),
   }
   goal_constraints = {
       name: srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_states[name]])
       for name in names
   }

   trajectories = planner.plan_multi(start_states, goal_constraints)

   # Animate all robots simultaneously in the browser
   planner.animate_trajectory(trajectories, dt=0.05)

   planner.stop()
