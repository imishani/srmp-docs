Visualization Examples
======================

These examples show how to use SRMP's Viser-based interactive visualizer.  See the
:doc:`../user_guide/visualization` page for installation instructions and a feature
summary.

Viser Interactive Visualization
---------------------------------

Full interactive visualization with joint sliders and goal-driven planning:

.. code-block:: python

   import srmp
   import numpy as np
   import time

   # Create a planner and attach a Viser visualizer to it
   planner = srmp.PlannerInterface()
   viz = planner.start_visualizer(type="viser", port=8080)

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

   print(f"Open: {viz.url}")   # → http://localhost:8080

   # Add per-joint sliders (drag them in the browser to move the robot)
   viz.add_robot_controls("panda")

   # Add goal-driven planning: check the robot, drag its goal gizmo, then
   # click "Plan to goal" in the browser
   viz.add_plan_controls()

   # Keep the server alive for interactive use
   time.sleep(120)

   viz.stop()

Viser GUI Object Controls
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np
   import time

   planner = srmp.PlannerInterface()
   viz = planner.start_visualizer(type="viser", port=8080)
   planner.add_articulation("panda", "panda_hand", "/path/to/panda.urdf")

   # Add the interactive "Object Controls" panel to the browser sidebar.
   # Clicking "Add Object", "Update Object", or "Remove Object" in the
   # browser directly calls the corresponding Python method.
   viz.add_gui_controls()

   # You can still load an existing object into the GUI from Python
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.4, 0.0, 0.5])
   planner.add_box("my_box", np.array([0.1, 0.1, 0.1]), box_pose)
   viz.load_object_to_gui("my_box")

   print(f"Open {viz.url} and use the Object Controls panel")
   time.sleep(120)

Viser Multi-Robot Animation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np

   planner = srmp.PlannerInterface()
   viz = planner.start_visualizer(type="viser", port=8080)

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
   viz.animate_trajectory(trajectories, dt=0.05)

   viz.stop()
