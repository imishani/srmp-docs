Single Robot Planning Examples
==============================

This section provides comprehensive examples for single robot motion planning with SRMP.

Basic Planning Example
----------------------

Here's a simple example showing the complete workflow for planning a trajectory for a single Panda robot:

.. code-block:: python

   import srmp
   import numpy as np
   import time

   # Create planner interface
   planner = srmp.PlannerInterface()

   # Add robot to the scene
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Define start and goal configurations
   start_state = np.array([50, 47, -10, -35, -22, 93, 39])
   start_state = np.radians(start_state)

   goal_state = np.array([21, 29, -30, -104, -162, 52, -118])
   goal_state = np.radians(goal_state)

   # Create goal constraint
   goal_constraint = srmp.GoalConstraint(srmp.GoalType.JOINTS, goal_state)

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   # Plan trajectory
   start_time = time.time()
   trajectory = planner.plan(start_state, goal_constraint)
   planning_time = time.time() - start_time

   print(f"Planning completed in {planning_time:.3f} seconds")
   print(f"Trajectory has {len(trajectory.positions)} waypoints")

Planning with Obstacles
-----------------------

This example shows how to add obstacles to the environment and plan around them:

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner and add robot
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Add various obstacles
   # Box obstacle
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("box_obstacle", np.array([0.1, 0.1, 0.4]), box_pose)

   # Sphere obstacle
   sphere_pose = srmp.Pose()
   sphere_pose.p = np.array([0.3, -0.3, 0.6])
   planner.add_sphere("sphere_obstacle", 0.1, sphere_pose)

   # Cylinder obstacle
   cylinder_pose = srmp.Pose()
   cylinder_pose.p = np.array([-0.2, 0.4, 0.5])
   planner.add_cylinder("cylinder_obstacle", 0.05, 0.3, cylinder_pose)

   # Configure planner for obstacle avoidance
   planner.make_planner(["panda"], {
       "planner_id": "ARAstar",
       "heuristic": "bfs",
       "weight": "10.",
       "weight_delta": "1.",
       "final_weight": "1."
   })

   # Plan with obstacles
   start_state = np.radians([50, 47, -10, -35, -22, 93, 39])

   # Goal as end-effector pose
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.642, -0.068, 0.505])
   goal_pose.q = np.array([0.0, 0.0, 0.0, 1.0])
   goal_constraint = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal_constraint)
   print(f"Planned trajectory with {len(trajectory.positions)} waypoints")

Different Planner Algorithms
-----------------------------

SRMP supports various planning algorithms. Here's how to use different planners:

wAstar (Weighted A*)
~~~~~~~~~~~~~~~~~~~~

Fast planning with suboptimal solutions:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "joint_euclidean",
       "weight": "50."  # Higher weight = faster but less optimal
   })

ARAstar (Anytime Repairing A*)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Iteratively improves solution quality:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "ARAstar",
       "heuristic": "bfs",
       "weight": "10.",
       "weight_delta": "1.",   # Weight reduction per iteration
       "final_weight": "1."    # Final weight (1.0 = optimal)
   })

MHAstar (Multi-Heuristic A*)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Uses multiple heuristics for better performance:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "MHAstar",
       "inadmissible_heuristics": "bfs",
       "w1": "100.",  # Weight for anchor heuristic
       "w2": "100."   # Weight for inadmissible heuristics
   })

wPASE (Weighted Parallel A* Search)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Parallel search for improved performance:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "wPASE",
       "heuristic": "bfs",
       "weight": "50.",
       "num_threads": "8"  # Number of parallel threads
   })

Planning to End-Effector Poses
-------------------------------

This example shows how to plan to specific end-effector poses rather than joint configurations:

.. code-block:: python

   import srmp
   import numpy as np

   # Setup planner and robot
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   # Start configuration
   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])

   # Goal as 6DOF pose
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.6, -0.1, 0.5])    # Position [x, y, z]
   goal_pose.q = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion [x, y, z, w]

   # Normalize quaternion to be safe
   goal_pose.q = goal_pose.q / np.linalg.norm(goal_pose.q)

   goal_constraint = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   # Plan to pose
   trajectory = planner.plan(start_state, goal_constraint)

   print(f"Planned trajectory to pose: {goal_pose.p}")
   print(f"Trajectory length: {len(trajectory.positions)}")


Trajectory Analysis
-------------------

After planning, you can analyze and visualize the resulting trajectory:

.. code-block:: python

   import srmp
   import numpy as np
   import matplotlib.pyplot as plt

   # ... (setup planner and plan trajectory as before) ...
   trajectory = planner.plan(start_state, goal_constraint)

   # Extract trajectory data
   positions = trajectory.positions
   num_waypoints = len(positions)

   print(f"Trajectory Statistics:")
   print(f"  Number of waypoints: {num_waypoints}")
   print(f"  Start configuration: {np.degrees(positions[0])}")
   print(f"  Goal configuration: {np.degrees(positions[-1])}")

   # Calculate joint ranges of motion
   joint_ranges = []
   for joint_idx in range(7):  # 7 joints for Panda
       joint_values = [pos[joint_idx] for pos in positions]
       joint_range = max(joint_values) - min(joint_values)
       joint_ranges.append(np.degrees(joint_range))
       print(f"  Joint {joint_idx+1} range: {joint_range:.2f} degrees")

   # Plot trajectory (optional - requires matplotlib)
   try:
       fig, axes = plt.subplots(7, 1, figsize=(10, 14))
       for joint_idx in range(7):
           joint_trajectory = [np.degrees(pos[joint_idx]) for pos in positions]
           axes[joint_idx].plot(joint_trajectory)
           axes[joint_idx].set_ylabel(f'Joint {joint_idx+1} (deg)')
           axes[joint_idx].grid(True)

       axes[-1].set_xlabel('Waypoint')
       plt.title('Joint Trajectories')
       plt.tight_layout()
       plt.show()
   except ImportError:
       print("matplotlib not available for plotting")

Planning with Point Clouds
---------------------------

SRMP supports point cloud obstacles for sensor-based planning:

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner and add robot
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Generate sample point cloud (table surface)
   table_points = []
   for x in np.linspace(0.3, 0.7, 30):
       for y in np.linspace(-0.2, 0.2, 15):
           table_points.append([x, y, 0.4])  # Table at height 0.4m

   table_cloud = np.array(table_points)

   # Add point cloud with specified resolution
   planner.add_point_cloud("table", table_cloud, resolution=0.01)

   # Configure planner and plan trajectory
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10.0"
   })

   start_state = np.radians([0, -30, 0, -120, 0, 90, 45])

   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.5, 0.0, 0.6])  # Above table
   goal_pose.q = np.array([0, 0, 0, 1])
   goal_constraint = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal_constraint)

   if trajectory:
       print(f"Planned around point cloud: {len(trajectory.positions)} waypoints")

   # Load point cloud from file
   def load_point_cloud_from_file(filename):
       """Load point cloud from XYZ text file"""
       return np.loadtxt(filename, usecols=(0, 1, 2))

   # Usage: point_cloud = load_point_cloud_from_file("/path/to/points.txt")
   # planner.add_point_cloud("loaded_obstacles", point_cloud, resolution=0.02)