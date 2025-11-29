Multi-Robot Planning Examples
=============================

This section demonstrates how to plan coordinated motions for multiple robots using SRMP's multi-agent planning capabilities.

Basic Two-Robot Planning
-------------------------

Here's a complete example of planning for two Panda robots:

.. code-block:: python

   import srmp
   import numpy as np
   import time

   # Create planner
   planner = srmp.PlannerInterface()

   # Add two robots (SRDF is optional). You can pass `srdf_path` if available,
   # or omit it and provide only the URDF.
   for i in range(2):
       # With SRDF (recommended when available):
       # planner.add_articulation(urdf_path=f"/path/to/panda{i}.urdf",
       #                          srdf_path=f"/path/to/panda{i}.srdf",
       #                          name=f"panda{i}",
       #                          end_effector=f"panda{i}_hand")

       # Or without SRDF (URDF only):
       planner.add_articulation(
           urdf_path=f"/path/to/panda{i}.urdf",
           name=f"panda{i}",
           end_effector=f"panda{i}_hand"
       )

   # Set base poses for robots using Pose objects
   pose0 = srmp.Pose()
   pose0.p = np.array([-0.5, 0.5, 0])
   pose0.q = np.array([1, 0, 0, 0])
   planner.set_base_pose("panda0", pose0)

   pose1 = srmp.Pose()
   pose1.p = np.array([0.5, 0.3, 0])
   pose1.q = np.array([0, 0, 0, 1])  # 180-degree rotation
   planner.set_base_pose("panda1", pose1)

   # Configure multi-robot planner following test_mramp.py patterns
   articulation_names = ["panda0", "panda1"]
   planner_context = {
       "planner_id": "xECBS",
       "weight_low_level_heuristic": "55.0",
       "high_level_focal_suboptimality": "1.8",
       "low_level_focal_suboptimality": "1.0",
   }

   # Add robot-specific parameters (heuristics and timed mprims)
   for i, name in enumerate(articulation_names):
       planner_context[f"heuristic_{name}"] = "joint_euclidean_remove_time"
       planner_context[f"mprim_path_{name}"] = "/path/to/manip_7dof_timed_mprim.yaml"

   planner.make_planner(articulation_names=articulation_names, planner_context=planner_context)

   # Define start and goal states
   start_states = {
       "panda0": np.radians([-40, 0, 0, -85, 0, 57, 0]),
       "panda1": np.radians([-40, 0, 0, -85, 0, 57, 0])
   }

   goal_states = {
       "panda0": np.radians([40, 0, 0, -70, 0, 50, 0]),
       "panda1": np.radians([40, 0, 0, -95, 0, 67, 0])
   }

   # Create goal constraints
   goal_constraints = {}
   for art_name in goal_states.keys():
       goal_constraints[art_name] = srmp.GoalConstraint(
           srmp.GoalType.JOINTS, [goal_states[art_name]]
       )

   # Plan trajectories
   start_time = time.time()
   trajectories = planner.plan_multi(start_states, goal_constraints)
   planning_time = time.time() - start_time

   print(f"Multi-robot planning completed in {planning_time:.3f} seconds")
   for robot_name, trajectory in trajectories.items():
       print(f"{robot_name}: {len(trajectory.positions)} waypoints")

Three-Robot Coordination
------------------------

This example shows coordination between three robots:

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add three robots
   robot_names = ["panda0", "panda1", "panda2"]
   base_positions = [
       np.array([-0.5, 0.5, 0]),
       np.array([0.5, 0.3, 0]),
       np.array([0.0, 0.0, 0])
   ]
   base_orientations = [
       np.array([1, 0, 0, 0]),
       np.array([0, 0, 0, 1]),
       np.array([0, 0, 0, 1])
   ]

   for i, name in enumerate(robot_names):
       planner.add_articulation(
           urdf_path=f"/path/to/{name}.urdf",
           srdf_path=f"/path/to/{name}.srdf",
           name=name,
           end_effector=f"{name}_hand"
       )

       # Set base pose
       pose = srmp.Pose()
       pose.p = base_positions[i]
       pose.q = base_orientations[i]
       planner.set_base_pose(name, pose)

   # Configure planner for three robots (use per-robot heuristics and timed mprims)
   articulation_names = robot_names
   planner_context = {
       "planner_id": "xECBS",
       "weight_low_level_heuristic": "55.0",
       "high_level_focal_suboptimality": "1.8",
       "low_level_focal_suboptimality": "1.0"
   }

   for name in robot_names:
       planner_context[f"heuristic_{name}"] = "joint_euclidean_remove_time"
       planner_context[f"mprim_path_{name}"] = "/path/to/manip_7dof_timed_mprim.yaml"

   planner.make_planner(articulation_names=articulation_names, planner_context=planner_context)

   # Define complex start and goal states
   start_states = {
       "panda0": np.radians([0, -30, 0, -85, 0, 57, 0]),
       "panda1": np.radians([0, -30, 0, -85, 0, 57, 0]),
       "panda2": np.radians([-90, -30, 0, -85, 0, 57, 0])
   }

   goal_states = {
       "panda0": np.radians([0, 40, 0, -35, 60, 57, 0]),
       "panda1": np.radians([0, 40, 0, -35, 60, 57, 0]),
       "panda2": np.radians([-90, 90, 0, -15, 0, 7, 90])
   }

   # Create goal constraints
   goal_constraints = {}
   for name in robot_names:
       goal_constraints[name] = srmp.GoalConstraint(
           srmp.GoalType.JOINTS, [goal_states[name]]
       )

   # Add environment obstacles
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.0, 0.4, 1.1])
   planner.add_box("shared_obstacle", np.array([0.1, 0.1, 0.2]), box_pose)

   # Plan coordinated motion
   trajectories = planner.plan_multi(start_states, goal_constraints)

   if trajectories:
       print("Three-robot planning successful!")
       for name, traj in trajectories.items():
           print(f"  {name}: {len(traj.positions)} waypoints")
   else:
       print("Planning failed")

Multi-Robot with Mixed Goal Types
----------------------------------

This example shows how to use different goal types for different robots:

.. code-block:: python

   import srmp
   import numpy as np

   # Setup two robots
   planner = srmp.PlannerInterface()

   for i in range(2):
       planner.add_articulation(
           urdf_path=f"/path/to/panda{i}.urdf",
           srdf_path=f"/path/to/panda{i}.srdf",
           name=f"panda{i}",
           end_effector=f"panda{i}_hand",
           planned=True
       )

   # Set base poses
   pose0 = srmp.Pose()
   pose0.p = np.array([-0.6, 0, 0])
   pose0.q = np.array([1, 0, 0, 0])
   planner.set_base_pose("panda0", pose0)

   pose1 = srmp.Pose()
   pose1.p = np.array([0.6, 0, 0])
   pose1.q = np.array([0, 0, 0, 1])
   planner.set_base_pose("panda1", pose1)

   # Configure planner (use per-robot heuristics and timed motion primitives)
   articulation_names = ["panda0", "panda1"]
   planner_context = {
       "planner_id": "xECBS",
       "weight_low_level_heuristic": "55.0",
       "high_level_focal_suboptimality": "1.5",
       "low_level_focal_suboptimality": "1.0",
   }
   for name in articulation_names:
       planner_context[f"heuristic_{name}"] = "joint_euclidean_remove_time"
       planner_context[f"mprim_path_{name}"] = "/path/to/manip_7dof_timed_mprim.yaml"

   planner.make_planner(articulation_names=articulation_names, planner_context=planner_context)

   # Start states
   start_states = {
       "panda0": np.radians([0, -45, 0, -135, 0, 90, 45]),
       "panda1": np.radians([0, -45, 0, -135, 0, 90, 45])
   }

   # Mixed goal types: joint space for panda0, end-effector pose for panda1
   goal_constraints = {}

   # Joint space goal for panda0
   goal_joints = np.radians([45, -30, 0, -120, 0, 90, 0])
   goal_constraints["panda0"] = srmp.GoalConstraint(
       srmp.GoalType.JOINTS, [goal_joints]
   )

   # End-effector pose goal for panda1
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.5, 0.2, 0.6])
   goal_pose.q = np.array([0, 0, 0, 1])
   goal_constraints["panda1"] = srmp.GoalConstraint(
       srmp.GoalType.POSE, [goal_pose]
   )

   # Plan with mixed goals
   trajectories = planner.plan_multi(start_states, goal_constraints)

   print("Mixed goal planning results:")
   print(f"  panda0 (joint goal): {len(trajectories['panda0'].positions)} waypoints")
   print(f"  panda1 (pose goal): {len(trajectories['panda1'].positions)} waypoints")

Collision Avoidance Between Robots
-----------------------------------

This example demonstrates collision avoidance between robots working in close proximity:

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add two robots close to each other
   for i in range(2):
       planner.add_articulation(
           urdf_path=f"/path/to/panda{i}.urdf",
           srdf_path=f"/path/to/panda{i}.srdf",
           name=f"panda{i}",
           end_effector=f"panda{i}_hand"
       )

   # Place robots close together to force collision avoidance
   pose0 = srmp.Pose()
   pose0.p = np.array([-0.3, 0, 0])  # Close spacing
   pose0.q = np.array([1, 0, 0, 0])
   planner.set_base_pose("panda0", pose0)

   pose1 = srmp.Pose()
   pose1.p = np.array([0.3, 0, 0])   # Close spacing
   pose1.q = np.array([0, 0, 0, 1])
   planner.set_base_pose("panda1", pose1)

   # Configure planner with collision checking
   articulation_names = ["panda0", "panda1"]
   planner_context = {
       "planner_id": "xECBS",
       "weight_low_level_heuristic": "10.0",
       "high_level_focal_suboptimality": "1.2",
       "low_level_focal_suboptimality": "1.0",
   }
   for name in articulation_names:
       planner_context[f"heuristic_{name}"] = "joint_euclidean_remove_time"
       planner_context[f"mprim_path_{name}"] = "/path/to/manip_7dof_timed_mprim.yaml"

   planner.make_planner(articulation_names=articulation_names, planner_context=planner_context)

   # Start with robots in potential conflict
   start_states = {
       "panda0": np.radians([30, -45, 0, -90, 0, 45, 0]),
       "panda1": np.radians([-30, -45, 0, -90, 0, 45, 0])
   }

   # Goals that require crossing paths
   goal_states = {
       "panda0": np.radians([-30, -45, 0, -90, 0, 45, 0]),
       "panda1": np.radians([30, -45, 0, -90, 0, 45, 0])
   }

   goal_constraints = {}
   for name in ["panda0", "panda1"]:
       goal_constraints[name] = srmp.GoalConstraint(
           srmp.GoalType.JOINTS, [goal_states[name]]
       )

   # Plan collision-free trajectories
   trajectories = planner.plan_multi(start_states, goal_constraints)

   if trajectories:
       print("Collision-free planning successful!")
       # Check trajectory synchronization
       max_length = max(len(traj.positions) for traj in trajectories.values())
       min_length = min(len(traj.positions) for traj in trajectories.values())
       print(f"Trajectory lengths: max={max_length}, min={min_length}")
   else:
       print("Failed to find collision-free solution")
