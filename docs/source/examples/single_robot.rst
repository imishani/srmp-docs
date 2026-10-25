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

   # Add robot to the scene (downloads automatically if needed)
   planner.add_robot("panda")

   # Or use explicit paths:
   # planner.add_articulation(
   #     urdf_path="/path/to/panda.urdf",
   #     srdf_path="/path/to/panda.srdf",
   #     name="panda",
   #     end_effector="panda_hand"
   # )

   # Define start and goal configurations
   start_state = np.array([50, 47, -10, -35, -22, 93, 39])
   start_state = np.radians(start_state)

   goal_state = np.array([21, 29, -30, -104, -162, 52, -118])
   goal_state = np.radians(goal_state)

   # Create goal constraint
   goal_constraint = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_state])

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
   planner.add_robot("panda")

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

Planning to End-Effector Poses
-------------------------------

This example shows how to plan to specific end-effector poses rather than joint configurations:

.. code-block:: python

   import srmp
   import numpy as np

   # Setup planner and robot
   planner = srmp.PlannerInterface()
   planner.add_robot("panda")

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

A trajectory carries times, velocities and accelerations as well as positions. This plots
each joint over time. The plots use matplotlib, which srmp does not install
(``pip install matplotlib``):

.. code-block:: python

   import numpy as np
   import matplotlib.pyplot as plt
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   planner.add_box("box", size=[0.1, 0.4, 0.4], pose=srmp.Pose(p=[0.45, 0.0, 0.2]))
   planner.make_planner(["panda"], {"planner_id": "wAstar", "heuristic": "bfs", "weight": "50"})

   start = np.radians([-60, 30, 0, -100, 0, 130, 45])
   goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [np.radians([60, 30, 0, -100, 0, 130, 45])])
   traj = planner.plan(start, goal)

   t = np.array(traj.times)
   q = np.degrees(np.array(traj.positions))
   qd = np.degrees(np.array(traj.velocities))
   print(f"{len(t)} waypoints over {t[-1]:.2f} s")
   print("range of motion per joint (deg):", np.round(q.max(axis=0) - q.min(axis=0), 1))

   fig, (ax_q, ax_qd) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
   ax_q.plot(t, q)
   ax_q.set_ylabel("position (deg)")
   ax_qd.plot(t, qd)
   ax_qd.set_ylabel("velocity (deg/s)")
   ax_qd.set_xlabel("time (s)")
   ax_q.legend([f"joint {i + 1}" for i in range(q.shape[1])], fontsize="small", ncol=4)
   plt.tight_layout()
   plt.show()

Pick-and-Place with a Gripper
------------------------------

Robots with a gripper (like ``yam``) classify their finger joints separately from the arm
move group, so the planner never has to reason about them. Open/close the gripper with
:meth:`~srmp.PlannerInterface.set_gripper_qpos` before and after planning the arm motion —
it never affects :meth:`~srmp.PlannerInterface.set_qpos`/:meth:`~srmp.PlannerInterface.plan`,
which only ever see the arm's move-group joints.

A realistic pick-and-place is usually two phases: a search-planned, obstacle-avoiding motion
to a *pre-grasp* pose, followed by a short, direct final approach into the object. That final
segment rarely needs a full search plan — it's typically a straight, controlled motion right
up to the grasp, often combined with a small final twist to align the fingers, which makes it
a natural fit for :meth:`~srmp.PlannerInterface.plan_screw` instead of another
:meth:`~srmp.PlannerInterface.plan` call:

.. code-block:: python

   import srmp
   import srmp.robots as robots
   import numpy as np

   # yam classifies its two gripper finger joints automatically (see the robot registry)
   planner = srmp.PlannerInterface()
   name = planner.add_robot("yam")

   gripper_joints = planner.get_gripper_joint_names(name)
   print(f"Gripper joints: {gripper_joints}")

   # Open the gripper before approaching the object
   planner.set_gripper_qpos(name, [0.02, 0.02])

   planner.make_planner([name], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   start_state = np.array(robots.get("yam").default_qpos)
   link_name = robots.get("yam").end_effector

   # Phase 1: search-plan to a pre-grasp pose, offset back from the object. The slight
   # orientation difference from the final grasp is optional -- it just makes phase 2 a
   # translate-and-twist rather than a straight line; both are supported.
   pre_grasp_pose = srmp.Pose()
   pre_grasp_pose.p = np.array([0.2, 0.0, 0.2])
   pre_grasp_pose.q = np.array([0.2588, 0.0, 0.0, 0.9659])  # 150 degrees about z
   pre_grasp_goal = srmp.GoalConstraint(srmp.GoalType.POSE, [pre_grasp_pose])

   trajectory = planner.plan(start_state, pre_grasp_goal)
   print(f"Reached pre-grasp in {len(trajectory.positions)} waypoints")
   planner.set_qpos(name, trajectory.positions[-1])

   # Phase 2: final approach with plan_screw -- a short translate-and-twist right up to
   # the object. start_qpos defaults to the robot's current qpos, so it picks up exactly
   # where phase 1 left off. Joint limits and collision are checked at every step; pass
   # collision_aware=False if closing on the object trips the collision check.
   grasp_pose = srmp.Pose()
   grasp_pose.p = np.array([0.3, 0.0, 0.2])
   grasp_pose.q = np.array([0.0, 0.0, 0.0, 1.0])  # 180 degrees about z

   approach = planner.plan_screw(name, link_name, end_pose=grasp_pose)
   print(f"Final approach in {len(approach.positions)} steps")
   planner.set_qpos(name, approach.positions[-1])

   # Close the gripper to grasp — arm move-group state is untouched
   planner.set_gripper_qpos(name, [0.0, 0.0])

If the grasp needs no reorientation, give ``end_pose`` the pre-grasp orientation and the
approach becomes a straight line — see :ref:`screw-translation-only`.

To check up front that the grasp pose is actually achievable in the current scene — and, if
it is not, whether the problem is reach or clutter — run
:meth:`~srmp.PlannerInterface.collision_aware_ik` on it before planning phase 1. See
:ref:`collision-aware-ik`.

Planning with Point Clouds
---------------------------

SRMP supports point cloud obstacles for sensor-based planning:

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner and add robot
   planner = srmp.PlannerInterface()
   planner.add_robot("panda")

   # Generate sample point cloud (table surface)
   table_points = []
   for x in np.linspace(0.3, 0.7, 30):
       for y in np.linspace(-0.2, 0.2, 15):
           table_points.append([x, y, 0.4])  # Table at height 0.4m

   table_cloud = np.array(table_points)

   planner.add_point_cloud("table", table_cloud)

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
   # planner.add_point_cloud("loaded_obstacles", point_cloud)